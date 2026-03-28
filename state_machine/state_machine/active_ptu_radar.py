#!/usr/bin/env python3
import threading # multitask
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
#from std_msgs.msg import Header

from radar_msg.msg import RadarData
from radar_msg.action import RadarBeamform, PtuSweep
#from radar_package.parametros import *

class PtuRadarScan(Node):
    """
    Espera Bool en /start_scan. Al recibir True:
      - Recorre el PTU en [angle_min:angle_step:angle_max]
      - Llama a radar_beamform en cada ángulo del PTU
      - Apila filas y publica un mensaje RadarData de captura en 'radar_data'
    No apaga rclpy: el nodo sigue vivo hasta que cierres el proceso.
    """

    def __init__(self):
        super().__init__('ptu_radar_scan')

        # ----------------- Parámetros -----------------
        self.declare_parameter('angle_min', ANGLE_MIN_PTU_PAN)
        self.declare_parameter('angle_max', ANGLE_MAX_PTU_PAN)
        self.declare_parameter('angle_step', ANGLE_STEP_PTU_PAN)
        self.declare_parameter('wait_timeout_server_s', WAIT_TIMEOUT_SERVER)
        self.declare_parameter('trigger_topic', 'start_scan')
        self.declare_parameter('publish_topic', 'radar_data')

        p = self.get_parameter
        self.angle_min_ptu_pan = int(p('angle_min').value)
        self.angle_max_ptu_pan = int(p('angle_max').value)
        self.angle_step_ptu_pan = int(p('angle_step').value)
        self.wait_timeout = float(p('wait_timeout_server_s').value)

        self.trigger_topic = str(p('trigger_topic').value)
        self.publish_topic = str(p('publish_topic').value)

        # ----------------- Vector de ángulos -----------------
        #self.pan_list = list(range(self.angle_min_ptu_pan, self.angle_max_ptu_pan + 1, self.angle_step_ptu_pan))
        #self.pan_list = [0]
        #self.pan_list = [0, 0, 0, 0, 0]
        #self.pan_list = [90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90]
        self.pan_list = [45, 30, 15, 0, -15, -30, -45]
        #self.pan_list = [90, 60, 30, 0, -30, -60, -90]
        #self.tilt_list = [-20, -15, -10, -5, 0, 5, 10, 15, 20]
        #self.tilt_list = [-20, -15, -10, -5, 0]
        self.tilt_list = [0]
        #self.tilt_list = [0] * len(self.pan_list) # tilt fijo 0, mismo largo que pan_list

        # numero de puntos
        num_pos = len(self.pan_list)

        if num_pos == 0:
            raise RuntimeError("El rango de ángulos es vacío. Revisa angle_min/angle_max/angle_step.")

        # Action clients
        self.ptu_client = ActionClient(self, PtuSweep, 'ptu_sweep')
        self.radar_client = ActionClient(self, RadarBeamform, 'radar_beamform')

        # Publisher del resultado final
        self.pub_final = self.create_publisher(RadarData, self.publish_topic, 10)

        # Subscriber de disparo
        self.sub_trigger = self.create_subscription(Bool, self.trigger_topic, self.trigger_cb, 10)

        # Estado
        self.is_running = False
        self._run_lock = threading.Lock() # Solo un hilo a la vez puede tener el lock, los demás hilos esperan hasta que se libere

        self.robot_pose_id = -1 # posicion determinada del robot -> cambiar por gps

        self.get_logger().info(
            f"Listo. Puntos={num_pos} | pan_list={self.pan_list} | tilt_list={self.tilt_list} | "
            f"Suscripcion='{self.trigger_topic}' | publica en '{self.publish_topic}'"
        )

    # ----------------- utilidades -----------------

    # acciones desde lógica secuencial
    # se convierte una acción ROS2 asíncrona en una sincrónica segura, usando callbacks + threading.Event, sin romper el spin() activo
    def _send_goal_wait_result(self, client: ActionClient, goal_msg, action_name: str):
        """
        Envía goal y espera el resultado usando callbacks + threading.Event,
        compatible con un spin() ros2 ya corriendo.
        Retorna (succes: bool, result_obj or None, error_msg or None).
        """
        try:
            done_evt = threading.Event() # bloqueo controlado, esperar resultado sin romper el loop de ros2
            holder = {'goal_handle': None, 'result': None, 'error': None} # retorno

            def on_goal_done(fut): # future
                try:
                    gh = fut.result() # GoalHandle manejo de objetivo
                    holder['goal_handle'] = gh
                    if gh is None or not gh.accepted:
                        holder['error'] = f"Goal rechazado por {action_name}"
                        done_evt.set() # se libera el evento
                        return # se termina el flujo

                    def on_res_done(fut2): # callback del resultado final
                        try:
                            holder['result'] = fut2.result().result
                            # fut2.result() -> GetResult.Response
                            # .result -> el objeto Result de la acción
                        except Exception as e2:
                            holder['error'] = f"Error resultado {action_name}: {e2!r}"
                        finally:
                            done_evt.set() # liberar el bloqueo

                    gh.get_result_async().add_done_callback(on_res_done) # se pide el resultado y se registra el callback

                except Exception as e:
                    holder['error'] = f"Error al enviar goal a {action_name}: {e!r}"
                    done_evt.set()

            client.send_goal_async(goal_msg).add_done_callback(on_goal_done) # se envia el goal y se registra el callback
            done_evt.wait() # bloquea hasta tener result o error, no bloquea ROS, sigue procesando callbacks y cuando llega el resultado se libera el programa con set()

            if holder['error'] is not None:
                return False, None, holder['error']
            return True, holder['result'], None

        except Exception as e:
            return False, None, f"Excepción inesperada con {action_name}: {e!r}"

    # ----------------- trigger -----------------
    def trigger_cb(self, msg: Bool):
        if not msg.data:
            self.get_logger().info("Trigger False: ignorando.")
            return

        with self._run_lock: # un solo hilo en ejecucion
            if self.is_running: # hay una rutina activa?
                self.get_logger().warn("Ya hay una rutina en ejecución; ignoro nuevo trigger.")
                return
            self.robot_pose_id += 1 # movimiento del robot manual/forzado
            self.is_running = True

        # Ejecutar rutina en hilo aparte
        t = threading.Thread(target=self._run_scan, daemon=True)
        t.start()

    # ----------------- rutina principal -----------------

    # rutina de escaneo con acciones de forma sucuencial
    def _run_scan(self):
        try:
            # Esperar servidores
            if not self.ptu_client.wait_for_server(timeout_sec=self.wait_timeout):
                self.get_logger().error("Servidor 'ptu_sweep' no disponible.")
                return
            if not self.radar_client.wait_for_server(timeout_sec=self.wait_timeout):
                self.get_logger().error("Servidor 'radar_beamform' no disponible.")
                return

            cols_expected = None

            num_pan  = len(self.pan_list)
            num_tilt = len(self.tilt_list)
            total    = num_pan * num_tilt
            step     = 0

            for ip, tilt in enumerate(self.tilt_list, start=1):
                for it, pan in enumerate(self.pan_list, start=1):
                    step += 1
                    tag = f"[{step}/{total} | pan {ip}/{num_pan}, tilt {it}/{num_tilt}]"

                    # 1) PTU
                    self.get_logger().info(f"{tag} PTU pan={pan}°, tilt={tilt}°")
                    ptu_goal = PtuSweep.Goal()
                    # mensaje PTU dentro del Goal
                    #ptu_goal.target_ptu.header.stamp = self.get_clock().now().to_msg()
                    ptu_goal.target_ptu.pan_deg  = int(pan)
                    ptu_goal.target_ptu.tilt_deg = int(tilt)
                    ok, ptu_res, err = self._send_goal_wait_result(self.ptu_client, ptu_goal, 'ptu_sweep') # enviar objetivo y esperar resultado
                    if not ok or (hasattr(ptu_res, 'success') and not ptu_res.success):
                        msg = getattr(ptu_res, 'message', '')
                        self.get_logger().error(f"{tag} Falló PTU pan={pan}°, tilt={tilt}°. {err or msg}")
                        return

                    # 2) Radar
                    self.get_logger().info(f"{tag} Radar beamform pan={pan}°, tilt={tilt}°")
                    rb_goal = RadarBeamform.Goal()
                    rb_goal.pan_deg = int(pan)
                    rb_goal.tilt_deg = int(tilt)
                    ok, rb_res, err = self._send_goal_wait_result(self.radar_client, rb_goal, 'radar_beamform')
                    # fallo: falso, falso
                    if not ok or (hasattr(rb_res, 'success') and not rb_res.success):
                        msg = getattr(rb_res, 'message', '')
                        self.get_logger().error(f"{tag} Falló beamform pan={pan}°, tilt={tilt}°. {err or msg}")
                        return

                    # proteccion ante error en envio de mensaje de radar
                    if not hasattr(rb_res, 'radar_data'):
                        self.get_logger().error("RadarBeamform.Result no trae 'radar_data'. Revisa tu Beamform.action.")
                        return

                    # rd debe ser del tipo RadarData
                    rd: RadarData = rb_res.radar_data # del resultado del accion se obtiene el mensaje de radar

                    if cols_expected is None:
                        cols_expected = rd.cols
                    elif rd.cols != cols_expected:
                        self.get_logger().error(f"{tag} Inconsistencia: cols {cols_expected} vs {rd.cols}.")
                        return
                
                    rd.robot_pose_id = int(self.robot_pose_id)

                    self.pub_final.publish(rd)
                    self.get_logger().info(f"{tag} Publicado RadarData ({rd.rows}x{rd.cols}) en '{self.publish_topic}'")

            self.get_logger().info("Escaneo completado.")

        finally:
            with self._run_lock: # un solo hilo en ejecucion
                self.is_running = False # libera el proceso para una nueva rutina


def main(args=None):
    rclpy.init(args=args)
    node = PtuRadarScan()
    try:
        # Nodo persistente: espera triggers por tópico
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
