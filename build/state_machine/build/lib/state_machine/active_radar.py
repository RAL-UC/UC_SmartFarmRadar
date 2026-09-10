#!/usr/bin/env python3
import rclpy # ros2 
from rclpy.node import Node # clase nodo de ros2
from rclpy.action import ActionClient # acciones de ros2
from radar_msg.msg import RadarData # mensaje de radar personalizado
from radar_msg.action import RadarBeamform # accion de beamforming

# nodo que permite ejecutar acciones de escaneo beamforming de radar
class RadarScan(Node):
    def __init__(self):
        super().__init__('radar_scan')

        # ----------------- Parámetros -----------------
        self.declare_parameter('repeat', -1) # -1 para infinitas capturas
        self.declare_parameter('wait_timeout_server_s', 30) # tiempo de espera para conectar el cliente con el servidor

        # carga de valores
        # radar.yaml
        self.repeat = self.get_parameter('repeat').value
        self.wait_timeout = self.get_parameter('wait_timeout_server_s').value
        
        self.infinite = (self.repeat <= 0) # <=0 => infinito
        self.client = ActionClient(self, RadarBeamform, 'radar_beamform')
        # 10 representa la profundidad de la cola de mensajes
        self.radar_pub = self.create_publisher(RadarData, 'radar_data', 10)

        self.current_iter = 0 # numero de escaneos
        self.busy = False # flag control de estado

        self.start()

    def start(self):
        if not self.client.wait_for_server(timeout_sec=self.wait_timeout): # si no logra conectarse apaga el nodo
            self.get_logger().error("Servidor de acción 'radar_beamform' no disponible.")
            rclpy.shutdown()
            return
        
        rep_msg = "inf" if self.infinite else str(self.repeat)
        self.get_logger().info(f"Beamforming: se repetirá {rep_msg} veces")
        self.send_goal()

    def send_goal(self):
        if self.busy: # false no esta ocupado, true esta ocupado
            return


        if (not self.infinite) and (self.current_iter >= self.repeat): # se cumplieron todas las repeticiones
            self.get_logger().info("Todas las repeticiones completadas.")
            rclpy.shutdown()
            return

        # actualizacion de estado y reporte
        self.busy = True
        # Se calcula el intento actual solo para el log (sin modificar la variable de clase)
        attempt_num = self.current_iter + 1
        tag = f"{attempt_num}/{self.repeat}" if not self.infinite else f"[{attempt_num}]"
        #self.get_logger().info(f"{tag} radar callback en pan={self.pan_deg}°, tilt={self.tilt_deg}°")
        self.get_logger().info(f"[Action] {tag} radar beamforming")

        goal = RadarBeamform.Goal() # objetivo

        # se envia el objetivo y se registra el callback de retroalimentacion
        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb) # verificacion de si el servidor aceptó o rechazó la meta solicitada

    def feedback_cb(self, fb):
        msg = getattr(fb.feedback, 'status', 'ejecutando…') # si no detecta el campo status devuelve ejecutando
        self.get_logger().info(f"[Feedback] {msg}")

    def goal_response_cb(self, future):
        exc = future.exception() # guarda el error que haya ocurrido
        if exc:
            self.busy = False # liberar proceso
            self.get_logger().error(f'Error al enviar goal Radar: {exc!r}')
            return

        # goal handle, manejador de meta
        # indica si el servidor aceptó la meta
        gh = future.result()
        if gh is None or not gh.accepted:
            self.busy = False # liberar proceso
            self.get_logger().warn('Goal Radar rechazado por el servidor.')
            return

        # solicitar el resultado final de la acción al servidor de forma asíncrona
        # definir qué hacer cuando la tarea termine
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.result_cb)

    def result_cb(self, future):
        exc = future.exception() # guarda el error que haya ocurrido
        if exc:
            self.busy = False # liberar proceso
            self.get_logger().error(f'Error recibiendo resultado Radar: {exc!r}')
            self.send_goal()
            return

        # valida la respuesta final devuelta por el servidor
        res = future.result().result
        if not res.success:
            self.busy = False # liberar proceso
            self.get_logger().warn(f'Beamforming falló: {res.message}')
            self.send_goal()
            return

        self.current_iter += 1
        # extraer datos de radar
        rd = getattr(res, 'radar_data', None)
        if rd is not None:
            #rd.header.stamp = self.get_clock().now().to_msg() # ya vienen con una marca de tiempo
            # se realiza doble publicación por accion y luego por mensaje
            self.radar_pub.publish(rd)
            self.get_logger().info(
                f'Beamforming OK | Publicado RadarData {rd.rows}x{rd.cols}')
        else:
            self.get_logger().info(f'Beamforming OK ({res.message}) sin radar_data')

        # Liberar estado y solicitar la siguiente iteración
        self.busy = False
        self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node = RadarScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
