import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionServer
from threading import Event
import time
import re
from radar_msg.action import PtuSweep

# convencion
# positivo izquierda
# negativo derecha 

# limites minimo y maximo de cada grado de libertad
# paneo
#PN * -> -3090 -> - 158.91423666666665
#PX * -> 3090 -> 158.91423666666665

# tilteo
#TN * -> -907 -> -46.645699888888885 hacia delante
#TX * -> 604 -> 31.062847555555553 hacia atras


# misma resolucion en ambas direcciones
# 90, 75, 60, 45, 30, 15, 0
# 1750, 1361, 1167, 875, 583, 292, 0

def grados_a_pasos(grados):
    # convierte angulos en grados a cantidad de pasos para el PTU-C46 considerando una resolucion de 185.1428 arcsec por paso (0.0514285°)
    resolucion_grados = 185.1428 / 3600  # ≈ 0.0514285°
    pasos = round(grados / resolucion_grados)
    return pasos

def pasos_a_grados(pasos):
    # convierte una cantidad de pasos del PTU-C46 a grados considerando una resolución de 185.1428 arcsec por paso (0.0514285°)

    resolucion_grados = 185.1428 / 3600  # ≈ 0.0514285°
    grados = round(pasos * resolucion_grados)
    return grados

class PtuRoutineNode(Node):
    def __init__(self):
        super().__init__('ptu_routine_node')
        # topicos
        self.cmd_pub = self.create_publisher(String, '/ptu_cmd', 10)
        self.rx_sub = self.create_subscription(String, '/ptu_response', self.rx_cb, 50)

        #self._beamform_client = ActionClient(self, Beamform, 'beamform')
        #self.beamforming_pub = self.create_publisher(Bool, '/allow_beamforming', 10)

        #self.allow_sub = self.create_subscription(Bool, '/allow_routine_ptu', self.allow_cb, 10)
        #self.allow_bunker_pub = self.create_publisher(Bool, '/allow_bunker', 10)

        self.ptu_angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        self.current_index = 0
        self._sweep_done_evt = Event()
        self._sweep_success = False
        self._active_goal = None
        self._ptu_server = ActionServer(self, PtuSweep, 'ptu_sweep', execute_callback=self.execute_ptu_sweep_cb, goal_callback=self.ptu_goal_cb, cancel_callback=self.ptu_cancel_cb)

        self.target_steps = None # objetivo en pasos
        self.waiting_for_pp = False # esperando respuesta a 'pp'
        self.last_rx_position = None # última posición en pasos
        self.query_timer = None # timer de consulta periódica
        self.query_period = 1 # s entre consultas 'pp'
        self.query_timeout_s = 30.0 # timeout total para confirmar
        self.query_started = None # timestamp de inicio
        self.tolerance_steps = 0 # tolerancia de coincidencia en pasos

        # reenvío de setpoint mientras consulto
        self.command_resend_period = 2
        self._last_cmd_send_time = None

        self._pan_re = re.compile(r'Current\s+Pan\s+position\s+is\s+(-?\d+)', re.IGNORECASE)


        #self.get_logger().info('Publica True en /allow_routine_ptu para avanzar un paso del recorrido.')


    # ACTION SERVER: policy de aceptación / cancelación
    def ptu_goal_cb(self, goal_request):
        # rechaza si ya estás en un sweep activo
        if self._active_goal is not None:
            self.get_logger().warn("PTU ocupado: rechazando nueva meta.")
            return rclpy.action.GoalResponse.REJECT
        return rclpy.action.GoalResponse.ACCEPT
    
    def ptu_cancel_cb(self, goal_handle):
        # soportar cancelación real: se debe añadir un flag y respetarlo en el loop
        self.get_logger().info("Solicitud de cancelación recibida.")
        return rclpy.action.CancelResponse.ACCEPT
    
     # ACTION SERVER: ejecución del sweep completo
    async def execute_ptu_sweep_cb(self, goal_handle):
        self._active_goal = goal_handle
        self._sweep_done_evt.clear()
        self._sweep_success = False

        # Config desde la Goal (con defaults sensatos)
        angles = list(goal_handle.request.angles_deg) if goal_handle.request.angles_deg else self.ptu_angles
        self.tolerance_steps = goal_handle.request.tolerance_steps or self.tolerance_steps
        self.query_timeout_s = goal_handle.request.query_timeout_s or self.query_timeout_s

        # Reset índice y arranca
        self.current_index = 0
        self._kickoff_step(angles)

        # espera a que termine todo el barrido (éxito o aborto)
        self._sweep_done_evt.wait()

        # Construye Result
        result = PtuSweep.Result()
        result.success = self._sweep_success
        result.message = "OK" if self._sweep_success else "Abortado o falló algún paso"

        # Marca estado terminal
        if self._sweep_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        # Libera
        self._active_goal = None
        return result
    
    # realizacion pasos de ptu
    def _kickoff_step(self, angles):
        """Envía movimiento al ángulo actual y prepara el ciclo de consulta."""
        if self.current_index >= len(angles):
            # Terminado el sweep
            self._sweep_success = True
            self._sweep_done_evt.set()
            return

        angulo = angles[self.current_index]
        self._publish_feedback(angulo, f"Moviendo a {angulo}°")

        # mover PTU
        self.target_steps = grados_a_pasos(angulo)
        self._send_cmd(f"pp{self.target_steps}")
        self._last_cmd_send_time = time.monotonic()

        # iniciar verificación
        self.waiting_for_pp = True
        self.last_rx_position = None
        self.query_started = time.monotonic()
        if self.query_timer is None:
            self.query_timer = self.create_timer(self.query_period, lambda: self._query_tick(angles))

        # primera consulta
        self._send_cmd('pp')

    def _publish_feedback(self, angle, status):
        if self._active_goal:
            fb = PtuSweep.Feedback()
            fb.current_index = self.current_index
            fb.current_angle = int(angle)
            fb.status = status
            self._active_goal.publish_feedback(fb)

    

    #def allow_cb(self, msg: Bool):
    #    if not msg.data or self.waiting_for_pp:
    #        return
    #    
    #    #if self.current_index >= len(self.ptu_angles):
    #    #    self.current_index = 0
    #
    #    # calculo del paso y el comando
    #    angulo = self.ptu_angles[self.current_index]
    #    self.target_steps = grados_a_pasos(angulo)
    #    comando = f"pp{self.target_steps}"
    #
    #    # publicar
    #    self._send_cmd(comando)
    #    self._last_cmd_send_time = time.monotonic()
    #
    #    self.get_logger().info(
    #        f"Paso {self.current_index+1}/{len(self.ptu_angles)}: "
    #        f"Enviado '{comando.strip()}' ({angulo}°). Verificando llegada..."
    #    )
    #
    #    self._start_query_loop() # loop de verificacion
    
    # publicar comando
    def _send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    # verificación de llegada + reenvío setpoint
    def _query_tick(self, angles):
        if not self.waiting_for_pp:
            return

        # si ya tenemos posición y coincide -> éxito
        if self.last_rx_position is not None and self._matches_target(self.last_rx_position):
            self._on_reached_target(angles, self.last_rx_position)
            return

        # revisar si se supera el timeout
        elapsed = time.monotonic() - self.query_started
        if elapsed > self.query_timeout_s:
            self._on_query_timeout()
            return

        # Reenviar 'pp' para consultar
        self._send_cmd('pp')

        # Reinyectar el setpoint cada cierto periodo
        now = time.monotonic()
        if (self.target_steps is not None
            and self._last_cmd_send_time is not None
            and (now - self._last_cmd_send_time) >= self.command_resend_period):
            self._send_cmd(f'pp{self.target_steps}') # reinyecta el objetivo
            self._last_cmd_send_time = now
            # self.get_logger().debug(f"Reenvío setpoint pp{self.target_steps}")

    def _matches_target(self, pos_steps: int) -> bool:
        return (self.target_steps is not None and
                abs(pos_steps - self.target_steps) <= self.tolerance_steps)
    
    def _on_reached_target(self, angles, pos_steps: int):
        # parar loop
        self.waiting_for_pp = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None

        self._last_cmd_send_time = None # limpiar

        ang = pasos_a_grados(pos_steps)
        self._publish_feedback(ang, f"Llegó a {ang}°; solicitando beamforming...")
        #self.get_logger().info(f"Llegó a {pos_steps} pasos ({ang:.2f}°). Solicitando beamforming...")

        # avanzar índice y reiniciar si esta al final
        #if self.current_index < len(self.ptu_angles) - 1:
        #    msg = Bool()
        #    msg.data = True
        #    self.beamforming_pub.publish(msg)
        #    self.current_index += 1
        #
        #else:
        #    self.current_index = 0
        #    msg = Bool()
        #    msg.data = True
        #    self.allow_bunker_pub.publish(msg)
        #    self.get_logger().info("Recorrido completado. Siguiente True reinicia en -90°.")

        self._request_beamforming_then_next(angles, ang)

    def _on_query_timeout(self, angles):
        self.waiting_for_pp = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None

        self._last_cmd_send_time = None # limpiar

        self._publish_feedback(
            self.ptu_angles[self.current_index] if self.current_index < len(self.ptu_angles) else 0,
            f"Timeout verificando llegada a {self.target_steps} pasos"
        )

        # Abortar todo el sweep
        self._sweep_success = False
        self._sweep_done_evt.set()

        #self.get_logger().warn(
        #    f"Timeout verificando llegada a {self.target_steps} pasos. "
        #    f"Última lectura: {self.last_rx_position}."
        #)
        # No avanzamos de índice: requerirá otro True para reintentar o pasar al siguiente

    # Beamforming (cliente de acción Radar)
    #def _request_beamforming_then_next(self, angles, angle_deg: int):
    #    # Esperar disponibilidad del servidor (no bloquea el executor)
    #    if not self._beamform_client.wait_for_server(timeout_sec=2.0):
    #        self.get_logger().warn("Servidor de acción 'beamform' no disponible")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        # Decide qué hacer: reintentar, marcar error, etc.
    #        return
#
    #    goal = Beamform.Goal()
    #    goal.angle_deg = int(angle_deg)
#
    #    send_future = self._beamform_client.send_goal_async(
    #        goal,
    #        feedback_callback=lambda fb: self._publish_feedback(angle_deg, fb.feedback.status)
    #    )
    #    send_future.add_done_callback(lambda fut: self._beamform_goal_response_cb(fut, angles))

    #def _beamform_goal_response_cb(self, future, angles):
    #    exc = future.exception()
    #    if exc:
    #        self.get_logger().error(f"Error al enviar goal de beamforming: {exc!r}")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        return
#
    #    goal_handle = future.result()
    #    if goal_handle is None or not goal_handle.accepted:
    #        self.get_logger().warn("Goal de beamforming rechazada")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        return
#
    #    result_future = goal_handle.get_result_async()
    #    result_future.add_done_callback(lambda fut: self._beamform_result_cb(fut, angles))

    #def _beamform_result_cb(self, future, angles):
    #    exc = future.exception()
    #    if exc:
    #        self.get_logger().error(f"Error obteniendo resultado de beamforming: {exc!r}")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        return
#
    #    result_container = future.result()
    #    if result_container is None:
    #        self.get_logger().error("Contenedor de resultado de beamforming es None.")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        return
#
    #    result = result_container.result
    #    if not result.success:
    #        self.get_logger().warn(f"Beamforming falló: {result.message}")
    #        self._sweep_success = False
    #        self._sweep_done_evt.set()
    #        return
#
    #    # Paso OK → siguiente ángulo
    #    self.current_index += 1
    #    self._publish_feedback(
    #        angles[self.current_index-1] if self.current_index-1 < len(angles) else 0,
    #        "Beamforming OK; avanzando al siguiente ángulo"
    #    )
    #    self._kickoff_step(angles)

    # loop de consulta 'pp' hasta coincidir o timeout
    #def _start_query_loop(self):
    #    self.waiting_for_pp = True
    #    self.last_rx_position = None
    #    self.query_started = time.monotonic()
    #
    #    if self._last_cmd_send_time is None:
    #        self._last_cmd_send_time = self.query_started
    #
    #    # Enviar primera consulta
    #    self._send_cmd('pp')
    #
    #    # timer periódico
    #    if self.query_timer is None:
    #        self.query_timer = self.create_timer(self.query_period, self._query_tick)

    #def _beamform_feedback_cb(self, feedback):
    #    self.get_logger().info(f"[Radar feedback] {feedback.feedback.status}")

    def rx_cb(self, msg: String):
        # busca entero con signo en la línea
        m = self._pan_re.search(msg.data.strip())
        #self.get_logger().info(f"mensaje: {m}")
        if m:
            pos = int(m.group(1))
            self.last_rx_position = pos
            #if self.waiting_for_pp and self._matches_target(pos):
            #    # confirmación inmediata sin esperar otro tick
            #    self._on_reached_target(pos)


def main(args=None):
    rclpy.init(args=args)
    node = PtuRoutineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo interrumpido por el usuario.")
    except Exception as e:
        print(f"Excepción no controlada: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
