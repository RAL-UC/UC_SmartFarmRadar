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

def grados_a_pasos(grados: int) -> int:
    """Convierte grados a pasos para PTU-C46 (185.1428 arcsec/paso ≈ 0.0514285°)."""
    resolucion_grados = 185.1428 / 3600 # ≈ 0.0514285°
    pasos = round(grados / resolucion_grados)
    return pasos

def pasos_a_grados(pasos: int) -> int:
    """Convierte pasos a grados para PTU-C46 (185.1428 arcsec/paso ≈ 0.0514285°)."""
    resolucion_grados = 185.1428 / 3600 # ≈ 0.0514285°
    grados = round(pasos * resolucion_grados)
    return grados

class PtuRoutineNode(Node):
    def __init__(self):
        super().__init__('ptu_routine_node')
        # topicos
        self.cmd_pub = self.create_publisher(String, '/ptu_cmd', 10)
        self.rx_sub = self.create_subscription(String, '/ptu_response', self.rx_cb, 50)

        #self.ptu_angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        #self.current_index = 0

        self._active_goal = None
        self._done_evt = Event()
        self._success = False
        self._final_message = "OK"

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

        self._server = ActionServer(
            self,
            PtuSweep,
            'ptu_sweep',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

    # ActionServer: aceptación/cancelación
    def goal_cb(self, goal_request):
        # Rechaza si ya hay una goal activa (este server es mono-objetivo)
        if self._active_goal is not None:
            self.get_logger().warn("PTU ocupado: rechazando nueva meta.")
            return rclpy.action.GoalResponse.REJECT
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Solicitud de cancelación recibida.")
        # Aquí podrías agregar un flag para abortar la espera; por ahora se permite.
        return rclpy.action.CancelResponse.ACCEPT
    
    async def execute_cb(self, goal_handle):
        self._active_goal = goal_handle
        self._done_evt.clear()
        self._success = False
        self._final_message = "OK"

        target_deg = goal_handle.request.angles_deg
        self.tolerance_steps = goal_handle.request.tolerance_steps or self.tolerance_steps
        self.query_timeout_s = goal_handle.request.query_timeout_s or self.query_timeout_s

        # manda movimiento
        self._publish_feedback(target_deg, f"Moviendo a {target_deg}°")
        self.target_steps = grados_a_pasos(target_deg)
        self._send_cmd(f"pp{self.target_steps}")
        self._last_cmd_send_time = time.monotonic()

        # Inicia verificación
        self.waiting_for_pp = True
        self.last_rx_position = None
        self.query_started = time.monotonic()
        if self.query_timer is None:
            self.query_timer = self.create_timer(self.query_period, self._query_tick)

        # Primera consulta
        self._send_cmd('pp')

        # Espera a que termine (éxito o timeout/abort)
        self._done_evt.wait()

        # resultado
        result = PtuSweep.Result()
        result.success = self._success
        result.message = self._final_message

        if self._success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._active_goal = None
        return result
    
    def _query_tick(self):
        if not self.waiting_for_pp:
            return

        # ¿llegó?
        if self.last_rx_position is not None and self._matches_target(self.last_rx_position):
            ang = pasos_a_grados(self.last_rx_position)
            self._publish_feedback(ang, f"Llegó a {ang}°")
            self._stop_query_loop()
            self._finish(True, "OK")
            return

        # timeout
        elapsed = time.monotonic() - self.query_started
        if elapsed > self.query_timeout_s:
            aprox = pasos_a_grados(self.last_rx_position) if self.last_rx_position is not None else 0
            self._publish_feedback(aprox, f"Timeout verificando llegada a {self.target_steps} pasos")
            self._stop_query_loop()
            self._finish(False, "Timeout")
            return

        # re-consulta y reinyecta setpoint
        self._send_cmd('pp')
        now = time.monotonic()
        if (self.target_steps is not None
                and self._last_cmd_send_time is not None
                and (now - self._last_cmd_send_time) >= self.command_resend_period):
            self._send_cmd(f'pp{self.target_steps}')
            self._last_cmd_send_time = now

    def _stop_query_loop(self):
        self.waiting_for_pp = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None
        self._last_cmd_send_time = None

    def _matches_target(self, pos_steps: int) -> bool:
        return (self.target_steps is not None and
                abs(pos_steps - self.target_steps) <= self.tolerance_steps)
    

    def _publish_feedback(self, angle_deg: int, status: str):
        if self._active_goal:
            fb = PtuSweep.Feedback()
            fb.current_angle = angle_deg
            fb.status = status
            self._active_goal.publish_feedback(fb)
    
    # publicar comando
    def _send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def _finish(self, ok: bool, msg: str):
        self._success = ok
        self._final_message = msg
        self._done_evt.set()

    def rx_cb(self, msg: String):
        # busca entero con signo en la línea
        m = self._pan_re.search(msg.data.strip())
        #self.get_logger().info(f"mensaje: {m}")
        if m:
            pos = int(m.group(1))
            self.last_rx_position = pos


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
