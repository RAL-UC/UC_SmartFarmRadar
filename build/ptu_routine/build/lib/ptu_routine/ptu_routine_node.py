import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import re

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
        self.allow_sub = self.create_subscription(Bool, '/allow_routine_ptu', self.listener_callback, 10)
        self.rx_sub = self.create_subscription(String, '/ptu_response', self.rx_cb, 50)

        self.ptu_angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        self.current_index = 0

        self.target_steps = None # objetivo en pasos
        self.waiting_for_pp = False # esperando respuesta a 'pp'
        self.last_rx_position = None # última posición en pasos
        self.query_timer = None # timer de consulta periódica
        self.query_period = 0.5 # s entre consultas 'pp'
        self.query_timeout_s = 8.0 # timeout total para confirmar
        self.query_started = None # timestamp de inicio
        self.tolerance_steps = 2 # tolerancia de coincidencia en pasos


        self.get_logger().info('Publica True en /allow_routine_ptu para avanzar un paso del recorrido.')

    def allow_cb(self, msg: Bool):
        if not msg.data or self.waiting_for_pp:
            return

        # calculo del paso y el comando
        angulo = self.ptu_angles[self.current_index]
        self.target_steps = grados_a_pasos(angulo)
        comando = f"pp{self.target_steps}"

        # publicar
        self._send_cmd(comando)
        self.get_logger().info(
            f"Paso {self.current_index+1}/{len(self.ptu_angles)}: "
            f"Enviado '{comando.strip()}' ({angulo}° a {self.target_steps } pasos). Verificando llegada..."
        )

        self._start_query_loop() # loop de verificacion
    
    # publicar comando
    def _send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    # loop de consulta 'pp' hasta coincidir o timeout
    def _start_query_loop(self):
        self.waiting_for_pp = True
        self.last_rx_position = None
        self.query_started = time.monotonic()

        # Enviar primera consulta
        self._send_cmd('pp')

        # timer periódico
        if self.query_timer is None:
            self.query_timer = self.create_timer(self.query_period, self._query_tick)

    # cada tick re-consulta y evalúa timeout
    def _query_tick(self):
        if not self.waiting_for_pp:
            return

        # si ya tenemos posición y coincide -> éxito
        if self.last_rx_position is not None and self._matches_target(self.last_rx_position):
            self._on_reached_target(self.last_rx_position)
            return

        # revisar si se supera el timeout
        elapsed = time.monotonic() - self.query_started
        if elapsed > self.query_timeout_s:
            self._on_query_timeout()
            return

        # Reenviar 'pp' para consultar
        self._send_cmd('pp')

    def _matches_target(self, pos_steps: int) -> bool:
        if self.target_steps is None:
            return False
        return abs(pos_steps - self.target_steps) <= self.tolerance_steps
    
    def _on_reached_target(self, pos_steps: int):
        # parar loop
        self.waiting_for_pp = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None

        ang = pasos_a_grados(pos_steps)
        self.get_logger().info(
            f"Llegó a {pos_steps} pasos ({ang:.2f}°)."
        )

        # avanzar índice y reiniciar si esta al final
        self.current_index += 1
        if self.current_index >= len(self.ptu_angles):
            self.current_index = 0
            self.get_logger().info("Recorrido completado. Siguiente True reinicia en -90°.")

    def _on_query_timeout(self):
        self.waiting_for_pp = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None

        self.get_logger().warn(
            f"Timeout verificando llegada a {self.target_steps} pasos. "
            f"Última lectura: {self.last_rx_position}."
        )
        # No avanzamos de índice: requerirá otro True para reintentar o pasar al siguiente

    def rx_cb(self, msg: String):
        line = msg.data.strip()
        # busca entero con signo en la línea
        m = re.search(r'Current\s+Pan\s+position\s+is\s+(-?\d+)', line, re.IGNORECASE)
        if m:
            pos = int(m.group(1))
            self.last_rx_position = pos
            if self.waiting_for_pp and self._matches_target(pos):
                # confirmación inmediata sin esperar otro tick
                self._on_reached_target(pos)


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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
