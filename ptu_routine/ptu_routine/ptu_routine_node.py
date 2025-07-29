import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

# convencion
# positivo izquierda
# negativo derecha 

# limites
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
    grados = pasos * resolucion_grados
    return grados

class PtuRoutineNode(Node):
    def __init__(self):
        super().__init__('ptu_routine_node')
        self.publisher_ = self.create_publisher(String, '/ptu_cmd', 10)
        self.subscription = self.create_subscription(Bool, '/allow_routine_ptu', self.listener_callback, 10)
        self.angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        self.current_index = 0

        self.get_logger().info('Publica True en /allow_routine_ptu para avanzar.')

    def listener_callback(self, msg: Bool):
        if not msg.data:
            return

        # calculo del paso y el comando
        angulo = self.angles[self.current_index]
        pasos = grados_a_pasos(angulo)
        comando = f"pp{pasos}"

        # publicar
        out = String()
        out.data = comando
        self.publisher_.publish(out)

        self.get_logger().info(
            f"Paso {self.current_index+1}/{len(self.angles)}: "
            f"Enviado '{comando.strip()}' ({angulo}° a {pasos} pasos)"
        )

        # avanzar el índice y reiniciar si esta al final
        self.current_index += 1
        if self.current_index >= len(self.angles):
            self.current_index = 0
            self.get_logger().info("Recorrido completado: próximo True reinicia en ángulo -90°")

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
