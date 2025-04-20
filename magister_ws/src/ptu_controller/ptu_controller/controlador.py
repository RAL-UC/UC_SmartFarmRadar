#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

def grados_a_pasos(grados):
    """Convierte grados a pasos PTU‑D46 (0.0514285° por paso)."""
    resolucion = 185.1428 / 3600  # en grados
    return round(grados / resolucion)

class PTUController(Node):
    def __init__(self):
        super().__init__('ptu_controller')
        # Parámetros de conexión serie
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info(f'Abierta conexión serie en {port}@{baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir {port}: {e}')
            self.ser = None

        # Suscriptores para pan y tilt
        self.sub_pan  = self.create_subscription(Float64, '/pan_angle',  self.cb_pan, 10)
        self.sub_tilt = self.create_subscription(Float64, '/tilt_angle', self.cb_tilt, 10)

    def send_cmd(self, prefix: str, pasos: int):
        """Formatea y envía comando. Ej: 'PN1234' + CR+LF."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning('Puerto serie no disponible')
            return
        cmd = f"{prefix}{pasos}\r\n".encode('ascii')
        self.ser.write(cmd)
        self.get_logger().info(f'Enviado: {cmd!r}')

    def cb_pan(self, msg: Float64):
        pasos = grados_a_pasos(msg.data)
        # límite [-3090, +3090]
        pasos = max(-3090, min(3090, pasos))
        self.send_cmd('pt', pasos)

    def cb_tilt(self, msg: Float64):
        pasos = grados_a_pasos(msg.data)
        # límite [-907, +604]
        pasos = max(-907, min(604, pasos))
        self.send_cmd('TN', pasos)

def main(args=None):
    rclpy.init(args=args)
    node = PTUController()
    try:
        rclpy.spin(node)
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
