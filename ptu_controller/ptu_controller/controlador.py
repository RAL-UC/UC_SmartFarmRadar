#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

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

        # Suscriptor único para comandos genéricos
        self.sub_cmd = self.create_subscription(String, '/ptu_command', self.cb_cmd, 10)

    def send_raw(self, cmd_str: str):
        """Envía cualquier comando crudo y lee múltiples líneas de respuesta del PTU."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning('Puerto serie no disponible')
            return

        # Enviar comando con terminador CR+LF
        cmd_str = cmd_str + "\r"
        cmd = f"{cmd_str}".encode('ascii')
        self.ser.write(cmd)
        #self.ser.write("\r\n".encode('ascii'))
        self.get_logger().info(f'Enviado: {cmd!r}')

        time.sleep(0.05)  # pequeña espera para asegurar que llegue la respuesta (ajustable)

        # Leer múltiples líneas de respuesta
        try:
            lines = []
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    lines.append(line)

            if lines:
                for i, line in enumerate(lines):
                    self.get_logger().info(f'Respuesta línea {i+1}: {line}')
            else:
                self.get_logger().warning('No se recibió respuesta del PTU')

        except Exception as e:
            self.get_logger().error(f'Error leyendo respuesta: {e}')

    def cb_cmd(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            self.get_logger().warning('Comando vacío recibido en /ptu_command')
            return
        self.send_raw(cmd)

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
