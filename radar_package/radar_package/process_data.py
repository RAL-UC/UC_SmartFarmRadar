#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData

class RadarDataSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe_radar_dara')
        self.create_subscription(
            RadarData,
            'radar_data',
            self.listener_callback,
            10
        )
        # Inicializar ventana de Matplotlib
        plt.ion()  # Modo interactivo
        self.fig, self.ax = plt.subplots()
        self.im = None

    def listener_callback(self, msg: RadarData):
        # Mostrar Header
        self.get_logger().info(
            f'Recibido header: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, frame_id="{msg.header.frame_id}"'
        )
        # Reconstruir numpy array
        arr = np.array(msg.data, dtype=msg.dtype)
        arr = arr.reshape((msg.rows, msg.cols))
        # Mostrar info
        self.get_logger().info(
            f'Recibido RadarData: {arr.shape}, dtype={arr.dtype}'
        )
        # Muestra los primeros 5 valores
        flat = arr.flatten()
        preview = ', '.join(f'{v:.2f}' for v in flat[:5])
        self.get_logger().info(f'Primeros 5 valores: [{preview}]')

        # --- NUEVA PARTE: Mostrar matriz ---
        if self.im is None:
            # Primera vez: crear la imagen
            self.im = self.ax.imshow(arr, aspect='auto', cmap='viridis', origin='lower')
            self.fig.colorbar(self.im)
        else:
            # Actualizar imagen
            self.im.set_data(arr)
            self.im.set_clim(vmin=np.min(arr), vmax=np.max(arr))
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = RadarDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()