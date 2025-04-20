#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
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

    def listener_callback(self, msg: RadarData):
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

def main(args=None):
    rclpy.init(args=args)
    node = RadarDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()