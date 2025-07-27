#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from radar_msg.msg import RadarData

class RadarDataPublisher(Node):
    def __init__(self):
        super().__init__('publish_radar_data')
        # Tópico de publicación
        self.pub = self.create_publisher(RadarData, 'radar_data', 10)
        # Parámetro: ruta al .npy
        self.declare_parameter('ruta_npy', '/home/diego/Desktop/magister/magister_ws/src/radar_msg/scripts/pos_0_0_angle_-30_0.npy')
        # Programa el callback a 5 s
        self._timer = self.create_timer(5.0, self.publish_data_radar)

    def publish_data_radar(self):
        ruta = self.get_parameter('ruta_npy').get_parameter_value().string_value
        arr = np.load(ruta) # shape (161,4096), dtype float64

        msg = RadarData()
        msg.rows = arr.shape[0]
        msg.cols = arr.shape[1]
        msg.dtype = str(arr.dtype)
        msg.data = arr.flatten().tolist()

        self.pub.publish(msg)
        self.get_logger().info(
            f'Publicado RadarData: {msg.rows} {msg.cols}, dtype={msg.dtype}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()