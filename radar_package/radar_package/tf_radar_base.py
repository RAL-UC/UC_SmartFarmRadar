#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster  # o TransformBroadcaster si dinámico
import tf_transformations
from builtin_interfaces.msg import Time as TimeMsg

class RadarTF(Node):
    def __init__(self):
        super().__init__('radar_tf_broadcaster')
        self.br = StaticTransformBroadcaster(self)  # para TF estática
        # Define la pose de base_radar respecto de base_link
        self.publish_static_tf()

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'     # padre
        t.child_frame_id  = 'base_radar'    # hijo

        # translación en metros
        t.transform.translation.x = 0.10
        t.transform.translation.y = -0.05
        t.transform.translation.z = 0.20

        # orientación: aquí desde RPY a cuaternión
        roll, pitch, yaw = 0.0, 0.0, 0.0
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)
        self.get_logger().info("TF estática publicada: base_link -> base_radar")

def main():
    rclpy.init()
    node = RadarTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
