#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped # definicion de mensaje geométricos: posiciones, vectores, poses, transformaciones, etc
from tf2_ros import StaticTransformBroadcaster  # publicar transformaciones estáticas entre marcos de referencia
import tf_transformations # operaciones matemáticas con transformaciones 3D, especialmente rotaciones y orientaciones
#from builtin_interfaces.msg import Time as TimeMsg

# broadcaster: publicar informacion de forma continua -> locutor

class RadarTF(Node):
    def __init__(self):
        super().__init__('radar_tf_broadcaster')
        self.br = StaticTransformBroadcaster(self) # para TF estática
        # Define la pose de base_radar respecto de base_link
        self.publish_static_tf()

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() # marca temporal
        t.header.frame_id = 'base_link' # padre: quien es ?
        t.child_frame_id  = 'base_radar' # hijo: quien es ?

        # translación en metros: coordenadas reales
        t.transform.translation.x = 0.10
        t.transform.translation.y = -0.05
        t.transform.translation.z = 0.20

        # orientación: aquí desde RPY a cuaternión
        # RPY: roll, pitch, yaw
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
