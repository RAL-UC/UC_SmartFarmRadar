from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Calibración del phaser: uso puntual, no lanzar junto con launch.py (mismo hardware)
def generate_launch_description():
    pkg = get_package_share_directory('radar_package')

    radar_params = os.path.join(pkg, 'config', 'radar.yaml') # conexion y hardware
    calibracion_params = os.path.join(pkg, 'config', 'calibracion.yaml') # pasos y medicion

    return LaunchDescription([
        Node(
            package='radar_package', # Paquete ROS
            executable='calibracion', # nodo ejecutable
            name='calibracion_node', # nombre en 'ros2 node list'
            parameters=[radar_params, calibracion_params], # parametros admitidos en la ejecucion
            output='screen', # salida en la terminal
        ),
    ])
