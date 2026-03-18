from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#from radar_package.parametros import *

def generate_launch_description():
    pkg = get_package_share_directory('radar_package')

    radar_params = os.path.join(pkg, 'config', 'radar.yaml')
    processing_params   = os.path.join(pkg, 'config', 'processing.yaml')
    ptu_params   = os.path.join(pkg, 'config', 'ptu.yaml')

    return LaunchDescription([
        # ----------------- hardware connection -----------------------
        Node(
            package='radar_package',               # Paquete ROS
            executable='radar_node',          # nodo ejecutable
            name='radar_node',                # nombre en 'ros2 node list'
            parameters=[radar_params],# parametros admitidos en la ejecucion
            output='screen',                       # salida en la terminal
        ),
        Node(
            package='ptu_driver',
            executable='ptu_node_driver', 
            name='ptu_node_driver',
            parameters=[{
                'serial_port': '/dev/ttyUSB0' # para pc bunker debe estar en ttyUSB1, pc host ttyUSB0 -> identificado de forma manual
            }],
            output='screen',
        ),
        Node(
            package='ptu_package',
            executable='ptu_node',
            name='ptu_node',
            output='screen'
        ),
        # ------------------------------------------------------------
        # ----------------- visualizer -----------------------
        Node(
            package='radar_package',
            executable='radar_visualizer',
            name='radar_visualizer',
            parameters=[radar_params, processing_params],
            output='screen'
        ),
        Node(
            package='radar_package',
            executable='mapa_cartesiano',
            name='mapa_cartesiano',
            output='screen'
        ),
        #Node(
        #    package='radar_package',
        #    executable='waterfall',
        #    name='waterfall',
        #    parameters=[radar_params],
        #    output='screen'
        #),
        # ------------------------------------------------------------
        # ----------------- data processing -----------------------
        Node(
            package='radar_package',
            executable='data_processing',
            name='data_processing',
            parameters=[radar_params, processing_params],
            output='screen'
        ),
        Node(
            package='radar_package',
            executable='radar_map_accumulator',
            name='radar_map_accumulator',
            output='screen'
        ),
        #Node(
        #    package='radar_package',
        #    executable='medicion_fondo',
        #    name='medicion_fondo',
        #    parameters=[radar_params],
        #    output='screen'
        #),
        # ------------------------------------------------------------
        # ----------------- Simulador -----------------------
        #Node(
        #    package='radar_package',
        #    executable='simulador_radar',
        #    name='simulador_radar',
        #    parameters=[ptu_params],
        #    output='screen'
        #),
        #Node(
        #    package='radar_package',
        #    executable='simulador_ptu',
        #    name='simulador_ptu',
        #    output='screen'
        #),
        # ---------------------------------------------------
        # ----------------- fuera de uso -----------------------
        #Node(
        #    package='radar_package',
        #    executable='process_data_node',
        #    name='process_data_node',
        #    parameters=[radar_params, processing_params],
        #    output='screen'
        #),
        #Node(
        #    package='radar_package',
        #    executable='mosaic_node',
        #    name='mosaic_node',
        #    parameters=[processing_params],
        #    output='screen'
        #),
        #Node(
        #    package='radar_package',
        #    executable='data_processing_node',
        #    name='data_processing_node',
        #    parameters=[radar_params, processing_params],
        #    output='screen'
        #),
        # ------------------------------------------------------------
    ])

