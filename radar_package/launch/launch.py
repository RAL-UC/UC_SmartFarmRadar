from launch import LaunchDescription
from launch_ros.actions import Node
from radar_package.parametros import *

def generate_launch_description():
    return LaunchDescription([
        # ----------------- hardware connection -----------------------
        Node(
            package='radar_package',               # Paquete ROS
            executable='radar_node',          # nodo ejecutable
            name='radar_node',                # nombre en 'ros2 node list'
            output='screen',                       # salida en la terminal
            #parameters=[{
            #    'angle_min': ANGLE_MIN_RADAR_BEAM,                  # parametros admitidos en la ejecucion
            #    'angle_max': ANGLE_MAX_RADAR_BEAM,
            #    'angle_step': ANGLE_STEP_RADAR_BEAM
            #}]
        ),
        Node(
            package='ptu_package',
            executable='ptu_node',
            name='ptu_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0' # para pc bunker debe estar en ttyUSB1, pc host ttyUSB0
            }]
        ),
        Node(
            package='ptu_routine',
            executable='ptu_routine_node',
            name='ptu_routine_node',
            output='screen'
        ),
        # ------------------------------------------------------------
        # ----------------- visualizer -----------------------
        Node(
            package='radar_package',
            executable='radar_visualizer',
            name='radar_visualizer',
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
        #    output='screen'
        #),
        # ------------------------------------------------------------
        # ----------------- data processing -----------------------
        Node(
            package='radar_package',
            executable='data_processing',
            name='data_processing',
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
        #    output='screen'
        #),
        # ------------------------------------------------------------
        # ----------------- Simulador -----------------------
        #Node(
        #    package='radar_package',
        #    executable='simulador_radar',
        #    name='simulador_radar',
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
        #    output='screen'
        #),
        #Node(
        #    package='radar_package',
        #    executable='mosaic_node',
        #    name='mosaic_node',
        #    output='screen'
        #),
        #Node(
        #    package='radar_package',
        #    executable='data_processing_node',
        #    name='data_processing_node',
        #    output='screen'
        #),
        # ------------------------------------------------------------
    ])

