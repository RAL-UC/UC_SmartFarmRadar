from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='radar_package',               # Paquete ROS
        #    executable='radar_sync_node',          # nodo ejecutable
        #    name='radar_sync_node',                # nombre en 'ros2 node list'
        #    output='screen',                       # salida en la terminal
        #    parameters=[{
        #        'angle_min': -80,                  # parametros admitidos en la ejecucion
        #        'angle_max': 80,
        #        'angle_step': 1
        #    }]
        #),
        #Node(
        #    package='ptu_package',
        #    executable='ptu_node',
        #    name='ptu_node',
        #    output='screen',
        #    parameters=[{
        #        'serial_port': '/dev/ttyUSB0'
        #    }]
        #),
        #Node(
        #    package='ptu_routine',
        #    executable='ptu_routine_node',
        #    name='ptu_routine_node',
        #    output='screen'
        #)
        #Node(
        #    package='radar_package',
        #    executable='visulizacion_node',
        #    name='visulizacion_node',
        #    output='screen',
        #    parameters=[{
        #        'angle_min': -80,
        #        'angle_max': 80,
        #        'angle_step': 1
        #    }]
        #),
        Node(
            package='radar_package',
            executable='process_data_node',
            name='process_data_node',
            output='screen',
            parameters=[{
                'angle_min': -80,
                'angle_max': 80,
                'angle_step': 1
            }]
        )
    ])

