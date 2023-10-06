from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='figueroa_ros2_project',
            executable='printer_sim_node',
            name='printer_node',
        ),
        Node(
            package='figueroa_ros2_project',
            executable='service_node',
            name='service_server_node',
        ),
        Node(
            package='figueroa_ros2_project',
            executable='robot_logic',
            name='logic_sim_node',
        ),
        Node(
            package='figueroa_ros2_project',
            executable='motor_x',
            name='motor_x_control_sim_node',
        ),
        Node(
            package='figueroa_ros2_project',
            executable='motor_y',
            name='motor_y_control_sim_node',
        ),
    ])
