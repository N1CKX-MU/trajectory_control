from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share   = get_package_share_directory('trajectory_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    nodes = TimerAction(period=1.0, actions=[
        Node(package='trajectory_controller',
             executable='waypoints_node',
             name='waypoints_node',
             output='screen',
             parameters=[params_file]),
        Node(package='trajectory_controller',
             executable='catmullrom_node',
             name='catmullrom_node',
             output='screen',
             parameters=[params_file]),
        Node(package='trajectory_controller',
             executable='bezier_node',
             name='bezier_node',
             output='screen',
             parameters=[params_file]),
        Node(package='trajectory_controller',
             executable='gradient_descent_node',
             name='gradient_descent_node',
             output='screen',
             parameters=[params_file]),
        Node(package='trajectory_controller',
             executable='trajectory_generator_node',
             name='trajectory_generator_node',
             output='screen',
             parameters=[params_file]),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('trajectory_controller'),
                 'rviz', 'visualize_trajectory.rviz'
             ])],
             output='screen'),
    ])

    return LaunchDescription([nodes])
