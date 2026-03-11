from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    tb3_gazebo= get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('trajectory_controller')
    params_file    = os.path.join(pkg_share, 'config', 'params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': os.path.join(
            pkg_share, 'worlds', 'obstacle_course.world'
        )}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
    )

    waypoints = TimerAction(period=5.0, actions=[
        Node(package='trajectory_controller',
             executable='waypoints_node',
             name='waypoints_node',
             output='screen',
             parameters=[params_file])
    ])

    catmullrom = TimerAction(period=5.0, actions=[
        Node(package='trajectory_controller',
             executable='catmullrom_node',
             name='catmullrom_node',
             output='screen',
             parameters=[params_file])
    ])

    bezier = TimerAction(period=5.0, actions=[
        Node(package='trajectory_controller',
             executable='bezier_node',
             name='bezier_node',
             output='screen',
             parameters=[params_file])
    ])

    gradient = TimerAction(period=5.0, actions=[
        Node(package='trajectory_controller',
             executable='gradient_descent_node',
             name='gradient_descent_node',
             output='screen',
             parameters=[params_file])
    ])

    trajectory = TimerAction(period=5.0, actions=[
        Node(package='trajectory_controller',
             executable='trajectory_generator_node',
             name='trajectory_generator_node',
             output='screen',
             parameters=[params_file])
    ])

    detector = TimerAction(period=6.0, actions=[
        Node(package='trajectory_controller',
             executable='obstacle_detector_node',
             name='obstacle_detector_node',
             output='screen',
             parameters=[params_file])
    ])

    avoider = TimerAction(period=7.0, actions=[
        Node(package='trajectory_controller',
             executable='obstacle_avoider_node',
             name='obstacle_avoider_node',
             output='screen',
             parameters=[params_file])
    ])

    controller = TimerAction(period=15.0, actions=[
        Node(package='trajectory_controller',
             executable='controller_node',
             name='controller_node',
             output='screen',
             parameters=[params_file])
    ])

    rviz = TimerAction(period=5.0, actions=[
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('trajectory_controller'),
                 'rviz', 'visualize_trajectory.rviz'
             ])],
             output='screen')
    ])

    return LaunchDescription([
        gazebo,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        waypoints,
        catmullrom,
        bezier,
        gradient,
        trajectory,
        detector,
        avoider,
        controller,
        rviz,
    ])
