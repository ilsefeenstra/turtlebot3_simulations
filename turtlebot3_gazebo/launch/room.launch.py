import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    world_path = LaunchConfiguration('world_path', default='/home/ifeenstra/ros2_ws/maps/train/train_1/random_room_world_1.sdf')
    pcd_path = LaunchConfiguration('pcd_path', default='/home/ifeenstra/ros2_ws/maps/train/train_1/random_room_map_1.pcd')

    robot_node = Node(
        package='robot',
        executable='robot',
        name='robot',
        parameters=[{'pcd_path': pcd_path}, {'world_path': world_path}],
        output='screen'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ])
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        )
    )
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_path', default_value=world_path),
        DeclareLaunchArgument('pcd_path', default_value=pcd_path),
        robot_node,
        # gzserver_cmd,
        # gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_entity_cmd
        
    ])