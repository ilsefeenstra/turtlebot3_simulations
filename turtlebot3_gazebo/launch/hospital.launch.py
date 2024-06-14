#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command

# def generate_launch_description():
#     launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
#     # Fixed start position
#     x_pose = LaunchConfiguration('x_pose', default='-2.0')
#     y_pose = LaunchConfiguration('y_pose', default='-0.5')

#     world = os.path.join(
#         get_package_share_directory('turtlebot3_gazebo'),
#         'worlds',
#         'hospital.world'
#     )

#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     robot_state_publisher_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
#         ),
#         launch_arguments={'use_sim_time': use_sim_time}.items()
#     )

    
#     spawn_turtlebot_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
#         ),
#         launch_arguments={
#             'x_pose': x_pose,
#             'y_pose': y_pose
#         }.items()
        
#     )

#     ld = LaunchDescription()

#     # Add the commands to the launch description
#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_turtlebot_cmd)

   
#     return ld

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_position_publisher = get_package_share_directory('robot')
    pcd_path = '/home/ifeenstra/ros2_ws/maps/map_4.pcd'
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')
    world_path = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'hospital.world')
    
    position_publisher_node = Node(
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
        launch_arguments={'world': os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'hospital.world')}.items()
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

    return LaunchDescription([
        position_publisher_node,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd
    ])