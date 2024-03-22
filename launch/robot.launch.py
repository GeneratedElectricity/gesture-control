#! /usr/bin/env python3
import xacro
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
import launch.conditions
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    # Path to xacro file

    xacro_file = get_package_share_directory('gesturecontrol') + '/urdf/mr_robot.xacro'
  

    # Include the gazebo.launch.py file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'pause': 'false'}.items()
    )

    # Publishing robot_state into topic robot_description
    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command([
            'xacro ', xacro_file,
            # ' kinect_enabled:=', kinect_enabled,
            # ' lidar_enabled:=', lidar_enabled,
            # ' camera_enabled:=', camera_enabled,
        ])}]
    )
    
    # Spawn mr_robot using the topic "/robot_description"
    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_mr_robot',
        output='screen',
        arguments=[
            '-entity', 'mr_robot',
            '-topic', '/robot_description',
            '-x', '0.0',  # Set the initial X position
            '-y', '0.0',  # Set the initial Y position
            '-z', '0.0',  # Set the initial Z position
            '-Y', '-3.14'  # Set the initial Z position
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state,
        robot_spawn
    ])


