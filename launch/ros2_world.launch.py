"""
/**
 * @file subscriber_func.hpp
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
"""
import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.5')

    launch_pkg = os.path.join(get_package_share_directory('ros2_gazebo_tut'), 'launch')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    #Getting from the turlebot3 package
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    actions= [
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    ),

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    ),

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    ),

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pkg, 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    ),
    ]

    ld = LaunchDescription()
    actions.append(Node(
        package='walker',
        executable='walker',
        name='walker',
        output='screen',
    ))
    
    return ld(actions)
