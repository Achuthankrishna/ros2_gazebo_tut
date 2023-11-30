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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    topics = DeclareLaunchArgument('topics', default_value='True')
    rbag = DeclareLaunchArgument('bag_record', default_value=TextSubstitution(text="True"), choices=['True', 'False'], description="Recording ROS bag")

    return LaunchDescription([
        topics,
        rbag,
        Node(
            package='ros2_gazebo_tut',
            executable='walker',
            name='MovementBot',
            parameters=[{"topics": LaunchConfiguration('topics')}]
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('bag_record')),
            cmd=[
                'ros2', 'bag', 'record',
                '/camera/camera_info','/tf', '/clock', '/cmd_vel',
                '/events/read_split', '/odom', '/parameter_events','/robot_description', '/tf_static',
                '--output', "./src/ros2_gazebo_tut/results/Bags"
            ],
            shell=True
        )
    ])


