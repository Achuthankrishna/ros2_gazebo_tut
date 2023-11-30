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
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition

def generate_launch_description():
    topics=LaunchConfiguration('topics')
    rbag=DeclareLaunchArgument('bag_record',default_value=TextSubstitution(text="True"),choices=['True','False'],description="Recording ROS bag")

    return LaunchDescription([

        DeclareLaunchArgument('topics',default_value='True'),

        Node(
            package='ros2_gazebo_tut',
            executable='walker',
            parameters=[
                {"topics":LaunchConfiguration('topics')}

            ]
        ),
        # ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('bag_record')),
            cmd=['ros2', 'bag', 'record', '-o', "./src/ros2_gazebo_tut/results/Bags",'-a','-x','/scan.*'],
            shell=True
        )
    ])