#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    max_index_arg = DeclareLaunchArgument(
        'max_index',
        default_value='113',
        description='Maximum index of images to publish (0 to max_index)'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='10.0',
        description='Frequency (Hz) at which to publish images'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop through images continuously (true/false)'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/camera/image_raw',
        description='Topic name for publishing images'
    )
    
    # Create the node
    image_publisher_node = Node(
        package='computer_vision',
        executable='image_publisher_node.py',
        name='image_publisher_node',
        output='screen',
        parameters=[{
            'max_index': LaunchConfiguration('max_index'),
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'loop': LaunchConfiguration('loop'),
            'topic_name': LaunchConfiguration('topic_name'),
        }]
    )
    
    return LaunchDescription([
        max_index_arg,
        frequency_arg,
        loop_arg,
        topic_arg,
        image_publisher_node,
    ])

