from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    frameFeature_node = Node(
        package="sloam",
        executable="frameFeature"
        )
    
    lidarOdometry_node = Node(
        package="sloam",
        executable="lidarOdometry"
        )

    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[],
            output='screen'
        )
    
    launch_description = LaunchDescription([frameFeature_node, lidarOdometry_node])
        
    return launch_description
