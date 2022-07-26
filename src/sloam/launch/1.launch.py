from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    
    frameFeature_node = Node(
        package="sloam",
        executable="frameFeature"
        )
    
    lidarOdometry_node = Node(
        package="sloam",
        executable="lidarOdometry"
        )
    
    MapOptmization_node = Node(
        package="sloam",
        executable="MapOptmization"
        )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[],
            output='screen'
        )
    
    launch_description = LaunchDescription([
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', "-l", 'bag/bag.db3'],
            # cmd=['ros2', 'bag', 'play', "-l", 'bag/last.db3'],
            output='screen'
        ),
        frameFeature_node, lidarOdometry_node, rviz_node,
        # Node(
        #     package='imu_calibration_tools',
        #     executable='eskf_node',
        #     name='eskf_node',
        #     output='screen'
        # ),
        MapOptmization_node,
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
        #    ),
        Node(
            package='imu_calibration_tools',
            executable='a_star_node',
            name='a_star_node',
            output='screen'
        ),
        ])
        
    return launch_description
