import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Try to find tomogram_rsc share directory
    try:
        tomogram_rsc_share = get_package_share_directory('tomogram_rsc')
    except Exception:
        raise AttributeError("tomogram_rsc package not found. Please ensure it is installed or provide the absolute path.")

    rsg_root_arg = DeclareLaunchArgument(
        'rsg_root',
        default_value=tomogram_rsc_share,
        description='Root directory for resources (containing pcd, rviz, tomogram folders)'
    )

    scene_name_arg = DeclareLaunchArgument(
        'scene_name',
        default_value='plaza',
        description='Name of the scene to load (e.g., plaza, building)'
    )

    # Node configuration
    tomography_node = Node(
        package='tomography',
        executable='tomography_node',
        name='tomography_node',
        output='screen',
        parameters=[{
            'rsg_root': LaunchConfiguration('rsg_root'),
            'scene_name': LaunchConfiguration('scene_name')
        }]
    )

    # RViz configuration
    rviz_config_path = [LaunchConfiguration('rsg_root'), '/rviz/pct_ros2.rviz']
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rsg_root_arg,
        scene_name_arg,
        rviz_node,
        tomography_node
    ])
