import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Try to find tomogram_rsc share directory for resources (if needed)
    try:
        tomogram_rsc_share = get_package_share_directory('tomogram_rsc')
    except Exception:
        # Fallback or just let it fail if critical
        tomogram_rsc_share = '/tmp' 

    # Example argument - adjust based on what planner_node actually needs
    rsg_root_arg = DeclareLaunchArgument(
        'rsg_root',
        default_value=tomogram_rsc_share,
        description='Root directory for resources'
    )

    scene_name_arg = DeclareLaunchArgument(
        'scene_name',
        default_value='Plaza',
        description='Name of the scene to load (e.g., Plaza, Building, Spiral)'
    )

    # Node configuration for pct_planner
    planner_node = Node(
        package='pct_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'rsg_root': LaunchConfiguration('rsg_root'),
            'scene_name': LaunchConfiguration('scene_name'),
        }]
    )

    return LaunchDescription([
        rsg_root_arg,
        scene_name_arg,
        planner_node
    ])
