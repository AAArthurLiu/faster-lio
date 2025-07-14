from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def check_config_file(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    assert config_file
    return []

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Name of the config file for faster_lio.'
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('faster_lio'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # Node 1
    laser_mapping_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )

    # Node 2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '/home/arthurycliu/repo/faster-lio/src/faster_lio/rviz_cfg/loam_livox.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        config_file_arg,
        OpaqueFunction(function=check_config_file),
        laser_mapping_node,
        rviz_node
    ])
