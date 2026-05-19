import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Parse collision_skip_pairs as a list from a comma-separated string
    collision_skip_pairs_str = LaunchConfiguration('collision_skip_pairs').perform(context)
    collision_skip_pairs = collision_skip_pairs_str.split(',') if collision_skip_pairs_str else []

    # All other parameters must be resolved to Python types (not LaunchConfiguration) for OpaqueFunction
    trajectory_time_step = float(LaunchConfiguration('trajectory_time_step').perform(context))
    planning_timeout = float(LaunchConfiguration('planning_timeout').perform(context))
    base_link = str(LaunchConfiguration('base_link').perform(context))
    right_tip = str(LaunchConfiguration('right_tip').perform(context))
    tcp_offset_x = LaunchConfiguration('tcp_offset_x').perform(context)
    planner_type = str(LaunchConfiguration('planner_type').perform(context))
    adaptive_orientation_enabled = LaunchConfiguration('adaptive_orientation_enabled').perform(context)
    config_file = str(LaunchConfiguration('config_file').perform(context))

    parameters = {
        'trajectory_time_step': trajectory_time_step,
        'planning_timeout': planning_timeout,
        'base_link': base_link,
        'right_tip': right_tip,
        'planner_type': planner_type,
    }
    if tcp_offset_x:
        parameters['tcp_offset_x'] = float(tcp_offset_x)
    if adaptive_orientation_enabled:
        parameters['adaptive_orientation_enabled'] = adaptive_orientation_enabled.lower() == 'true'
    if collision_skip_pairs:
        parameters['collision_skip_pairs'] = collision_skip_pairs

    return [
        Node(
            package='unitree_g1_dex3_stack',
            executable='ik_fcl_ompl_planner',
            name='ik_fcl_ompl_planner',
            output='screen',
            parameters=[config_file, parameters],
            arguments=[
                '--ros-args',
                '--log-level',
                'trac_ik.ros.trac_ik:=DEBUG'
            ]
        )
    ]

def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    default_config_file = os.path.join(package_share, 'config', 'apriltag.yaml')

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config_file),
        DeclareLaunchArgument('trajectory_time_step', default_value='0.05'),
        DeclareLaunchArgument('planning_timeout', default_value='1.0'),
        DeclareLaunchArgument('base_link', default_value='torso_link'),
        DeclareLaunchArgument('right_tip', default_value='right_tcp_link'),
        DeclareLaunchArgument('tcp_offset_x', default_value=''),
        DeclareLaunchArgument('planner_type', default_value='RRTConnect'),
        DeclareLaunchArgument('adaptive_orientation_enabled', default_value=''),
        DeclareLaunchArgument('collision_skip_pairs', default_value='right_hand_thumb_0_link:right_wrist_yaw_link'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
