import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
    realsense_share = get_package_share_directory('realsense2_camera')

    # ---------- launch arguments ----------
    imshow_arg = DeclareLaunchArgument(
        'imshow',
        default_value='true',
        description='Open OpenCV detection window',
    )

    adaptive_arg = DeclareLaunchArgument(
        'adaptive_orientation_enabled',
        default_value='true',
        description='Pass through to planner.launch.py',
    )

    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='1.0',
        description='Planning timeout in seconds',
    )

    # ---------- immediate-start components ----------

    # 4a. robot.launch.py include (CycloneDDS env owner, do NOT duplicate)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py')
        )
    )

    # 4b. RealSense rs_launch.py include (RGB only, 640x480x15, no depth)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'serial_no': '_243722074823',
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
            'align_depth.enable': 'false',
            'rgb_camera.color_profile': '640x480x15',
            'initial_reset': 'true',
        }.items(),
    )

    # 4c. d435_link -> camera_link static TF
    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
    )

    # ---------- delayed components (TimerAction period=3.0) ----------

    # 5a. apriltag_detector_node
    apriltag_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_detector_node.py',
        name='apriltag_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(package_share, 'config', 'apriltag.yaml'),
            {'imshow': LaunchConfiguration('imshow')},
        ],
    )

    # 5b. apriltag_goal_bridge
    bridge_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_goal_bridge.py',
        name='apriltag_goal_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'reach_max_distance': 0.55,
            'stale_threshold_s': 1.0,
            'smoothing_window': 5,
            'trigger_key': 'g',
        }],
    )

    # 5c. planner.launch.py include (with LaunchConfiguration passthrough)
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'planner.launch.py')
        ),
        launch_arguments={
            'planning_timeout': LaunchConfiguration('planning_timeout'),
            'adaptive_orientation_enabled': LaunchConfiguration('adaptive_orientation_enabled'),
        }.items(),
    )

    # 5d. control.launch.py include
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'control.launch.py')
        )
    )

    # ---------- TimerAction: delayed start for detector/bridge/planner/control ----------
    delayed_actions = TimerAction(period=3.0, actions=[
        apriltag_node,
        bridge_node,
        planner_launch,
        control_launch,
    ])

    return LaunchDescription([
        imshow_arg,
        adaptive_arg,
        planning_timeout_arg,
        robot_launch,
        realsense_launch,
        d435_to_camera_link,
        delayed_actions,
    ])
