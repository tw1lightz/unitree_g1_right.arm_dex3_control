import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
    tf_remappings = [
        ('/tf', LaunchConfiguration('tf_topic')),
        ('/tf_static', LaunchConfiguration('tf_static_topic')),
    ]

    # ---------- launch arguments ----------
    imshow_arg = DeclareLaunchArgument(
        'imshow',
        default_value='true',
        description='Open local OpenCV window (use MJPEG stream instead)',
    )

    adaptive_arg = DeclareLaunchArgument(
        'adaptive_orientation_enabled',
        default_value='',
        description='Pass through to planner.launch.py',
    )

    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='1.0',
        description='Planning timeout in seconds',
    )

    camera_only_arg = DeclareLaunchArgument(
        'camera_only',
        default_value='false',
        description='Only launch robot description, RealSense camera, and camera TF',
    )

    tf_topic_arg = DeclareLaunchArgument(
        'tf_topic',
        default_value='/unitree_g1_dex3/tf',
        description='TF topic used by this launch',
    )

    tf_static_topic_arg = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/unitree_g1_dex3/tf_static',
        description='TF static topic used by this launch',
    )

    # ---------- immediate-start components ----------

    # 4a. robot.launch.py include (CycloneDDS env owner, do NOT duplicate)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py')
        ),
        launch_arguments={
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
        }.items(),
    )

    # 4b. RealSense node (RGB only, 640x480x6, no depth)
    realsense_launch = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'serial_no': '_243722074823',
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'enable_sync': False,
            'align_depth.enable': False,
            'rgb_camera.color_profile': '640x480x6',
            'rgb_camera.color_format': 'RGB8',
            'accelerate_gpu_with_glsl': False,
            'initial_reset': False,
            'publish_tf': True,
            'tf_publish_rate': 0.0,
            'rgb_camera.power_line_frequency': 1,
        }],
        remappings=tf_remappings,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # 4c. d435_link -> camera_link static TF
    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
        remappings=tf_remappings,
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
        remappings=tf_remappings,
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
            'fixed_orientation_enabled':False,
            'fixed_rpy': [-0.0873, -0.0340, 0.0199],
        }],
        remappings=tf_remappings,
    )

    # 5c. planner.launch.py include (with LaunchConfiguration passthrough)
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'planner.launch.py')
        ),
        launch_arguments={
            'planning_timeout': LaunchConfiguration('planning_timeout'),
            'adaptive_orientation_enabled': LaunchConfiguration('adaptive_orientation_enabled'),
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
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
    ], condition=UnlessCondition(LaunchConfiguration('camera_only')))

    return LaunchDescription([
        imshow_arg,
        adaptive_arg,
        planning_timeout_arg,
        camera_only_arg,
        tf_topic_arg,
        tf_static_topic_arg,
        robot_launch,
        realsense_launch,
        d435_to_camera_link,
        delayed_actions,
    ])
