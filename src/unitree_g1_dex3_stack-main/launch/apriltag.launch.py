"""Phase 7 standalone test launch for the AprilTag detector.

Composes:
- robot.launch.py (robot_state_publisher with the locked-waist URDF
  + CycloneDDS env vars)
- realsense2_camera/launch/rs_launch.py (RGB-only, 640x480x15,
  align_depth disabled per D-15)
- a static d435_link → camera_link transform (so RealSense's
  camera_*_optical_frame chain attaches under the URDF's d435_link)
- apriltag_detector_node.py loaded with config/apriltag.yaml,
  with `imshow` overridable via launch arg.

Single-purpose detection launch — intentionally omits the visualizer,
the manipulation pipeline, the executor, and the trigger node (Phase 9
composes the end-to-end launch separately).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    realsense_share = get_package_share_directory('realsense2_camera')

    # ---------- launch arguments ----------
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf',
        description='URDF filename used by robot_state_publisher',
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Optional full URDF path that overrides urdf_name',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share, 'config', 'apriltag.yaml'),
        description='AprilTag detector parameter YAML file',
    )
    imshow_arg = DeclareLaunchArgument(
        'imshow',
        default_value='true',
        description='Open OpenCV detection window (set false for headless / SSH)',
    )

    # ---------- robot (robot_state_publisher + CycloneDDS env) ----------
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'urdf_name': LaunchConfiguration('urdf_name'),
            'urdf_path': LaunchConfiguration('urdf_path'),
        }.items(),
    )

    # ---------- RealSense D435i (RGB only, 640x480x15) ----------
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
            'accelerate_gpu_with_glsl': 'true',
            'initial_reset': 'true',
        }.items(),
    )

    # ---------- d435_link → camera_link static TF ----------
    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
    )

    # ---------- AprilTag detector node ----------
    # parameters list: YAML loads first; the dict overlay forces `imshow`
    # to follow the launch argument regardless of YAML.
    apriltag_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_detector_node.py',
        name='apriltag_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'imshow': LaunchConfiguration('imshow')},
        ],
    )

    return LaunchDescription([
        urdf_name_arg,
        urdf_path_arg,
        config_file_arg,
        imshow_arg,
        robot_launch,
        realsense_launch,
        d435_to_camera_link,
        apriltag_node,
    ])
