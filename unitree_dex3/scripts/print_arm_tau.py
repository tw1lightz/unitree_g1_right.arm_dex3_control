#!/usr/bin/env python3
"""Print right-arm tau_est (motor torque feedback) at 2 Hz for verification."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_hg.msg import LowState

# Right arm joint indices (must match g1_dex3_joint_defs.hpp)
RIGHT_ARM_JOINTS = {
    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
    "right_wrist_pitch": 27,
    "right_wrist_yaw": 28,
}


class ArmTauPrinter(Node):
    def __init__(self):
        super().__init__("arm_tau_printer")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(
            LowState, "/lf/lowstate", self.callback, qos
        )
        self.timer_ = self.create_timer(0.5, self.print_tau)  # 2 Hz
        self.latest_msg_ = None

    def callback(self, msg):
        self.latest_msg_ = msg

    def print_tau(self):
        if self.latest_msg_ is None:
            self.get_logger().info("Waiting for /lf/lowstate...")
            return

        parts = []
        for name, idx in RIGHT_ARM_JOINTS.items():
            ms = self.latest_msg_.motor_state[idx]
            parts.append(f"{name}: q={ms.q:.4f} dq={ms.dq:.4f} tau={ms.tau_est:.3f}")

        self.get_logger().info("--- Right Arm Torque Feedback ---")
        for p in parts:
            self.get_logger().info(f"  {p}")


def main():
    rclpy.init()
    node = ArmTauPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
