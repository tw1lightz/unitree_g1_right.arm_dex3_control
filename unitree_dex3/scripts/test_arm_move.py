#!/usr/bin/env python3
"""Minimal test: move right shoulder pitch by 0.3 rad from current position."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_hg.msg import LowState, LowCmd

K_NOT_USED_JOINT = 8

class TestArmMove(Node):
    def __init__(self):
        super().__init__("test_arm_move")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_ = self.create_publisher(LowCmd, "/arm_sdk", qos)
        self.sub_ = self.create_subscription(LowState, "/lf/lowstate", self.cb, qos)
        self.state_ = None

    def cb(self, msg):
        self.state_ = msg

    def run(self):
        # Wait for state
        while self.state_ is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Got lowstate, starting move...")

        # Move shoulder pitch (idx=22) by +0.3 from current
        target_q22 = self.state_.motor_state[22].q + 0.3
        self.get_logger().info(f"Current q[22]={self.state_.motor_state[22].q:.4f}, target={target_q22:.4f}")

        # Publish for 3 seconds at 250 Hz
        for step in range(750):
            rclpy.spin_once(self, timeout_sec=0.0)
            cmd = LowCmd()

            # Lock all body joints at current positions (skip master switch)
            for i in range(29):
                if i == K_NOT_USED_JOINT:
                    continue
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q = self.state_.motor_state[i].q
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = 60.0
                cmd.motor_cmd[i].kd = 1.5
                cmd.motor_cmd[i].tau = 0.0

            # Override shoulder pitch
            cmd.motor_cmd[22].q = target_q22
            cmd.motor_cmd[22].kp = 100.0
            cmd.motor_cmd[22].kd = 3.0

            # Master switch
            cmd.motor_cmd[K_NOT_USED_JOINT].q = 1.0

            self.pub_.publish(cmd)
            time.sleep(0.004)

            if step % 250 == 0:
                actual = self.state_.motor_state[22].q
                self.get_logger().info(f"  t={step*0.004:.1f}s q[22]={actual:.4f} (target={target_q22:.4f})")

        # Ramp back
        self.get_logger().info("Ramping back...")
        for step in range(250):
            rclpy.spin_once(self, timeout_sec=0.0)
            t = step / 250.0
            cmd = LowCmd()
            for i in range(29):
                if i == K_NOT_USED_JOINT:
                    continue
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q = self.state_.motor_state[i].q
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = 60.0
                cmd.motor_cmd[i].kd = 1.5
                cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[K_NOT_USED_JOINT].q = float(1.0 - t)
            self.pub_.publish(cmd)
            time.sleep(0.004)
        self.get_logger().info("Done")


def main():
    rclpy.init()
    node = TestArmMove()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
