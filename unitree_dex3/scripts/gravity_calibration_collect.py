#!/usr/bin/env python3
"""
Gravity Calibration Data Collector
===================================
Commands the right arm to a set of static poses, waits for settling,
and records joint angles (q) + motor torque feedback (tau_est).

Uses unitree_sdk2py direct DDS (same as free_arm_demo.py).
Does NOT need any ROS2 launch — only needs robot standing.

Usage:
  python3 scripts/gravity_calibration_collect.py
  python3 scripts/gravity_calibration_collect.py --sim   # simulation (domain=1)

Output: data/calibration_data.csv
"""

import os
import sys
import csv
import time
import math
import signal
import logging
import argparse
import threading
import numpy as np

# Add xr_teleoperate path for robot_arm imports
XR_DIR = os.path.expanduser("~/Desktop/xr_teleoperate")
if XR_DIR not in sys.path:
    sys.path.insert(0, XR_DIR)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.crc import CRC
from teleop.robot_control.robot_arm import (
    G1_29_JointIndex,
    hg_LowCmd, hg_LowState,
    ChannelPublisher, ChannelSubscriber,
    unitree_hg_msg_dds__LowCmd_,
    G1_29_Num_Motors,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("gravity_cal")

# Right arm joint indices
RIGHT_ARM = [
    ("right_shoulder_pitch", G1_29_JointIndex.kRightShoulderPitch),
    ("right_shoulder_roll",  G1_29_JointIndex.kRightShoulderRoll),
    ("right_shoulder_yaw",   G1_29_JointIndex.kRightShoulderYaw),
    ("right_elbow",          G1_29_JointIndex.kRightElbow),
    ("right_wrist_roll",     G1_29_JointIndex.kRightWristRoll),
    ("right_wrist_pitch",    G1_29_JointIndex.kRightWristPitch),
    ("right_wrist_yaw",      G1_29_JointIndex.kRightWristYaw),
]
RIGHT_ARM_INDICES = [idx for _, idx in RIGHT_ARM]
RIGHT_ARM_NAMES  = [name for name, _ in RIGHT_ARM]

# Master switch
K_NOT_USED_JOINT = G1_29_JointIndex.kNotUsedJoint0

# Standing pose for right arm (read from free_arm_demo.py)
STANDING_RIGHT_ARM = [0.2644, -0.2881, 0.0927, 0.7943, -0.0118, 0.0130, -0.0001]

# Weak motors need lower gains
WEAK_MOTORS = {
    G1_29_JointIndex.kLeftAnklePitch,
    G1_29_JointIndex.kRightAnklePitch,
    G1_29_JointIndex.kLeftShoulderPitch,
    G1_29_JointIndex.kLeftShoulderRoll,
    G1_29_JointIndex.kLeftShoulderYaw,
    G1_29_JointIndex.kLeftElbow,
    G1_29_JointIndex.kRightShoulderPitch,
    G1_29_JointIndex.kRightShoulderRoll,
    G1_29_JointIndex.kRightShoulderYaw,
    G1_29_JointIndex.kRightElbow,
}


def generate_calibration_poses(n_poses=40, seed=42):
    """Generate diverse poses using Latin Hypercube Sampling within safe joint limits."""
    limits = [
        (-1.63, 1.44),   # shoulder pitch
        (-1.48, 0.82),   # shoulder roll
        (-1.57, 1.57),   # shoulder yaw
        (-0.42, 1.47),   # elbow
        (-1.18, 1.18),   # wrist roll
        (-0.97, 0.97),   # wrist pitch
        (-0.97, 0.97),   # wrist yaw
    ]

    rng = np.random.default_rng(seed)
    n_joints = len(limits)
    poses = [[0.0] * n_joints for _ in range(n_poses)]

    for j in range(n_joints):
        lo, hi = limits[j]
        intervals = np.linspace(lo, hi, n_poses + 1)
        samples = rng.uniform(intervals[:-1], intervals[1:])
        rng.shuffle(samples)
        for i in range(n_poses):
            poses[i][j] = float(samples[i])

    return poses


class GravityCalibrationCollector:
    def __init__(self, domain_id=0):
        ChannelFactoryInitialize(domain_id, networkInterface=None)

        self.cmd_pub = ChannelPublisher("rt/arm_sdk", hg_LowCmd)
        self.cmd_pub.Init()
        self.state_sub = ChannelSubscriber("rt/lowstate", hg_LowState)
        self.state_sub.Init()

        self.crc = CRC()
        self.latest_state = None
        self.shutdown_requested = False
        self._lock = threading.Lock()
        self.body_q_locked = None  # fixed body joint positions, set once on first state

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Background thread to read state
        self._sub_thread = threading.Thread(target=self._subscribe_loop, daemon=True)
        self._sub_thread.start()

    def _signal_handler(self, sig, frame):
        log.info("Shutdown requested, ramping to standing...")
        self.shutdown_requested = True

    def _subscribe_loop(self):
        while not self.shutdown_requested:
            msg = self.state_sub.Read()
            if msg is not None:
                with self._lock:
                    self.latest_state = msg
            time.sleep(0.002)

    def wait_for_state(self, timeout=10.0):
        start = time.time()
        while (time.time() - start) < timeout:
            with self._lock:
                if self.latest_state is not None:
                    return True
            time.sleep(0.1)
        return False

    def get_state(self):
        with self._lock:
            return self.latest_state

    def get_right_arm_q(self):
        s = self.get_state()
        if s is None:
            return [0.0] * 7
        return [s.motor_state[idx].q for idx in RIGHT_ARM_INDICES]

    def get_right_arm_tau(self):
        s = self.get_state()
        if s is None:
            return [0.0] * 7
        return [s.motor_state[idx].tau_est for idx in RIGHT_ARM_INDICES]

    def get_right_arm_dq(self):
        s = self.get_state()
        if s is None:
            return [999.0] * 7
        return [s.motor_state[idx].dq for idx in RIGHT_ARM_INDICES]

    def lock_body_pose(self):
        """Capture current body joint positions as the fixed target."""
        state = self.get_state()
        if state is None:
            return
        self.body_q_locked = {}
        for jid in G1_29_JointIndex:
            if jid.value >= G1_29_Num_Motors:
                continue
            self.body_q_locked[jid] = state.motor_state[jid].q
        log.info("Body pose locked")

    def send_cmd(self, right_arm_q, right_arm_kp=None, right_arm_kd=None, authority=1.0):
        """Send command holding body at locked pose + commanding right arm."""
        if right_arm_kp is None:
            right_arm_kp = [80.0] * 4 + [40.0] * 3
        if right_arm_kd is None:
            right_arm_kd = [3.0] * 4 + [1.5] * 3

        state = self.get_state()
        if state is None:
            return

        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = state.mode_machine

        # Set all body joints to FIXED locked positions (not current state)
        for jid in G1_29_JointIndex:
            if jid.value >= G1_29_Num_Motors:
                continue
            msg.motor_cmd[jid].mode = 1
            if self.body_q_locked is not None and jid in self.body_q_locked:
                msg.motor_cmd[jid].q = self.body_q_locked[jid]
            else:
                msg.motor_cmd[jid].q = state.motor_state[jid].q
            msg.motor_cmd[jid].dq = 0
            msg.motor_cmd[jid].tau = 0
            if jid in WEAK_MOTORS:
                msg.motor_cmd[jid].kp = 80.0
                msg.motor_cmd[jid].kd = 3.0
            else:
                msg.motor_cmd[jid].kp = 300.0
                msg.motor_cmd[jid].kd = 3.0

        # Override right arm
        for k, idx in enumerate(RIGHT_ARM_INDICES):
            msg.motor_cmd[idx].mode = 1
            msg.motor_cmd[idx].q = float(right_arm_q[k])
            msg.motor_cmd[idx].dq = 0
            msg.motor_cmd[idx].kp = float(right_arm_kp[k])
            msg.motor_cmd[idx].kd = float(right_arm_kd[k])
            msg.motor_cmd[idx].tau = 0

        # Master switch
        msg.motor_cmd[K_NOT_USED_JOINT].q = float(authority)

        msg.crc = self.crc.Crc(msg)
        self.cmd_pub.Write(msg)

    def move_to_pose(self, target_q, duration=2.0, rate_hz=250):
        """Smoothly interpolate from current to target."""
        start_q = self.get_right_arm_q()
        n_steps = int(duration * rate_hz)
        dt = duration / n_steps

        for step in range(n_steps + 1):
            if self.shutdown_requested:
                return False
            t = step / n_steps
            s = 0.5 * (1.0 - math.cos(math.pi * t))
            q_interp = [
                (1.0 - s) * start_q[k] + s * target_q[k]
                for k in range(7)
            ]
            self.send_cmd(q_interp)
            time.sleep(dt)
        return True

    def wait_settle(self, threshold_dq=0.02, timeout=2.0):
        """Wait until joint velocities are below threshold."""
        start = time.time()
        while (time.time() - start) < timeout:
            if self.shutdown_requested:
                return False
            dq = self.get_right_arm_dq()
            if all(abs(v) < threshold_dq for v in dq):
                return True
            time.sleep(0.01)
        log.warning("Settle timeout — proceeding anyway")
        return True

    def record_samples(self, n_samples=50, interval=0.02):
        """Record n_samples of q and tau_est, return averaged values."""
        q_all = []
        tau_all = []
        for _ in range(n_samples):
            if self.shutdown_requested:
                return None, None
            q_all.append(self.get_right_arm_q())
            tau_all.append(self.get_right_arm_tau())
            # Keep publishing to maintain authority
            self.send_cmd(self.get_right_arm_q())
            time.sleep(interval)

        return np.mean(q_all, axis=0).tolist(), np.mean(tau_all, axis=0).tolist()

    def ramp_to_standing(self, duration=3.0):
        """Smoothly return to standing and release authority."""
        self.move_to_pose(STANDING_RIGHT_ARM, duration=duration)

        # Release authority over 1 second
        n_steps = 250
        for step in range(n_steps + 1):
            if self.shutdown_requested and step > 10:
                break
            t = step / n_steps
            self.send_cmd(STANDING_RIGHT_ARM, authority=1.0 - t)
            time.sleep(0.004)

    def run_collection(self):
        log.info("Waiting for DDS lowstate...")
        if not self.wait_for_state(timeout=10.0):
            log.error("No lowstate received. Is robot standing?")
            return

        log.info("DDS connected!")
        self.lock_body_pose()
        poses = generate_calibration_poses(n_poses=40)
        log.info(f"Generated {len(poses)} calibration poses")

        # Prepare output
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(os.path.dirname(script_dir), "data")
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, "calibration_data.csv")

        header = (
            [f"q_{name}" for name in RIGHT_ARM_NAMES]
            + [f"tau_{name}" for name in RIGHT_ARM_NAMES]
        )

        rows = []

        for i, pose in enumerate(poses):
            if self.shutdown_requested:
                break

            log.info(f"Pose {i+1}/{len(poses)}: {' '.join(f'{v:.3f}' for v in pose)}")

            if not self.move_to_pose(pose, duration=2.0):
                break
            if not self.wait_settle(threshold_dq=0.02, timeout=2.0):
                break

            q_avg, tau_avg = self.record_samples(n_samples=50, interval=0.02)
            if q_avg is None:
                break

            rows.append(q_avg + tau_avg)

            log.info(
                f"  Recorded: q=[{', '.join(f'{v:.4f}' for v in q_avg)}] "
                f"tau=[{', '.join(f'{v:.3f}' for v in tau_avg)}]"
            )

        # Save CSV
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)

        log.info(f"Saved {len(rows)} poses to {csv_path}")

        log.info("Ramping back to standing...")
        self.ramp_to_standing(duration=3.0)
        log.info("Calibration data collection complete.")


def main():
    parser = argparse.ArgumentParser(description="Gravity calibration data collector")
    parser.add_argument("--sim", action="store_true", help="Simulation mode (domain=1)")
    args = parser.parse_args()

    domain_id = 1 if args.sim else 0
    collector = GravityCalibrationCollector(domain_id=domain_id)
    collector.run_collection()


if __name__ == "__main__":
    main()
