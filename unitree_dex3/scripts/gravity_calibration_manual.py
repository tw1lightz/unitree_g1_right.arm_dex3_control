#!/usr/bin/env python3
"""
Manual Gravity Calibration Data Collector
==========================================
Free-drag the right arm to poses, press Enter to record each one.

Flow per pose:
  1. Right arm is in free-drag mode (kp=0, low kd)
  2. You position the arm manually
  3. Press Enter → arm locks at current position (high kp/kd)
  4. Script waits for settle, records q + tau_est (50 samples averaged)
  5. Arm returns to free-drag mode for next pose

Press 'q' + Enter to finish and save data.

Usage:
  python3 scripts/gravity_calibration_manual.py
  python3 scripts/gravity_calibration_manual.py --sim

Output: data/calibration_data.csv
"""

import os
import sys
import csv
import time
import signal
import logging
import argparse
import threading
import numpy as np

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
log = logging.getLogger("gravity_cal_manual")

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

K_NOT_USED_JOINT = G1_29_JointIndex.kNotUsedJoint0

STANDING_RIGHT_ARM = [0.2644, -0.2881, 0.0927, 0.7943, -0.0118, 0.0130, -0.0001]

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

RIGHT_ARM_SET = set(RIGHT_ARM_INDICES)

# Free-drag gains
FREE_KP = 0.0
FREE_KD = 0.5
FREE_WRIST_KP = 0.0
FREE_WRIST_KD = 0.3

# Position-hold gains for recording
HOLD_KP = [80.0] * 4 + [40.0] * 3
HOLD_KD = [3.0] * 4 + [1.5] * 3


class ManualCalibrationCollector:
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
        self.body_q_locked = None

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._sub_thread = threading.Thread(target=self._subscribe_loop, daemon=True)
        self._sub_thread.start()

    def _signal_handler(self, sig, frame):
        log.info("Shutdown requested...")
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
        state = self.get_state()
        if state is None:
            return
        self.body_q_locked = {}
        for jid in G1_29_JointIndex:
            if jid.value >= G1_29_Num_Motors:
                continue
            self.body_q_locked[jid] = state.motor_state[jid].q
        log.info("Body pose locked")

    def send_cmd(self, right_arm_q, right_arm_kp, right_arm_kd, authority=1.0):
        """Send command with specified right arm gains."""
        state = self.get_state()
        if state is None:
            return

        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = state.mode_machine

        # Body joints: hold locked position
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

        # Right arm with specified gains
        for k, idx in enumerate(RIGHT_ARM_INDICES):
            msg.motor_cmd[idx].mode = 1
            msg.motor_cmd[idx].q = float(right_arm_q[k])
            msg.motor_cmd[idx].dq = 0
            msg.motor_cmd[idx].kp = float(right_arm_kp[k])
            msg.motor_cmd[idx].kd = float(right_arm_kd[k])
            msg.motor_cmd[idx].tau = 0

        msg.motor_cmd[K_NOT_USED_JOINT].q = float(authority)
        msg.crc = self.crc.Crc(msg)
        self.cmd_pub.Write(msg)

    def set_free_mode(self):
        """Continuously send free-drag commands for right arm."""
        state = self.get_state()
        if state is None:
            return
        q_cur = self.get_right_arm_q()
        kp = [FREE_KP] * 4 + [FREE_WRIST_KP] * 3
        kd = [FREE_KD] * 4 + [FREE_WRIST_KD] * 3
        self.send_cmd(q_cur, kp, kd)

    def set_hold_mode(self, q_target):
        """Send position-hold commands for right arm."""
        self.send_cmd(q_target, HOLD_KP, HOLD_KD)

    def record_samples(self, q_hold, n_samples=50, interval=0.02):
        """Hold position and record q + tau_est averaged."""
        q_all = []
        tau_all = []
        for _ in range(n_samples):
            if self.shutdown_requested:
                return None, None
            q_all.append(self.get_right_arm_q())
            tau_all.append(self.get_right_arm_tau())
            self.set_hold_mode(q_hold)
            time.sleep(interval)
        return np.mean(q_all, axis=0).tolist(), np.mean(tau_all, axis=0).tolist()

    def ramp_to_standing(self, duration=3.0):
        """Smoothly return to standing."""
        import math
        start_q = self.get_right_arm_q()
        n_steps = int(duration * 250)
        dt = duration / n_steps
        for step in range(n_steps + 1):
            t = step / n_steps
            s = 0.5 * (1.0 - math.cos(math.pi * t))
            q_interp = [
                (1.0 - s) * start_q[k] + s * STANDING_RIGHT_ARM[k]
                for k in range(7)
            ]
            self.send_cmd(q_interp, HOLD_KP, HOLD_KD)
            time.sleep(dt)

        # Release authority
        for step in range(251):
            t = step / 250.0
            self.send_cmd(STANDING_RIGHT_ARM, HOLD_KP, HOLD_KD, authority=1.0 - t)
            time.sleep(0.004)

    def run(self):
        log.info("Waiting for DDS lowstate...")
        if not self.wait_for_state(timeout=10.0):
            log.error("No lowstate received. Is robot standing?")
            return

        log.info("DDS connected!")
        self.lock_body_pose()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(os.path.dirname(script_dir), "data")
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, "calibration_data.csv")

        header = (
            [f"q_{name}" for name in RIGHT_ARM_NAMES]
            + [f"tau_{name}" for name in RIGHT_ARM_NAMES]
        )
        rows = []

        print("\n" + "=" * 60)
        print("  Manual Gravity Calibration")
        print("=" * 60)
        print("  Right arm is now in FREE-DRAG mode.")
        print("  Drag the arm to a pose, then press Enter to record.")
        print("  Type 'q' + Enter to finish and save.")
        print("  Aim for 30-50 diverse poses covering the workspace.")
        print("=" * 60 + "\n")

        # Start free-drag loop in background
        free_mode_active = threading.Event()
        free_mode_active.set()

        def free_loop():
            while not self.shutdown_requested:
                if free_mode_active.is_set():
                    self.set_free_mode()
                time.sleep(0.004)

        free_thread = threading.Thread(target=free_loop, daemon=True)
        free_thread.start()

        pose_idx = 0
        try:
            while not self.shutdown_requested:
                q_now = self.get_right_arm_q()
                print(f"[Pose {pose_idx+1}] Current q: [{', '.join(f'{v:.3f}' for v in q_now)}]")
                user_input = input("  Press Enter to record, 'q' to finish: ").strip().lower()

                if user_input == 'q':
                    break

                # Stop free mode, switch to hold
                free_mode_active.clear()
                time.sleep(0.01)

                q_hold = self.get_right_arm_q()
                log.info(f"Locking at q=[{', '.join(f'{v:.3f}' for v in q_hold)}]")

                # Hold for settle
                for _ in range(int(1.5 / 0.004)):  # 1.5s settle
                    self.set_hold_mode(q_hold)
                    time.sleep(0.004)

                # Record
                q_avg, tau_avg = self.record_samples(q_hold, n_samples=50, interval=0.02)
                if q_avg is None:
                    break

                rows.append(q_avg + tau_avg)
                pose_idx += 1

                print(f"  ✓ Recorded pose {pose_idx}:")
                print(f"    q   = [{', '.join(f'{v:.4f}' for v in q_avg)}]")
                print(f"    tau = [{', '.join(f'{v:.3f}' for v in tau_avg)}]")
                print()

                # Resume free mode
                free_mode_active.set()

        except (KeyboardInterrupt, EOFError):
            pass

        # Save
        if rows:
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(rows)
            print(f"\nSaved {len(rows)} poses to {csv_path}")
        else:
            print("\nNo poses recorded.")

        # Return to standing
        print("Ramping back to standing...")
        free_mode_active.clear()
        time.sleep(0.05)
        self.ramp_to_standing(duration=3.0)
        print("Done!")


def main():
    parser = argparse.ArgumentParser(description="Manual gravity calibration data collector")
    parser.add_argument("--sim", action="store_true", help="Simulation mode (domain=1)")
    args = parser.parse_args()

    domain_id = 1 if args.sim else 0
    collector = ManualCalibrationCollector(domain_id=domain_id)
    collector.run()


if __name__ == "__main__":
    main()
