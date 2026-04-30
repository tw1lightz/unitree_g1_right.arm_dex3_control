---
status: complete
quick_id: 260430-fxj
slug: tcp-torso-link
date: 2026-04-30
---

## Summary

Created `scripts/tcp_torso_pose.py` — a ROS2 Python node that computes the right-arm TCP pose in `torso_link` frame using dynamic FK (KDL `ChainFkSolverPos_recursive`).

### What was built

- **`scripts/tcp_torso_pose.py`** (176 lines): ROS2 Node that:
  - Subscribes to `/joint_states` for live joint angles
  - Loads URDF (from `/robot_description` param, file path, or default URDF)
  - Builds KDL chain `torso_link` → `right_wrist_yaw_link`
  - Runs FK with current joint positions
  - Applies configurable +X TCP offset (param `tcp_offset_x`, default 0.145 m)
  - Outputs `xyz=[X.XXXX, Y.XXXX, Z.XXXX] m, rpy=[R.XXXX, P.XXXX, Y.XXXX] rad` at configurable rate (default 10 Hz)

### Tasks completed
1. Create `tcp_torso_pose.py` node — `4573d54`
2. Register in `CMakeLists.txt` install block — `bfaabc8`
