#!/usr/bin/env python3
"""Compare KDL gravity torques with measured tau_est and fit per-joint corrections.

Reads calibration_data.csv, computes KDL gravity torques for each pose,
and fits: tau_corrected = scale * tau_kdl + bias  per joint.

Output: scale and bias arrays to hardcode in joint_trajectory_executor.cpp
"""
import os
import sys
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'data')
CSV_PATH = os.path.join(DATA_DIR, 'calibration_data.csv')

URDF_PATH = os.path.join(
    os.path.dirname(SCRIPT_DIR),
    'src', 'unitree_g1_dex3_stack-main', 'robots', 'g1_description',
    'g1_29dof_lock_waist_with_hand_rev_1_0.urdf')

JOINT_NAMES = [
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
    'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw',
]


# ── KDL tree builder (same as tcp_torso_pose.py) ──────────────────────

def _kdl_pose(origin):
    if origin is None:
        return PyKDL.Frame()
    rpy = origin.rpy if origin.rpy else [0, 0, 0]
    xyz = origin.xyz if origin.xyz else [0, 0, 0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        PyKDL.Vector(xyz[0], xyz[1], xyz[2]))


def _kdl_inertia(i):
    origin = _kdl_pose(i.origin)
    inertia = i.inertia
    return origin.M * PyKDL.RigidBodyInertia(
        i.mass, origin.p,
        PyKDL.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz,
                                inertia.ixy, inertia.ixz, inertia.iyz))


def _kdl_joint(jnt):
    F = _kdl_pose(jnt.origin)
    if jnt.type in ('revolute', 'continuous'):
        return PyKDL.Joint(jnt.name, F.p, F.M * PyKDL.Vector(*jnt.axis),
                           PyKDL.Joint.RotAxis)
    if jnt.type == 'prismatic':
        return PyKDL.Joint(jnt.name, F.p, F.M * PyKDL.Vector(*jnt.axis),
                           PyKDL.Joint.TransAxis)
    return PyKDL.Joint(jnt.name, PyKDL.Joint.Fixed)


def _add_children_to_tree(robot_model, root, tree):
    inert = PyKDL.RigidBodyInertia(0)
    if root.inertial:
        inert = _kdl_inertia(root.inertial)
    parent_joint_name, parent_link_name = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]
    sgm = PyKDL.Segment(
        root.name,
        _kdl_joint(parent_joint),
        _kdl_pose(parent_joint.origin),
        inert)
    if not tree.addSegment(sgm, parent_link_name):
        return False
    if root.name not in robot_model.child_map:
        return True
    for _jn, child_link_name in robot_model.child_map[root.name]:
        child = robot_model.link_map[child_link_name]
        if not _add_children_to_tree(robot_model, child, tree):
            return False
    return True


def build_kdl_chain(urdf_path):
    """Build KDL chain torso_link -> right_wrist_yaw_link (same as C++ executor)."""
    with open(urdf_path, 'rb') as f:
        robot = URDF.from_xml_string(f.read())
    root = robot.link_map[robot.get_root()]
    tree = PyKDL.Tree(root.name)
    for _jn, child_link_name in robot.child_map[root.name]:
        child = robot.link_map[child_link_name]
        _add_children_to_tree(robot, child, tree)
    chain = tree.getChain('torso_link', 'right_wrist_yaw_link')
    return chain


def compute_kdl_gravity(chain, q_array):
    """Compute KDL gravity torques for a 7-element joint array."""
    nj = chain.getNrOfJoints()
    solver = PyKDL.ChainDynParam(chain, PyKDL.Vector(0, 0, -9.81))
    q = PyKDL.JntArray(nj)
    for i in range(min(nj, 7)):
        q[i] = q_array[i]
    tau = PyKDL.JntArray(nj)
    solver.JntToGravity(q, tau)
    return np.array([tau[i] for i in range(min(nj, 7))])


def main():
    print("=" * 60)
    print("KDL Gravity Torque Calibration")
    print("=" * 60)

    # Load data
    data = np.genfromtxt(CSV_PATH, delimiter=',', skip_header=1)
    n_samples = data.shape[0]
    q_all = data[:, :7]       # joint positions
    tau_all = data[:, 7:14]   # measured tau_est
    print(f"\nLoaded {n_samples} samples from {CSV_PATH}")

    # Build KDL chain
    print(f"Building KDL chain from {URDF_PATH}")
    chain = build_kdl_chain(URDF_PATH)
    nj = chain.getNrOfJoints()
    print(f"  Chain: {chain.getNrOfSegments()} segments, {nj} joints")

    # Compute KDL gravity torques for all samples
    tau_kdl = np.zeros((n_samples, 7))
    for i in range(n_samples):
        tau_kdl[i] = compute_kdl_gravity(chain, q_all[i])

    # Compare
    print(f"\n{'Joint':<18} {'KDL mean':>10} {'Meas mean':>10} {'KDL std':>10} {'Meas std':>10}")
    print("-" * 60)
    for j in range(7):
        print(f"  {JOINT_NAMES[j]:<16} {tau_kdl[:,j].mean():>10.3f} {tau_all[:,j].mean():>10.3f} "
              f"{tau_kdl[:,j].std():>10.3f} {tau_all[:,j].std():>10.3f}")

    # Fit per-joint linear correction: tau_measured = scale * tau_kdl + bias
    print(f"\n{'Joint':<18} {'Scale':>8} {'Bias':>8} {'RMSE_before':>12} {'RMSE_after':>12}")
    print("-" * 60)
    scales = np.zeros(7)
    biases = np.zeros(7)
    for j in range(7):
        x = tau_kdl[:, j]
        y = tau_all[:, j]

        rmse_before = np.sqrt(np.mean((y - x) ** 2))

        # Least squares: y = scale * x + bias
        A = np.column_stack([x, np.ones(n_samples)])
        result, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        scales[j] = result[0]
        biases[j] = result[1]

        y_pred = scales[j] * x + biases[j]
        rmse_after = np.sqrt(np.mean((y - y_pred) ** 2))

        print(f"  {JOINT_NAMES[j]:<16} {scales[j]:>8.3f} {biases[j]:>8.3f} "
              f"{rmse_before:>12.3f} {rmse_after:>12.3f}")

    # Generate C++ code
    print("\n" + "=" * 60)
    print("C++ code for joint_trajectory_executor.cpp:")
    print("=" * 60)
    scale_str = ', '.join(f'{s:.4f}f' for s in scales)
    bias_str = ', '.join(f'{b:.4f}f' for b in biases)
    print(f"""
  // KDL gravity torque correction (from calibrate_kdl_tau.py)
  const std::array<float, 7> gravity_scale_ = {{{scale_str}}};
  const std::array<float, 7> gravity_bias_  = {{{bias_str}}};

  // In computeGravityTorques, replace:
  //   torques[i] = static_cast<float>(gravity_torques(i));
  // with:
  //   torques[i] = gravity_scale_[i] * static_cast<float>(gravity_torques(i)) + gravity_bias_[i];
""")

    # Save result
    result_path = os.path.join(DATA_DIR, 'kdl_tau_calibration.txt')
    with open(result_path, 'w') as f:
        f.write(f"scales: [{', '.join(f'{s:.4f}' for s in scales)}]\n")
        f.write(f"biases: [{', '.join(f'{b:.4f}' for b in biases)}]\n")
    print(f"Results saved to {result_path}")


if __name__ == '__main__':
    main()
