#!/usr/bin/env python3
"""
Gravity Calibration Solver
============================
Reads calibration_data.csv, builds a gravity torque regressor using PyKDL,
and solves for optimal inertial parameters (mass * CoM) via least squares.

Usage:
  python3 scripts/gravity_calibration_solve.py

Input:  data/calibration_data.csv
Output: data/calibration_result.yaml (calibrated parameters)
        data/calibration_report.txt  (error report)
"""

import os
import csv
import yaml
import numpy as np
from pathlib import Path

import PyKDL
from urdf_parser_py.urdf import URDF


# Right arm chain: torso_link -> right_wrist_yaw_link
RIGHT_ARM_LINKS = [
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
]

GRAVITY = np.array([0.0, 0.0, -9.81])


def load_urdf_model(urdf_path):
    """Load URDF and extract right-arm inertial parameters."""
    with open(urdf_path, 'rb') as f:
        robot = URDF.from_xml_string(f.read())
    return robot


def build_kdl_chain_from_urdf(urdf_path, base_link="torso_link", tip_link="right_wrist_yaw_link"):
    """Build KDL chain from URDF by traversing the kinematic tree manually."""
    import xml.etree.ElementTree as ET
    tree_xml = ET.parse(urdf_path)
    root = tree_xml.getroot()

    # Parse joints and links
    joints = {}
    for j in root.findall('joint'):
        name = j.get('name')
        jtype = j.get('type')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        origin = j.find('origin')
        if origin is not None:
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
        else:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        axis_elem = j.find('axis')
        axis = [float(x) for x in axis_elem.get('xyz', '0 0 1').split()] if axis_elem is not None else [0, 0, 1]
        joints[name] = {
            'type': jtype, 'parent': parent, 'child': child,
            'xyz': xyz, 'rpy': rpy, 'axis': axis
        }

    # Build parent->child map
    parent_to_joints = {}
    for jname, jdata in joints.items():
        parent_to_joints.setdefault(jdata['parent'], []).append((jname, jdata))

    # Walk from base_link to tip_link
    chain = PyKDL.Chain()
    current_link = base_link
    visited = set()

    while current_link != tip_link:
        if current_link in visited:
            raise RuntimeError(f"Loop detected at {current_link}")
        visited.add(current_link)

        child_joints = parent_to_joints.get(current_link, [])
        # Find the joint leading toward tip_link (BFS)
        found = False
        for jname, jdata in child_joints:
            if _link_leads_to(jdata['child'], tip_link, parent_to_joints):
                xyz = jdata['xyz']
                rpy = jdata['rpy']
                axis = jdata['axis']

                frame = PyKDL.Frame(
                    PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
                    PyKDL.Vector(xyz[0], xyz[1], xyz[2])
                )

                if jdata['type'] == 'revolute' or jdata['type'] == 'continuous':
                    joint = PyKDL.Joint(jname, PyKDL.Vector(0, 0, 0),
                                        PyKDL.Vector(axis[0], axis[1], axis[2]),
                                        PyKDL.Joint.RotAxis)
                elif jdata['type'] == 'fixed':
                    joint = PyKDL.Joint(jname, PyKDL.Joint.Fixed)
                else:
                    joint = PyKDL.Joint(jname, PyKDL.Joint.Fixed)

                segment = PyKDL.Segment(jdata['child'], joint, frame)
                chain.addSegment(segment)
                current_link = jdata['child']
                found = True
                break

        if not found:
            raise RuntimeError(f"No path from {current_link} to {tip_link}")

    return chain


def _link_leads_to(start, target, parent_to_joints):
    """BFS check if start can reach target."""
    if start == target:
        return True
    queue = [start]
    visited = set()
    while queue:
        link = queue.pop(0)
        if link in visited:
            continue
        visited.add(link)
        for jname, jdata in parent_to_joints.get(link, []):
            if jdata['child'] == target:
                return True
            queue.append(jdata['child'])
    return False


def compute_link_frames(chain, q_array):
    """Compute the frame (position + rotation) of each link origin given joint angles.

    Returns list of (position_3, rotation_3x3) for each segment.
    """
    n_joints = chain.getNrOfJoints()
    n_segments = chain.getNrOfSegments()
    assert len(q_array) == n_joints

    q_kdl = PyKDL.JntArray(n_joints)
    for i in range(n_joints):
        q_kdl[i] = float(q_array[i])

    # Use FK solver to compute each frame
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

    frames = []
    joint_axes = []
    joint_origins = []

    # Frame 0 is base (identity)
    current_frame = PyKDL.Frame.Identity()

    joint_idx = 0
    for seg_idx in range(n_segments):
        segment = chain.getSegment(seg_idx)
        joint = segment.getJoint()

        if joint.getType() != PyKDL.Joint.JointType.Fixed:
            # Get frame up to this segment
            frame_out = PyKDL.Frame()
            fk_solver.JntToCart(q_kdl, frame_out, seg_idx + 1)

            # Joint axis in base frame
            # The joint axis is defined in the segment's joint frame
            # After FK, we can get it from the frame rotation
            if seg_idx > 0:
                parent_frame = PyKDL.Frame()
                fk_solver.JntToCart(q_kdl, parent_frame, seg_idx)
            else:
                parent_frame = PyKDL.Frame.Identity()

            # Joint origin = position of parent frame's origin (where joint is)
            joint_origin = np.array([
                frame_out.p.x() - (frame_out.M * segment.getFrameToTip().Inverse().p).x(),
                frame_out.p.y() - (frame_out.M * segment.getFrameToTip().Inverse().p).y(),
                frame_out.p.z() - (frame_out.M * segment.getFrameToTip().Inverse().p).z(),
            ])

            frames.append(frame_out)
            joint_idx += 1
        else:
            # Fixed joint — still compute frame
            frame_out = PyKDL.Frame()
            fk_solver.JntToCart(q_kdl, frame_out, seg_idx + 1)
            frames.append(frame_out)

    return frames


def compute_gravity_regressor(chain, q_array):
    """
    Compute the gravity torque regressor Y(q) such that:
        tau_gravity = Y(q) @ pi
    where pi = [m1, m1*cx1, m1*cy1, m1*cz1, m2, ..., m7*cz7] (28 params)

    For revolute joint i with axis z_i at position p_i:
        tau_i = sum_{j>=i} (z_i x (p_cj - p_i)) . (m_j * g)

    where p_cj = p_j_frame_origin + R_j * [cx_j, cy_j, cz_j]^T

    This is linear in [m_j, m_j*cx_j, m_j*cy_j, m_j*cz_j].
    """
    n_joints = chain.getNrOfJoints()
    n_segments = chain.getNrOfSegments()
    n_params = 4 * n_joints  # 4 params per actuated link

    q_kdl = PyKDL.JntArray(n_joints)
    for i in range(n_joints):
        q_kdl[i] = float(q_array[i])

    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

    # Compute frames for each segment
    # We need: joint axis in base frame, joint position, link frame for each actuated link
    actuated_segments = []  # (segment_index, joint_index)
    ji = 0
    for si in range(n_segments):
        seg = chain.getSegment(si)
        if seg.getJoint().getType() != PyKDL.Joint.JointType.Fixed:
            actuated_segments.append((si, ji))
            ji += 1

    assert len(actuated_segments) == n_joints

    # Get frame of each segment tip in base frame
    seg_frames = []
    for si in range(n_segments):
        f = PyKDL.Frame()
        fk_solver.JntToCart(q_kdl, f, si + 1)
        seg_frames.append(f)

    # For each actuated joint, get its axis and position in base frame
    joint_axes_base = []
    joint_positions_base = []

    for (si, ji) in actuated_segments:
        seg = chain.getSegment(si)
        joint = seg.getJoint()

        # Joint position in base frame: use the parent frame's tip
        if si == 0:
            parent_frame = PyKDL.Frame.Identity()
        else:
            parent_frame = seg_frames[si - 1]

        # Joint axis in base frame
        # KDL joint axis is in the joint's local frame (typically defined in segment)
        # For a revolute joint, the axis in base = parent_rotation * joint_axis_local
        joint_axis_local = joint.JointAxis()
        joint_axis_base = parent_frame.M * joint_axis_local

        joint_axes_base.append(np.array([
            joint_axis_base.x(), joint_axis_base.y(), joint_axis_base.z()
        ]))

        # Joint position = parent frame origin + parent_rotation * joint_origin
        joint_origin_in_parent = seg.getJoint().JointOrigin()
        jp = parent_frame * joint_origin_in_parent
        joint_positions_base.append(np.array([
            parent_frame.p.x(), parent_frame.p.y(), parent_frame.p.z()
        ]))

    # Build regressor matrix Y (n_joints x n_params)
    Y = np.zeros((n_joints, n_params))

    for i, (si_i, ji_i) in enumerate(actuated_segments):
        z_i = joint_axes_base[i]
        p_i = joint_positions_base[i]

        # Sum over all links j >= i (links downstream of joint i)
        for j, (si_j, ji_j) in enumerate(actuated_segments):
            if j < i:
                continue

            # Frame of link j (segment j's tip frame)
            fj = seg_frames[si_j]
            p_j_origin = np.array([fj.p.x(), fj.p.y(), fj.p.z()])
            R_j = np.array([
                [fj.M[0, 0], fj.M[0, 1], fj.M[0, 2]],
                [fj.M[1, 0], fj.M[1, 1], fj.M[1, 2]],
                [fj.M[2, 0], fj.M[2, 1], fj.M[2, 2]],
            ])

            # d = p_j_origin - p_i
            d = p_j_origin - p_i

            # Parameter column offset for link j
            col_base = j * 4

            # tau_i contribution from m_j:
            # (z_i x d) . g * m_j
            cross_z_d = np.cross(z_i, d)
            Y[i, col_base + 0] = np.dot(cross_z_d, GRAVITY)

            # tau_i contribution from m_j * cx_j:
            # (z_i x (R_j * [1,0,0])) . g * (m_j*cx_j)
            r_col_x = R_j[:, 0]
            cross_z_rx = np.cross(z_i, r_col_x)
            Y[i, col_base + 1] = np.dot(cross_z_rx, GRAVITY)

            # tau_i contribution from m_j * cy_j:
            r_col_y = R_j[:, 1]
            cross_z_ry = np.cross(z_i, r_col_y)
            Y[i, col_base + 2] = np.dot(cross_z_ry, GRAVITY)

            # tau_i contribution from m_j * cz_j:
            r_col_z = R_j[:, 2]
            cross_z_rz = np.cross(z_i, r_col_z)
            Y[i, col_base + 3] = np.dot(cross_z_rz, GRAVITY)

    return Y


def load_calibration_data(csv_path):
    """Load q and tau data from CSV."""
    q_data = []
    tau_data = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            values = [float(v) for v in row]
            q_data.append(values[:7])
            tau_data.append(values[7:14])
    return np.array(q_data), np.array(tau_data)


def extract_urdf_inertials(robot, link_names):
    """Extract current inertial parameters from URDF model."""
    params = []
    for name in link_names:
        link = robot.link_map[name]
        if link.inertial is None:
            params.extend([0.0, 0.0, 0.0, 0.0])
        else:
            m = link.inertial.mass
            cx, cy, cz = link.inertial.origin.xyz
            params.extend([m, m * cx, m * cy, m * cz])
    return np.array(params)


def params_to_mass_com(params):
    """Convert parameter vector [m, mcx, mcy, mcz, ...] to [(mass, [cx, cy, cz]), ...]"""
    result = []
    for i in range(0, len(params), 4):
        m = params[i]
        if abs(m) < 1e-6:
            result.append((0.0, [0.0, 0.0, 0.0]))
        else:
            cx = params[i + 1] / m
            cy = params[i + 2] / m
            cz = params[i + 3] / m
            result.append((m, [cx, cy, cz]))
    return result


def main():
    script_dir = Path(__file__).parent
    project_dir = script_dir.parent
    data_dir = project_dir / "data"
    urdf_path = str(project_dir / "src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf")
    csv_path = str(data_dir / "calibration_data.csv")

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found. Run gravity_calibration_collect.py first.")
        return

    print("=" * 60)
    print("  Gravity Calibration Solver")
    print("=" * 60)

    # 1. Load data
    print(f"\n[1] Loading calibration data from {csv_path}...")
    q_data, tau_data = load_calibration_data(csv_path)
    n_poses = q_data.shape[0]
    print(f"    Loaded {n_poses} poses")

    # 2. Load URDF
    print(f"\n[2] Loading URDF from {urdf_path}...")
    robot = load_urdf_model(urdf_path)
    chain = build_kdl_chain_from_urdf(urdf_path)
    n_joints = chain.getNrOfJoints()
    print(f"    Chain: {chain.getNrOfSegments()} segments, {n_joints} joints")

    # 3. Extract current URDF parameters
    print(f"\n[3] Extracting current URDF inertial parameters...")
    pi_urdf = extract_urdf_inertials(robot, RIGHT_ARM_LINKS)
    print(f"    Current params (28): {pi_urdf}")

    # 4. Build stacked regressor
    print(f"\n[4] Building gravity regressor for {n_poses} poses...")
    Y_stack = []
    tau_stack = []
    for i in range(n_poses):
        Y_i = compute_gravity_regressor(chain, q_data[i])
        Y_stack.append(Y_i)
        tau_stack.append(tau_data[i])

    Y_all = np.vstack(Y_stack)      # (7*N) x 28
    tau_all = np.concatenate(tau_stack)  # (7*N,)
    print(f"    Regressor shape: {Y_all.shape}")
    print(f"    Tau vector shape: {tau_all.shape}")

    # 5. Check model prediction with current URDF params
    tau_model_urdf = Y_all @ pi_urdf
    err_urdf = tau_all - tau_model_urdf
    rmse_urdf = np.sqrt(np.mean(err_urdf**2))
    print(f"\n[5] Current URDF model RMSE: {rmse_urdf:.4f} Nm")
    print(f"    Max error: {np.max(np.abs(err_urdf)):.4f} Nm")

    # 6. Least squares solve with physically-constrained regularization
    print(f"\n[6] Solving constrained least squares...")
    from scipy.optimize import lsq_linear
    n_params = Y_all.shape[1]
    n_joints = 7

    # Add per-joint torque bias columns (7 extra params) to handle friction/offsets
    # Each joint gets a constant offset: tau_measured = Y @ pi + bias_j
    n_total = n_params + n_joints  # 28 inertial + 7 bias
    B_bias = np.zeros((Y_all.shape[0], n_joints))
    for j in range(n_joints):
        B_bias[j::n_joints, j] = 1.0  # bias for joint j

    Y_aug = np.hstack([Y_all, B_bias])

    # Tikhonov regularization toward URDF values (inertial) and 0 (bias)
    lambda_weights = np.ones(n_total)
    for link_idx in range(7):
        base = link_idx * 4
        if link_idx >= 4:  # wrist links: stronger regularization
            lambda_weights[base:base+4] = 2.0
        else:  # shoulder/elbow links
            lambda_weights[base:base+4] = 0.3
    # Bias terms: light regularization toward 0
    lambda_weights[n_params:] = 0.1

    W_reg = np.diag(lambda_weights)
    pi_prior = np.concatenate([pi_urdf, np.zeros(n_joints)])  # prior: URDF + zero bias
    A = np.vstack([Y_aug, W_reg])
    b = np.concatenate([tau_all, W_reg @ pi_prior])

    # Bounds
    lb = np.full(n_total, -np.inf)
    ub = np.full(n_total, np.inf)
    for link_idx in range(7):
        base = link_idx * 4
        m_urdf = pi_urdf[base]
        lb[base] = max(0.01, m_urdf * 0.3)   # mass >= 30% of URDF
        ub[base] = m_urdf * 3.0                # mass <= 300% of URDF
        for k in range(1, 4):
            mcx_urdf = pi_urdf[base + k]
            delta = m_urdf * 0.10  # allow ±10cm CoM shift
            lb[base + k] = mcx_urdf - delta
            ub[base + k] = mcx_urdf + delta
    # Bias bounds: ±3 Nm
    lb[n_params:] = -3.0
    ub[n_params:] = 3.0

    result = lsq_linear(A, b, bounds=(lb, ub))
    pi_opt = result.x[:n_params]  # extract inertial params only
    bias_opt = result.x[n_params:]

    print(f"    Torque biases: [{', '.join(f'{v:.3f}' for v in bias_opt)}] Nm")

    # 7. Evaluate calibrated model (inertial + bias)
    tau_model_cal = Y_all[:len(tau_all)] @ pi_opt + B_bias @ bias_opt
    err_cal = tau_all - tau_model_cal
    rmse_cal = np.sqrt(np.mean(err_cal**2))
    print(f"\n[7] Calibrated model RMSE: {rmse_cal:.4f} Nm")
    print(f"    Max error: {np.max(np.abs(err_cal)):.4f} Nm")
    print(f"    Improvement: {(1 - rmse_cal/rmse_urdf)*100:.1f}%")

    # 8. Convert to mass + CoM
    print(f"\n[8] Calibrated parameters:")
    calibrated = params_to_mass_com(pi_opt)
    original = params_to_mass_com(pi_urdf)

    for i, link_name in enumerate(RIGHT_ARM_LINKS):
        m_orig, com_orig = original[i]
        m_cal, com_cal = calibrated[i]
        print(f"    {link_name}:")
        print(f"      URDF:       mass={m_orig:.4f} CoM=[{com_orig[0]:.6f}, {com_orig[1]:.6f}, {com_orig[2]:.6f}]")
        print(f"      Calibrated: mass={m_cal:.4f} CoM=[{com_cal[0]:.6f}, {com_cal[1]:.6f}, {com_cal[2]:.6f}]")
        print(f"      Δmass={m_cal-m_orig:+.4f} kg")

    # 9. Save results
    result_yaml = {
        "calibration_result": {
            "rmse_before": float(rmse_urdf),
            "rmse_after": float(rmse_cal),
            "improvement_pct": float((1 - rmse_cal / rmse_urdf) * 100),
            "links": {}
        }
    }
    for i, link_name in enumerate(RIGHT_ARM_LINKS):
        m_cal, com_cal = calibrated[i]
        result_yaml["calibration_result"]["links"][link_name] = {
            "mass": float(m_cal),
            "com_xyz": [float(c) for c in com_cal],
        }

    result_path = str(data_dir / "calibration_result.yaml")
    with open(result_path, "w") as f:
        yaml.dump(result_yaml, f, default_flow_style=False)
    print(f"\n[9] Results saved to {result_path}")

    # 10. Save report
    report_path = str(data_dir / "calibration_report.txt")
    with open(report_path, "w") as f:
        f.write("Gravity Calibration Report\n")
        f.write("=" * 40 + "\n\n")
        f.write(f"Poses used: {n_poses}\n")
        f.write(f"RMSE before: {rmse_urdf:.4f} Nm\n")
        f.write(f"RMSE after:  {rmse_cal:.4f} Nm\n")
        f.write(f"Improvement: {(1 - rmse_cal/rmse_urdf)*100:.1f}%\n\n")

        f.write("Per-joint error breakdown (calibrated model):\n")
        for j in range(7):
            joint_errs = err_cal[j::7]
            f.write(f"  Joint {j} ({RIGHT_ARM_LINKS[j]}): "
                    f"RMSE={np.sqrt(np.mean(joint_errs**2)):.4f} Nm, "
                    f"max={np.max(np.abs(joint_errs)):.4f} Nm\n")

    print(f"    Report saved to {report_path}")
    print(f"\n{'=' * 60}")
    print(f"  Next step: python3 scripts/update_urdf_inertials.py")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
