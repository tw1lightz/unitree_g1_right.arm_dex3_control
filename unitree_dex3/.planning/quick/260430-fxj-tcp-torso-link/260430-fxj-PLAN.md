---
phase: quick
quick_id: 260430-fxj-tcp-torso-link
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py
  - src/unitree_g1_dex3_stack-main/CMakeLists.txt
autonomous: true
requirements: []

must_haves:
  truths:
    - "运行时，终端输出 TCP 在 torso_link 下的 xyz (m) 和 rpy (rad)，4位小数"
    - "TCP 偏移量通过 ROS2 parameter 配置，默认 0.145m"
    - "节点订阅 /joint_states 获取实时关节角，动态计算 FK"
  artifacts:
    - path: "src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py"
      provides: "ROS2 Python 节点：加载 URDF、构建 KDL chain、订阅 /joint_states、计算 FK、应用 TCP offset、输出 xyz+rpy"
      min_lines: 80
    - path: "src/unitree_g1_dex3_stack-main/CMakeLists.txt"
      provides: "注册脚本的 install(PROGRAMS ...) 条目"
      contains: "tcp_torso_pose.py"
  key_links:
    - from: "tcp_torso_pose.py FK solver"
      to: "KDL chain (torso_link → right_wrist_yaw_link)"
      via: "kdl_parser.treeFromUrdfModel + getChain"
      pattern: "treeFromUrdfModel|getChain"
    - from: "tcp_torso_pose.py joint callback"
      to: "/joint_states topic"
      via: "create_subscription(sensor_msgs.msg.JointState)"
      pattern: "create_subscription.*JointState"
    - from: "tcp_torso_pose.py TCP offset"
      to: "ROS2 parameter tcp_offset_x"
      via: "declare_parameter + get_parameter"
      pattern: "declare_parameter.*tcp_offset"
---

<objective>
Create a ROS2 Python node that computes the right-arm TCP pose in torso_link frame using dynamic FK.
The node subscribes to /joint_states, builds a KDL kinematic chain from torso_link to right_wrist_yaw_link,
runs FK with current joint angles, applies a +X TCP offset (configurable, default 0.145m), and outputs
xyz (m) + rpy (rad) at a configurable rate.

Purpose: 用于实际点位示教/验证时实时查看 TCP 在 torso_link 下的位姿。
Output: `src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py`
</objective>

<execution_context>
@/home/unitree/.claude/get-shit-done/workflows/execute-plan.md
@/home/unitree/.claude/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/quick/260430-fxj-tcp-torso-link/260430-fxj-CONTEXT.md

<interfaces>
<!-- Key contracts the executor needs. Extracted from the codebase. -->

From `ik_fcl_ompl_planner.cpp` — KDL FK pattern (lines 107-167):
```cpp
#include <kdl_parser/kdl_parser.hpp>
// Build tree:
if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) { ... }
// Extract chain:
if (!kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)) { ... }
// Create FK solver:
fk_right_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_right);
// Use it:
KDL::Frame result;
int fk_ret = fk_right_solver->JntToCart(joint_array, result);
// result.p gives xyz (KDL::Vector); result.M gives orientation (KDL::Rotation)
// result.M.GetRPY() -> fixed-axis XYZ roll,pitch,yaw
```

From `ultralytics_detector.py` — ROS2 Python node pattern:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        self.declare_parameter('param_name', default_value)
        value = self.get_parameter('param_name').value
        self.sub = self.create_subscription(MsgType, 'topic', self.callback, 10)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

URDF chain: torso_link → right_shoulder_pitch_joint → ... → right_wrist_yaw_joint → right_wrist_yaw_link

KDL chain segment order for joints: right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint, right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint (7 DOF)

PyKDL Rotation.GetRPY() returns fixed-axis XYZ Euler angles: rotate about X (roll), then Y (pitch), then Z (yaw).
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: Create tcp_torso_pose.py node</name>
  <files>src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py</files>
  <action>
Create the ROS2 Python node `tcp_torso_pose.py` in `scripts/` following the same structure as `ultralytics_detector.py`.

The node must:

**Parameters** (declare_parameter + get_parameter):
- `urdf_path` (str): path to URDF file, default: "" (empty = load from `/robot_description` ROS param or fallback to package share default)
- `tcp_offset_x` (float): TCP X offset from right_wrist_yaw_link, default `0.145` (per CONTEXT TCP definition). Declared as ROS2 parameter — NOT hardcoded.
- `base_link` (str): base link name, default `"torso_link"`
- `tip_link` (str): tip link name, default `"right_wrist_yaw_link"`
- `publish_rate` (float): output rate in Hz, default `10.0`

**Initialization (__init__):**
1. Get all parameters
2. Load URDF:
   - If `urdf_path` is non-empty: read file and parse with `urdf_parser_py.urdf.URDF.from_xml_string()`
   - If `urdf_path` is empty: try `/robot_description` param first; if unavailable, fall back to the default URDF at the package share path `ament_index_python.get_package_share_directory('unitree_g1_dex3_stack') + '/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf'`
   - Parse with: `from urdf_parser_py.urdf import URDF` then `robot_urdf = URDF.from_xml_string(urdf_xml)`
3. Build KDL tree: `from kdl_parser_py.urdf import treeFromUrdfModel` then `ok, kdl_tree = treeFromUrdfModel(robot_urdf)`. If not OK, log FATAL and exit.
4. Extract chain: `ok, chain = kdl_tree.getChain(base_link, tip_link)`. If not OK, log FATAL with the link names attempted.
5. Build ordered joint name list matching KDL chain segment order:
   ```python
   kdl_joint_names = []
   for i in range(chain.getNrOfSegments()):
       seg = chain.getSegment(i)
       joint = seg.getJoint()
       if joint.getType() != PyKDL.Joint.None:
           kdl_joint_names.append(joint.getName())
   ```
   Log the resolved joint names at INFO level so the user can verify.
6. Create FK solver: `fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)`
7. Create /joint_states subscriber: `self.create_subscription(sensor_msgs.msg.JointState, '/joint_states', self.joint_state_callback, 10)`
8. Create timer for periodic output: `self.create_timer(1.0 / publish_rate, self.timer_callback)`
9. Flags: `self.got_first_state = False`, `self.first_state_msg_printed = False`, `self.current_joints = None`

**joint_state_callback(msg):**
1. Build `PyKDL.JntArray(chain.getNrOfJoints())`
2. For each index `i`, joint name `jn` in `kdl_joint_names`: find index of `jn` in `msg.name` list, copy `msg.position[idx]` into `JntArray[i]`. If joint name not found in message, default to 0.0.
3. Store `self.current_joints = joint_array`, set `self.got_first_state = True`

**timer_callback:**
1. If `not self.got_first_state`: print "Waiting for /joint_states..." once (use flag), return
2. Clear the waiting flag if set
3. Run FK: `fk_solver.JntToCart(self.current_joints, fk_result)`
4. Apply TCP offset: `tcp_frame = fk_result * PyKDL.Frame(PyKDL.Vector(tcp_offset_x, 0.0, 0.0))`
5. Extract xyz: `tcp_frame.p.x()`, `.y()`, `.z()`
6. Extract rpy: `tcp_frame.M.GetRPY()` → (roll, pitch, yaw) in fixed-axis XYZ order
7. Log: `self.get_logger().info(f'TCP in torso_link: xyz=[{x:.4f}, {y:.4f}, {z:.4f}] m, rpy=[{r:.4f}, {p:.4f}, {y:.4f}] rad')`

**main function:**
Standard rclpy spin pattern with KeyboardInterrupt handling, matching `ultralytics_detector.py`.

Make file executable: `chmod +x scripts/tcp_torso_pose.py`.
  </action>
  <verify>
    <automated>python3 -c "import ast; ast.parse(open('src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py').read()); print('Syntax OK')"</automated>
  </verify>
  <done>
New file `scripts/tcp_torso_pose.py` exists, is syntactically valid Python, implements all CONTEXT-mandated behaviors (dynamic FK via /joint_states subscription, TCP offset as ROS2 parameter default 0.145, xyz+rpy 4dp output on fixed-axis XYZ), follows ROS2 Python node conventions from `ultralytics_detector.py`.
  </done>
</task>

<task type="auto">
  <name>Task 2: Register script in CMakeLists.txt</name>
  <files>src/unitree_g1_dex3_stack-main/CMakeLists.txt</files>
  <action>
Add the new script to the `install(PROGRAMS ...)` block in CMakeLists.txt, right after the existing `scripts/ultralytics_detector.py` line.
The install block currently reads:
```
install(PROGRAMS
  scripts/ultralytics_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
```
Add `  scripts/tcp_torso_pose.py` on a new line immediately after `scripts/ultralytics_detector.py`, keeping the existing indentation.
  </action>
  <verify>
    <automated>grep -q "tcp_torso_pose.py" /home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/CMakeLists.txt && echo "Registered OK" || echo "MISSING"</automated>
  </verify>
  <done>
`tcp_torso_pose.py` is listed in the `install(PROGRAMS ...)` block of CMakeLists.txt, so `colcon build` will install it alongside other scripts.
  </done>
</task>

</tasks>

<verification>
**Syntax check:** `python3 -c "import ast; ast.parse(open('src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py').read())"` passes.
**CMake install check:** `grep -q "tcp_torso_pose.py" CMakeLists.txt` returns 0.
**Manual integration test** (after colcon build + source + ros2 launch robot.launch.py): run `ros2 run unitree_g1_dex3_stack tcp_torso_pose.py` — verify terminal outputs xyz and rpy at 10 Hz in the format `xyz=[X.XXXX, Y.XXXX, Z.XXXX] m, rpy=[R.XXXX, P.XXXX, Y.XXXX] rad`.
**Parameter check:** `ros2 run unitree_g1_dex3_stack tcp_torso_pose.py --ros-args -p tcp_offset_x:=0.2` should use 0.2m offset.
**Parameter list verify:** `ros2 param list` should show `/tcp_torso_pose` node with `tcp_offset_x`, `base_link`, `tip_link`, `urdf_path`, `publish_rate`.
</verification>

<success_criteria>
- [ ] Script runs without import errors after `colcon build`
- [ ] Node subscribes to `/joint_states` and outputs xyz+rpy at configured rate
- [ ] TCP offset (default 0.145m +X from wrist_yaw_link) is a ROS2 parameter, not hardcoded
- [ ] Output format: `xyz=[x.xxxx, y.xxxx, z.xxxx] m, rpy=[r.xxxx, p.xxxx, y.xxxx] rad`
- [ ] CMakeLists.txt installs the script
</success_criteria>

<output>
After completion, create `.planning/quick/260430-fxj-tcp-torso-link/260430-fxj-SUMMARY.md`
</output>
