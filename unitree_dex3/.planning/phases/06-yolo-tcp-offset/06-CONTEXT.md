# Phase 6: YOLO 清理 + TCP Offset 集成 - Context

**Gathered:** 2026-05-15
**Status:** Ready for planning

<domain>
## Phase Boundary

彻底移除不可行的 YOLO 检测代码及其依赖（包括 `project_to_3d_node`、`detection_to_goal_node`、`bboxes_ex_msgs` 包），将 0.175m TCP offset 通过 URDF 虚拟 link 集成到 planner IK 链末端，精简 `reach.launch.py` 为可用的手动测试配置。

交付物：
1. YOLO 相关文件全部删除（含 `perception.launch.py`、`best.pt`、`run_perception.sh`）
2. URDF 添加 `right_tcp_link`（fixed joint, x=0.175）
3. Planner `right_tip` 参数默认改为 `right_tcp_link`，支持 `tcp_offset_x` ROS 参数覆盖
4. `reach.launch.py` 精简为 robot + planner + control

</domain>

<decisions>
## Implementation Decisions

### A — YOLO 清理范围
- **D-01:** 彻底删除以下文件/目录：
  - `src/unitree_g1_dex3_stack-main/scripts/ultralytics_detector.py`
  - `src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp`
  - `src/unitree_g1_dex3_stack-main/src/detection_to_goal_node.cpp`
  - `src/unitree_g1_dex3_stack-main/src/visual_detection_yolo_tester.cpp`
  - `src/unitree_g1_dex3_stack-main/launch/perception.launch.py`
  - `src/unitree_g1_dex3_stack-main/launch/visual_detect_yolo.launch.py`
  - `src/unitree_g1_dex3_stack-main/docs/ARCHITECTURE(yolo,failed).md`
  - `src/bboxes_ex_msgs/` （整个包）
  - `best.pt`
  - `run_perception.sh`
- **D-02:** 保留 `visual_detection_tester.cpp` 和 `visual_detect_click.launch.py`（对 AprilTag 调试有参考价值）。
- **D-03:** CMakeLists.txt 和 package.xml 中移除所有对已删文件/包的引用（`bboxes_ex_msgs` 依赖、编译目标等）。

### B — TCP Offset 集成
- **D-04:** 在 URDF 中 `right_wrist_yaw_link` 之后添加虚拟 fixed link `right_tcp_link`，joint origin x=0.175m。TRAC-IK 天然支持 fixed joint 延伸链末端。
- **D-05:** 修改两个 URDF 文件：`g1_29dof_lock_waist_with_hand_rev_1_0.urdf`（默认）和 `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`（planner 碰撞检测版）。RViz 可视化能看到 TCP frame。
- **D-06:** Planner `right_tip` 参数默认值从 `right_wrist_yaw_link` 改为 `right_tcp_link`（launch 文件和代码默认值都改）。
- **D-07:** 添加 ROS 参数 `tcp_offset_x`（double, 默认 0.175）。Planner 启动时读取该参数，用其值覆盖 KDL chain 末端 segment 的偏移量（即 URDF 写默认值，代码可运行时覆盖）。

### C — reach.launch.py 处理
- **D-08:** 精简 `reach.launch.py` 为 robot + planner + control。移除 perception include、keyboard_trigger_node、model_path/target_class/imshow 参数。保留 d435_tf static publisher（后续 AprilTag 需要）和 planning_timeout 参数。
- **D-09:** Phase 9 会创建新的 `apriltag_reach.launch.py` 替代完整 pipeline。

### D — Planner 接口
- **D-10:** Planner 的 `/goal_pose`（PoseStamped）订阅接口不变。Phase 7 AprilTag 节点完成前，用 `ros2 topic pub /goal_pose` 手动验证 TCP offset 集成正确性。

### Agent's Discretion
- `visual_detection_tester.cpp` 和 `visual_detect_click.launch.py` 的编译依赖处理方式（从 CMakeLists.txt 移除编译目标 / 重构依赖 / 其他）— agent 选最简单不报错的方案。
- `tcp_offset_x` 参数覆盖 KDL chain 末端偏移的具体实现方式（修改 segment / 重建 chain / 其他）。
- `keyboard_trigger_node.py` 是否从 install 中清理（它不在 launch 中了但可能仍被 install）。

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### 被修改的文件
- `src/unitree_g1_dex3_stack-main/CMakeLists.txt` — 移除已删节点的编译目标和 `bboxes_ex_msgs` 依赖
- `src/unitree_g1_dex3_stack-main/package.xml` — 移除 `bboxes_ex_msgs` 依赖
- `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` — `right_tip` 默认值改为 `right_tcp_link`，添加 `tcp_offset_x` 参数
- `src/unitree_g1_dex3_stack-main/launch/reach.launch.py` — 精简为 robot + planner + control
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf` — 添加 `right_tcp_link`
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — 添加 `right_tcp_link`

### Planner 源码（TCP offset 集成）
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — L65 `right_tip` 参数声明，L107 获取，L127 KDL chain 构建，L247 默认值。需添加 `tcp_offset_x` 参数和 chain 末端偏移覆盖逻辑。

### TCP offset 参考实现
- `src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py` — 已验证的 TCP offset 模式：FK 到 wrist_yaw_link 后 `Frame(Vector(tcp_offset_x, 0, 0))` 偏移。Planner 的 IK 反向等价。

### 前序阶段约束
- `.planning/phases/05-end-to-end-integration/05-CONTEXT.md` — D-08: `detection_to_goal_node` 保留决策（本阶段推翻：v1.1 彻底清理）
- `.planning/phases/04-right-arm-only-executor/04-CONTEXT.md` — D-06: 28 关节 kp=60 coexistence 模式（不受本阶段影响）
- `.planning/phases/03-trajectory-smoothing-validation/03-CONTEXT.md` — velocity_scale=0.2, min_time_step=0.02（不受影响）

### Joint 定义
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` — Joint enums，确认 right arm chain 范围

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `tcp_torso_pose.py` 的 FK + offset 模式 — 验证了 0.175m X 轴偏移的正确性，planner 的 URDF virtual link 是同一概念的 IK 等价
- `planner.launch.py` 的 `OpaqueFunction` + `LaunchConfiguration.perform()` 模式 — 添加 `tcp_offset_x` 参数时复用
- `reach.launch.py` 的 `IncludeLaunchDescription` 结构 — 精简时保留框架，只删 perception 和 keyboard 部分

### Established Patterns
- URDF fixed joint 添加：参考现有 hand link 的 fixed joint 结构
- ROS 参数声明：`declare_parameter` + `get_parameter` 模式（planner 中已有多个示例）
- Launch 参数传递：`DeclareLaunchArgument` + `LaunchConfiguration` → Node parameters

### Integration Points
- Planner 的 KDL chain 构建（L127）：`kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)` — 改 `right_tip_` 默认值即可让 chain 自动延伸到 `right_tcp_link`
- TRAC-IK solver 初始化：使用同一 chain，自动包含 fixed joint 延伸
- FCL 碰撞检测：`right_tcp_link` 无碰撞几何体（fixed link, 无 collision element），不影响碰撞检测

</code_context>

<specifics>
## Specific Ideas

- URDF 中 `right_tcp_link` 不需要 visual 或 collision 元素（它是一个虚拟参考点），只需 `<link name="right_tcp_link"/>` + fixed joint with origin xyz="0.175 0 0"。
- `tcp_offset_x` 参数覆盖时，需要在 KDL chain 构建之后、IK solver 初始化之前修改 chain 末端 segment 的 frame。
- 精简后的 `reach.launch.py` 保留 `d435_tf_node`（static transform publisher for d435_link → camera_link），Phase 7 AprilTag 需要这个 TF。

</specifics>

<deferred>
## Deferred Ideas

- **keyboard_trigger_node 改造** — 未来可改为订阅 AprilTag 检测结果而非 `/detections_3d`，属于 Phase 9 集成范围。
- **多 TCP offset 配置** — 如果未来有不同工具（不同 offset），可扩展为 YAML 配置。当前单一 offset 足够。
- **URDF xacro 化** — 将所有 URDF 改为 xacro 以支持参数化，工作量大且当前不需要。

</deferred>

---

*Phase: 06-yolo-tcp-offset*
*Context gathered: 2026-05-15*
