# Phase 1 Debug 全记录（中文）

> 从 Plan 01-04（安全关机）到 Plan 01-12b（最终修复），共 **9 轮迭代**。这是 Unitree G1 右臂轨迹执行器 end-of-trajectory snap 问题的完整 Debug 时间线。

---

## 背景

目标：轨迹执行完成后，机器人右臂应平滑回到站立姿态，而非瞬间弹回（snap）。

系统：ROS 2 Foxy + arm_sdk（bare DDS 发布到 firmware）+ `joint_trajectory_executor`。

---

## 迭代 1：Plan 01-04 — 安全关机功能

**用户报告**：执行器关机时手臂没有平滑释放，担心安全问题。

**修复**：
- 禁用 rclcpp 默认 SIGINT handler
- 安装自定义 handler，设置 `g_shutdown_requested` 原子标志
- `main()` 中轮询代替 `rclcpp::spin()`
- 关机前执行 3 秒 `gracefulRelease()`，线性降低 master switch（kNotUsedJoint.q）1.0→0.0

**结果**：关机不抖动 ✅，但轨迹**完成后**仍有 snap ❌

---

## 迭代 2：Plan 01-06 — 修复 gracefulRelease 夺权跳变

**用户报告**：轨迹完成后手臂会猛抖一下。

**诊断**：`gracefulRelease()` 在 `main()` 中执行，此时 firmware 已重新获得身体控制权，但 `gracefulRelease` 把 master 重新拉回 1.0→0.0，瞬间夺权→放权的跳变导致冲击。

**修复**：
- 删除 `gracefulRelease()` 方法
- 删除 `main()` 中的调用
- 在 `trajectoryCallback` 的 waypoint 循环中加入 `g_shutdown_requested` 检查，中断后落到已有 end-of-trajectory ramp

**结果**：关机路径正确 ✅，但轨迹完成后仍有 snap ❌

---

## 迭代 3：Plan 01-07 — Ramp 填充实际关节位置

**用户报告**：仍有弹跳。

**诊断**：end-of-trajectory ramp 的 `LowCmd` 中 `motor_cmd[idx].q = 0.0f`（默认初始化），导致 ramp 期间命令手臂弹到零位。

**修复**：用 `latest_joint_positions_` 填充 ramp 的 q 值，kp=60, kd=1.5。

**结果**：snap 减轻但仍存在 ❌

---

## 迭代 4：Plan 01-08 — kp/kd 衰减

**诊断猜测**：body controller 在 ramp 期间可能和我们竞争，kp/kd 衰减可以让控制权平滑交接。

**修复**：3 秒 ramp / 150 步，线性衰减 kp 60→0, kd 1.5→0。

**结果**：没有改善 ❌，且因 master switch 不是混合模式，衰减无效。

**回退**：Plan 01-09 撤销此修改。

---

## 迭代 5：Plan 01-09 — 显式 q 插值回 standing

**诊断**：即使 q 填了实际位置，ramp 只维持同一位置 3 秒，没有主动把手臂带回 standing。如果 firmware 在 ramp 结束时接管，而手臂还停在半空，firmware 会把它拽回 standing → snap。

**修复**：
- callback 入口处 snapshot `standing_pose = latest_joint_positions_`
- ramp 中 q 从 trajectory end-point 线性插值到 `standing_pose`
- kp/kd 恢复 60/1.5（撤销 01-08 的衰减）

**结果**：仍 snap ❌

---

## 迭代 6：Plan 01-10 — 启用 PD 控制模式

**诊断**：对比参考实现 `robot_arm.py`，发现它设置了 `self.msg.motor_cmd[id].mode = 1`。我们的 executor 中 mode 默认为 0，firmware 可能不响应 q/kp/kd。

**修复**：在两个 per-joint 填充循环中加入 `motor_cmd[idx].mode = 1`：
1. 轨迹跟随循环
2. end-of-trajectory ramp 循环

**结果**：仍 snap ❌（用户反馈 "还是不行"）

---

## 迭代 7：Plan 01-11 — 消除 2 秒发布空窗 + 提升 ramp 频率

**诊断**：
- `ros2 topic info -v /arm_sdk`：publisher 1 个，类型为 bare DDS app
- `ros2 topic hz /arm_sdk`：静默 → ROS 2 工具看不到 bare DDS 的发布，但**我们的发布能到达 firmware**（asymmetric IDL）
- 参考实现 `smooth_exit()` 是 500 步 / 2 秒 = **250 Hz** 无间断发布
- 对比发现 executor 在最后一个 waypoint 和 ramp 之间有 **2 秒 sleep**：
  ```
  sleep_for(1s)  // 等 last command 生效
  publish hand_close
  sleep_for(1s)  // 等 hand close
  ```
  这 2 秒 executor 完全不发布 → firmware 自己控制手臂 → 已经 snap 了
- 且 ramp 频率：150 步 / 3 秒 = **50 Hz**，远低于参考的 250 Hz

**修复**：
1. 替换两个 `sleep_for(1s)` 为 1 秒 × 250Hz 的 hold-publish loop（master=0.5, mode=1, kp=60, kd=1.5, q=latest），手关闭指令在第 0 帧发出
2. ramp steps 150→750（3 秒 / 250 Hz）

**结果**：snap 提前了！轨迹一结束就弹回 standing ❌

---

## 迭代 8：Plan 01-12 — 修复 stale latest_joint_positions_

**诊断**："弹回"意味着手臂被命令回到 standing。

关键发现：**`trajectoryCallback` 在单线程 executor 中运行，`lowstateCallback` 被阻塞。** 整个 callback 期间 `latest_joint_positions_` 永远是 callback 入口时的 standing 值。

所以：
- hold loop：`q = latest_joint_positions_[idx]` = standing → 命令回 standing → 弹！
- `ramp_start_positions = latest_joint_positions_` = standing → ramp 从 standing 到 standing → 无意义

**修复**：从 `msg->points.back()` 显式提取 trajectory endpoint：
```cpp
std::vector<float> trajectory_endpoint = standing_pose; // baseline
for (last_point.positions) {
    trajectory_endpoint[idx] = clamp(last_point.positions[j], limits);
}
```
- hold loop 使用 `trajectory_endpoint`（手臂停在终点）
- ramp 使用 `trajectory_endpoint` 作为起点（手臂从终点回到 standing）

**结果**：不再弹回，但到达终点后**冲击 overshoot** 一段再缓慢回 standing ❌

---

## 迭代 9：Plan 01-12b — 修复 master 不连续

**诊断**：
| 阶段 | master (kNotUsedJoint.q) |
|------|--------------------------|
| 轨迹循环 | 0.5 |
| Hold loop | 0.5 |
| **Ramp step 0** | **1.0** ← 跳变！ |
| Ramp step 750 | 0.0 |

master 从 0.5 跳到 1.0，planner 权重瞬间翻倍 → arm_sdk 猛推手臂超过目标 → overshoot。

**修复**：ramp master `1.0→0.0` 改为 `0.5→0.0`，与轨迹/hold 阶段连续。

**结果**：✅ **成功！平滑完成，无 snap，无 overshoot。**

---

## 最终正确时序

```
[轨迹循环]        master=0.5, q=waypoint, mode=1, kp=60, kd=1.5 @ 轨迹频率
     ↓
[Hold loop]       master=0.5, q=trajectory_endpoint, mode=1, kp=60, kd=1.5 @ 250Hz × 1s
     ↓                                    ↑
     └────────────────────────────────────┘  hand_close 第 0 帧发出，并行执行
     ↓
[Ramp]            master: 0.5→0.0, q: endpoint→standing, mode=1, kp=60, kd=1.5 @ 250Hz × 3s
     ↓
[结束]            master=0.0, firmware 接管，手臂已在 standing
```

**全程无发布空窗，master 连续，q 连续。**

---

## 核心教训

1. **单线程 executor 阻塞 callback**：`latest_joint_positions_` 在 callback 内永远不会更新，不能依赖它获取轨迹终点。
2. **发布空窗 = 控制权丢失**：sleep 期间 firmware 接管，等你想平滑过渡时已经晚了。
3. **参考实现的频率本身就是协议的一部分**：250 Hz 不是可选优化，是连续控制的最低要求。
4. **所有信号必须连续**：master、q、mode、kp、kd——任何跳变都是冲击源。
5. **asymmetric IDL 的坑**：ROS 2 工具（`ros2 topic hz`）看不到 bare DDS publisher，不代表 topic 没数据。不要据此推断系统状态。

---

*归档时间：2025-04-30*
*对应 git 提交：`5a29e0f` (01-12b master 0.5→0.0)*
