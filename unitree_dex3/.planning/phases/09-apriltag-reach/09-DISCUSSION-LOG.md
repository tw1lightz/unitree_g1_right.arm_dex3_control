# Phase 9: 端到端集成 - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in `09-CONTEXT.md` — this log preserves the alternatives considered.

**Date:** 2026-05-19
**Phase:** 09-apriltag-reach
**Areas discussed:** 触发模型, 桥接实现, 过时位姿+平滑, UAT 验收范围

---

## Area 1 — 触发模型（Trigger Model）

### 1.1 触发方式

| Option | Description | Selected |
|--------|-------------|----------|
| 按键触发 | 按 G 时一次性发 latest target_pose 到 /goal_pose；安全可预测；与 Phase 5 D-05 同形态 | ✓ |
| 自动触发 | detector 稳定输出 target_pose 即立即规划+执行；流畅；人手风险大 | |
| 混合 (planner 持续规划 + executor 等按键) | 安全 + 流畅；但要修改 executor / 加 gate 节点；复杂度↑ | |

**User's choice:** 按键触发
**Notes:** 沿用 Phase 5 D-05 安全设计，符合 "演示 + 小批量场景人手风险高" 的考虑。

### 1.1b 触发键位

| Option | Description | Selected |
|--------|-------------|----------|
| 沿用 K | muscle memory；与老 YOLO 触发语义混淆风险 | |
| 换一个键（区分 YOLO 时代）| | ✓ |
| Launch arg 配置 | trigger_key:=K 默认 | |

**User's choice:** 换一个键（区分 YOLO 时代）— 后续具体到 G

### 1.1c 具体新键

| Option | Description | Selected |
|--------|-------------|----------|
| G ("go") | 单字符；与 K 视觉区分明显 | ✓ |
| A ("apriltag") | 语义直观 | |
| T ("trigger") | 泛用 | |

**User's choice:** G

### 1.2 重复按键防护

| Option | Description | Selected |
|--------|-------------|----------|
| 拒绝 + WARN | 等当前一次完成；保守 | ✓ |
| 接受最新一次 | 替换前一个 /goal_pose；中途变向风险 | |
| 不防护 | planner / executor 自然处理 | |

**User's choice:** 拒绝 + WARN
**Notes:** Bridge 需维护 `waiting_for_completion` 状态机；监听完成信号源 = the agent's discretion（D-25）。

---

## Area 2 — 桥接实现（Bridge Implementation）

### 2.1 桥接节点策略

| Option | Description | Selected |
|--------|-------------|----------|
| 新建 apriltag_goal_bridge.py + 删除老 keyboard_trigger_node.py | 干净；旧文件无 v1.1 用途 | ✓ |
| 新建 + 保留老文件不在 launch | 保留历史参考；下次 cleanup 还得删 | |
| 原地改造 keyboard_trigger_node.py | diff 最小；文件名隐含 YOLO 误导 | |

**User's choice:** 新建 + 删除老文件
**Notes:** Phase 6 已清理大部分 YOLO 残留，keyboard_trigger 是最后一个孤儿。

### 2.2 节点职责边界

| Option | Description | Selected |
|--------|-------------|----------|
| 纯桥接 | 只缓存最新 + 按 G republish | |
| 带 reachability 预检 | 距离/方向粗检，明显不可达直接拒绝 | ✓ |
| 带 OpenCV visualization | 与 detector imshow 重复 | |

**User's choice:** 带 reachability 预检
**Notes:** 在 bridge 层提前拒绝可避免 planner 1s timeout 浪费 + 减少 ROS 日志噪声。

### 2.3 桥接节点 launch 归属

| Option | Description | Selected |
|--------|-------------|----------|
| 仅 apriltag_reach.launch.py 启动 | 独立无意义（依赖 detector）| ✓ |
| 也提供独立 apriltag_goal_bridge.launch.py | 便于离线 ros2 topic pub 验证桥接 | |

**User's choice:** 仅 apriltag_reach 启动

### 2.2b 预检判据

| Option | Description | Selected |
|--------|-------------|----------|
| 仅距离 ≥ 0.80 m | reach_max_distance 默认 0.80 | |
| 距离 + 工作侧（不越中线） | | |
| 距离 + Z 范围 | 避免地面以下/头顶以上 | |
| 全部三项 | 阈值都 ROS 参数 | |
| Other：reach-radius ≥ 0.55 m | 用户指定经验阈值（基于 Phase 8 UAT center-far 不可达点）| ✓ |

**User's choice:** Other — reach-radius ≥ 0.55 m 单条件
**Notes:** 0.55 m 来自 Phase 8 UAT center-far 实测；近端 (<0.05 m) 由 planner D-08 处理，bridge 不重复。阈值落 ROS 参数 `reach_max_distance`。

### 2.2c 拒绝时 UX

| Option | Description | Selected |
|--------|-------------|----------|
| WARN 一行 + 不发 /goal_pose | 与 Phase 7 静默风格一致 | ✓ |
| WARN + ROS service/topic 反馈 | 调试更友好；surface 膨胀 | |

**User's choice:** WARN 一行（保持简单）

---

## Area 3 — 过时位姿 + 平滑

### 3.1 取值策略

| Option | Description | Selected |
|--------|-------------|----------|
| 最新单帧 | 最简；对 PnP 噪声不抗 | |
| 滑动平均（最近 N 帧）| 抗噪；移动 tag 有 ~0.3s 滞后 | ✓ |
| 滑动中位数 | 更抗离群；实现稍复杂 | |

**User's choice:** 滑动平均

### 3.2 Stale 阈值

| Option | Description | Selected |
|--------|-------------|----------|
| 拒绝 + WARN，T=0.5s | 安全；偶发遮挡可能误拒 | |
| 拒绝 + WARN，T=1.0s | 更宽松，偶发遮挡仍可触发 | ✓ |
| 不拒绝，stale 也照发 | planner / 用户负责安全 | |

**User's choice:** T=1.0s

### 3.3 缓存为空（首次启动 + 按 G）

| Option | Description | Selected |
|--------|-------------|----------|
| 拒绝 + WARN "no AprilTag detected yet" | | ✓ |
| 拒绝 + 一次性提示 "make sure tag is in camera view" | | |

**User's choice:** WARN "no AprilTag detected yet"

### 3.1b 滑动窗口 N

| Option | Description | Selected |
|--------|-------------|----------|
| N = 5 (~0.33 s @ 15 Hz) | 默认建议 | ✓ |
| N = 3 (~0.20 s) | 响应快，抗噪弱 | |
| N = 10 (~0.67 s) | 抗噪强，滞后明显 | |

**User's choice:** N = 5

### 3.1c 平均范围

| Option | Description | Selected |
|--------|-------------|----------|
| 仅位置；orientation 取最近一帧 | 反正 planner 覆盖 orientation | ✓ |
| 位置 + orientation 都平均 | quaternion slerp/avg 复杂；零收益 | |

**User's choice:** 仅位置
**Notes:** Phase 8 D-09/D-10 默认 `adaptive_orientation_enabled=true`，planner 覆盖 `/goal_pose.orientation`，bridge 端算朝向是浪费。

---

## Area 4 — UAT 验收范围

### 4.1 测试 tag 位置集

| Option | Description | Selected |
|--------|-------------|----------|
| 沿用 Phase 8 8 点 tabletop set | 连续性最好；可与 adaptive A/B 对照；含已知不可达点 | |
| 缩减到 4 点 | 0.55 m + 右侧筛选后；安全且可达子集 | ✓ |
| 全新 3 点（最小集）| 仅证明 pipeline 通；不做精度统计 | |

**User's choice:** 缩减到 4 点
**Notes:** Phase 8 已发现 center-far / left-of-mid 物理不可达 / 越中线，应用 0.55 m 自动剔除。

### 4.2 TCP 误差测量方法

| Option | Description | Selected |
|--------|-------------|----------|
| FK 软件法（/joint_states → right_tcp_link） | 自动化、易归档；不暴露 URDF 模型差 | ✓ |
| 外部物理测量（直尺 / 第二个 tag） | 真实精度；人工不可批量 | |
| 1 + 2 都做 | 首跑两种建立偏置；后续仅 FK | |

**User's choice:** FK 软件法

### 4.3 误差阈值

| Option | Description | Selected |
|--------|-------------|----------|
| 2 cm | PnP+URDF 量级匹配；紧但合理 | |
| 3 cm | 给打印/摆放/标定误差余量 | ✓ |
| 5 cm | 演示导向；够近即可 | |

**User's choice:** 3 cm

### 4.4 Pass criteria

| Option | Description | Selected |
|--------|-------------|----------|
| All-pass（全部通过）| 严格 | |
| Phase 8 风格 N/M | 失败点 known issue 化 | ✓ |
| 仅记录，无硬阈值 | 人 review 决定 milestone | |

**User's choice:** N/M 风格

### 4.4b N/M 具体比例

| Option | Description | Selected |
|--------|-------------|----------|
| 3/4 | Phase 8 partial 风格余量 | |
| 4/4 | milestone 收官无余量；3 cm 已含余量应可达 | ✓ |
| Other | | |

**User's choice:** 4/4
**Notes:** 与 D-23 (3 cm 阈值已含余量) 配套；端到端是 milestone v1.1 收官。

### 4.5 apriltag_reach.launch.py 默认 imshow

| Option | Description | Selected |
|--------|-------------|----------|
| true | 演示阶段操作员看 OpenCV 窗口判断 tag 检测可信度 | ✓ |
| false | headless 友好；显式 imshow:=true 覆盖 | |
| Other | | |

**User's choice:** true

### 4.6 reach.launch.py 处置

| Option | Description | Selected |
|--------|-------------|----------|
| 保留，作为 planner-only manual test 入口 | 与 apriltag_reach 并列两条路径 | ✓ |
| 删除 | 端到端 launch 已替代 | |

**User's choice:** 保留
**Notes:** README 写明双入口用途：apriltag_reach（端到端）+ reach（planner 手动测试）+ apriltag（检测调试）。

---

## the Agent's Discretion

讨论中明确委托给下游 researcher / planner / agent 自行决定的事项：

- bridge 触发接受 / 拒绝日志的具体格式
- bridge 监听 "上一次完成" 的具体信号源（推荐订阅 `/joint_trajectory_targets` 取 `points[-1].time_from_start`；fallback 用 `/joint_states` velocity-zero 检测）
- shoulder origin TF lookup 重试策略（间隔 + 上限；建议 0.5 s 间隔 / 最多 10 次后 fatal）
- 滑动窗口实现选型（推荐 `collections.deque(maxlen=5)`）
- 4 点 tabletop 集体的具体坐标（从 Phase 8 8 点中筛选满足 0.55 m + 右侧条件即可）
- bridge 节点语言（推荐 Python，与 keyboard_trigger / apriltag_detector 一致）
- bridge 终端键盘读取实现（沿用 keyboard_trigger 的 raw mode 模式）
- UAT harness 的执行方式（推荐操作员逐点摆 tag → 按 G → harness 记录）
- UAT harness 是否引入额外依赖（建议不引入，纯 Python `print` 表格够用）

## Deferred Ideas

讨论中浮现但**不属于 Phase 9 范围**的想法：

- **Future ORI-02 multi-candidate orientation fallback** — 讨论收尾时由用户主动提出 backlog 票据归位需求；处理为创建 `.planning/todos/pending/ORI-02-multi-candidate-orientation.md`（汇集 Phase 8 UAT 三类失败现象 + 触发条件 + scope 边界）+ REQUIREMENTS.md ORI-02 单行 → ticket 引用。
- Future REQ TAG-05 多 tag 支持
- Future REQ ORI-03 tag 法线推导接近方向
- 自动触发模式（手动 G 更安全）
- bridge auto-retry on planner failure
- bridge 预检扩展到工作侧 / Z 范围
- bridge 反馈 topic / service
- emergency-stop / cancel goal topic
- bridge 性能优化 / C++ 重写
