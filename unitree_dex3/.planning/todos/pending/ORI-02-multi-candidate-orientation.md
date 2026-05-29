# ORI-02 — Multi-Candidate Orientation Fallback

**Status:** Pending (backlog)
**Created:** 2026-05-19 (during Phase 9 discuss)
**Source:** REQUIREMENTS.md §"Future Requirements"
**Related Phase 8 ADRs:** D-04 (single deterministic orientation, multi-candidate explicitly out of scope)

---

## Goal

让 planner 在单 orientation 求解失败（IK 不可行 / OMPL 找不到无碰撞路径）时，自动尝试多个候选末端朝向，覆盖 Phase 8 自适应 orientation 没救回来的三类失败模式。

## Origin — Phase 8 UAT 失败现象

详见 `.planning/debug/resolved/08-uat-5of8.md`（Phase 8 现场 A/B 实测，2026-05-19）。
adaptive=true 取得 5/8 PASS vs baseline 4/8 PASS，余 3 点失败按现场分析归类如下：

| 失败点 | 现象 | 根因 | ORI-02 是否能救 |
|--------|------|------|-----------------|
| center-far | 物理不可达（≥ 0.85 m）| 单 orientation 与机械臂全展开都不够 | **否** — 需要上层限制 reach 或换更长臂；ORI-02 救不了物理距离 |
| left-of-mid | 双模式中线碰撞 | TCP 路径需穿越躯干中线，碰撞约束硬卡 | **是** — 多候选 orientation 中，肩内旋 / 手肘外摆的姿态可能绕开中线碰撞 |
| center-near | 单 orientation 取舍：adaptive 反而比 baseline 难 | 单一 look-at 朝向把 IK 解推到关节极限附近 | **是** — 多候选中至少一个会回到 baseline 风格的朝向，恢复可行性 |

> 期望成功率：8/8（roadmap 原文 "明显提升" 的目标）；ORI-02 在 collision 类两点上有救，center-far 一类需另作处理。

## Trigger Condition（什么时候做这个票）

任一条件满足就值得排期：
1. milestone v1.2 或后续把 "右臂可达 ≥ 7/8 tabletop targets" 列为门槛；
2. Phase 9 端到端 UAT 中发现新场景的 left-of-mid / center-near 类失败被工地复现；
3. 接下来要做 `Future REQ TAG-05`（多 tag）后，每个 tag 摆放都可能碰到 collision-prone 朝向，单 orientation 失败成本更高。

如果一直只在 right-side / front-near tabletop 范围内演示，ORI-02 可以无限延后。

## Scope Sketch（实现时再细化）

- **入口：** Phase 8 已有的 `IKFCLPlannerNode::computeAdaptiveOrientation` 返回单四元数；ORI-02 改为返回 `std::vector<Quat>`（首位仍是当前确定式 look-at 朝向）。
- **候选生成：** 围绕 look-at +X 轴做有序 roll 偏移（如 ±15°、±30°、±45°）；越靠前候选越接近 Phase 8 当前实现，保持 deterministic & backward-compatible。
- **求解循环：** 顺序对每个候选跑 TRAC-IK + OMPL；首个成功的就用。失败计数 + 最后失败原因写日志。
- **超时分配：** 现有 `planning_timeout`（默认 1.0 s）按候选均分，或顶层包一个总 budget。
- **Backward compatibility：** 默认候选数 = 1 时，行为字节级等同 Phase 8 D-04；新增 ROS 参数 `orientation_candidate_count`（默认 1）。

## Out of Scope（避免与其他 Future REQ 混淆）

- **不**用 tag normal 推导接近方向（那是 Future ORI-03，单独一票）。
- **不**做 reach-radius 物理截断（那已是 Phase 9 桥接节点的 reach-radius 预检 0.55 m，覆盖 center-far 类）。
- **不**改 `right_tcp_link` 或 TCP offset 语义（Phase 6 锁定）。

## References

- `.planning/debug/resolved/08-uat-5of8.md` — UAT 失败现场分析
- `.planning/phases/08-adaptive-orientation/08-CONTEXT.md` D-01..D-04 — Phase 8 单 orientation 决策
- `.planning/phases/08-adaptive-orientation/08-VERIFICATION.md` — verifier 11/11 PASS + HV 记录
- `.planning/REQUIREMENTS.md` §"Future Requirements" - ORI-02 单行条目
- `.planning/ROADMAP.md` Phase 8 success #3/#4 — partial 注释指向本票
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — `computeAdaptiveOrientation` 是改造入口

---
*Move to `.planning/todos/done/` 当 ORI-02 落地并合入。*
