# 电梯按键视觉方案 — 方案B增强版

> 无侵入 · 语义感知 · 触觉闭环  
> 适用平台：Unitree G1 + Dex3-1 灵巧手 + Intel D435i + ROS 2

---

## 开发策略：三步递进

### 第一步 — SatArw/yolonas_ocr 现有模型（立即尝试）

先用 [SatArw/yolonas_ocr](https://github.com/SatArw/yolonas_ocr) 仓库中的现有模型验证检测+识别效果：

| 组件 | 文件 | 说明 |
|------|------|------|
| 按钮检测 | `frozen_model/detection_graph.pb` | Faster R-CNN，TF1，类别：`button` |
| 字符识别 | `frozen_model/ocr_graph.pb` | 专用OCR，47字符，输入180×180 |

**接入方式：**
- 用 `tf.compat.v1` 加载 `.pb` 冻结图（兼容TF2环境）
- 检测输出 BBox → 深度投影 → 发布 `/elevator/target_pose`
- OCR 输出楼层字符 → 发布 `/elevator/floor_label`

**验证目标：** 确认现有模型在目标电梯环境下的检测率和识别率是否可用。

### 第二步 — 纯 OCR 兜底（如第一步效果差）

若 `detection_graph.pb` 在目标环境表现不佳，切换为 PaddleOCR 全图扫描：

```python
import re
valid = [d for d in ocr_results if re.match(r'^(B?\d+F?|[A-Z]?\d+)$', d['text'])]
```

### 第三步 — 训练 YOLOv11n（如前两步均不满足要求）

自采数据（50-100张）+ labelImg 标注 → 训练专用模型。

```
yolo train model=yolo11n.pt data=elevator.yaml epochs=50 imgsz=640
```

### 决策流程

```
尝试 SatArw 现有模型
  ├── 检测率 > 80% 且识别准确 → 直接集成，进入执行层开发
  ├── 检测差但识别OK         → 换 PaddleOCR 全图 + 保留 ocr_graph.pb
  └── 整体效果不满足         → 训练 YOLOv11n（第三步）
```

---

## 背景与动机

### 现有方案（AprilTag）的局限

| 问题 | 说明 |
|------|------|
| 侵入性 | 必须在电梯面板贴 AprilTag 标签 |
| 无语义 | 只知道"按哪里"，不知道"按几楼" |
| 依赖人工 | 每个电梯环境都需要重新布置标签 |

### 方案B增强版目标

- 直接识别电梯原生按钮，无需贴标签
- 通过 OCR 获取楼层语义（"按3楼"）
- 触觉闭环判定按压成功，不依赖纯视觉坐标

---

## 整体架构

```
感知层（双环语义定位）
  ├── YOLOv11  →  按钮 2D BBox
  ├── D435i 深度图  →  Z 坐标（毫米精度）
  └── PaddleOCR  →  楼层数字识别

        ↓ /elevator/target_pose (PoseStamped, torso_link)

决策层（五阶段状态机）
  IDLE → DETECT → ALIGN → PRESS → VERIFY → RETURN

        ↓ /goal_pose + Dex3 setpoint

执行层（顺应性按压）
  ├── 渐进式轨迹控制（模拟 Z 轴柔顺）
  └── 触觉阵列闭环（/dex3/right/tactile_agg）
```

---

## 一、感知层：双环语义定位

### 1.1 粗检索 — YOLO 按钮检测

- 订阅 `/camera/color/image_raw`
- YOLOv11n 框选面板区域内所有按钮（2D BBox）
- PaddleOCR 对每个 BBox crop 识别楼层字符
- 建立映射表：`{楼层字符 → 像素中心坐标 (cx, cy)}`

**YOLO 权重选择：**

| 选项 | 工作量 | 精度 |
|------|--------|------|
| 通用 YOLO + 手工 ROI 裁剪送 OCR | 0天 | 中 |
| 标注 ~200 张图微调 YOLOv11n | 2-3天 | 高 |

### 1.2 精定位 — D435i 深度投影

锁定目标按键后，从深度图提取 Z 坐标：

```python
# 深度图切片 + 双边滤波去噪（应对金属反光面板）
roi = depth_img[y1:y2, x1:x2]
roi_filtered = cv2.bilateralFilter(roi.astype(np.float32), 9, 75, 75)
depth_m = np.median(roi_filtered[roi_filtered > 0]) / 1000.0

# 反投影：像素 → 相机坐标系
x_c = (cx - K[0,2]) * depth_m / K[0,0]
y_c = (cy - K[1,2]) * depth_m / K[1,1]
z_c = depth_m

# TF2 变换至 torso_link → 发布 /elevator/target_pose
```

**精度估算（机器人站稳状态）：**

- D435i 深度误差：±2mm @ 0.5m，±6mm @ 1.5m
- YOLO 中心点像素误差：±3~5px ≈ ±3mm
- **综合误差：< 10mm** ✅ 满足按压需求

### 1.3 ROS Topic 接口

| Topic | 方向 | 说明 |
|-------|------|------|
| `/camera/color/image_raw` | 订阅 | RGB 图像（已有） |
| `/camera/color/camera_info` | 订阅 | 内参（已有） |
| `/camera/depth/image_rect_raw` | 订阅 | **新增** |
| `/elevator/target_pose` | 发布 | 目标按键三维位姿 |
| `/elevator/floor_label` | 发布 | OCR 识别的楼层字符（String） |

---

## 二、决策层：五阶段状态机

```
┌─────────┐    站稳 + 触发     ┌──────────┐
│  IDLE   │ ─────────────────► │  DETECT  │
└─────────┘                    └──────────┘
                                     │ 目标楼层确认
                                     ▼
                               ┌──────────┐
                               │  ALIGN   │ ← (可选) Livox 对齐面板朝向
                               └──────────┘
                                     │
                                     ▼
                               ┌──────────┐
                               │  PRESS   │ ← 渐进刺击 + Dex3 伸指
                               └──────────┘
                                     │
                                     ▼
                               ┌──────────┐
                               │  VERIFY  │ ← 触觉阵列判定成功
                               └──────────┘
                                     │
                                     ▼
                               ┌──────────┐
                               │  RETURN  │ ← 回缩 + return_to_standing
                               └──────────┘
```

| 阶段 | 触发条件 | 执行动作 |
|------|---------|---------|
| IDLE | — | 等待触发键（G 键 或 ROS topic） |
| DETECT | 触发 | YOLO + OCR 建立楼层→坐标映射 |
| ALIGN | 检测完成 | (可选) 激光雷达辅助调整朝向 |
| PRESS | 对齐完成 | 渐进轨迹 + Dex3 伸中指 → 按压 |
| VERIFY | 触觉阈值达到 | 判定成功，Dex3 复位 |
| RETURN | 验证完成 | 手臂回缩 → return_to_standing |

---

## 三、执行层：顺应性按压

### 3.1 渐进式轨迹控制（Z 轴柔顺替代方案）

不修改底层控制器，用多步渐进模拟顺应性：

```
pre-contact（目标前 5cm）
  → 每步前进 2mm，检查触觉反馈
  → 合力超阈值 → 停止，判定成功
  → 超出最大步数 → 失败回退
```

参数建议：单步 2mm，最大 30 步（共 6cm 探进空间），每步超时 500ms。

### 3.2 触觉闭环判定（核心安全保障）

**数据来源：** `/dex3/right/tactile_agg`（Float32MultiArray）

```python
INVALID = 30000
SCALE   = 10000.0

valid = [p / SCALE - baseline[i]
         for i, p in enumerate(msg.data)
         if p != INVALID]

# 合力 > 阈值 且持续 0.5s → 判定按压成功
if sum(valid) > PRESS_THRESHOLD:  # 建议 5N-8N
    press_success = True
```

触觉判定优于纯视觉坐标到达：视觉有 5~10mm 残差，刚性"到坐标即停"可能顶坏减速器；触觉是物理接触的直接证明。

---

## 四、实施路线图

### Phase 1 — 核心路径（预计 3-5 天）

```
[ ] 新建 button_detector_node.py
      YOLO + D435i深度 + PaddleOCR
      发布 /elevator/target_pose + /elevator/floor_label

[ ] 改造 apriltag_button_press_node.py
      订阅 /dex3/right/tactile_agg
      用力阈值判定替换原"等待轨迹时间"逻辑
      兼容接收 /elevator/target_pose
```

### Phase 2 — 增强（预计 1-2 周）

```
[ ] 五阶段状态机重构
[ ] YOLOv11n 按钮数据集标注 + 微调
[ ] Livox Mid-360 面板朝向对齐集成
```

---

## 五、与被否定方案的区分

| 方案 | 否定原因 | 本方案的区别 |
|------|---------|------------|
| YoloNAS OCR | TF1依赖 + 无3D坐标 + 模型未开源 | 用 PaddleOCR（已在项目）+ D435i真实深度 |
| YOLO-3D (niconielsen) | Depth Anything 相对深度，无物理单位 | 用 D435i 硬件深度（绝对毫米精度） |

---

## 六、关键参数速查

```yaml
# 感知层
yolo_conf_threshold: 0.45
ocr_lang: "ch"                    # 中文楼层（如"3F"）或 "en"
depth_bilateral_d: 9
depth_bilateral_sigma: 75

# 执行层
pre_contact_offset_x: 0.05       # 预接触退后 5cm（现有）
press_step_m: 0.002              # 每步前进 2mm
press_max_steps: 30              # 最大探进 6cm
press_step_timeout_s: 0.5        # 每步超时

# 触觉闭环
tactile_press_threshold_n: 6.0  # 判定按压成功的合力阈值（N）
tactile_hold_duration_s: 0.5    # 持续时间要求
tactile_invalid_value: 30000    # 无效 taxel 标记值
tactile_scale: 10000.0          # 压力归一化系数
```

---

*文档生成时间：2026-06-09*  
*基于项目 `unitree_g1_right.arm_dex3_control` 现有代码分析*
