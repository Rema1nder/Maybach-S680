# CircleHandler 状态机与元素处理说明

本文档描述当前 `User/CircleHandler.c` 的实现逻辑，重点覆盖：
- 外层分段状态机（赛道阶段）
- 内层圆环状态机（入环/环内/出环）
- 元素处理（横线、直角、停车）
- 线性加减速策略与蓝牙事件

---

## 1. 总体结构

当前控制采用**双层状态机**：

1. **外层阶段 `s_phase`**（赛道流程）
   - `PHASE_SEG1_START_STRAIGHT`
   - `PHASE_SEG2_RIGHT_CIRCLE`
   - `PHASE_SEG3_STRAIGHT_CURVE`
   - `PHASE_SEG4_SHARP_LINE`
   - `PHASE_SEG5_STRAIGHT_SWITCH`
   - `PHASE_SEG6_LEFT_CIRCLE`
   - `PHASE_SEG7_RIGHT_ANGLE_PARK`

2. **内层圆环状态 `s_state`**（圆环动作）
   - `CIRCLE_STATE_NORMAL`
   - `CIRCLE_STATE_ENTERING`
   - `CIRCLE_STATE_IN_CIRCLE`
   - `CIRCLE_STATE_EXITING`

`Circle_Update()` 在 `SysTick`（2ms）中被调用，按优先级执行：
- 停车处理
- 急停/退出保护
- 第7段停车并行检测
- 状态机更新

---

## 2. 外层分段状态机（s_phase）

## SEG1 发车+直线
- 启动后等待 `RIGHT_ENABLE_DELAY_MS`（1s）
- 检测右环入口特征（连续 `ENTRY_CONFIRM_CNT` 帧）
- 成功后：
  - 发送 `EVT:SEG_END:1`
  - 进入 `SEG2`，并进入内层 `ENTERING`

## SEG2 右环段
- 由内层圆环状态机驱动（`ENTERING -> IN_CIRCLE -> EXITING`）
- 右环完成时在 `Circle_CompleteExit()` 中切到 `SEG3`
  - 发送 `EVT:SEG_END:2`

## SEG3 直线+缓曲线（含两个独立直角序列）

### 3.1 两次独立直角序列
第3段有两次独立序列，每次均为：
1. 线性降速 1m 到 70
2. 开启直角检测
3. 直角完成后线性加速 1m 到 110

- **第1次序列**：进入 SEG3 后立即启动
- **第2次序列**：三横线计满后启动

`Turn Sequence` 完成次数由 `s_seg3_turns_done` 计数：
- `<2`：继续留在 SEG3
- `>=2`：发送 `EVT:SEG_END:3`，切换到 SEG4，并 `Odometer_Reset()`

### 3.2 三横线计数门控
三横线计数仅在满足后才启用：
- 里程门控：`seg3_traveled_mm >= SEG3_CROSSLINE_ENABLE_DISTANCE_MM`（当前 7500mm）
- 启用时发送：`EVT:SEG3_7500`

横线判定防误触：
- 连续确认：`SEG3_CROSSLINE_CONFIRM_CNT`（当前 4 帧）
- 相邻横线最小间隔：`SEG3_CROSSLINE_MIN_GAP_MM`（当前 350mm）
- 计数目标：`SEG3_CROSSLINE_TARGET_COUNT`（3 条）

## SEG4 急折线段
- 条件：里程 `> 6000mm`
- 动作：发送 `EVT:SEG_END:4`，切换 SEG5

## SEG5 直线+换曲线段
- 检测左环入口特征（连续确认）
- 成功后发送 `EVT:SEG_END:5`，切入 SEG6 + 内层 `ENTERING`

## SEG6 左环段
- 由内层圆环状态机驱动
- 左环完成时在 `Circle_CompleteExit()` 中：
  - 发送 `EVT:SEG_END:6`
  - 切入 SEG7

## SEG7 连续直角弯+停车
- 停车检测并行启用（延时 `PARKING_ENABLE_DELAY_MS`）
- 横线计数达到 `CROSSLINE_TARGET_COUNT` 后抱死停车
- 停车持续 `PARKING_STOP_DURATION_MS` 后退出巡线并发送 `EVT:SEG_END:7`

---

## 3. 内层圆环状态机（s_state）

## NORMAL
- 外层阶段分发逻辑的主执行态

## ENTERING
- 半侧追线引导入环（右环取右半侧，左环取左半侧）
- 定时 `ENTERING_DURATION_MS` 到后进入 `IN_CIRCLE`

## IN_CIRCLE
- 全幅巡线 + 角速度积分
- 满足出环触发条件后转 `EXITING`
  - 右环：里程窗口 + 边缘黑点条件
  - 左环：目标侧边缘黑点条件

## EXITING
- 先半侧引导，后强制直行中心位
- 满足完成判据（里程+黑点）或超时兜底后调用 `Circle_CompleteExit()`

---

## 4. 元素处理细节

## 4.1 右/左环入口检测
- 使用左右侧黑点分布 + 中间区黑点进行特征判定
- 通过连续帧确认去抖

## 4.2 横线检测
- 使用对称外侧对：(0,15)、(1,14)、(2,13)
- 至少 2 对同时为黑判为横线

## 4.3 直角检测（顺序约束）
`Circle_DetectRightAngleDone()` 规则：
1. 启动检测时先 `LSM6DSR_ClearYaw()`
2. 必须先出现 `|Yaw| > RIGHT_ANGLE_DONE_YAW_DEG`（当前 75）
   - 发送调试事件 `EVT:TURN90_YAW_OK`
3. 在 yaw 条件成立后，再检测中间传感器（7 或 8）黑，连续 `RIGHT_ANGLE_CENTER_CONFIRM_CNT` 帧（当前 2）
4. 满足后发送 `EVT:TURN90_DONE`

---

## 5. 线性加减速机制

使用统一接口：
- `Circle_StartBaseSpeedRampByDistance(target_base_speed, ramp_distance_mm)`
- `Circle_UpdateBaseSpeedRampByDistance()`

机制：
- 调用时读取当前 `base_speed/speed_range`
- 按里程线性插值更新 `base_speed`
- 到达目标后自动停止并发送：
  - `EVT:SPD_RAMP_DONE:<target>`

当前在 SEG3 的使用：
- 每次直角序列前：降速到 70（1m）
- 每次直角完成后：加速到 110（1m）

---

## 6. 蓝牙事件总览

分段事件：
- `EVT:SEG_END:1` ~ `EVT:SEG_END:7`

第3段相关：
- `EVT:SEG3_7500`
- `EVT:SPD_RAMP_DONE:70`
- `EVT:SPD_RAMP_DONE:110`
- `EVT:TURN90_YAW_OK`
- `EVT:TURN90_DONE`

---

## 7. 关键可调参数（当前值）

- `SEG3_CROSSLINE_ENABLE_DISTANCE_MM = 7500.0f`
- `SEG3_CROSSLINE_TARGET_COUNT = 3`
- `SEG3_CROSSLINE_CONFIRM_CNT = 4`
- `SEG3_CROSSLINE_MIN_GAP_MM = 350.0f`
- `RIGHT_ANGLE_DONE_YAW_DEG = 75.0f`
- `RIGHT_ANGLE_CENTER_CONFIRM_CNT = 2`

---

## 8. 调参建议

- 若直角仍偏迟：先小幅下调 `RIGHT_ANGLE_DONE_YAW_DEG`（如 75 -> 70）
- 若误触发偏多：上调 `RIGHT_ANGLE_CENTER_CONFIRM_CNT` 或 `SEG3_CROSSLINE_MIN_GAP_MM`
- 若三横线仍误计数：上调 `SEG3_CROSSLINE_CONFIRM_CNT`
