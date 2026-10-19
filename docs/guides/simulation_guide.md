# Gazebo 仿真（M1）：12 库位 + 三轴龙门 + 控制/视觉桩

> 版本：2026-10-19（M1 端到端自验通过：真值桩 12/12、**真视觉 `aic_vision_trad` 12/12**，
> 单点约 3.0 s；注入 `drop_rate:=0.6` 时巡视不中断）
> 代码：`src/simulation/aic_gazebo_sim/`、`src/vision/aic_vision_trad/`
> 环境：Ignition Gazebo 6.18（Fortress）+ ROS 2 Humble + ros_gz_bridge

## 1. 它在整条链路里的位置

```text
aic_behavior_tree (C++)          本包（仿真侧）
  GotoNextSlot ── /control/material_task ──► gantry_controller.py ──► /aic/joint/*/cmd_pos
                                                                          │ ros_gz_bridge
                                                                          ▼
                                                                   Gazebo 关节控制器
  WaitVisionResult ◄── /vision/material_type ── sim_vision_node.py ◄── /sim/gantry_arrived
```

- **决策侧一行不改**（除新增的 `control_wait_ready` 握手，见第 5 节坑 10）。
- 本包的 `gantry_controller.py` 就是 **control 侧的第一个可用实现**；
  真机 control 接进来时，把本包换掉即可，话题契约不变。
- 本包的 `sim_vision_node.py` 是 **vision 侧的 ground-truth 桩**；
  `aic_vision_trad` / `aic_vision_dl` 就绪后换掉它，话题契约不变。

## 2. 场景与坐标

唯一数据源：`src/simulation/aic_gazebo_sim/config/sim_layout.yaml`。
世界文件 `worlds/aic_warehouse.sdf` 由 `scripts/gen_world.py` 生成，**不要手改**。

```text
world frame = warehouse（与 MaterialTask.header.frame_id 一致）
+X = 操作者视角「右」（4 列方向）      +Y = 「远」（row 3）
+Z = 上
库位: slot_id = (row-1)*4 + col，row 1 在 -Y（前排）
  col 1..4 → x = -0.375, -0.125, +0.125, +0.375
  row 1..3 → y = -0.1667, 0, +0.1667
台面 z = 0.10，库位地贴厚 3 mm，铁片 φ80×6 mm（顶面 z = 0.109）
```

龙门（`models/aic_gantry/model.sdf`）：

| 关节 | 轴 | 行程 | 零位含义 |
|---|---|---|---|
| `x_joint` | +X | −0.45 … +0.45 | 相机在货架中线 |
| `y_joint` | +Y | −0.24 … +0.24 | 同上 |
| `z_joint` | +Z | −0.42 … 0 | 0 = 末端最高位（夹指底面 0.135 m） |

末端：两根法兰横条 + 两根黄色夹指（x = ±0.036）+ 俯视相机
（相机在 link 内 `0 0 -0.18 0 1.5708 -1.5708`，行进高度下距铁片约 0.26 m，
视野约 0.30 m 宽，铁片占画面约 27%）。
图像方向：**右 = +X（列增大），上 = −Y（行号减小）**。

![末端相机俯视图](assets/sim_camera_view.png)

> 上面是行进高度下 3 号位点（污渍）的实拍：中间灰色圆盘是铁片，盘内棕色圆斑是
> 「污渍」缺陷，左右两根金色的是夹指，四周深灰是库位地贴。
> 该图由 `ros2 run aic_gazebo_sim sim_snapshot.py` 直接抓取。

## 3. 控制侧实现（gantry_controller.py）

- 订阅 `/control/material_task`，**FIFO 队列**逐条执行（决策是 fire-and-forget，不等回执）。
- 运动 = **协调梯形速度剖面**（`_profile()`），50 Hz 流式下发 setpoint；
  x/y 同起同停，Z 单独按 `z_speed` 走。参数在 `sim_layout.yaml` 的 `gantry:` 段
  （`xy_speed 0.30 m/s`、`z_speed 0.20 m/s`、`pick_z -0.04`、`handle_dwell_s 0.35`）。
- 任务语义：
  - `GOTO`：先抬 Z 到行进高度 → x/y 到位 → 等实际停稳 → 发 `/sim/gantry_arrived(slot_id)`
  - `HANDLE`：Z 下降到 `pick_z` → 停留 0.35 s（夹爪动作占位）→ 抬回 → 发 `/sim/handle_done(slot_id)`
  - `HOME`：回零位（预留）
- **到位判定是闭环的**：剖面走完后进入「保持 setpoint」阶段，等
  `/world/aic_warehouse/model/aic_gantry/joint_state`（经桥转成 `sensor_msgs/JointState`）
  里目标轴的 `position` 进 `pos_tol`、`velocity` 进 `vel_tol`，连续 `confirm_ticks` 拍成立
  才报到位；超 `move_timeout_s` 按到位放行并告警。拿不到反馈时退化为固定 `settle_s`。
  实测稳态误差 0.0–1.2 mm，剖面结束后 0.08–0.44 s 判到位。

## 4. 视觉侧桩（sim_vision_node.py）

- 订阅 `/sim/gantry_arrived`，延时 `settle_s(0.2) + delay_s` 后发一条
  `std_msgs/String`，取值 `"1".."4"`（来自 `sim_layout.yaml` 的 `materials.types`）。
- 故障注入（复现决策侧兜底路径）：

```bash
# 60% 位点「识别失败」→ 决策记 TIMEOUT，巡视继续
ros2 launch aic_gazebo_sim sim_decision.launch.py drop_rate:=0.6 seed:=7
# 30% 位点返回非法值 "9" → 决策丢弃并计数
ros2 launch aic_gazebo_sim sim_decision.launch.py invalid_rate:=0.3
```

### 4.1 真视觉后端（aic_vision_trad）

`vision_backend:=trad` 时不起真值桩，改起 `src/vision/aic_vision_trad`：

```bash
./scripts/run_sim.sh --vision trad                              # 端到端
./scripts/run_sim.sh --vision trad --debug-dir logs/vision_debug # 存标注图（判错的现场证据）
```

链路：`/camera/image` + `/vision/trigger` → `vision_node.py` → `/vision/material_type`。
定位与判据不依赖「画面居中」：整图 Hough 找盘 + 内亮外暗环对比选优，
再做「盘内亮度基底 → 暗区连通域 → 相对盘半径归一化」的四类判据。
离线回归脚本 `scripts/bench_offline.py` 可对 `logs/vision_ds/` 里的样本一次性打分。

## 5. 踩过的坑（改仿真前必读）

1. **物理系统不会自动加载**。本机 Fortress 在有 `<physics>` 段时**不**自动装
   `Physics` system，且 dartsim 引擎库在 `ign-physics-5/engine-plugins/`，
   默认搜索路径找不到。表现极具迷惑性：世界能起、时钟在走、相机出图，
   **但什么都不动**（自由落体都不落）。
   解法：世界 SDF 里显式
   `<plugin filename="ignition-gazebo-physics-system" name="...Physics"><engine>ignition-physics-dartsim-plugin</engine></plugin>`
   + launch 里补 `IGN_PHYSICS_ENGINE_PATH`。
   **一分钟自检**：放一个自由落体球，2 s 后看 `pose/info` 里它掉没掉。
2. **`joint_index` 是「该关节内部的自由度序号」**，不是「模型里第几个关节」。
   三个单自由度 prismatic 全部填 `0`；填 `0/1/2` 时只有 X 能动，Y/Z 静默失效。
3. **没有 `SceneBroadcaster` 就没有 `/world/<w>/state` 与 `pose/info`**：
   `ign model -m ... -l ...` 会一直 "Service call timed out"，
   你会误判成"服务不通/沙箱问题"。
4. **关节反馈要看对话题**：`/world/<w>/pose/info` 里 prismatic 的 `axis1` 确实缺省
   （那是「位姿」话题，不是「关节」话题）；但 **`JointStatePublisher` 发的
   `/world/<w>/model/<model>/joint_state` 是有 `position`/`velocity` 的**，桥成
   `sensor_msgs/JointState` 即可（`JointState[ignition.msgs.Model`）。
   早期把它误判成「不回传」，于是用「剖面 + 定时」判到位，代价见坑 11。
5. **Z 轴（竖直 prismatic）会被重力拉到底**：位置控制器是速度伺服，
   在 controller 起来之前的窗口里 Z 会滑到下限，之后**卡在下限爬不回来**。
   M1 直接给 `z_carriage` 关掉重力（`<gravity>false</gravity>`），
   位置保持完全交给控制器；恢复真实重力时要配制动/重力前馈。
   另外控制器**必须每拍都下发**（不只是运动时），否则 Z 会继续下沉。
6. **ros_gz_bridge 方向符**：`topic@ros_type]gz_type` 是 ROS→GZ（右括号），
   `topic@ros_type[gz_type` 是 GZ→ROS（左括号）。写反了桥会"建好了"，但一条都收不到。
7. **相机不能与 Z 立柱同轴**：立柱是实体视觉，相机埋在里面拍到的是"管子内壁"。
   现在立柱偏到 x = −0.075，法兰做成两根横条，中间留光路。
8. **`declare_parameter(name, 0.5)` 会把类型钉死 double**：
   命令行 `-p count:=5` 传进来是 INTEGER，rclpy 抛
   `InvalidParameterTypeException`，节点**启动即崩**（表现为"抓图没输出"）。
   本包统一用 `sim_layout.declare_number()`（`dynamic_typing`）规避。
9. **`HOME` 缺失**时 ign gazebo 会把日志/缓存退化到 `/.ignition` 并刷警告，
   容器/CI 里要显式给 `HOME`（launch 已兜底）。
10. **首条 GOTO 会丢**：决策是 fire-and-forget，control 订阅尚未建立时消息直接蒸发，
    表现是「第 1 个位点必然 TIMEOUT」。解法：仿真 control 周期发 `/control/ready`，
    决策侧 `control_wait_ready=true` 时先等就绪再开跑
    （`bt_runner.cpp` 里新增的等待段，超时 `control_ready_timeout_s=10 s`）。
11. **「剖面走完」≠「机台到位」**（最贵的一个坑，2026-10-19）。
    初版 `JointPositionController` 用 `p_gain=20 / d_gain=0.6`，伺服严重欠阻尼：
    375 mm 阶跃实测「1.25 s 剖面结束 → 4.8 s 才追上 → 过冲 17.7 mm → 8.6 s 才停稳」。
    而控制侧在剖面 + 0.15 s 就报到位，那一刻实际位置差 185 mm，**相机画面偏一整格**，
    视觉必然判不出铁片；看起来像「相机出图滞后 2 s」，其实是机台还没到。
    两个修正缺一不可：
    - 增益提到 `p=1600 / d=200`（过冲 0.5 mm、约 1.9 s 进 ±1 mm）；
    - 到位改闭环（读 `joint_state` 的 position/velocity），不再数时间。
    另外做了对照实验才排除「渲染排队」：相机从 20 Hz 降到 5 Hz，画面差异**完全没有改善**，
    同一位置 +0.9 s 与 +2.8 s 两帧仍不同 —— 说明瓶颈不在渲染。
12. **`WaitVisionResult` 的窗口从「GOTO 下发」开始计时，不是从「到位」开始**。
    控制器是 FIFO，GOTO(N+1) 下发时队列里还压着 HANDLE(N)（降 Z + 停留 + 升 Z ≈ 1.1 s），
    再加 GOTO 自身 ≈ 1.3 s，`vision_timeout_s=3.0` 必然不够：实测每轮 3~4 个位点
    TIMEOUT，**更糟的是迟到的结果会被下一个位点当成自己的答案收下**（类型错位，
    而结果表仍显示 OK）。`sim_decision.launch.py` 默认值已改 5.0，并写明它必须覆盖
    「HANDLE + GOTO + 视觉延迟」。

## 6. 已验证 / 未做

**已验证（2026-10-19）**

- 真值桩单轮 12 位点闭环：OK 12 / TIMEOUT 0，结果 CSV 与 `materials.types` 逐位一致，
  平均单点 3.03 s。
- **真视觉 `aic_vision_trad` 单轮 12 位点：OK 12 / TIMEOUT 0，四类全对**（平均单点 3.05 s）。
  判据输出示例：划痕 `elong≈20.7 circ≈0.12`、孔洞 `circ≈0.98 drop≈124`、
  污渍 `frac≈0.09 circ≈0.93`、缺角 `rad≈0.78 elong≈3.0`。
- 故障注入：`drop_rate:=0.6` → OK 4 / TIMEOUT 8，`FAILURE` 不出现，巡视跑完。
- 到位精度：反馈判定，稳态误差 0.0–1.2 mm。
- 无头渲染出图（EGL，无需 X），`sim_snapshot.py` 可批量存 PNG。
- `ros2 launch` 在决策退出后自动关停（`OnProcessExit → Shutdown`）。

**未做（M2 候选）**

1. **抓取/码放的真实物理**：现在是「下降 + 停留 + 抬升」的动作占位，
   铁片是 `static` 模型；要做 `DetachableJoint` 或 `ros2_control` + 夹爪多关节。
2. **分类码放区**：赛题若要求按类型放到 4 个区域，需要在世界里加 4 个料框并改 HANDLE 轨迹。
3. **视觉从仿真迁移到实拍**：`aic_vision_trad` 的阈值是在渲染图上标的
   （背景单一、无光照变化）。实拍要重新标定：先固定相机→盘面距离，
   再按「盘半径像素」归一化所有几何量（现在已经是相对量，换尺度只需改 `disc_rmin_frac`）。
4. **GUI 交互**：现在 `gui:=true` 只起观察窗口，没有做模型拖动/摆位工具。
