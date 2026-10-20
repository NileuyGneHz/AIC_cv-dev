# Gazebo 仿真（M2）：12 库位 + 三轴龙门 + 抓取码放闭环

> 版本：2026-10-20（M2 端到端自验通过：真视觉 `aic_vision_trad` **12/12 四类全对**，
> 且 12 块铁片**全部落到对应类型的码放格**；单点约 6.3 s，整轮约 80 s）
> 历史：M1（2026-10-19）打通「到位 → 识别 → 记分」，单点 3.0 s。
> 代码：`src/simulation/aic_gazebo_sim/`、`src/vision/aic_vision_trad/`
> 环境：Ignition Gazebo 6.18（Fortress）+ ROS 2 Humble + ros_gz_bridge

## 1. 它在整条链路里的位置

```text
aic_behavior_tree (C++)          本包（仿真侧）
  GotoNextSlot ── /control/material_task ──► gantry_controller.py ──► /aic/joint/*/cmd_pos
                                                    │  ▲                  │ ros_gz_bridge
                                                    │  │ joint_state      ▼
                                                    │  │            Gazebo 关节控制器
                                                    │  └── /world/<w>/model/aic_gantry/joint_state
                                                    │
                                                    ├── /sim/attach_XX,/sim/detach_XX ──► DetachableJoint
                                                    ▼
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
台面 z = 0.10，库位地贴厚 3 mm（只有 visual，不参与碰撞）
铁片 φ80×6 mm 落在台面板上（中心 z = 0.103）
```

码放区（`place_zones:`，M2 新增）：+Y 侧另起一条 900×140 mm 的台面，4 个格口沿 X 排列：

| 类型 | 1 划痕 | 2 污渍 | 3 缺角 | 4 孔洞 |
|---|---|---|---|---|
| 格心 x | −0.36 | −0.12 | +0.12 | +0.36 |
| 格心 y | +0.30 | +0.30 | +0.30 | +0.30 |

> 真机台面 1000×500 已被 12 库位占满，龙门行程也只有 y ∈ [−0.24, 0.32]，
> 挤不下 4 个料框。仿真为了把「识别 → 抓取 → 搬运 → 按类型释放」整条动作跑通，
> 在货架后沿接了一条 120 mm 深的码放条（台面总深变 640 mm）。
> **真机布置不同时只改这里**：坐标是唯一数据源，生成器与控制器都读它。
> 同类型 3 块铁片会落在同一个格心，**因此必然叠放**（见坑 15）。

龙门（`models/aic_gantry/model.sdf`）：

| 关节 | 轴 | 行程 | 零位含义 |
|---|---|---|---|
| `x_joint` | +X | −0.45 … +0.45 | 相机在货架中线 |
| `y_joint` | +Y | −0.24 … **+0.32** | 同上（上限放到 0.32 才够到码放条） |
| `z_joint` | +Z | −0.42 … 0 | 0 = 末端最高位（夹指底面 0.135 m）；−0.04 = 抓取/放置位 |

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
  - `HANDLE`：**下探 → 停留 → 夹持 → 抬升 → 平移到该类型的码放格 → 放下（触底即停）
    → 停留 → 释放 → 抬升 → 发 `/sim/handle_done(slot_id)`**（M2 完整搬运，单次约 5 s）。
    类型未知（0）或没配码放格时退化为「下探 + 停留 + 抬升」，不抓取 ——
    对应决策侧 `SkipSlot` 的语义（未识别不放料）。
  - `HOME`：回零位（预留）
- **抓取靠 `DetachableJoint`**：世界由 `gen_world.py` 生成时，把龙门**内联**进世界，
  再给 12 块铁片各挂一个插件（`parent_link=z_carriage`、`child_model=material_XX`、
  话题 `/sim/attach_XX` / `/sim/detach_XX`）。控制器发 `std_msgs/Empty` 经桥转成
  `ignition.msgs.Empty` 即完成夹/放。
  **开机必须主动松爪**：插件在首次 PreUpdate 就把 12 块铁片全焊到 z_carriage 上（坑 13）。
- **到位判定是闭环的**：剖面走完后进入「保持 setpoint」阶段，等
  `/world/aic_warehouse/model/aic_gantry/joint_state`（经桥转成 `sensor_msgs/JointState`）
  里目标轴的 `position` 进 `pos_tol`、`velocity` 进 `vel_tol`，连续 `confirm_ticks` 拍成立
  才报到位；超 `move_timeout_s` 按到位放行并告警。拿不到反馈时退化为固定 `settle_s`。
  实测稳态误差 0.0–1.2 mm，剖面结束后 0.08–0.44 s 判到位。
  放料那一段用独立的 `place` 步：判据是「已触底」（速度≈0 + 静差 ≤ `place_tol`），
  因为叠放的第二块必然比目标高一个片厚（坑 15）。

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
    控制器是 FIFO，GOTO(N+1) 下发时队列里还压着 HANDLE(N)（M2 的完整搬运 ≈ 4.3 s），
    再加 GOTO 自身 ≈ 1.5 s，`vision_timeout_s` 必须覆盖「HANDLE + GOTO + 视觉延迟」。
    偏小的后果不是「慢」，而是**迟到的结果被下一个位点当成自己的答案收下**
    （类型错位，而结果表仍显示 OK）。
    实测：M2 下成功位点的等待落在 5.4–8.4 s，8.0 只剩 0.1 s 余量 → 稳定丢 3 个；
    现默认 **12.0**。真机 HANDLE 不含搬运（夹爪一开一合），参数应按真机节拍重定。
13. **`DetachableJoint` 不是「等 attach 话题才连」——它在首次 `PreUpdate` 就把 child 焊上**
    （最坑的一个，2026-10-20）。Fortress 版插件只认 `parent_link` / `child_model` /
    `child_link` / `attach_topic` / `detach_topic` / `output_topic` 六个字段，
    **没有 `initial_attach` 开关**。12 个插件 = 12 块铁片开机全挂在 z_carriage 上：
    静置 Z 被压到 **−3.675 mm**，正好等于 `12 × 0.05 kg × 9.81 ÷ p_gain(1600)`，
    而速度恒为 0 —— 闭环判据（pos + vel 双阈值）永远等不到，每段 Z 都「到位超时 8 s」。
    更隐蔽的是**指令一下发就动不了**：铁片焊在末端，Z 8 s 只走了 0.2 mm（目标 −0.04，
    实际 −0.0002，误差 39.8 mm）。
    解法：开机广播几轮 `detach` 把 12 块全部放开。两个前提缺一不可：
    - 等**第一帧 `joint_state`** 再发（它同时证明 bridge 已起、ROS→GZ 的 24 个订阅已建好）；
    - **`/control/ready` 等松爪跑完再发**。实测决策端在控制器启动后 0.02 s 就发来
      第一条 GOTO，若用「收到任务就停发」来保护，松爪会被永久跳过。
    自检：静置 `z_joint.axis1.position` 应 ≈ 0（现在是 −1e−48），铁片 z 应 ≈ 0.103。
14. **`<collision>` 漏写 `<pose>`：碰撞体与可视体错位 90 mm**。
    `rack` 的台面 visual 有 `pose z=top_z−0.01`，collision 没写 → 碰撞板落在 link 原点
    （z ∈ [−0.01, 0.01]）。铁片还是 `static` 时完全看不出来；一改动态就**整批掉到这块
    看不见的碰撞板上**（实测全部停在 z = 0.013，而相机里看台面还是 0.10，视觉跟着失配）。
    `place_table` 更狠：collision 连 Y 也没给，落在 y = 0（货架下方），码放到 y = 0.30
    的铁片会直接穿到地面。**凡是 visual 带 pose 的，collision 必须给同一份 pose。**
15. **同类型铁片会叠放，「精确到位」判据必然超时**。
    12 个位点 / 4 个类型 = 每格 3 块，第二块落在第一块上，比目标高**一个片厚 6 mm**，
    第三块高 12 mm。用「位置±3 mm + 速度≤10 mm/s」判，Z 永远差 6 mm → 每块白等
    `move_timeout_s`（实测 8 s），**而且会把下一个位点的视觉窗口挤爆**，连锁出 3 个
    TIMEOUT + 类型错位。解法：放料单独用 `place` 步，判据改成「已触底」：
    残余速度 ≤ 3 mm/s 且静差 ≤ `place_tol(15 mm)`。日志里打印「已触底（静差 X mm）」。
16. **图像订阅 `depth` 不能是 5**：识别（Hough）在定时器里跑几十毫秒，期间图像回调在
    队列里排队，depth=5 时执行器会把 5 帧旧图逐帧送进来 → 「到位后第一帧」可能还是
    0.25 s 前运动中拍的（实测整盘偏出画面中心 30 px，被当成污渍）。
    订阅端改 `depth=1`（只要最新一帧）+ `settle_skip_s` 0.15 → 0.25 s。
17. **缺角的 `frac` 阈值不要贴着实测值**：`frac` 用**估计出的盘半径**归一化，
    而 Hough 半径在同一台相机下有 ±9% 抖动（实测 80.3 px vs 87.3 px）。
    r 偏小时 0.93r 的掩膜会把压在盘缘的缺角切掉大半，`frac` 从 0.072 掉到 0.030 ——
    原来 0.035 的阈值正好卡在实测边缘，12 位点里会漏 1 个（判成污渍）。
    现改 0.020；真正挡住「圆柱侧壁暗沿」的是 `elong ≤ 5`（侧壁实测 7.4–11.3）。

## 6. 已验证 / 未做

**已验证（2026-10-20，M2）**

- **真视觉 + 完整搬运，单轮 12 位点：OK 12 / TIMEOUT 0，四类全对**
  （划痕 3 / 污渍 3 / 缺角 3 / 孔洞 3，与 `materials.types` 逐位一致），平均单点 6.3 s，
  整轮约 80 s。
- **落位核对**（`pose/info` 读 12 块铁片的最终世界坐标）：

  | 类型 | 格心 x | 落位铁片 | 实测 x | 实测 z |
  |---|---|---|---|---|
  | 1 划痕 | −0.36 | slot 1 / 6 / 10 | −0.3600 / −0.3600 / −0.3601 | 0.103 / 0.109 / 0.115 |
  | 2 污渍 | −0.12 | slot 3 / 5 / 11 | −0.1200 / −0.1200 / −0.1201 | 0.103 / 0.109 / 0.115 |
  | 3 缺角 | +0.12 | slot 2 / 8 / 9 | 0.1200 / 0.1200 / 0.1201 | 0.103 / 0.109 / 0.115 |
  | 4 孔洞 | +0.36 | slot 4 / 7 / 12 | 0.3600 / 0.3600 / 0.3600 | 0.103 / 0.109 / 0.115 |

  y 全部 0.300±0.0001；z 每叠一块正好 +6 mm（片厚），说明「叠放」是物理结果而非穿模。
- 夹持/释放 12 对全部成对出现，8 次「已触底」，**0 次到位超时**。
- 单点抓放探针（只跑 slot 1）：铁片从 (−0.375, −0.1667) 精确落到 (−0.360, 0.300)，
  其余 11 块原地不动。

**M1 遗留（仍有效）**

- 真值桩单轮 12 位点闭环：OK 12 / TIMEOUT 0，平均单点 3.03 s。
- 故障注入：`drop_rate:=0.6` → OK 4 / TIMEOUT 8，`FAILURE` 不出现，巡视跑完。
- 到位精度：反馈判定，稳态误差 0.0–1.2 mm；无头 EGL 出图；
  `ros2 launch` 在决策退出后自动关停。
  > 注意：M2 之后单点耗时从 3 s 涨到 6.3 s（多了搬运），
  > `drop_rate` 复测请按新节拍看结果。

**未做**

1. **GUI 交互**：现在 `gui:=true` 只起观察窗口，没有做模型拖动/摆位工具。
2. **视觉从仿真迁移到实拍**：`aic_vision_trad` 的阈值是在渲染图上标的
   （背景单一、无光照变化、缺陷是几何体）。实拍要重标：先固定相机→盘面距离，
   再按「盘半径像素」归一化（几何量现在已经是相对量，换尺度只需改 `disc_rmin_frac`）；
   另外缺角的 `frac` 依赖盘半径估计精度（坑 17），实拍时建议加一步圆拟合精修。
3. **多格位扩展**：现在每个类型只有一个格心（3 块叠放）。若真机是「一个料框一条，
   框内并排」，把 `place_zones` 改成 `xs` 二维（每型多个落点）即可，控制器只读它。
4. **夹爪几何**：现在夹指是两根静止的黄色方块，靠 `DetachableJoint` 做「吸附」，
   没有真正的开合关节与夹持力。要更真实可换 `ros2_control` + 两指 prismatic。
