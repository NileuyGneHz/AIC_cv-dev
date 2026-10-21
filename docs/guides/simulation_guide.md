# Gazebo 仿真（M4）：来料架 3x4 + 三轴龙门（吸盘）+ 识别后分拣入架

> 本文是**原理深读文档**（架构、实现细节、23 个坑的来龙去脉）；
> 装环境 / 构建 / 跑仿真 / 自验 / 配置 / 排障的**操作手册**见 `simulation_manual.md`，
> 两份按「为什么读这篇 / 怎么办读那篇」分工。

> 版本：2026-10-20（M2 端到端自验通过：真视觉 `aic_vision_trad` **12/12 四类全对**，
> 且 12 块铁片**全部落到对应类型的码放格**；单点约 6.3 s，整轮约 85 s）
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
+X = 操作者视角「右」（类型列 1→4）    +Y = 「远」（货架在 +Y 侧）
+Z = 上
放置货架: 4 列（列 c = 缺陷类型 c） x 3 层（层沿 +Z，层间距 0.06）
  col 1..4 → x = -0.375, -0.125, +0.125, +0.375
  第 L 层 y = 0.31 - (L-1)*0.11（阶梯错开，防止吸盘升降撞上层托盘）
  第 L 层托盘面 z = 0.10 + (L-1)*0.06
置物台（取料源）: (0.30, -0.15)，台面 z = 0.10
供料管（12 块物料开机叠放）: (-0.42, -0.15)
回收位（识别失败的物料）: (-0.10, -0.15)
物料 φ80×6 mm，落到台面/托盘后中心 z = 面高 + 0.003
```

> **M3 与 M2 的流程是反的**：M2 从货架 12 库位取料、放到 4 个码放格堆叠；
> M3 对齐官方 VirtualNext 平台：置物台上料 → 拍照识别 → 吸盘取料 → 放入货架
> 对应类型列。货架每格只放一块（官方摆放要求，不堆叠）。

**物料供给**（`magazine` 段）：12 块物料在世界加载时就全部存在（否则
DetachableJoint 绑不上，见坑 13），开机叠在供料管里；每轮 GOTO 用
`ign service set_pose` 把 `material_NN` 传送到置物台。识别失败没被取走的物料，
下一轮 GOTO 时先送到回收位，保证台面同时只有一块。

龙门（`models/aic_gantry/model.sdf`）：

| 关节 | 轴 | 行程 | 零位含义 |
|---|---|---|---|
| `x_joint` | +X | −0.45 … +0.45 | 0 = 置物台/货架中线 |
| `y_joint` | +Y | −0.24 … **+0.32** | 覆盖供料管(−0.15)、置物台(−0.15)、货架(+0.31) |
| `z_joint` | +Z | **−0.20** … 0 | 0 = 行进高度（吸口下沿 0.239 m）；−0.133 = 取料/放料位 |

末端：法兰座 + 吸盘（吸口下沿 `pad_z0 = 0.239 m`）。相机已**移出龙门**，
改为固定式 `ee_cam`（装在置物台正上方，`camera.mount`），因此 GOTO 停在
置物台 + `goto_offset_y`（0.10 m）的偏移位让吸盘退出光路；识别通过后
HANDLE 才平移到正上方下探。
图像方向：**右 = +X（列增大），上 = −Y**。

![末端相机俯视图](assets/sim_camera_view.png)

> 上面是行进高度下 3 号位点（污渍）的实拍：中间灰色圆盘是铁片，盘内棕色圆斑是
> 「污渍」缺陷，左右两根金色的是夹指，四周深灰是库位地贴。
> 该图由 `ros2 run aic_gazebo_sim sim_snapshot.py` 直接抓取。

## 3. 控制侧实现（gantry_controller.py）

- 订阅 `/control/material_task`，**FIFO 队列**逐条执行（决策是 fire-and-forget，不等回执）。
- 运动 = **协调梯形速度剖面**（`_profile()`），50 Hz 流式下发 setpoint；
  x/y 同起同停，Z 单独按 `z_speed` 走。参数在 `sim_layout.yaml` 的 `gantry:` 段
  （`xy_speed 0.30 m/s`、`z_speed 0.20 m/s`、`pick_z -0.04`、`handle_dwell_s 0.35`）。
- 任务语义（M3）：
  - `GOTO(slot)`：回收上一轮没取走的物料 → 上料 `material_slot` 到置物台 →
    抬到行进高度 → 走到「置物台 + offset」→ 停稳 → 发 `/sim/gantry_arrived(slot_id)`。
    `slot_id` 在这里只是**轮次编号**（1..12），不再对应取料库位。
  - `HANDLE(slot, type)`：平移到置物台正上方 → 下探 → 停留 → 吸盘吸附
    （`attach material_slot`）→ 抬升 → 平移到类型 `type` 列的**最低空层** →
    放入（触底即停）→ 停留 → 断气（detach）→ 抬升 → 发 `/sim/handle_done(slot_id)`。
    类型未知（0）时不下探不吸附，物料留在台面（下一轮送回收位）—— 对应决策侧
    `SkipSlot` 的语义。单次约 12.8 s（M3 实测），比 M2 的 4–5 s 长得多。
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

### 4.2 图像集图版（提交物素材）

省赛技术文档要求「图像集展示」。跑一轮带调试图的仿真后，把标注帧拼成一页图版：

```bash
./scripts/run_sim.sh --vision trad --debug-dir logs/m2_debug
/usr/bin/python3 src/vision/aic_vision_trad/scripts/make_atlas.py \
    --frames logs/m2_debug --out docs/guides/assets/vision_atlas.png
```

![缺陷图像集](assets/vision_atlas.png)

- 每行一类、每行 3 张，绿圈是拟合出的盘面，橙框是判据实际使用的那个暗区。
- 图下三行是**复算**的判据输入（不是抄日志），所以图和数一定对得上：
  `frac` 暗区占盘面积比、`circ` 圆度、`rad` 离盘心距离/盘半径、
  `elong` 最小外接矩长宽比、`drop` 相对盘内亮度基底的落差。
- 四类的典型值（同一批 12 帧）：

| 类型 | frac | circ | elong | rad | drop | core |
|---|---|---|---|---|---|---|
| 1 划痕 | 0.024 | 0.11 | **23.5** | 0.19 | 57 | 0.00 |
| 2 污渍 | 0.097 | 0.92 | 1.0 | 0.22 | 77 | 0.00 |
| 3 缺角 | 0.053–0.064 | 0.30 | 3.0 | **0.77** | 81 | 0.00 |
| 4 孔洞 | 0.021 | **0.98** | 1.0 | 0.32 | **123** | **0.73** |

`core` 是斑内近黑像素（< 盘内亮度 −120）的占比，专门用来认「贴着盘缘、几何量失效」的
孔洞（坑 21）。

判据顺序与间隔见 `defect_classifier.py` 的表头注释；四条判据的最小间隔都在 2 倍以上。

**泛化检验（缺陷几何随机化）**：上面 12 张是固定几何，阈值就是在这批上整定的，
等于同一个样本测了 12 次。把 `sim_layout.yaml` 的 `materials.jitter_seed` 设成非 0
（或给生成器 `--seed N`），生成器按 `(seed, slot_id)` 给每块铁片重抽缺陷的
朝向 / 盘内位置 / 尺寸（0.85–1.20 倍），约束是「盘内缺陷整体留在 0.93r 掩膜内、
缺角压在 0.90–0.96r 盘缘」—— 类型不变，所以视觉真值不受影响。

关掉随机化（`jitter_seed: 0`）时生成物与 M2 **逐字节一致**（`git diff` 为空），
这条本身就是「几何参数化没改动既有行为」的回归证据。

一键多种子回归（每个种子约 85 s，结束时自动把世界恢复成基线）：

```bash
./scripts/verify_sim_robust.sh --frames 1 2 3 4 5   # --frames 顺手存调试图
```

| seed | 决策 | 单轮耗时 | 说明 |
|---|---|---|---|
| 1 | 12/12 OK | 83 s | |
| 2 | 12/12 OK | 83 s | 第一次跑被「改脚本」误杀（坑 22），改回来复跑通过 |
| 3 | 12/12 OK | 82 s | |
| 4 | 12/12 OK | 82 s | |
| 5 | 12/12 OK | 82 s | 存了调试图 |

合计 **60 个位点 0 误判**；seed 2 / 5 的 24 帧离线复算也是 0 错（`make_atlas.py` 的
`--features-csv` 会把每帧的判据输入写出来）。随机几何下 6 帧/类的分布：

| 类型 | frac | circ | rad | elong | drop |
|---|---|---|---|---|---|
| 1 划痕 | 0.024–0.029 | 0.10–0.16 | 0.05–0.22 | 17.6–27.9 | 54–58 |
| 2 污渍 | 0.082–0.116 | 0.93–0.94 | 0.09–0.48 | 1.0 | 76–78 |
| 3 缺角 | **0.033**–0.083 | 0.31–**0.72** | 0.73–0.80 | 1.3–2.7 | 82–84 |
| 4 孔洞 | 0.016–0.022 | 0.96–0.98 | 0.17–0.73 | 1.0–1.1 | 122–124 |

必须记住的四条：

- **划痕 `elong` 下限从 23.4 掉到 17.6**（阈值 12，余量 1.5×）。朝向随机后细条在像素
  网格上更容易「变粗」，`elong` 会低 —— 这就是阈值不能贴着 23 取的原因。
- **缺角 `frac` 下限 0.033**（阈值 0.020，余量 1.65×），比固定几何的 2.6× 收窄。
  但**不能往上调**：圆精修被拒、退回 Hough 整像素圆时 M2 实测 `frac` 会掉到 0.030。
- **缺角 `circ` 上限 0.72 已越过孔洞的 0.70 闸门**，能分开靠的是 `drop ≥ 100`
  （缺角 82–84 / 孔洞 122–124）和 `frac ≤ 0.045` 两道独立闸门 —— 这也正是坑 21
  要再加「近黑核心」的原因：别让单条几何量独自承担分类。
- 精修被拒 1/24，该帧仍判对 → 回退路径有效。

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
18. **Hough 的圆半径会抖 ±9%，凡按半径归一化的量都会跟着抖**（2026-10-20）。
    `HoughCircles` 的输出落在 `dp` 网格上（dp=1.2 → 坐标是 1.2 px 的整数倍），
    同一相机同一高度下逐帧半径在 80.3~87.4 px 之间跳、圆心 `cy` 差 7 px。
    而 `frac` / `rad` 全按盘半径归一化：r 小 9%，0.93r 掩膜就把压在盘缘的缺角切掉大半。
    解法：`find_disc_global()` 末尾统一调 `refine_circle()`（径向边缘扫描 +
    Kasa 拟合），并把结果用「内点占比 ≥0.45、|dr|/r ≤0.12、残差 ≤1.5 px」筛一遍，
    不合格就退回 Hough 圆（日志里会标「精修被拒」）。
    实测组内半径抖动 2.30 px → 0.10~0.49 px，四类特征的帧间散差随之塌到 1e-3 量级。
19. **决策一退出就 `Shutdown`，最后一块料还在夹爪里**（2026-10-20）。
    决策发完最后一个 `HANDLE` 就结束进程，但 HANDLE 是 fire-and-forget：
    控制器 FIFO 里还压着它（夹持 + 搬运 ≈5 s）。旧写法 `OnProcessExit → Shutdown`
    立刻关仿真，结果 **12 次 attach 只有 11 次**、第 12 块铁片始终留在货架上
    （CSV 却显示 12/12 OK，只看结果表完全发现不了）。
    解法：`OnProcessExit → TimerAction(shutdown_grace_s=12) → Shutdown`。
    自检：日志里 `夹持（DetachableJoint attach）` 与 `释放（… detach）` 各应为 **12**。
    做省赛视频时这条尤其致命 —— 少放一块料，成绩就少一分。
20. **仿真可能整条 run loop 卡死**（ogre2 无头 + 12 个 DetachableJoint 长时间跑，
    偶发）：`gzserver` 进程还在、无任何报错，但 `joint_state` 不再更新、相机也不再出帧。
    现场表现是「到位超时 + 视觉超时」，极易误判成控制或视觉挂了。
    控制器现在有关节反馈看门狗：断流 2 s 打一条 error，提示
    「自检 `ign topic -e -t /clock` 看时间戳是否还在走」。
21. **孔洞贴着盘缘时，圆柱侧壁的阴影会和孔口连成一块，几何判据全部失效**（2026-10-20）。
    缺陷几何随机化（`jitter_seed=7`）后跑出来一个：真值孔洞、被判成缺角。
    该连通域 `circ` 从 0.98 掉到 **0.34**、`elong` 1.0→2.1、`rad=0.78`、`drop=101` ——
    几何上就是个缺角，`drop` 又正好夹在缺角 81–86 与孔洞 123 之间，光看亮度也分不开。
    解法：加一条**近黑核心**判据 —— 孔是通透的，斑内 `< 盘内亮度−120` 的像素占
    **0.23–0.75**，而划痕/污渍/缺角实测全是 **0.000**。阈值取 0.05，余量 4 倍以上。
    这条判据放在「缺角」之前，且不依赖 `circ`/`frac`，所以贴边与否都能认。
22. **跑仿真时改 `scripts/*.sh` 会被误诊成「仿真卡死」**（2026-10-20 实测）。
    bash 是边读边执行的：脚本跑到一半被改写，正在跑的实例会读到半截报
    `unexpected EOF while looking for matching '`，随即退出并触发它自己的 `cleanup`
    —— 桥被 `SIGINT` 掉，`ros2 launch` 里其余节点还活着。现场就是
    「`joint_state` 断流 + 相机不再出帧 + `/clock` 采不到」，与坑 20 的 run-loop 卡死
    **肉眼无法区分**。区分办法：在 launch 日志里找
    `signal_handler(SIGINT/SIGTERM)` 与 `process has finished cleanly` —— 有这两行就是桥被杀了，不是仿真死了。
    实测 seed 2 第一次跑只有 1/12，改回脚本后同一 seed 复跑 12/12。
    同理 **不要在同一台机器上并发起第二套仿真** —— 这点已实测（2026-10-20）：
    两套实例共用同一个 ROS domain 时话题名完全重合（`/world/aic_warehouse/*`、
    `/control/*`、`/vision/*`），**真值桩会替对方的位点秒回类型**，一轮里出现
    **3 个 TIMEOUT + 4 个类型错位 + 只夹起 7 块**；更糟的是两边的 `cleanup` 都
    `pkill gzserver`，互相把对方的桥杀掉。表象与「视觉退化」几乎一样。
    - 起跑前检查：`ps -eo pid,etimes,cmd | grep -E 'gzserver|verify_sim'`，看到两个就是串台；
    - 改脚本前先确认没有仿真在跑：`pgrep -af gzserver`。
23. **并行隔离「没生效」其实是两套选了同一个号**（2026-10-20 结案，此前结论是错的）。
    早先记录说「设了 `IGN_PARTITION` 仍能看到同名话题 ⇒ 分区不生效」，**这个判断错了**：
    先用一条空分区做对照（`ign topic -l | wc -l`）：

    ```
    IGN_PARTITION=aic_sim_99   → 0 条      ← 分区过滤是好的，没人用就什么都看不到
    IGN_PARTITION=aic_sim_7    → 53 条
    未设                        → 26 条
    ```

    再查两个 gzserver 的进程环境，直接坐实「撞号」：

    ```
    PID 34515  ROS_DOMAIN_ID=7 IGN_PARTITION=aic_sim_7   ← 我这路
    PID 34850  ROS_DOMAIN_ID=7 IGN_PARTITION=aic_sim_7   ← 另一个会话
    ```

    **坑点在于「固定号」本身**：两个会话各自 `--domain 7` 就必然撞车，与分区机制无关。
    另一个坑：**别用话题条数判断撞没撞** —— 单套实例本身就是 53 条，两套也是 53 条
    （同名话题在 `ign topic -l` 里只列一次）。唯一可靠的判断是读
    `/proc/<pid>/environ` 比对 `IGN_PARTITION`。

    处置改成**自动分配**（`scripts/lib/sim_domain.sh`）：
    扫 `/proc/*/environ` 收已用 domain → 排除登记表 `logs/.sim_domains` 里仍存活的号
    → 再用「该分区 `ign topic -l` 为 0 条」实测确认 → `flock` 持锁登记，防同一瞬间抢同一个号。

    ```bash
    ./scripts/verify_sim_m2.sh --domain auto   # 推荐；三个人同时跑也不会串
    ./scripts/run_sim.sh --domain auto
    ```

    手工 `ign topic` 查某个隔离实例时，要带同样的 `IGN_PARTITION`（脚本会打印分配的号）。

24. **`set_pose` 在 Fortress 有三个连环坑，任何一个都会让你误判成"动态模型不能传送"**
    （2026-10-20，M3 上料功能卡了两轮）。三层依次踩：

    | 层 | 现象 | 真因 | 正解 |
    |---|---|---|---|
    | 服务不存在 | `Service call timed out`（等满 timeout，不是立刻报错） | 世界没加载 `UserCommands` 插件；`ign service -l` 只有 26 条，**没有** set_pose | 世界加 `ignition-gazebo-user-commands-system`（服务变 38 条） |
    | 请求建不出来 | `Unable to create request of type[ignition.msgs.SetPose]` | Fortress 没有 `SetPose` 这个消息；请求类型是 **`ignition.msgs.Pose`** | `--reqtype ignition.msgs.Pose`，字段扁平：`name` + `position` + `orientation`（**不要** `pose{...}` 嵌套，也不要 `entity{...}`） |
    | 返回 true 但不动 | `data: true`，位置一点没变 | 动态模型被 DetachableJoint 焊在龙门上（开机松爪前），约束每帧把它拉回原位 | 先发 `/sim/detach_NN` 再传送；控制器开机已广播 12 路 detach，所以正常流程没问题，**单独用 CLI 调试时最容易踩** |

    ```bash
    # 正确调用（静态/动态模型都一样）
    ign service -s /world/aic_warehouse/set_pose \
      --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
      --req 'name: "material_01" position: {x: 0.3 y: -0.15 z: 0.109} orientation: {w: 1.0}'
    # 回读确认：data: true 之后一定要看 pose/info，别只看返回值
    ```

    另外：`ign topic -e` 打印的是 protobuf 文本格式，**零值字段会被省略**（x=0 那一行
    根本不出现），按固定字段数解析会错位 —— 用 `--reqtype` 之外的任何解析都要按行名匹配。

25. **台面立柱顶面与台面共面 → z-fighting，在相机里是一块"亮方块"**（2026-10-20）。
    置物台是「台面 0.16×0.16 + 立柱 0.10×0.10」，立柱顶原来正好在 z=0.09，与台面
    底面共面。俯视相机渲染出来台面中央一块明显更亮的方块，把盘周围的暗背景切掉一半，
    真视觉的「内亮外暗环」判据直接找不到盘（表现为全 TIMEOUT，而非判错）。
    修法：立柱下移 2 mm 藏到台面下方（`gen_world.py` 里 `z=0.048`）。
    **同类问题**：任何"装饰件与工作面共面"都会这样，改完几何先抓一张图看。

26. **决策的视觉窗口被上一位点的 HANDLE 吃满**（2026-10-20，M3 实测）。
    决策是 fire-and-forget：`GOTO(N)` 下发时，控制队列里还压着 `HANDLE(N-1)`。
    M3 的 HANDLE 含「取料 + 搬到货架 + 放入」，实测 **12.8 s**，再加 GOTO 2.5 s，
    从 GOTO 下达到视觉结果回来要 **15.5 s**。`vision_timeout_s=15.0` 正好切在边界，
    12 位点稳定丢 2 个（第 2、8 位），且日志里是"视觉超时未返回"，很像视觉退化。
    证据（同一轮）：

    ```
    GOTO slot=3 到达      1551.12   ← 决策从这一刻开始计时
    material 04 吸附      1554.63   ← 控制器还在做上一位点的 HANDLE
    material 04 释放      1561.85
    slot 4 处理完成       1563.93
    轮次 3 到位置物台      1566.59   ← 15.47 s，超了 15.0
    ```

    处置：仿真 side 的 `vision_timeout_s` 提到 **30.0**（`sim_decision.launch.py`），
    按「最坏队列时延」给，不按 HANDLE 典型值估。真机 HANDLE 只有夹爪开合，
    参数应以真机节拍重定；但**判断方法一样**：量 GOTO→视觉结果的实际分布，取 p99 再留余量。



27. **关停瞬间视觉节点报 `RuntimeError: Unable to convert call argument to Python object`**
    （2026-10-20）。整轮已经 12/12，SIGINT 之后 launch 打印
    `process has died [exit code 1]`，看着像真故障。真因是 rclpy 在上下文销毁过程中
    仍从已失效的订阅取消息（图像话题 20 Hz，正好卡在关停窗口）。
    修法：`vision_node.py` 的 `main()` 捕获 `RuntimeError`，仅在 `rclpy.ok()` 已为假
    （即确属关停）时吞掉并打一行「正常收敛」；运行期同类异常仍会抛出，不掩盖真问题。
    判断是否同类：看 traceback 是否紧跟在 `signal_handler(SIGINT/SIGTERM)` 之后。

28. **层位占用与「已取走」标记必须在生成步骤时提交，不能等执行到 detach**
    （2026-10-20，一次"看起来更严谨"的重构引入的回归）。
    决策是 fire-and-forget：`DispatchMaterialTask action=HANDLE` 之后立刻
    `RecordSlotResult result=OK`，**不等 `handle_done`** 就发下一轮 GOTO。
    所以控制器队列里可能同时压着 `GOTO(N) / HANDLE(N) / GOTO(N+1)`。

    把 `self._placed[mtype] = level` 和 `self._picked = True` 挪到 detach 回调里以后：
    - 同类型的下一块 HANDLE 生成时读到旧的 `_placed`，两块抢同一层；
    - `GOTO(N+1)` 生成时 `_picked` 仍是 False，误判「上轮物料没被取走」，
      把已安排取走的料送进回收位（实测 `material_04` 落到 z=0.023）。

    自验会直接抓到（C3/C4/C5 三条 FAIL，x 偏 −475 mm）。判读口诀：
    **凡是"下一轮要用的状态"，都必须在收到任务时就写，不能等动作执行完**。
    真正的执行结果只用于统计/判分（看板走 `attach_XX`/`detach_XX` 是另一条路，不受影响）。

29. **`pkill -f gzserver` 在 Fortress 上是空操作；清理代码的 `&&` 短路会漏杀子进程**
    （2026-10-21，一次「PASS 之后仍有两个仿真在跑」）。

    两件事叠在一起：

    - 进程名不是 `gzserver`。Fortress 起的是 `ign gazebo -s -r --headless-rendering ...`，
      `gzserver` 是 classic 的名字。`pkill -f gzserver` 匹配 0 个进程，**退出码非 0 但
      脚本里带 `2>/dev/null` 就完全看不出来**，于是「清场」等于没做。
    - `verify_sim_m2.sh` 的收尾写成
      `pgrep -P "$LAUNCH_PID" && kill_tree ...`。launch 收到 SIGINT 后往往 1–2 s 内先退出，
      子进程被 reparent 到 1，`pgrep -P` 再也查不到 → `kill_tree` 永不执行 →
      gzserver 变成孤儿继续跑。

    症状极具迷惑性：单看某一轮，`PASS：决策 12/12`，落位 0.0 mm，一切正常。
    但接着跑 PLC 接管时，**两个物理世界订阅同一批 `/clock`、`/aic/joint/*/cmd_pos`，
    互相抢关节指令**：RTF 掉到约 1/10，日志出现 `到位超时 8.0 s … 按到位继续`，
    单轮从 10 s 涨到 36 s，落位偏移几十毫米——而 PLC 侧 `SHELF_BITS` 仍是 12 个 1。

    修法（已落地）：
    - 新增 `scripts/lib/sim_cleanup.sh`：`sim_pids / sim_kill_stale / sim_count_gazebo`，
      按真实进程名清，并排除调用者自身（`pkill -f` 会匹配到自己的命令行，
      我在 shell 里直接 `pkill -f "ros2 launch"` 就把自己那条命令杀了）；
    - 收尾顺序改成**先快照子进程再发信号**，最后再用 `sim_kill_stale` 兜一遍；
    - `test_plc_{one,full}.sh` 的 trap 也走同一套。

    一分钟自检：`ps -eo args= | grep -c "[i]gn gazebo"`，跑之前必须是 0，跑起来必须是 1。
    （不要用 `pgrep -f "ign gazebo"`：它会匹配到调用者自己的命令行，永远多 1–2 个。）
    验收时若某轮突然 30 s+，先数实例，再看代码。

## 6. 已验证 / 未做

**一键自验**：`./scripts/verify_sim_m2.sh`（M3 一轮约 200 s）。起真视觉跑一整轮，等日志里第 12 次
`detach` 出现（**不用固定 sleep**，否则最后一块还在飞行中就被 kill），抓 `pose/info`，
再调 `scripts/check_sim_m2.py` 断言三组：

| 组 | 断言 |
|---|---|
| A 决策结果 | 12 行全 `OK`；每个位点按 `slot_id` 查 `materials.types`，类型逐位一致 |
| B 夹持闭环 | 日志 `吸附` 与 `释放` 各 12 条（只数 attach 会漏掉坑 19） |
| C 物理落位 | 12 块物料按轮次编号查真值类型 → 落在对应类型列；x 偏列心 ≤4 mm；y 与该层阶梯线 ≤4 mm；z 在对应层托盘高度区间 |

`--vision gt` 可切真值桩，用来把「视觉判错」与「控制/物理问题」分开。
负向自测：拿 `drop_rate:=0.6` 的结果喂进去，A2/A3/B1/B2 全报 FAIL（脚本本身也验过）。

**缺陷几何随机化回归**：`./scripts/verify_sim_robust.sh --frames 1 2 3 4 5`
（5 个种子 × 12 位点，约 7 min）。零假设是「阈值只在那 12 张固定图上成立」；
实测 **60 位点 0 误判**，判据输入分布与余量见 §4.2 的表。

**已验证（2026-10-20，M2）**

- **真视觉 + 完整搬运，单轮 12 位点：OK 12 / TIMEOUT 0，四类全对**
  （划痕 3 / 污渍 3 / 缺角 3 / 孔洞 3，与 `materials.types` 逐位一致），平均单点 6.3 s，
  整轮约 85 s（另加 `shutdown_grace_s` 12 s 收尾）。
- **落位核对**：同一次闭环（真视觉 → 决策 → 抓取 → 码放）跑完后，
  从 `/world/aic_warehouse/pose/info` 读 12 块铁片的最终世界坐标：

  | 类型 | 格心 x | 落位铁片 | 实测 x | 实测 z |
  |---|---|---|---|---|
  | 1 划痕 | −0.36 | slot 1 / 6 / 10 | −0.3592 / −0.3592 / −0.3592 | 0.103 / 0.109 / 0.115 |
  | 2 污渍 | −0.12 | slot 3 / 5 / 11 | −0.1192 / −0.1192 / −0.1192 | 0.103 / 0.109 / 0.115 |
  | 3 缺角 | +0.12 | slot 2 / 8 / 9 | 0.1208 / 0.1208 / 0.1209 | 0.103 / 0.109 / 0.115 |
  | 4 孔洞 | +0.36 | slot 4 / 7 / 12 | 0.3608 / 0.3608 / 0.3608 | 0.103 / 0.109 / 0.115 |

  x 与格心差 0.8 mm（夹指半开 36 mm、片半径 40 mm，落点受夹指间隙约束）；
  y 全部 0.2987（比格心靠里 1.3 mm）；z 每叠一块正好 +6 mm（片厚），
  说明「叠放」是物理结果而非穿模。
- **夹持 / 释放 12 对全部成对出现**（日志各 12 条），8 次「已触底」，**0 次到位超时**。
  这一条是坑 19 的回归检查项：只数 attach 不够，必须 attach 与 detach 都等于位点数。
- 单点抓放探针（只跑 slot 1）：铁片从 (−0.375, −0.1667) 精确落到 (−0.360, 0.300)，
  其余 11 块原地不动。

**M1 遗留（仍有效）**

- 真值桩单轮 12 位点闭环：OK 12 / TIMEOUT 0，平均单点 3.03 s。
- 故障注入：`drop_rate:=0.6` → 巡视跑完，`FAILURE` 不出现。
  **M2 节拍下已复测**（2026-10-20）：OK 4 / TIMEOUT 8 / 夹持 4，平均单点 9.1 s、
  整轮约 125 s。结论与 M1 一致，只是失败位点从「等 5.4 s」变成「空等 12 s」——
  `vision_timeout_s` 调大的代价是明确的，真机把 HANDLE 压短后应同步收小。
- 到位精度：反馈判定，稳态误差 0.0–1.2 mm；无头 EGL 出图；
  `ros2 launch` 在决策退出后自动关停（M2 起带 `shutdown_grace_s` 宽限）。
- **圆精修**（坑 18）：组内半径抖动 2.30 px → 0.10~0.49 px，
  四类特征帧间散差塌到 1e-3 量级；精修被拒时日志会标「精修被拒」。

**未做**

1. **GUI 交互**：现在 `gui:=true` 只起观察窗口，没有做模型拖动/摆位工具。
2. **视觉从仿真迁移到实拍**：`aic_vision_trad` 的阈值是在渲染图上标的
   （背景单一、无光照变化、缺陷是几何体）。实拍要重标：先固定相机→盘面距离，
   再按「盘半径像素」归一化（几何量现在已经是相对量，换尺度只需改 `disc_rmin_frac`）；
   另外缺角的 `frac` 依赖盘半径估计精度（坑 18）：`refine_circle()` 的
   `refine_grad_min`（默认 6 级灰阶）是按渲染图的边缘对比度定的，实拍要按新图重调，
   否则精修会频繁被拒、退回 ±9% 抖动的 Hough 圆。
3. **多格位扩展**：现在每个类型只有一个格心（3 块叠放）。若真机是「一个料框一条，
   框内并排」，把 `place_zones` 改成 `xs` 二维（每型多个落点）即可，控制器只读它。
4. **夹爪几何**：现在夹指是两根静止的黄色方块，靠 `DetachableJoint` 做「吸附」，
   没有真正的开合关节与夹持力。要更真实可换 `ros2_control` + 两指 prismatic。
