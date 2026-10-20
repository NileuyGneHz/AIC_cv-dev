# AIC 仓储仿真使用手册（aic_gazebo_sim）

> 版本：2026-10-20 · 适用于 **M3**（置物台上料 → 固定相机识别 → 吸盘取料 → 货架 4 列 × 3 层）
> 定位：这是**操作手册**——装环境、构建、跑仿真、自验、配置、排障，按需查节。
> 原理与全部 27 个坑的来龙去脉见 `simulation_guide.md`；接口契约细节见
> `decision_control_interface.md` / `decision_vision_interface.md`。
> **要录汇报/参赛视频**：直接看 `obs_recording_playbook.md`（录制前清单、OBS 参数、逐段旁白）。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Ignition Gazebo 6.18（Fortress）+ ros_gz_bridge

## 1. 系统一句话

对齐官方 VirtualNext 平台的节拍：**置物台上料 → 固定相机拍照识别 → 三轴龙门吸盘取料 → 放入货架对应类型列的最低空层（每格一块，不堆叠）**。
一轮比赛 = 12 个上料轮次：每个轮次把 `material_NN` 从供料管传送到置物台、拍照判型，再按判型放进对应列。

```text
决策 bt_runner (C++)   —— 12 个"位点" = 12 个上料轮次（slot_id 只是轮次编号）
   │ /control/material_task (aic_msgs/MaterialTask)
   ▼
gantry_controller.py ── 上料：ign service set_pose 把 material_NN 从供料管传到置物台
   │                 ── 运动：/aic/joint/{x,y,z}/cmd_pos ─► ros_gz_bridge ─► Gazebo
   │                 ── 取/放：/sim/attach_NN / /sim/detach_NN（DetachableJoint 模拟吸盘）
   │  ▲ ── /world/aic_warehouse/model/aic_gantry/joint_state（闭环到位判定）
   ├──► /sim/gantry_arrived(UInt8 slot) ──► 视觉（真值桩或 aic_vision_trad，固定 ee_cam 拍置物台）
   └──► /sim/handle_done(UInt8 slot)
视觉 ──► /vision/material_type (String "1".."4") ──► 决策
```

## 2. 前置条件

| 依赖 | 用途 | 检查命令 |
|---|---|---|
| ROS 2 Humble | 全部节点 | `ls /opt/ros/humble/setup.bash` |
| Ignition Fortress（含 gz-sim6） | 仿真世界 | `ign gazebo --version` |
| ros_gz_bridge / ros_gz_sim | ROS↔GZ 桥与启动 | `ros2 pkg list \| grep ros_gz` |
| OpenCV + cv_bridge（python3-opencv） | 真视觉 | `python3 -c "import cv2"` |
| colcon + ament_cmake | 构建 | `which colcon` |
| 工作区 | 本项目 | `~/AIC_cv-dev`（真正的工作树；`/media/...` 下是从 GitHub clone 的旧副本，别在那儿改） |

## 3. 构建与首次自检

```bash
cd ~/AIC_cv-dev
./scripts/build.sh                     # 增量构建全部包
./scripts/build.sh --clean             # 全量重建（构建产物异常时用）
```

构建要点（脚本已内置处理，这里说明为什么）：

- 本机 `/tmp` 是 10 MB 小 tmpfs，gcc 中间汇编会写满 → 编译带 `-pipe`（脚本已设）。
- colcon 需要系统 python3.10，脚本会把 `/usr/bin` 提到 PATH 最前。
- `--packages-select aic_behavior_tree` 可只编单个包（改决策侧时用）。

**构建后 30 秒体检**：`./scripts/run_sim.sh` 跑一轮，结果 CSV 应为 12 行 OK。
若世界能起、时钟在走但什么都不动 → 见排障表第 1 条。

## 4. 快速开始（三条命令）

```bash
./scripts/build.sh                 # 1. 构建
./scripts/run_sim.sh               # 2. 无头跑一轮（真值桩视觉，约 40 s）
./scripts/verify_sim_m2.sh         # 3. 完整自验（真视觉 + 抓放 + 落位断言，约 100 s）
```

看到 `PASS` 即整条链路（视觉 → 决策 → 抓取 → 码放 → 落位）全部正常。

### 4.1 确认工作成果：按验证目标选命令

| 你想确认的成果 | 命令 | 通过标准 | 耗时 |
|---|---|---|---|
| 链路通不通（决策→控制→仿真闭环） | `./scripts/run_sim.sh` | 结果 CSV 12 行全 OK | ≈110 s（gt 桩） |
| **端到端完整成果**（真视觉识别 + 物理抓放 + 落位） | `./scripts/verify_sim_m2.sh` | **PASS**（三组断言全过） | ≈160 s |
| 四类缺陷识别都对 | 同上 + 看 CSV 类型列 vs 真值 | 逐位一致（按 slot_id 对） | ≈160 s |
| 识别对随机几何也稳（泛化） | `./scripts/verify_sim_robust.sh` | 汇总表 0 误判 | ≈7 min |
| TIMEOUT 兜底不崩 | `./scripts/run_sim.sh --drop-rate 0.6` | 巡视走完 12 位点、无 FAILURE | ≈120 s（M3 未重测） |
| 边看画面边确认（判分绿/红 + 抓取过程 + 自由机位） | `ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad` | 浏览器 `127.0.0.1:8080`：12 格全绿、态势图 12 块落对应层位 | ≈150 s |
| 识别失败现场证据 | `./scripts/run_sim.sh --vision trad --debug-dir logs/dbg` | `logs/dbg/vision_fail_*.png`（无 = 没判错过） | ≈150 s |

## 5. 运行模式：run_sim.sh 全参数

入口：`./scripts/run_sim.sh [选项]`。产物固定在 `logs/sim_launch_<stamp>.log` 与结果 CSV。

| 选项 | 默认 | 作用 |
|---|---|---|
| `--gui` | 关 | 带 Gazebo GUI 观察窗口（录像/演示用；无头跑也能出相机图，EGL 渲染） |
| `--loop` | 单轮 | 常驻多轮（`once:=false` + `auto_restart:=true`，决策一轮结束自动重开），Ctrl-C 结束。**注意**：M3 的物料是「按库位号」从供料管取的，第二轮会把已放进货架的物料重新传送回置物台（等于把布局打乱重来）——真视觉结果不可作数。**只用于 gt 桩调决策 / 看板连拍演示，录像必须单轮。** 看板会在新一轮开始时自动清盘 |
| `--vision gt\|trad` | `gt` | 视觉后端：`gt`=真值桩（按布局表秒回类型）；`trad`=真视觉（读相机图识别） |
| `--drop-rate 0.6` | 0.0 | 注入 60% 位点「识别失败」（视觉不回结果 → 决策记 TIMEOUT，巡视继续） |
| `--invalid-rate 0.3` | 0.0 | 注入 30% 位点返回非法值 `"9"`（决策丢弃并计数） |
| `--debug-dir logs/xxx` | 关 | 真视觉把每帧标注图存到该目录（判错的现场证据 + 图版素材） |
| `--results logs/my.csv` | 自动带时间戳 | 指定结果 CSV 路径 |
| `--domain auto` | 不隔离 | 同机并行隔离：自动挑空闲 ROS domain + GZ 分区（**同机跑第二套仿真必用**，见 §11） |

等价的 launch 直调（脚本本质是它的包装）：

```bash
ros2 launch aic_gazebo_sim sim_decision.launch.py gui:=true once:=false drop_rate:=0.15
```

常用组合：

```bash
./scripts/run_sim.sh --vision trad                                # 端到端（真视觉）
./scripts/run_sim.sh --vision trad --debug-dir logs/vision_debug  # 存标注图
./scripts/run_sim.sh --drop-rate 0.6                              # 验证 TIMEOUT 兜底
./scripts/run_sim.sh --gui --loop                                 # 演示：常驻 + 画面
```

单点抓图（场景自检 / 视觉取数）：

```bash
ros2 run aic_gazebo_sim sim_snapshot.py --ros-args -p out_dir:=logs/snapshots -p count:=3
```

## 6. 场景配置（唯一数据源 sim_layout.yaml）

文件：`src/simulation/aic_gazebo_sim/config/sim_layout.yaml`。
**改任何几何/布局/类型后必须：重跑 `scripts/gen_world.py` → 重启仿真。** 世界 SDF 是生成物，不要手改。

```bash
cd ~/AIC_cv-dev
/usr/bin/python3 src/simulation/aic_gazebo_sim/scripts/gen_world.py
```

### 6.1 改库位类型分配（最常改的一行）

```yaml
materials:
  types: [1, 3, 2, 4, 2, 1, 4, 3, 3, 1, 2, 4]   # slot 1..12 的缺陷类型
  jitter_seed: 0                                  # 0=固定几何；>0=随机化（见 §7）
```

类型编号：**1=划痕 2=污渍 3=缺角 4=孔洞**。改完重跑 gen_world.py，视觉真值随之变化。

### 6.2 各段速览

| 段 | 关键字段 | 含义 |
|---|---|---|
| `rack` | `cols:4 rows:3 pitch_x:0.25 level_pitch_z:0.06 y_front:0.31 level_step_y:0.11` | 放置货架：4 类型列 × 3 层，每格一块；层位沿 +Y 阶梯错开（防吸盘升降撞上层托盘） |
| `plate` | `diameter:0.08 thickness:0.006 mass:0.05` | 圆形物料 φ80×6 mm |
| `materials` | `types` / `jitter_seed` | 类型真值表（第 k 轮 = `types[k-1]`）/ 几何随机化种子 |
| `gantry` | `xy_speed:0.30 z_speed:0.20 travel_z:0.02 pad_z0:0.239` + 闭环阈值 | 运动剖面、行进高度、吸盘基准与到位判据 |
| `camera` | `mount:[x y z r p y]` `width:640 height:480` | **固定式**末端相机（装在置物台正上方；M3 起不再挂在龙门上） |
| `staging` | `x:0.30 y:-0.15 z:0.10 goto_offset_y:0.10` | 置物台（取料源）+ GOTO 停靠偏移（让吸盘退出光路） |
| `magazine` | `x:-0.42 floor_z:0.085 tube_h:0.09` | 供料管：12 块物料开机全部叠在这里，按轮次传送 |
| `recycle` | `x:-0.10 y:-0.15 z:0.02` | 回收位：识别失败没被取走的物料送到这，保证台面同时只有一块 |
| `scene_cam` | `pose / fov / width:960 height:540 / update_rate:15` | 自由摄像机（看板全景主画面 `/scene/image`） |

坐标系：+X 右（类型列 1→4）、+Y 远（货架在 +Y 侧）、+Z 上；相机画面 **右=+X、上=−Y**。

### 6.3 决策侧参数

`src/decision/aic_behavior_tree/config/aic_decision_params.yaml`。
最敏感的一个：`vision_timeout_s`——它必须覆盖「上一个位点的 HANDLE + 本次 GOTO + 视觉延迟」。
决策是 fire-and-forget：GOTO(N) 下发时，控制队列里还压着 HANDLE(N−1)。M3 实测该队列时延
**12.8 s（HANDLE）+ 2.5 s（GOTO）= 15.5 s**，所以仿真 launch 默认给到 **30.0**
（`sim_decision.launch.py` 的 `vision_timeout_s`，会覆盖 yaml 里的值）。
偏小的后果不只是「慢」，更是**迟到的识别结果被下一个位点收下（类型错位，结果表仍显示 OK）**。
真机 HANDLE 只有夹爪一开一合，实际参数应以真机节拍重定。

## 7. 缺陷几何随机化（阈值泛化检验）

`jitter_seed: 0` 时生成物与基线**逐字节一致**（回归证据）；设为 N 后按 `(N, slot_id)`
给每块铁片重抽朝向 / 盘内位置 / 尺寸（0.85–1.20×），**类型不变**（视觉真值不受影响）。

一键多种子回归（每个种子约 150–170 s，结束自动恢复基线世界）：

```bash
./scripts/verify_sim_robust.sh                 # 默认 seed 1 2 3 4
./scripts/verify_sim_robust.sh 5 6 7           # 指定种子
./scripts/verify_sim_robust.sh --frames 5      # 顺手存调试图到 frames_seed_5/
./scripts/verify_sim_robust.sh --vision gt 1   # 真值桩（排除视觉，只看随机几何下的物理）
```

产物：`logs/robust_<stamp>/seed_<N>.{csv,log}`，末尾打印汇总表。
当前成绩：**5 种子 × 12 位点 = 60 位点 0 误判**。

## 8. 自验与数据工具箱

| 命令 | 干什么 | 耗时 |
|---|---|---|
| `./scripts/verify_sim_m2.sh` | **主自验**：真视觉整轮 + 三组断言（见下） | ≈160 s |
| `./scripts/verify_sim_m2.sh --vision gt` | 真值桩版（把「视觉判错」与「控制/物理问题」分开） | ≈110 s |
| `./scripts/verify_sim_m2.sh --timeout 300` | 机器慢时放宽等待上限 | — |
| `./scripts/verify_sim_robust.sh` | 多种子随机几何回归（§7） | ≈7 min |
| `./scripts/verify_decision_loop.sh` | 纯决策闭环回归（A/B 两场景，不动仿真） | 快 |

`verify_sim_m2.sh` 断言的三组（`scripts/check_sim_m2.py` 执行，全过打 PASS）：

| 组 | 断言 |
|---|---|
| A 决策结果 | 12 行全 OK；按 `slot_id` 查 `materials.types` 类型逐位一致 |
| B 夹持闭环 | 日志「夹持」「释放」各 12 条（attach/detach 必须成对，只数 attach 会漏） |
| C 物理落位 | 12 块铁片落在对应类型格心 ±4 mm、z 在台面高度（叠放 z +6 mm/块） |

**全过程抓帧**（做汇报/技术文档配图，或与录像并行留素材）：

```bash
# 单独起（与仿真并行即可，只订阅不发布；跑完最后一轮自动退出）
ros2 run aic_gazebo_sim capture_storyboard.py \
    --ros-args -p out_dir:=logs/storyboard -p detail_slots:="1,2,6,12"

# 拼成手册用的 3x4 图版
/usr/bin/python3 docs/guides/gen_storyboard.py logs/storyboard docs/guides/assets/storyboard.png
```

产物：`<out_dir>/NN_<轮次>_<阶段>.png` + `storyboard.txt`（时间索引）。
机位默认正俯视（`0,0.02,1.55,0,1.5708,1.5708`，能看到供料管 / 置物台 / 货架全貌），
用 `-p scene_pose:="x,y,z,roll,pitch,yaw"` 可换机位。

离线工具：

```bash
# 视觉离线回归（对已存样本打分，不起仿真）
/usr/bin/python3 src/vision/aic_vision_trad/scripts/bench_offline.py logs/vision_ds/
# 缺陷图像集图版（技术文档「图像集展示」素材；每张复算判据再标注）
/usr/bin/python3 src/vision/aic_vision_trad/scripts/make_atlas.py \
    --frames logs/m2_debug --out docs/guides/assets/vision_atlas.png
```

## 9. 产物与日志位置

| 文件 | 内容 |
|---|---|
| `logs/sim_results_<stamp>.csv` | 决策结果表：每行一位点（slot_id、类型、OK/TIMEOUT） |
| `logs/sim_launch_<stamp>.log` | ros2 launch 全量日志（排障主战场） |
| `logs/verify_m2_<stamp>.{csv,log}` + `logs/verify_m2_pose_<stamp>.txt` | 自验三件套（pose 文本供落位核对） |
| `logs/<debug-dir>/vision_*.png` | 真视觉标注帧；`vision_fail_*` 是失败帧（判错现场证据） |
| `logs/robust_<stamp>/` | 多种子回归逐种子产物 |
| `logs/storyboard/` | 全过程分镜帧（`NN_轮次_阶段.png`）+ `storyboard.txt` 时间索引 |

**结果表判读两条铁律**：

1. 表是**巡视顺序**，`materials.types` 是**库位顺序**——对答案必须用行里的 `slot_id` 查表。
2. CSV 12/12 OK 不等于物理成功：坑 19 里出现过「CSV 全 OK 但第 12 块还留在货架上」。
   物理是否成功看 B 组（attach/detach 各 12）与 C 组（落位），或直接跑 verify 脚本。

## 10. 话题与接口速查

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/control/material_task` | `aic_msgs/MaterialTask` | 决策→控制 | GOTO/HANDLE/HOME 指令（含 slot_id 与视觉判型） |
| `/sim/gantry_arrived` | `std_msgs/UInt8` | 控制→视觉 | 到位（slot_id），触发识别 |
| `/sim/handle_done` | `std_msgs/UInt8` | 控制→决策 | HANDLE 完成 |
| `/vision/material_type` | `std_msgs/String` | 视觉→决策 | `"1".."4"`；识别失败**不发**（不发特殊值） |
| `/control/ready` | `std_msgs/Bool` | 控制→决策 | 控制侧就绪握手（决策 `control_wait_ready=true` 时等它，防首条 GOTO 丢） |
| `/camera/image` | `sensor_msgs/Image` | GZ→ROS | 末端俯视相机（best_effort） |
| `/world/aic_warehouse/model/aic_gantry/joint_state` | `sensor_msgs/JointState` | GZ→ROS | 关节反馈（闭环到位判定用） |
| `/sim/attach_XX` / `/sim/detach_XX` | `std_msgs/Empty` | 控制→GZ | 第 XX 块铁片夹/放（经桥转 ignition.msgs） |
| `/world/aic_warehouse/pose/info` | ignition.msgs.Pose_V | GZ→ROS | 全模型真值位姿（落位核对数据源） |

## 11. 故障排查速查表

> 完整机理见 `simulation_guide.md` §5 的 23 个坑；这里按「症状 → 先查什么」索引。

| 症状 | 先查 | 要点 |
|---|---|---|
| 世界能动时钟在走，但什么都不动（自由落体都不掉） | 坑 1 | Physics system 未加载；世界 SDF 必须显式挂 physics 插件 + `IGN_PHYSICS_ENGINE_PATH`。自检：放个球看 2 s 后掉不掉 |
| 第 1 个位点必然 TIMEOUT | 坑 10 | 决策 fire-and-forget，control 订阅未建消息蒸发；确认 `control_wait_ready=true` 且 `/control/ready` 在发 |
| 相机画面偏一整格、视觉全判不出 | 坑 11 | 不是渲染滞后，是机台没到位：确认到位判定走闭环（joint_state 双阈值），增益 p=1600/d=200 |
| 视觉结果迟到且**类型错位**、结果表却显示 OK | 坑 12 | `vision_timeout_s` 太小（须覆盖 HANDLE+GOTO+延迟；现默认 12.0） |
| 每段 Z 运动都「到位超时 8 s」，指令一下发就动不了 | 坑 13 | DetachableJoint 开机自吸附：确认开机松爪广播在跑，且 `/control/ready` 等松爪跑完再发。自检：静置 z≈0、铁片 z≈0.103 |
| 铁片整批停在 z=0.013（或穿到地面） | 坑 14 | 碰撞体与可视体错位：visual 带 pose 的，collision 必须给同一份 pose |
| 放料每块白等 8 s，还连锁 TIMEOUT | 坑 15 | 叠放必高 6 mm：放料用独立 `place` 步「触底判据」（日志打「已触底」） |
| 到位首帧画面还是运动中的 → 判错 | 坑 16 | 图像订阅 depth 必须 1 + `settle_skip_s` 跳过停稳期旧帧 |
| 缺角偶发判成污渍 / 特征帧间乱跳 | 坑 17/18 | 圆精修是否被拒（日志标「精修被拒」）；`frac` 阈值不能贴实测值 |
| 贴边孔洞被判成缺角 | 坑 21 | 近黑核心判据（`core ≥ 0.05`）是否在位、放在缺角之前 |
| joint_state 断流 + 相机不出帧 | 坑 20/22 | 先区分两种死法：launch 日志找 `signal_handler(SIGINT)` + `process has finished cleanly`——有就是**桥被杀**（多半是有人改了正在跑的脚本），没有才是 run-loop 卡死 |
| 两套仿真互相污染：类型整片错位 + 莫名 TIMEOUT | 坑 22/23 | **同机并发必须 `--domain auto`**。起跑前 `ps -eo pid,etimes,cmd \| grep -E 'gzserver|verify_sim'`；判断撞号唯一可靠方法是比 `/proc/<pid>/environ` 的 `IGN_PARTITION`（话题条数不可靠） |
| 改脚本/改代码时正跑着的实例突然死 | 坑 22 | bash 边读边执行；**改 scripts/*.sh 前先 `pgrep -af gzserver`** 确认没实例在跑 |
| 节点启动即崩（InvalidParameterTypeException） | 坑 8 | `declare_parameter` 钉死类型；命令行传参走 `declare_number()`（dynamic_typing） |
| 容器/CI 里日志写 `/.ignition` | 坑 9 | 显式给 `HOME`（launch 已兜底） |

**并发隔离标准用法**（同机多会话 / 多人同跑）：

```bash
./scripts/run_sim.sh --domain auto
./scripts/verify_sim_m2.sh --domain auto      # 推荐：三个人同时跑也不会串
```

## 12. 判分看板与录像工作流（省赛视频用）

### 12.1 判分看板（aic_dashboard）

对标官方平台「放置正确变绿 / 错误变红、自动计分」的过审硬指标 3、4：

```bash
ros2 launch aic_dashboard dashboard.launch.py                     # 一条命令起齐仿真+决策+看板
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad
```

浏览器打开 `http://127.0.0.1:8080`（只绑本机），单页三栏：

- **全景 · 自由摄像机**（主画面）：世界上方的 `scene_cam`，页面按钮可切 4 个预设机位
  （全景俯视 / 正俯视 / 货架特写 / 码放特写）+ 六向平移。实现走
  `ign service set_pose`（`POST /api/cam`），不改控制侧。
- **末端相机**（画中画）：夹爪下的识别视图，叠加当前位点 / 识别 / 比分。
- **俯视态势图**：12 块铁片实时位置（圆点颜色 = 缺陷类型；夹起后跟随龙门移动，
  放下后落在对应类型码放格）+ 龙门十字 + 码放格。数据来源：
  `/sim/attach_XX`/`detach_XX`（夹/放事件）+ `joint_state`（龙门实时位置），零新增桥。
- 右侧：12 库位状态格（绿=正确 / 红=错误或超时 / 琥珀=巡视中）+ 计分板 + 事件流水
  （含「铁片 N 已夹起 / 已放入码放格」的过程记录）。

判分口径：「放到正确列」⟺「视觉判型 == 该库位真值类型」，真值只取 `materials.types` 一份事实。
超时判定：决策开始巡视下一个库位而上一库位仍无识别结果 → 记 TIMEOUT。

- 多轮连拍：已落判位点再次被巡视 = 新一轮，看板自动清盘（也可 `curl -X POST
  http://127.0.0.1:8080/api/reset` 手动清）。
- 看板是**纯旁路**（只订阅不发布）：关掉它链路照常跑；它挂了也不影响仿真。
- 与 `run_sim.sh` 并存：看板单独起（`ros2 run aic_dashboard score_node.py`）也能给
  run_sim.sh 的实例判分，但同机多实例时注意 `--domain auto` 会让它看不到对方——
  隔离实例要连看板一起隔离（在同一 domain 环境里起）。

### 12.2 录像硬要求与流程

硬要求回顾：一镜到底、无剪切、无人为干预、≥1080P、清晰反映比赛结果（详见
`docs/competition/platform_and_video_checklist.md`）。

1. **起跑前检查**：`pgrep -af gzserver` 确认无残留实例；隔离场景用 `--domain auto`。
2. **一键带看板跑**：`ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad gui:=true`
   （真视觉 12/12 单轮约 145 s + `shutdown_grace_s` 12 s 收尾）。
3. **录制**：OBS 录浏览器看板窗口（或加仿真 GUI 窗口双源合成）。
4. **结果定格**：结果 CSV + `pose/info` 落位核对（`logs/verify_m2_pose_*.txt` 的读法见
   `check_sim_m2.py`）作为「清晰反映比赛结果」的证据帧。
5. **排障红线**：录制全程不得改任何脚本；少放一块料（坑 19）按作弊级事故对待——
   录完先数日志里「夹持」「释放」是否各 12。

## 13. 已验证基线（2026-10-20，M3）

| 项 | 结果 |
|---|---|
| 真视觉 + 完整搬运 | **OK 12 / TIMEOUT 0**，四类全对，整轮 **≈145 s**（12 轮，含上料与搬运） |
| 落位 | 12 块落对应类型列、按 L1→L3 逐层填满；x/y 落位误差 **0.0 mm** |
| 夹持闭环 | attach 12 / detach 12 成对，0 次到位超时 |
| 视觉窗口 | `vision_timeout_s=30.0`（覆盖最坏队列时延 HANDLE 12.8 s + GOTO 2.5 s） |
| 抓帧分镜 | 单轮 23 帧，12 个阶段全覆盖（`logs/storyboard2/storyboard.txt`） |
| 看板终态 | 12 ok / 12 placed / 0 recycled |
| 三组断言 | `check_sim_m2.py` PASS（决策结果 / 夹持闭环 / 物理落位） |

> M2 时期的历史数据（真值桩单点 3.03 s、drop_rate=0.6 的 4/8、seed 1–5 共 60 位点 0 误判）
> 尚未在 M3 几何下重测，引用时注明来源版本。

已知边界（未做）：GUI 摆位工具、视觉阈值向实拍迁移、每型多格位（真机料框排布确认后再改）、真实夹爪开合。
