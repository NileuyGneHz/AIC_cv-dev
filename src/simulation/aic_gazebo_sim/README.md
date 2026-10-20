# aic_gazebo_sim — AIC 赛题 Gazebo 仿真（M1）

Ignition Gazebo 6（Fortress）仿真：**12 库位货架 + 四类缺陷铁片 + 三轴龙门（XYZ + 末端俯视相机）**，
外加控制侧与视觉侧的仿真实现，用来把 `decision → control → vision → decision` 整条闭环跑起来。

M1 已自验：`ros2 launch aic_gazebo_sim sim_decision.launch.py` 一轮 12 位点全 OK，
单点约 2 s，结果 CSV 与决策侧结果一致。

## 快速开始

```bash
# 1) 编译（在工作区根目录）
cd ~/AIC_cv-dev && ./scripts/build.sh

# 2) 端到端：仿真 + 龙门 + 视觉桩 + 决策行为树（跑完一轮自动退出）
./scripts/run_sim.sh

# 3) 本机看画面（需要图形界面）
./scripts/run_sim.sh --gui

# 4) 只起仿真（不起决策），自己玩话题
ros2 launch aic_gazebo_sim sim.launch.py gui:=true

# 5) 抓图（视觉侧取数）
ros2 run aic_gazebo_sim sim_snapshot.py --ros-args -p out_dir:=logs/snap -p count:=5
```

## 场景几何（唯一数据源 `config/sim_layout.yaml`）

| 项 | 值 |
|---|---|
| 货架 | 3 行 × 4 列 = 12 库位，1.0 m × 0.5 m，台面 z = 0.10 m |
| 列 / 行间距 | 0.25 m / 0.1667 m |
| 库位编号 | `slot_id = (row-1)*4 + col`，row 1 在 −Y（操作者前排）；与 `aic_warehouse` 同口径 |
| 铁片 | φ80 mm × 6 mm，静态模型，底面贴库位地贴 |
| 缺陷 | 1 划痕（细条）/ 2 污渍（扁圆斑）/ 3 缺角（边缘缺口）/ 4 孔洞（小圆孔），几何体表达 |
| 龙门 | X ±0.45 m、Y ±0.24 m、Z −0.42…0 m；末端夹指 + 俯视相机 |

改布局或缺陷类型的分配：改 `config/sim_layout.yaml` → 重跑 `scripts/gen_world.py` → 重启仿真。
世界文件是生成物（`worlds/aic_warehouse.sdf`），不要手改。
缺陷几何默认固定；做泛化检验用 `gen_world.py --seed N`（或改 `materials.jitter_seed`），
再 `./scripts/verify_sim_robust.sh 1 2 3 4 5` 多种子跑一轮，见 `docs/guides/simulation_guide.md` §4.2。

## 话题（全部经 ros_gz_bridge 转 ROS 2）

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| ROS → GZ | `/aic/joint/{x,y,z}/cmd_pos` | `std_msgs/Float64` | 关节位置指令（m） |
| GZ → ROS | `/camera/image` | `sensor_msgs/Image` | 640×480 rgb8 @20 Hz |
| GZ → ROS | `/camera/camera_info` | `sensor_msgs/CameraInfo` | fx=fy=554.3, cx=320, cy=240 |
| decision → 本包 | `/control/material_task` | `aic_msgs/MaterialTask` | 见 `docs/guides/decision_control_interface.md` |
| 本包 → decision | `/sim/gantry_arrived` | `std_msgs/UInt8` | 到位通知（slot_id），触发视觉桩 |
| 本包 → decision | `/sim/handle_done` | `std_msgs/UInt8` | HANDLE 动作完成 |
| 本包 → decision | `/control/ready` | `std_msgs/Bool` | 就绪握手（1 Hz 重发） |
| 本包 → decision | `/vision/material_type` | `std_msgs/String` | ground truth 桩的识别结果 |

## 文件

```text
config/sim_layout.yaml     布局 / 缺陷分配 / 运动参数（单一数据源）
models/aic_gantry/         三轴龙门 SDF：3 个 prismatic 关节 + 位置控制器 + 俯视相机
worlds/aic_warehouse.sdf   生成物：地面 + 货架 + 12 块铁片 + 龙门 include
scripts/gen_world.py       由 sim_layout.yaml 生成世界
scripts/gantry_controller.py  /control/material_task → 梯形剖面 → 关节指令
scripts/sim_vision_node.py   到位 → ground truth 缺陷类型（可注入失败/脏数据）
scripts/sim_snapshot.py      抓图存 PNG
launch/sim.launch.py         仿真 + 桥 + 控制/视觉桩
launch/sim_decision.launch.py 上面 + 决策行为树（一次跑完一轮）
```

详细设计、坐标换算与踩过的坑见 `docs/guides/simulation_guide.md`。
