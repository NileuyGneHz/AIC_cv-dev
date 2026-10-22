# 交接文档：从 Windows 切到 Ubuntu（Gazebo 自建仿真路线）

> 版本：2026-10-19　｜　读者：接手本工作区的自己 / 队友
> 背景：Windows 侧只做文档与决策链开发，**没有 ROS 2 / Gazebo 运行环境**；仿真与联调全部
> 转到 Ubuntu 22.04。本文是"到 Ubuntu 上第一小时该干什么"的完整清单。
> 相关技能：`aic-decision-loop`（决策链）、`aic-official-sim`（协议 + 录制合规）、
> `aic-vn-mock`（协议 mock）、`aic-defect-vision`（识别）、`aic-submission-pack`（提交物）。

---

## 0. 三十秒上手

```bash
cd ~/AIC_cv-dev                    # 仓库位置按实际克隆路径调整
./scripts/build.sh                 # 编译决策侧（-pipe 已内置，规避小 /tmp）
./scripts/verify_decision_loop.sh  # 端到端自验：场景 A 正常 / B 抗扰动
```

两条都过 = 环境没问题，可以开始搭 Gazebo。

---

## 1. 当前状态一页纸

| 模块 | 状态 | 证据 / 位置 |
|---|---|---|
| 决策链（行为树 12 位点巡视） | **已实现、已自验** | `src/decision/aic_behavior_tree`；`logs/verify_full_run.log` |
| 货架布局生成器 | **已实现、gtest 通过** | `src/decision/aic_warehouse`；改布局只改 YAML |
| 任务消息契约 | **已定稿** | `src/plugins/aic_msgs/msg/MaterialTask.msg` |
| vision 本体 | **留白**（只有话题契约） | `docs/guides/decision_vision_interface.md` |
| control 本体 | **留白**（只有话题契约） | `docs/guides/decision_control_interface.md` |
| Gazebo 世界 / 三轴 / 货架 / 相机 | **未开始** | 本文第 6 节是施工单 |
| mock（协议级） | **设计定稿，未实现** | `docs/guides/vn_mock_design.md` |

**一句话**：大脑有了，手和眼还没长出来，场地也还没搭。

---

## 2. 赛事硬事实（写代码前必须知道）

| 项 | 内容 | 证据 |
|---|---|---|
| 任务 | 圆形铁片四类缺陷识别（划痕/污渍/缺角/孔洞）→ 三轴吸取 → 12 库位分类码放 | `docs/competition/` |
| 场地 | 台面 1000×500mm；库位 50×70×110mm；三轴行程 X/Y/Z = 300/200/300mm | 赛题规则 docx |
| 时间 | **5 分钟内完成 12 个正确放置**为满分档；9–11 个 = 15 分；6–8 个 = 10 分 | 评分表 |
| 节拍预算 | 每个物料 ≈ **25 秒**（移动 + 拍照识别 + 抓取 + 放置 + 回位） | 由 300s / 12 推算 |
| 计分 | 四列对应四类缺陷；每库位有且只有一个正确物料；放对绿/放错红；**正确后再堆叠判错** | 平台说明书 §2.2.2 |
| 目录映射 | `material_type`：1=划痕 2=污渍 3=缺角 4=孔洞（**待赛题最终确认**） | `MaterialTask.msg` 注释 |
| 录屏合规 | 官方答疑（2026-10-19）：非必须购买平台，可用**同类型数字孪生平台**拍摄，但视频须满足比赛文件要求；说明书内无此条款，需留证 | `aic-official-sim` §6 |
| 提交物 | 源代码 zip + 竞赛视频 mp4 + **视觉识别图集 jpg/png** + 答辩 pptx | `aic-submission-pack` |

---

## 3. 仓库地图（Ubuntu 侧只需要关心这些）

```text
AIC_cv-dev/
├── AGENTS.md                      AI 工作约定（入口）
├── skills.md                      技能注册表（唯一登记处）
├── docs/
│   ├── competition/               官方文档 10 件（赛题/说明书/图纸/PLC/承诺书）
│   └── guides/                    接口与架构文档（vision / control / BT / mock 设计）
├── src/
│   ├── plugins/aic_msgs/          任务消息
│   ├── decision/aic_behavior_tree/ 决策核心（8 节点 + bt_runner）
│   ├── decision/aic_warehouse/    12 库位模型 + 蛇形路径
│   ├── vision/  control/          留白，待实现
│   └── simulation/                aic_gazebo_sim / aic_sim_devices 留白
├── scripts/                       build / verify / run / 桩
└── data/                          samples / calibration / results（不进 git）
```

---

## 4. 环境准备（两条路，先用 A）

### A. 复用现有 nav 容器（推荐，省 1 小时）

已有镜像 `alphabet2006/gxu2026-nav-env:amd64-latest`（Humble + Fortress 齐活）。

```bash
xhost +local:root                       # 宿主执行一次
docker run -it --rm \
  --net=host --ipc=host --pid=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/AIC_cv-dev:/ws \
  --gpus all \
  alphabet2006/gxu2026-nav-env:amd64-latest bash
```

容器内先验：`ls /opt/ros/humble/setup.bash && ign gazebo --versions`。

- 坑：容器内 `ign gazebo` 起不来 GUI，九成是 `DISPLAY` 或 X11 挂载没给，不是显卡问题；
  再查 `echo $DISPLAY`（宿主应为 `:0` 或 `:1`）。
- 坑：`--net=host` 是必须的，后面 ROS 2 DDS 与 Gazebo 传输都靠它。

### B. 宿主机直装（容器里缺包时）

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-gz \
                    ros-humble-behaviortree-cpp-v3 \
                    ignition-fortress
# 决策链还依赖 colcon / ament
sudo apt install -y python3-colcon-common-extensions
```

- 版本对齐：**Ubuntu 22.04 ↔ Humble ↔ Fortress（Ignition Gazebo 6）**。不要装 Harmonic
  （那是 Jazzy 的搭档）。
- 坑：Humble 的桥包名历史上叫 `ros-humble-ros-ign-bridge`，新源里是 `ros-humble-ros-gz-bridge`；
  `apt search ros-humble-ros-gz` 实测一下，别硬记。
- 坑：BT.CPP 在 Humble 源里是 `behaviortree-cpp-v3`，实际版本 3.8；本项目按 **4.10** 写的
  （`package.xml` 只写 `behaviortree_cpp`），若编译报 API 不匹配，先 `rosdep install` 再核版本。

### 环境自检（一次性跑完）

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "ros_gz|ros_ign|behaviortree|ignition" 
ign gazebo --versions
gazebo --version 2>/dev/null || true      # 旧 gazebo（classic）不必装
nvidia-smi | head -5                      # 有独立显卡就确认驱动在
echo $DISPLAY
```

---

## 5. 第一次构建与自验（必须两条都过）

```bash
cd ~/AIC_cv-dev
./scripts/build.sh --clean
source install/setup.bash
./scripts/verify_decision_loop.sh
```

预期：脚本打印两个场景（A 正常 12 位点 / B 每 5 条注入非法值仍不中断），末尾断言全过；
日志落 `logs/verify_full_run.log`，CSV 落 `logs/`。

- 坑：`colcon build` 在 `/mnt/f`（9P）上极慢——**仓库放 Linux 原生分区**（`~/AIC_cv-dev`），
  不要 `cd /mnt/f/...` 构建。
- 坑：`build.sh` 已内置 `-pipe`（小 `/tmp` 会导致 `No space left on device`）与
  `PATH=/usr/bin` 前置（避免别的 python3 抢 colcon），**别删这两行**。
- 单独起决策节点：`./scripts/run_decision.sh`；参数全在
  `src/decision/aic_behavior_tree/config/aic_decision_params.yaml`。

---

## 6. 下一步施工单：`aic_gazebo_sim` 包（本阶段核心）

### 6.1 包结构

```text
src/simulation/aic_gazebo_sim/
├── CMakeLists.txt / package.xml
├── worlds/aic_cell.sdf              # 台面 + 光照 + 三轴 + 货架 + 相机
├── models/
│   ├── gantry_3axis/model.sdf       # 3 个 prismatic joint（X/Y/Z）
│   ├── rack_12slot/model.sdf        # 4 列 × 3 行，库位 50×70×110mm
│   └── disc_workpiece/model.sdf     # 圆形铁片 φ50，带缺陷贴图
├── config/bridge.yaml               # ros_gz_bridge 话题映射
├── launch/sim.launch.py             # 一键起世界 + 桥 + 决策 + 视觉
└── scripts/run_hud.py               # 计时 / 步序 / 判定 HUD（录屏用）
```

### 6.2 三个关键插件（Fortress 版）

| 需求 | 插件 | 备注 |
|---|---|---|
| 三轴位置控制 | `JointPositionController` | 每轴一个，`<joint_name>` 对应 prismatic |
| 吸盘吸附/释放 | `DetachableJoint` | 末端接 parent、物料接 child；置位/复位即吸附/释放 |
| 相机出图 | `Sensors` 系统 + `<sensor type="camera">` | 720×540，固定俯拍，关阴影 |

- 坑：Fortress 的插件库文件名是 `libignition-gazebo-*-system.so`（不是 `gz-sim-*`），
  写错的表现是"世界能加载但功能静默失效"，**启动时看终端有没有 `Unable to find system plugin`**。
- 坑：相机要出 ROS 图像必须经 `ros_gz_bridge`，映射
  `/world/aic_cell/model/cam/link/cam_link/sensor/cam/image` → `/camera/image_raw`。

### 6.3 视觉域对齐（决定模型能不能迁移到官方平台）

官方平台是**纯白背景 + 均匀光 + 固定俯拍**。Gazebo 默认有阴影和方向光，直接训练会掉点：

1. 世界文件里只留一个 `<light type="directional">`，`<cast_shadows>false</cast_shadows>`；
2. 台面材质纯白（`<diffuse>1 1 1 1</diffuse>`），环境光拉满；
3. 相机正交/窄 FOV 俯拍，分辨率锁 720×540；
4. 缺陷用**贴图**（直接拿 48 例图纸裁切），不用几何挖孔——形态最接近官方。

### 6.4 判分与 HUD（录屏合规靠它）

- 判分：Python 侧按库位坐标判定（列对应类、一物一位、放对绿/放错红），
  **不要**写在 Gazebo 插件里，方便复用与复盘。
- HUD：`run_hud.py` 常驻显示「第 N/12｜类别｜判定｜累计用时」，
  直接满足「清晰反映比赛结果」；同时输出 CSV 供评分表填写。
- 录屏：单条 `ros2 launch aic_gazebo_sim sim.launch.py` 跑完全程，中途不碰键鼠。

### 6.5 与现有决策链的接法（不改决策代码）

```text
gz sim ──image──▶ /camera/image_raw ──▶ vision 节点 ──▶ /vision/material_type
                                                   └──▶ /control/material_task 的订阅方（control 桥）
control 桥 ──▶ gz 的 JointPositionController（/model/gantry/joint/x/cmd_pos）
           └─▶ DetachableJoint 的 attach/detach（/detachable_joint/attach）
```

**关键**：决策层一行不改，全部通过"桥"适配。这也是后面接官方 VirtualNext 协议的前提——
桥是可替换的，大脑不动。

---

## 7. 坑清单

### Windows 侧已踩（供理解历史决策，不必复现）

1. Bash 工具在本机不可用（`dirname: command not found`），当时改用 PowerShell；
2. PowerShell 命令成功但 stdout 不回显，且中文经管道必乱码——对策是让 Python 直接写 UTF-8 文件；
3. 受管 Python 无 PDF 库，用 venv `C:\Users\saber\.workbuddy\binaries\python\envs\default`；
4. 所有提取产物落 `.workbuddy/extracted/`，不再写 C 盘。

### Ubuntu 侧预判（先记住，遇到就对号）

1. **`/mnt/f` 构建慢**：9P 文件系统，colcon 会慢到无法忍受；仓库必须落原生分区；
2. **`ign gazebo` GUI 白屏**：`DISPLAY` / X11 挂载 / 容器网络三件套，逐项排查；
3. **QoS 不兼容**：vision 发布端用 best_effort 而决策订阅 reliable → 订阅成功但零消息、无报错
   （契约文档已标注，这是最隐蔽的坑）；
4. **桥话题名写错**：`ros_gz_bridge` 参数里 world/model 名必须与 SDF 完全一致，
   对不上时表现为"进程正常、话题不存在"；
5. **`/tmp` 太小**：编译报 `No space left on device` 用 `-pipe`（脚本已内置）；
6. **docs/guides 与 yaml 的两处不一致**（已知，未修）：
   文档写 `/vision/trigger` + `enable_trigger`，yaml 实际是 `/vision/capture` +
   `vision_trigger_enable`。对接 vision 前先以 **yaml 为准**，或顺手统一（属代码改动，需单独提）。

---

## 8. 文件与证据索引

| 要找什么 | 去哪 |
|---|---|
| 官方赛题 / 说明书 / 图纸 / PLC 手册 / 承诺书 | `docs/competition/`（10 件，含 44.9MB 快速入门指南） |
| vision / control 契约 | `docs/guides/decision_vision_interface.md`、`decision_control_interface.md` |
| 行为树结构与改树三个坑 | `docs/guides/decision_bt_architecture.md` |
| mock 设计（协议细节） | `docs/guides/vn_mock_design.md` |
| 任务消息字段 | `src/plugins/aic_msgs/msg/MaterialTask.msg` |
| 运行参数（话题 / 超时 / tick） | `src/decision/aic_behavior_tree/config/aic_decision_params.yaml` |
| 技能（怎么干活的沉淀） | `.workbuddy/skills/*/SKILL.md`（不进 git） |
| 提取产物（PDF 文本 / 渲染图 / 脚本） | `.workbuddy/extracted/`（不进 git） |

---

## 9. 验收标准（做到这几条才算接上）

1. `./scripts/verify_decision_loop.sh` 两场景全过；
2. `ign gazebo worlds/aic_cell.sdf` 能起，看到台面、三轴、12 库位、相机视角；
3. `/camera/image_raw` 有 720×540 图像，存一张图与官方图纸比对形态；
4. 单条 launch 能自动跑完 12 个物料，HUD 显示 12/12 且用时 < 300s；
5. 录屏一次成片（不剪辑），判分结果与 CSV 一致。

---

## 10. 未决问题（动手前先问清）

1. `material_type` 的 1–4 映射是否与官方 `Camera:Result` 一致（待赛题/平台确认）；
2. 官方答疑"同类型数字孪生平台"是否限定平台名单——**争取群公告**；
3. 混合缺陷（一物多缺陷）如何归类——优先级规则未定；
4. 走 Gazebo 还是先做 mock：建议 **mock 1 天跑通协议 → Gazebo 1 周出画面**，
   两者共用同一套桥接口。
