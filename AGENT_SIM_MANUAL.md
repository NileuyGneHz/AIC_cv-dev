# AIC 三轴分拣 —— Agent 仿真操作手册

> 版本：2026-10-21 首版
> 读者：接手本工作区的另一个 agent（无上下文，只读这份就能开工）
> 依据：`~/AIC_cv-dev` 现有代码 + 3 份官方 PDF（2026-10 版）+ `PLC-softwares/` 三个官方包
> 配套文档：`docs/guides/simulation_manual.md`（怎么跑仿真）、`plc_integration.md`（变量映射）、
> `plcnext_rsc_setup.md`（RSC 接线）、`docs/competition/platform_and_video_checklist.md`（提交要求）
>
> **本文不重复上面四份的内容**，只做三件事：给全局图、标清楚哪里文档和代码不一致、把活拆成可独立验收的小块。

---

## 0. 开工前必读（3 分钟）

### 0.1 三条铁律

1. **只在 `~/AIC_cv-dev` 改。** `/media/nileuy/编程与信息技术/AIC_cv-dev` 是 2026-10-15 的旧副本，
   缺文件、带未提交改动。改错地方等于白干。
2. **改 `scripts/*.sh` 前先 `pgrep -af gzserver`。** bash 是边读边执行，脚本跑到一半被改写会
   直接杀死正在跑的实例（历史上踩过，见 `simulation_manual.md` 坑 22）。
3. **`PLC-softwares/` 只读。** 里面是官方原始包（`.so`、`.pcwlx`、`wheel`），需要改的东西一律
   复制到 `src/` 或 `third_party/` 再改。

### 0.2 这个项目到底在做什么

比赛要求：**圆形铁片四类缺陷（划痕/污渍/缺角/孔洞）→ 视觉判型 → 三轴抓取 → 放进货架对应类型列**。
官方平台是一台菲尼克斯的三轴分拣机（vPLC + 海康相机 + ESTUN 伺服 + 货架）。
本仓库用 **ROS 2 + Gazebo** 复刻了同一套工艺流程，用来在没机器的时候开发算法、录省赛视频。

### 0.3 三套"世界"，别混

| 世界 | 在哪 | 通信 | 用途 | 本仓库支持度 |
|---|---|---|---|---|
| **A. 官方真机（VC）** | 赛场 / 实验室 | RSC over TCP 41100 | 国赛现场 | 有 `rsc_link.py`，**未接进 ROS** |
| **B. 官方仿真（VirtualNext）** | Windows 上位机 | TCP 字符串（`Camera:Trigger=True`） | 线上赛录屏 | **不支持**（可选任务 T14） |
| **C. 本仓库 Gazebo** | 本机 Ubuntu | ROS 2 话题 + Modbus 网关 | 开发 / 自验 / 录视频 | 主战场，已跑通 |

一个常见误解先破掉：**RSC 和 Modbus 不是二选一的技术选型，是两条不同阶段的通路。**
RSC 是官方 vPLC 的正路；Modbus 是"手上没有 vPLC 时用来对拍 PLC 逻辑"的替身。
两者承载的变量语义相同（见 `plc_integration.md` §5）。

---

## 1. 官方平台事实（从 PDF 提炼，可直接引用）

来源：`大赛设备(vc版)操作指导手册_2026.10.20.pdf`（44 页，简称【手册】）、
`VC算法封装和部署（VC和EPC区别）-使用指导.pdf`（13 页，简称【封装】）、
`EPC控制器更换为VC使用指导.pdf`（9 页，简称【替换】）、
`VC程序下载及RSC通信-使用指导.pdf`（7 页，简称【RSC】，同目录下，强烈建议一起读）。

### 1.1 硬件与控制链

```
海康工业相机 ──TCP/IP──┐
                       ├─► StarterKit vPLC（RK3568, ARM64, Podman 容器）
算法容器（Python）─────┘        │ PROFINET
                                ▼
                    AXL F BK PN TPS 耦合器 + Inline DI/DO
                                │
                                ▼
                    ESTUN ED3L PN 伺服 × 3（X/Y/Z）
```

| 项 | 值 | 出处 |
|---|---|---|
| 设备尺寸 | 1000 × 500 mm | 【手册】§1.2 |
| 控制器 | StarterKit vPLC，RK3568，ARM64，三网口 | 【手册】§1.3 |
| vPLC 形态 | **Podman 容器**（不是物理 PLC），**不可删除** | 【封装】§1 / §5.5 |
| 伺服 | ESTUN ED3L PN × 3 | 【手册】§1.3 |
| 相机 | 海康，采集帧率 5.0 fps | 【手册】§3.2.2 |
| 相机参数 | 像素格式必须与算法一致；黑电平使能打开；改完存"用户集" | 【手册】§3.2.2 |

### 1.2 网络与账号（现场第一张表）

| 目标 | IP | 账号 | 用途 |
|---|---|---|---|
| StartKit 宿主机 | `192.168.3.20` | `root` / `root` | SSH（MobaXterm），算法容器部署 |
| vPLC 对外（Macvlan） | `192.168.3.99` | `admin` / `plcnext` | eHMI 网页、**PC 侧 RSC 调试** |
| vPLC 内部桥接 | `10.88.0.2` | `admin` / `plcnext` | **容器内算法访问 vPLC 必须用这个** |
| 宿主机 ETH1 | `192.168.2.20` | — | 接相机（相机须在 2 网段） |
| 容器管理页 | `192.168.3.20:3001` | `admin` / `plcnext` | 恢复被删的 vPLC |
| PROFINET 耦合器 | `192.168.3.15` | — | — |
| 三台伺服 | `192.168.3.21/22/23` | — | — |
| 调试电脑 | `192.168.3.100` 以后 | — | 不要与上面冲突 |

> **坑**：`rsc通信demo/main.py` 里写的是 `password='381d3976'`（机身密码，EPC 时代）。
> VC 平台按【RSC】§2 是 `admin/plcnext`。现场先试 `plcnext`，不行再问工程师要机身密码。

### 1.3 eHMI 操作流程（真机，必须背下来）

【手册】§2.3.2，六步：

1. 上电 → 按蓝色**复位**按钮。
2. **零位标定**：浏览器开 `192.168.3.99` → 设置 → 零点标定 → 激活 X → 寻零 → 步骤重置 →
   Y → Z。寻零时注意别撞相机。
3. 示教（一般出厂已做）：手动模式 → 选位置号 → 点动到位 → 伺服示教 → 写入当前位置寄存器。
4. **自动模式 → 初始化 → 激活初始化**（会检查自动运行条件）。
5. **模式启动**（绿灯常亮）。
6. 检测台放料 → 按按钮盒**绿色启动** → 拍照识别 → 分拣 → eHMI 货架对应格点亮。

停止：红色停止按钮 → 确认停止 → 关总电源。**等系统完全关闭再重启。**

> **坑**：重新下载 PLC 程序后，**示教位置会被重写**，必须重新示教（【RSC】§1 末）。

### 1.4 官方数据契约：**有两代，别猜**

这是本项目最容易出错的地方。两份官方文档给的不是同一套变量。

**第一代：`Cam` 结构体握手（【手册】§3.1 图）**

```pascal
Cam : STRUCT
    CommOK  : BOOL;   // 通讯 OK
    Ready   : BOOL;   // 相机准备好
    Start   : BOOL;   // 拍照启动
    Done    : BOOL;   // 拍照完成
    Reset   : BOOL;   // 相机复位
    Prog    : INT;    // 相机程序号
    ErrCode : WORD;   // 错误代码
    Data    : INT;    // 拍照结果，100 = 拍照失败
END_STRUCT
```

**第二代：单触发 + 单结果（【RSC】§2，2026.10 的 VC 示例代码）**

```python
plc_var_name = "Arp.Plc.Eclr/imageClassify.image_result"   # UDT: udt_ImageClassify{index: INT}
plc_ip = '192.168.3.99'        # 容器内改 10.88.0.2
plc_user, pwd = 'admin', 'plcnext'

v = data_srv.ReadSingle("Arp.Plc.Eclr/Rsc_xExecute").Value.GetValue()   # 触发拍照
if v:
    data_srv.WriteSingle(WriteItem("Arp.Plc.Eclr/Rsc_xExecute", RscVariant.of(False)))  # 复位
```

注意两代的路径形态不同：`Rsc_xExecute` **没有程序名前缀**（是 GVL 全局变量），
`imageClassify.image_result` 才有（`imageClassify` 是程序名）。

> **铁律**：现场第一件事不是写代码，是**探变量**（任务 T11）。两份文档都可能是错的/过期的，
> 只有 `ReadSingle` 返回 `DataAccessError.NONE` 的名字才是真的。

### 1.5 算法封装与部署（VC 与 EPC 的关键差别）

【封装】§1 的对比表，摘要：

| 项 | EPC1502（旧） | VC-StartKit（现在） |
|---|---|---|
| PLC 形态 | x86 物理机 | **Podman 容器**，跑在 RK3568 |
| CPU | x86-64 | **ARM64** |
| 算法镜像 | x86_64 | **必须 arm64** |
| SDK/wheel | x86 | **aarch64/arm64** |
| 容器引擎 | Docker | **Podman** |
| RSC IP | EPC 硬件 IP | PC 调试 `192.168.3.99` / 容器内 `10.88.0.2` |

部署链（【封装】§3–4）：Windows Docker Desktop → `docker pull arm64v8/python:3.x-slim` →
装依赖（全 arm64）→ `docker export -o detectarm64.tar` → MobaXterm 传到宿主机 →
`podman import` / `podman load` → `podman run -it --network host` → 改 `run_ToPLC_new.py` 里的 IP →
`python3 run_ToPLC_new.py`。

> **坑**：`PLC-softwares/LibMvCamera(1)/` 里的 11 个 `.so` 用 `file` 看过，**全是 x86-64**。
> 它们只能在 PC/容器（x86）上用；要在 RK3568 上跑必须先拿到 **aarch64 版 MVS SDK**。

### 1.6 官方仿真平台 VirtualNext（可选路线）

来源：`VirtualNext Smart开发平台使用说明.pdf`（同目录）。要点：

- Windows 上的 3D 仿真，三轴 + 相机 + 货架，**TCP socket** 通信。
- 视觉侧两个端口：一个当 Server 收 PLC 触发，一个当 Client 收图像。
- 协议是**字符串**，不是二进制：

| 方向 | 报文 |
|---|---|
| PLC → 视觉 | `Camera:Trigger=True` |
| VirtualNext → 视觉 | `Camera:Start=True` |
| 视觉 → PLC | `Camera:Result={值}` |
| 错误码 | `-1` 图像不完整 / `-2` 超时 / `-3` 通信错误 / `-4` 连接拒绝 |

- 轴行程：X `0..300`、Y `0..300`、Z `0..200`，速度 `1..5`。
- 运动控制变量：`SpeedSetting / PositionSetting / Start / SpeedFeedback / PositionFeedback / Arrived`，
  IO：`MaterialFeed / SuctionEnable / SuctionEnabled / MaterialInPosition`。
- 判分：4 列 × 3 位；放对变绿、放错变红；**已放对的格子再堆叠一律不得分**。

这套协议和我们 Gazebo 的 ROS 话题完全不同，需要一层适配（任务 T14，低优先级）。

---

## 2. 本仓库框架（现状）

### 2.1 包清单

| 包 | 路径 | 语言 | 职责 | 状态 |
|---|---|---|---|---|
| `aic_msgs` | `src/plugins/aic_msgs` | msg | `MaterialTask.msg` | 可用 |
| `aic_warehouse` | `src/decision/aic_warehouse` | C++ | 3×4 库位模型、蛇形巡视顺序 | 可用 |
| `aic_behavior_tree` | `src/decision/aic_behavior_tree` | C++ | BT.CPP 4.10 主树 + VisionClient/ControlBridge | 可用 |
| `aic_vision_trad` | `src/vision/aic_vision_trad` | Python | 传统视觉四分类 | 可用 |
| `aic_gazebo_sim` | `src/simulation/aic_gazebo_sim` | Python/XML | 世界生成、龙门控制器、抓帧 | 可用 |
| `aic_plc_gateway` | `src/plugins/aic_plc_gateway` | Python | PLC 网关（Modbus / RSC 双后端） | 两种后端均可用；RSC 仅干跑验证，**未接真 vPLC** |
| `aic_dashboard` | `src/plugins/aic_dashboard` | Python | 判分看板 + 自由机位 | 可用 |

> 2026-10-22：上表就是**全部**包（7 个，每个都有 `package.xml` 且能 `colcon build`）。
> 原先列在这里的 5 个占位目录已删除——只有 `.gitkeep` 的目录不能被构建，却容易让人
> 以为已有实现。后续规划（`aic_motion` / `aic_io_bridge` / `aic_vision_core` /
> `aic_vision_dl` / `aic_bringup`）连同「何时该建」写在 `AGENTS.md` §5，不再建空目录。

### 2.2 数据流（当前唯一跑通的链路）

```
bt_runner (C++)
   │  /control/material_task  (aic_msgs/MaterialTask)
   ▼
gantry_controller.py ──► /aic/joint/{x,y,z}/cmd_pos ──► ros_gz_bridge ──► Gazebo
   │  ▲                  (闭环：joint_state 双阈值判到位)
   │  └── /sim/attach_NN / detach_NN（DetachableJoint 模拟吸盘）
   ├──► /sim/gantry_arrived (UInt8)
   ├──► /sim/handle_done    (UInt8)
   └──► /control/ready      (Bool)
                         │
                         ▼
            vision_node.py ──► /vision/material_type (String "1".."4") ──► bt_runner
                         ▲
             /camera/image (sensor_msgs/Image, 末端 ee_cam)
```

**物料流转（M4 基线）**：来料架 3 行 × 4 列（12 块）→ `set_pose` 传到置物台 → 固定相机拍照 →
吸盘下探 attach → 抬升 → 移到分拣货架类型列最低空层 → detach。一轮 12 个位点。

> 为什么不在分拣货架上直接开局铺满：12 格全满 + 类型乱序时，单臂 + 单缓冲无法完成环形置换，
> 物理上不可解。这条结论写在项目记忆里，别推翻重来。

### 2.3 三种通信协议在本仓库的落点

| 协议 | 代码 | 谁连谁 | 现在能用来干什么 | 缺口 |
|---|---|---|---|---|
| ROS 2 话题 | 全部节点 | 进程间 | 仿真全链路 | 无 |
| Modbus TCP | `plc_gateway.py`（自写从站，纯标准库） | PLC 当主站，连网关 5020 | 软 PLC 对拍、教学 | 无 |
| PLCnext RSC | `rsc_link.py` + `RscBackend` | 网关当客户端，连 vPLC 41100 | `backend:=rsc` 与 Modbus 同一套状态机；干跑可跑整轮握手 | 没有 vPLC，现场变量名未核对 |

### 2.4 关键文件索引（改之前先读）

| 文件 | 行数 | 一句话 |
|---|---|---|
| `scripts/run_sim.sh` | 105 | 仿真总入口，包了 `sim_decision.launch.py` |
| `scripts/verify_sim_m2.sh` | — | 端到端自验（真视觉 + 抓放 + 落位断言） |
| `src/simulation/aic_gazebo_sim/scripts/gen_world.py` | 763 | 从 `sim_layout.yaml` **生成** 世界 SDF（生成物不要手改） |
| `.../scripts/gantry_controller.py` | 659 | 三轴动作序列（`_goto_steps` / `_handle_steps`），到位闭环 |
| `.../scripts/sim_layout.py` | — | 布局公式，与生成器同源 |
| `src/decision/aic_behavior_tree/src/bt_runner.cpp` | 199 | 建树 + 主循环 |
| `src/decision/aic_behavior_tree/src/control_bridge.cpp` | — | 发 `MaterialTask` |
| `src/decision/aic_behavior_tree/src/vision_client.cpp` | — | 收识别结果，`seq` 过滤 |
| `src/vision/aic_vision_trad/scripts/vision_node.py` | 216 | 订阅 `/camera/image`，发 `String` |
| `src/plugins/aic_dashboard/scripts/score_node.py` | 806 | 判分看板（纯旁路，只订阅） |
| `src/plugins/aic_plc_gateway/scripts/plc_gateway.py` | 900+ | 状态机 + Modbus/RSC 双后端 |
| `src/plugins/aic_plc_gateway/scripts/rsc_link.py` | 660 | RSC 标签表层（被 `RscBackend` 引用） |
| `src/plugins/aic_plc_gateway/config/plc_rsc_map.yaml` | 83 | 26 个标签 → PLC 变量路径 |
| `docs/guides/visual_walkthrough.md` | — | 窗口手册：Gazebo / 看板 / 终端怎么开、该看到什么 |
| `docs/plc/main.st` | 198 | PLC 侧巡视状态机 + 看门狗（骨架，字段名与 YAML 对齐） |
| `docs/adr/0002-plc-contract.md` | — | PLC 数据契约选型：默认 5 字段，官方两代作现场分支 |

### 2.5 官方 ↔ 仿真 对照（写汇报材料直接用）

| 官方 | 本仓库 | 差异 |
|---|---|---|
| vPLC IEC 61131-3 程序 | `gantry_controller.py` | 官方 PLC 逐动作编排；我们是"宏动作" `HANDLE` 一次做完 |
| `Cam.Start/Done/Data`（或 `Rsc_xExecute` + `image_result`） | `/sim/gantry_arrived` → `/vision/trigger` → `/vision/material_type` | 我们是话题，官方是变量 |
| `Pos`（由 `Camera.Data` 映射） | `rack_cell(mtype, level)` | 官方 PLC 查表；我们决策侧算 |
| eHMI 库位点亮 | `aic_dashboard` | 语义一致 |
| PROFINET + 伺服 | Gazebo `JointPositionController` | 仿真无背隙、无噪声，精度 0.0 mm |
| 人工上料 | `set_pose` 瞬移 | 真机是人工放检测台 |

> **汇报红线**：不要把仿真的 0.0 mm 落位精度当成真机精度写进材料。

---

## 3. 已知不一致与坑（Agent 必须先知道）

按"会不会让人白干"排序。

| # | 问题 | 证据 | 处理 |
|---|---|---|---|
| 1 | `decision_control_interface.md` 描述的是**旧版消息**（`command`/`visit_index`/`material_type:string`），实际 `MaterialTask.msg` 是 `action:string` + `uint8` 字段 | 对比 `docs/guides/decision_control_interface.md` §2 与 `src/plugins/aic_msgs/msg/MaterialTask.msg` | **以 `.msg` 为准**；改代码时别照文档抄字段名 |
| 2 | ~~`plc_rsc_map.yaml:4` 写 `backend:=rsc`，但 launch 没有这个参数~~ | ~~`launch/plc_gateway.launch.py` 全文~~ | **已修**（`f92f629`）：注释改为 `rsc_link.py` 的实际用法，并写明 RSC 尚未接进网关状态机 |
| 3 | ~~`CMakeLists.txt` 只装 2 个 py + `launch/`~~ | `install/aic_plc_gateway/` 实查 | **已修**（`f92f629`）：补齐 `config/`、`third_party/`、`rsc_link.py`；换目录 `ros2 run` 也能找到 SDK |
| 4 | ~~`rsc_link.py --read` 读不到 `gw2plc` 标签~~ | `rsc_link.py:369` | **已修**：`read_all(include_writes=True)`，CLI `--read` 自动带上 |
| 5 | 官方变量有两代（`Cam` 结构体 / `Rsc_xExecute`+`image_result`），文档之间不一致 | 【手册】§3.1 vs【RSC】§2 | 任务 T11 现场探测，不要二选一硬编码 |
| 6 | `LibMvCamera` 全是 x86-64 | `file` 实测 | 上机前换 aarch64 版 |
| 7 | `PLCnextBase` 只有工具 FB，**没有运动控制 FB** | 解包后 74 个 POU 分类 | 别指望它让轴动 |
| 8 | 重下 PLC 程序会清掉示教位置 | 【RSC】§1 末 | 每次下载后重新示教 |
| 9 | 删了 vPLC 容器就没了 | 【封装】§5.5 | 只用 `192.168.3.20:3001` 恢复，**永不 `podman rm`** |

---

## 4. 任务分解

每个任务都是**可独立验收**的一小块。做完一个就提交一次（`type(scope): 中文摘要`），
不要攒一大坨。任务之间的依赖用 `→` 标。

### 阶段 A：先把基线跑起来（1 小时内）

#### T01 复现当前基线
- **目标**：确认"我拿到的是一个能跑的系统"，拿到自己的耗时数字。
- **命令**：
  ```bash
  cd ~/AIC_cv-dev
  ./scripts/build.sh
  ./scripts/run_sim.sh              # 真值桩，期望结果 CSV 12 行全 OK（耗时以本机实测为准）
  ./scripts/verify_sim_m2.sh        # 真视觉，≈160 s，期望 PASS
  ```
- **验收**：`verify_sim_m2.sh` 输出 `PASS`；记录整轮耗时、`logs/` 下 CSV 路径。
- **坑**：`/tmp` 只有 10 MB，构建必须走 `scripts/build.sh`（已带 `-pipe` 和 python3.10 处理）。

#### T02 建"事实索引"
- **目标**：把 5 份 PDF 的关键结论固化到仓库，后面所有 agent 不用再翻 PDF。
- **动作**：新建 `docs/competition/platform_facts.md`，抄 §1 的四张表（网络、账号、契约两代、
  VC/EPC 差异），每条标出处（文档名 + 章节）。
- **验收**：文档里每个数字都能追溯到一个章节号；不写任何"我觉得"。
- **依赖**：无。

### 阶段 B：把 RSC 接成第二后端（核心，1–2 天）

> 这一段的目标不是"删掉 Modbus"，而是让 `PlcGateway` 的状态机与协议解耦：
> **同一套状态机，能挂 Modbus，也能挂 RSC**。

#### T03 补齐安装表 —— **已完成（`f92f629`）**
- **文件**：`src/plugins/aic_plc_gateway/CMakeLists.txt`
- **动作**：
  ```cmake
  install(PROGRAMS scripts/plc_gateway.py scripts/plc_master_demo.py scripts/rsc_link.py
          DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config third_party
          DESTINATION share/${PROJECT_NAME})
  ```
- **验收**：`colcon build --packages-select aic_plc_gateway` 后，
  `ls install/aic_plc_gateway/share/aic_plc_gateway/config/plc_rsc_map.yaml` 存在，
  `install/aic_plc_gateway/lib/aic_plc_gateway/rsc_link.py` 存在。

#### T04 launch 加 backend 参数 —— **已完成**（`backend` + 6 个 RSC 参数已加）
- **文件**：`src/plugins/aic_plc_gateway/launch/plc_gateway.launch.py`
- **动作**：加 `backend`（默认 `modbus`）、`rsc_host`、`rsc_port`（41100）、`rsc_user`（admin）、
  `rsc_map`（默认 share 下的 yaml）。`backend:=rsc` 时把对应参数传进节点。
- **验收**：`ros2 launch aic_plc_gateway plc_gateway.launch.py --show-args` 能看到全部新参数；
  `backend:=modbus` 行为与现在**完全一致**。

#### T05 抽出 IoBackend（重构，不改行为） —— **已完成**（`IoBackend`/`ModbusBackend` 已落地）
- **文件**：`scripts/plc_gateway.py`
- **动作**：定义最小接口
  ```python
  class IoBackend:
      def open(self): ...
      def close(self): ...
      def read(self) -> dict:   # {标签: 值}
      def write(self, values: dict) -> int: ...
  ```
  把现在 `_poll_commands()` / `_publish_status()` 里对 `self.store` 的直接读写
  （`plc_gateway.py:556-687`）改成 `self.io.read()` / `self.io.write()`。
  `ModbusBackend` 内部继续用 `DataStore + ModbusTcpSlave`，**寄存器布局一个字节都不许动**。
- **验收**：`bash scripts/test_plc_proto.sh` 仍通过；`plc_master_demo.py --mode observe`
  输出与改造前一模一样（拿改前的日志 diff）。
- **坑**：`_on_joint_state` / `_on_vision` 等回调会直接写 `store.holding[...]`，这些要么改成写
  本地状态再由 `write()` 汇总，要么让 backend 暴露 `set_status()`。**别让回调绕过 backend**。

#### T06 实现 RscBackend —— **已完成**（`RscBackend` 已落地，干跑整轮通过）
- **文件**：`scripts/plc_gateway.py`（或新文件 `scripts/rsc_backend.py`）
- **动作**：`RscBackend` 直接包 `rsc_link.RscLink`——它的 `read_all()/write_all()` 已经是
  `{标签: 值}` 语义，天然对齐 `IoBackend`。注意 `read_all()` 只在连接时建 schema。
- **验收**：`backend:=rsc` + 假 PLC（`RscLink(..., device=FakeDevice)` 或 dry_run 参数）
  能跑一轮 `plc_master_demo` 等价流程，日志里能看到 26 个标签读写。
- **依赖**：T05。

#### T07 契约对齐（决定要不要兼容 `Cam`） —— **已完成**（写了 `docs/adr/0002-plc-contract.md`：默认自定义 5 字段，官方两代作为现场分支，触发条件已写明）
- **背景**：官方有两代契约（§1.4）。我们现在的 YAML 是自创的 5 字段 `Camera`
  （`Request/State/Data/Ms/Err`），**和官方两代都不一样**。
- **动作**：二选一，写进 ADR：
  - **方案 A（推荐）**：新增 `plc_rsc_map_cam.yaml`，按【手册】`Cam` 8 字段建标签，
    保留现有 `plc_rsc_map.yaml` 作为"我们自己的简化契约"；
  - 方案 B：只保留 `Rsc_xExecute + image_result` 最小契约（【RSC】）。
- **验收**：`docs/adr/0002-plc-contract.md` 写清选型理由、字段对照表、切换方法。
- **依赖**：无（可与 T05 并行）。

#### T08 修 `--read` 读不到 gw2plc 标签 —— **已完成**（已修）
- **文件**：`scripts/rsc_link.py:369-408`
- **动作**：给 `read_all()` 加 `include_writes=False` 参数；CLI 显式 `--read` 某个标签时强制读它。
- **验收**：`--dry-run --write HEARTBEAT=0 --read HEARTBEAT` 能打印 `0` 而不是 `None`。

### 阶段 C：真实相机（1 天，需要相机或至少需要库）

#### T09 摸清 MVS 库
- **目标**：在**不改项目**的前提下，验证这 11 个 `.so` 能不能在本机加载、能不能枚举到相机。
- **动作**：写 `scripts/probe_mvs.py`（ctypes）：
  `MV_CC_Initialize` → `MV_CC_EnumDevices` → 打印设备列表 → `MV_CC_Finalize`。
  库路径通过 `LD_LIBRARY_PATH` 指到 `PLC-softwares/LibMvCamera(1)`。
- **验收**：无相机时返回"0 个设备"而不是段错误；有相机时能打印型号/序列号/IP。
- **坑**：只有 `.so` 没有头文件，函数签名要按 MVS SDK 文档对；**x86-64 库在 ARM 上必失败**，
  先 `uname -m` 确认。

#### T10 相机节点
- **文件**：新建 `src/vision/aic_camera_mvs/`（package.xml + CMakeLists + `camera_node.py`）
- **动作**：发布 `/camera/image`（`sensor_msgs/Image`，`bgr8`），参数 `exposure`、`gain`、
  `fps`（默认 5.0）、`pixel_format`。**接口与仿真相机完全一致**，`vision_node.py` 不用改。
- **验收**：`ros2 run aic_camera_mvs camera_node.py` 后 `ros2 topic hz /camera/image` ≈ 5 Hz；
  `vision_node.py` 能直接吃这一路跑出 `"1".."4"`。
- **依赖**：T09。

### 阶段 D：PLC 侧程序（半天写，一天调）

#### T11 变量探测脚本（上机第一件事）
- **文件**：新建 `scripts/probe_plc_vars.py`
- **动作**：候选名单逐个 `ReadSingle`，打印 `DataAccessError`：
  ```
  Arp.Plc.Eclr/Rsc_xExecute
  Arp.Plc.Eclr/imageClassify.image_result
  Arp.Plc.Eclr/Main.Cam.Start / .Done / .Data / .CommOK / .Ready
  Arp.Plc.Eclr/Main.Camera.Request / .Data
  Arp.Plc.Eclr/Main.Pos
  ```
  存在的记 `NONE`，不存在的记 `NotExists`，输出一张表。
- **验收**：`python3 probe_plc_vars.py --host 192.168.3.99` 能生成 `logs/plc_vars.txt`，
  里面至少有一条 `NONE`（全 `NotExists` 说明 POU 名或权限不对）。
- **坑**：必须 python3.10；容器内跑要换 `10.88.0.2`。

#### T12 PLC 侧 ST 骨架 —— **已完成**（`docs/plc/main.st`：12 位点状态机 + 看门狗，关键字配平已校验）
- **文件**：`docs/plc/` 下新建 `main.st`（示意代码，不进 PLCnext Engineer）
- **动作**：按 T07 选定的契约写：`Cam` 握手状态机（IDLE→START→WAIT_DONE→READ_DATA→MOVE）
  + 看门狗（心跳 2 s 不变切安全态）。
- **验收**：能被 PLCnext Engineer 直接粘贴编译（人工确认）；状态迁移表与
  `plc_integration.md` §7 的握手时序一致。

#### T13 PLCnextBase 使用点 —— **已完成**（`docs/plc/README.md`：三个 FB 的用途与坑，接口指向官方 CHM）
- **动作**：在 `docs/plc/README.md` 记三条：`PBCL_SysGetMicrotick`（测拍照耗时）、
  `PBCL_FileCsvWrite`（结果落盘）、`PBCL_Conv*`（与 PC 端 int16 补码对齐）。
- **验收**：写清每个 FB 的输入输出，别抄 CHM 原文。

### 阶段 E：联调与验收

#### T14 VirtualNext 适配器（可选，低优先级）
- **目标**：如果线上赛必须用官方 VirtualNext，能把我们的视觉算法接上去。
- **动作**：新建 `src/plugins/aic_vnext_bridge/`，实现字符串协议
  （`Camera:Trigger=True` / `Camera:Start=True` / `Camera:Result={值}`），值域 1..4，错误码 -1..-4。
- **验收**：用 `nc -l` 假装 VirtualNext，能完成一次 trigger→result 往返。
- **依赖**：无。**没有官方平台就先别做。**

#### T15 五步联调（有机器才做）
- 严格按 `docs/guides/plcnext_rsc_setup.md` §5：干跑 → 只读 → 单点写 → 心跳看门狗 → 握手闭环。
- **验收**：`HEARTBEAT` 在 PLC 变量表里每秒 +1；断开网关 2 s 后 PLC 报警。

#### T16 性能基线
- **动作**：记录 RSC 单次 `Read` 往返、`read_all()` 全量往返、Modbus 全量往返，写进
  `docs/guides/plc_integration.md` 新增一节。
- **验收**：三个数字 + 测量方法（脚本 + 循环次数），可复现。

### 阶段 F：材料

#### T17 把映射表搬进 PPT
- **动作**：§2.5 的表 + 两张架构图（官方、我们）做成 2 页，进技术文档。
- **验收**：每张图能单独看懂；图上标注"仿真 vs 真机"的差异。

---

## 5. 每个任务的通用工作流

```
1. 读     —— 先读要改的文件 + 相关 docs，别凭记忆
2. 说     —— 改超过 50 行先说方案（动哪些文件、为什么）
3. 改     —— 一次只改一个关注点
4. 构     —— ./scripts/build.sh（或 --packages-select）
5. 验     —— 跑对应验收命令，**把输出贴进提交说明**
6. 记     —— 结果写 .workbuddy/memory/YYYY-MM-DD.md；坑写进对应 docs
7. 提     —— git -c user.name=NileuyGneHz -c user.email=saber2089467296@163.com commit
```

### Definition of Done

- [ ] `./scripts/build.sh` 无 error（warning 可留）
- [ ] 该任务声明的验收命令跑过，输出留档
- [ ] 没动无关文件（`git status` 干净）
- [ ] 新增的坑/约定写进 docs 或 memory
- [ ] 提交信息符合 `feat|fix|docs|chore(scope): 中文摘要`

---

## 6. 禁止事项

1. **不 `rm -rf` 任何东西**，尤其 `/`、`~`、`PLC-softwares/`。官方文档专门警告过。
2. **不删 vPLC 容器**（`podman rm`）。删了要重新下载 PLC 程序 + 重新示教。
3. **不在 `/media/...` 的旧副本上改**。
4. **不手改生成的 world SDF**，改 `sim_layout.yaml` 后重跑 `gen_world.py`。
5. **不在仿真跑着的时候改 `scripts/*.sh`**。
6. **不把 12 块料开局直接铺在分拣货架上**（物理不可解，见 §2.2）。
7. **不擅自 push 到 GitHub**——比赛仓库，push 前问。
8. **不把 `LibMvCamera` 的 x86 `.so` 拷进 ARM 容器**。

---

## 7. 命令速查

```bash
# 构建 / 跑仿真 / 自验
./scripts/build.sh
./scripts/run_sim.sh                       # 真值桩
./scripts/run_sim.sh --vision trad --gui   # 真视觉 + 界面
./scripts/verify_sim_m2.sh                 # 端到端 PASS
./scripts/run_sim.sh --domain auto         # 同机多实例隔离

# 看板
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad
# 浏览器 http://127.0.0.1:8080

# PLC 网关（Modbus）
ros2 launch aic_plc_gateway plc_gateway.launch.py
python3 src/plugins/aic_plc_gateway/scripts/plc_master_demo.py --mode observe

# RSC 自检（不需要 PLC）
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --describe
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --dry-run --watch

# RSC 连真机（只读）
export PLCNEXT_HOST=192.168.3.99 PLCNEXT_PASSWORD=plcnext
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --read CAM_REQ --read POS_SET
```

---

## 8. 一页图：三条链路的关系

```
                     ┌──────────────── 官方真机（国赛） ────────────────┐
                     │  算法容器 ──RSC 10.88.0.2:41100──► vPLC ──► 伺服  │
                     └──────────────────────────────────────────────────┘
                                        ▲
                          同一套变量语义（Cam / Pos / Axis）
                                        ▼
  ┌──────── 本仓库 Gazebo（开发/录屏） ────────┐      ┌── 官方 VirtualNext ──┐
  │ bt_runner → gantry_controller → Gazebo     │      │ TCP 字符串协议        │
  │        ↕ /control/material_task            │      │ Camera:Trigger=True  │
  │ vision_node ← /camera/image                │      │ Camera:Result={n}    │
  │        ↕ Modbus 5020（对拍）                │      └──────────────────────┘
  └────────────────────────────────────────────┘
```

**记住一句话**：仿真验证算法，RSC 验证接口，Modbus 验证时序。
三者互不替代，别拿其中一个的结论去证明另一个。
