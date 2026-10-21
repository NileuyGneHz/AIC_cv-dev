# PLC 协议集成指南（aic_plc_gateway）

> 版本：2026-10-21 · 适用 M4 仿真基线
> 目标读者：要把 Gazebo 仿真接到真实 PLC（或软 PLC）上做联调、演示、教学的人。
> 相关文档：`simulation_manual.md`（仿真怎么跑）、`decision_control_interface.md`（ROS 话题契约）

---

## 0. 结论先说

**能接，但协议网关不应该塞进 Gazebo 进程。**

Gazebo 是物理引擎，不是设备。真机上 PLC 连的是伺服驱动器和 IO 模块，从不直连"仿真器"。
所以要接的那一层是 **`gantry_controller`（运动控制）**，网关放在 ROS 2 侧：

```
PLC  <--Modbus TCP-->  aic_plc_gateway  <--ROS 话题-->  仿真(Gazebo)
```

这样做的好处：网关崩了不影响仿真；换协议只改网关；真机上同一个网关可以直接复用。

已实现的 `src/plugins/aic_plc_gateway/` 是一个 **Modbus TCP 从站**，只用标准库
（`socket` / `struct` / `threading`），**不需要 `pip install pymodbus`** ——
现场机器没网也能跑。

---

## 1. 官方设备的真实架构

以下来自两份菲尼克斯手册（`VC算法封装和部署`、`大赛设备(vc版)操作指导手册`）：

```
海康工业相机 ──TCP/IP──> 算法容器 (Python, Podman on RK3568)
                              │
                              │  PLCnext RSC (PyPlcnextRsc / gRPC)
                              ▼
                         vPLC (IEC 61131-3)  ← 192.168.3.99 / 桥接 10.88.0.2
                              │
                              │  PROFINET
                              ▼
              AXL F BK PN TPS 耦合器 + Inline DI/DO
                              │
                              ▼
                    ESTUN ED3L PN 伺服 × 3 (X/Y/Z)
```

关键事实：

| 项 | 内容 |
|---|---|
| 控制器 | StarterKit vPLC，**IEC 61131-3 程序**，跑在 RK3568 的 Podman 容器里 |
| 现场总线 | PROFINET（耦合器 + DI/DO 模块 + 三台伺服） |
| 算法侧通信 | **PLCnext RSC**（Python 库 `PyPlcnextRsc`）或 gRPC&C++ |
| 数据契约 | 结构体变量 `Camera`：`Camera.Request`（PLC→算法）、`Camera.Data`（算法→PLC） |
| 结果映射 | PLC 把 `Camera.Data` 映射成变量 `Pos`，决定货架位置 |
| 设备尺寸 | 1000 × 500 mm |
| 相机 | 海康，采集帧率 5 fps，MVS V4.4.0 调参 |
| eHMI | 192.168.3.99，admin/plcnext；宿主机 192.168.3.20 root/root |

**这套架构里没有 ROS，也没有行为树。** 运动时序全在 PLC 的 IEC 程序里。

---

## 2. 仿真与官方怎么对应

| 官方 | 本项目 | 说明 |
|---|---|---|
| vPLC 的 IEC 61131-3 程序 | `gantry_controller.py` | 运动时序、互锁、吸盘动作 |
| 算法容器 + RSC | `vision_node` + `bt_runner` | 识别 + 巡视决策 |
| `Camera.Request` | `/sim/gantry_arrived` → `/vision/trigger` | 到位即请求拍照 |
| `Camera.Data` | `/vision/material_type`（String "1".."4"） | 物料编号 |
| 变量 `Pos` | `rack_cell(mtype, level)` | 类型列 + 层 |
| eHMI 库位点亮 | 看板 `score_node` | 绿/红/灰 |
| PROFINET + 伺服 | Gazebo `JointPositionController` | 位置伺服 |

### 三处结构差异（重要，别混为一谈）

1. **职责划分不同**：仿真里"决策"和"运动控制"是两个节点；官方 PLC 一个程序全包。
2. **动作颗粒度不同**：仿真的 `HANDLE` 是**宏动作**（下探→吸附→抬升→平移→放入→释放
   一气呵成），PLC 只能整段调用，无法逐轴/逐气缸控制。官方是 PLC 逐动作编排。
3. **触发方式不同**：官方靠 eHMI 人工按启动、人工放料；仿真由行为树自动巡视 12 个位点。

第 2 条决定了接管模式下 PLC 能做什么：只能下发 `GOTO` / `HANDLE`，不能单独发
"Z 轴下探到 239 mm"。要改这一点必须拆 `gantry_controller._handle_steps()`。

---

## 3. 三种接法怎么选

| 接法 | 位置 | 优点 | 代价 | 什么时候选 |
|---|---|---|---|---|
| **A. ROS 侧网关**（已实现） | 独立节点 | 不改现有契约、可热改、真机能复用 | 多一跳 1–3 ms | 默认 |
| B. Gazebo System Plugin | 引擎进程内（C++ + libmodbus） | 延迟最低、与物理同频 | 改一次要重编译、崩了拖垮仿真 | 需要 <1 ms 硬实时 |
| C. 纯软件对拍 | 不进仿真 | 验证协议栈最快 | 不驱动仿真 | 只调梯形图逻辑 |

A 与 B 的差别只在"网关放哪"，不在"能不能"。

---

## 4. 协议怎么选

| 协议 | 库 | 适配真实 PLC | 判断 |
|---|---|---|---|
| **Modbus TCP**（已实现） | 自写 / pymodbus | 几乎所有 PLC | 首选，最省事 |
| OPC UA | asyncua | 西门子/倍福/汇川 | 语义好（类型、订阅），配置重 |
| PLCnext RSC | PyPlcnextRsc | 菲尼克斯 vPLC | **官方路径**，需真机或可仿 vPLC |
| S7comm | python-snap7 | 西门子 S7-1200/1500 | 只能仿 S7 |
| PROFINET / EtherCAT | SOEM / IgH | 是 | 二层协议 + 实时内核，仿真里不现实 |

**注意**：官方推荐 RSC，本实现选 Modbus 是因为没有 vPLC 硬件时 RSC 无法真联调。
两者的**变量语义完全相同**，换协议只换传输层（见 §9）。

---

## 5. 变量映射表

地址偏移从 0 起算；多数 PLC 软件显示为 `40001 + 偏移`。

### 保持寄存器（4x，PLC 读写）

| 偏移 | 名称 | 方向 | 含义 |
|---|---|---|---|
| 0 | `CAM_REQ` | PLC→算法 | 1 = 请求拍照（网关取走后清 0） |
| 1 | `CAM_STATE` | 算法→PLC | 0=IDLE 1=BUSY 2=DONE 3=FAIL |
| 2 | `CAM_DATA` | 算法→PLC | 物料编号 1..4，0=未识别（官方 `Camera.Data`） |
| 3 | `CAM_MS` | 算法→PLC | 本次识别耗时 ms |
| 4 | `CAM_ERR` | 算法→PLC | 错误码 |
| 5 | `TASK_CMD` | PLC→控制 | 1=GOTO 2=HANDLE（配合 `POS_SET`，执行后清 0） |
| 10 | `POS_SET` | PLC→控制 | 目标库位 1..12（官方变量 `Pos`） |
| 11 | `X_ACT` | 控制→PLC | X 实际位置 mm（**int16 补码**） |
| 12 | `Y_ACT` | 控制→PLC | Y 实际位置 mm（int16 补码） |
| 13 | `Z_ACT` | 控制→PLC | Z 实际位置 mm（int16 补码） |
| 14 | `AXIS_STATE` | 控制→PLC | 0=IDLE 1=MOVING 2=INPOS 9=FAULT |
| 20 | `SHELF_BITS` | 控制→PLC | 货架占用位图，bit(n-1) = 第 n 号已入位 |
| 21 | `SHELF_FULL` | 控制→PLC | 1 = 12 格放满 |
| 22 | `PLACED_CNT` | 控制→PLC | 已放入计数 |
| 23 | `TASK_STATE` | 控制→PLC | 0=IDLE 1=GOTO 2=HANDLE 5=DONE 9=FAULT |
| 30 | `HEARTBEAT` | 网关→PLC | 每秒 +1，PLC 用它判断网关存活 |
| 31 | `ERR` | 网关→PLC | 最近错误码，0=无，见下表 |

**坑：`POS_SET` 和 `TASK_CMD` 地址不相邻（10 与 5），必须分两次写。**
写成 `write_registers(10, [slot, 1])` 会把 `1` 写进只读的 `X_ACT`，
`TASK_CMD` 仍是 0 —— 表现为「主站连上了、心跳在跳、但龙门一动不动」，
而且不报错，只会一路等到超时。

`ERR` 错误码：

| 值 | 含义 | 处理 |
|---|---|---|
| 0 | 无 | — |
| 1 | `POS_SET` 越界 | 目标位不在 1..12，别把 40001 这类地址写进来 |
| 2 | 台面无料 | 先发 GOTO 上料，再发 HANDLE |
| 3 | 观察模式拒绝控制写 | 加 `allow_control:=true` 并停 `bt_runner` |
| 4 | HANDLE 时无有效识别结果 | `CAM_DATA` 不在 1..4；等 `CAM_STATE=2` 再发 HANDLE |

坐标是**有符号 16 位补码**存放：读 `0xFFF6` 应解释成 `-10 mm`。多数 PLC 软件选
`INT` 类型即可，选 `UINT` 会看到 65526。

### 线圈（0x，PLC 写，网关执行后自动清）

| 偏移 | 名称 | 含义 |
|---|---|---|
| 0 | `COIL_TRIGGER` | 上升沿 → 触发一次拍照 |
| 1 | `COIL_ATTACH` | 上升沿 → 吸盘吸附（**宏动作流程下不需要**，见 §8） |
| 2 | `COIL_DETACH` | 上升沿 → 吸盘释放（同上） |
| 3 | `COIL_RESET` | 上升沿 → 清计数与状态 |

### 离散输入（1x，只读）

| 偏移 | 名称 | 含义 |
|---|---|---|
| 0 | `DI_INPOS` | 轴已到位（可拍照） |
| 1 | `DI_CAM_DONE` | 拍照结果就绪 |
| 2 | `DI_HELD` | 吸盘持有物料（**逻辑标志，不是真空传感器**） |
| 3 | `DI_FULL` | 货架放满 |
| 4 | `DI_READY` | 控制就绪 |

---

## 6. 快速开始

### 6.1 观察模式（先跑这个）

PLC 只读，不改控制。**可以和决策层 `bt_runner` 同时跑**，适合先确认链路。

```bash
# 终端 1：起仿真（决策 + 视觉 + 控制 + 看板）
cd ~/AIC_cv-dev
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad

# 终端 2：起网关
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch aic_plc_gateway plc_gateway.launch.py

# 终端 3：软件 PLC 监视
python3 src/plugins/aic_plc_gateway/scripts/plc_master_demo.py --mode observe
```

终端 3 会打出：

```
HB=42 AX=MOVING CAM=DONE  data=3  412ms  X=  123 Y=  -15 Z=  45  货架=000000000001  1/12  ERR=0
```

### 6.2 接管模式（PLC 下发任务）

必须**关掉决策层**，否则两条指令流会打架。

```bash
# 终端 1：起仿真但不启动 bt_runner
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad start_decision:=false

# 终端 2：网关开接管
ros2 launch aic_plc_gateway plc_gateway.launch.py allow_control:=true

# 终端 3：软件 PLC 跑一轮
python3 src/plugins/aic_plc_gateway/scripts/plc_master_demo.py --mode control
```

### 6.3 接真实 PLC

1. PLC 与仿真主机同网段（或直连），确认能 ping 通；
2. PLC 侧建 Modbus TCP 客户端，目标 `仿真主机IP:5020`（`502` 需 root，不建议）；
3. 按 §5 建变量表；
4. 网关侧 `allow_control:=true`，并停 `bt_runner`。

---

## 7. PLC 侧怎么写

### 握手时序（control 模式）

```
PLC: POS_SET := slot ; TASK_CMD := 1        # 去某位
网关: → material_task(GOTO) → 龙门运动 → /sim/gantry_arrived
网关: CAM_STATE := BUSY                      # 到位待拍
视觉: → material_type → 网关 CAM_STATE := DONE, CAM_DATA := type
PLC: 读 CAM_DATA ; TASK_CMD := 2             # 按类型抓放
网关: → material_task(HANDLE) → 抓放 → /sim/handle_done
网关: SHELF_BITS 置位, TASK_STATE := DONE
```

### 结构化文本（ST，IEC 61131-3 风格）

```pascal
(* 自动流程：逐位巡视 + 拍照 + 抓放 *)
CASE step OF
0:  IF DI_READY AND NOT SHELF_FULL THEN
        POS_SET := slot; TASK_CMD := 1; step := 10;
    END_IF

10: IF AXIS_STATE = 2 THEN                  (* INPOS *)
        CAM_REQ := 1; step := 20;
    END_IF

20: IF CAM_STATE = 2 THEN                   (* DONE *)
        material := CAM_DATA;
        CAM_REQ := 0; TASK_CMD := 2; step := 30;
    ELSIF CAM_STATE = 3 THEN                (* FAIL *)
        (* 记录异常，跳到下一位 *)
        slot := slot + 1; step := 0;
    END_IF

30: IF TASK_STATE = 5 THEN                  (* DONE *)
        IF slot >= 12 THEN step := 90 ELSE slot := slot + 1; step := 0; END_IF
    END_IF

90: (* 放满，等待人工取料 *)
END_CASE
```

### 梯形图要点

- `TASK_CMD` 与 `CAM_REQ` 都是**脉冲型**：网关执行后自动清 0，PLC 侧不要持续置 1。
- 用 `DI_INPOS` 的上升沿触发 `CAM_REQ`，不要用电平（会重复拍）。
- 用 `HEARTBEAT` 做看门狗：2 s 不变即判定网关掉线，切安全态。

---

## 8. 与真机的差异（如实说明）

| 项 | 仿真 | 真机 |
|---|---|---|
| 吸盘 | `DetachableJoint` 刚性约束 | 真空泵 + 真空开关，会漏气、会吸不住 |
| 上料 | `set_pose` 瞬移 | 人工放检测台 / 输送带 |
| `DI_HELD` | 网关的逻辑标志 | 真空压力开关的真实输入 |
| `HANDLE` | 宏动作，一次调用做完全套 | PLC 逐动作编排（下探/吸/抬/移/放/断） |
| 精度 | 0.0 mm（无噪声无背隙） | 受伺服、机械、标定影响 |
| 碰撞 | 有（DART 物理） | 有，但会真撞坏 |

**别把仿真的 0.0 mm 当作真机精度报出去。** 这一点在汇报时最容易被追问。

另外：`COIL_ATTACH` / `COIL_DETACH` 在当前的宏动作流程下**不需要用**——控制器在
`HANDLE` 内部自己做了 attach/place/detach。它们保留是为了将来把 `HANDLE` 拆成
基本动作后使用（见 §2 差异 2）。

---

## 9. 换成真 PLC / RSC 要改什么

变量语义不变，只换传输层：

| 本实现 | PLCnext RSC 等价写法 |
|---|---|
| 读保持寄存器 0 | `rsc.read_variable("Camera.Request")` |
| 写保持寄存器 0 = 1 | `rsc.write_variable("Camera.Request", 1)` |
| 读保持寄存器 2 | `rsc.read_variable("Camera.Data")` |
| 读保持寄存器 10 | `rsc.read_variable("Pos")` |
| 读保持寄存器 11–13 | `rsc.read_variable("Axis.X.Actual")` 等 |
| 读离散输入 0 | `rsc.read_variable("Axis.X.InPos")` |

要改的只有 `ModbusTcpSlave` 这一层（约 200 行），`PlcGateway` 里的状态机、
ROS 话题订阅、位图逻辑全部不动。这也是当初把协议层单独抽出来的原因。

真机部署时还要注意（手册里的坑）：

- vPLC 是**容器，不可删除**；
- 算法容器访问 vPLC 必须用桥接 IP `10.88.0.2`，不是 `192.168.3.99`；
- 镜像要 **ARM64**（RK3568），x86 镜像跑不起来；
- 相机 SDK 也要 aarch64 版。

---

## 10. 排障

| 症状 | 原因 | 处理 |
|---|---|---|
| 主站报「连接被对端关闭」 | 从站空闲超时 < 主站扫描周期 | 已修：从站 30 s；主站间隔别超 30 s |
| 连不上，`Connection refused` | 网关没起 / 端口不对 | `ss -ltnp \| grep 5020`；看网关日志 |
| 读寄存器全 0 | 仿真没跑 / domain 不一致 | 两边 `ROS_DOMAIN_ID` 必须相同 |
| `POS_SET` 写了没反应 | 观察模式 | 加 `allow_control:=true`，且停 `bt_runner` |
| `ERR=3` | 观察模式收到控制写 | 同上 |
| `ERR=1` | `POS_SET` 越界（不在 1..12） | 检查 PLC 变量类型，别写成 40001 |
| `ERR=2` | 台面无料却要吸附 | 先 `TASK_CMD=1` 上料 |
| 坐标读出 65526 | 寄存器是 uint16，你按无符号读了 | 改成 `INT`（int16）解读 |
| 坐标永远 0 | 两处都容易错：话题名 + QoS | Gazebo 的关节话题是 `/world/aic_warehouse/model/aic_gantry/joint_state`（**不是** `/joint_states`），且发布端是 `BEST_EFFORT`；订阅端用默认 `RELIABLE` 会**静默不匹配**，不报错但一帧收不到 |
| 心跳在跳、龙门不动 | 主站把 `POS_SET` 和 `TASK_CMD` 一次性写进了相邻寄存器 | 分两次写：先 10 后 5（见 §5 的坑） |
| `TASK_STATE=5` 但货架没变化 | HANDLE 没带 `material_type`，控制器走「未识别」快速分支 | 确保发 HANDLE 前 `CAM_STATE=2` 且 `CAM_DATA=1..4`；网关已加 `ERR=4` 拦截 |
| 心跳不动 | 网关卡住 / 进程已死 | `pgrep -f plc_gateway`；看网关日志尾部 |
| 端口 502 绑定失败 | 低于 1024 需 root | 用 5020，或 `sudo setcap` |

---

## 11. 自测

```bash
# 协议层（不需要仿真）：起网关 + 主站读 3 次
bash scripts/test_plc_proto.sh

# 端到端（需要完整仿真，约 160 s）
bash scripts/test_plc_full.sh      # 12 块料、PLC 接管整轮
bash scripts/test_plc_one.sh       # 只跑 1 块，改完代码先跑这个，省时间
```

三个脚本都在 `scripts/` 下，会自己拉仿真、起网关、跑主站、收尾；端口可用环境变量 `PORT` 改。

### 实测基线（2026-10-21，M4）

| 项 | 结果 |
|---|---|
| 接管整轮 | **成功 12/12**，总耗时 **134.1 s** |
| 单轮节拍 | 到位 1.5–3.1 s、识别 0.27–0.32 s、抓放 9.2–13.0 s |
| 识别 | 12 块全部正确（与 `materials.types` 真值逐位一致） |
| 落位 | 四列各三层，按类型填满 L1→L3 |
| 末态 | `SHELF_BITS=111111111111`、`SHELF_FULL=1`、`PLACED_CNT=12` |
| 协议层 | 只读模式心跳递增、寄存器/线圈/离散输入读写正常 |

接管整轮比决策层整轮（145 s）略快，因为主站是「一次握手接着下一次」没有
行为树的 tick 周期；两者动作序列完全相同。
