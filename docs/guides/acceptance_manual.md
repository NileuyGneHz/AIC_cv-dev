# AIC 仿真 / PLC 验收手册

> 版本：2026-10-21 · 对应 M4（来料架 3×4 → 置物台识别 → 分拣货架 4 列×3 层）
> **这份手册只解决一个问题：怎么用最短的路径确认「它现在确实是好的」，以及坏了先看哪里。**
> 装环境、改参数、调机位看 `simulation_manual.md`；PLC 协议细节看 `plc_integration.md`；
> 原理与 29 个坑看 `simulation_guide.md`。

---

## 0. 五级验收阶梯

从便宜到贵排。**任何一级失败，先修完再往下**，不要跳级——后面的失败多半是前面的残留。

| 级别 | 验什么 | 命令 | 耗时 | 通过标志 |
|---|---|---|---|---|
| L1 | 全部包能编过 | `./scripts/build.sh` | 10–120 s | colcon 无 `error:` |
| L2 | Modbus 从站能连、心跳在走 | `PORT=5031 bash scripts/test_plc_proto.sh` | 6 s | `HB=` 数字递增 |
| L3 | RSC 后端握手正确 | `bash scripts/test_plc_rsc.sh` | 1–3 s | `全部通过：RSC 后端整轮握手 OK` |
| L4 | 仿真本体：决策 / 视觉 / 抓放 / 落位 | `./scripts/verify_sim_m2.sh --domain auto` | 165 s | `PASS：决策 12/12 OK ...` |
| L5 | PLC 真正接管整轮 + 物理落位 | `bash scripts/test_plc_full.sh` | 150 s | `汇总：成功 12/12` 且 `PASS：主站 12/12` |

一条命令跑完 L1–L4（推荐，结果自动汇总成表）：

```bash
cd ~/AIC_cv-dev
./scripts/verify_all.sh              # 约 3 分钟
./scripts/verify_all.sh --with-plc   # 追加 L5，约 6 分钟
```

实测输出长这样：

```
================================================================
 AIC 仿真 / PLC 验收   2026-10-21 23:34:33
 工作区：/home/nileuy/AIC_cv-dev
================================================================
  L1  PASS    1s  构建 —— 7 个包编过
  L2  PASS    6s  协议层 Modbus —— 心跳 4→5
  L3  PASS    1s  RSC 干跑整轮 —— 21 项断言全过
  L4  PASS  141s  仿真基线 12 轮 —— 决策 12/12 OK、12 块全部落格
----------------------------------------------------------------
 结果：4/4 通过，总耗时 2 分 28 秒
```

退出码 0 = 全过，1 = 有失败（末尾会列出哪级、证据是什么），2 = 环境问题。

---

## 1. 环境：两行必须记住的东西

```bash
cd ~/AIC_cv-dev
source /opt/ros/humble/setup.bash     # 顺序不能反
source install/setup.bash
```

`verify_all.sh` 与 `test_plc_*.sh` 内部已经 source 过，**只有你手敲 `ros2 launch` / `ros2 run` 时才需要自己 source**。

两个真会咬人的点：

1. **`install/` 被清过**（跑过 `rm -rf build install log`）会报 `Package 'aic_dashboard' not found`。
   这时报错里的搜索路径只有 `/opt/ros/humble`——看到这个就是没 source，不是包丢了。
2. **`#!/usr/bin/env python3` 可能命中没装 cv2 的解释器**。本机有两个 3.13/3.10，
   视觉节点起来就 `ModuleNotFoundError: cv2`，而下游表现成「等待视觉结果超时」。
   测试脚本里都写了 `export PATH=/usr/bin:$PATH`；你手敲 `ros2 launch` 时若遇到这个症状，
   先 `which python3` 看是不是 `/home/nileuy/.workbuddy/...`。

---

## 2. L1 构建

```bash
./scripts/build.sh                                   # 增量
./scripts/build.sh --packages-select aic_plc_gateway # 只编一个包
```

通过标志：末尾若干行 `Finished <<< <包名>`，没有 `Failed <<<`。

失败先看是不是这三类：

| 报错 | 原因 |
|---|---|
| `error writing to /tmp/ccXXXX.s: No space left` | `/tmp` 只有 10 MB。`build.sh` 已用 `-DCMAKE_CXX_FLAGS=-pipe` 规避；自己手敲 colcon 时要带上 |
| `Package 'xxx' not found` | 没 source `/opt/ros/humble` |
| `Could not find a package configuration file ... aic_msgs` | 先编 `aic_msgs`：`--packages-select aic_msgs`，它没有依赖 |

---

## 3. L2 协议层（不需要仿真）

```bash
PORT=5031 bash scripts/test_plc_proto.sh
```

它在 5031 端口起一个 Modbus 从站，然后用软件主站读 3 次。**期望看到**：

```
[plc_gateway.py-1] [INFO] [plc_gateway]: PLC 网关就绪：modbus tcp 0.0.0.0:5031 unit=1，模式=观察（只读，可与 bt_runner 共存）
=== 主站读 3 次（observe） ===
已连接 127.0.0.1:5031，网关心跳=4

[23:33:57] HB=4 AX=IDLE   CAM=IDLE  data=0    0ms  X=    0 Y=    0 Z=   0  货架=000000000000  0/12  ERR=0
[23:33:57] HB=4 AX=IDLE   CAM=IDLE  data=0    0ms  X=    0 Y=    0 Z=   0  货架=000000000000  0/12  ERR=0
[23:33:57] HB=5 AX=IDLE   CAM=IDLE  data=0    0ms  X=    0 Y=    0 Z=   0  货架=000000000000  0/12  ERR=0
```

判定只看一件事：**`HB=` 的最后一个数比第一个大**。坐标全 0 是对的（没仿真，龙门没动）。

心跳不动 = 网关定时器没跑或进程死了：

```bash
pgrep -af plc_gateway          # 有没有进程
tail -20 logs/plc_gw.log       # 有没有 traceback
```

`Address already in use` = 上一次的网关没退干净，`pkill -f plc_gateway` 后再跑，或换 `PORT=5032`。

---

## 4. L3 RSC 干跑整轮（不需要 vPLC）

```bash
bash scripts/test_plc_rsc.sh
```

单进程里同时扮演 PLC、控制器、视觉三方，用内存假 PLC 跑完一整轮握手。**期望末尾**：

```
  21 项 OK ...
全部通过：RSC 后端整轮握手 OK
```

判定以 `全部通过：RSC 后端整轮握手 OK` 为准（断言条数会随脚本演进，当前 21 条）。要害几条：

| 断言 | 防的是什么 |
|---|---|
| `GOTO` 后 `TASK_STATE=1` | 执行位写了但状态机没接 |
| `HANDLE` 携带 `material_type=2` | 类型丢了会走「未识别」快速分支，**不动也回成功** |
| `CAM_DATA` 回写 | 视觉结果没落到寄存器 |
| `SHELF_BITS` 置位 | 位图算错，货架位与实际不符 |
| `-10 mm` 仍是 `-10` | 负坐标被折成 `65526`（换 RSC 后最容易犯） |
| 心跳递增 | 定时器没跑 |

**这条过了只能说明映射与状态机对，不能说明现场 PLC 变量名对。** 后者要真机，见 §8。

---

## 5. L4 仿真基线（真视觉 12 轮）

```bash
./scripts/verify_sim_m2.sh --domain auto
```

`--domain auto` 会扫进程表找一个没被占用的 ROS domain，**同机开着别的仿真也不会互相污染**。
只有确认没人用某个号时才写死 `--domain 7`。

**期望末尾**（这是唯一需要逐字对照的地方，以下为 2026-10-21 实测）：

```
model                         x        y        z   dx(mm)   dy(mm)
material_01             -0.3750   0.3100   0.1030      0.0      0.0  (L1)
material_05             -0.1250   0.2000   0.1630     -0.0     -0.0  (L2)
material_09              0.1250   0.0900   0.2230     -0.0     -0.0  (L3)
（共 12 行，四列 × 三层）

PASS：决策 12/12 OK、夹持与释放各 12 次、12 块铁片全部落在对应类型的格内
[verify] 通过。产物：logs/verify_m2_20261021_233747.csv
```

三层的 `y` 与 `z` 必须各不相同（实测 L1 0.310/0.103、L2 0.200/0.163、L3 0.090/0.223）。
**如果两层出现相同的 (x, y)，说明阶梯货架退化成垂直堆叠**——吸盘会撞上层托盘，
表现为 z 卡住 60–87 mm，不是视觉问题。

`x` 只有四个值（−0.375 / −0.125 / 0.125 / 0.375，列距 0.25），
正好对应四个类型列的列心；出现别的值就是落位算错。

三组断言分别管什么：

| 组 | 断言 | 失败含义 |
|---|---|---|
| A | A1 结果 12 行 / A2 全 `OK` / A3 类型与真值一致 | 视觉判错、或决策超时 |
| B | B1 夹持 12 次 / B2 释放 12 次 | 吸盘没吸住，或最后一块卡在吸盘里 |
| C | C3 x 偏格心 / C4 y 偏层深线 / C5 z 在托盘区间 | 落位算错，或层位被占 |

`dx/dy` 全是 `0.0` 是正常的（仿真里几何是精确的）。**真机不会有这个数字，别把它当精度指标。**

机器慢导致等不到结果时放宽上限，不要改判定：

```bash
./scripts/verify_sim_m2.sh --domain auto --timeout 300
```

想看「视觉判错」和「控制/物理问题」分开，用真值桩跑一遍：

```bash
./scripts/verify_sim_m2.sh --vision gt --domain auto
```

真值桩也失败 = 与视觉无关，问题在控制或物理。

---

## 6. L5 PLC 接管整轮

```bash
bash scripts/test_plc_full.sh      # 12 块，约 150 s
bash scripts/test_plc_one.sh       # 1 块，约 30 s（改完代码先跑这个）
```

这两个脚本会**清掉本机所有仿真进程**（不是只杀 Gazebo），**别在录制或演示时跑**。

**期望末尾**（2026-10-21 实测）：

```
汇总：成功 12/12，总耗时 129.4 s，货架 111111111111（12/12），放满标志=1
MASTER_RC=0
[plc] 物理落位核对 ---------------------------------------------------
model                         x        y        z   dx(mm)   dy(mm)
material_01             -0.3750   0.3100   0.1030      0.0      0.0  (L1)
（共 12 行）
PASS：主站 12/12 OK、夹持与释放各 12 次、12 块铁片全部落在对应类型的格内
```

**为什么要多这一段落位核对**：网关的 `SHELF_BITS` 是「收到 detach 事件」累出来的，
不是真实坐标。控制器在「到位超时」时会打印 `按到位继续` 并照样放料——一次 RTF 抖动
就能把料放在格口外，而 PLC 侧仍显示 `111111111111`。所以 L5 结束前会用与决策路径
同一套断言核对 `pose/info`，`dx/dy` 必须都在 4 mm 以内。

判定四个数：

| 数 | 含义 | 不对时 |
|---|---|---|
| `成功 12/12` | 12 轮握手都收到了预期状态 | 少的那几轮看 `logs/plc_full_gw.log` 的 `ERR=` |
| 单轮 ≈11 s（总 129–160 s） | 1.5–3 s 到位 + 0.3 s 识别 + 9–13 s 抓放 | 0.3 s/轮 = 走了「未识别」快速分支；30 s+/轮 = 有第二个 Gazebo 实例 |
| `111111111111` | 12 个格位都置位 | 与 `placed` 计数不符 = 位图逻辑问题 |
| `PASS：主站 12/12 ...` | 12 块料的**实际坐标**都在对应格内 | `C3/C4` = 落位偏了，先查日志里的 `到位超时` |

**单轮时间是最灵敏的指标**：`0.3 s` 意味着控制器没动就回了 `handled`，一定是 `HANDLE` 没带
`material_type`。网关现在会前置拦截并报 `ERR=4`，但如果你绕开网关直接发话题，这个坑还在。

---

## 7. 人工确认（脚本判不了的部分）

脚本能判数字，判不了「画面是不是对的」。开看板看一眼，四项：

```bash
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad
# 浏览器 127.0.0.1:8080
```

| 检查项 | 应该看到 | 不对说明 |
|---|---|---|
| 左上有实时画面 | 正俯视，能同时看到来料架 3×4 与分拣货架 4×3 | 机位参数被改过 |
| 来料架逐格变空 | 每轮少一块，12 轮后全空 | 上料没按格位取 |
| 分拣架逐格填满 | 四列各三层，颜色按类型聚在一起 | 类型判错或层位抢用 |
| 12 个判分格变绿 | 每格显示类型与耗时，无红 | 看 `TIMEOUT` 还是 `WRONG` |

录制一镜到底的流程在 `obs_recording_playbook.md`，这里不重复。

---

## 8. 没有硬件时验不了的三件事

这三条不要写成「已验证」，汇报时也要分开说：

| 项 | 为什么验不了 | 有机器后怎么验 |
|---|---|---|
| RSC 连真 vPLC | 沙箱没有 PLCnext，也没有 192.168.3.99 | `plcnext_rsc_setup.md` §5 五步联调：先只读、再单点写、最后握手 |
| PLC 侧 `main.st` 编译 | 没有 PLCnext Engineer | 在 Engineer 里新建工程 → 粘贴 `docs/plc/main.st` → 编译，看是否只有「变量未声明」类错误 |
| 海康相机取流 | 没有相机，也没有 aarch64 版 MVS SDK | 按 `AGENT_SIM_MANUAL.md` T09/T10 走 |

---

## 9. 验收记录表（复制到汇报里填）

| 日期 | L1 | L2 | L3 | L4 | L5 | 总耗时 | 备注 |
|---|---|---|---|---|---|---|---|
| 2026-10-21 | PASS 7 包 | PASS 心跳 4→5 | PASS 21 项 | PASS 12/12、0.0 mm | PASS 12/12、0.0 mm、129.4 s | 约 5 min | M4 基线 |

每级留证据的方式：

```bash
# 一条命令留下的东西
ls logs/acc_*.log                      # L1–L5 的完整输出
ls logs/verify_m2_*.csv                # L4 的逐位点结果（Excel 可直接开）
grep "汇总" logs/acc_plc.log           # L5 的最终计数
```

---

## 10. 出问题时的定位顺序

**先分清是哪一类假象，再动手。** 这三类各骗过我一次：

| 症状 | 大概率不是 | 实际原因 | 一条命令确认 |
|---|---|---|---|
| 「等待视觉结果超时」`ERR=0` | 网关 / 协议 / 后端 | 视觉节点 `import cv2` 失败已退出 | `pgrep -f vision_node`；`grep -i "cv2\|ModuleNotFound" logs/*.log` |
| 网关连上、心跳在跳、龙门一动不动 | 仿真卡了 | 主站把 `POS_SET(10)` 和 `TASK_CMD(5)` 当相邻寄存器一次写 | `grep "PLC ->" logs/plc_*_gw.log`——没有这行就是没触发 |
| `TASK_STATE=5` 但货架没变化 | 抓取机构问题 | `HANDLE` 没带 `material_type`，走「未识别」分支 | 看单轮耗时是不是 0.3 s |
| 某一轮突然 30 s+，日志有 `到位超时 8.0 s` | 机器慢、代码回归 | **有第二个 Gazebo 实例在跑**，两个物理世界抢同一批关节指令 | `ps -eo args= \| grep -c "[i]gn gazebo"` 必须等于 **1** |

最后一条值得展开，因为它最隐蔽：

**`pkill -f gzserver` 在本栈上是空操作。** Fortress 的进程名是 `ign gazebo -s -r ...`，
`gzserver` 是 Gazebo classic 的名字。旧脚本用它做「跑前清场」，看起来执行了、实际一个
进程都没杀掉。于是上一轮的实例继续跑、新实例照起，两套世界订阅同一批 `/clock`、
`/aic/joint/*/cmd_pos`，互相抢关节指令。表现是 RTF 掉到十分之一、轴「到位超时」、
落位偏移几十毫米——**而 PLC 侧仍报 12/12**，因为货架位图是 detach 事件累出来的。

现在统一用 `scripts/lib/sim_cleanup.sh`：

```bash
source scripts/lib/sim_cleanup.sh
sim_kill_stale          # 按进程名清干净（排除调用者自己）
sim_count_gazebo        # 应为 0；跑起来后应为 1
```

如果你手敲命令排查，先确认实例数再下结论：

```bash
ps -eo args= | grep -c "[i]gn gazebo"   # 0 = 没起；1 = 正常；2 = 有残留，先清
```

这里特意不用 `pgrep -f "ign gazebo"`：它会匹配到**你自己这条命令**的命令行
（里面就写着这个字符串），于是永远多出 1–2 个，看起来像有残留。
方括号写法 `[i]gn` 让正则匹配 `ign`、而命令行里字面是 `[i]gn`，就不会自匹配。

排障的通用三步，按顺序做，不要跳：

1. **看进程**：`pgrep -af "gzserver|plc_gateway|vision_node"` —— 该在的不在，就是崩了。
2. **看日志尾部**：`tail -30 logs/<最近的那个>.log` —— 崩了会有 traceback；没崩会停在某一步。
3. **看数字对不对**：`HB` 有没有涨、`TASK_STATE` 走到几、单轮耗时多少。

还判不了，把这三样贴出来就能定位：`logs/acc_*.log` 对应那一级、`pgrep` 的输出、失败那一轮的时间戳。

---

## 11. 这份手册与其它文档的分工

| 文档 | 什么时候看 |
|---|---|
| **本文** | 想确认「现在好不好」、验收、出问题先分哪一类 |
| `simulation_manual.md` | 想知道参数怎么改、机位怎么调、模式有哪些 |
| `simulation_guide.md` | 想懂原理，或撞上了 29 个坑里的某一个 |
| `plc_integration.md` | 改网关、加协议、对寄存器、写 PLC 握手 |
| `plcnext_rsc_setup.md` | 手上真有一台 vPLC，要现场联调 |
| `obs_recording_playbook.md` | 要录省赛视频 |
| `AGENT_SIM_MANUAL.md` | 交接给另一个人（或另一个 agent） |
