# PLC 侧程序（骨架）

> 面向：要在 PLCnext Engineer 里写/改 vPLC 程序的人。
> 配套：`docs/guides/plc_integration.md`（网关侧）、`docs/guides/plcnext_rsc_setup.md`（变量声明与联调）。

## 这里有什么

| 文件 | 内容 |
|---|---|
| `main.st` | `PROGRAM Main`：12 位点巡视状态机 + 心跳看门狗，**可编译的骨架** |
| `README.md` | 本文件：怎么用、三处必须补的真机逻辑、三个官方 FB 的真实用法 |

`main.st` 只放 PROGRAM，`udt_Camera` / `udt_Axis` / `udt_Task` / `udt_Shelf` /
`udt_Diag` / `udt_Cmd` / `udt_Di` 的声明在 `plcnext_rsc_setup.md` §2.1，逐字照抄即可。
变量名与 `src/plugins/aic_plc_gateway/config/plc_rsc_map.yaml` 的 `path` 一一对应。

## 怎么用

1. PLCnext Engineer 新建工程 → 建 §2.1 的 7 个 UDT + `PROGRAM Main`；
2. 把 `main.st` 粘进 Main，编译（应无 error）；
3. 下载 → 重新示教（**下载会清示教位置**，见 `plc_integration.md` 的坑）；
4. 按 `plcnext_rsc_setup.md` §5 五步联调：干跑 → 只读 → 单点写 → 心跳 → 握手闭环。

## 两个不能省的设计点（`main.st` 里已实现）

**1. 执行位要「保持到被接受」，不能发一个扫描周期就撤。**
网关轮询 50 Hz（20 ms），PLC 扫描 10–20 ms，一个周期的脉冲可能正好落在两次轮询之间被漏掉。
`main.st` 里 `Task.Cmd` 保持到 `Task.State` 变成预期值才清 0。

这也顺带解决了 Modbus 与 RSC 的行为差异：Modbus 下网关自己把 `Task.Cmd` 清 0；
RSC 下这个变量归 PLC，网关写不动，必须 PLC 清。同一段逻辑两边都对。

**2. `Camera.Request` 不要用。**
本项目的网关在「轴到位」时自动触发视觉（`_on_arrived` → `/vision/trigger`），
结果由网关写进 `Camera.State / Camera.Data`。PLC 再拉 `Camera.Request` 会重复触发一次识别
（网关的 `CAM_REQ` 分支只在 `allow_control:=true` 时生效，但两条路同时开会打乱
`CAM_STATE` 的 BUSY 计时）。

## 上机前必须补的三件事

`main.st` **不含**这些——它们依赖现场接线与安全规范，不能照抄：

| 缺什么 | 为什么必须补 | 建议 |
|---|---|---|
| 伺服使能 / Ready 检查 | 骨架只管发位置，不管轴能不能动 | 进 `state:=10` 前检查三轴 `ServoReady`，不满足直接 FAULT |
| 软限位 | 官方行程 X/Y 0–300、Z 0–200（VirtualNext 手册口径） | 每轴加 `LMX/LMN` 比较，越界立刻停 |
| 急停链路 | 骨架里 `Stop` 只是个 BOOL，真机要接硬件急停回路 | 急停走硬件，软件只做「检测到急停 → state:=90」 |

## 三个官方 FB 的真实用法（PLCnextBase 1.6.2）

FB 名是从 `PLC-softwares/PLCnextBase_1.6.2.0/` 的包内容里核过的，**接口参数以同目录
`PLCnextBase_1_6_2_FBFun.chm` 为准**（这里不抄签名，避免抄错）。

| FB | 在本题里的用途 | 注意 |
|---|---|---|
| `PBCL_SysGetMicrotick_` | 测「拍照请求 → 结果回来」的耗时，写进 `Camera.Ms` 做现场性能证据 | 微秒计数，两次相减后除 1000 转 ms；与网关写的 `Camera.Ms` 二选一，别两边都写 |
| `PBCL_FileCsvWrite_` | 把每格的 `slot / Camera.Data / 时间 / 结果` 落成 CSV，作为比赛结果的过程证据 | 路径要在 vPLC 容器可写目录；文件句柄用完关，否则下次写失败 |
| `PBCL_ConvElementaryToArray_` | 需要按字节看 `Shelf.Bits` 时才用 | **补码不需要转换**：坐标声明成 `INT` 本来就是有符号，声明成 `WORD` 才会读出 65526 |

## 与网关的职责边界

| 谁 | 负责 |
|---|---|
| PLC（本程序） | 巡视顺序、下发 `Pos`/`Task.Cmd`、看门狗、故障停机 |
| 网关 | 协议转换、握手状态、把 `Task.Cmd` 翻成 ROS 的 `GOTO`/`HANDLE`、把 `Camera.Data` 装进任务 |
| 决策层（`bt_runner`） | **PLC 接管时不启动**（`start_decision:=false`），否则两条指令流打架 |

也就是说：**同一时刻只能有一个「大脑」**。要么行为树，要么 PLC。
