# ADR 0002 — PLC 侧数据契约：保留自定义 `Camera` 5 字段，官方的 `Cam` / `Rsc_xExecute` 作为现场分支

- 状态：已接受（现场触发条件见文末）
- 日期：2026-10-21
- 关联：`docs/guides/plcnext_rsc_setup.md` §2、`docs/guides/plc_integration.md` §5、
  `docs/plc/main.st`、`src/plugins/aic_plc_gateway/config/plc_rsc_map.yaml`

## 背景

官方两份文档给的 PLC 数据契约**不是同一套**：

| 来源 | 形态 |
|---|---|
| 【手册】§3.1 | `Cam` 结构体 8 字段：`CommOK / Ready / Start / Done / Reset / Prog / ErrCode / Data`（`Data=100` 表示拍照失败） |
| 【RSC】§2 | `Rsc_xExecute`（GVL 全局，无程序名前缀）+ `imageClassify.image_result`（UDT，字段 `index`） |

两者字段名、握手方式、甚至变量路径形态都不同（一个有程序名前缀、一个是全局）。
本仓库现有的 `plc_rsc_map.yaml` 是第三套：`Camera{Request, State, Data, Ms, Err}`。

必须选一套当主线，否则 `plc_gateway.py` 的握手判据会写成三不像。

## 决策

**默认用自定义 `Camera` 5 字段**（`Request / State / Data / Ms / Err`），
官方的两代契约作为现场分支，不在默认路径上。

理由三条：

1. **握手语义更贴合本项目的时序。** 官方 `Cam` 是 `Start`/`Done` 两个 BOOL 的脉冲握手，
   要求 PLC 与算法严格互锁；我们用 `State` 枚举（0=IDLE 1=BUSY 2=DONE 3=FAIL），
   网关侧一个寄存器就能表达，仿真里自检也简单（`rsc_round_test.py` 直接断言 `State==2`）。
2. **`Cam` 的 `Data=100` 表示失败**是「哨兵值」写法，容易和物料编号撞语义边界；
   我们用 `State=3` + `Data=0` 分开表达，错误码另有 `Err` 字段。
3. **现场大概率能改 PLC 程序。** VC 平台本来就带 PLCnext Engineer，
   队伍要自己写巡视逻辑，顺手把变量声明成这套 5 字段成本很低（`main.st` 已验证可编译）。

## 代价

- 如果国赛现场 vPLC 程序**不可改**（出厂固化了 `Cam` 或 `Rsc_xExecute`），
  本仓库默认契约直接用不了，必须走下面的分支。
- 这个风险不能靠「应该能改」赌，所以把它写成一个**有明确触发条件的待办**（见文末）。

## 现场分支（不改协议层）

真机第一步是探变量（`AGENT_SIM_MANUAL.md` T11）。探完按结果二选一：

| 探到的变量 | 动作 | 改动范围 |
|---|---|---|
| 有 `Main.Camera.*`（我们这套，或队伍自己建的） | 什么都不用做 | — |
| 只有 `Main.Cam.*`（官方第一代） | 新增 `config/plc_rsc_map_cam.yaml`（8 字段），`plc_gateway.py` 里加一个 `CamHandshake` 分支：`Start` 上升沿 → 触发视觉；结果写 `Done:=TRUE` + `Data` | 只在 `_on_*` 回调与判据，`IoBackend` 不动 |
| 只有 `Rsc_xExecute` + `image_result` | 新增最小映射表；`Rsc_xExecute` 上升沿触发，`image_result.index` 当结果 | 同上，另加一个 UDT 映射（`structs:` 段已有支持） |

三种情况下**寄存器/变量语义映射表都是 YAML**，状态机与话题契约不变——这正是
`IoBackend` 抽出来的目的（见 ADR 前一次重构 `08bf6a6`）。

## 触发条件（什么时候回来改这份 ADR）

满足任意一条就回来落地对应的现场分支：

1. 拿到 vPLC 实机并完成 T11 探变量；
2. 官方通知明确「PLC 程序由赛方提供、不得修改」；
3. 队伍决定直接用赛方工程做国赛，不自写 PLC 程序。

在此之前，默认路径（5 字段 + `main.st`）是自洽且可验证的：
`rsc_round_test.py` 21 项断言、Modbus 接管整轮 12/12。
