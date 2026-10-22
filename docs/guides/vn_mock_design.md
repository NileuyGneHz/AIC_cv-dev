# VirtualNext mock 平台设计方案（vn_mock）

> 版本：2026-10-19 设计定稿，**未实现**（用户决策：暂不购买官方平台，只做 mock；录屏路线赛前再议）。
> 依据：`docs/competition/VirtualNext Smart开发平台使用说明.pdf`（协议事实）、
> `docs/competition/视觉逻辑算法应用赛规则-线上 (1).docx`（判分与录屏要求）。
> 适用技能：`aic-vn-mock`（操作手册）、`aic-official-sim`（协议来源）。

## 1. 目标与边界

**目标**：在 0 元预算下，让视觉算法、决策链、通信桥在开发期完成端到端联调，
不必等官方平台到手。

**边界（必须先看）**：

1. mock **不能用于省赛录屏**——2026-10-19 官方答疑允许「同类型数字孪生平台」拍摄视频，
   但 mock 是 2D 俯视、无三维场景与几何 1:1，不构成同类型平台；合规录屏走 Gazebo 或官方平台
   （见 `aic-official-sim` §6.1）；
2. mock 只在**协议层与行为层**逼近官方，3D 画面用 2D 俯视替代，时间模型用线性近似；
3. 官方未写明的细节（图像传输格式、端口、行结束符）一律做成**可配置**，
   平台到手后按实测校准，不改上层逻辑。

## 2. 官方拓扑与 mock 扮演的角色

官方三端（说明书 §3.2、§4.2）：

```text
[运动控制 C#] --client--> [VirtualNext Smart 相机服务器]   请求图像
[运动控制 C#] --client--> [Python 视觉框架 服务器端]       发 Camera:Trigger=True
[Python 视觉 客户端] --client--> [VirtualNext Smart 相机]  收图像
[Python 视觉] --reply--> [运动控制]                       回 Camera:Result={整数}
```

mock 需要扮演两方（可分别启动，也可同进程）：

| mock 组件 | 扮演 | 对端被测对象 |
|---|---|---|
| `MockCamera` | VirtualNext Smart 相机服务器 | 官方/自研视觉框架的客户端 |
| `MockPlc` | 运动控制（C# 框架的契约行为） | 视觉框架的服务器端 + 仿真内核 |

两种使用模式：

- **模式 1（测视觉）**：`MockCamera + MockPlc` ↔ 视觉算法——只验证 `process_image`
  分类正确率与响应时间；
- **模式 2（测决策）**：`MockCamera + MockPlc + SimCore` ↔ 决策链桥——验证 12 库位循环、
  判分与异常兜底。

## 3. 协议层设计

### 3.1 消息（逐字对齐说明书）

| 方向 | 报文 | 说明 |
|---|---|---|
| PLC → 视觉 | `Camera:Trigger=True` | 触发拍照 |
| VirtualNext → 视觉 | `Camera:Start=True` | 相机就绪 / 开始取图 |
| 视觉 → PLC | `Camera:Result={整数}` | 处理结果 |

错误码：`-1` 图像不完整、`-2` 连接超时、`-3` 通信错误、`-4` 连接拒绝。

### 3.2 未定义项与默认取值（**待平台实测校准**）

| 项 | 官方说明书 | mock 默认 | 备注 |
|---|---|---|---|
| 行结束符 | 未写 | `\n`（可配置 `\r\n`） | 用 `--eol` 切换 |
| 端口 | 未写 | 相机 6000 / 视觉 6001 | 用 `--camera-port` / `--vision-port` |
| 图像传输格式 | 只写「接收图像数据」 | 模式 A：JPEG 字节流 + 4 字节长度前缀；模式 B：文件路径字符串 | **最大未知**，两种都实现，实测二选一 |
| 握手 / 保活 | 未写 | 无，连接即可用；30s 无数据判定超时 | 保守 |

### 3.3 协议编解码接口

```python
class Protocol:
    TRIGGER = "Camera:Trigger=True"
    START   = "Camera:Start=True"
    ERR_INCOMPLETE, ERR_TIMEOUT, ERR_COMM, ERR_REFUSED = -1, -2, -3, -4

    @staticmethod
    def encode_result(value: int) -> bytes: ...      # b"Camera:Result=3\n"
    @staticmethod
    def parse_result(line: bytes) -> int: ...        # 反向解析，含错误码校验
```

## 4. 仿真内核（SimCore）

### 4.1 状态

| 状态 | 取值 | 来源 |
|---|---|---|
| 轴位置 `x/y/z` | X 0–300，Y 0–300，Z 0–200 | 说明书 §2.2.3 |
| 速度档 | 1–5 | 同上 |
| 吸盘 | off / on | `SuctionEnable` / `SuctionEnabled` |
| 库位 | 4 列 × 3 行 = 12，列对应四类缺陷 | 说明书 §2.2.2 |
| 位点状态 | empty / occupied(correct) / occupied(wrong) | 判分规则 |
| 物料队列 | 待抓物料序列（类别 + 图） | `data/samples/` 轮转或指定 |

### 4.2 判分（严格对齐官方）

1. 落到某库位：类别与**该列**一致 → correct（绿）；不一致 → wrong（红），**不得分**；
2. 该库位已 correct 后再放任何物料 → 判 wrong（堆叠判错）；
3. 出库位统计：`correct_count`（0–12）、`wrong_count`、`elapsed_sec`——
   对齐评分「5 分钟内正确放置数量」口径。

### 4.3 时间模型

线性近似：`耗时 = 距离 / (基础速度 × 档位系数)`，Z 轴单独系数；吸盘动作固定 0.3s。
目的只是让「5 分钟跑 12 个」的量级可评估，不追求与真机一致。

### 4.4 接口

```python
class SimCore:
    def __init__(self, layout_path: str, sample_dir: str): ...
    def reset(self) -> None: ...
    def move(self, axis: str, position: int, speed: int) -> bool: ...   # 越界回 False
    def suction(self, enable: bool) -> None: ...
    def place(self) -> PlacementResult: ...                             # correct/wrong/stacked
    def stats(self) -> Stats: ...                                       # 计数 + 用时
    def snapshot(self) -> SimSnapshot: ...                              # 给 viewer
```

## 5. 可视化（Viewer）

2D 俯视，OpenCV 绘制，1000×500 场地（等比缩放）：

- 底图：置物平台（左上）、货架 4×3（右侧）、轴十字线与末端吸盘；
- 物料：圆形铁片缩略图（缺陷可见，便于人工核对）；
- 判定着色：correct 绿 / wrong 红 / 空 灰；
- HUD：`correct/12`、`wrong`、`elapsed`、当前位点、最近一次 Result 值。

不追求 3D，只替代「看得见在跑什么」，同时便于截图进技术文档（2D 俯视不构成官方答疑所说的
「同类型数字孪生平台」，**不能用于比赛录屏**）。

## 6. 模块划分（实现时按此落盘）

```text
tools/vn_mock/
├── __init__.py
├── protocol.py        # 报文编码/解码、错误码、可配置 EOL
├── camera_server.py   # MockCamera：等客户端连入，Camera:Start → 回图
├── plc_client.py      # MockPlc：连视觉服务器端，Trigger → 等 Result
├── sim_core.py        # 三轴/吸盘/库位/判分/时间模型
├── viewer.py          # 2D 俯视渲染
└── main.py            # CLI 组装：--mode vision|decision，--camera-port 等
scripts/
├── run_vn_mock.sh     # WSL2/Linux
└── run_vn_mock.ps1    # Windows
```

**不改动** `src/decision/**` 与现有 `scripts/*.py`；决策链通过既有 vision/control
桥（`docs/guides/decision_vision_interface.md`、`decision_control_interface.md`）接入。

## 7. 验收标准（转「可用」判据）

1. 视觉框架模板（官方到手后）直连 `MockCamera`，完成 `Trigger → 图 → Result` 一轮；
2. 决策链经 `MockPlc + SimCore` 跑满 12 库位，`scripts/check_decision_run.py` 断言通过；
3. 异常注入：Result 返回 −1…−4、连接静默超时——决策侧 TIMEOUT 兜底不中断（对齐自验场景 B）；
4. 判分与官方规则一致：放错不得分、正确后堆叠判错、correct/12 与用时统计正确。

## 8. 风险与坑

1. **图像传输格式未定义**是最大未知——两种模式都实现，避免押错；
2. 端口/行结束符同样未定义，必须可配置，否则平台到手后返工；
3. mock 录像不能提交——官方答疑放行的是「同类型数字孪生平台」（三维场景 + 几何一致），
   2D 俯视 mock 不满足；文档与代码注释都要写明，合规录屏另走 Gazebo / 官方平台；
4. 官方 C# 运动框架依赖 Windows/.NET，若要完整复刻控制侧，mock 在 Windows 跑；
   纯 Python 版跨平台，优先保证与视觉框架联调；
5. 时间模型是近似值，不能拿 mock 耗时当比赛成绩预估。

## 9. 工时估算（供排期）

| 阶段 | 内容 | 预估 |
|---|---|---|
| 1 | `protocol.py` + `camera_server.py`（两种图像模式） | 0.5 天 |
| 2 | `sim_core.py`（判分 + 时间模型） | 0.5 天 |
| 3 | `viewer.py`（2D 渲染 + HUD） | 1 天 |
| 4 | `main.py` + 脚本 + 联调（含决策链接入） | 0.5–1 天 |
| 合计 | | **2.5–3 天** |
