# PLCnext RSC 接入手册（算法侧 ↔ vPLC）

> 版本：2026-10-21 首版（2026-10-22 校正 vPLC 凭据与版本）
> 适用：`src/plugins/aic_plc_gateway/` 的 RSC 传输层（`rsc_link.py` + `config/plc_rsc_map*.yaml`）
> **VC 版设备先读**：`vplc_deployment.md`（镜像 arm64、Python 3.10、官方 `Cam` 结构体、部署五步）
> 前置阅读：`plc_integration.md`（变量语义与 Modbus 映射表，本文不重复）
> 本文补上两份 YAML 里指向的 `docs/guides/plcnext_rsc_setup.md`（此前是悬空引用）。

---

## 0. 一句话

**RSC 是官方路径，Modbus 是没有 vPLC 时的替身。** 两者变量语义完全相同，
差别只在传输层：RSC 按「变量名」读写 GDS，Modbus 按「寄存器偏移」读写。

```
海康相机 --\                                    /-- PROFINET --> AXL 耦合器 --> ESTUN 伺服 x3
            +--> 算法侧(Python) --RSC/TCP 41100--+
PLC 变量树 --/                                    \-- IEC 61131-3 程序（时序、互锁）
```

---

## 1. 现场参数（先核对这张表）

| 项 | 值 | 来源 / 备注 |
|---|---|---|
| PLC IP（宿主机视角） | `192.168.3.99` | 官方 demo `main.py:28` |
| PLC IP（容器内视角） | `10.88.0.2` | `plc_integration.md` §9；**容器里必须用桥接 IP** |
| RSC 端口 | `41100` | `PyPlcnextRsc/common/device.py:161` 默认值 |
| 传输加密 | `useTls=True` | `device.py` `ExtraConfigure` 默认开；PLC 侧关了会握手失败 |
| 保活 | `keepAlive_ms=30000` | 同上 |
| 认证 | vPLC：`admin` / **`plcnext`**；EPC：`admin` / 机身密码 | 部署指导 p2、设备手册 5.3；demo `main.py:28` 的 `381d3976` 是 EPC 机身密码 |
| 变量路径 | `Arp.Plc.Eclr/<POU>.<变量>[.<字段>]` | demo `main.py:36` 用的是 `Arp.Plc.Eclr/imageClassify.image_result` |
| POU 名 | 本项目假设 `Main`，**待现场确认** | demo 里官方工程叫 `imageClassify`，两者不同名 |

`<POU>` 是 PLC 工程里的程序名，不是项目名。**换 PLC 工程只改 YAML 的 `program:` 一行**，
`rsc_link.py` 的 `_resolve_path()` 会把 `{program}` 替换掉。

---

## 2. PLC 侧要建什么变量

`plc_rsc_map.yaml` 里 26 个标签全部落在 `Main` 程序的 8 个结构体变量上。
PLC 侧声明必须**逐字一致**（大小写、字段名都算），否则读回 `NotExists`、写入 `TypeMismatch`。

### 2.1 类型声明（ST）

```pascal
TYPE
    udt_Camera : STRUCT
        Request : BOOL;      (* PLC -> 算法：请求拍照，算法取走后清 FALSE *)
        State   : INT;       (* 算法 -> PLC：0=IDLE 1=BUSY 2=DONE 3=FAIL *)
        Data    : INT;       (* 算法 -> PLC：物料编号 1..4，0=未识别 *)
        Ms      : INT;       (* 算法 -> PLC：本次识别耗时 *)
        Err     : INT;
    END_STRUCT;

    udt_AxisOne : STRUCT
        Actual : INT;        (* mm，有符号 *)
    END_STRUCT;

    udt_Axis : STRUCT
        X     : udt_AxisOne;
        Y     : udt_AxisOne;
        Z     : udt_AxisOne;
        State : INT;         (* 0=IDLE 1=MOVING 2=INPOS 9=FAULT *)
    END_STRUCT;

    udt_Job : STRUCT
        Cmd   : INT;         (* 1=GOTO 2=HANDLE；算法执行后清 0 *)
        State : INT;
    END_STRUCT;

    udt_Shelf : STRUCT
        Bits  : DWORD;       (* bit(n-1) = 第 n 号已入位；见 §2.3 第 3 条 *)
        Full  : BOOL;
        Count : INT;
    END_STRUCT;

    udt_Diag : STRUCT
        Heartbeat : DINT;    (* 每秒 +1，PLC 做看门狗 *)
        Err       : INT;
    END_STRUCT;

    udt_Cmd : STRUCT
        Trigger : BOOL;
        Attach  : BOOL;
        Detach  : BOOL;
        Reset   : BOOL;
    END_STRUCT;

    udt_Di : STRUCT
        InPos   : BOOL;
        CamDone : BOOL;
        Held    : BOOL;
        Full    : BOOL;
        Ready   : BOOL;
    END_STRUCT;
END_TYPE
```

### 2.2 程序声明

```pascal
PROGRAM Main
VAR
    Camera : udt_Camera;
    Pos    : INT;          (* 目标库位 1..12 *)
    Job    : udt_Job;
    Axis   : udt_Axis;
    Shelf  : udt_Shelf;
    Diag   : udt_Diag;
    Cmd    : udt_Cmd;
    Di     : udt_Di;
END_VAR
```

### 2.3 三个必须现场确认的点

1. **`Task` 不能做变量名**。`TASK` 是 IEC 61131-3 关键字，多数编译器直接报错
   （推断，取决于厂商实现）。上面已改名 `Job`，YAML 对应要改成
   `{program}.Job.Cmd` / `{program}.Job.State`。
2. **`Shelf.Bits` 用 `DWORD`，不是 `DINT`**。IEC 只有 ANY_BIT 类型（BYTE/WORD/DWORD）
   支持 `Shelf.Bits.3 := TRUE` 这种位访问；`DINT` 属于整数类型，严格实现不允许位选。
   YAML 里同步把 `type: DINT` 改成 `type: DWORD`，否则写入会被判 `TypeMismatch`。
3. **POU 名**。若官方工程沿用 demo 的 `imageClassify`，把 YAML 的
   `program: Main` 改成 `program: imageClassify` 即可，代码不动。

---

## 3. 两种映射：逐字段 vs 整块 UDT

| | 逐字段（`plc_rsc_map.yaml`，默认） | 整块 UDT（`plc_rsc_map_udt.yaml`） |
|---|---|---|
| 一次往返 | 读 7 个标量 = 1 次 `Read` | 1 次 `Read` 搬完整块 |
| PLC 侧要求 | 普通变量即可 | 必须有同名 `TYPE ... STRUCT`，字段名逐字一致 |
| 方向约束 | 每个标签单独声明 | `dir=both` 会整块覆盖 PLC 写入，被 `RscMap.load()` 直接拒绝 |
| 适合 | 首次联调、变量经常改 | 握手稳定后压延迟 |

整块写有个坑（`rsc_link.py:241-250` 已拦住）：`Camera.Request` 由 PLC 写、
算法只读，若把 `Camera` 整块 `dir=both`，算法每个周期都会把上一拍的 `FALSE` 冲回去，
PLC 的请求就会被吞掉。规则是：**整块只读 + 逐字段写**，或整块归一方独占。

---

## 4. PC 侧怎么跑

```bash
# 0) 环境：必须 python3.10（PyPlcnextRsc 0.1.12 用了 3.11 已删的 enum._decompose）
/usr/bin/python3 --version        # 期望 3.10.x
export PLCNEXT_HOST=192.168.3.99        # vPLC 对外 IP；容器内改成 10.88.0.2
export PLCNEXT_PASSWORD='plcnext'       # vPLC 固定 admin/plcnext；EPC 才是机身密码

# 1) 不连 PLC，先验标签表本身
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --describe
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --dry-run --watch

# 2) 连真机，只读
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py \
    --read CAM_REQ --read POS_SET --read TASK_CMD

# 3) 单点写（挑一个无副作用的）
/usr/bin/python3 src/plugins/aic_plc_gateway/scripts/rsc_link.py --write HEARTBEAT=0
```

SDK 加载优先级（`rsc_link.py:96-110`）：`PLCNEXT_RSC_WHEEL` 环境变量 →
包内 `third_party/PyPlcnextRsc-*.whl` → 系统 `pip install` 的版本。
wheel 是纯 Python，可直接加进 `sys.path` 当 zip 导入，**不需要 pip install**。

### 已知限制（联调时会踩）

- **`--read` 读不到 gw2plc 的标签**。`read_all()` 只读 `plc2gw` / `both` 方向的标签
  （`rsc_link.py:373`），所以刚写下去的 `HEARTBEAT` 立刻 `--read` 会打印 `None`。
  验证写入是否生效，要用 PLC 侧变量表看，或临时把该标签改成 `dir: both`。
- **写失败会被拉黑**。`_failed` 里的标签直到重连都不再重试（`rsc_link.py:425`），
  这是为了不刷屏；改完 YAML 要重启进程。
- **RSC 不做类型转换**。`DINT` 变量写 `INT` 值会返回 `TypeMismatch`（错误码 3），
  不是静默截断。

---

## 5. 五步联调顺序（别跳步）

| 步 | 做什么 | 通过标准 |
|---|---|---|
| 1 | `--dry-run` | 26 个标签全部解析，无异常 |
| 2 | `--read`，只读 | 有值返回，`failed_tags` 为空（名字全对） |
| 3 | `--write HEARTBEAT=0` | 返回 0 错误；PLC 变量表里能看到值 |
| 4 | PLC 侧做心跳看门狗 | 网关停 2 s 后 PLC 能报「算法离线」 |
| 5 | 跑 `plc_master_demo.py --mode control` 的等价 ST 流程 | `TASK_STATE` 走完 1→2→5 |

第 4 步最容易漏：RSC 是客户端主动连，链路断的时候 **PLC 侧没有任何异常**，
不自己数心跳就会一直等下一条指令。`Diag.Heartbeat` 每秒 +1 就是给这个用的。

---

## 6. 错误码对照（`DataAccessError`）

| 码 | 名字 | 实际原因 |
|---|---|---|
| 0 | `NONE` | 成功 |
| 1 | `NotExists` | 变量名拼错 / POU 名不对 / 变量不在 `Main` 里 |
| 2 | `NotAuthorized` | 密码错，或该用户没有 GDS 读权限 |
| 3 | `TypeMismatch` | YAML 的 `type` 与 PLC 声明不一致（最常见：INT vs DINT、BOOL vs BYTE） |
| 4/5 | `PortNameSyntaxError` / `SemanticError` | 路径分隔符写错（`/` 只用于分隔 POU，字段用 `.`） |
| 11 | `NoData` | 订阅还没被任务写过一轮，读到的是初始 Void |

---

## 7. 与 Modbus 的对应

完整映射表见 `plc_integration.md` §5。换成 RSC 时只是把「偏移」换成「变量名」：

| Modbus | RSC |
|---|---|
| 保持寄存器 0 | `Main.Camera.Request` |
| 保持寄存器 2 | `Main.Camera.Data` |
| 保持寄存器 5 | `Main.Job.Cmd` |
| 保持寄存器 10 | `Main.Pos` |
| 保持寄存器 11–13 | `Main.Axis.X/Y/Z.Actual` |
| 离散输入 0 | `Main.Di.InPos` |

`plc_gateway.py` 里的状态机（观察/接管、位图、心跳）与协议无关，
换传输层不应该动它 —— 这是 `rsc_link.py` 单独成层的原因。

---

## 8. 待确认清单（现场问清楚再动手）

- [ ] PLC 工程里的 POU 名到底是 `Main` 还是别的？变量是全局 GVL 还是程序内局部？
- [ ] 连的是 VC 还是 EPC？VC 固定 `admin/plcnext`，EPC 才用机身密码
- [ ] 官方工程里 `Cam` 类型的实例名是不是 `Camera`？POU 名是什么？（决定 RSC 路径）
- [ ] 41100 端口是否被防火墙挡？容器内是否必须走 `10.88.0.2`？
- [ ] RSC 的 TLS 是否开启（默认开）？PLC 侧证书是自签还是关闭校验？
- [ ] 三轴到底是 PLC 侧闭环还是算法侧给目标位？（决定 `Axis.*` 是反馈还是指令）
- [ ] `PLCnextBase` 只提供工具类 FB，**没有运动控制 FB** —— 伺服时序是自己写还是另有库？
