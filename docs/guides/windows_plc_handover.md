# Windows PLC 开发交接文档

> 交接日期：2026-10-22
> 仓库：`https://github.com/NileuyGneHz/AIC_cv-dev`（私有，main）
> 交接点：`6b212b1`
> 接手环境：Windows（双系统中的 Windows 侧），Ubuntu 侧继续负责 ROS 2 / Gazebo

---

## 0. 一句话现状

**PLC 协议层与仿真闭环已经完成，真机 vPLC 一次都没连过。**
Windows 侧要做的是：把「官方 `Cam` 契约 → arm64 算法镜像 → 设备部署 → 真机联调」这条链补齐。

| 模块 | 状态 | 证据 |
|---|---|---|
| Modbus TCP 从站 + 软件主站 | 完成 | `scripts/test_plc_proto.sh`；L2 PASS |
| RSC 标签层 + 断线重连 | 完成 | `rsc_round_test.py`，28 条断言 |
| 网关状态机（观察 / 接管） | 完成 | 仿真接管整轮 12/12、128.9 s |
| 官方 `Cam` 8 字段契约 | 已确认字段，未真机验证 | 设备手册 2026.Q3.V1 p18 |
| `run_to_plc.py`（无 ROS 入口） | **未做** | 本文 §6 |
| arm64 算法镜像 | **未做** | 本文 §6 |
| 真 vPLC 联调 | **未做** | 无设备；步骤见 `vplc_deployment.md` |

注意：12/12、128.9 s 是「软件主站 + Gazebo 仿真」的成绩，不是真机成绩。
写进汇报材料时必须标明，别让评委误以为是设备实测。

---

## 1. Windows 环境清单

按这个版本装，别用「最新版」：

| 软件 | 版本 | 用途 |
|---|---|---|
| Git for Windows | 最新 | 克隆、提交 |
| Python | **3.10.x** | RSC 库只支持 3.7–3.10 |
| PyCharm | 2023.3.3+ | 官方推荐，非必需 |
| PLCnext Engineer | **2026.3**（最低 2025.9） | 写 / 下载 IEC 程序 |
| Docker Desktop | 最新 | 构建 arm64 镜像（可选） |
| MVS | **V4.4.0** | 海康相机调试 |
| MobaXterm 或 Windows Terminal | 最新 | SSH 到宿主机 `192.168.3.20` |
| Netname+ | 最新 | 扫设备 IP（可选） |

Python 3.10 检查：

```powershell
py -0p                 # 应能看到 3.10
py -3.10 --version     # Python 3.10.x
```

坑：`python` 很可能指向 3.13（Store 版或别的装法）。规则分两段：

- **建 venv 之前**用 `py -3.10`；
- **激活 venv 之后**一律用 `python`，不要再加 `py -3.10`——launcher 可能绕过 venv，
  装好的 PyYAML / wheel 会找不到。

`PyPlcnextRsc` 在 3.11+ 会因为 `enum._decompose` 被删而导入失败。

---

## 2. 克隆与 5 分钟自检（不需要设备）

目录不要带中文、空格，也不要放 OneDrive。仓库是私有的，首次 `clone` 会弹浏览器登录，
用有权限的 GitHub 账号走 Git Credential Manager 即可，不要把 PAT 写进 URL：

```powershell
mkdir C:\work
cd C:\work
git clone https://github.com/NileuyGneHz/AIC_cv-dev.git
cd AIC_cv-dev

py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-plc.txt
python -m pip install src\plugins\aic_plc_gateway\third_party\PyPlcnextRsc-0.1.12-py3-none-any.whl
```

首次激活 venv 若报「禁止运行脚本」：

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

自检（纯离线，用内存假 PLC）：

```powershell
python src\plugins\aic_plc_gateway\scripts\rsc_link.py --describe
python src\plugins\aic_plc_gateway\scripts\rsc_link.py --dry-run --watch
```

第一行期望：

```text
program=Main  标签 26（读 7 / 写 19）  结构体 0  前缀 Arp.Plc.Eclr/
```

`--dry-run --watch` 会每秒打印一遍标签值；`Ctrl+C` 退出。
这一步过了，说明 Python、PyYAML、wheel 注入、标签表都正常。

---

## 3. 连真 vPLC

### 3.1 网络

- Windows 有线网卡设 `192.168.3.100/24`，网关留空。
- 网线接设备 **ETH2**。
- vPLC 对外 `192.168.3.99`，宿主机 `192.168.3.20`。
- 不要设成 `.99` 或 `.20`，会冲突。

```powershell
Test-Connection 192.168.3.99 -Count 2
Test-NetConnection 192.168.3.99 -Port 41100
```

### 3.2 只读探测（先别写）

```powershell
$env:PLCNEXT_PASSWORD = "plcnext"
python src\plugins\aic_plc_gateway\scripts\rsc_link.py `
  --host 192.168.3.99 --read CAM_REQ
```

**两个注意**：

1. 这里读的是我们自创的 `Main.Camera.Request`，官方工程里大概率不叫这个。
   第一次上设备不要用它判断「连没连上」，先做 §4 的变量表核对，
   或等 `run_to_plc.py --probe` 写好。
2. `--read` 只能读 `plc2gw` / `both` 方向的标签；`CAM_DATA` 是 `gw2plc`，
   用它试读会打印 `None`，不是链路坏了。

### 3.3 写测试

只在确认变量存在后做，挑一个无副作用的（如 `HEARTBEAT`）：

```powershell
python src\plugins\aic_plc_gateway\scripts\rsc_link.py `
  --host 192.168.3.99 --write HEARTBEAT=0
```

返回 0 错误即通。写失败会被拉黑到重连为止，改完 YAML 要重启进程。

---

## 4. PLCnext Engineer 侧要拿到的三件事

1. **POU 名**：`Main`？`PLC_PRG`？`imageClassify`？决定 RSC 路径前缀。
2. **实例名**：官方手册写 `Camera.Data`，类型名是 `Cam`。确认工程里实例就叫 `Camera`。
3. **8 个字段的读写方向**：`Start/Reset/Prog` 是 PLC 写，`CommOK/Ready/Done/ErrCode/Data` 是算法写
   （这是按流程推断，手册没给方向表）。

把这些抄进 `docs/plc/` 下的现场记录，别只留在聊天里。

下载程序时记住：**重新下载会重置示教位置**，下载完要重新示教。

---

## 5. 代码地图（哪些能碰，哪些别碰）

| 文件 | 作用 | Windows 能不能跑 |
|---|---|---|
| `src/plugins/aic_plc_gateway/scripts/rsc_link.py` | RSC 传输层 + YAML 标签映射 | **能**，纯 Python |
| `.../scripts/plc_master_demo.py` | Modbus 主站参考实现 | 能，但需要一个 Modbus 从站 |
| `.../scripts/plc_gateway.py` | ROS 2 网关节点（Modbus 从站 + 状态机） | **不能**，依赖 `rclpy` + `aic_msgs` |
| `.../scripts/rsc_round_test.py` | RSC 整轮自测 | **不能**，依赖 ROS |
| `.../config/plc_rsc_map.yaml` | 自创 5 字段 `Camera`，仿真用 | 能读，**别拿去连官方工程** |
| `src/vision/aic_vision_trad/scripts/plate_detector.py` | 盘面定位，纯 OpenCV | 能 |
| `.../defect_classifier.py` | 四类缺陷判据，纯 Python | 能 |
| `.../vision_node.py` | ROS 视觉节点 | 不能，订阅 `/camera/image` |
| `docs/guides/plc_integration.md` | 寄存器表、握手、排障 | 文档 |
| `docs/guides/plcnext_rsc_setup.md` | RSC 标签映射细节 | 文档 |
| `docs/guides/vplc_deployment.md` | 版本表、arm64 镜像、部署五步 | 文档 |
| `docs/plc/main.st` | PLC 侧巡视状态机骨架（ST） | 文档 |

**结论：Windows 上能开发的是 `rsc_link.py`、视觉算法、以及待写的 `run_to_plc.py`。
`plc_gateway.py` 是仿真桥，不要试图在 Windows 上跑它。**

---

## 6. 待办（按顺序做，每项有验收）

### P0-1 官方 8 字段标签表

新增 `src/plugins/aic_plc_gateway/config/cam_official.yaml`：

- `program` 先写 `Main`，现场改；
- 8 个 tag：`CAM_COMM_OK / CAM_READY / CAM_START / CAM_DONE / CAM_RESET / CAM_PROG / CAM_ERRCODE / CAM_DATA`；
- 类型：BOOL×5、INT×2（Prog/Data）、WORD×1（ErrCode）；
- 方向：Start/Reset/Prog = `plc2gw`，其余 `gw2plc`。

验收：

```powershell
python src\plugins\aic_plc_gateway\scripts\rsc_link.py `
  --map src\plugins\aic_plc_gateway\config\cam_official.yaml --describe
```

应打印 8 个标签，路径全是 `Arp.Plc.Eclr/Main.Camera.*`。

### P0-2 `run_to_plc.py`

放 `src/vision/aic_vision_trad/scripts/run_to_plc.py`，无 ROS：

- `--source folder --dir <目录>`：离线跑图，用于没相机时回归；
- `--source mvs`：海康取图（延迟导入 `MvCameraControl_class`，缺库时给明确报错）；
- `--dry-run`：用内存假 PLC，不连设备；
- `--probe`：逐条尝试 `Cam` / `Camera` / `imageClassify.image_result`，打印存在性；
- `--once`：拍一张、判一次、写一次就退出，便于联调。

握手：读 `CAM_START` → 取图 → `detect()` → `classify()` → 先写 `CAM_DATA` 再置 `CAM_DONE`；
失败写 `Data=100` + `ErrCode≠0`；等 `Reset` 清 `Done`。

验收：用 `logs/` 里的 12 张仿真帧，`--source folder --dry-run` 分类 12/12。

### P0-3 arm64 镜像

- `deploy/vplc/Dockerfile`：`FROM --platform=linux/arm64 python:3.10-slim`；
- `deploy/vplc/requirements-arm64.txt`：`opencv-python-headless`、`numpy`、`PyYAML`、
  `PyPlcnextRsc==0.1.12`；
- 拷 `plate_detector.py`、`defect_classifier.py`、`run_to_plc.py`、`rsc_link.py`。

验收：`docker image inspect` 的 `Architecture` = `arm64`；容器内 `import PyPlcnextRsc` 不报错。

### P0-4 真机四步

1. `--probe` 找到 `Camera`；
2. 确认 `CAM_START` 随 PLC 变化（`Start` 是 `plc2gw`，可用 `--read` 验）；
   `Data` 是算法写、PLC 读，不要用 CLI 的 `--read` 判断它有没有写成功；
3. `--once` 单次拍照，`Data∈1..4`；
4. 整轮 12 块，eHMI 计数与物理位置一致。

### P1

- `deploy/vplc/README.md`：设备上的 `podman load/run/logs` 命令卡；
- `scripts/check_vplc_link.ps1`：ping → 41100 → 只读，三步体检；
- `docs/plc/` 现场记录：POU 名、实例名、8 字段方向、示教值。

---

## 7. 契约（照抄，别自己改名）

设备手册 2026.Q3.V1 §3.1 p18：

```pascal
Cam : STRUCT
    CommOK  : BOOL;   // 通讯OK
    Ready   : BOOL;   // 相机准备好
    Start   : BOOL;   // 拍照启动
    Done    : BOOL;   // 拍照完成
    Reset   : BOOL;   // 相机复位
    Prog    : INT;    // 相机程序号
    ErrCode : WORD;   // 错误代码
    Data    : INT;    // 拍照结果，100=拍照失败
END_STRUCT
```

- 类型名 `Cam`，实例名 `Camera`，路径 `Arp.Plc.Eclr/<POU>.Camera.Data`。
- `Data=100` 是拍照失败，不是「未识别 = 0」。
- 官方 RSC demo 用的 `imageClassify.image_result` 是教学工程，不是比赛设备。

---

## 8. 坑（Windows 特有 + PLC 特有）

**Windows**

1. `python` 指向 3.13 → 建 venv 用 `py -3.10`；激活后坚持用 `python`，
   不要用 `py -3.10`（可能绕过 venv）。
2. venv 激活被 ExecutionPolicy 拦 → `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`。
3. 路径带中文/空格/OneDrive → CMake、venv、Docker 构建都可能挂；固定 `C:\work`。
4. 首次监听 5020 会弹防火墙 → 只勾「专用网络」，别勾公网。
5. `.sh` 在 Git Bash 里也跑不了：脚本用了 `pkill`/`pgrep`/`/proc`，这些 Windows 没有。
6. CRLF：仓库已加 `.gitattributes` 强制 `.sh` 为 LF，别用记事本改脚本。

**PLC**

7. vPLC 容器不可删；删了去 `192.168.3.20:3001`（admin/plcnext）恢复，恢复后要重下程序。
8. 重下 PLC 程序会重置示教位置。
9. PC 用 `192.168.3.99`，算法容器内用 `10.88.0.2`；写错表现为超时，不是认证失败。
10. `PLC-softwares/LibMvCamera(1)` 只有 7 个 `.so`，实测 **x86-64**，且没有
    `MvCameraControl_class.py`；arm64 镜像要重新下 SDK + Python 封装。
11. RSC 默认 TLS，PC 与设备时间差太大可能握手失败（推断）；先 `w32tm /resync` 对时。
12. 官方文档自己也有 EPC 残留（设备手册 2.2 仍写「发送给 EPC」），看到 EPC 字样按 VC 理解。

---

## 9. 与 Ubuntu 侧怎么协作

- **Windows 负责**：`deploy/vplc/`、`src/.../run_to_plc.py`、`cam_official.yaml`、
  PLCnext Engineer 工程、MVS 调参、PPT/录制。
- **Ubuntu 负责**：ROS 2 + Gazebo、行为树、看板、L1–L5 验收、`verify_all.sh`。
- 两套各自克隆，走 GitHub 同步；不要在 `/mnt/c` 或 NTFS 里 `colcon build`。
- 提交格式：`feat/fix/docs/chore(scope): 中文摘要`。
- 不要提交 `.tar` 镜像、`.pptx` 大文件、MVS SDK 二进制；`logs/` 已在 `.gitignore`。

---

## 10. 完成标准（Windows 阶段）

- [ ] `rsc_link.py --dry-run` 26 标签全绿
- [ ] `cam_official.yaml --describe` 8 标签、路径正确
- [ ] `run_to_plc.py --source folder --dry-run` 12/12
- [ ] 容器 `--probe` 读到 `Camera`
- [ ] 单次拍照 `Data∈1..4`；遮镜头 `Data=100`
- [ ] 整轮 12/12，eHMI 与物理位置一致
- [ ] 拔网线 10 s 再插回，30 s 内 RSC 自动重连
- [ ] 上述结果写回 `docs/plc/`，并更新 `README.md` 状态表

---

## 11. 文档索引

| 文档 | 什么时候看 |
|---|---|
| `docs/guides/vplc_deployment.md` | 装版本、打镜像、部署五步 |
| `docs/guides/plcnext_rsc_setup.md` | RSC 标签怎么映射、五步联调 |
| `docs/guides/plc_integration.md` | 寄存器表、握手时序、错误码 |
| `docs/guides/competition_host.md` | 上位机网络、双系统分工、三大风险 |
| `docs/plc/main.st` | PLC 侧 ST 骨架 |
| `AGENT_SIM_MANUAL.md` | 仿真全景（Windows 侧可跳过） |
