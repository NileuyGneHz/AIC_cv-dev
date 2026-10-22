# 比赛上位机（Ubuntu）改造清单

> 版本：2026-10-22
> 场景：比赛时 Ubuntu 笔记本作为上位机，设备是 VC-StartKit vPLC
> 先读：`vplc_deployment.md`（版本表、arm64 镜像、官方 `Camera` 契约）

---

## 0. 先确认上位机演什么角色

这两个角色改动完全不同，不要混：

| | A 工程站（推荐，符合通知） | B 算法机 |
|---|---|---|
| 视觉跑在哪 | vPLC 里的算法容器 | Ubuntu 本机 |
| Ubuntu 干什么 | eHMI、SSH/podman、录制、本机预演 | 取图判型 + RSC |
| 需要 arm64 镜像 | 是 | 否（提交材料仍建议有） |
| 相机接哪 | 设备 ETH1 | Ubuntu 第二网口 |
| RSC 地址 | 容器内 `10.88.0.2` | `192.168.3.99` |
| 与官方通知一致 | 一致 | 不一致，需向组委会确认 |

**按 A 准备**：通知写明「视觉程序导入到 VPLC 设备中」。但代码写成两边都能跑——
同一个入口，换 `--host` 与 `--source` 即可，B 只作为赛前本机预演。

---

## 1. 接线与 IP

| 链路 | 设备侧 | Ubuntu 侧 | 用途 |
|---|---|---|---|
| 3 网段 | ETH2 `192.168.3.20`；vPLC `192.168.3.99` | `192.168.3.100/24` | eHMI、SSH、RSC 调试 |
| 2 网段 | ETH1 `192.168.2.20` → 海康相机 | 仅预演时：`192.168.2.66/24` | Ubuntu 本机取图 |
| 桥接 | vPLC 容器内 `10.88.0.2` | 不配 | 算法容器访问 vPLC |

```bash
# 连接名先用 `nmcli con show` 查，不一定叫 Wired connection 1
# 3 网段（比赛必配）
nmcli con mod "Wired connection 1" ipv4.method manual \
  ipv4.addresses 192.168.3.100/24 ipv4.gateway "" ipv4.dns ""
nmcli con up "Wired connection 1"

# 2 网段（只在本机预演取图时接，平时不用）
nmcli con mod "Wired connection 2" ipv4.method manual \
  ipv4.addresses 192.168.2.66/24 ipv4.gateway "" ipv4.dns ""
```

坑：

- **不要**把 Ubuntu 设成 `192.168.3.99` 或 `.20`，会和 vPLC / 宿主机冲突。
- 两个网口**不要桥接**：3 网段是控制网，2 网段是相机网，桥接会把广播混在一起。
- 直连网线即可，不需要交换机；RK3568 网口支持 auto-MDIX。

---

## 2. 代码要改什么

原则：**仿真主线一个字不动**。`vision_node.py`、行为树、Gazebo、看板保持原样；
比赛走一条新增的无 ROS 旁路。

### P0（不改就跑不起来）

| 动作 | 文件 | 内容 |
|---|---|---|
| 新增 | `src/vision/aic_vision_trad/scripts/run_to_plc.py` | 无 ROS 入口：取图 → `detect()` → `classify()` → RSC 读写 `Camera`；带 `--probe` / `--dry-run` / `--once` |
| 新增 | `src/plugins/aic_plc_gateway/config/cam_official.yaml` | 官方 `Cam` 8 字段标签表，`program` 名留占位 |
| 新增 | `deploy/vplc/Dockerfile` | `--platform=linux/arm64` + `python:3.10-slim`，拷视觉脚本与 `rsc_link.py` |
| 新增 | `deploy/vplc/requirements-arm64.txt` | `opencv-python-headless` / `numpy` / `PyYAML` / `PyPlcnextRsc==0.1.12` |
| 新增 | `scripts/build_vplc_image.sh` | Ubuntu 原生构建 arm64 并导出 tar |
| 新增 | `scripts/run_competition.sh` | 上位机一键起入口（默认 `--host 192.168.3.99`） |
| 修改 | `src/plugins/aic_plc_gateway/scripts/rsc_link.py` | 只加一个 `probe(paths)` 只读探测辅助；不动现有读写逻辑 |

### P1（赛前该有）

| 动作 | 文件 | 内容 |
|---|---|---|
| 新增 | `scripts/check_vplc_link.sh` | ping → 41100 端口 → 只读变量，三步体检 |
| 新增 | `deploy/vplc/README.md` | 设备上的 `podman load/run/logs` 命令卡 |
| 修改 | `docs/guides/acceptance_manual.md` | 加一节「真机验收」，与 L1–L5 并列 |

### P2（可选，别在赛前赶）

| 动作 | 内容 |
|---|---|
| 容器内 Flask 状态页 | 显示最近一次 `Data` / 耗时 / 错误码，替代看板 |
| 多配方 | `Prog` 字段映射到不同的行列参数 |

---

## 3. 在 Ubuntu 上构建 arm64 镜像

官方教程用 Windows Docker Desktop；Ubuntu 可以原生做，不必装 Windows：

```bash
sudo apt install -y podman qemu-user-static binfmt-support
sudo podman build --arch arm64 -t aic-vision:1.0 -f deploy/vplc/Dockerfile .
podman save -o aic-vision-arm64.tar aic-vision:1.0
```

或者继续用 Docker：

```bash
docker buildx build --platform linux/arm64 -t aic-vision:1.0 \
  -f deploy/vplc/Dockerfile . --load
docker save aic-vision:1.0 -o aic-vision-arm64.tar
```

构建完先验架构，别把 x86 镜像拖到设备上才发现：

```bash
podman inspect aic-vision:1.0 --format '{{.Architecture}}'   # 必须是 arm64
```

---

## 4. 比赛日流程（上位机视角）

1. 接线，配 `192.168.3.100`，`ping 192.168.3.20` 与 `192.168.3.99`。
2. 浏览器开 `http://192.168.3.99`，确认 eHMI 能进（vPLC 活着）。
3. `ssh root@192.168.3.20`，`podman load -i`，`podman run -d --network host ...`。
4. `podman logs -f aic-vision`，确认三行：RSC 连上、`Camera` 探测到、等待 `Start`。
5. eHMI：清库位 → 初始化 → 激活初始化 → 模式启动。
6. 按按钮盒绿色启动，盯第一块：`Data` 是否 1..4、`Done` 是否翻转。
7. 异常顺序：容器日志 → eHMI 报警 → 变量表，不要先重启设备。

---

## 5. 三个硬风险

1. **PLCnext Engineer 只有 Windows 版**（推断：官方未发布 Linux 包）。
   Ubuntu 上位机要改 PLC 程序，只能开 KVM/VirtualBox 的 Windows 虚拟机并桥接网卡，
   或另带一台 Windows 机器。比赛日只部署算法、不改 PLC 程序的话，可以不装；
   但 vPLC 被删后恢复要重新下载程序，这一步绕不开。
2. **相机 Python 封装不在仓库里**。`PLC-softwares/LibMvCamera(1)` 只有 7 个 `.so`，
   没有 `MvCameraControl_class.py`，而且实测是 x86-64。arm64 SDK 要连 Python 封装一起拿，
   否则只能自己写 ctypes 绑定。
3. **看板用不了**。`aic_dashboard` 订阅的是 ROS 话题，比赛链路没有 ROS；
   现场 UI 只有 eHMI。要自己的状态页，就在算法容器里加 Flask（P2）。

---

## 6. 赛前验收

- [ ] Ubuntu `ping 192.168.3.99` 通，`nc -z 192.168.3.99 41100` 通
- [ ] `run_to_plc.py --source folder --dry-run` 在 `logs/` 的样本上 12/12
- [ ] 容器 `--probe` 能读到 `Camera` 的 8 个字段
- [ ] 单次拍照：`Start` 上升沿 → `Done=TRUE`、`Data∈1..4`
- [ ] 遮住镜头：`Data=100`、`ErrCode≠0`
- [ ] 拔网线 10 s 再插回，30 s 内自动重连（`rsc_link` 退避 2→10 s）
- [ ] 整轮 12 块全部入架，eHMI 计数与物理位置一致

---

## 7. 双系统怎么分工（Windows / Ubuntu）

结论：**不要整体迁到 Windows。** 这个仓库有两半，恰好一半一半：

| 工作 | Windows 原生 | Ubuntu | 说明 |
|---|---|---|---|
| PLCnext Engineer 2026.3 | 可以 | 不行 | 官方只发 Windows 包 |
| MVS 调相机 | 可以 | 不行 | 官方 V4.4.0 是 Windows 软件 |
| 构建 arm64 算法镜像 | 可以 | 可以 | Windows 用 Docker Desktop（官方流程），Ubuntu 用 podman+buildx |
| `run_to_plc.py` 连 vPLC | 可以 | 可以 | 纯 Python + socket，无 POSIX 依赖 |
| Modbus 网关 / 主站 demo | 可以 | 可以 | 只用 `socket`，没有 `fcntl`/`termios` |
| 视觉离线跑图 / 调阈值 | 可以 | 可以 | OpenCV + numpy |
| 文档 / PPT / 录制 | 更顺 | 可以 | Office、OBS 都在 Windows 更成熟 |
| ROS 2 + Gazebo 仿真 | 基本不行 | 可以 | Fortress 没有 Windows 原生支持 |
| `scripts/*.sh` 验收 | 不行 | 可以 | 20 处依赖 `pkill`/`pgrep`/`/proc` |

建议分工：

- **Windows**：`deploy/vplc/`、`cam_official.yaml`、`run_to_plc.py` 的本机调试、
  PLCnext Engineer、MVS、PPT 与录制。
- **Ubuntu**：ROS 2 + Gazebo、行为树、L1–L5 验收、`verify_all.sh`。

两条硬规矩：

1. **仓库各克隆一份，用 GitHub 同步**，不要在 Windows 的 `/mnt/c` 或 NTFS 盘里
   `colcon build`——9p 文件系统慢，权限位和符号链接也会丢。
2. **`.sh` 必须保持 LF**。仓库已加 `.gitattributes` 强制；Windows 侧不要用
   “记事本另存为”改脚本，否则 bash 会报 `\r: command not found`。

Python 版本：Windows 侧装 **3.10**（`py -3.10`），不要用 3.13；
`PyPlcnextRsc` 在 3.11+ 导入即崩。脚本里的 `/usr/bin/python3` 是 Ubuntu 路径，
Windows 上换成 `py -3.10` 即可。

WSL2 只能当备选：ROS 2 能跑，但 Gazebo GUI 走 D3D12 经常花屏/掉帧，
USB 相机直通也麻烦。真要跑仿真，回 Ubuntu 双系统启动。
