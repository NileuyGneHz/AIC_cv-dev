# vPLC（VC-StartKit）接入手册

> 版本：2026-10-22 首版
> 依据：`Downloads/VC算法封装和部署（VC和EPC区别）-使用指导.pdf`（2026/10）、
> `Downloads/VC程序下载及RSC通信-使用指导.pdf`（2026/10）、
> `Downloads/大赛设备(vc版)操作指导手册_2026.10.20.pdf`（2026.Q3.V1）
> 关系：`plcnext_rsc_setup.md` 讲「我们的标签表怎么映射」，本文讲「东西怎么装进 vPLC」。

---

## 0. 结论：按 VC 版准备，版本号照这张表

| 项 | 选定 | 依据 |
|---|---|---|
| 设备形态 | **VC-StartKit（vPLC）**，飞凌 RK3568 / ARM64 / Podman | 部署指导 p2；设备手册 1.3 |
| PLC 编程与下载 | **PLCnext Engineer 2026.3**（最低 2025.9） | 部署指导 p11；两份指导首页 |
| 算法 Python | **3.10.x**（3.10.0 起，别上 3.11） | 部署指导 p11；`rsc_link.py:24-27` |
| RSC 库 | **PyPlcnextRsc 0.1.12**（`py3-none-any`，纯 Python） | 官方 demo 自带 wheel；本仓库 `third_party/` 同版本 |
| 基础镜像 | **arm64 的 Python 3.10-slim** | 部署指导 p2「基础镜像选择 arm64v8/python」 |
| 相机 | **MVS V4.4.0 + arm64/aarch64 SDK** | 设备手册 3.2.1、部署指导 p2 |
| 部署方式 | 镜像 tar → 宿主机 `192.168.3.20` → `podman load/import` | 部署指导 p3-4；设备手册 §5 |
| RSC 地址 | PC 调试 `192.168.3.99`；**容器内 `10.88.0.2`** | 部署指导 p2；设备手册 5.3 |
| RSC 账号 | `admin` / **`plcnext`** | 部署指导 p2；设备手册 5.3 |

**一句话：不是 EPC1502，是 VC；镜像必须 arm64，Python 锁 3.10，RSC 库用 0.1.12。**

---

## 1. EPC 与 VC 的差别（决定要不要重做镜像）

官方对照表（部署指导 p2）：

| 项 | EPC1502 | VC-StartKit |
|---|---|---|
| PLC 形态 | 物理整机，PLCnext 直接跑在本机 | vPLC 是 Podman 容器，跑在 RK3568 宿主机 |
| CPU | x86-64 | **ARM64** |
| 算法镜像 | x86_64 | **必须 arm64** |
| SDK / whl | x86 | **aarch64 / arm64** |
| 容器引擎 | Docker | **Podman**（宿主机原生） |
| RSC IP | EPC 硬件 IP | PC 用 `192.168.3.99`，容器内用 `10.88.0.2` |

所以从 EPC 迁移过来的镜像、SDK、whl **全部不能直接用**，必须重打。

---

## 2. 我们项目里哪些上设备，哪些不上

这是最容易搞混的一点：仓库里的网关是「PLC 的对端」，不是「跑在 PLC 里的程序」。

```
仿真（现在）：
  Gazebo 相机 -> vision_node -> /vision/material_type
                              -> aic_behavior_tree -> /control/material_task
                              -> plc_gateway -> Modbus/RSC -> PLC 测试替身

真机（比赛）：
  海康相机 -> 算法容器（detector + classifier + RSC 客户端）
           -> RSC 10.88.0.2 -> vPLC 的 IEC 61131-3 程序 -> PROFINET -> 伺服
```

| 组件 | 仿真 | 上 vPLC |
|---|---|---|
| `aic_gazebo_sim` / `aic_behavior_tree` / `aic_dashboard` | 用 | **不带** |
| `plc_gateway.py`（Modbus 从站 / 状态机） | 用 | **不带** |
| `rsc_link.py` 的标签层 | 用 | **可直接复用**（无 ROS 依赖），只新增一份官方 `Camera` 的 YAML |
| `plate_detector.py` / `defect_classifier.py` | 用 | **带**，纯 OpenCV，无 ROS 依赖（`plate_detector.py` 首行注释即写明） |
| `vision_node.py` | 用 | **不带**，它订阅 `/camera/image`，真机没有这条话题 |

结论：**比赛镜像里只装「取图 + 判型 + RSC 握手」三件事**，不需要 ROS，也不需要 Gazebo。
好处是镜像小、启动快，跟官方 `run_ToPLC_new.py` 的形态一致。

---

## 3. 五步接入

### 第 0 步：装对版本（Windows 侧）

1. PLCnext Engineer **2026.3**（2025.9 也可，推荐 2026.3）。
2. Python **3.10.x**（官方清单写 3.10.X，任意小版本）。
3. Docker Desktop 最新版（要能 `--platform linux/arm64` 构建）。
4. MVS **V4.4.0**（相机调试）+ **arm64 SDK**（放进镜像，见 §5 坑 2）。

### 第 1 步：把 PLC 程序下载进 vPLC

1. PC 网线接设备 **ETH2**，PC 设 `192.168.3.100`（不要和 `.20` / `.99` 冲突）。
2. PLCnext Engineer 里双击项目 → 联机控制器 → 选网卡 → 扫描，应发现 VC。
3. 右键 VC → **写入并启动项目**。
4. 坑：**重新下载 PLC 程序会重置示教位置**，下载完要重新示教（RSC 指导 p4 明确提示）。

### 第 2 步：构建 arm64 算法镜像

在仓库根目录（`/home/nileuy/AIC_cv-dev`）准备 `deploy/vplc/Dockerfile`（尚未创建，见 §7）后：

```bash
# x86 开发机上交叉构建 arm64；Docker Desktop 需开启 containerd 镜像存储
docker buildx build --platform linux/arm64 \
  -t aic-vision:1.0 -f deploy/vplc/Dockerfile . --load

# 导出带元数据的镜像（推荐，能保留 ENTRYPOINT，便于自启动）
docker save aic-vision:1.0 -o aic-vision-arm64.tar
```

官方示例是 `docker export`（导出容器文件系统）+ `podman import`，体积更小，
但**会丢 ENTRYPOINT/CMD**，导入后必须手动敲 `python3 run_to_plc.py`。
两种都能用：要自启动选 `save/load`，要最小体积选 `export/import`。

### 第 3 步：导入宿主机

```bash
# 宿主机（不是 vPLC）：192.168.3.20  root/root
scp aic-vision-arm64.tar root@192.168.3.20:/root/
ssh root@192.168.3.20
podman load -i /root/aic-vision-arm64.tar
podman image ls
```

若是 `docker export` 的包：

```bash
podman import /root/aic-vision-arm64.tar aic-vision:1.0
```

### 第 4 步：起容器

```bash
podman run -d --name aic-vision \
  --network host --restart=always \
  -e PLCNEXT_HOST=10.88.0.2 \
  -e PLCNEXT_USER=admin \
  -e PLCNEXT_PASSWORD=plcnext \
  aic-vision:1.0 \
  python3 /opt/aic/run_to_plc.py
```

- `--network host` 是必须的：相机在 ETH1 的 `192.168.2.x`，vPLC 桥接地址是 `10.88.0.2`，
  只有 host 网络能同时够到两边（官方也是 `--network host`）。
- 不要把 `10.88.0.2` 写成 `192.168.3.99`：容器里走的是桥接网，写对外 IP 会连不上。
- 调试时先 `podman run -it --rm ... /bin/bash`，手动跑一遍再改成 `-d`。

### 第 5 步：eHMI 联调

1. vPLC 网页 `192.168.3.99`，`admin/plcnext`。
2. 「材料状态」页 → **清除库位**。
3. 「自动运行」页 → 初始化 → 激活初始化 → 模式启动。
4. 先看容器日志确认握手，再按按钮盒绿色启动。

---

## 4. 官方 `Cam` 契约（别直接搬我们的 5 字段结构体）

设备手册 §3.1（p18）原文：

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

**类型名是 `Cam`，实例变量名是 `Camera`**（手册正文一直写 `Camera.Data`）。
RSC 路径因此是 `Arp.Plc.Eclr/<POU>.Camera.Data`，不是 `.../Cam.Data`。

方向手册没给表，下面按流程推断（标「推断」）：

| 字段 | 方向（推断） | 用法 | 对应本仓库语义 |
|---|---|---|---|
| `CommOK` | 算法→PLC | 连上 RSC 后置 TRUE，断开清 FALSE | 新增 |
| `Ready` | 算法→PLC | 相机取流成功、算法就绪 | 新增 |
| `Start` | PLC→算法 | 拍照启动（上升沿） | `Camera.Request` |
| `Done` | 算法→PLC | 本次结果已写入 | `Camera.State==DONE` |
| `Reset` | PLC→算法 | 清 `Done`/`Data`/`ErrCode` | 无 |
| `Prog` | PLC→算法 | 相机程序号 / 配方 | 无 |
| `ErrCode` | 算法→PLC | 0=OK，非 0 错误码 | `Camera.Err` |
| `Data` | 算法→PLC | 1..4 物料编号；**100=拍照失败** | `Camera.Data`（0 要改成 100） |

现场第一件事是**探变量**，不要按文档猜 POU 名：

```bash
# 容器里，先用只读方式确认路径存在（入口待建，见 §7）
python3 /opt/aic/run_to_plc.py --probe --host 10.88.0.2
```

`--probe` 要依次尝试 `Cam` / `Camera` / `imageClassify.image_result` 三种命名，
把「存在/不存在」打出来。官方 RSC demo 用的是 `imageClassify.image_result`，
那是教学工程；比赛设备手册明确写 `Camera`，两者不是一回事。

---

## 5. 六个版本/部署坑

1. **Python 3.11 起导入即崩**。`PyPlcnextRsc` 用了 `enum._decompose`，3.11 删了这个私有符号
   （`rsc_link.py:24-27` 有记录）。镜像里锁 `python:3.10-slim`，不要用 `python:3-slim`。
2. **仓库里的相机 SDK 是 x86 的，不能进镜像**。`PLC-softwares/LibMvCamera(1)/libMvCameraControl.so`
   实测为 `ELF 64-bit ... x86-64`；vPLC 要 aarch64 版本，必须重新下载。
3. **`docker export` 丢启动命令**。用 `export/import` 就必须在 `podman run` 末尾显式写
   `python3 run_to_plc.py`；否则容器起来就退出。
4. **容器内 IP 是 `10.88.0.2`**。`192.168.3.99` 只给 PC 调试用；写错表现为 RSC 连接超时，
   不是认证失败。
5. **vPLC 容器不可删**。删了要登录 `192.168.3.20:3001`（admin/plcnext）重新加载；
   官方反复警告不要 `rm -rf /*`，会连授权一起没。
6. **PLC 程序一重下载，示教位置就没了**。先示教再跑自动，顺序不能反。

---

## 6. 赛前 checklist

- [ ] 镜像 `docker image inspect` 的 `Architecture` = `arm64`
- [ ] 容器内 `python3 -c "import PyPlcnextRsc"` 不报错（验证 3.10 + 0.1.12）
- [ ] 容器内能 `ping 10.88.0.2`、能发现相机（MVS 枚举到设备）
- [ ] `--probe` 能读到 `Camera` 的 8 个字段
- [ ] 单次拍照：`Start` 上升沿 → `Done=TRUE`、`Data∈1..4`
- [ ] 失败路径：遮住镜头 → `Data=100`、`ErrCode≠0`
- [ ] 整轮：eHMI 清库位 → 初始化 → 启动，12 块全部入架
- [ ] 断电重启后容器能自启（`--restart=always` 或 systemd 单元）

---

## 7. 还缺什么（下一步）

镜像要能构建，还差四样，都在 `deploy/vplc/` 下（尚未创建）：

1. `Dockerfile`：`FROM --platform=linux/arm64 python:3.10-slim`，装 `opencv-python-headless`
   `numpy` `PyPlcnextRsc==0.1.12`，拷 `plate_detector.py` / `defect_classifier.py`，
   以及 `rsc_link.py`（纯 Python，无 ROS，可直接进镜像）。
2. `run_to_plc.py`：取图（MVS 或本地目录）→ `detect()` → `classify()` → 用 `RscLink` 读写 `Camera`。
   需要带 `--probe` / `--dry-run`，没有设备也能在 x86 上验分类链路。
3. `cam_official.yaml`：官方 8 字段的标签表，`program` 待现场确认；`rsc_link.py` 不用改。
4. `requirements-arm64.txt`：锁版本，避免在 RK3568 上现场编译。

这四样要等 §4 的变量名在真机上探过再定稿；现在写死字段名，到了现场大概率要返工。
