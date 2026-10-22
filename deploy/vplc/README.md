# vPLC 真机部署（arm64 容器）

> 版本：2026-10-22 · 对应交接点 `6b212b1` 之后
> 读者：要在设备上跑算法容器的人。Windows 侧开发/自检见 `docs/guides/windows_plc_handover.md`。

## 1. 三件事先分清

| 在哪里跑 | 跑什么 | 网络地址 |
|---|---|---|
| Windows 开发机 | `rsc_link.py --describe/--probe`、`run_to_plc.py --dry-run` | PLC `192.168.3.99:41100` |
| 设备上的算法容器 | `run_to_plc.py --source mvs`（真相机） | vPLC 桥接 IP `10.88.0.2:41100` |
| PLCnext Engineer | IEC 程序（`docs/plc/main.st`）、示教 | — |

算法容器里用 `192.168.3.99` 会**超时**，不是认证失败——这是最容易误判的一条。

## 2. 构建镜像

### 2.1 在设备上构建（推荐，快）

```bash
# 把仓库拷到设备（U 盘或 scp）
cd /opt/aic/AIC_cv-dev
podman build -f deploy/vplc/Dockerfile -t aic-vision:arm64 .
podman images | grep aic-vision
```

### 2.2 在 Windows 上交叉构建（次选）

```powershell
cd F:\AIC_cv-dev
docker buildx build --platform linux/arm64 -f deploy/vplc/Dockerfile -t aic-vision:arm64 --load .
docker save aic-vision:arm64 -o aic-vision-arm64.tar
```

`--load` 才能进本地镜像库；QEMU 模拟下第一次构建约 5–15 min。
产物体积约 400–600 MB（opencv-headless + numpy 占大头）。

### 2.3 验收

```powershell
docker image inspect aic-vision:arm64 --format "{{.Architecture}}"   # 必须是 arm64
```

设备上：

```bash
podman run --rm aic-vision:arm64 \
  python /app/run_to_plc.py --map /app/config/cam_official.yaml --probe --dry-run
```

期望：8 个标签全部打印出来（干跑时全部"存在"是正常的，假设备不会报缺失）。

## 3. 部署五步

```bash
# 1) 导入
podman load -i aic-vision-arm64.tar          # 交叉构建的产物才需要这一步

# 2) 密码走环境变量，不要写进命令行（会进 shell history）
export PLCNEXT_PASSWORD='<机身密码>'

# 3) 只探测变量，不拍照
podman run --rm -e PLCNEXT_PASSWORD aic-vision:arm64 \
  python /app/run_to_plc.py --map /app/config/cam_official.yaml --probe

# 4) 单次拍照（忽略握手，拍一次就退）
podman run --rm --network host -e PLCNEXT_PASSWORD aic-vision:arm64 \
  python /app/run_to_plc.py --map /app/config/cam_official.yaml --source mvs --once

# 5) 常驻等 PLC 的 Start（比赛运行态）
podman run -d --name aic-vision --restart unless-stopped \
  --network host -e PLCNEXT_PASSWORD \
  aic-vision:arm64 \
  python /app/run_to_plc.py --map /app/config/cam_official.yaml --source mvs
```

坑：`--network host` 在 podman 的 rootless 模式下等价于 `--network=slirp4netns`，
容器里的 `10.88.0.2` 未必可达。**先在容器里 `curl`/`nc` 测一下 41100**，
再决定是 host 网络还是显式路由。

## 4. 日志与排障

```bash
podman logs -f aic-vision                 # 跟随
podman logs --tail 100 aic-vision         # 最近 100 行
podman exec -it aic-vision python -c "import PyPlcnextRsc; print('ok')"
podman exec -it aic-vision python /app/run_to_plc.py --probe   # 容器内再探一次
```

| 现象 | 大概率原因 |
|---|---|
| `exec format error` | 镜像是 x86-64 的（构建时没加 `--platform linux/arm64`） |
| `libGL.so.1: cannot open` | 用了 `opencv-python` 而不是 `-headless`，或漏装 `libgl1` |
| 连 41100 超时 | 用了 `192.168.3.99`；容器内要用 `10.88.0.2`（或网络模式不对） |
| `NotAuthorized` | 密码是网页登录的 `plcnext`，不是机身密码 |
| `NotExists` | POU 名不对：改 `cam_official.yaml` 的 `program` |
| `TypeMismatch` | 字段类型对不上，最常见是 `ErrCode` 声明成 `INT` 而不是 `WORD` |
| 一直 `Data=100` | 取图失败：相机被 MVS 客户端占用 / 像素格式不是三通道 / 曝光过暗 |

## 5. 更新镜像

```bash
podman stop aic-vision && podman rm aic-vision
podman load -i aic-vision-arm64.tar
# 重跑第 3 节的第 5 步
```

比赛现场不要 `podman rmi` 删除旧镜像前先确认新镜像的 `--probe` 通过。
