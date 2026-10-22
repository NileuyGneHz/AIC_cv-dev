#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉 → 官方 Cam 契约（真机 vPLC）入口。**不依赖 ROS**，Windows 可直接跑。

为什么单独一个入口
------------------
仿真链路的视觉是 `vision_node.py`（订阅 `/camera/image`，走 ROS 话题）。
真机上 PLC 只认 RSC 变量：`Camera.Start / Done / Data / ErrCode`。
把「取图 → 定位 → 判型 → 写 PLC」这四步从 ROS 里摘出来，好处是：
  - Windows 侧（无 ROS）就能联调；容器里也不用装 ROS；
  - 判据代码与仿真共用同一份（plate_detector / defect_classifier），不会两套漂移。

握手时序（设备手册 2026.Q3.V1 §3.1；交接文档 §6 P0-2）
------------------------------------------------------
    PLC: Camera.Start := TRUE
    算法: 取图 → detect() → classify()
    算法: Camera.Data := 1..4（失败 100）; Camera.ErrCode := 0..5
    算法: Camera.Done := TRUE
    PLC: 读 Data/ErrCode；Camera.Reset := TRUE
    算法: Camera.Done := FALSE; Camera.ErrCode := 0

坑（实测 / 手册）
----------------
1. `Data = 100` 表示**拍照失败**，不是「未识别 = 0」。0 留给「还没拍」。
2. 先写 Data/ErrCode 再置 Done —— 反过来 PLC 可能先看到 Done、读到上一轮的 Data。
3. `CAM_START` 是电平还是脉冲由 PLC 程序决定；这里用**上升沿**触发，所以 PLC 可以不
   自动复位 Start，但同一轮里算法不会重复拍。
4. RSC 不做类型转换：`ErrCode` 是 `WORD`，写 int 时由 rsc_link 按 IEC 类型打包；
   YAML 里 type 写错会直接 TypeMismatch（错误码 3），不是静默截断。

用法
----
    # 离线回归（不需要 PLC，不需要相机）：12 张仿真帧应 12/12
    python run_to_plc.py --source folder --dir logs/frames --dry-run --once

    # 只看链路与变量名，不拍照
    python run_to_plc.py --probe --dry-run

    # 真机：等 PLC 的 Start
    set PLCNEXT_PASSWORD=<机身密码>
    python run_to_plc.py --source mvs --map ..\\..\\plugins\\aic_plc_gateway\\config\\cam_official.yaml --host 192.168.3.99

    # 单次联调（忽略握手，拍一次就退）
    python run_to_plc.py --source mvs --once
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
# src/vision/aic_vision_trad/scripts -> 仓库根
REPO = HERE.parents[2]
GW_SCRIPTS = REPO / "src" / "plugins" / "aic_plc_gateway" / "scripts"
DEFAULT_MAP = REPO / "src" / "plugins" / "aic_plc_gateway" / "config" / "cam_official.yaml"

for _p in (str(HERE), str(GW_SCRIPTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 官方约定：100 = 拍照失败
DATA_FAIL = 100

# 错误码（与 cam_official.yaml 的 CAM_ERRCODE 注释一致）
ERR_NONE = 0
ERR_GRAB = 1
ERR_LOCATE = 2
ERR_CLASSIFY = 3
ERR_WRITE = 4
ERR_TIMEOUT = 5

IMG_EXT = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


# --------------------------------------------------------------------------- 取图
def imread_unicode(path):
    """读图（兼容中文路径）。

    坑：Windows 上 `cv2.imread` 遇到非 ASCII 路径直接返回 None，
    在 `D:\\比赛素材\\...` 这种目录下表现为「取图失败」，很难联想到编码。
    """
    import numpy as np
    import cv2

    buf = np.fromfile(str(path), dtype=np.uint8)
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


class FolderSource(object):
    """离线跑图：按文件名排序循环取帧。没有相机时用它回归判据。"""

    name = "folder"

    def __init__(self, root):
        self.root = Path(root)
        if not self.root.is_dir():
            raise SystemExit("目录不存在：%s" % self.root)
        self.files = sorted(
            [p for p in self.root.iterdir() if p.suffix.lower() in IMG_EXT])
        if not self.files:
            raise SystemExit("目录里没有图片（支持 %s）：%s"
                             % (", ".join(sorted(IMG_EXT)), self.root))
        self.idx = 0

    def grab(self):
        p = self.files[self.idx % len(self.files)]
        self.idx += 1
        img = imread_unicode(p)
        if img is None:
            return None, "读图失败：%s" % p.name
        return img, str(p)


class MvsSource(object):
    """海康 MVS 取图（真机）。

    状态：**未实机验证**（本机无相机、无 aarch64 版 MVS SDK）。
    只做「打开第一个设备 → 取一帧 → 关设备」，不做曝光/增益调参；
    现场参数请先在 MVS 客户端里调好（曝光、增益、白平衡），再跑本脚本。
    """

    name = "mvs"

    def __init__(self):
        try:
            from MvCameraControl_class import MvCamera  # noqa: F401
        except ImportError as e:
            raise SystemExit(
                "找不到 MvCameraControl_class（海康 MVS Python 封装）。\n"
                "  1) 装 MVS V4.4.0 客户端；\n"
                "  2) 把 MVS 安装目录下的 Samples/Python/MvImport 加进 PYTHONPATH；\n"
                "  3) arm64 容器要换成 aarch64 版 SDK（x86-64 的 .so 在 RK3568 上加载不了）。\n"
                "原始错误：%s" % e)
        self._cam = None
        self._buf = None

    def open(self):
        import ctypes
        from MvCameraControl_class import MvCamera

        device_list = MvCamera.MV_CC_DEVICE_INFO_LIST()
        layer = MvCamera.MV_GIGE_DEVICE | MvCamera.MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(layer, device_list)
        if ret != 0 or device_list.nDeviceNum == 0:
            raise SystemExit("MVS 未发现相机（ret=%d，设备数=%d）"
                             % (ret, device_list.nDeviceNum))
        cam = MvCamera()
        info = ctypes.cast(device_list.pDeviceInfo[0],
                           ctypes.POINTER(MvCamera.MV_CC_DEVICE_INFO)).contents
        if cam.MV_CC_CreateHandle(info) != 0:
            raise SystemExit("MV_CC_CreateHandle 失败")
        if cam.MV_CC_OpenDevice(layer) != 0:
            raise SystemExit("MV_CC_OpenDevice 失败（相机被其它程序占用？MVS 客户端要先关掉）")
        cam.MV_CC_StartGrabbing()
        self._cam = cam
        # 缓冲区一次分配，逐帧复用
        self._buf = (ctypes.c_ubyte * (256 * 1024 * 4))()

    def grab(self):
        import numpy as np
        from MvCameraControl_class import MV_FRAME_OUT_INFO_EX, MV_OK

        if self._cam is None:
            self.open()
        info = MV_FRAME_OUT_INFO_EX()
        ret = self._cam.MV_CC_GetOneFrameTimeout(self._buf, len(self._buf), info, 1000)
        if ret != MV_OK:
            return None, "MV_CC_GetOneFrameTimeout 失败 ret=0x%x" % ret
        w, h = int(info.nWidth), int(info.nHeight)
        # 坑：这里按 BGR8 三通道解释。若相机输出 Mono8，帧长只有 w*h，
        #      reshape 会抛 ValueError —— 此时在 MVS 客户端把像素格式改成 RGB8/BGR8。
        try:
            arr = np.frombuffer(self._buf, dtype=np.uint8, count=w * h * 3)
            return arr.reshape((h, w, 3)).copy(), "mvs"
        except ValueError:
            return None, "像素格式不是三通道（Mono8？请在 MVS 里改成 RGB8/BGR8）"

    def close(self):
        if self._cam is None:
            return
        try:
            self._cam.MV_CC_StopGrabbing()
            self._cam.MV_CC_CloseDevice()
            self._cam.MV_CC_DestroyHandle()
        except Exception:  # noqa: BLE001 - 关设备失败不该改变退出码
            pass
        self._cam = None
        self._buf = None


# ----------------------------------------------------------------------- 识别
def detect_and_classify(img):
    """返回 (type_id, err_code, reason)。type_id=0 表示失败，err_code 说明原因。

    与仿真 `vision_node.py` 用同一份 detector / classifier，避免两套阈值漂移。
    """
    import plate_detector
    import defect_classifier

    res = plate_detector.detect(img)
    if res is None:
        return 0, ERR_LOCATE, "盘面定位失败（Hough + 内亮外暗环都没有候选）"
    comps = res.get("comps") or []
    if not comps:
        return 0, ERR_CLASSIFY, "无缺陷候选（暗区连通域为空，检查光照/曝光）"
    mtype, reason = defect_classifier.classify(comps)
    if mtype == 0:
        return 0, ERR_CLASSIFY, reason
    return mtype, ERR_NONE, reason


# ----------------------------------------------------------------------- 写 PLC
def publish_result(link, mtype, err_code):
    """先写 Data/ErrCode，再置 Done。

    坑：一次写完再置 Done 的两步顺序不能反 —— PLC 扫到 Done 上升沿就会去读 Data，
    同一次 RSC 往返里两个变量不保证 PLC 已在同一扫描周期看到。
    """
    data = mtype if err_code == ERR_NONE else DATA_FAIL
    ok1 = link.write_all({"CAM_DATA": data, "CAM_ERRCODE": err_code}, force=True)
    ok2 = link.write_all({"CAM_DONE": True}, force=True)
    return ok1 > 0 and ok2 > 0


def clear_done(link):
    link.write_all({"CAM_DONE": False, "CAM_ERRCODE": ERR_NONE}, force=True)


def announce_ready(link):
    link.write_all({"CAM_COMM_OK": True, "CAM_READY": True,
                    "CAM_DONE": False, "CAM_ERRCODE": ERR_NONE}, force=True)


# ----------------------------------------------------------------------- 模式
def do_one_shot(args, link, source):
    """拍一张 → 判一次 → 写一次。返回进程退出码。"""
    t0 = time.time()
    img, tag = source.grab()
    if img is None:
        print("[run_to_plc] 取图失败：%s" % tag)
        err = ERR_GRAB
        mtype = 0
        reason = tag
    else:
        mtype, err, reason = detect_and_classify(img)
        print("[run_to_plc] %s -> type=%d err=%d（%.0f ms）%s"
              % (Path(tag).name if tag else "-", mtype, err,
                 (time.time() - t0) * 1000.0, reason))
        if err == ERR_NONE and args.save_dir:
            _save_debug(img, mtype, args.save_dir)
    if link is not None:
        if not publish_result(link, mtype, err):
            print("[run_to_plc] 写 PLC 失败（看上面的 RSC 报错：变量名 / 类型 / 权限）")
            return 3
    return 0 if err == ERR_NONE else 2


def run_handshake(args, link, source):
    """等 PLC 的 Start 上升沿；收到 Reset 清 Done。"""
    announce_ready(link)
    print("[run_to_plc] 就绪：等 Camera.Start（Ctrl+C 退出）")
    prev_start = False
    prev_reset = False
    deadline = time.time() + args.timeout
    shots = 0
    while True:
        if args.timeout > 0 and time.time() > deadline:
            print("[run_to_plc] 等待 Start 超时（%.0f s），退出" % args.timeout)
            return 1
        rd = link.read_all()
        start = bool(rd.get("CAM_START", False))
        reset = bool(rd.get("CAM_RESET", False))

        if reset and not prev_reset:
            clear_done(link)
            print("[run_to_plc] PLC 复位 -> Done 清零")

        if start and not prev_start:
            shots += 1
            print("[run_to_plc] 第 %d 次拍照" % shots)
            do_one_shot(args, link, source)
            if args.loops and shots >= args.loops:
                print("[run_to_plc] 已跑满 %d 轮，退出" % shots)
                return 0

        prev_start, prev_reset = start, reset
        time.sleep(args.poll_s)


def run_probe(link, mapping):
    """逐条报告 8 个标签在 PLC 上是否存在（读一遍，写方向也读一次）。"""
    names = list(mapping.tags.keys())
    vals = link.read_all(include_writes=True)
    missing = [n for n in names if n not in vals]
    print("== 变量探测：program=%s ==" % mapping.program)
    for n in names:
        v = vals.get(n, None)
        print("  %-12s %-10s %s" % (n, "存在" if n in vals else "缺失",
                                    "" if n in vals else mapping.tags[n].path))
    if missing:
        print("\n缺失 %d 个。三种常见原因，按顺序排：" % len(missing))
        print("  1) POU 名不对：改 cam_official.yaml 的 program（Main / PLC_PRG / imageClassify）")
        print("  2) 实例名不是 Camera（官方契约是 Camera，若工程里改成别的要同步 YAML）")
        print("  3) 8 个字段类型与 PLC 声明不一致（ErrCode 必须是 WORD）")
        return 1
    print("\n8 个标签全部存在。")
    return 0


def _save_debug(img, mtype, out_dir):
    """存一张带类型的调试图（现场判错的证据）。"""
    import cv2

    d = Path(out_dir)
    d.mkdir(parents=True, exist_ok=True)
    name = "plc_%s_type%d.png" % (time.strftime("%Y%m%d_%H%M%S"), mtype)
    cv2.imwrite(str(d / name), img)
    print("[run_to_plc] 调试图：%s" % (d / name))


# ----------------------------------------------------------------------- main
def build_source(args):
    if args.source == "folder":
        if not args.dir:
            raise SystemExit("--source folder 需要 --dir <目录>")
        return FolderSource(args.dir)
    return MvsSource()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="视觉 → PLC（官方 Cam 契约），无 ROS")
    ap.add_argument("--source", choices=("folder", "mvs"), default="folder",
                    help="folder=离线跑图；mvs=海康相机")
    ap.add_argument("--dir", default="", help="--source folder 的图片目录")
    ap.add_argument("--map", default=str(DEFAULT_MAP), help="标签表 YAML")
    ap.add_argument("--host", default=os.environ.get("PLCNEXT_HOST", "192.168.3.99"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("PLCNEXT_PORT", "41100")))
    ap.add_argument("--user", default=os.environ.get("PLCNEXT_USER", "admin"))
    ap.add_argument("--password-env", default="PLCNEXT_PASSWORD",
                    help="从该环境变量取机身密码（不要写在命令行里）")
    ap.add_argument("--dry-run", action="store_true",
                    help="内存假 PLC，不连设备（验证标签表与判据）")
    ap.add_argument("--probe", action="store_true", help="只探测 8 个变量是否存在")
    ap.add_argument("--once", action="store_true",
                    help="拍一张→判一次→写一次就退出（联调用，不等握手）")
    ap.add_argument("--loops", type=int, default=0, help="握手模式下跑满 N 轮后退出")
    ap.add_argument("--timeout", type=float, default=0.0, help="握手等待上限（秒，0=不限）")
    ap.add_argument("--poll-s", type=float, default=0.05, help="轮询 Start/Reset 的间隔")
    ap.add_argument("--save-dir", default="", help="存调试图的目录")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    import logging
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")

    try:
        import rsc_link
    except ImportError as e:
        raise SystemExit("导入 rsc_link 失败（检查仓库结构是否完整）：%s" % e)

    mapping = rsc_link.RscMap.load(args.map)
    print("[run_to_plc] 标签表：%s" % args.map)
    print("[run_to_plc] %s" % mapping.describe())

    if args.dry_run:
        print("[run_to_plc] 干跑模式：不连 PLC，写操作只进内存")

    # 干跑也走 RscLink（它内部用内存假设备），这样读写路径与真机完全一致
    link = rsc_link.RscLink(mapping, args.host, args.port, args.user,
                            os.environ.get(args.password_env),
                            dry_run=args.dry_run)
    try:
        link.connect()
    except Exception as e:  # noqa: BLE001
        print("[run_to_plc] RSC 连接失败：%s" % e)
        return 4

    source = None
    try:
        if args.probe:
            return run_probe(link, mapping)

        source = build_source(args)
        if args.once:
            return do_one_shot(args, link, source)
        return run_handshake(args, link, source)
    except KeyboardInterrupt:
        print("\n[run_to_plc] 收到 Ctrl+C，退出")
        return 0
    finally:
        if source is not None and hasattr(source, "close"):
            source.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
