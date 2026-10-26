#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""逐块拍「置物台俯视特写」，用于图案 / 判据联调（配合 analyze_patterns.py）。

为什么需要：改 `gen_world.defect_visual` 或判据阈值后，唯一可靠的验收是
「每类各拍一张、看判型对不对」。跑整轮 L4 要 3 分钟且日志里看不到单块细节，
这里直接把 12 块料逐块送到置物台、用主相机（`/camera/image`）各拍一张。

用法（由 probe_patterns.sh 调用，也可以自己起仿真后单独跑）：
  /usr/bin/python3 scripts/probe_patterns.py --out logs/patterns
  /usr/bin/python3 scripts/analyze_patterns.py logs/patterns   # 出特征 + 判型

实现要点：
  * 上料走 `/control/spawn`（UInt8），与看板「生成物料」按钮同一条链路；
  * 拍完一块先用 `ign service set_pose` 把上一块挪到画面外，避免两块叠在同一处；
  * 相机帧用 BEST_EFFORT 订阅（与 gz 桥一致），取「最新一帧」而不是等新帧 ——
    上料后等 settle 秒，缓存里的最后一帧就已经是稳定后的画面。
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8

WORLD = "aic_warehouse"


class Probe(Node):
    def __init__(self, args):
        super().__init__("pattern_probe")
        self.args = args
        self.bridge = CvBridge()
        self.frame = None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, args.topic, self._on_image, qos)
        self.spawn_pub = self.create_publisher(UInt8, "/control/spawn", 10)

    def _on_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:                                     # noqa: BLE001
            self.get_logger().warn("图像转换失败: {}".format(e))

    def _spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _park(self, idx):
        """把某块料挪到画面外（底板下方很远处），避免和下一块叠在一起。"""
        req = 'name: "material_{:02d}" position: {{x: -0.6 y: 0.0 z: -0.5}}'.format(idx)
        subprocess.run(
            ["ign", "service", "-s", "/world/{}/set_pose".format(WORLD),
             "--reqtype", "ignition.msgs.Pose",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", "1500", "--req", req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)

    def run(self):
        os.makedirs(self.args.out, exist_ok=True)
        self._spin(1.0)
        if self.frame is None:
            print("[probe] 没收到 {} 的帧；仿真起了吗？".format(self.args.topic))
            return 1
        for k in range(1, self.args.count + 1):
            if k > 1:
                self._park(k - 1)
                self._spin(0.2)
            msg = UInt8()
            msg.data = k
            self.spawn_pub.publish(msg)
            self._spin(self.args.settle)
            if self.frame is None:
                print("[probe] m{:02d} 没拿到帧".format(k))
                continue
            path = os.path.join(self.args.out, "m{:02d}.png".format(k))
            cv2.imwrite(path, self.frame)
            print("[probe] {} 已存".format(path))
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="logs/patterns")
    ap.add_argument("--topic", default="/camera/image")
    ap.add_argument("--count", type=int, default=12)
    ap.add_argument("--settle", type=float, default=1.6,
                    help="上料后等几秒再拍（盘要落稳、朝向已随机）")
    args = ap.parse_args(argv)

    rclpy.init()
    node = Probe(args)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
