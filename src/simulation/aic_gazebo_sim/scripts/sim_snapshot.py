#!/usr/bin/python3
"""抓一张（或几张）仿真相机图存成 PNG —— 视觉侧取数 / 场景自检用。

用法：
    ros2 run aic_gazebo_sim sim_snapshot.py --ros-args \
        -p out_dir:=logs/snapshots -p count:=3 -p interval_s:=0.5

产物：<out_dir>/snapshot_<n>_<stamp>.png（RGB，尺寸同相机配置 640x480）
"""
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from sim_layout import declare_number


class SnapshotNode(Node):
    def __init__(self):
        super().__init__("sim_snapshot")
        self.topic = self.declare_parameter("image_topic", "/camera/image").value
        self.out_dir = self.declare_parameter("out_dir", "logs/snapshots").value
        self.count = declare_number(self, "count", 1, int)
        self.interval_s = declare_number(self, "interval_s", 0.5)
        os.makedirs(self.out_dir, exist_ok=True)
        self.saved = 0
        self.last_save = 0.0
        self.create_subscription(Image, self.topic, self._on_image, 1)
        self.get_logger().info(
            "[snapshot] 订阅 %s，存 %d 张到 %s" % (self.topic, self.count, self.out_dir))

    def _on_image(self, msg):
        now = time.time()
        if self.saved >= self.count:
            return
        if self.saved > 0 and (now - self.last_save) < self.interval_s:
            return

        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "rgb8").lower()
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        try:
            if enc in ("rgb8", "bgr8"):
                img = buf.reshape(h, w, 3)
                if enc == "bgr8":
                    img = img[:, :, ::-1]
            elif enc in ("rgba8", "bgra8"):
                img = buf.reshape(h, w, 4)
                img = img[:, :, 2::-1] if enc == "bgra8" else img[:, :, :3]
            elif enc in ("mono8", "8uc1"):
                img = np.repeat(buf.reshape(h, w, 1), 3, axis=2)
            else:
                self.get_logger().error("[snapshot] 不支持的编码 %s" % msg.encoding)
                return
        except ValueError as exc:
            self.get_logger().error("[snapshot] 图像数据长度不对（%s），跳过" % exc)
            return

        from PIL import Image as PILImage
        path = os.path.join(
            self.out_dir,
            "snapshot_%02d_%s.png" % (self.saved, time.strftime("%Y%m%d_%H%M%S")))
        PILImage.fromarray(img).save(path)
        self.saved += 1
        self.last_save = now
        self.get_logger().info("[snapshot] 已存 %s（%dx%d %s）" % (path, w, h, msg.encoding))
        if self.saved >= self.count:
            self.get_logger().info("[snapshot] 完成，退出")
            raise SystemExit(0)


def main():
    rclpy.init()
    node = SnapshotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
