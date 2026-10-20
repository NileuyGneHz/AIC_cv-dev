#!/usr/bin/env python3
"""传统视觉识别节点：相机图像 + 采集请求 -> /vision/material_type。

接口（见 docs/guides/decision_vision_interface.md）：
  订阅 /camera/image        sensor_msgs/Image   相机（仿真是 gz 相机桥出来的）
  订阅 /vision/trigger      std_msgs/Empty      采集请求（龙门到位后由其发出）
  发布 /vision/material_type std_msgs/String     "1".."4"，每次请求最多一条

为什么订阅端用 BEST_EFFORT（坑）：
  gz -> ros 的图像桥按 sensor data QoS 发（best effort）。reliable 订阅者连 best effort
  发布者是「不兼容」，表现为一条都收不到且无报错。反过来（reliable 发 / best effort 收）
  是兼容的，所以订阅侧统一用 best_effort 最安全。

为什么到位判定不用图像时间戳（坑）：
  仿真里图像 stamp 是 sim time，节点时钟也是 sim time，看似能算 age；一旦上游桥没带
  stamp 或时钟未同步，age 会变成几十万秒，触发永远被判「帧太旧」。这里改用
  收到帧的本地单调时钟时刻，和 ROS 时钟完全解耦。
"""

import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from defect_classifier import DEFAULT_RULES, TYPE_NAMES, classify   # noqa: E402
from plate_detector import DetectorParams, detect                   # noqa: E402


class VisionTradNode(Node):
    def __init__(self):
        super().__init__("vision_trad_node")

        self.image_topic = self.declare_parameter("image_topic", "/camera/image").value
        self.trigger_topic = self.declare_parameter("trigger_topic", "/vision/trigger").value
        self.result_topic = self.declare_parameter("result_topic", "/vision/material_type").value
        self.debug_dir = self.declare_parameter("debug_dir", "").value
        self.frame_wait_s = float(self.declare_parameter("frame_wait_s", 1.20).value)
        self.settle_skip_s = float(self.declare_parameter("settle_skip_s", 0.25).value)
        self.max_stale_s = float(self.declare_parameter("max_stale_s", 1.50).value)
        self.debounce_s = float(self.declare_parameter("debounce_s", 0.20).value)

        # 判据阈值全部暴露成 ROS 参数，现场可调，不必改代码重编
        self.rules = {}
        for k, v in DEFAULT_RULES.items():
            self.rules[k] = float(self.declare_parameter(k, float(v)).value)
        self.det = DetectorParams(
            dark_delta=float(self.declare_parameter("dark_delta", 30.0).value),
            min_comp_area=int(self.declare_parameter("min_comp_area", 120).value),
            roi_frac=float(self.declare_parameter("roi_frac", 0.93).value),
        )

        self.bridge = CvBridge()
        self._frame = None
        self._frame_rx = 0.0          # 收到帧的本地单调时钟时刻
        self._deadline = None         # 本次请求的放弃时刻（None = 空闲）
        self._min_frame_rx = 0.0      # 只接受这个时刻之后收到的帧
        self._last_pub = 0.0

        # 坑：depth 一定要 1。识别（Hough）在定时器里跑几十毫秒，期间图像回调会在
        #     队列里排队；depth=5 时执行器会把 5 帧旧图逐帧送进来，于是「到位后第一帧」
        #     可能还是 0.25 s 前运动中拍的（实测整盘偏出画面中心 30 px，判成污渍）。
        #     只要最新一帧就不会有这个问题。
        img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self._on_image, img_qos)
        self.create_subscription(Empty, self.trigger_topic, self._on_trigger, 10)
        self.pub = self.create_publisher(String, self.result_topic, 10)

        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            "[vision] 就绪: {} + {} -> {} (debug_dir={})".format(
                self.image_topic, self.trigger_topic, self.result_topic,
                self.debug_dir or "off"))

    # ---------- 回调 ----------
    def _on_image(self, msg):
        try:
            # 识别只要灰度，进来就转掉：省一次全图 cvtColor，也保证下游只有一种编码
            self._frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().error("[vision] 图像转换失败: {}".format(e))
            return
        self._frame_rx = time.monotonic()

    def _on_trigger(self, _msg):
        if self._deadline is not None:
            self.get_logger().warn("[vision] 上一次请求未处理完，忽略本次")
            return
        now = time.monotonic()
        self._deadline = now + self.frame_wait_s
        # 坑：到位那一刻相机正在停稳，第一帧常常还是「运动中」的画面（实测偏移一整个库位）。
        #     直接拿它识别会稳定失败，所以跳过 settle_skip_s 内到达的帧，之后持续重试到超时。
        self._min_frame_rx = now + self.settle_skip_s
        self.get_logger().info("[vision] 收到采集请求")

    # ---------- 主循环 ----------
    def _tick(self):
        if self._deadline is None:
            return
        now = time.monotonic()
        if self._frame is None or self._frame_rx <= self._min_frame_rx:
            if now < self._deadline:
                return
            self.get_logger().warn(
                "[vision] {:.2f}s 内没有等到可用新帧，本次不发结果".format(self.frame_wait_s))
            if self.debug_dir and self._frame is not None:
                self._dump_fail("nofresh")
            self._deadline = None
            return
        if now - self._frame_rx > self.max_stale_s:
            self.get_logger().warn(
                "[vision] 图像已过期 {:.2f}s（>{:.2f}s），本次不发结果".format(
                    now - self._frame_rx, self.max_stale_s))
            self._deadline = None
            return
        # 坑：rclpy 定时器回调里抛异常会冒到 executor 把节点打死，单帧失败必须只影响单帧。
        try:
            done = self._process()
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().error("[vision] 识别异常，跳过本帧: {}".format(e))
            done = False
        if done:
            self._deadline = None
            return
        # 失败：等下一帧重试（视觉停稳需要几帧，一帧失败不代表这个位点没救）
        self._min_frame_rx = self._frame_rx
        if now >= self._deadline:
            if self.debug_dir:
                self._dump_fail("nolocate")
            self.get_logger().warn("[vision] 超时前始终未识别成功，本次不发结果")
            self._deadline = None

    def _process(self):
        """尝试识别当前帧。返回 True = 本轮结束（已发布或处于去抖窗口）。"""
        if time.monotonic() - self._last_pub < self.debounce_s:
            return True
        res = detect(self._frame, self.det)
        if res is None:
            self.get_logger().warn("[vision] 本帧未定位到铁片，等下一帧重试")
            return False
        tid, why = classify(res["comps"], self.rules)
        if tid == 0:
            self.get_logger().warn("[vision] 本帧盘面无缺陷候选（{}），等下一帧重试".format(why))
            return False

        msg = String()
        msg.data = str(tid)
        self.pub.publish(msg)
        self._last_pub = time.monotonic()
        self.get_logger().info("[vision] 判定 {}（{}）<- {}（r={:.1f}px{}）".format(
            TYPE_NAMES[tid], tid, why, res["plate"]["r"],
            "" if res["plate"].get("refined") else "，精修被拒"))

        if self.debug_dir:
            self._dump_debug(res, tid)
        return True

    def _dump_fail(self, tag):
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            name = "vision_fail_{}_{}_{}.png".format(
                time.strftime("%Y%m%d_%H%M%S"), tag, int(time.time() * 1000) % 1000)
            cv2.imwrite(os.path.join(self.debug_dir, name), self._frame)
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().warn("[vision] 失败帧保存失败: {}".format(e))

    def _dump_debug(self, res, tid):
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            # self._frame 是 mono8，画彩色标注前先转 BGR，否则 cv2.circle 会报颜色维度不符
            vis = cv2.cvtColor(self._frame, cv2.COLOR_GRAY2BGR)
            pl = res["plate"]
            cv2.circle(vis, (int(pl["cx"]), int(pl["cy"])), int(pl["r"]), (0, 255, 0), 2)
            cv2.putText(vis, "{}={}".format(TYPE_NAMES[tid], tid), (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            name = "vision_{}_{}.png".format(
                time.strftime("%Y%m%d_%H%M%S"), TYPE_NAMES[tid])
            cv2.imwrite(os.path.join(self.debug_dir, name), vis)
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().warn("[vision] 调试图保存失败: {}".format(e))


def main():
    rclpy.init()
    node = VisionTradNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # 坑：关停瞬间 rclpy 仍在从已销毁的订阅里取消息，会抛
        # "Unable to convert call argument to Python object"。
        # 这发生在 SIGINT 之后（本轮 12/12 已跑完），但会让 launch 报
        # 「process has died [exit code 1]」，录像/自验时看着像真故障。
        if rclpy.ok():
            raise
        node.get_logger().info("[vision] 收到退出信号（%s），正常收敛" % e)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
