#!/usr/bin/env python3
"""抓「Gazebo 渲染全景 + 末端相机」关键帧，产出使用手册用的全过程分镜。

为什么不用 GUI 截图：GUI 窗口只是把同一条渲染管线的画面显示出来，抓帧节点订阅
`/scene/image`（自由摄像机 scene_cam，960x540@15Hz）拿到的就是**同一台相机、同一帧**
的画面，且不受窗口缩放/遮挡影响。手册里「GUI 里该看到什么」与这里的图一一对应。

用法：
    ros2 run aic_gazebo_sim capture_storyboard.py \
        --ros-args -p out_dir:=logs/storyboard -p detail_slots:="1,2,6,12"

事件源（全部只订阅，不发布）：
    /control/material_task  GOTO/HANDLE 指令（判断第几轮、什么动作）
    /sim/gantry_arrived     到位（物料已上料、停在置物台，识别开始）
    /sim/attach_NN          吸盘吸附（第 NN 块）
    /sim/handle_done        HANDLE 完成（物料已放入货架）

产物：
    <out_dir>/NN_<tag>.png    按时间顺序编号的全景帧（tag 里带轮次与阶段）
    <out_dir>/storyboard.txt  顺序清单：文件名 / 相对时间 / 说明（拼图脚本消费）
"""
import math
import os
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, UInt8

from aic_msgs.msg import MaterialTask


class StoryboardCapture(Node):
    def __init__(self):
        super().__init__("capture_storyboard")

        self.declare_parameter("out_dir", "logs/storyboard")
        self.declare_parameter("scene_topic", "/scene/image")
        self.declare_parameter("cam_topic", "/camera/image")
        # 详拍轮次：这些轮次抓「上料/到位/吸附/搬运/放入」，其余轮次只抓「放入」
        self.declare_parameter("detail_slots", "1,2,6,12")
        self.declare_parameter("last_slot", 12)
        self.declare_parameter("settle_s", 1.5)      # 吸附后再等多久抓「搬运中」
        self.declare_parameter("start_delay_s", 4.0)  # 首帧到达后多久抓「初始态」
        self.declare_parameter("exit_after_final_s", 3.0)
        self.declare_parameter("auto_exit", True)
        # 抓帧机位：正上方垂直向下，能同时看到来料架 3x4 / 置物台 / 分拣货架 4x3 格口。
        # 留空 = 不改机位（用世界里的 scene_cam 默认位姿）。
        # 注意：改机位走 ign service set_pose，必须等世界起来（首帧到达即证明已起来）。
        # 默认正俯视：同时看到来料架 3x4 与分拣货架 4x3，便于判断"哪格空了、哪格填了"。
        # 传 "none" 则沿用世界里的 scene_cam 机位（斜俯视）。
        self.declare_parameter("scene_pose", "0.0,0.02,1.62,0.0,1.5708,1.5708")
        self.declare_parameter("world", "aic_warehouse")

        self.out_dir = self.get_parameter("out_dir").value
        self.detail = {
            int(x) for x in str(self.get_parameter("detail_slots").value).replace(" ", "").split(",")
            if x
        }
        self.last_slot = int(self.get_parameter("last_slot").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.auto_exit = bool(self.get_parameter("auto_exit").value)

        os.makedirs(self.out_dir, exist_ok=True)
        self.index_path = os.path.join(self.out_dir, "storyboard.txt")
        with open(self.index_path, "w", encoding="utf-8") as f:
            f.write("# 序号\t相对时间(s)\t文件\t说明\n")

        self.t0 = None
        self.seq = 0
        self.latest = {"scene": None, "cam": None}
        self.pending = []           # [(deadline, key, tag, note)]；key=None 表示退出哨兵
        self.last_arrived = None
        self.done_seen = set()
        self.final_deadline = None
        self.slot2visit = {}         # slot_id -> 轮次（决策的 visit_index）
        self.first_scene_at = None
        self.cam_tries = 0

        img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter("scene_topic").value,
                                 lambda m: self._on_image(m, "scene"), img_qos)
        self.create_subscription(Image, self.get_parameter("cam_topic").value,
                                 lambda m: self._on_image(m, "cam"), img_qos)
        self.create_subscription(MaterialTask, "/control/material_task", self._on_task, 10)
        self.create_subscription(UInt8, "/sim/gantry_arrived", self._on_arrived, 10)
        self.create_subscription(UInt8, "/sim/handle_done", self._on_done, 10)
        for s in range(1, 33):        # 库位数上限 32，多订阅几个无副作用
            self.create_subscription(Empty, "/sim/attach_{:02d}".format(s),
                                     lambda m, s=s: self._on_attach(s), 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            "[storyboard] 抓帧就绪 → %s（详拍轮次 %s）" % (self.out_dir, sorted(self.detail)))

    # ---------- 图像 ----------
    def _on_image(self, msg, key):
        if msg.encoding == "mono8":
            frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif msg.encoding == "bgr8":
            frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        else:  # rgb8
            frame = cv2.cvtColor(
                np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3),
                cv2.COLOR_RGB2BGR)
        self.latest[key] = frame

        # 首帧证明世界已起来：此时才可能调 set_pose，也才是抓「初始态」的时机
        if key == "scene" and self.first_scene_at is None:
            now = time.time()
            self.first_scene_at = now
            # scene_pose 传 "none" 表示不改机位（注意：传空串会让 rcl 解析参数直接失败）
            pose_raw = str(self.get_parameter("scene_pose").value).strip()
            if pose_raw and pose_raw.lower() != "none":
                self.pending.append((now + 0.5, "cam", "", "切到抓帧机位"))
            self.pending.append(
                (now + float(self.get_parameter("start_delay_s").value), "scene", "r00_start",
                 "初始态：12 块物料铺在来料架 3x4，分拣货架空"))

    # ---------- 事件 ----------
    def _on_task(self, msg):
        if msg.action == MaterialTask.ACTION_GOTO:
            # 巡视顺序 != 轮次顺序（决策按库位号走），轮次只认 visit_index
            self.slot2visit[int(msg.slot_id)] = int(msg.visit_index)
            if int(msg.visit_index) == 1:
                self.pending.append((time.time() + 1.0, "scene", "r01_loading",
                                     "第 1 轮上料：material_01 正被传送到置物台"))

    def _round_of(self, slot):
        return self.slot2visit.get(int(slot), int(slot))

    def _on_arrived(self, msg):
        slot = int(msg.data)
        self.last_arrived = slot
        rnd = self._round_of(slot)
        if rnd in self.detail:
            self._save("scene", "r{:02d}_arrived".format(rnd),
                       "第 %d 轮到位置物台（库位 %d）：吸盘退到偏移位，固定相机开始拍照"
                       % (rnd, slot))
            self._save("cam", "r{:02d}_cam".format(rnd),
                       "第 %d 轮末端相机原图（视觉输入，库位 %d）" % (rnd, slot))

    def _on_attach(self, mat):
        # 注意：attach_NN 的 NN 是**物料编号 = 库位号**，不是轮次；巡视顺序与库位号不同序
        rnd = self._round_of(mat)
        if rnd in self.detail:
            self._save("scene", "r{:02d}_grip".format(rnd),
                       "第 %d 轮吸附（库位 %d）：吸盘贴住物料，DetachableJoint 建约束"
                       % (rnd, mat))
            self.pending.append((time.time() + self.settle_s, "scene",
                                 "r{:02d}_carry".format(rnd),
                                 "第 %d 轮搬运：物料悬空跟随龙门移向货架" % rnd))

    def _on_done(self, msg):
        slot = int(msg.data)
        rnd = self._round_of(slot)
        self.done_seen.add(rnd)
        if rnd in self.detail or rnd == self.last_slot:
            self._save("scene", "r{:02d}_placed".format(rnd),
                       "第 %d 轮放入（库位 %d）：物料落到类型列最低空层，吸盘断气脱离"
                       % (rnd, slot))
        if rnd == self.last_slot and self.final_deadline is None:
            self.final_deadline = time.time() + float(
                self.get_parameter("exit_after_final_s").value) + 1.0
            self.pending.append((self.final_deadline, "scene", "r99_final",
                                 "终态：12 格全部填满（4 类型列 x 3 层）"))
            if self.auto_exit:
                self.pending.append((self.final_deadline + 1.0, None, None, None))

    # ---------- 机位 ----------
    def _apply_scene_pose(self):
        """把 scene_cam 挪到抓帧机位（ign service set_pose，不改控制侧）。

        坑（与看板同源）：Fortress 的请求类型是 ignition.msgs.Pose（扁平字段），
        不是 SetPose、也不接受 entity/pose 嵌套；服务不存在时只是 timeout。
        """
        raw = str(self.get_parameter("scene_pose").value)
        try:
            x, y, z, roll, pitch, yaw = [float(v) for v in raw.split(",")]
        except ValueError:
            self.get_logger().warn("[storyboard] scene_pose 应为 6 个数：%s" % raw)
            return
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        req = ('name: "scene_cam" position: {{x: {:.4f} y: {:.4f} z: {:.4f}}} '
               'orientation: {{x: {:.4f} y: {:.4f} z: {:.4f} w: {:.4f}}}').format(
                   x, y, z, qx, qy, qz, qw)
        cmd = [shutil.which("ign") or "ign", "service", "-s",
               "/world/{}/set_pose".format(self.get_parameter("world").value),
               "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
               "--timeout", "2000", "--req", req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
            if "data: true" in (r.stdout or ""):
                self.get_logger().info("[storyboard] 抓帧机位已生效：%s" % raw)
                return True
            self.get_logger().warn("[storyboard] set_pose 未确认：%s"
                                   % ((r.stdout or "") + (r.stderr or ""))[:160])
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[storyboard] set_pose 异常：%s" % e)
        # 世界刚起来时服务可能还没注册，重试一次
        if self.cam_tries < 3:
            self.cam_tries += 1
            self.pending.append((time.time() + 1.5, "cam", "", "重试切机位"))
        return False

    # ---------- 存盘 ----------
    def _save(self, key, tag, note):
        frame = self.latest.get(key)
        if frame is None:
            self.get_logger().warn("[storyboard] 跳过 %s：%s 尚未收到帧" % (tag, key))
            return
        if self.t0 is None:
            self.t0 = time.time()
        name = "{:02d}_{}.png".format(self.seq, tag)
        path = os.path.join(self.out_dir, name)
        cv2.imwrite(path, frame)
        with open(self.index_path, "a", encoding="utf-8") as f:
            f.write("{:02d}\t{:.1f}\t{}\t{}\n".format(
                self.seq, time.time() - self.t0, name, note))
        self.seq += 1
        self.get_logger().info("[storyboard] %s  %s" % (name, note))

    def _tick(self):
        now = time.time()
        remain = []
        for item in self.pending:
            deadline, key, tag, note = item[0], item[1], item[2], item[3]
            if now < deadline:
                remain.append(item)
                continue
            if key is None:      # 退出哨兵
                self.get_logger().info("[storyboard] 一轮结束，收工（共 %d 帧）" % self.seq)
                rclpy.shutdown()
                return
            if key == "cam":
                self._apply_scene_pose()
                continue
            self._save(key, tag, note)
        self.pending = remain


def main():
    rclpy.init()
    node = StoryboardCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
