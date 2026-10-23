#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时判分可视化节点：正确绿 / 错误红 + 计分板 + 双相机流 + 全过程态势图，单窗口可录。

对标官方平台的过审硬指标 3、4（docs/competition/platform_and_video_checklist.md）：
  3. 实时判分可视化 —— 放置正确变绿 / 错误变红，自动计分；
  4. 单窗口合成画面 —— 全景相机（自由视角）+ 末端相机 + 俯视态势图 + 库位状态 + 得分，
     一个页面里，OBS 直接录浏览器。

抓取过程可视化（态势图数据来源，全部零新增桥）：
  - 龙门实时位置：/world/<w>/model/aic_gantry/joint_state（已桥的关节反馈，x/y 即世界坐标
    —— 龙门零位 = 货架中心正上方）；
  - 12 块铁片位置：订阅 /sim/attach_XX、/sim/detach_XX（控制侧本来就发的夹/放信号）——
    在架上 = 库位中心，夹起 = 跟随龙门实时位置，放下 = 对应类型码放格中心。
    页面俯视图里能看到铁片从货架「飞」到码放格的全过程。

自由摄像机（scene_cam，gen_world.py 生成的 static 相机模型）：
  POST /api/cam {"preset":"overhead|top|place|rack"} 切预设机位；
  POST /api/cam {"dx":..,"dy":..,"dz":..} 六向平移。
  实现走 `ign service -s /world/<w>/set_pose`（subprocess），不改控制侧、不加服务桥。

判分口径（与 4 列码放布局等价）：
  「物料被放到正确列」⟺「视觉判出的类型 == 该库位的真值类型」。
  真值来自 sim_layout.yaml 的 materials.types（与视觉真值同源，不引入第二份事实）。

HTTP 接口（默认 127.0.0.1:8080，只绑本机）：
  GET  /               仪表盘页面（暗色，适合录屏）
  GET  /api/state      JSON 状态（前端 0.5 s 轮询）
  POST /api/reset      清空本轮判分
  POST /api/cam        自由摄像机：预设机位 / 相对平移
  GET  /stream.mjpg    末端相机 MJPEG 流（叠加当前位点 / 识别 / 比分）
  GET  /scene.mjpg     自由摄像机（全景）MJPEG 流（干净画面）

只读订阅，不发布任何话题 —— 判分器挂了不影响链路本身。
"""
import json
import math
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Empty, Float64, String, UInt8
from aic_msgs.msg import MaterialTask

import yaml

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "破损", 4: "孔洞"}
TYPE_NAMES_EN = {1: "SCRATCH", 2: "STAIN", 3: "CHIP", 4: "HOLE"}
# 库位墙格口边框色（BGR）：跟随该格物料的流转状态
RACK_STATUS_COLORS = {
    "stock": (85, 85, 85), "staged": (250, 170, 80), "held": (60, 190, 250),
    "placed": (80, 205, 80), "recycled": (200, 120, 200),
}
STATUS_LABEL = {
    "pending": "待巡视", "visiting": "识别中", "handling": "搬运中",
    "ok": "正确", "wrong": "错误", "timeout": "超时",
    "stock": "未生成", "staged": "识别中", "held": "搬运中",
    "placed": "已入位", "recycled": "已回收",
}

# 自由摄像机预设机位：(x, y, z, roll, pitch, yaw)。俯视图默认机位来自 sim_layout.yaml。
CAM_PRESETS = {
    "overhead": (0.00, 0.02, 1.62, 0.0, 1.5708, 1.5708),   # 全景正俯视（默认）
    "top":      (0.00, -0.95, 1.15, 0.0, 0.88, 1.5708),   # 斜俯视（看立体关系）
    "place":    (0.00, -0.15, 0.75, 0.0, 0.965, 1.5708),  # 码放区特写
    "rack":     (0.00, -0.70, 0.85, 0.0, 0.82, 1.5708),   # 货架区特写
}
CAM_LIMITS = {"x": (-1.4, 1.4), "y": (-1.8, 1.0), "z": (0.25, 2.8)}


class FrameHub:
    """每路相机一个帧缓存（写：ROS 回调线程；读：HTTP 流线程）。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame = None
        self._ts = 0.0

    def publish(self, frame):
        with self._cond:
            self._frame = frame
            self._ts = time.monotonic()
            self._cond.notify_all()

    def wait_frame(self, last_ts, timeout=1.0):
        """阻塞到有比 last_ts 新的帧；返回 (frame, ts)，无新帧返回 (None, last_ts)。"""
        with self._cond:
            if self._ts == last_ts:
                self._cond.wait(timeout=timeout)
                if self._ts == last_ts:
                    return None, last_ts
            return self._frame, self._ts


class ScoreState:
    """判分 + 态势状态（线程安全：ROS 回调线程写，HTTP 线程读）。

    M3 语义：slot_id 既是决策的巡视位点号，也是物料编号（material_slot）。
    物料流：停料区（stock，底板下不可见）→ 置物台（staged）→ 吸盘（held）
    → 货架列位（placed）/ 回收位（recycled，识别失败未取走）。
    判分真值：types[slot-1]（与决策 CSV / gt 桩同一份事实）。
    """

    def __init__(self, doc, types):
        self.doc = doc
        self.types = list(types)                  # slot_id -> 真值类型
        rack = doc.get("rack", {})
        self.rows = int(rack.get("rows", 3))
        self.cols = int(rack.get("cols", 4))
        self.pitch_x = float(rack.get("pitch_x", 0.25))
        self.rack_y_front = float(rack.get("y_front", rack.get("y", 0.30)))
        self.step_y = float(rack.get("level_step_y", 0.0))
        self.top_z = float(rack.get("top_z", 0.10))
        self.lpz = float(rack.get("level_pitch_z", 0.06))
        st = doc.get("staging", {})
        self.staging = (float(st.get("x", 0.30)), float(st.get("y", -0.15)))
        # 停料区（M5）：12 块物料停在底板下方，看板不画；上料后才出现在置物台
        self.magazine = (0.0, self.staging[1])
        rc = doc.get("recycle", {})
        self.recycle = (float(rc.get("x", -0.10)), float(rc.get("y", -0.15)))
        # 吸盘工具基准（z_joint=0 的吸口下沿高度）与盘厚：用于从 gantry z 反推层位
        self.pad_z0 = float(doc.get("gantry", {}).get("pad_z0", 0.239))
        self.plate_th = float(doc.get("plate", {}).get("thickness", 0.006))

        self.lock = threading.Lock()
        self.slots = {}                           # slot(=轮次位点) -> dict(status, got, visit, t)
        for s in range(1, len(types) + 1):
            # since = 进入 visiting 的时刻（epoch），只给「末位点超时」用；
            # t 是给人看的 HH:MM:SS 字符串，两者别混用。
            self.slots[s] = {"status": "pending", "got": None,
                             "visit": None, "since": None, "t": None}
        self.task_type = {}                       # HANDLE 已知的判型（slot -> type）
        self.events = []
        self.extra = {}          # 节点侧补充的状态（手动模式/帧龄/端口），随快照下发                          # [(time_str, text)]
        self.cur_visit = 0
        self.cur_slot = None
        self.last_vision = None                   # 最近一条识别结果（显示用）

        # ---- 态势：龙门 + 物料 ----
        self.gantry = {"x": 0.0, "y": 0.0, "z": 0.0, "moving": False}
        self.plates = {}                          # slot -> dict(type,status,x,y)
        for s in range(1, len(types) + 1):
            self.plates[s] = {"type": int(types[s - 1]), "status": "stock",
                              "x": 0.0, "y": 0.0}

    def col_x(self, mtype):
        """类型列的格心 x（列序即类型号 1..cols）。"""
        return (int(mtype) - (self.cols + 1) / 2.0) * self.pitch_x

    def level_of_z(self, z_joint):
        """由放置时的 z_joint 反推层位（1 起）：deck = z_joint + pad_z0 - 盘厚。"""
        deck = z_joint + self.pad_z0 - self.plate_th
        return max(1, min(self.rows,
                          int(round((deck - self.top_z) / self.lpz)) + 1))

    # ---------- ROS 回调侧 ----------
    def on_task(self, msg):
        with self.lock:
            if msg.action == "GOTO":
                slot = int(msg.slot_id)
                self.cur_visit, self.cur_slot = int(msg.visit_index), slot
                self.gantry["moving"] = True
                # 超时兜底：决策已开始巡视下一个位点 = 上一位点视觉没回结果 = TIMEOUT
                for s, st in self.slots.items():
                    if s != slot and st["status"] == "visiting":
                        self._judge_locked(s, None)
                # 台面上未被取走的物料（上一位点识别失败）才会被控制侧送回收位。
                # 不能一见新 GOTO 就回收：决策把 HANDLE(N) 与 GOTO(N+1) 背靠背发出，
                # 这时 HANDLE(N) 已到、物料却还在台面上（控制器队列里 HANDLE 排前面），
                # 无条件回收会把正常搬运的物料误标「已回收」（实测一轮误标 5/12）。
                for i, pl in self.plates.items():
                    if (i != slot and pl["status"] == "staged"
                            and i not in self.task_type):
                        pl["status"] = "recycled"
                        pl["x"], pl["y"] = self.recycle
                # 上料：material_slot -> 置物台
                pl = self.plates.get(slot)
                if pl is not None and pl["status"] in ("stock", "recycled"):
                    pl["status"] = "staged"
                    pl["x"], pl["y"] = self.staging
                if self.slots[slot]["status"] == "pending":
                    self.slots[slot]["status"] = "visiting"
                st = self.slots[slot]
                if st["status"] == "visiting":
                    st["visit"] = self.cur_visit
                    st["since"] = time.time()
            elif msg.action == "HANDLE":
                slot = int(msg.slot_id)
                self.task_type[slot] = int(msg.material_type)
                if self.slots[slot]["status"] in ("pending", "visiting"):
                    self.slots[slot]["status"] = "handling"

    def on_arrived(self, msg):
        slot = int(msg.data)
        with self.lock:
            st = self.slots.get(slot)
            if st is None:
                return
            self.gantry["moving"] = False
            # 已落判的位点又被巡视 = 决策开了新一轮：自动清盘（多轮连拍时
            # 免去手动 POST /api/reset；单轮模式不会触发——每轮每位点只走一次）。
            if st["status"] in ("ok", "wrong", "timeout"):
                for s2 in self.slots.values():
                    s2.update(status="pending", got=None, visit=None,
                              since=None, t=None)
                self.task_type.clear()
                self.events.append((time.strftime("%H:%M:%S"), "检测到新一轮巡视，已自动清盘"))
            st["status"] = "visiting"
            st["visit"] = self.cur_visit
            st["since"] = time.time()

    def on_vision(self, msg):
        with self.lock:
            self.last_vision = msg.data

    def on_done(self, msg):
        slot = int(msg.data)
        with self.lock:
            got = self.task_type.pop(slot, None)
            if slot in self.slots and self.slots[slot]["status"] in (
                    "pending", "visiting", "handling"):
                self._judge_locked(slot, got)

    def on_joint(self, msg):
        """龙门实时位置（关节值即世界 x/y；z 为 z_joint）。"""
        idx = {}
        for i, name in enumerate(msg.name):
            if "x_joint" in name:
                idx["x"] = i
            elif "y_joint" in name:
                idx["y"] = i
            elif "z_joint" in name:
                idx["z"] = i
        if not idx or len(msg.position) <= max(idx.values()):
            return  # 名字对不上时放弃本帧（保持上一位置即可）
        with self.lock:
            self.gantry["x"] = float(msg.position[idx["x"]])
            self.gantry["y"] = float(msg.position[idx["y"]])
            self.gantry["z"] = float(msg.position[idx["z"]])
            for pl in self.plates.values():
                if pl["status"] == "held":
                    pl["x"], pl["y"] = self.gantry["x"], self.gantry["y"]

    def on_grip(self, slot, attach):
        """吸/放事件：物料开始跟随龙门 / 落到对应类型列的层位。

        门槛（防误判）：开机时控制器会广播 12 路 detach 松爪（坑 13），
        那不是「放到货架」——所以 detach 只认「物料当前是 held」的；
        attach 只认「物料已上到置物台」的。
        """
        with self.lock:
            pl = self.plates.get(slot)
            if pl is None:
                return
            if attach and pl["status"] == "staged":
                pl["status"] = "held"
                pl["x"], pl["y"] = self.gantry["x"], self.gantry["y"]
                self.events.append((time.strftime("%H:%M:%S"), "物料 %02d（%s）已吸取" % (
                    slot, TYPE_NAMES.get(pl["type"], pl["type"]))))
                del self.events[:-40]
            elif not attach and pl["status"] == "held":
                t = pl["type"]
                # 落位：类型列格心；层位由放置时的 z_joint 反推（阶梯 y）
                pl["status"] = "placed"
                level = self.level_of_z(self.gantry["z"])
                pl["x"] = self.col_x(t)
                # 第 1 层（最低）在最靠 -Y 的一侧，往上逐层后退（与货架 V 形一致）
                pl["y"] = self.rack_y_front - (self.rows - level) * self.step_y
                pl["level"] = level
                self.events.append((time.strftime("%H:%M:%S"),
                                    "物料 %02d 放入货架（%s 列 · 第 %d 层）" % (
                                        slot, TYPE_NAMES.get(t, t), level)))
                del self.events[:-40]

    def _judge_locked(self, slot, got):
        st = self.slots[slot]
        true_t = self.types[slot - 1]
        st["got"] = got
        st["t"] = time.strftime("%H:%M:%S")
        if got is None:
            st["status"] = "timeout"
            text = "位点 %d：未识别（TIMEOUT）" % slot
        elif got == true_t:
            st["status"] = "ok"
            text = "位点 %d：正确（%s）" % (slot, TYPE_NAMES.get(got, got))
        else:
            st["status"] = "wrong"
            text = "位点 %d：错误（真值 %s，判成 %s）" % (
                slot, TYPE_NAMES.get(true_t, true_t), TYPE_NAMES.get(got, got))
        self.events.append((st["t"], text))
        del self.events[:-30]
        if all(s["status"] in ("ok", "wrong", "timeout") for s in self.slots.values()):
            ok = sum(1 for s in self.slots.values() if s["status"] == "ok")
            self.events.append((time.strftime("%H:%M:%S"),
                                "本轮完成：正确 %d / %d" % (ok, len(self.types))))

    def reset(self):
        with self.lock:
            for st in self.slots.values():
                st.update(status="pending", got=None, visit=None, since=None, t=None)
            self.task_type.clear()
            # 物料也要一起归位：不重置的话第二轮起 plates 还停在上一轮的
            # placed/recycled，态势图会把「已放进货架」画成仍占着来料架格位。
            for idx, pl in self.plates.items():
                pl.update(status="stock", x=0.0, y=0.0)
            self.events.append((time.strftime("%H:%M:%S"), "已重置，等待新一轮巡视"))
            self.cur_visit, self.cur_slot = 0, None
            self.last_vision = None

    def check_timeout(self, limit_s):
        """末位点兜底：最后一条 GOTO 之后不会再有新任务，无法靠「换位」推断超时。

        前 11 个位点靠「新 GOTO 到达时上一位仍在识别中」判 TIMEOUT；
        第 12 位没有下一条 GOTO，若识别失败会永远停在「识别中」，
        看板就永远差一格不变绿。这里用时间兜底（阈值要大于正常
        「移动 + 识别 + 搬运」的总时长，默认 35 s）。
        """
        now = time.time()
        with self.lock:
            for slot, st in self.slots.items():
                if st["status"] != "visiting" or not st.get("since"):
                    continue
                if now - st["since"] > limit_s:
                    self._judge_locked(slot, None)

    # ---------- HTTP 侧 ----------
    def snapshot(self):
        with self.lock:
            ok = sum(1 for s in self.slots.values() if s["status"] == "ok")
            wrong = sum(1 for s in self.slots.values() if s["status"] == "wrong")
            to = sum(1 for s in self.slots.values() if s["status"] == "timeout")
            pending = len(self.slots) - ok - wrong - to
            col_xs = [self.col_x(c) for c in range(1, self.cols + 1)]
            return {
                "total": len(self.slots),
                "score": {"ok": ok, "wrong": wrong, "timeout": to, "pending": pending},
                "cur_visit": self.cur_visit,
                "cur_slot": self.cur_slot,
                "last_vision": self.last_vision,
                "done": pending == 0,
                "slots": [{"id": s, "true_type": self.types[s - 1], **st}
                          for s, st in sorted(self.slots.items())],
                "events": list(self.events),
                # ---- 态势图 ----
                "gantry": dict(self.gantry),
                "plates": [{"id": s, **pl} for s, pl in sorted(self.plates.items())],
                "sys": dict(self.extra),
                "map": {"cols": self.cols, "rows": self.rows,
                        "col_xs": col_xs, "rack_y": self.rack_y_front,
                        "step_y": self.step_y, "lpz": self.lpz,
                        "staging": list(self.staging),
                        "magazine": list(self.magazine),
                        "recycle": list(self.recycle)},
            }


def build_page():
    """判分看板单页：布局对齐官方 VirtualNext 控制窗口。

    左：全景视图 + 底部三联（库位监视 / 放料位结果 / 态势图）
    右：(6) 运行状态 (2) XYZ 手动调试 (3) 抓取/放置/生成物料/拍照 (4) 日志 (5) 重置/退出
    """
    return """<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>AIC 视觉分拣 · 控制与判分</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; margin: 0; }
  body { background:#0f172a; color:#e2e8f0; font-family:"Noto Sans CJK SC","WenQuanYi Micro Hei",sans-serif;
         height:100vh; display:flex; flex-direction:column; padding:10px 14px; gap:8px; overflow:hidden; }
  header { display:flex; align-items:center; gap:18px; flex:0 0 auto; }
  h1 { font-size:19px; font-weight:700; letter-spacing:1px; }
  .chip { padding:4px 14px; border-radius:9px; font-size:18px; font-weight:700; }
  .chip small { font-size:11px; font-weight:400; opacity:.85; margin-right:6px; }
  .ok   { background:#14532d; color:#4ade80; border:1px solid #22c55e; }
  .err  { background:#7f1d1d; color:#f87171; border:1px solid #ef4444; }
  .rest { background:#1e293b; color:#cbd5e1; border:1px solid #334155; }
  #cur { font-size:13px; color:#94a3b8; }
  main { flex:1 1 auto; min-height:0; display:grid; grid-template-columns:minmax(0,1fr) 372px; gap:10px; }
  .left { min-width:0; min-height:0; display:grid; grid-template-rows:minmax(140px,1fr) auto 252px; gap:8px; }
  .strip { display:grid; grid-template-columns:1.12fr 1fr 0.86fr; gap:10px; min-height:0; }
  .panel { background:#1e293b; border-radius:12px; overflow:hidden; min-width:0; }
  .cell { display:flex; flex-direction:column; min-height:0; }
  .cell .cap { font-size:12px; color:#94a3b8; padding:6px 10px 4px; flex:0 0 auto; }
  .cell img { width:100%; height:0; object-fit:contain; background:#0b1220; flex:1 1 auto; min-height:0; }
  .cell svg { flex:1 1 auto; min-height:0; display:block; }
  .vwrap { position:relative; background:#000; }
  .vwrap > img { width:100%; height:100%; object-fit:contain; }
  .vlabel { position:absolute; top:8px; right:10px; font-size:12px; color:#94a3b8;
            background:rgba(2,6,23,.65); padding:2px 10px; border-radius:6px; }
  .cambar { display:grid; grid-template-columns:repeat(6,1fr); gap:5px; }
  .cambar button { background:#111c2e; color:#cbd5e1; border:1px solid #334155;
                   border-radius:7px; padding:6px 2px; font-size:12px; cursor:pointer; }
  .cambar button:hover { border-color:#64748b; color:#fff; }
  .cambar .preset { border-color:#7c3aed; color:#c4b5fd; }
  #banner { position:absolute; left:10px; top:8px; padding:4px 12px; font-size:13px;
            border-radius:6px; background:rgba(2,6,23,.72); display:none; }
  #banner.ok { color:#4ade80; } #banner.vis { color:#fbbf24; }
  aside { min-height:0; display:flex; flex-direction:column; gap:8px; overflow-y:auto; padding-right:2px; }
  main > * { min-height:0; }
  .sec { padding:8px 10px; flex:0 0 auto; }
  .sec h2 { font-size:12px; color:#94a3b8; font-weight:600; letter-spacing:.5px;
            margin-bottom:6px; display:flex; justify-content:space-between; align-items:center; }
  .sec h2 .tag { font-size:11px; color:#64748b; font-weight:400; }
  .stat { display:grid; grid-template-columns:auto 1fr; gap:2px 10px; font-size:12px; color:#cbd5e1; }
  .stat b { color:#94a3b8; font-weight:400; }
  .stat .on { color:#4ade80; } .stat .off { color:#f87171; }
  .xyz { display:grid; grid-template-columns:20px 62px 1fr 26px 26px 44px 44px; gap:4px;
         align-items:center; font-size:12px; margin-bottom:5px; }
  .xyz b { color:#7dd3fc; }
  .xyz .now { color:#94a3b8; text-align:right; font-variant-numeric:tabular-nums; }
  .xyz input, .spd input { background:#0b1220; border:1px solid #334155; color:#e2e8f0;
         border-radius:5px; padding:3px 4px; font-size:12px; width:100%; text-align:right; }
  .xyz button, .spd button, .acts button, .foot button, .mbtn {
         background:#0b1220; color:#cbd5e1; border:1px solid #334155; border-radius:6px;
         font-size:12px; padding:4px 2px; cursor:pointer; }
  .xyz button:hover, .acts button:hover, .foot button:hover, .mbtn:hover { border-color:#64748b; color:#fff; }
  .spd { display:grid; grid-template-columns:auto 70px 58px 1fr 78px; gap:6px; align-items:center;
         font-size:12px; margin-top:6px; color:#94a3b8; }
  .spd .hint { font-size:11px; color:#475569; }
  .mbtn { padding:6px 10px; }
  .mbtn.on { background:#7c2d12; border-color:#f97316; color:#fdba74; }
  .acts { display:grid; grid-template-columns:1fr 1fr; gap:6px; }
  .acts button { padding:8px 4px; font-size:13px; }
  .acts button.pri { border-color:#0ea5e9; color:#7dd3fc; }
  #log { flex:1 1 auto; min-height:120px; overflow-y:auto; background:#020617;
         border:1px solid #1e293b; border-radius:10px; padding:6px 9px; font-size:12px; line-height:1.6; }
  #log .e-ok { color:#4ade80; } #log .e-bad { color:#f87171; } #log .e-info { color:#94a3b8; }
  .foot { display:grid; grid-template-columns:1fr 1fr; gap:6px; }
  .foot button { padding:7px 0; font-size:13px; }
  .foot .danger { border-color:#7f1d1d; color:#fca5a5; }
  #grid { display:grid; grid-template-columns:repeat(4,1fr); gap:5px; padding:0 6px 6px;
          flex:1 1 auto; min-height:0; }
  .slot { border-radius:7px; background:#0b1220; border:2px solid #334155; display:flex;
          flex-direction:column; align-items:center; justify-content:center; min-height:0; }
  .slot .id { font-size:15px; font-weight:800; line-height:1.1; }
  .slot .gt { font-size:10px; color:#94a3b8; min-height:12px; }
  .slot.pending { opacity:.45; }
  .slot.visiting, .slot.handling { border-color:#fbbf24; background:#3a2e10; }
  .slot.ok { border-color:#22c55e; background:#14532d; }
  .slot.wrong, .slot.timeout { border-color:#ef4444; background:#7f1d1d; }
  #toast { position:fixed; right:16px; bottom:14px; background:#0b1220; border:1px solid #334155;
           color:#e2e8f0; border-radius:8px; padding:7px 12px; font-size:12px; opacity:0;
           transition:opacity .2s; max-width:360px; }
  svg text { font-family:inherit; }
</style></head>
<body>
<header>
  <h1>AIC 视觉分拣 · 控制与判分</h1>
  <div class="chip ok"><small>正确</small><span id="s-ok">0</span></div>
  <div class="chip err"><small>错误</small><span id="s-err">0</span></div>
  <div class="chip rest"><small>剩余</small><span id="s-rest">12</span></div>
  <div id="cur"></div>
</header>
<main>
  <section class="left">
    <div class="vwrap panel" id="scene_v">
      <img src="/scene.mjpg" alt="scene">
      <div class="vlabel">全景 · 自由摄像机</div>
      <div id="banner"></div>
    </div>
    <div class="cambar">
      <button class="preset" onclick="cam({preset:'overhead'})">全景俯视</button>
      <button class="preset" onclick="cam({preset:'top'})">正俯视</button>
      <button class="preset" onclick="cam({preset:'rack'})">货架特写</button>
      <button class="preset" onclick="cam({preset:'place'})">码放特写</button>
      <button onclick="cam({dy:-0.15})">前</button>
      <button onclick="cam({dy:0.15})">后</button>
      <button onclick="cam({dx:-0.15})">左</button>
      <button onclick="cam({dx:0.15})">右</button>
      <button onclick="cam({dz:0.15})">升</button>
      <button onclick="cam({dz:-0.15})">降</button>
      <button onclick="cam({dx:-0.5,dy:0.5,dz:0.3,preset:'overhead'})">回位</button>
      <button onclick="cam({home:1})">默认</button>
    </div>
    <div class="strip">
      <div class="panel cell">
        <div class="cap">库位监视 · 4 列 × 3 层（边框：灰 空 / 琥珀 放入中 / 绿 已放）</div>
        <img src="/rack.mjpg" alt="rack wall">
      </div>
      <div class="panel cell">
        <div class="cap">放料位结果 · 12 个位点</div>
        <div id="grid"></div>
      </div>
      <div class="panel cell">
        <div class="cap">态势图</div>
        <svg id="map" viewBox="0 0 372 300" preserveAspectRatio="xMidYMid meet"
             style="width:100%;height:100%"></svg>
      </div>
    </div>
  </section>
  <aside>
    <div class="panel sec">
      <h2>运行状态 <span class="tag">(6)</span></h2>
      <div class="stat">
        <b>视觉相机</b><span id="st-img">-</span>
        <b>全景相机</b><span id="st-scene">-</span>
        <b>关节反馈</b><span id="st-joint">-</span>
        <b>库位相机</b><span id="st-rack">-</span>
        <b>控制模式</b><span id="st-mode">-</span>
        <b>看板端口</b><span id="st-port">-</span>
      </div>
    </div>

    <div class="panel sec">
      <h2>手动调试 · XYZ <span class="tag">(2)</span></h2>
      <div class="xyz"><b>X</b><span class="now" id="x-now">0.0</span>
        <input id="x-in" value="0.0"><button onclick="step('x',-1)">−</button><button onclick="step('x',1)">+</button>
        <button onclick="moveAxis('x')">移动</button><button onclick="copyNow('x')">位置</button></div>
      <div class="xyz"><b>Y</b><span class="now" id="y-now">0.0</span>
        <input id="y-in" value="0.0"><button onclick="step('y',-1)">−</button><button onclick="step('y',1)">+</button>
        <button onclick="moveAxis('y')">移动</button><button onclick="copyNow('y')">位置</button></div>
      <div class="xyz"><b>Z</b><span class="now" id="z-now">0.0</span>
        <input id="z-in" value="0.0"><button onclick="step('z',-1)">−</button><button onclick="step('z',1)">+</button>
        <button onclick="moveAxis('z')">移动</button><button onclick="copyNow('z')">位置</button></div>
      <div class="spd">
        <span>速度</span><input id="spd-in" value="0.20"><span class="hint">m/s</span>
        <button class="mbtn" id="mbtn" onclick="toggleManual()">手动：关</button>
        <button onclick="goHome()">原点</button>
      </div>
    </div>

    <div class="panel sec">
      <h2>手动调试 · 动作 <span class="tag">(3)</span></h2>
      <div class="acts">
        <button class="pri" onclick="cmd({action:'grab'})">抓取</button>
        <button class="pri" onclick="cmd({action:'release'})">放置</button>
        <button onclick="cmd({action:'spawn'})">生成物料</button>
        <button onclick="cmd({action:'photo'})">拍照</button>
      </div>
    </div>

    <div class="panel sec" style="display:flex;flex-direction:column;min-height:150px;flex:1 1 auto">
      <h2>运行日志 <span class="tag">(4)</span></h2>
      <div id="log"></div>
    </div>

    <div class="panel sec">
      <h2>系统 <span class="tag">(5)</span></h2>
      <div class="foot">
        <button onclick="cmd({action:'reset'})">重置判分</button>
        <button class="danger" onclick="cmd({action:'exit'})">退出看板</button>
      </div>
    </div>
  </aside>
</main>
<div id="toast"></div>
<script>
const TN = {1:"划痕",2:"污渍",3:"破损",4:"孔洞"};
const TC = {1:"#38bdf8",2:"#fb923c",3:"#a78bfa",4:"#34d399"};
const grid = document.getElementById("grid");
for (let i = 1; i <= 12; i++) {
  grid.insertAdjacentHTML("beforeend",
    `<div class="slot pending" id="slot-${i}"><div class="id">${i}</div><div class="gt"></div></div>`);
}
let lastState = null, sysManual = false, editing = null;

function toast(msg) {
  const t = document.getElementById("toast");
  t.textContent = msg; t.style.opacity = 1;
  clearTimeout(t._h); t._h = setTimeout(() => { t.style.opacity = 0; }, 2200);
}
async function cam(body) {
  try { await fetch("/api/cam", {method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify(body)}); } catch (e) {}
}
async function cmd(body) {
  try {
    const r = await fetch("/api/cmd", {method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify(body)});
    const d = await r.json();
    toast((d.ok ? "✓ " : "✗ ") + d.info);
    if (body.action === "manual") { sysManual = !!body.on; paintManual(); }
  } catch (e) { toast("命令失败：" + e); }
}
function paintManual() {
  const b = document.getElementById("mbtn");
  b.textContent = "手动：" + (sysManual ? "开" : "关");
  b.className = "mbtn" + (sysManual ? " on" : "");
}
function toggleManual() { cmd({action:"manual", on: !sysManual}); }
function step(a, sgn) {
  const el = document.getElementById(a + "-in");
  const v = parseFloat(el.value || "0") + sgn * 1.0;
  el.value = v.toFixed(1);
}
function copyNow(a) {
  if (!lastState) return;
  document.getElementById(a + "-in").value = (lastState.gantry[a] * 1000).toFixed(1);
}
function moveAxis(a) {
  if (!sysManual) { toast("先打开手动模式（自动流程占用中，点动无效）"); return; }
  const v = parseFloat(document.getElementById(a + "-in").value || "0") / 1000.0;
  const body = {action:"jog"}; body[a] = v; cmd(body);
}
function goHome() {
  if (!sysManual) { toast("先打开手动模式"); return; }
  cmd({action:"home"});
}
document.getElementById("spd-in").addEventListener("change", () => {
  cmd({action:"speed", value: parseFloat(document.getElementById("spd-in").value || "0.2")});
});
for (const a of ["x","y","z"]) {
  const el = document.getElementById(a + "-in");
  el.addEventListener("focus", () => { editing = a; });
  el.addEventListener("blur", () => { editing = null; });
}

function drawMap(d) {
  const S = 320, ox = 0.58, oy = 0.42;
  const X = x => (x + ox) * S, Y = y => (y + oy) * S;
  const m = d.map;
  let s = `<rect width="100%" height="100%" fill="#0b1220"/>`;
  for (let lv = 0; lv < m.rows; lv++) {
    const ry = m.rack_y - (m.rows - 1 - lv) * m.step_y;
    m.col_xs.forEach((zx, i) => {
      s += `<rect x="${X(zx)-17}" y="${Y(ry)-17}" width="34" height="34"
        fill="#1e293b" stroke="#475569" rx="4"/>`;
      if (lv === 0) {
        s += `<text x="${X(zx)}" y="${Y(ry)+4}" fill="#64748b" font-size="10"
          text-anchor="middle">T${i+1}</text>`;
      }
      s += `<text x="${X(zx)+14}" y="${Y(ry)-8}" fill="#334155" font-size="8"
        text-anchor="middle">L${lv+1}</text>`;
    });
  }
  s += `<rect x="${X(m.staging[0])-20}" y="${Y(m.staging[1])-20}" width="40" height="40"
    fill="#132a1e" stroke="#22c55e" rx="4" stroke-dasharray="4 3"/>
    <text x="${X(m.staging[0])}" y="${Y(m.staging[1])+28}" fill="#22c55e" font-size="9"
    text-anchor="middle">置物台</text>`;
  s += `<circle cx="${X(m.recycle[0])}" cy="${Y(m.recycle[1])}" r="12" fill="none"
    stroke="#a16207" stroke-dasharray="3 3"/>
    <text x="${X(m.recycle[0])}" y="${Y(m.recycle[1])+24}" fill="#a16207" font-size="9"
    text-anchor="middle">回收位</text>`;
  for (const p of d.plates) {
    if (p.status === 'stock') continue;
    const pulse = p.status === 'held' ? ` stroke="#fbbf24" stroke-width="3"` : "";
    s += `<circle cx="${X(p.x)}" cy="${Y(p.y)}" r="11" fill="${TC[p.type]}"${pulse}/>
      <text x="${X(p.x)}" y="${Y(p.y)+4}" fill="#0b1220" font-size="10" font-weight="bold"
      text-anchor="middle">${p.id}</text>`;
  }
  const gx = X(d.gantry.x), gy = Y(d.gantry.y);
  s += `<line x1="${gx-14}" y1="${gy}" x2="${gx+14}" y2="${gy}" stroke="#fbbf24" stroke-width="2"/>
    <line x1="${gx}" y1="${gy-14}" x2="${gx}" y2="${gy+14}" stroke="#fbbf24" stroke-width="2"/>
    <circle cx="${gx}" cy="${gy}" r="8" fill="none" stroke="#fbbf24" stroke-width="2"/>`;
  document.getElementById("map").innerHTML = s;
}

function age(v, unit) {
  if (v === null || v === undefined) return `<span class="off">未连接</span>`;
  return (v < 2.0 ? `<span class="on">在线 ${v.toFixed(1)}s</span>`
                  : `<span class="off">超时 ${v.toFixed(1)}s</span>`);
}

async function tick() {
  try {
    const r = await fetch("/api/state"); const d = await r.json();
    lastState = d;
    document.getElementById("s-ok").textContent = d.score.ok;
    document.getElementById("s-err").textContent = d.score.wrong + d.score.timeout;
    document.getElementById("s-rest").textContent = d.score.pending;
    document.getElementById("cur").textContent =
      d.done ? "本轮完成" : ("巡视中 · 第 " + d.cur_visit + " 位点");
    const vis = d.last_vision ? ("识别结果：" + (TN[d.last_vision] || ("非法值 " + d.last_vision))) : "";
    const banner = document.getElementById("banner");
    banner.style.display = vis ? "block" : "none";
    banner.textContent = vis;
    banner.className = (d.last_vision && d.cur_slot && d.slots[d.cur_slot-1]
                        && d.slots[d.cur_slot-1].got == d.last_vision) ? "ok" : "vis";
    for (const s of d.slots) {
      const el = document.getElementById("slot-" + s.id);
      el.className = "slot " + s.status;
      el.querySelector(".gt").textContent =
        s.got ? ("判 " + TN[s.got]) : (s.status === "timeout" ? "未识别" : ("真 " + TN[s.true_type]));
    }
    drawMap(d);
    for (const a of ["x","y","z"]) {
      const mm = (d.gantry[a] * 1000);
      document.getElementById(a + "-now").textContent = mm.toFixed(1);
      if (editing !== a) document.getElementById(a + "-in").value = mm.toFixed(1);
    }
    const sys = d.sys || {};
    if (sys.manual !== undefined && sys.manual !== sysManual) { sysManual = sys.manual; paintManual(); }
    document.getElementById("st-img").innerHTML = age(sys.img_age);
    document.getElementById("st-scene").innerHTML = age(sys.scene_age);
    document.getElementById("st-joint").innerHTML = age(sys.joint_age);
    document.getElementById("st-rack").innerHTML =
      `<span class="${sys.rack_cams === sys.rack_cams_total ? 'on' : 'off'}">`
      + (sys.rack_cams || 0) + " / " + (sys.rack_cams_total || 12) + "</span>";
    document.getElementById("st-mode").textContent = sys.manual ? "手动调试" : "自动巡视";
    document.getElementById("st-port").textContent = "127.0.0.1:" + (sys.port || 8080);
    document.getElementById("spd-in").value = (sys.speed || 0.2).toFixed(2);
    const log = document.getElementById("log");
    const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 24;
    log.innerHTML = (d.events || []).map(([t, txt]) => {
      const cls = txt.includes("正确") ? "e-ok" : (txt.includes("错误") || txt.includes("超时") ? "e-bad" : "e-info");
      return `<div class="${cls}">${t} ${txt}</div>`;
    }).join("");
    if (atBottom) log.scrollTop = log.scrollHeight;
  } catch (e) { /* 后端重启中，下一拍再试 */ }
}
tick(); setInterval(tick, 500);
</script>
</body></html>"""


class DashboardHTTP:
    """把 ScoreState 与两路相机帧以 HTTP 暴露出去（只绑 127.0.0.1）。"""

    def __init__(self, state, port, move_cam, cmd=None):
        self.state = state
        self.move_cam = move_cam
        self.cmd = cmd or (lambda body: (False, "no cmd"))          # (dict) -> None，自由摄像机控制

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):   # 静默访问日志，别刷 ROS 日志
                pass

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/":
                    self._send(200, build_page().encode("utf-8"), "text/html; charset=utf-8")
                elif self.path == "/api/state":
                    self._send(200, json.dumps(self.server.state.snapshot(),
                                               ensure_ascii=False).encode("utf-8"),
                               "application/json; charset=utf-8")
                elif self.path == "/stream.mjpg":
                    self._stream(self.server.hub_ee)
                elif self.path == "/scene.mjpg":
                    self._stream(self.server.hub_scene)
                elif self.path == "/rack.mjpg":
                    self._stream(self.server.hub_rack)
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if self.path == "/api/reset":
                    self.server.state.reset()
                    self._send(200, b'{"ok":true}', "application/json")
                elif self.path == "/api/cmd":
                    try:
                        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])) or b"{}")
                        ok, info = self.server.cmd(body)
                        self._send(200 if ok else 400,
                                   json.dumps({"ok": ok, "info": info}, ensure_ascii=False).encode(),
                                   "application/json")
                    except Exception as e:                            # noqa: BLE001
                        self._send(400, json.dumps({"ok": False, "info": str(e)}).encode(),
                                   "application/json")
                elif self.path == "/api/cam":
                    try:
                        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                        self.server.move_cam(body)
                        self._send(200, b'{"ok":true}', "application/json")
                    except Exception as e:                            # noqa: BLE001
                        self._send(400, json.dumps({"ok": False, "err": str(e)}).encode(),
                                   "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

            def _stream(self, hub):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last_ts = 0.0
                while True:
                    frame, last_ts = hub.wait_frame(last_ts)
                    if frame is None:
                        continue
                    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
                    if not ok:
                        continue
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg.tobytes())
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return

        self.httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.httpd.state = state
        self.httpd.move_cam = move_cam
        self.httpd.cmd = self.cmd
        self.httpd.hub_ee = FrameHub()      # 末端相机
        self.httpd.hub_scene = FrameHub()   # 自由摄像机
        self.httpd.hub_rack = FrameHub()    # 库位监视墙（12 格合成图）
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True, name="dashboard-http")

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()

    def publish_frame(self, frame, scene=False, rack=False):
        """相机线程调用：缓存最新帧并唤醒流客户端。"""
        hub = self.httpd.hub_rack if rack else (
            self.httpd.hub_scene if scene else self.httpd.hub_ee)
        hub.publish(frame)


class ScoreNode(Node):
    def __init__(self):
        super().__init__("aic_score_node")
        layout_yaml = self.declare_parameter("layout_yaml", "").value
        port = int(self.declare_parameter("http_port", 8080).value)
        self.port = port
        self.world = self.declare_parameter("world", "aic_warehouse").value
        # 这些话题原来「声明了但没用」，订阅处写死常量；现场改话题名会静默失效。
        task_topic = self.declare_parameter("task_topic", "/control/material_task").value
        arrived_topic = self.declare_parameter("arrived_topic", "/sim/gantry_arrived").value
        handled_topic = self.declare_parameter("handled_topic", "/sim/handle_done").value
        vision_topic = self.declare_parameter("vision_topic", "/vision/material_type").value
        image_topic = self.declare_parameter("image_topic", "/camera/image").value
        scene_image_topic = self.declare_parameter("scene_image_topic", "/scene/image").value
        # 末位点超时兜底（见 ScoreState.check_timeout）
        self.visit_timeout_s = float(
            self.declare_parameter("visit_timeout_s", 35.0).value)
        self.declare_parameter("joint_state_topic",
                               "/world/aic_warehouse/model/aic_gantry/joint_state")

        self.layout = self._load_layout(layout_yaml)
        types = [int(t) for t in self.layout.get("materials", {}).get("types", [])]
        if not types:
            raise RuntimeError("sim_layout.yaml 里没有 materials.types，无法判分")
        self.state = ScoreState(self.layout, types)
        self.cam = CAM_PRESETS.get(
            "overhead", (0.0, -0.95, 1.15, 0.0, 0.88, 1.5708))
        default_cam = (self.layout.get("scene_cam") or {}).get("pose")
        if default_cam and len(default_cam) == 6:
            self.cam = tuple(float(v) for v in default_cam)
        self.cam_lock = threading.Lock()

        self.create_subscription(MaterialTask, task_topic, self.state.on_task, 10)
        self.create_subscription(UInt8, arrived_topic, self.state.on_arrived, 10)
        self.create_subscription(UInt8, handled_topic, self.state.on_done, 10)
        self.create_subscription(String, vision_topic, self.state.on_vision, 10)
        js_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, self.get_parameter(
            "joint_state_topic").value, self._on_joint, js_qos)
        img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self._on_image, img_qos)
        self.create_subscription(Image, scene_image_topic, self._on_scene_image, img_qos)

        # ---- 库位监视墙：每格一只隐藏相机（gen_world.rack_cam_model）----
        rack_cfg = self.layout.get("rack") or {}
        self.rack_cols = int(rack_cfg.get("cols", 4))
        self.rack_rows = int(rack_cfg.get("rows", 3))
        self.rack_wall_hz = float(self.declare_parameter("rack_wall_hz", 5.0).value)
        cam_tpl = self.declare_parameter("rack_cam_topic", "/rack/cam_{:02d}/image").value
        self.rack_frames = {}
        self.rack_lock = threading.Lock()
        for sid in range(1, self.rack_cols * self.rack_rows + 1):
            self.create_subscription(
                Image, cam_tpl.format(sid),
                lambda m, s=sid: self._on_rack_image(m, s), img_qos)
        # 夹/放信号（控制侧本来就发，桥 ROS->GZ；ROS 侧订阅同样收得到）
        for s in range(1, len(types) + 1):
            self.create_subscription(Empty, "/sim/attach_{:02d}".format(s),
                                     lambda m, s=s: self.state.on_grip(s, True), 10)
            self.create_subscription(Empty, "/sim/detach_{:02d}".format(s),
                                     lambda m, s=s: self.state.on_grip(s, False), 10)

        # ---- 手动调试 / 平台控制（对应官方控制面板 2/3/5/6 区）----
        self.manual_on = False
        self.manual_speed = 0.20
        self.cmd_pub = {
            "manual": self.create_publisher(Bool, "/control/manual", 10),
            "jog": self.create_publisher(Vector3, "/control/jog", 10),
            "speed": self.create_publisher(Float64, "/control/speed", 10),
            "home": self.create_publisher(Empty, "/control/home", 10),
            "spawn": self.create_publisher(UInt8, "/control/spawn", 10),
            "trigger": self.create_publisher(Empty, "/vision/trigger", 10),
        }
        self.attach_pub = {s: self.create_publisher(Empty, "/sim/attach_{:02d}".format(s), 10)
                           for s in range(1, len(types) + 1)}
        self.detach_pub = {s: self.create_publisher(Empty, "/sim/detach_{:02d}".format(s), 10)
                           for s in range(1, len(types) + 1)}
        self._t_img = self._t_scene = self._t_joint = 0.0
        self._t_rack = {}
        self.create_timer(1.0, self._update_status)

        # 1 Hz 检查末位点是否卡在「识别中」（第 12 位没有下一条 GOTO 可依赖）
        self.create_timer(1.0, lambda: self.state.check_timeout(self.visit_timeout_s))

        self.dash = DashboardHTTP(self.state, port, self._move_cam, self._on_cmd)
        self.dash.start()
        # 合成线程必须等 dash 建好再起（否则第一拍会 AttributeError: 'dash'）
        threading.Thread(target=self._rack_wall_loop, daemon=True,
                         name="rack-wall").start()
        self.get_logger().info(
            "[dashboard] 判分看板：http://127.0.0.1:%d  （真值 %d 库位：%s；自由摄像机就绪）"
            % (port, len(types), "".join(str(t) for t in types)))

    # ---------- 配置 ----------
    def _load_layout(self, layout_yaml):
        """从 sim_layout.yaml 拿真值与几何（与视觉真值/生成器同一份事实）。"""
        path = layout_yaml
        if not path:
            import os
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("aic_gazebo_sim"),
                                "config", "sim_layout.yaml")
            if not os.path.isfile(path):  # 源码树里直跑
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "..", "..", "simulation", "aic_gazebo_sim",
                                    "config", "sim_layout.yaml")
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    # ---------- 自由摄像机 ----------
    def _on_joint(self, msg):
        self._t_joint = time.monotonic()
        self.state.on_joint(msg)

    def _update_status(self):
        now = time.monotonic()
        rack_alive = sum(1 for t in self._t_rack.values() if now - t < 3.0)
        self.state.extra = {
            "manual": self.manual_on,
            "speed": round(self.manual_speed, 2),
            "img_age": None if not self._t_img else round(now - self._t_img, 1),
            "scene_age": None if not self._t_scene else round(now - self._t_scene, 1),
            "joint_age": None if not self._t_joint else round(now - self._t_joint, 1),
            "rack_cams": rack_alive,
            "rack_cams_total": self.rack_cols * self.rack_rows,
            "port": self.port,
        }

    def _on_cmd(self, body):
        """看板右侧控制面板的动作分发（官方面板 2/3/5 区的对应实现）。

        返回 (ok, info)。所有动作都是「发一条话题就返回」，不等结果——
        结果由状态轮询回显（XYZ 数值、日志、判分格）。
        """
        act = str(body.get("action", "")).strip().lower()
        try:
            if act == "manual":
                on = bool(body.get("on"))
                m = Bool(); m.data = on
                self.cmd_pub["manual"].publish(m)
                self.manual_on = on
                return True, "手动模式" + ("已开启（自动任务暂停）" if on else "已关闭")
            if act == "jog":
                m = Vector3()
                for a in ("x", "y", "z"):
                    v = body.get(a)
                    m.__setattr__(a, float("nan") if v is None else float(v))
                self.cmd_pub["jog"].publish(m)
                return True, "已下发目标位置"
            if act == "speed":
                v = max(0.02, min(0.60, float(body.get("value", 0.20))))
                m = Float64(); m.data = v
                self.cmd_pub["speed"].publish(m)
                self.manual_speed = v
                return True, "速度 %.2f m/s" % v
            if act == "home":
                self.cmd_pub["home"].publish(Empty())
                return True, "回原点"
            if act == "spawn":
                snap = self.state.snapshot()
                target = body.get("id")
                if target is None:
                    rest = [p["id"] for p in snap["plates"] if p["status"] == "stock"]
                    if not rest:
                        return False, "没有待生成的物料（12 块都已上过台面）"
                    target = rest[0]
                m = UInt8(); m.data = int(target)
                self.cmd_pub["spawn"].publish(m)
                return True, "生成物料 %02d -> 置物台" % int(target)
            if act in ("grab", "release"):
                snap = self.state.snapshot()
                want = "staged" if act == "grab" else "held"
                ids = [p["id"] for p in snap["plates"] if p["status"] == want]
                if not ids:
                    return False, ("置物台上没有待抓物料" if act == "grab" else "吸盘上没夹着物料")
                pid = int(ids[0])
                (self.attach_pub if act == "grab" else self.detach_pub)[pid].publish(Empty())
                return True, ("吸附 material %02d" if act == "grab" else "释放 material %02d") % pid
            if act == "photo":
                self.cmd_pub["trigger"].publish(Empty())
                return True, "已触发拍照识别"
            if act == "reset":
                self.state.reset()
                return True, "判分与物料状态已复位"
            if act == "exit":
                threading.Thread(target=self._shutdown, daemon=True).start()
                return True, "看板正在退出（仿真仍在运行，Ctrl-C 结束 launch）"
            return False, "未知动作：%s" % act
        except Exception as exc:                                      # noqa: BLE001
            return False, "命令失败：%s" % exc

    def _shutdown(self):
        try:
            self.dash.stop()
        finally:
            time.sleep(0.3)
            rclpy.shutdown()

    def _move_cam(self, body):
        """预设机位 / 相对平移。子进程调 ign service，不阻塞 HTTP/ROS 线程。"""
        with self.cam_lock:
            if body.get("preset") in CAM_PRESETS or body.get("home"):
                self.cam = CAM_PRESETS.get(body.get("preset"),
                                           CAM_PRESETS["overhead"])
            cx, cy, cz, roll, pitch, yaw = self.cam
            cx = max(CAM_LIMITS["x"][0], min(CAM_LIMITS["x"][1], cx + float(body.get("dx", 0))))
            cy = max(CAM_LIMITS["y"][0], min(CAM_LIMITS["y"][1], cy + float(body.get("dy", 0))))
            cz = max(CAM_LIMITS["z"][0], min(CAM_LIMITS["z"][1], cz + float(body.get("dz", 0))))
            self.cam = (cx, cy, cz, roll, pitch, yaw)
        # 坑：Fortress 的 set_pose 请求类型是 ignition.msgs.Pose（扁平字段），
        # 不是 SetPose；也不接受 entity{...} pose{...} 嵌套。类型写错时报
        # "Unable to create request"，服务没加载时报 "Service call timed out"
        # （需世界里有 UserCommands 插件）。欧拉角按 R = Rz*Ry*Rx 转四元数，
        # 与 SDF <pose> 的 rpy 口径一致。
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cyw, syw = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        qw = cr * cp * cyw + sr * sp * syw
        qx = sr * cp * cyw - cr * sp * syw
        qy = cr * sp * cyw + sr * cp * syw
        qz = cr * cp * syw - sr * sp * cyw
        exe = shutil.which("ign") or "ign"
        req = ('name: "scene_cam" position: {{x: {:.4f} y: {:.4f} z: {:.4f}}} '
               'orientation: {{x: {:.4f} y: {:.4f} z: {:.4f} w: {:.4f}}}').format(
                   cx, cy, cz, qx, qy, qz, qw)
        cmd = [exe, "service", "-s", "/world/{}/set_pose".format(self.world),
               "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
               "--timeout", "2000", "--req", req]
        def run():
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
                out = ((r.stdout or "") + (r.stderr or "")).strip()
                if r.returncode != 0 or "data: true" not in (r.stdout or ""):
                    self.get_logger().warn("[dashboard] set_pose 失败: %s" % out[:200])
            except Exception as e:                                    # noqa: BLE001
                self.get_logger().warn("[dashboard] set_pose 异常: %s" % e)
        threading.Thread(target=run, daemon=True, name="cam-move").start()

    # ---------- 相机 ----------
    def _to_bgr(self, msg):
        if msg.encoding == "mono8":
            return cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        return cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)

    def _on_image(self, msg):
        try:
            frame = self._to_bgr(msg)
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[dashboard] 图像转换失败: %s" % e, throttle_duration_sec=5.0)
            return
        self._t_img = time.monotonic()
        self.dash.publish_frame(self._overlay(frame), scene=False)

    def _on_scene_image(self, msg):
        try:
            frame = self._to_bgr(msg)
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[dashboard] 场景图转换失败: %s" % e, throttle_duration_sec=5.0)
            return
        self._t_scene = time.monotonic()
        self.dash.publish_frame(frame, scene=True)

    # ---------- 库位监视墙 ----------
    def _on_rack_image(self, msg, sid):
        try:
            frame = self._to_bgr(msg)
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[dashboard] 库位相机 %d 图像转换失败: %s" % (sid, e),
                                   throttle_duration_sec=10.0)
            return
        with self.rack_lock:
            self.rack_frames[sid] = frame
        self._t_rack[sid] = time.monotonic()

    def _compose_rack_wall(self):
        """把 12 路格口相机拼成 4 列 x 3 层的一张图（层 3 在上，与现场视角一致）。

        每格左上角标 T<列> L<层>，边框颜色跟随该格物料状态：
        灰=未上料 / 蓝=在置物台 / 琥珀=吸盘持有 / 绿=已放入 / 紫=回收位。
        """
        cols, rows = self.rack_cols, self.rack_rows
        with self.rack_lock:
            frames = dict(self.rack_frames)
        # 用相机原始分辨率拼（128x96），不放大：分辨率由 sim_layout.rack_cams 决定
        sample = next(iter(frames.values()), None)
        th, tw = (sample.shape[:2] if sample is not None else (96, 128))
        canvas = np.full((rows * th, cols * tw, 3), (26, 20, 14), np.uint8)
        # 格口状态要按「类型列 + 层」映射，不能用物料号直接索引格口：
        # 物料的落点由 controller 按「该类型列最低空层」决定，与物料号无关。
        snap = self.state.snapshot()
        status = {}
        for p in snap["plates"]:
            if p["status"] == "placed" and p.get("level"):
                status[(int(p["type"]), int(p["level"]))] = "placed"
        for p in snap["plates"]:
            if p["status"] == "held":
                t = int(p["type"])
                used = [lv for (c, lv) in status if c == t]
                status.setdefault((t, (max(used) + 1) if used else 1), "held")
        for level in range(1, rows + 1):
            for col in range(1, cols + 1):
                sid = (level - 1) * cols + col
                x0, y0 = (col - 1) * tw, (rows - level) * th
                f = frames.get(sid)
                if f is not None:
                    canvas[y0:y0 + th, x0:x0 + tw] = cv2.resize(
                        f, (tw, th), interpolation=cv2.INTER_AREA)
                else:
                    cv2.putText(canvas, "no signal", (x0 + 8, y0 + th // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1, cv2.LINE_AA)
                color = RACK_STATUS_COLORS.get(
                    status.get((col, level), "stock"), (85, 85, 85))
                cv2.rectangle(canvas, (x0 + 1, y0 + 1), (x0 + tw - 2, y0 + th - 2), color, 2)
                label = "T%d L%d" % (col, level)
                cv2.putText(canvas, label, (x0 + 6, y0 + 17), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (15, 15, 15), 3, cv2.LINE_AA)
                cv2.putText(canvas, label, (x0 + 6, y0 + 17), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (235, 235, 235), 1, cv2.LINE_AA)
        return canvas

    def _rack_wall_loop(self):
        period = 1.0 / max(1.0, self.rack_wall_hz)
        while True:
            t0 = time.monotonic()
            try:
                self.dash.publish_frame(self._compose_rack_wall(), rack=True)
            except Exception as e:                                    # noqa: BLE001
                self.get_logger().warn("[dashboard] 库位墙合成失败: %s" % e,
                                       throttle_duration_sec=10.0)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _overlay(self, frame):
        """在末端相机帧上叠加当前位点 / 识别 / 比分（cv2 字体不支持中文，用英文）。"""
        snap = self.state.snapshot()
        ok, err = snap["score"]["ok"], snap["score"]["wrong"] + snap["score"]["timeout"]
        vision = snap["last_vision"] or "-"
        vis_name = TYPE_NAMES_EN.get(int(vision), vision) if str(vision).isdigit() else vision
        slot = snap["cur_slot"]
        true_name = TYPE_NAMES_EN.get(snap["slots"][slot - 1]["true_type"], "?") if slot else "-"
        line1 = "VISIT %02d  SLOT %s   VISION: %s   TRUE: %s" % (
            snap["cur_visit"], ("%02d" % slot) if slot else "--", vis_name, true_name)
        line2 = "SCORE  OK %d / ERR %d / REST %d" % (ok, err, snap["score"]["pending"])
        bar_h = 56
        cv2.rectangle(frame, (0, frame.shape[0] - bar_h), (frame.shape[1], frame.shape[0]),
                      (15, 23, 42), -1)
        cv2.putText(frame, line1, (10, frame.shape[0] - 33),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (226, 232, 240), 2)
        cv2.putText(frame, line2, (10, frame.shape[0] - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (74, 222, 128) if err == 0 else (248, 113, 113), 2)
        return frame


def main():
    rclpy.init()
    node = ScoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # 关停时 spin 抛这个（它继承 RuntimeError，且 rclpy.ok() 仍为真，会穿过下面
        # 的 ok() 判断）——不单独接住就会 exit 1，launch 报 process has died。
        pass
    except RuntimeError as e:
        # 坑（与 vision_node 同源）：SIGINT 后 rclpy 上下文已销毁，executor 仍去建 wait set，
        # 抛 RCLError("the given context is not valid")。此时本轮判分早已完成，
        # 不该让 launch 报「process has died [exit code 1]」伪装成故障。
        if rclpy.ok():
            raise
        node.get_logger().info("[dashboard] 收到退出信号（%s），正常收敛" % e)
    finally:
        try:
            node.dash.stop()
        except Exception:                                          # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
