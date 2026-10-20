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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String, UInt8
from aic_msgs.msg import MaterialTask

import yaml

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
TYPE_NAMES_EN = {1: "SCRATCH", 2: "STAIN", 3: "CHIP", 4: "HOLE"}
STATUS_LABEL = {
    "pending": "待巡视", "visiting": "识别中", "handling": "搬运中",
    "ok": "正确", "wrong": "错误", "timeout": "超时",
    "magazine": "待上料", "staged": "识别中", "held": "搬运中",
    "placed": "已入位", "recycled": "已回收",
}

# 自由摄像机预设机位：(x, y, z, roll, pitch, yaw)。俯视图默认机位来自 sim_layout.yaml。
CAM_PRESETS = {
    "overhead": (0.00, -0.95, 1.15, 0.0, 0.88, 1.5708),   # 全景俯视（默认）
    "top":      (0.00,  0.02, 1.50, 0.0, 1.5708, 0.0),    # 正上方垂直向下
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
    物料流：供料管（magazine）→ 置物台（staged）→ 吸盘（held）→ 货架列位（placed）
    / 回收位（recycled，识别失败未取走）。
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
        mg = doc.get("magazine", {})
        self.magazine = (float(mg.get("x", -0.42)), float(mg.get("y", -0.15)))
        rc = doc.get("recycle", {})
        self.recycle = (float(rc.get("x", -0.10)), float(rc.get("y", -0.15)))
        # 吸盘工具基准（z_joint=0 的吸口下沿高度）与盘厚：用于从 gantry z 反推层位
        self.pad_z0 = float(doc.get("gantry", {}).get("pad_z0", 0.239))
        self.plate_th = float(doc.get("plate", {}).get("thickness", 0.006))

        self.lock = threading.Lock()
        self.slots = {}                           # slot(=轮次位点) -> dict(status, got, visit, t)
        for s in range(1, len(types) + 1):
            self.slots[s] = {"status": "pending", "got": None,
                             "visit": None, "t": None}
        self.task_type = {}                       # HANDLE 已知的判型（slot -> type）
        self.events = []                          # [(time_str, text)]
        self.cur_visit = 0
        self.cur_slot = None
        self.last_vision = None                   # 最近一条识别结果（显示用）

        # ---- 态势：龙门 + 物料 ----
        self.gantry = {"x": 0.0, "y": 0.0, "z": 0.0, "moving": False}
        self.plates = {}                          # slot -> dict(type,status,x,y)
        for s in range(1, len(types) + 1):
            self.plates[s] = {"type": int(types[s - 1]), "status": "magazine",
                              "x": self.magazine[0], "y": self.magazine[1]}

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
                if pl is not None and pl["status"] in ("magazine", "recycled"):
                    pl["status"] = "staged"
                    pl["x"], pl["y"] = self.staging
                if self.slots[slot]["status"] == "pending":
                    self.slots[slot]["status"] = "visiting"
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
                    s2.update(status="pending", got=None, visit=None, t=None)
                self.task_type.clear()
                self.events.append((time.strftime("%H:%M:%S"), "检测到新一轮巡视，已自动清盘"))
            st["status"] = "visiting"

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
                pl["y"] = self.rack_y_front - (level - 1) * self.step_y
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
                st.update(status="pending", got=None, visit=None, t=None)
            self.task_type.clear()
            self.events.append((time.strftime("%H:%M:%S"), "已重置，等待新一轮巡视"))
            self.cur_visit, self.cur_slot = 0, None
            self.last_vision = None

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
                "map": {"cols": self.cols, "rows": self.rows,
                        "col_xs": col_xs, "rack_y": self.rack_y_front,
                        "step_y": self.step_y, "lpz": self.lpz,
                        "staging": list(self.staging),
                        "magazine": list(self.magazine),
                        "recycle": list(self.recycle)},
            }


def build_page():
    """仪表盘单页（无外部依赖，浏览器直开即录）。"""
    return """<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>AIC 判分看板</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; margin: 0; }
  body { background:#0f172a; color:#e2e8f0; font-family:"Noto Sans CJK SC","WenQuanYi Micro Hei",sans-serif;
         height:100vh; display:flex; flex-direction:column; padding:12px 16px; gap:10px; }
  header { display:flex; align-items:center; gap:22px; }
  h1 { font-size:21px; font-weight:700; letter-spacing:1px; }
  .chip { padding:6px 18px; border-radius:10px; font-size:21px; font-weight:700; }
  .chip small { font-size:12px; font-weight:400; opacity:.85; margin-right:8px; }
  .ok   { background:#14532d; color:#4ade80; border:1px solid #22c55e; }
  .err  { background:#7f1d1d; color:#f87171; border:1px solid #ef4444; }
  .rest { background:#1e293b; color:#94a3b8; border:1px solid #475569; }
  #cur  { font-size:15px; color:#fbbf24; }
  main { flex:1; display:flex; gap:14px; min-height:0; }
  .col { display:flex; flex-direction:column; gap:10px; min-width:0; }
  .panel { background:#1e293b; border-radius:12px; overflow:hidden; }
  .vwrap { position:relative; background:#000; border-radius:12px; overflow:hidden; }
  .vwrap img { width:100%; height:100%; object-fit:contain; }
  .vlabel { position:absolute; top:8px; left:10px; font-size:13px; color:#94a3b8;
            background:rgba(2,6,23,.6); padding:2px 10px; border-radius:6px; }
  #scene_v { flex:1; }
  #pip { height:31%; }
  #banner { position:absolute; left:0; right:0; bottom:0; padding:8px 14px; font-size:16px;
            background:rgba(15,23,42,.78); display:none; }
  #banner.ok { color:#4ade80; } #banner.vis { color:#fbbf24; }
  .camctl { display:grid; grid-template-columns:repeat(6,1fr); gap:6px; }
  .camctl button { background:#1e293b; color:#cbd5e1; border:1px solid #334155; border-radius:8px;
                   padding:7px 2px; font-size:13px; cursor:pointer; }
  .camctl button:hover { border-color:#64748b; color:#fff; }
  .camctl .preset { border-color:#7c3aed; color:#c4b5fd; }
  #grid { display:grid; grid-template-columns:repeat(4,1fr); gap:8px; }
  .slot { border-radius:10px; padding:7px 6px; text-align:center; background:#1e293b;
          border:2px solid #334155; transition:all .25s; }
  .slot .id { font-size:21px; font-weight:800; }
  .slot .gt { font-size:12px; margin-top:2px; min-height:15px; color:#94a3b8; }
  .slot.pending  { opacity:.45; }
  .slot.visiting, .slot.handling { border-color:#fbbf24; background:#3a2e10; }
  .slot.ok       { border-color:#22c55e; background:#14532d; }
  .slot.wrong, .slot.timeout { border-color:#ef4444; background:#7f1d1d; }
  #log { flex:1; overflow-y:auto; background:#020617; border:1px solid #1e293b; border-radius:12px;
         padding:8px 12px; font-size:13px; line-height:1.65; }
  #log .e-ok { color:#4ade80; } #log .e-bad { color:#f87171; } #log .e-info { color:#94a3b8; }
  svg text { font-family:inherit; }
</style></head>
<body>
<header>
  <h1>AIC 视觉分拣 · 实时判分</h1>
  <div class="chip ok"><small>正确</small><span id="s-ok">0</span></div>
  <div class="chip err"><small>错误</small><span id="s-err">0</span></div>
  <div class="chip rest"><small>剩余</small><span id="s-rest">12</span></div>
  <div id="cur"></div>
</header>
<main>
  <div class="col" style="flex:1.35">
    <div class="vwrap panel" id="scene_v"><img src="/scene.mjpg" alt="scene">
      <div class="vlabel">全景 · 自由摄像机</div></div>
    <div class="camctl">
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
      <button onclick="cam({dz:0,dx:0,dy:0,home:1})">默认</button>
    </div>
  </div>
  <div class="col" style="flex:1">
    <div class="vwrap panel" id="pip"><img src="/stream.mjpg" alt="end-effector">
      <div class="vlabel">末端相机</div><div id="banner"></div></div>
    <div id="grid"></div>
    <div id="log"></div>
  </div>
  <div class="col" style="flex:1">
    <div class="panel" style="padding:8px"><svg id="map" viewBox="0 0 372 300" width="100%"></svg></div>
    <div style="font-size:12px;color:#64748b;padding:0 4px">
      态势图：圆点=物料（颜色=缺陷类型，暗=待上料），十字=龙门，绿框=置物台，灰框=供料管</div>
  </div>
</main>
<script>
const TN = {1:"划痕",2:"污渍",3:"缺角",4:"孔洞"};
const TC = {1:"#38bdf8",2:"#fb923c",3:"#a78bfa",4:"#34d399"};
const grid = document.getElementById("grid");
for (let i = 1; i <= 12; i++) {
  grid.insertAdjacentHTML("beforeend",
    `<div class="slot pending" id="slot-${i}"><div class="id">${i}</div><div class="gt"></div></div>`);
}
async function cam(body) {
  try { await fetch("/api/cam", {method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify(body)}); } catch (e) {}
}
function drawMap(d) {
  const S = 320, ox = 0.58, oy = 0.42;
  const X = x => (x + ox) * S, Y = y => (y + oy) * S;
  const m = d.map;
  let s = `<rect width="100%" height="100%" fill="#0b1220"/>`;
  // 放置货架：4 类型列 x 3 层（层位沿 +Y 阶梯错开，与 sim_layout.rack 一致）
  for (let lv = 0; lv < m.rows; lv++) {
    const ry = m.rack_y - lv * m.step_y;
    m.col_xs.forEach((zx, i) => {
      s += `<rect x="${X(zx)-19}" y="${Y(ry)-19}" width="38" height="38"
        fill="#1e293b" stroke="#475569" rx="4"/>`;
      if (lv === 0) {
        s += `<text x="${X(zx)}" y="${Y(ry)+4}" fill="#64748b" font-size="11"
          text-anchor="middle">T${i+1}</text>`;
      }
      s += `<text x="${X(zx)+16}" y="${Y(ry)-9}" fill="#334155" font-size="9"
        text-anchor="middle">L${lv+1}</text>`;
    });
  }
  s += `<text x="${X(m.col_xs[0])}" y="${Y(m.rack_y)-30}" fill="#475569" font-size="10"
    text-anchor="middle">放置货架（4 类型列 x 3 层）</text>`;
  // 置物台 / 供料管 / 回收位
  s += `<rect x="${X(m.staging[0])-22}" y="${Y(m.staging[1])-22}" width="44" height="44"
    fill="#132a1e" stroke="#22c55e" rx="4" stroke-dasharray="4 3"/>
    <text x="${X(m.staging[0])}" y="${Y(m.staging[1])+30}" fill="#22c55e" font-size="10"
    text-anchor="middle">置物台</text>`;
  s += `<rect x="${X(m.magazine[0])-20}" y="${Y(m.magazine[1])-20}" width="40" height="40"
    fill="#1e293b" stroke="#64748b" rx="4"/>
    <text x="${X(m.magazine[0])}" y="${Y(m.magazine[1])+30}" fill="#64748b" font-size="10"
    text-anchor="middle">供料管</text>`;
  s += `<circle cx="${X(m.recycle[0])}" cy="${Y(m.recycle[1])}" r="14" fill="none"
    stroke="#a16207" stroke-dasharray="3 3"/>
    <text x="${X(m.recycle[0])}" y="${Y(m.recycle[1])+28}" fill="#a16207" font-size="10"
    text-anchor="middle">回收位</text>`;
  // 物料圆点（待命=供料管处，识别中=置物台，搬运=跟随龙门，已入位=货架列）
  for (const p of d.plates) {
    const pulse = p.status === 'held' ? ` stroke="#fbbf24" stroke-width="3"` : "";
    const dim = p.status === 'magazine' ? ` opacity="0.45"` : "";
    s += `<circle cx="${X(p.x)}" cy="${Y(p.y)}" r="13" fill="${TC[p.type]}"${pulse}${dim}/>
      <text x="${X(p.x)}" y="${Y(p.y)+4}" fill="#0b1220" font-size="11" font-weight="bold"
      text-anchor="middle">${p.id}</text>`;
  }
  // 龙门十字
  const gx = X(d.gantry.x), gy = Y(d.gantry.y);
  s += `<line x1="${gx-16}" y1="${gy}" x2="${gx+16}" y2="${gy}" stroke="#fbbf24" stroke-width="2"/>
    <line x1="${gx}" y1="${gy-16}" x2="${gx}" y2="${gy+16}" stroke="#fbbf24" stroke-width="2"/>
    <circle cx="${gx}" cy="${gy}" r="9" fill="none" stroke="#fbbf24" stroke-width="2"/>`;
  document.getElementById("map").innerHTML = s;
}
async function tick() {
  try {
    const r = await fetch("/api/state"); const d = await r.json();
    document.getElementById("s-ok").textContent = d.score.ok;
    document.getElementById("s-err").textContent = d.score.wrong + d.score.timeout;
    document.getElementById("s-rest").textContent = d.score.pending;
    document.getElementById("cur").textContent =
      d.done ? "本轮完成" : ("巡视中 · 第 " + d.cur_visit + " 位点");
    const vis = d.last_vision ? ("识别结果：" + (TN[d.last_vision] || ("非法值 " + d.last_vision))) : "";
    const banner = document.getElementById("banner");
    banner.style.display = vis ? "block" : "none";
    banner.textContent = vis;
    banner.className = d.last_vision && d.cur_slot &&
      d.slots[d.cur_slot-1] && d.slots[d.cur_slot-1].got == d.last_vision ? "ok" : "vis";
    for (const s of d.slots) {
      const el = document.getElementById("slot-" + s.id);
      el.className = "slot " + s.status;
      el.querySelector(".gt").textContent =
        s.got ? ("判 " + TN[s.got]) : (s.status === "timeout" ? "未识别" : "");
    }
    drawMap(d);
    const log = document.getElementById("log");
    log.innerHTML = d.events.slice().reverse()
      .map(([t, x]) => `<div class="${x.includes("正确（") ? "e-ok" :
        (x.includes("错误") || x.includes("超时") || x.includes("未识别") ? "e-bad" : "e-info")}">[${t}] ${x}</div>`)
      .join("");
  } catch (e) { /* 节点未起时静默重试 */ }
}
setInterval(tick, 400); tick();
</script></body></html>"""


class DashboardHTTP:
    """把 ScoreState 与两路相机帧以 HTTP 暴露出去（只绑 127.0.0.1）。"""

    def __init__(self, state, port, move_cam):
        self.state = state
        self.move_cam = move_cam          # (dict) -> None，自由摄像机控制

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
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if self.path == "/api/reset":
                    self.server.state.reset()
                    self._send(200, b'{"ok":true}', "application/json")
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
        self.httpd.hub_ee = FrameHub()      # 末端相机
        self.httpd.hub_scene = FrameHub()   # 自由摄像机
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True, name="dashboard-http")

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()

    def publish_frame(self, frame, scene=False):
        """相机线程调用：缓存最新帧并唤醒流客户端。"""
        (self.httpd.hub_scene if scene else self.httpd.hub_ee).publish(frame)


class ScoreNode(Node):
    def __init__(self):
        super().__init__("aic_score_node")
        layout_yaml = self.declare_parameter("layout_yaml", "").value
        port = int(self.declare_parameter("http_port", 8080).value)
        self.world = self.declare_parameter("world", "aic_warehouse").value
        self.declare_parameter("task_topic", "/control/material_task")
        self.declare_parameter("arrived_topic", "/sim/gantry_arrived")
        self.declare_parameter("handled_topic", "/sim/handle_done")
        self.declare_parameter("vision_topic", "/vision/material_type")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("scene_image_topic", "/scene/image")
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

        self.create_subscription(MaterialTask, "/control/material_task",
                                 self.state.on_task, 10)
        self.create_subscription(UInt8, "/sim/gantry_arrived", self.state.on_arrived, 10)
        self.create_subscription(UInt8, "/sim/handle_done", self.state.on_done, 10)
        self.create_subscription(String, "/vision/material_type", self.state.on_vision, 10)
        js_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, self.get_parameter(
            "joint_state_topic").value, self.state.on_joint, js_qos)
        img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/camera/image", self._on_image, img_qos)
        self.create_subscription(Image, self.get_parameter(
            "scene_image_topic").value, self._on_scene_image, img_qos)
        # 夹/放信号（控制侧本来就发，桥 ROS->GZ；ROS 侧订阅同样收得到）
        for s in range(1, len(types) + 1):
            self.create_subscription(Empty, "/sim/attach_{:02d}".format(s),
                                     lambda m, s=s: self.state.on_grip(s, True), 10)
            self.create_subscription(Empty, "/sim/detach_{:02d}".format(s),
                                     lambda m, s=s: self.state.on_grip(s, False), 10)

        self.dash = DashboardHTTP(self.state, port, self._move_cam)
        self.dash.start()
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
        self.dash.publish_frame(self._overlay(frame), scene=False)

    def _on_scene_image(self, msg):
        try:
            frame = self._to_bgr(msg)
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[dashboard] 场景图转换失败: %s" % e, throttle_duration_sec=5.0)
            return
        self.dash.publish_frame(frame, scene=True)

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
