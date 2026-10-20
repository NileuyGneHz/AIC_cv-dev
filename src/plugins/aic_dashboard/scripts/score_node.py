#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时判分可视化节点：正确绿 / 错误红 + 计分板 + 相机流，浏览器单窗口可录。

对标官方平台的过审硬指标 3、4（docs/competition/platform_and_video_checklist.md）：
  3. 实时判分可视化 —— 放置正确变绿 / 错误变红，自动计分；
  4. 单窗口合成画面 —— 仿真相机流 + 库位状态 + 得分在一个页面里，OBS 直接录浏览器。

判分口径（与 4 列码放布局等价）：
  「物料被放到正确列」⟺「视觉判出的类型 == 该库位的真值类型」。
  真值来自 sim_layout.yaml 的 materials.types（与视觉真值同源，不引入第二份事实）。

判定事件链（每个位点）：
  GOTO(visit) ──► /sim/gantry_arrived ──► /vision/material_type
              ──► HANDLE(带判型) ──► /sim/handle_done ──► 落判（绿/红）

  超时兜底：决策 TIMEOUT 的位点不会有 HANDLE/handle_done，
  靠「下一条 GOTO(visit N+1) 到达时上一 visit 还没落判」推断为 TIMEOUT。
  （对应 launch 里 vision_timeout_s 的语义，见 simulation_guide.md 坑 12。）

HTTP 接口（默认 127.0.0.1:8080，只绑本机）：
  GET  /            仪表盘页面（暗色，适合录屏）
  GET  /api/state   JSON 状态（前端 0.5 s 轮询）
  POST /api/reset   清空本轮判分（多轮连跑时用）
  GET  /stream.mjpg  相机 MJPEG 流（叠加当前位点 / 识别 / 比分）

只读订阅，不发布任何话题 —— 判分器挂了不影响链路本身。
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt8

from aic_msgs.msg import MaterialTask

import yaml

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
TYPE_NAMES_EN = {1: "SCRATCH", 2: "STAIN", 3: "CHIP", 4: "HOLE"}
STATUS_LABEL = {
    "pending": "待巡视", "visiting": "识别中", "handling": "搬运中",
    "ok": "正确", "wrong": "错误", "timeout": "超时",
}


class ScoreState:
    """判分状态（线程安全：ROS 回调线程写，HTTP 线程读）。"""

    def __init__(self, types, rows, cols):
        self.types = list(types)                  # slot_id -> 真值类型
        self.rows, self.cols = rows, cols
        self.lock = threading.Lock()
        self.slots = {}                           # slot -> dict(status, got, visit, t)
        for s in range(1, len(types) + 1):
            self.slots[s] = {"status": "pending", "got": None,
                             "visit": None, "t": None}
        self.task_type = {}                       # HANDLE 已知的判型（slot -> type）
        self.events = []                          # [(time_str, text)]
        self.cur_visit = 0
        self.cur_slot = None
        self.last_vision = None                   # 最近一条识别结果（显示用）

    # ---------- ROS 回调侧 ----------
    def on_task(self, msg):
        with self.lock:
            if msg.action == "GOTO":
                slot = int(msg.slot_id)
                self.cur_visit, self.cur_slot = int(msg.visit_index), slot
                # 超时兜底（不依赖 visit_index 的编号：决策对 GOTO/HANDLE 都递增 visit，
                # 编号相邻推不出位点相邻）。语义推断：只可能有一个「识别中」的位点，
                # 决策已经开始巡视下一个库位 = 上一库位视觉没回结果 = TIMEOUT。
                for s, st in self.slots.items():
                    if s != slot and st["status"] == "visiting":
                        self._judge_locked(s, None)
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
         height:100vh; display:flex; flex-direction:column; padding:14px 18px; gap:12px; }
  header { display:flex; align-items:center; gap:26px; }
  h1 { font-size:22px; font-weight:700; letter-spacing:1px; }
  .chip { padding:8px 20px; border-radius:10px; font-size:22px; font-weight:700; }
  .chip small { font-size:13px; font-weight:400; opacity:.85; margin-right:8px; }
  .ok   { background:#14532d; color:#4ade80; border:1px solid #22c55e; }
  .err  { background:#7f1d1d; color:#f87171; border:1px solid #ef4444; }
  .rest { background:#1e293b; color:#94a3b8; border:1px solid #475569; }
  #cur  { font-size:16px; color:#fbbf24; }
  main { flex:1; display:flex; gap:16px; min-height:0; }
  #video { flex:1.4; background:#000; border-radius:12px; overflow:hidden; position:relative; }
  #video img { width:100%; height:100%; object-fit:contain; }
  #banner { position:absolute; left:0; right:0; bottom:0; padding:10px 16px; font-size:18px;
            background:rgba(15,23,42,.78); display:none; }
  #banner.ok    { color:#4ade80; } #banner.wrong { color:#f87171; } #banner.vis { color:#fbbf24; }
  #right { flex:1; display:flex; flex-direction:column; gap:12px; min-width:0; }
  #grid { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; }
  .slot { border-radius:12px; padding:10px 8px; text-align:center; background:#1e293b;
          border:2px solid #334155; transition:all .25s; }
  .slot .id { font-size:26px; font-weight:800; }
  .slot .tt { font-size:12px; color:#94a3b8; margin-top:4px; }
  .slot .gt { font-size:13px; margin-top:2px; min-height:16px; }
  .slot.pending  { opacity:.45; }
  .slot.visiting, .slot.handling { border-color:#fbbf24; background:#3a2e10; }
  .slot.ok       { border-color:#22c55e; background:#14532d; }
  .slot.wrong, .slot.timeout { border-color:#ef4444; background:#7f1d1d; }
  #log { flex:1; overflow-y:auto; background:#020617; border:1px solid #1e293b; border-radius:12px;
         padding:10px 14px; font-size:14px; line-height:1.7; }
  #log .e-ok { color:#4ade80; } #log .e-bad { color:#f87171; } #log .e-info { color:#94a3b8; }
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
  <div id="video"><img src="/stream.mjpg" alt="camera"><div id="banner"></div></div>
  <div id="right">
    <div id="grid"></div>
    <div id="log"></div>
  </div>
</main>
<script>
const TN = {1:"划痕",2:"污渍",3:"缺角",4:"孔洞"};
const grid = document.getElementById("grid");
for (let i = 1; i <= 12; i++) {
  grid.insertAdjacentHTML("beforeend",
    `<div class="slot pending" id="slot-${i}"><div class="id">${i}</div>
     <div class="tt"></div><div class="gt"></div></div>`);
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
      el.querySelector(".tt").textContent = "真值 " + TN[s.true_type];
      el.querySelector(".gt").textContent =
        s.got ? ("判 " + TN[s.got]) : (s.status === "timeout" ? "未识别" : "");
    }
    const log = document.getElementById("log");
    log.innerHTML = d.events.slice().reverse()
      .map(([t, x]) => `<div class="${x.includes("正确（") ? "e-ok" :
        (x.includes("错误") || x.includes("超时") || x.includes("未识别") ? "e-bad" : "e-info")}">[${t}] ${x}</div>`)
      .join("");
  } catch (e) { /* 节点未起时静默重试 */ }
}
setInterval(tick, 500); tick();
</script></body></html>"""


class DashboardHTTP:
    """把 ScoreState 与相机帧以 HTTP 暴露出去（只绑 127.0.0.1）。"""

    def __init__(self, state, port):
        self.state = state

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
                    self._stream()
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if self.path == "/api/reset":
                    self.server.state.reset()
                    self._send(200, b'{"ok":true}', "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last_ts = 0.0
                while True:
                    with self.server.frame_cond:
                        if self.server.frame_ts == last_ts:
                            self.server.frame_cond.wait(timeout=1.0)
                            if self.server.frame_ts == last_ts:
                                continue
                        frame = self.server.frame
                        last_ts = self.server.frame_ts
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
        self.httpd.frame = None
        self.httpd.frame_ts = 0.0
        self.httpd.frame_cond = threading.Condition()
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True, name="dashboard-http")

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()

    def publish_frame(self, frame):
        """相机线程调用：缓存最新帧并唤醒流客户端。"""
        cond = self.httpd.frame_cond
        with cond:
            self.httpd.frame = frame
            self.httpd.frame_ts = time.monotonic()
            cond.notify_all()


class ScoreNode(Node):
    def __init__(self):
        super().__init__("aic_score_node")
        layout_yaml = self.declare_parameter("layout_yaml", "").value
        port = int(self.declare_parameter("http_port", 8080).value)
        self.declare_parameter("task_topic", "/control/material_task")
        self.declare_parameter("arrived_topic", "/sim/gantry_arrived")
        self.declare_parameter("handled_topic", "/sim/handle_done")
        self.declare_parameter("vision_topic", "/vision/material_type")
        self.declare_parameter("image_topic", "/camera/image")

        types, rows, cols = self._load_truth(layout_yaml)
        self.state = ScoreState(types, rows, cols)

        self.create_subscription(MaterialTask, "/control/material_task",
                                 self.state.on_task, 10)
        self.create_subscription(UInt8, "/sim/gantry_arrived", self.state.on_arrived, 10)
        self.create_subscription(UInt8, "/sim/handle_done", self.state.on_done, 10)
        self.create_subscription(String, "/vision/material_type", self.state.on_vision, 10)
        img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/camera/image", self._on_image, img_qos)

        self.dash = DashboardHTTP(self.state, port)
        self.dash.start()
        self.get_logger().info(
            "[dashboard] 判分看板：http://127.0.0.1:%d  （真值 %d 库位：%s）"
            % (port, len(types), "".join(str(t) for t in types)))

    # ---------- 真值 ----------
    def _load_truth(self, layout_yaml):
        """从 sim_layout.yaml 拿 materials.types（与视觉真值同一份事实）。"""
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
            doc = yaml.safe_load(f)
        types = [int(t) for t in doc.get("materials", {}).get("types", [])]
        if not types:
            raise RuntimeError("sim_layout.yaml 里没有 materials.types，无法判分")
        rack = doc.get("rack", {})
        return types, int(rack.get("rows", 3)), int(rack.get("cols", 4))

    # ---------- 相机 ----------
    def _on_image(self, msg):
        try:
            if msg.encoding == "mono8":
                frame = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width), cv2.COLOR_GRAY2BGR)
            elif msg.encoding == "bgr8":
                frame = np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width, 3).copy()
            else:  # rgb8 及其余按 rgb 处理
                frame = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn("[dashboard] 图像转换失败: %s" % e, throttle_duration_sec=5.0)
            return
        self.dash.publish_frame(self._overlay(frame))

    def _overlay(self, frame):
        """在相机帧上叠加当前位点 / 识别 / 比分（cv2 字体不支持中文，用英文）。"""
        snap = self.state.snapshot()
        ok, err = snap["score"]["ok"], snap["score"]["wrong"] + snap["score"]["timeout"]
        vision = snap["last_vision"] or "-"
        vis_name = TYPE_NAMES_EN.get(int(vision), vision) if str(vision).isdigit() else vision
        slot = snap["cur_slot"]
        true_name = TYPE_NAMES_EN.get(snap["slots"][slot - 1]["true_type"], "?") if slot else "-"
        line1 = "VISIT %02d  SLOT %s   VISION: %s   TRUE: %s" % (
            snap["cur_visit"], ("%02d" % slot) if slot else "--", vis_name, true_name)
        line2 = "SCORE  OK %d / ERR %d / REST %d" % (ok, err, snap["score"]["pending"])
        bar_h = 64
        cv2.rectangle(frame, (0, frame.shape[0] - bar_h), (frame.shape[1], frame.shape[0]),
                      (15, 23, 42), -1)
        cv2.putText(frame, line1, (12, frame.shape[0] - 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (226, 232, 240), 2)
        cv2.putText(frame, line2, (12, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (74, 222, 128) if err == 0 else (248, 113, 113), 2)
        return frame


def main():
    rclpy.init()
    node = ScoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dash.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
