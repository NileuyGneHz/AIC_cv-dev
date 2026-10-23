# -*- coding: utf-8 -*-
"""M5 场景态势图（双联：俯视 X-Y + 侧视 Y-Z），坐标全部读 sim_layout.yaml / model.sdf。

演进：M2 =「3x4 库位取料 + 4 个码放格」；M3 =「供料管上料」；
M4 =「来料架 3 行 x 4 列开局铺满」；**M5 = 无来料架**（12 块停在底板下方的隐藏托板，
每轮 set_pose 到置物台，等价官方「生成物料」）+ 4 列 x 3 层**垂直立面**货架 +
单立柱悬臂龙门（底部导轨沿 X、横梁沿 Y 指向货架）。

为什么不再画来料架：官方 VirtualNext 平台里没有这个结构，物料是运行时生成到
置物台上的。技术文档若还画着 3x4 来料架，会和仿真/视频对不上。

学术风配色：主蓝 #1E4FA8 / 藏蓝 #0E3F8C / 浅蓝 #E8EFF8 / 文本 #1A2230。
本机可用 CJK 字体：Noto Sans CJK JP（SC 未装）。
"""
import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplcfg")
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import yaml

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = "/home/nileuy/AIC_cv-dev"
OUT = os.path.join(ROOT, "docs/competition/AIC视觉分拣技术文档/assets/scene_map.png")

layout = yaml.safe_load(open(os.path.join(
    ROOT, "src/simulation/aic_gazebo_sim/config/sim_layout.yaml"), encoding="utf-8"))
rack, st, rc = layout["rack"], layout["staging"], layout["recycle"]
plate_d = float(layout["plate"]["diameter"])
pad_z0 = float(layout["gantry"]["pad_z0"])

# ---- 关节行程：直接从 model.sdf 读，避免与控制器/模型脱节 ----
tree = ET.parse(os.path.join(ROOT, "src/simulation/aic_gazebo_sim/models/aic_gantry/model.sdf"))
lim = {}
for j in tree.getroot().iter("joint"):
    nm = j.get("name") or ""
    if nm in ("x_joint", "y_joint", "z_joint"):
        e = j.find("axis/limit")
        lim[nm[0]] = (float(e.find("lower").text), float(e.find("upper").text))
BEAM_Z = 0.545          # 横梁中心高（z_joint=0）
BEAM_HALF = 0.0275
COL_Y = -0.475          # 立柱中心线（model.sdf column 的 pose）
RAIL_Y = (-0.500, -0.450)

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "破损", 4: "孔洞"}
TYPE_COLOR = {1: "#1E4FA8", 2: "#3D7BD9", 3: "#D9534F", 4: "#E8A33D"}

cols, rows = int(rack["cols"]), int(rack["rows"])
px, lpz = float(rack["pitch_x"]), float(rack["level_pitch_z"])
y_front, top_z = float(rack["y_front"]), float(rack["top_z"])
col_xs = [(c - (cols + 1) / 2.0) * px for c in range(1, cols + 1)]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.6, 5.6), dpi=200,
                               gridspec_kw={"width_ratios": [1.12, 1.0]})
for ax in (ax1, ax2):
    ax.set_facecolor("#F7F9FC")

# ============================ 左：俯视（X-Y） ============================
ax1.add_patch(mpatches.Rectangle((-0.70, -0.55), 1.40, 1.10, facecolor="#F2F6FB",
                                 edgecolor="#C9D3E0", lw=1.2))
ax1.text(0.68, -0.44, "整机底板 1.40 × 1.10 m", fontsize=8.5, color="#8B97A8",
         ha="right")

# 底部两条 X 导轨
for y in RAIL_Y:
    ax1.add_patch(mpatches.Rectangle((-0.55, y - 0.015), 1.10, 0.030,
                                     facecolor="#C7D0DB", edgecolor="none"))
ax1.text(0.0, -0.535, "底部导轨 ×2（沿 X）", fontsize=8.5, color="#6B7686",
         ha="center", va="top")

# 横梁（沿 Y 指向货架；画在立柱零位 x=0）
ax1.add_patch(mpatches.Rectangle((-BEAM_HALF, -0.52), 2 * BEAM_HALF, 1.00,
                                 facecolor="#DDE5EF", edgecolor="#8B97A8",
                                 lw=0.9, alpha=0.85))
ax1.text(BEAM_HALF + 0.012, 0.14, "横梁（沿 Y）", fontsize=8.5, color="#6B7686",
         rotation=90, va="center")

# 立柱可走范围 / 滑台可走范围
ax1.add_patch(mpatches.Rectangle((lim["x"][0], lim["y"][0]),
                                 lim["x"][1] - lim["x"][0], lim["y"][1] - lim["y"][0],
                                 fill=False, edgecolor="#8B97A8", lw=1.1, ls="--"))
ax1.plot([lim["x"][0], lim["x"][1]], [COL_Y, COL_Y], color="#1E4FA8", lw=2.4,
         solid_capstyle="butt")

# 分拣货架：立面 4 列 x 3 层（俯视只看得到最前一排格口）
cw = 0.12
for ci, x in enumerate(col_xs):
    t = ci + 1
    ax1.add_patch(mpatches.Rectangle((x - cw / 2, y_front - cw / 2), cw, cw,
                                     facecolor="#FFFFFF", edgecolor=TYPE_COLOR[t], lw=1.5))
    ax1.text(x, y_front, "T%d" % t, ha="center", va="center", fontsize=9,
             color=TYPE_COLOR[t], fontweight="bold")
ax1.text(0.0, y_front + 0.105,
         "分拣货架：4 列 × 3 层（垂直立面，俯视三层重合）",
         ha="center", fontsize=9, color="#4A5568")

# 置物台 / 回收位
ax1.add_patch(mpatches.Rectangle((st["x"] - 0.08, st["y"] - 0.08), 0.16, 0.16,
                                 facecolor="#E8F5EC", edgecolor="#2E7D4F", lw=1.6))
ax1.add_patch(mpatches.Circle((st["x"], st["y"]), plate_d / 2.0, facecolor="#E8A33D",
                              edgecolor="none", alpha=0.95))
ax1.text(st["x"], st["y"] - 0.125, "置物台（识别位）", ha="center", fontsize=9,
         color="#2E7D4F", fontweight="bold")

ax1.add_patch(mpatches.Circle((rc["x"], rc["y"]), 0.032, facecolor="none",
                              edgecolor="#B26A00", lw=1.4, ls=(0, (3, 3))))
ax1.text(rc["x"], rc["y"] - 0.062, "回收位", ha="center", fontsize=8.5, color="#B26A00")

# 动线：生成 -> 置物台 -> 货架
ax1.annotate("", xy=(st["x"] - 0.06, st["y"] - 0.05), xytext=(-0.30, -0.30),
             arrowprops=dict(arrowstyle="->", color="#2E7D4F", lw=1.4,
                             connectionstyle="arc3,rad=-0.18"))
ax1.text(-0.55, -0.335, "① 生成物料\n（停料区 set_pose）", fontsize=8.5,
         color="#2E7D4F", ha="left")

ax1.annotate("", xy=(0.30, y_front - 0.075), xytext=(0.30, st["y"] + 0.09),
             arrowprops=dict(arrowstyle="->", color="#1E4FA8", lw=1.4,
                             connectionstyle="arc3,rad=0.12"))
ax1.text(0.33, 0.10, "② 识别 → 吸取 → 放入类型列", fontsize=9, color="#1E4FA8")

ax1.set_xlim(-0.78, 0.78)
ax1.set_ylim(-0.60, 0.58)
ax1.set_xlabel("世界 X (m)", fontsize=10.5)
ax1.set_ylabel("世界 Y (m)", fontsize=10.5)
ax1.set_title("俯视：底部导轨沿 X、顶部横梁沿 Y 指向货架", fontsize=11.5,
              color="#1A2230", fontweight="bold")
ax1.set_aspect("equal")

# ============================ 右：侧视（Y-Z） ============================
ax2.add_patch(mpatches.Rectangle((-0.55, 0.0), 1.10, 0.02, facecolor="#F2F6FB",
                                 edgecolor="#C9D3E0", lw=1.0))

# 立柱 + 横梁
ax2.add_patch(mpatches.Rectangle((COL_Y - 0.03, 0.02), 0.06, 0.62,
                                 facecolor="#DDE5EF", edgecolor="#8B97A8", lw=0.9))
ax2.add_patch(mpatches.Rectangle((-0.52, BEAM_Z - BEAM_HALF), 1.00, 2 * BEAM_HALF,
                                 facecolor="#DDE5EF", edgecolor="#8B97A8", lw=0.9))
ax2.text(0.50, BEAM_Z + 0.028, "横梁 z=%.3f" % BEAM_Z, fontsize=8.5,
         color="#6B7686", ha="left")

# 置物台
ax2.add_patch(mpatches.Rectangle((st["y"] - 0.08, 0.02), 0.16, st["z"] - 0.02,
                                 facecolor="#E8F5EC", edgecolor="#2E7D4F", lw=1.2))
ax2.plot([st["y"] - 0.08, st["y"] + 0.08], [st["z"], st["z"]], color="#2E7D4F", lw=2.0)
ax2.text(st["y"], st["z"] + 0.018, "置物台 z=%.2f" % st["z"], fontsize=8.5,
         color="#2E7D4F", ha="center")

# 相机（固定式，置物台正上方 z=0.366）
cam_z = 0.366
ax2.plot([st["y"]], [cam_z], marker="v", markersize=6, color="#1E4FA8")
ax2.plot([st["y"], st["y"]], [cam_z - 0.012, st["z"] + 0.02], color="#1E4FA8",
         lw=0.9, ls=(0, (2, 2)))
ax2.text(st["y"] + 0.02, cam_z + 0.015, "固定相机 z=%.3f（横梁之上）" % cam_z,
         fontsize=8.5, color="#1E4FA8")

# 货架：4 列在 X 上重合，侧视只看到 3 层托盘（深 0.12，位于 y_front 附近）
for lv in range(rows):
    z = top_z + lv * lpz
    ax2.add_patch(mpatches.Rectangle((y_front - 0.06, z - 0.008), 0.12, 0.008,
                                     facecolor="#FFFFFF", edgecolor="#4A5568", lw=1.1))
    ax2.text(y_front + 0.075, z, "第 %d 层 z=%.2f" % (lv + 1, z), fontsize=8.5,
             color="#4A5568", va="center")
for y in (y_front - 0.065, y_front + 0.055):
    ax2.plot([y, y], [0.02, top_z + (rows - 1) * lpz + 0.02], color="#8B97A8", lw=1.4)

# 吸盘：GOTO 停靠位（实线）与放置位（虚线）
for y_tool, ls, alpha in ((-0.05, "-", 1.0), (y_front, (0, (3, 3)), 0.75)):
    ax2.plot([y_tool, y_tool], [BEAM_Z - BEAM_HALF, pad_z0], color="#E8A33D",
             lw=2.0, ls=ls, alpha=alpha)
    ax2.plot([y_tool - 0.02, y_tool + 0.02], [pad_z0, pad_z0], color="#E8A33D",
             lw=3.0, ls=ls, alpha=alpha)
ax2.text(-0.05 + 0.03, 0.32, "吸盘（z_joint=0 → 吸口 %.3f）" % pad_z0, fontsize=8.5,
         color="#B26A00")

ax2.set_xlim(-0.62, 0.64)
ax2.set_ylim(0.0, 0.72)
ax2.set_xlabel("世界 Y (m)", fontsize=10.5)
ax2.set_ylabel("世界 Z (m)", fontsize=10.5)
ax2.set_title("侧视：三层货架立面 + 横梁跨过相机上方", fontsize=11.5,
              color="#1A2230", fontweight="bold")
ax2.set_aspect("equal")

# 图例
handles = [mpatches.Patch(color=TYPE_COLOR[t], label="类型%d %s" % (t, TYPE_NAMES[t]))
           for t in (1, 2, 3, 4)]
handles.append(mpatches.Patch(facecolor="none", edgecolor="#8B97A8", ls="--",
                              label="滑台行程 x/y"))
handles.append(mpatches.Patch(color="#1E4FA8", label="立柱行程 x ±0.45 m"))
handles.append(mpatches.Patch(color="#E8A33D", label="吸盘"))
fig.legend(handles=handles, loc="upper center", ncol=7, fontsize=9.5, frameon=False,
           bbox_to_anchor=(0.5, 0.945))
fig.suptitle("M5 场景：生成物料 → 置物台识别 → 单立柱龙门吸取 → 4 列 × 3 层垂直货架",
             fontsize=13.5, color="#1A2230", fontweight="bold", y=0.985)
fig.tight_layout(rect=(0, 0.0, 1, 0.915))
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, bbox_inches="tight", facecolor="white")
print("saved", OUT)
