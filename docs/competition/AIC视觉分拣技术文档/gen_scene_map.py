# -*- coding: utf-8 -*-
"""场景态势图：按 sim_layout.yaml 真实数据绘制（3x4 库位 + 4 码放格 + 龙门行程范围）。
学术风配色：主蓝 #1E4FA8 / 藏蓝 #0E3F8C / 浅蓝 #E8EFF8 / 文本 #1A2230。
本机可用 CJK 字体：Noto Sans CJK JP（SC 未装）。"""
import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplcfg")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import yaml

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = "/home/nileuy/AIC_cv-dev"
layout = yaml.safe_load(open(os.path.join(
    ROOT, "src/simulation/aic_gazebo_sim/config/sim_layout.yaml")))
rack = layout["rack"]
types = layout["materials"]["types"]
pz = layout["place_zones"]
TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
TYPE_COLOR = {1: "#1E4FA8", 2: "#3D7BD9", 3: "#D9534F", 4: "#FFC107"}

fig, ax = plt.subplots(figsize=(9.6, 6.6), dpi=160)
ax.set_facecolor("#F7F9FC")

# 龙门行程范围（x/y 关节限位外推）
ax.add_patch(mpatches.Rectangle((-0.55, -0.34), 1.10, 0.71, fill=False,
                                edgecolor="#8B97A8", lw=1.2, ls="--"))
ax.text(-0.54, 0.375, "龙门行程范围（x/y 关节）", fontsize=9, color="#4A5568")

# 3x4 库位：row 1 在 -Y 前排（真实世界），画在负 Y 区
cols, rows = rack["cols"], rack["rows"]
px, py = rack["pitch_x"], rack["pitch_y"]
row_dy = 0.12               # 画图用纵向间距（拉开，给编号留位）
for r in range(rows):
    for c in range(cols):
        slot = r * cols + c + 1
        t = types[slot - 1]
        x = (c - (cols - 1) / 2) * px
        y = -0.05 - r * row_dy          # row1 最近 -Y
        ax.add_patch(mpatches.Rectangle((x - 0.04, y - 0.04), 0.08, 0.08,
                                        facecolor="#FFFFFF",
                                        edgecolor="#1E4FA8", lw=1.4))
        ax.add_patch(mpatches.Circle((x, y), 0.026, facecolor=TYPE_COLOR[t],
                                     edgecolor="none", alpha=0.9))
        ax.text(x, y - 0.058, "S%02d" % slot, ha="center", fontsize=7.5,
                color="#1A2230")
        ax.text(x, y, str(t), ha="center", va="center", fontsize=9,
                color="#FFFFFF", fontweight="bold")

# 4 码放格（+Y 侧，y=0.30）
for i, xz in enumerate(pz["xs"]):
    t = i + 1
    ax.add_patch(mpatches.Rectangle((xz - 0.065, pz["y"] - 0.045), 0.13, 0.09,
                                    facecolor="#E8EFF8",
                                    edgecolor="#0E3F8C", lw=1.6))
    ax.text(xz, pz["y"], "类型%d %s" % (t, TYPE_NAMES[t]), ha="center",
            va="center", fontsize=9, color="#0E3F8C", fontweight="bold")
ax.annotate("码放条（+Y 侧）", xy=(-0.40, pz["y"] + 0.062),
            fontsize=9, color="#0E3F8C", ha="left")
ax.annotate("", xy=(-0.50, pz["y"]), xytext=(-0.50, -0.10),
            arrowprops=dict(arrowstyle="->", color="#3D7BD9", lw=1.4))
ax.text(-0.515, 0.10, "抓取→搬运→码放", rotation=90, fontsize=8.5,
        color="#3D7BD9", va="center")

ax.text(-0.54, -0.385, "库位行 1 在 -Y 前排（巡视起点侧）；圆点颜色 = 缺陷类型，数字 = 类型真值",
        fontsize=9, color="#4A5568")

ax.set_xlim(-0.62, 0.62)
ax.set_ylim(-0.42, 0.43)
ax.set_xlabel("世界 X (m)", fontsize=10)
ax.set_ylabel("世界 Y (m)", fontsize=10)
ax.set_title("仓储场景布局：12 库位 + 4 类型码放格（按 sim_layout.yaml 真值绘制）",
             fontsize=12, color="#1A2230", fontweight="bold")
handles = [mpatches.Patch(color=TYPE_COLOR[t],
                          label="类型%d %s" % (t, TYPE_NAMES[t]))
           for t in (1, 2, 3, 4)]
ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.005, 0.5),
          fontsize=9, framealpha=0.9)
ax.set_aspect("equal")
fig.tight_layout()
out = os.path.join(ROOT, "docs/competition/AIC视觉分拣技术文档/assets/scene_map.png")
fig.savefig(out, bbox_inches="tight", facecolor="white")
print("saved:", out)
