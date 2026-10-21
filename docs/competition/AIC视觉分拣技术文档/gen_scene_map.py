# -*- coding: utf-8 -*-
"""M4 场景态势图：按 sim_layout.yaml 真值绘制（来料架 3x4 + 置物台 + 4 列 x 3 层货架）。

演进：M2 =「3x4 库位取料 + 4 个码放格」；M3 =「供料管上料」；
M4 =「来料架 3 行 x 4 列开局铺满 12 块 → 逐块传送到置物台 → 识别后放入类型列」。
图上所有坐标都直接读 yaml，不手写。

学术风配色：主蓝 #1E4FA8 / 藏蓝 #0E3F8C / 浅蓝 #E8EFF8 / 文本 #1A2230。
本机可用 CJK 字体：Noto Sans CJK JP（SC 未装）。
"""
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
OUT = os.path.join(ROOT, "docs/competition/AIC视觉分拣技术文档/assets/scene_map.png")

layout = yaml.safe_load(open(os.path.join(
    ROOT, "src/simulation/aic_gazebo_sim/config/sim_layout.yaml"), encoding="utf-8"))
rack, st = layout["rack"], layout["staging"]
inf, rc = layout["infeed"], layout["recycle"]

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
TYPE_COLOR = {1: "#1E4FA8", 2: "#3D7BD9", 3: "#D9534F", 4: "#E8A33D"}

cols, rows = int(rack["cols"]), int(rack["rows"])
px, lpz = float(rack["pitch_x"]), float(rack["level_pitch_z"])
y_front, step_y = float(rack["y_front"]), float(rack["level_step_y"])
top_z = float(rack["top_z"])
col_xs = [(c - (cols + 1) / 2.0) * px for c in range(1, cols + 1)]

icols, irows = int(inf["cols"]), int(inf["rows"])
ipx, ipy = float(inf["pitch_x"]), float(inf["pitch_y"])
iy0 = float(inf["y0"])
itypes = [int(t) for t in layout["materials"]["types"]]

fig, ax = plt.subplots(figsize=(8.2, 5.8), dpi=200)
ax.set_facecolor("#F7F9FC")

# 龙门行程范围（与 controller 的 LIMITS 同口径）
ax.add_patch(mpatches.Rectangle((-0.47, -0.50), 0.94, 0.88, fill=False,
                                edgecolor="#8B97A8", lw=1.1, ls="--"))

# ---- 来料架 3 行 x 4 列（-Y 侧，开局 12 块按格铺放）----
for r in range(irows):
    y = iy0 + r * ipy
    for c in range(icols):
        x = (c + 1 - (icols + 1) / 2.0) * ipx
        ax.add_patch(mpatches.Rectangle((x - 0.052, y - 0.032), 0.104, 0.064,
                                        facecolor="#FFFFFF", edgecolor="#8B97A8", lw=1.0))
        idx = r * icols + c
        if idx < len(itypes):
            ax.add_patch(mpatches.Circle((x, y), 0.027,
                                         facecolor=TYPE_COLOR[itypes[idx]],
                                         edgecolor="none", alpha=0.9))
ax.text(0.0, -0.545,
        "来料架：3 行 × 4 列，开局 12 块（颜色 = 缺陷类型，顺序打乱）",
        fontsize=10.5, color="#5A6474", ha="center")

# ---- 分拣货架 4 列 x 3 层（+Y 侧，层沿 +Y 阶梯错开）----
for lv in range(rows):
    y = y_front - lv * step_y
    for ci, x in enumerate(col_xs):
        t = ci + 1
        cw = 0.10
        ax.add_patch(mpatches.Rectangle((x - cw / 2, y - cw / 2), cw, cw,
                                        facecolor="#FFFFFF",
                                        edgecolor=TYPE_COLOR[t], lw=1.5,
                                        alpha=0.95 if lv == 0 else 0.75))
        ax.text(x, y, "T%d" % t, ha="center", va="center", fontsize=9,
                color=TYPE_COLOR[t], fontweight="bold", alpha=0.9)
    ax.text(col_xs[-1] + 0.085, y, "第 %d 层\nz=%.2f m" % (lv + 1, top_z + lv * lpz),
            fontsize=9.5, color="#4A5568", va="center")


# ---- 置物台 / 回收位 ----
ax.add_patch(mpatches.Rectangle((st["x"] - 0.08, st["y"] - 0.08), 0.16, 0.16,
                                facecolor="#E8F5EC", edgecolor="#2E7D4F", lw=1.6))
ax.text(st["x"], st["y"] + 0.105, "置物台（识别位）", ha="center", fontsize=11,
        color="#2E7D4F", fontweight="bold")
ax.add_patch(mpatches.Circle((st["x"], st["y"]), 0.027, facecolor="#E8A33D",
                             edgecolor="none", alpha=0.9))

ax.add_patch(mpatches.Circle((rc["x"], rc["y"]), 0.032, facecolor="none",
                             edgecolor="#B26A00", lw=1.4, ls=(0, (3, 3))))
ax.text(rc["x"], rc["y"] - 0.062, "回收位", ha="center", fontsize=9.5,
        color="#B26A00")

# ---- 动线 ----
ax.annotate("", xy=(st["x"] - 0.085, st["y"] - 0.02), xytext=(-0.30, -0.215),
            arrowprops=dict(arrowstyle="->", color="#2E7D4F", lw=1.3,
                            connectionstyle="arc3,rad=-0.18"))
ax.text(-0.52, -0.125, "① 上料\n（set_pose）", fontsize=9.5, color="#2E7D4F", ha="left")

ax.annotate("", xy=(st["x"], st["y"] + 0.085), xytext=(st["x"], st["y"] + 0.015),
            arrowprops=dict(arrowstyle="->", color="#1E4FA8", lw=1.3))
ax.text(st["x"] + 0.018, st["y"] + 0.05, "② 拍照识别", fontsize=10, color="#1E4FA8",
        va="center")

ax.annotate("", xy=(0.125, 0.20), xytext=(0.24, -0.05),
            arrowprops=dict(arrowstyle="->", color="#1E4FA8", lw=1.3,
                            connectionstyle="arc3,rad=-0.15"))
ax.text(0.10, 0.012, "③ 吸取 → 放入类型列", fontsize=9.5, color="#1E4FA8",
        ha="right")

ax.set_xlim(-0.56, 0.56)
ax.set_ylim(-0.60, 0.42)
ax.set_xlabel("世界 X (m)", fontsize=11.5)
ax.set_ylabel("世界 Y (m)", fontsize=11.5)
ax.set_title("M4 场景布局：来料架 3×4 → 置物台识别 → 分拣货架 4 列 × 3 层",
             fontsize=14.5, color="#1A2230", fontweight="bold")
handles = [mpatches.Patch(color=TYPE_COLOR[t], label="类型%d %s" % (t, TYPE_NAMES[t]))
           for t in (1, 2, 3, 4)]
handles.append(mpatches.Patch(facecolor="none", edgecolor="#8B97A8", ls="--",
                              label="三轴行程"))
ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.11),
          fontsize=10.5, frameon=False, ncol=5)
ax.set_aspect("equal")
fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, bbox_inches="tight", facecolor="white")
print("saved", OUT)
