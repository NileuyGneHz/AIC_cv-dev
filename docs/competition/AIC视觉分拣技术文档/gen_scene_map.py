# -*- coding: utf-8 -*-
"""M3 场景态势图：按 sim_layout.yaml 真值绘制（置物台 + 供料管 + 4 列 x 3 层货架）。

与 M2 版的区别：M2 是「3x4 库位取料 + 4 个码放格」，M3 是「置物台上料 ->
吸盘取料 -> 放入货架类型列的最低空层」。图上所有坐标都直接读 yaml，不手写。

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
rack, plate, st = layout["rack"], layout["plate"], layout["staging"]
mg, rc = layout["magazine"], layout["recycle"]

TYPE_NAMES = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
TYPE_COLOR = {1: "#1E4FA8", 2: "#3D7BD9", 3: "#D9534F", 4: "#E8A33D"}

cols, rows = int(rack["cols"]), int(rack["rows"])
px, lpz = float(rack["pitch_x"]), float(rack["level_pitch_z"])
y_front, step_y = float(rack["y_front"]), float(rack["level_step_y"])
top_z = float(rack["top_z"])
tile = float(rack["tile_size"])
col_xs = [(c - (cols + 1) / 2.0) * px for c in range(1, cols + 1)]

fig, ax = plt.subplots(figsize=(8.2, 5.6), dpi=200)
ax.set_facecolor("#F7F9FC")

# 龙门行程范围
ax.add_patch(mpatches.Rectangle((-0.47, -0.24), 0.94, 0.56, fill=False,
                                edgecolor="#8B97A8", lw=1.1, ls="--"))


# 4 列 x 3 层货架（层沿 +Y 阶梯错开）
for lv in range(rows):
    y = y_front - lv * step_y
    for ci, x in enumerate(col_xs):
        t = ci + 1
        cw = 0.10   # 画图用格口边长（真实 tile=0.12，与层距 0.11 会贴在一起看不出分层）
        ax.add_patch(mpatches.Rectangle((x - cw / 2, y - cw / 2), cw, cw,
                                        facecolor="#FFFFFF",
                                        edgecolor=TYPE_COLOR[t], lw=1.5,
                                        alpha=0.95 if lv == 0 else 0.75))
        ax.text(x, y, "T%d" % t, ha="center", va="center", fontsize=9,
                color=TYPE_COLOR[t], fontweight="bold", alpha=0.9)
    ax.text(col_xs[-1] + 0.10, y, "第 %d 层\nz=%.2f m" % (lv + 1, top_z + lv * lpz),
            fontsize=10, color="#4A5568", va="center")

ax.text(col_xs[0] - 0.10, y_front + 0.062, "放置货架：4 列（类型）× 3 层，每格一块",
        fontsize=11.5, color="#0E3F8C", fontweight="bold", ha="left")

# 置物台 / 供料管 / 回收位
ax.add_patch(mpatches.Rectangle((st["x"] - 0.08, st["y"] - 0.08), 0.16, 0.16,
                                facecolor="#E8F5EC", edgecolor="#2E7D4F", lw=1.6))
ax.text(st["x"], st["y"] + 0.105, "置物台（上料位）", ha="center", fontsize=11, color="#2E7D4F", fontweight="bold")

ax.add_patch(mpatches.Circle((mg["x"], mg["y"]), float(mg["inner_r"]),
                             facecolor="#DDE3EC", edgecolor="#5A6474", lw=1.4))
ax.text(mg["x"] + 0.055, mg["y"] - 0.055, "供料管\n（12 块叠放）", ha="left", fontsize=10,
        color="#5A6474")

ax.add_patch(mpatches.Circle((rc["x"], rc["y"]), 0.035, facecolor="none",
                             edgecolor="#B26A00", lw=1.4, ls=(0, (3, 3))))
ax.text(rc["x"], rc["y"] - 0.062, "回收位", ha="center", fontsize=10, color="#B26A00")

# 动线：上料 -> 拍照/吸取 -> 放货架
ax.annotate("", xy=(st["x"] - 0.085, st["y"]), xytext=(mg["x"] + 0.05, mg["y"]),
            arrowprops=dict(arrowstyle="->", color="#2E7D4F", lw=1.3,
                            connectionstyle="arc3,rad=-0.18"))
ax.text((st["x"] + mg["x"]) / 2, st["y"] + 0.085, "① 上料（set_pose）", fontsize=10,
        color="#2E7D4F", ha="center")
ax.annotate("", xy=(0.30, 0.05), xytext=(0.30, -0.05),
            arrowprops=dict(arrowstyle="->", color="#1E4FA8", lw=1.3))
ax.text(0.315, 0.0, "② 拍照识别\n③ 吸取", fontsize=10, color="#1E4FA8", va="center")
ax.annotate("", xy=(0.125, 0.26), xytext=(0.26, 0.03),
            arrowprops=dict(arrowstyle="->", color="#1E4FA8", lw=1.3))
ax.text(0.055, 0.205, "④ 放入类型列\n最低空层", fontsize=10, color="#1E4FA8",
        ha="right")

ax.set_xlim(-0.56, 0.56)
ax.set_ylim(-0.27, 0.41)
ax.set_xlabel("世界 X (m)", fontsize=11.5)
ax.set_ylabel("世界 Y (m)", fontsize=11.5)
ax.set_title("M3 场景布局：置物台上料 → 吸盘取料 → 货架 4 列 × 3 层",
             fontsize=15, color="#1A2230", fontweight="bold")
handles = [mpatches.Patch(color=TYPE_COLOR[t], label="类型%d %s" % (t, TYPE_NAMES[t]))
           for t in (1, 2, 3, 4)]
handles.append(mpatches.Patch(facecolor="none", edgecolor="#8B97A8", ls="--",
                              label="三轴行程"))
ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.12),
          fontsize=10.5, frameon=False, ncol=5)
ax.set_aspect("equal")
fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, bbox_inches="tight", facecolor="white")
print("saved", OUT)
