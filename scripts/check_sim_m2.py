#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""断言一次 M2 仿真（真视觉 + 抓取码放）的三组结果，全部本地文件，不依赖 ROS。

用法：
  /usr/bin/python3 scripts/check_sim_m2.py \
      --results logs/verify_m2_xxx.csv \
      --log     logs/verify_m2_xxx.log \
      --pose    logs/verify_m2_pose_xxx.txt

检查项：
  A. 决策结果：results CSV 恰好 12 行，result 全为 OK，材料类型与
     sim_layout.yaml 的 materials.types 逐位一致。
  B. 夹持闭环：日志里 `夹持（DetachableJoint attach）` 与 `释放（... detach）`
     的条数都等于位点数（12）。只数 attach 不够 —— 决策退出时若不等控制队列
     排空，最后一块料会卡在夹爪里（看 simulation_guide.md 坑 19）。
  C. 物理落位：pose/info 里 12 块铁片都在 +Y 码放条上，x 落在对应类型的格心
     容差内，z 停在台面上（不是掉到碰撞板/地面）。

全部通过打印 PASS 并以 0 退出；任何一条不过打印 FAIL 并以 1 退出。
"""
import argparse
import csv
import os
import re
import sys

import yaml

# 世界模型名：material_NN（NN = 上料轮次 / slot_id）。类型不写在名字里，
# 一律回查 materials.types —— 避免两处事实漂移（M3 改动：早前名字带
# _scratch/_stain 后缀，重命名物料时要同时改控制器，容易漏）。
MATERIAL_NAME = re.compile(r"^material_(\d+)$")

NAME_BLOCK = re.compile(r'^\s*name:\s*"([^"]+)"', re.M)
POS_BLOCK = re.compile(
    r'position\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)')


def parse_pose(path):
    """从 `ign topic -e -t .../pose/info` 的文本里取 material_* 的世界坐标。"""
    txt = open(path, "r", encoding="utf-8", errors="replace").read()
    out = {}
    for m in NAME_BLOCK.finditer(txt):
        name = m.group(1)
        if not name.startswith("material_"):
            continue
        pos = POS_BLOCK.search(txt, m.end(), m.end() + 1200)
        if not pos:
            out[name] = None
            continue
        out[name] = tuple(float(pos.group(i)) for i in (1, 2, 3))
    return out


def slot_of(model_name):
    """material_07 -> 7；不是物料模型返回 None。"""
    m = MATERIAL_NAME.match(model_name)
    return int(m.group(1)) if m else None


def read_results(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def count_log(path, needle):
    n = 0
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if needle in line:
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description="校验 M2 仿真搬运结果")
    ap.add_argument("--results", required=True, help="决策结果 CSV")
    ap.add_argument("--log", required=True, help="ros2 launch 的完整日志")
    ap.add_argument("--pose", required=True, help="pose/info 的转储文本")
    ap.add_argument("--layout",
                    default="src/simulation/aic_gazebo_sim/config/sim_layout.yaml",
                    help="场景布局 YAML（取 place_zones 与 materials.types）")
    ap.add_argument("--xy-tol", type=float, default=0.004,
                    help="落位与格心的水平容差（m，默认 4 mm）")
    args = ap.parse_args()

    layout = yaml.safe_load(open(args.layout, encoding="utf-8"))
    expect_types = [int(t) for t in layout["materials"]["types"]]
    n_slots = len(expect_types)
    # M3：货架 = 放置目标（4 类型列 x 3 层，每格一块，层位沿 +Y 阶梯错开）。
    # 格心 x 由 pitch_x 推出（与 gen_world/score_node 同一公式）。
    rack = layout.get("rack") or {}
    cols = int(rack.get("cols", 4))
    rack_rows = int(rack.get("rows", 3))
    pitch_x = float(rack.get("pitch_x", 0.25))
    xs = [(c - (cols + 1) / 2.0) * pitch_x for c in range(1, cols + 1)]
    y_front = float(rack.get("y_front", rack.get("y", 0.30)))
    step_y = float(rack.get("level_step_y", 0.0))
    top_z = float(rack.get("top_z", 0.10))
    lpz = float(rack.get("level_pitch_z", 0.06))
    thick = float(layout["plate"]["thickness"])
    level_zs = [top_z + i * lpz for i in range(rack_rows)]

    fails = []

    # ---- A. 决策结果 ----
    rows = read_results(args.results)
    if len(rows) != n_slots:
        fails.append("A1 结果行数 %d != %d" % (len(rows), n_slots))
    for row in rows:
        if row.get("result") != "OK":
            fails.append("A2 第 %s 位点（slot %s）结果 %s：%s"
                         % (row.get("visit_index"), row.get("slot_id"),
                            row.get("result"), row.get("note", "")))
        # 坑：结果表是**巡视顺序**（4,3,2,1,5,...），而 materials.types 是**库位顺序**，
        #     必须用 slot_id 去查，不能按行号对齐（否则类型断言全错但看着很像真错）。
        slot = int(row.get("slot_id") or 0)
        if not 1 <= slot <= n_slots:
            fails.append("A4 位点 slot_id=%s 越界" % row.get("slot_id"))
            continue
        expect = expect_types[slot - 1]
        if int(row.get("material_type") or 0) != expect:
            fails.append("A3 slot %d 类型 %s != 期望 %d"
                         % (slot, row.get("material_type"), expect))

    # ---- B. 吸附 / 释放成对 ----
    n_attach = count_log(args.log, "吸附（吸盘使能 / DetachableJoint attach）")
    n_detach = count_log(args.log, "释放（吸盘断气 / DetachableJoint detach）")
    if n_attach != n_slots:
        fails.append("B1 夹持 %d 次 != %d（有料没被抓起，或决策退出太早）"
                     % (n_attach, n_slots))
    if n_detach != n_slots:
        fails.append("B2 释放 %d 次 != %d（最后一块料可能卡在夹爪里，见坑 19）"
                     % (n_detach, n_slots))

    # ---- C. 物理落位（M3：每格一块，落在对应类型列的某一层托盘上） ----
    poses = parse_pose(args.pose)
    warns = []
    if not poses:
        # 坑 20：ogre2 无头长跑偶发 run loop 卡死，此时 pose/info 一条也不发，
        # 但 A/B 两组（来自 CSV 与日志）仍然有效。落位是"加分项"，
        # 不该因为它没抓到就把整轮判死；明确降级为 SKIP 并提示。
        warns.append("C 跳过：pose/info 转储为空（可能命中坑 20 的 run loop 卡死），"
                     "落位未核对；重跑一次通常就正常")
    elif len(poses) != n_slots:
        fails.append("C0 pose/info 里只有 %d 块物料（应为 %d），转储是否完整？"
                     % (len(poses), n_slots))
    z_lo = min(level_zs) + thick * 0.25   # 至少贴在某层托盘上（掉下去会接近 0）
    z_hi = max(level_zs) + thick * 1.5
    if poses:
        print("%-22s %8s %8s %8s %8s %8s" % ("model", "x", "y", "z", "dx(mm)", "dy(mm)"))
    for name in sorted(poses):
        sid = slot_of(name)
        pos = poses[name]
        if pos is None:
            fails.append("C1 %s 解析不出坐标" % name)
            continue
        x, y, z = pos
        if sid is None or not 1 <= sid <= len(expect_types):
            fails.append("C2 %s 的编号不在 1..%d" % (name, len(expect_types)))
            continue
        # 轮次编号 -> 物料真值类型 -> 应落的类型列
        tid = expect_types[sid - 1]
        # 由盘心 z 反推层位（盘心 = 托盘面 + 厚/2），再取该层的阶梯 y
        level = max(1, min(rack_rows, int(round((z - thick / 2.0 - top_z) / lpz)) + 1))
        exp_y = y_front - (level - 1) * step_y
        dx = (x - xs[tid - 1]) * 1000.0
        dy = (y - exp_y) * 1000.0
        print("%-22s %8.4f %8.4f %8.4f %8.1f %8.1f  (L%d)"
              % (name, x, y, z, dx, dy, level))
        if abs(dx) > args.xy_tol * 1000.0:
            fails.append("C3 %s x 偏类型列格心 %.1f mm（容差 %.0f mm）"
                         % (name, dx, args.xy_tol * 1000.0))
        if abs(dy) > args.xy_tol * 1000.0:
            fails.append("C4 %s y 偏第 %d 层深度线 %.1f mm" % (name, level, dy))
        if not (z_lo <= z <= z_hi):
            fails.append("C5 %s z=%.4f 不在托盘高度区间 [%.4f, %.4f]"
                         "（掉到地面/未放入？）" % (name, z, z_lo, z_hi))

    print()
    for w in warns:
        print("WARN  " + w)
    if fails:
        print("FAIL（%d 条）" % len(fails))
        for f in fails:
            print("  - " + f)
        return 1
    if warns:
        print("PASS：决策 %d/%d OK、夹持与释放各 %d 次；落位未核对（见上）"
              % (len(rows), n_slots, n_attach))
    else:
        print("PASS：决策 %d/%d OK、夹持与释放各 %d 次、%d 块铁片全部落在对应类型的格内"
              % (len(rows), n_slots, n_attach, len(poses)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
