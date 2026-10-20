#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""把一轮仿真里存的标注帧拼成「图像集」图版（省赛技术文档要求「图像集展示」）。

用法：
  /usr/bin/python3 src/vision/aic_vision_trad/scripts/make_atlas.py \
      --frames logs/m2_debug --out docs/guides/assets/vision_atlas.png

输入：`run_sim.sh --vision trad --debug-dir logs/m2_debug` 产出的
      `vision_<stamp>_<类型名>.png`（已在图上画过盘圆与类型标签）。
做法：按类型名分组成 4 行，每行最多取 3 张；对每张重新跑一遍检测与判据，
      把用于判类的五个归一化特征打在图上（复算而不是抄日志，图与数才对得上）。

注意：调试图本身带绿色盘圆标注，读成灰度后会变成一圈亮环。实测不影响 Hough
与判据（环在盘缘外侧），但**不要**把它当成"干净样本"去做阈值标定。
"""
import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plate_detector import DetectorParams, detect          # noqa: E402
from defect_classifier import TYPE_NAMES, classify         # noqa: E402

# 类型名 -> 编号（与 defect_classifier.TYPE_NAMES 一致）
NAME_TYPE = {v: k for k, v in TYPE_NAMES.items()}

TILE_W, TILE_H = 380, 285
CAP_H = 70   # 每格下方标注 3 行（类型 + 判据 + 余量），58 会把最后一行切掉
HEAD_H = 40
COLS = 3


def pick_frames(frames_dir, per_type=COLS):
    """{类型编号: [按时间排序的前 per_type 张]}；按文件名里的时间戳升序取。"""
    buckets = {}
    for path in sorted(glob.glob(os.path.join(frames_dir, "vision_*.png"))):
        stem = os.path.basename(path)[:-4]
        suffix = stem.rsplit("_", 1)[-1]
        tid = NAME_TYPE.get(suffix)
        if tid is None:            # vision_fail_* 之类，跳过
            continue
        buckets.setdefault(tid, []).append(path)
    return {t: ps[:per_type] for t, ps in buckets.items()}


def annotate(gray_bgr, res, tid):
    """在彩色帧上画盘圆 + 主缺陷框 + 特征串。"""
    vis = gray_bgr.copy()
    pl = res["plate"]
    cv2.circle(vis, (int(pl["cx"]), int(pl["cy"])), int(pl["r"]), (0, 220, 0), 2)
    cv2.drawMarker(vis, (int(pl["cx"]), int(pl["cy"])), (0, 220, 0),
                   cv2.MARKER_CROSS, 10, 1)
    # 只框面积最大的那个候选：判据就是从它开始按顺序匹配的
    if res["comps"]:
        c = max(res["comps"], key=lambda d: d["area"])
        x, y, w, h = c["bbox"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 165, 255), 2)
    return vis


def feature_text(res):
    """判据用的五个归一化量，拆成两行（一行太长会压到相邻格）。"""
    if not res["comps"]:
        return ("no defect found", "")
    c = max(res["comps"], key=lambda d: d["area"])
    return ("frac %.3f   circ %.2f   rad %.2f" % (c["frac"], c["circ"], c["rad"]),
            "elong %.1f   drop %.0f" % (c["elong"], c["drop"]))


def build(frames_dir, out_path, per_type=COLS, feat_csv=""):
    frames = pick_frames(frames_dir, per_type)
    if not frames:
        print("在 %s 下没有找到 vision_*.png，先跑 "
              "./scripts/run_sim.sh --vision trad --debug-dir %s" % (frames_dir, frames_dir))
        return 1

    n_rows = len(frames)
    width = TILE_W * per_type
    height = HEAD_H + (TILE_H + CAP_H) * n_rows + 14   # 底部留白，最后一行标注不贴边
    canvas = np.full((height, width, 3), 28, np.uint8)     # 深色底，与 IDE 深色主题一致

    cv2.putText(canvas, "AIC defect dataset  |  4 classes x %d samples  |  sim camera 640x480"
                % per_type, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1)

    ok, bad = 0, 0
    rows = []
    for row, tid in enumerate(sorted(frames)):
        y0 = HEAD_H + row * (TILE_H + CAP_H)
        for col, path in enumerate(frames[tid]):
            x0 = col * TILE_W
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            # 调试图左上角烧了「类型=编号」的标签，与本图版下方的说明重复；
            # 裁掉顶部 28 px 再检测，免得两套标注打架（说明文字统一由本脚本画）。
            if img.shape[0] > 160:
                img = img[28:, :]
            res = detect(img, DetectorParams())
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            rec = {"frame": os.path.basename(path), "gt": tid, "pred": "", "r": "",
                   "refined": "", "frac": "", "circ": "", "rad": "", "elong": "", "drop": ""}
            if res is not None:
                vis = annotate(vis, res, tid)
                pred, why = classify(res["comps"])   # (type_id, 判据理由)
                line1, line2 = feature_text(res)
                tint = (0, 220, 0) if pred == tid else (0, 0, 255)
                ok += pred == tid
                bad += pred != tid
                rec.update({"pred": pred, "r": round(res["plate"]["r"], 2),
                            "refined": int(bool(res["plate"].get("refined")))})
                if res["comps"]:
                    c = max(res["comps"], key=lambda d: d["area"])
                    rec.update({"frac": round(c["frac"], 4), "circ": round(c["circ"], 3),
                                "rad": round(c["rad"], 3), "elong": round(c["elong"], 2),
                                "drop": round(c["drop"], 1)})
            else:
                pred, why = None, "plate not found"
                line1, line2 = why, ""
                tint = (0, 0, 255)
                bad += 1
            rows.append(rec)
            vis = cv2.resize(vis, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
            canvas[y0:y0 + TILE_H, x0:x0 + TILE_W] = vis
            # 分隔线 + 三行说明（英文，cv2 不支持中文字形，中文见文档表格）
            cv2.line(canvas, (x0, y0 + TILE_H), (x0 + TILE_W, y0 + TILE_H), (70, 70, 70), 1)
            head = "%d %s" % (tid, TYPE_NAMES[tid])
            if pred is not None and pred != tid:
                head += "   -> misread as %s" % TYPE_NAMES[pred]
            cv2.putText(canvas, head, (x0 + 6, y0 + TILE_H + 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, tint, 1)
            cv2.putText(canvas, line1, (x0 + 6, y0 + TILE_H + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (205, 205, 205), 1)
            cv2.putText(canvas, line2, (x0 + 6, y0 + TILE_H + 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (205, 205, 205), 1)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cv2.imwrite(out_path, canvas)
    if feat_csv:
        os.makedirs(os.path.dirname(feat_csv) or ".", exist_ok=True)
        with open(feat_csv, "w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["frame", "gt", "pred", "r", "refined",
                                              "frac", "circ", "rad", "elong", "drop"])
            w.writeheader()
            w.writerows(rows)
        print("特征表：%s（%d 行）" % (feat_csv, len(rows)))
    print("图版：%s（%d 张，命中 %d，误判 %d）" % (out_path, ok + bad, ok, bad))
    return 0 if bad == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="把标注帧拼成缺陷图像集图版")
    ap.add_argument("--frames", default="logs/m2_debug", help="标注帧目录")
    ap.add_argument("--out", default="docs/guides/assets/vision_atlas.png")
    ap.add_argument("--per-type", type=int, default=COLS, help="每类取几张（默认 3）")
    ap.add_argument("--features-csv", default="",
                    help="把每张图的判据特征写成 CSV（用于统计各类分布/余量）")
    args = ap.parse_args()
    return build(args.frames, args.out, args.per_type, args.features_csv)


if __name__ == "__main__":
    sys.exit(main())
