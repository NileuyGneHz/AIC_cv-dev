"""离线回归：对一批带标注的图跑分类器，打印逐图特征、混淆矩阵与准确率。

用法（在包目录下）:
    /usr/bin/python3 scripts/bench_offline.py \
        --dataset ~/AIC_cv-dev/logs/vision_ds \
        --layout  ../../simulation/aic_gazebo_sim/config/sim_layout.yaml

目录约定二选一（自动识别）：
  1) slot_NN/*.png        —— 编号按 --layout 的 materials.types 取真值（仿真数据集）
  2) <label>/*.png        —— label ∈ {scratch,stain,chip,hole}（实拍数据集）

退出码：0 = 准确率 >= --min-acc；1 = 不达标（可直接塞进 CI）。
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plate_detector import DetectorParams, detect          # noqa: E402
from defect_classifier import TYPE_NAMES, classify, TYPE_SCRATCH  # noqa: E402

NAME_TO_ID = {v: k for k, v in TYPE_NAMES.items()}


def load_types(layout_path):
    if not layout_path:
        return None
    import yaml
    with open(layout_path) as f:
        return yaml.safe_load(f)["materials"]["types"]


def collect(dataset, types):
    """返回 [(path, gt_id)]。"""
    items = []
    for d in sorted(glob.glob(os.path.join(dataset, "*"))):
        if not os.path.isdir(d):
            continue
        base = os.path.basename(d)
        if base.startswith("slot_") and types:
            try:
                gt = int(types[int(base.split("_")[1]) - 1])
            except (IndexError, ValueError):
                continue
        elif base in NAME_TO_ID:
            gt = NAME_TO_ID[base]
        else:
            continue
        for f in sorted(glob.glob(os.path.join(d, "*.png"))):
            items.append((f, gt))
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=os.path.expanduser("~/AIC_cv-dev/logs/vision_ds"))
    ap.add_argument("--layout", default="")
    ap.add_argument("--min-acc", type=float, default=0.95)
    ap.add_argument("--dump", action="store_true", help="打印每张图的前 4 个候选特征")
    args = ap.parse_args()

    items = collect(args.dataset, load_types(args.layout))
    if not items:
        print("数据集为空或有误: {}".format(args.dataset))
        return 1

    p = DetectorParams()
    conf = np.zeros((5, 5), dtype=int)   # 0 表示未识别
    ok = 0
    rows = []
    for path, gt in items:
        gray = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
        res = detect(gray, p)
        if res is None:
            pred, why = 0, "定位失败"
        else:
            pred, why = classify(res["comps"])
        ok += int(pred == gt)
        conf[gt, pred] += 1
        rows.append((path, gt, pred, why, res))

    if args.dump:
        for path, gt, pred, why, res in rows:
            tag = "OK" if pred == gt else "XX"
            extra = ""
            if res:
                extra = "r=%.1f c=(%.0f,%.0f) lvl=%.0f" % (
                    res["plate"]["r"], res["plate"]["cx"], res["plate"]["cy"], res["level"])
            print("%s %-8s gt=%-7s pred=%-7s %s  <- %s" % (
                tag, os.path.basename(os.path.dirname(path)), TYPE_NAMES[gt],
                TYPE_NAMES.get(pred, "none"), extra, why))
            if res:
                for c in res["comps"][:4]:
                    print("        a=%4d frac=%.3f el=%.1f cir=%.2f rad=%.2f mean=%3.0f" % (
                        c["area"], c["frac"], c["elong"], c["circ"], c["rad"], c["mean"]))

    labels = [TYPE_NAMES[i] for i in range(1, 5)]
    print("\n混淆矩阵（行=真值，列=预测，末列为未识别）")
    print("%-9s" % "" + "".join("%9s" % s for s in labels) + "%9s" % "none")
    for i in range(1, 5):
        print("%-9s" % labels[i - 1] + "".join("%9d" % conf[i, j] for j in range(1, 5))
              + "%9d" % conf[i, 0])

    acc = ok / float(len(rows))
    print("\n准确率 %d/%d = %.1f%%（阈值 %.0f%%）" % (ok, len(rows), acc * 100, args.min_acc * 100))
    return 0 if acc >= args.min_acc else 1


if __name__ == "__main__":
    sys.exit(main())
