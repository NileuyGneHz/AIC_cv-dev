#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""逐帧打印缺陷候选的全部归一化特征，用于给四类图案整定阈值。

用法（一般由 probe_patterns.sh 自动调用）：
  /usr/bin/python3 scripts/analyze_patterns.py logs/patterns
"""
import glob
import os
import sys

import cv2
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/vision/aic_vision_trad/scripts"))
from plate_detector import DetectorParams, detect          # noqa: E402
from defect_classifier import DEFAULT_RULES, classify      # noqa: E402

TYPES = [1, 3, 2, 4, 2, 1, 4, 3, 3, 1, 2, 4]


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, "logs/patterns")
    p = DetectorParams()
    print("帧  类型 | 候选特征（按面积降序）")
    hits = 0
    for k in range(1, 13):
        path = os.path.join(d, "m%02d.png" % k)
        if not os.path.exists(path):
            print("m%02d 缺帧" % k)
            continue
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        res = detect(img, p)
        if res.get("plate") is None:
            print("m%02d type=%d  找盘失败" % (k, TYPES[k - 1]))
            continue
        pl = res["plate"]
        tid, why = classify(res["comps"], DEFAULT_RULES)
        ok = "OK " if tid == TYPES[k - 1] else "BAD"
        hits += int(tid == TYPES[k - 1])
        print("m%02d type=%d 判=%d %s r=%.1f level=%.0f  %s" % (
            k, TYPES[k - 1], tid, ok, pl["r"], res["level"], why))
        for c in res["comps"][:4]:
            print("      area=%5d frac=%.4f circ=%.2f elong=%5.1f rad=%.2f "
                  "fill=%.2f sol=%.2f drop=%3.0f enclosed=%.2f" % (
                      c["area"], c["frac"], c["circ"], c["elong"], c["rad"],
                      c["fill"], c["solidity"], c["drop"], c["enclosed"]))
    print("\n命中 %d/12" % hits)


if __name__ == "__main__":
    main()
