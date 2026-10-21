#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""把 PLC 主站（control 模式）的输出转成 check_sim_m2.py 能吃的 results CSV。

用法：
  /usr/bin/python3 scripts/plc_master_to_csv.py logs/plc_full_master.txt > logs/plc_results.csv

为什么需要转：PLC 接管路径不产生决策层的 results CSV，但落位断言要按 slot 查真值类型。
主站每轮打印 `[第 N 轮] slot=S 到位 ... 识别 type=T`，这里还原成 12 行。
识别失败或超时的轮次不写行——行数不足会被 A1 直接抓住，不会静默放过。
"""
import csv
import re
import sys

# 例：[第  1 轮] slot=1  到位  2.2s  识别 type=1 (287ms,  2.5s)  抓放完成 12.1s
LINE = re.compile(r"\[第\s*(\d+)\s*轮\]\s*slot=(\d+)\s+到位\s+[\d.]+s\s+识别 type=(\d+)")

FIELDS = ["visit_index", "slot_id", "material_type", "result", "note"]


def main():
    if len(sys.argv) < 2:
        print("用法：plc_master_to_csv.py <主站输出文件>", file=sys.stderr)
        return 2
    rows = []
    with open(sys.argv[1], encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = LINE.search(line)
            if not m:
                continue
            rows.append({"visit_index": int(m.group(1)),
                         "slot_id": int(m.group(2)),
                         "material_type": int(m.group(3)),
                         "result": "OK",
                         "note": "from plc_master"})
    rows.sort(key=lambda r: r["visit_index"])

    w = csv.DictWriter(sys.stdout, fieldnames=FIELDS, lineterminator="\n")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
