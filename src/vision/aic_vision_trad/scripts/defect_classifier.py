"""四类缺陷判据（1=划痕 2=污渍 3=缺角 4=孔洞）。

图案口径 = 官方图 8（赛题规则 §2「任务流程和物料」）：一类一种形状，
只有位置 / 朝向 / 尺度随机。

  A 缺角：盘缘被咬掉一个楔口 —— 整块缺陷**贴着盘缘**，是最硬的判据
         （2026-10-26 加深缺口后 rad 实测 0.78，阈值 0.62）
  B 孔洞：空心三角轮廓 —— 1.6 mm 细线围一圈，**能把中间围住**
  C 污渍：实心圆斑 —— 又圆又实，占满自己的外接框
  D 划痕：细弧钩 —— 细线、开口，不闭合也不实心（兜底）

为什么这个顺序：三类之间的判别量互不重叠，先判「最不可能被别的类命中」的。

实测（2026-10-26，logs/patterns4，640x480、盘半径 85 px、圆精修后）：
  类     frac    circ   elong  rad   fill  solidity enclosed
  划痕   0.010   0.11   2.1    0.21  0.19  0.24     0.00
  污渍   0.023   0.96   1.0    0.11  0.87  1.05     0.00
  缺角   0.033   0.49   1.0    0.78  0.66  1.04     0.00
  孔洞   0.016   0.27   1.1    0.11  0.25  0.44     1.22
四道判据的关键量间隔都在 2 倍以上：rad 0.78 vs 0.21；enclosed 1.22 vs 0.000；
circ 0.96 vs 0.49；其余落兜底。

坑（历史）：盘缘圆柱侧壁的暗沿本身是「细长、贴边、面积小」的连通域（elong 7~11、
rad 0.87），一度把早期版本骗到 6/12。现在它在「缺角」条被 elong <= 4.5 挡掉，
在「污渍」条被 circ 挡掉 —— 不要再把 elong 放到最后判。
"""

TYPE_SCRATCH = 1
TYPE_STAIN = 2
TYPE_CHIP = 3
TYPE_HOLE = 4

TYPE_NAMES = {
    TYPE_SCRATCH: "scratch",
    TYPE_STAIN: "stain",
    TYPE_CHIP: "chip",     # 官方称「缺角」：盘缘楔形缺口（平台说明书旧版写作「破损」）
    TYPE_HOLE: "hole",
}

# 看板 / 日志里的中文显示名（与 score_node 的 TYPE_NAMES 保持一致；规则正文把 3 写作「缺角」）
TYPE_CN = {
    TYPE_SCRATCH: "划痕",
    TYPE_STAIN: "污渍",
    TYPE_CHIP: "破损",
    TYPE_HOLE: "孔洞",
}

DEFAULT_RULES = {
    # A 缺角：贴盘缘（rad 接近 1）且不是细长条（盘缘暗沿 elong 7~11 要挡掉）
    "chip_rad_min": 0.62,
    "chip_elong_max": 4.5,
    "chip_frac_min": 0.004,
    # B 孔洞：空心轮廓围住的像素 / 轮廓自身像素。空心 1.22，其余三类恒为 0.000。
    "hole_enclosed_min": 0.30,
    "hole_frac_min": 0.004,
    "hole_frac_max": 0.060,
    # C 污渍：又圆又实的斑
    "stain_circ_min": 0.60,
    "stain_solidity_min": 0.85,
    "stain_rad_max": 0.60,
    "stain_frac_min": 0.004,
}


def classify(comps, rules=None):
    """comps: plate_detector.extract_defects 的输出。

    返回 (type_id, reason)；type_id 为 0 表示没有可用候选（调用方不该发消息）。
    需要「判据命中的是哪个连通域」时用 classify_full()。
    """
    tid, why, _c = classify_full(comps, rules)
    return tid, why


def classify_full(comps, rules=None):
    """同 classify()，额外返回**判据实际命中的那个连通域**（dict，可能为 None）。

    看板的「识别结果」日志要用它把关键特征（frac/circ/enclosed/...）一并显示出来，
    这样现场看一行日志就知道判据是靠哪个量过的，不必再翻调试图。
    """
    r = dict(DEFAULT_RULES)
    if rules:
        r.update(rules)
    if not comps:
        return 0, "no candidate", None

    # A 缺角（盘缘缺口）
    for c in comps:
        if (c["rad"] >= r["chip_rad_min"]
                and c["elong"] <= r["chip_elong_max"]
                and c["frac"] >= r["chip_frac_min"]):
            return TYPE_CHIP, "chip rad={:.2f} elong={:.1f} frac={:.3f}".format(
                c["rad"], c["elong"], c["frac"]), c

    # B 孔洞（空心轮廓：能把中间围住）
    for c in comps:
        if (c.get("enclosed", 0.0) >= r["hole_enclosed_min"]
                and r["hole_frac_min"] <= c["frac"] <= r["hole_frac_max"]):
            return TYPE_HOLE, "hole enclosed={:.2f} frac={:.3f}".format(
                c.get("enclosed", 0.0), c["frac"]), c

    # C 污渍（实心圆斑）
    for c in comps:
        if (c["circ"] >= r["stain_circ_min"]
                and c["solidity"] >= r["stain_solidity_min"]
                and c["rad"] <= r["stain_rad_max"]
                and c["frac"] >= r["stain_frac_min"]):
            return TYPE_STAIN, "stain circ={:.2f} sol={:.2f} frac={:.3f}".format(
                c["circ"], c["solidity"], c["frac"]), c

    # D 划痕（兜底：面积最大的候选，细弧钩）
    c = max(comps, key=lambda x: x["area"])
    return TYPE_SCRATCH, "scratch fill={:.2f} elong={:.1f} frac={:.3f}".format(
        c["fill"], c["elong"], c["frac"]), c
