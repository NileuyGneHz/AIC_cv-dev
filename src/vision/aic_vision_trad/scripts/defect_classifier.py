"""四类缺陷判据（1=划痕 2=污渍 3=缺角 4=孔洞）。

判据顺序按「最不容易被混淆」到「最容易被混淆」排：
  A 孔洞：小而圆、比盘面暗一大截（贯穿孔，反射率最低）
  B 缺角：成块暗区压在盘缘上
  C 划痕：极细长的直条
  D 污渍：盘内大片圆润灰斑（兜底）

顺序很重要。早期版本把「贴边」放在第一条，结果盘缘的圆柱侧壁暗沿
（rad≈0.85、细长、面积小）抢在划痕/孔洞之前命中，准确率只有 6/12。
现在用「细长（elong>5）就不是缺角」和「面积占比 ≥ 3.5%」两道独立闸门把它挡掉。

阈值都是在 12 张仿真样本上整定的，实测余量：
  hole   drop 124 vs 次暗 77      | chip frac 0.054~0.065 vs 阈值 0.035
  scratch elong 23.4 vs 侧壁 11.3 | 侧壁 elong 7.4~11.3 全部 > 5，不会误判缺角
"""

TYPE_SCRATCH = 1
TYPE_STAIN = 2
TYPE_CHIP = 3
TYPE_HOLE = 4

TYPE_NAMES = {
    TYPE_SCRATCH: "scratch",
    TYPE_STAIN: "stain",
    TYPE_CHIP: "chip",
    TYPE_HOLE: "hole",
}

DEFAULT_RULES = {
    "hole_circ_min": 0.70,
    "hole_frac_min": 0.008,
    "hole_frac_max": 0.045,
    "hole_drop_min": 100.0,
    "chip_rad_min": 0.68,
    "chip_elong_max": 5.0,
    "chip_frac_min": 0.035,
    "scratch_elong_min": 12.0,
    "scratch_circ_max": 0.25,
}


def classify(comps, rules=None):
    """comps: plate_detector.extract_defects 的输出。

    返回 (type_id, reason)；type_id 为 0 表示没有可用候选（调用方不该发消息）。
    """
    r = dict(DEFAULT_RULES)
    if rules:
        r.update(rules)
    if not comps:
        return 0, "no candidate"

    # A 孔洞
    for c in comps:
        if (c["circ"] >= r["hole_circ_min"]
                and r["hole_frac_min"] <= c["frac"] <= r["hole_frac_max"]
                and c["drop"] >= r["hole_drop_min"]):
            return TYPE_HOLE, "hole frac={:.3f} circ={:.2f} drop={:.0f}".format(
                c["frac"], c["circ"], c["drop"])

    # B 缺角
    for c in comps:
        if (c["rad"] >= r["chip_rad_min"]
                and c["elong"] <= r["chip_elong_max"]
                and c["frac"] >= r["chip_frac_min"]):
            return TYPE_CHIP, "chip rad={:.2f} frac={:.3f} elong={:.1f}".format(
                c["rad"], c["frac"], c["elong"])

    # C 划痕
    for c in comps:
        if c["elong"] >= r["scratch_elong_min"] and c["circ"] <= r["scratch_circ_max"]:
            return TYPE_SCRATCH, "scratch elong={:.1f} circ={:.2f}".format(
                c["elong"], c["circ"])

    # D 污渍（兜底：取「面积 x 圆度」最大的那个）
    c = max(comps, key=lambda x: x["area"] * x["circ"])
    return TYPE_STAIN, "stain frac={:.3f} circ={:.2f}".format(c["frac"], c["circ"])
