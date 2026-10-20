"""四类缺陷判据（1=划痕 2=污渍 3=缺角 4=孔洞）。

判据顺序按「最不容易被混淆」到「最容易被混淆」排：
  A 孔洞：小而圆、比盘面暗一大截（贯穿孔，反射率最低）
  B 缺角：成块暗区压在盘缘上
  C 划痕：极细长的直条
  D 污渍：盘内大片圆润灰斑（兜底）

顺序很重要。早期版本把「贴边」放在第一条，结果盘缘的圆柱侧壁暗沿
（rad≈0.85、细长、面积小）抢在划痕/孔洞之前命中，准确率只有 6/12。
现在用「细长（elong>5）就不是缺角」和「面积占比下限」两道独立闸门把它挡掉。

阈值在 24 张仿真帧（12 张 M1 样本 + 12 张 M2 运行帧）上整定，圆精修后的实测分布：
  hole    frac 0.020~0.021  circ 0.97~0.98  drop 123~126   （阈值 frac<=0.045, drop>=100）
  chip    frac 0.053~0.064  rad 0.77~0.78   elong 2.8~3.1  （阈值 frac>=0.020, elong<=5）
  scratch elong 23.4~23.5   circ 0.11                      （阈值 elong>=12, circ<=0.25）
  stain   frac 0.094~0.099  circ 0.92~0.94                 （兜底）
侧壁暗沿 elong 7.4~11.3 全部 > 5，不会误判缺角；四条判据的实际间隔都在 2 倍以上。
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
    # 贴边孔洞的兜底判据：斑内近黑像素（< level-120）占比。实测孔 0.23~0.75，其余 0.000。
    "hole_core_min": 0.05,
    "chip_rad_min": 0.68,
    "chip_elong_max": 5.0,
    # 缺角面积下限。取 0.020（不贴着实测值）：frac 用**估计出的盘半径**归一化。
    # 圆精修（plate_detector.refine_circle）之后半径稳定到亚像素（组内 std 0.1~0.5 px），
    # 缺角实测 frac 收在 0.053~0.064，阈值有 2.6 倍余量。
    # 仍要给最坏情况留口子：精修被拒时退回 Hough 的整像素圆（半径可差 ±9%），
    # 那时 0.93r 掩膜会把压在盘缘的缺角切掉大半，frac 掉到 0.030。
    # 真正挡住「圆柱侧壁暗沿」的是 elong<=5（侧壁实测 7.4~11.3），不是面积。
    "chip_frac_min": 0.020,
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

    # A2 贴边孔洞（几何量失效时靠「近黑核心」认）
    #    孔洞落在盘缘时，圆柱侧壁的阴影会和孔口连成一个连通域：circ 从 0.98 掉到 0.3、
    #    elong 从 1.0 涨到 2~9、rad 0.78 —— 上面那条和下面的缺角条都判不了它
    #    （实测被判成缺角，drop 101 夹在缺角 81~86 与孔洞 123 之间，光看亮度也分不开）。
    #    但孔是通透的：斑内 level-120 以下的像素占 0.23~0.75，其余三类全是 0.000。
    for c in comps:
        if c.get("core_frac", 0.0) >= r["hole_core_min"]:
            return TYPE_HOLE, "hole core={:.2f}（贴边，几何量不可靠）".format(
                c["core_frac"])

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
