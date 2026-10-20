# -*- coding: utf-8 -*-
"""生成《圆形铁片四缺陷识别与分拣》技术文档 PPT（16:9，13 页，学术答辩风）。

用法：
    PYTHONPATH=/home/nileuy/AIC_cv-dev/.pptx_deps \
      TMPDIR=/home/nileuy/AIC_cv-dev/.pip_tmp \
      /home/nileuy/.workbuddy/binaries/python/versions/3.13.12/bin/python3 build_ppt.py

设计口径（与 STORY.md 一致）：
  - 主色 宝蓝 #1E4FA8 / 藏蓝 #0E3F8C / 浅蓝 #E8EFF8；强调 橙 #E8A33D；
    正文 #1A2230；次要 #5A6474；分隔线 #D8DEE8
  - 字体 微软雅黑（本机由 Noto Sans CJK 兜底渲染，Windows/WPS 原生可用）
  - 每页：编号方块 + 标题 + 副标题 + 细分隔线 + 页脚
所有数字都来自实测（见 docs/guides/simulation_guide.md 与 competition 文档）。
"""
import os

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches, Pt
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
ATLAS = os.path.join(HERE, "..", "..", "guides", "assets", "vision_atlas.png")
OUT = os.path.join(HERE, "AIC视觉分拣技术文档.pptx")

BLUE = RGBColor(0x1E, 0x4F, 0xA8)
NAVY = RGBColor(0x0E, 0x3F, 0x8C)
LIGHT = RGBColor(0xE8, 0xEF, 0xF8)
ORANGE = RGBColor(0xE8, 0xA3, 0x3D)
TEXT = RGBColor(0x1A, 0x22, 0x30)
MUTED = RGBColor(0x5A, 0x64, 0x74)
LINE = RGBColor(0xD8, 0xDE, 0xE8)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
GREEN = RGBColor(0x2E, 0x7D, 0x4F)
RED = RGBColor(0xC0, 0x39, 0x2B)
FONT = "微软雅黑"

W, H = 13.333, 7.5
MARGIN = 0.72
CONTENT_W = W - 2 * MARGIN


def set_font(run, size=14, bold=False, color=TEXT, name=FONT, italic=False):
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.italic = italic
    run.font.color.rgb = color
    run.font.name = name
    # 东亚字体必须单独写 a:ea，否则 PowerPoint 对中文回落到宋体
    rPr = run._r.get_or_add_rPr()
    for tag in ("a:ea", "a:cs"):
        el = rPr.find(qn(tag))
        if el is None:
            el = etree.SubElement(rPr, qn(tag))
        el.set("typeface", name)


def para(tf, text, size=14, bold=False, color=TEXT, align=PP_ALIGN.LEFT,
         space_before=0, space_after=6, line=1.2, first=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    p.line_spacing = line
    r = p.add_run()
    r.text = text
    set_font(r, size, bold, color)
    return p


def textbox(slide, x, y, w, h, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = 0
    tf.margin_right = 0
    tf.margin_top = 0
    tf.margin_bottom = 0
    return tf


def rect(slide, x, y, w, h, fill=None, line=None, radius=None, shape=MSO_SHAPE.RECTANGLE):
    shp = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if fill is None:
        shp.fill.background()
    else:
        shp.fill.solid()
        shp.fill.fore_color.rgb = fill
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line
        shp.line.width = Pt(1.0)
    if radius is not None and shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        shp.adjustments[0] = radius
    shp.shadow.inherit = False
    shp.text_frame.word_wrap = True
    shp.text_frame.margin_left = Inches(0.10)
    shp.text_frame.margin_right = Inches(0.10)
    shp.text_frame.margin_top = Inches(0.04)
    shp.text_frame.margin_bottom = Inches(0.04)
    return shp


def header(slide, num, title, subtitle=""):
    rect(slide, MARGIN, 0.46, 0.46, 0.46, fill=BLUE)
    tf = slide.shapes[-1].text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf, "%02d" % num, 13, True, WHITE, PP_ALIGN.CENTER, first=True)
    tf = textbox(slide, MARGIN + 0.62, 0.44, 9.6, 0.5)
    para(tf, title, 25, True, TEXT, first=True, space_after=0)
    if subtitle:
        tf = textbox(slide, MARGIN + 0.63, 0.94, 11.0, 0.34)
        para(tf, subtitle, 11.5, False, MUTED, first=True, space_after=0)
    rect(slide, MARGIN, 1.335, W - 2 * MARGIN, 0.012, fill=LINE)


def footer(slide, idx, total=13):
    tf = textbox(slide, MARGIN, 7.06, 7.0, 0.28)
    para(tf, "AIC 视觉逻辑算法应用赛 · GXU RobotZ · 广西大学", 9, False, MUTED, first=True)
    tf = textbox(slide, W - MARGIN - 1.4, 7.06, 1.4, 0.28)
    para(tf, "%02d / %d" % (idx, total), 9, False, MUTED, PP_ALIGN.RIGHT, first=True)


def kpi(slide, x, y, w, h, value, label, sub=""):
    rect(slide, x, y, w, h, fill=LIGHT)
    rect(slide, x, y, 0.06, h, fill=BLUE)
    tf = textbox(slide, x + 0.22, y + 0.16, w - 0.3, 0.5)
    para(tf, value, 24, True, NAVY, first=True, space_after=0)
    tf = textbox(slide, x + 0.22, y + 0.68, w - 0.3, 0.32)
    para(tf, label, 11.5, False, TEXT, first=True, space_after=0)
    if sub:
        tf = textbox(slide, x + 0.22, y + 0.98, w - 0.3, 0.3)
        para(tf, sub, 9.5, False, MUTED, first=True, space_after=0)


def table(slide, data, x, y, w, col_w, row_h=0.40, font=10.5, header_h=None):
    rows, cols = len(data), len(data[0])
    shape = slide.shapes.add_table(rows, cols, Inches(x), Inches(y),
                                   Inches(w), Inches(row_h * rows))
    tbl = shape.table
    tbl.first_row = True
    tbl.horz_banding = False
    for i, cw in enumerate(col_w):
        tbl.columns[i].width = Emu(int(Inches(cw)))
    for r in range(rows):
        tbl.rows[r].height = Emu(int(Inches(header_h if (r == 0 and header_h) else row_h)))
        for c in range(cols):
            cell = tbl.cell(r, c)
            cell.margin_left = Inches(0.08)
            cell.margin_right = Inches(0.06)
            cell.margin_top = Inches(0.03)
            cell.margin_bottom = Inches(0.03)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.fill.solid()
            if r == 0:
                cell.fill.fore_color.rgb = NAVY
            else:
                cell.fill.fore_color.rgb = WHITE if r % 2 else RGBColor(0xF4, 0xF7, 0xFC)
            tf = cell.text_frame
            tf.word_wrap = True
            p = tf.paragraphs[0]
            p.alignment = PP_ALIGN.LEFT
            run = p.add_run()
            run.text = str(data[r][c])
            set_font(run, font, bold=(r == 0), color=WHITE if r == 0 else TEXT)
    return tbl


def picture_fit(slide, path, x, y, w, h):
    """按比例塞进 (x,y,w,h) 居中，不变形。"""
    from PIL import Image
    iw, ih = Image.open(path).size
    scale = min(w / iw, h / ih)
    pw, ph = iw * scale, ih * scale
    return slide.shapes.add_picture(path, Inches(x + (w - pw) / 2),
                                    Inches(y + (h - ph) / 2),
                                    Inches(pw), Inches(ph))


def bullets(slide, x, y, w, items, size=12, gap=0.42, color=TEXT, marker="·"):
    """items: [(标题, 说明)] 或 [str]。返回占用高度（英寸）。"""
    cy = y
    for it in items:
        if isinstance(it, tuple):
            head, body = it
            tf = textbox(slide, x, cy, w, 0.30)
            p = para(tf, "%s  %s" % (marker, head), size + 1, True, NAVY, first=True,
                     space_after=0)
            tf = textbox(slide, x + 0.20, cy + 0.30, w - 0.20, 0.30)
            para(tf, body, size, False, color, first=True, space_after=0, line=1.18)
            cy += gap + 0.30
        else:
            tf = textbox(slide, x, cy, w, 0.30)
            para(tf, "%s  %s" % (marker, it), size, False, color, first=True, space_after=0)
            cy += gap
    return cy - y


def build():
    prs = Presentation()
    prs.slide_width = Inches(W)
    prs.slide_height = Inches(H)
    blank = prs.slide_layouts[6]

    # ---------------------------------------------------------------- 01 封面
    s = prs.slides.add_slide(blank)
    rect(s, 0, 0, W, H, fill=NAVY)
    rect(s, 0, 0, 0.22, H, fill=ORANGE)
    tf = textbox(s, 1.05, 1.28, 11.0, 0.4)
    para(tf, "AIC 2026 · 算法应用赛 · 视觉逻辑算法应用赛", 13, False,
         RGBColor(0xC9, 0xD8, 0xF0), first=True)
    tf = textbox(s, 1.05, 1.90, 11.4, 1.0)
    para(tf, "圆形铁片四缺陷识别与分拣", 40, True, WHITE, first=True, space_after=6)
    tf = textbox(s, 1.05, 3.00, 11.4, 0.6)
    para(tf, "基于自建 Gazebo 数字孪生的视觉 — 决策 — 控制闭环", 19, False,
         RGBColor(0xDD, 0xE7, 0xF5), first=True)
    rect(s, 1.05, 3.86, 2.1, 0.045, fill=ORANGE)
    tf = textbox(s, 1.05, 4.16, 11.0, 0.9)
    para(tf, "GXU RobotZ · 广西大学", 15, True, WHITE, first=True, space_after=4)
    para(tf, "技术文档 v1.0 · 2026 年 11 月", 12, False, RGBColor(0xC9, 0xD8, 0xF0))
    tf = textbox(s, 1.05, 6.34, 11.2, 0.4)
    para(tf, "ROS 2 Humble · Ignition Fortress · BehaviorTree.CPP 4.10 · 传统几何视觉",
         10.5, False, RGBColor(0xA9, 0xBD, 0xDA), first=True)

    # ---------------------------------------------------------------- 02 目录
    s = prs.slides.add_slide(blank)
    header(s, 2, "目录", "五章：从平台对标到实测证据")
    items = [
        ("01", "方案总览", "四层架构、话题契约与三个核心指标"),
        ("02", "平台对标说明", "逐条对应官方规则，说明自建平台的等价性"),
        ("03", "关键算法", "铁片定位、四类缺陷判据、图像集"),
        ("04", "系统实现", "行为树决策、三轴控制、全过程可视化与实时判分"),
        ("05", "验证与展望", "主结果、泛化回归、负向注入与后续计划"),
    ]
    y = 1.78
    for num, name, desc in items:
        tf = textbox(s, MARGIN + 0.1, y, 0.9, 0.5)
        para(tf, num, 20, True, RGBColor(0xB9, 0xC7, 0xDD), first=True, space_after=0)
        tf = textbox(s, MARGIN + 1.0, y - 0.02, 3.4, 0.4)
        para(tf, name, 16, True, TEXT, first=True, space_after=0)
        tf = textbox(s, MARGIN + 4.3, y + 0.03, 7.2, 0.36)
        para(tf, desc, 11.5, False, MUTED, first=True, space_after=0)
        rect(s, MARGIN + 1.0, y + 0.455, W - MARGIN - 1.0, 0.010, fill=LINE)
        y += 0.92
    footer(s, 2)

    # ---------------------------------------------------------------- 03 方案总览
    s = prs.slides.add_slide(blank)
    header(s, 3, "方案总览", "四层解耦：仿真提供真值，视觉判型，决策调度，控制执行")
    cards = [
        ("仿真 Gazebo", "Ignition Fortress 数字孪生：12 块物料、4 列 × 3 层放置货架、\n三轴龙门 + 吸盘、固定相机；物理与渲染均为真实计算", 0.0, 0.0),
        ("视觉 aic_vision_trad", "OpenCV 传统几何：整图 Hough + 内亮外暗环对比 +\nKasa 圆精修 + 四类缺陷判据（无训练数据依赖）", 6.05, 0.0),
        ("决策 aic_behavior_tree", "BehaviorTree.CPP 4.10：12 轮巡视、GOTO/HANDLE 调度、\n视觉超时兜底与迟到结果防护", 0.0, 1.55),
        ("控制 gantry_controller", "FIFO 队列 + 梯形速度剖面 + 关节闭环到位；\n上料走 set_pose、取放走 DetachableJoint", 6.05, 1.55),
    ]
    for title, body, dx, dy in cards:
        x, y = MARGIN + dx, 1.62 + dy
        rect(s, x, y, 5.55, 1.34, fill=RGBColor(0xF7, 0xF9, 0xFC), line=LINE)
        rect(s, x, y, 0.055, 1.34, fill=BLUE)
        tf = textbox(s, x + 0.22, y + 0.14, 5.1, 0.32)
        para(tf, title, 13.5, True, NAVY, first=True, space_after=0)
        tf = textbox(s, x + 0.22, y + 0.50, 5.15, 0.78)
        para(tf, body, 10.5, False, TEXT, first=True, space_after=0, line=1.18)
    # 契约条
    rect(s, MARGIN, 4.62, CONTENT_W, 0.62, fill=LIGHT)
    tf = textbox(s, MARGIN + 0.18, 4.70, 2.0, 0.4)
    para(tf, "契约不变", 12, True, NAVY, first=True, space_after=0)
    tf = textbox(s, MARGIN + 1.55, 4.72, 10.4, 0.42)
    para(tf, "/control/material_task   ·   /vision/material_type   ·   "
             "/sim/gantry_arrived   ·   /sim/handle_done", 11, False, TEXT, first=True,
         space_after=0)
    # KPI
    kpi(s, MARGIN, 5.52, 3.55, 1.30, "12 / 12", "识别与放置全对", "真视觉、0 超时")
    kpi(s, MARGIN + 3.78, 5.52, 3.55, 1.30, "60 位点", "随机几何 0 误判", "5 个种子 × 12 块")
    kpi(s, MARGIN + 7.56, 5.52, 4.25, 1.30, "0.0 mm", "放置落位误差", "单轮约 145 s")
    footer(s, 3)

    # ---------------------------------------------------------------- 04 平台对标
    s = prs.slides.add_slide(blank)
    header(s, 4, "平台对标说明", "自建平台不是「替代品」：逐条实现官方规则，并用同一套指标验收")
    data = [
        ["官方规则 / 能力", "本方案实现", "验收证据"],
        ["置物台生成物料 → 拍照 → 抓取 → 放置", "每轮 set_pose 把 material_NN 传送到置物台，固定相机拍照后下探吸取",
         "上料 12 次 OK；吸附 / 释放各 12 次"],
        ["放置位四列对应四类缺陷，每列三个位置", "货架 4 列 × 3 层，按类型列的最低空层放入，每格一块",
         "12 块全部落对应列，L1→L3 逐层填满"],
        ["每个库位有且只有一个物料（不堆叠）", "控制器维护每列已用层数，放满即拒绝并报错", "落位断言：每格唯一，z 与层高一致"],
        ["三轴行程 X/Y/Z = 300/300/200 mm", "x ±0.45 m、y −0.24…+0.32 m、z −0.20…0 m", "行程覆盖供料管、置物台、最远层"],
        ["吸盘吸取（SuctionEnable）", "吸盘几何 + DetachableJoint 吸附/断气", "吸附与释放成对，物料随龙门移动"],
        ["自动运行、正确变绿 / 错误变红、自动计分", "行为树自动 12 轮；看板按类型列判分，绿 / 红 / 琥珀实时显示",
         "单轮约 145 s 无人干预；看板 12 绿"],
    ]
    table(s, data, MARGIN, 1.62, CONTENT_W, [3.30, 4.95, 3.60], row_h=0.66, font=10.5)
    tf = textbox(s, MARGIN, 6.52, CONTENT_W, 0.4)
    para(tf, "结论：官方平台的四项硬指标（上料节拍、四列三位置、吸盘取放、自动计分）在自建平台上均可复现，"
             "且每一步都有可回放的日志与位姿证据。", 10.5, False, NAVY, first=True, space_after=0)
    footer(s, 4)

    # ---------------------------------------------------------------- 05 任务与场景
    s = prs.slides.add_slide(blank)
    header(s, 5, "任务与场景", "布局由单一 YAML 驱动：改一处，世界、控制、判分同时生效")
    picture_fit(s, os.path.join(ASSETS, "scene_map.png"), MARGIN, 1.55, 7.35, 5.15)
    x = MARGIN + 7.65
    bullets(s, x, 1.62, 4.35, [
        ("上料位", "置物台 (0.30, −0.15)；12 块物料开机全部叠放在供料管内，按轮次传送"),
        ("放置货架", "4 列 = 四类缺陷；每列 3 层，层位沿 +Y 阶梯错开 0.11 m，避免吸盘升降撞上层托盘"),
        ("识别相机", "固定在置物台正上方，先拍照后下探，吸盘不遮挡光路"),
        ("坐标口径", "+X 向右（列 1→4），+Y 向远，+Z 向上；与 MaterialTask.frame_id 一致"),
    ], size=10.8, gap=0.62)
    footer(s, 5)

    # ---------------------------------------------------------------- 06 视觉定位
    s = prs.slides.add_slide(blank)
    header(s, 6, "关键算法 · 铁片定位", "先找「盘在哪」，再判「盘上是什么」——定位精度决定所有归一化判据")
    steps = [
        ("整图 Hough 圆检测", "不要求盘居中；对浅色台面 / 深色地贴都成立"),
        ("内亮外暗环对比选优", "用盘内外亮度差挑出真正的盘，排除孔洞与阴影误检"),
        ("径向扫描 + Kasa 拟合", "180 条射线 × 40 个半径，取边缘内点做最小二乘圆拟合"),
        ("失败兜底", "整图失败时退回 ROI 拟合圆；两条路径都过同一套精修"),
    ]
    y = 1.66
    for i, (head, desc) in enumerate(steps):
        rect(s, MARGIN, y, 0.42, 0.42, fill=BLUE, shape=MSO_SHAPE.OVAL)
        tf = s.shapes[-1].text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf, str(i + 1), 11, True, WHITE, PP_ALIGN.CENTER, first=True)
        tf = textbox(s, MARGIN + 0.62, y - 0.02, 5.5, 0.32)
        para(tf, head, 13, True, NAVY, first=True, space_after=0)
        tf = textbox(s, MARGIN + 0.62, y + 0.32, 5.6, 0.34)
        para(tf, desc, 10.5, False, TEXT, first=True, space_after=0)
        y += 1.18
    kpi(s, 7.35, 1.70, 2.55, 1.30, "0.1–0.5 px", "精修后半径抖动", "Hough 原始 ±9%")
    kpi(s, 10.10, 1.70, 2.55, 1.30, "85 px", "盘半径（640×480）", "距盘面约 0.26 m")
    rect(s, 7.35, 3.28, 5.30, 2.16, fill=RGBColor(0xF7, 0xF9, 0xFC), line=LINE)
    tf = textbox(s, 7.55, 3.44, 4.95, 2.0)
    para(tf, "为什么必须精修", 12, True, NAVY, first=True, space_after=6)
    para(tf, "缺角面积、孔洞半径都以「盘半径」归一化。Hough 的 dp=1.2 让半径落在 1.2 px 网格上，"
             "逐帧抖动 80.3–87.4 px；同一块料的 frac 会从 0.030 漂到 0.072，阈值无从下手。",
         10.5, False, TEXT, space_after=6, line=1.25)
    para(tf, "精修后组内标准差 0.10–0.49 px，缺角 frac 收窄到 0.053–0.064，阈值 0.020 才有 2.6 倍余量。",
         10.5, False, NAVY, line=1.25)
    rect(s, MARGIN, 6.12, 5.95, 0.50, fill=LIGHT)
    tf = textbox(s, MARGIN + 0.18, 6.20, 5.6, 0.34)
    para(tf, "定位失败时「不发消息」而非猜一个类型——决策侧记 TIMEOUT", 10.5, False, NAVY,
         first=True, space_after=0)
    footer(s, 6)

    # ---------------------------------------------------------------- 07 四类判据
    s = prs.slides.add_slide(blank)
    header(s, 7, "关键算法 · 四类缺陷判据", "几何量优先，几何失效时换物理量——判据链按「误判代价」排序")
    data = [
        ["类型", "主判据", "阈值", "实测余量"],
        ["4 孔洞", "圆度 + 面积 + 亮度差；贴边时用「近黑核心」兜底", "circ ≥ 0.70, 0.008 ≤ frac ≤ 0.045, drop ≥ 100",
         "孔 core 0.23–0.75，其余 0.000"],
        ["1 划痕", "细长比 + 圆度", "elong ≥ 12, circ ≤ 0.25", "实测 elong 20.6（划痕）"],
        ["3 缺角", "径向位置 + 圆度 + 面积", "rad ≥ 0.68, elong ≤ 5, frac ≥ 0.020", "frac 0.053–0.064，2.6 倍"],
        ["2 污渍", "以上都不满足的暗斑（兜底类）", "—", "与缺角以 elong 5.0 分界"],
    ]
    table(s, data, MARGIN, 1.62, CONTENT_W, [1.20, 4.30, 4.10, 2.25], row_h=0.66, font=9.5)
    rect(s, MARGIN, 5.12, CONTENT_W, 1.45, fill=RGBColor(0xF7, 0xF9, 0xFC), line=LINE)
    tf = textbox(s, MARGIN + 0.22, 5.26, CONTENT_W - 0.44, 1.2)
    para(tf, "两个设计取舍", 12, True, NAVY, first=True, space_after=5)
    para(tf, "① 阈值不贴实测值：缺角 frac 实测 0.053–0.064，阈值取 0.020 —— 为「精修被拒、退回整像素圆」"
             "的最坏情况留口子；真正挡住圆柱侧壁暗沿的是 elong ≤ 5（侧壁实测 7.4–11.3）。",
         10.5, False, TEXT, space_after=3, line=1.2)
    para(tf, "② 贴边孔洞会伪装成缺角：连通域被侧壁阴影粘住后 circ 0.98→0.34、rad 0.78，几何量整体失效；"
             "改用「斑内近黑像素占比」这一物理量，放在缺角判据之前。", 10.5, False, TEXT, line=1.2)
    footer(s, 7)

    # ---------------------------------------------------------------- 08 图像集
    s = prs.slides.add_slide(blank)
    header(s, 8, "图像集展示", "4 类 × 3 例：每格标注判据复算值，可直接核对阈值余量")
    picture_fit(s, ATLAS, MARGIN, 1.52, 6.0, 5.35)
    x = MARGIN + 6.30
    bullets(s, x, 1.66, 5.55, [
        ("采集口径", "全部来自 Gazebo 相机实时渲染，非预置图片；每帧 640×480，盘半径约 85 px"),
        ("标注内容", "类型、判据名与关键量（elong / circ / frac / drop / core），由脚本复算而非手写"),
        ("覆盖范围", "四类各 3 例，含贴边孔洞、细长划痕等边界样本"),
        ("用途", "作为阈值回归基线；几何随机化后同口径可再生成图版对比"),
    ], size=10.8, gap=0.70)
    tf = textbox(s, x, 5.30, 5.55, 1.4)
    para(tf, "图版由 scripts/make_atlas.py 从调试图重排生成：脚本会重新调用分类器，"
             "因此图上的判据值与当时的判定结果必然一致，不存在「图注与代码不同步」。",
         10, False, MUTED, first=True, space_after=0, line=1.25)
    footer(s, 8)

    # ---------------------------------------------------------------- 09 决策
    s = prs.slides.add_slide(blank)
    header(s, 9, "系统实现 · 行为树决策", "12 轮巡视是固定的，异常才是变量——兜底逻辑比主流程更重要")
    flow = [("巡视第 N 轮", "GOTO(slot)"), ("上料到位", "gantry_arrived"),
            ("视觉判型", "material_type"), ("取放执行", "HANDLE(slot,type)"),
            ("记录结果", "CSV + 看板")]
    bw, gap = 2.02, 0.36
    for i, (top, bot) in enumerate(flow):
        x = MARGIN + i * (bw + gap)
        shp = rect(s, x, 1.72, bw, 1.02, fill=LIGHT, line=BLUE,
                   shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.12)
        tf = shp.text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf, top, 11.5, True, NAVY, PP_ALIGN.CENTER, first=True, space_after=2)
        para(tf, bot, 9, False, MUTED, PP_ALIGN.CENTER, space_after=0)
        if i < len(flow) - 1:
            ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x + bw + 0.06),
                                    Inches(2.14), Inches(gap - 0.12), Inches(0.20))
            ar.fill.solid()
            ar.fill.fore_color.rgb = RGBColor(0xB9, 0xC7, 0xDD)
            ar.line.fill.background()
    guards = [
        ("视觉超时兜底", "决策对 GOTO/HANDLE 都递增 visit_index，不能用编号推相邻位点。"
                     "改为语义推断：新 GOTO 到达时，仍停在「识别中」的位点记 TIMEOUT。"),
        ("迟到结果防护", "识别结果晚到会被下一位点误收（类型错位但结果表仍显示 OK）。"
                     "窗口按最坏队列时延设定：M3 实测 GOTO→结果 15.5 s，现取 30 s。"),
    ]
    for i, (head, body) in enumerate(guards):
        x = MARGIN + i * (CONTENT_W / 2 + 0.10)
        rect(s, x, 3.20, CONTENT_W / 2 - 0.10, 1.62, fill=RGBColor(0xFD, 0xF6, 0xE7),
             line=RGBColor(0xE8, 0xC9, 0x8A))
        tf = textbox(s, x + 0.20, 3.34, CONTENT_W / 2 - 0.5, 1.36)
        para(tf, head, 12.5, True, RGBColor(0x9A, 0x62, 0x00), first=True, space_after=5)
        para(tf, body, 10.5, False, TEXT, space_after=0, line=1.25)
    rect(s, MARGIN, 5.12, CONTENT_W, 1.42, fill=LIGHT)
    tf = textbox(s, MARGIN + 0.22, 5.26, CONTENT_W - 0.44, 1.16)
    para(tf, "为什么用行为树", 12, True, NAVY, first=True, space_after=4)
    para(tf, "巡视、识别、取放三类动作的失败语义完全不同：识别失败要跳过、取放失败要记录、"
             "通信失败要重试。用树把「正常序列」与「异常分支」分开，"
             "决策层不含任何运动学细节，控制层可整体替换为真机 PLCnext 而不改一个节点。",
         10.5, False, TEXT, line=1.25)
    footer(s, 9)

    # ---------------------------------------------------------------- 10 控制
    s = prs.slides.add_slide(blank)
    header(s, 10, "系统实现 · 三轴控制", "指令是 fire-and-forget，执行必须自己保证「真的到位」")
    seq = ["上料：set_pose 传送到置物台", "走到停靠偏移位（吸盘让出光路）",
           "下探 → 吸盘吸附（DetachableJoint）", "抬升 → 平移到类型列最低空层",
           "触底放置 → 断气 → 抬升 → 完成"]
    y = 1.66
    for i, txt in enumerate(seq):
        rect(s, MARGIN, y + 0.06, 0.30, 0.30, fill=BLUE, shape=MSO_SHAPE.OVAL)
        tf = s.shapes[-1].text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf, str(i + 1), 9.5, True, WHITE, PP_ALIGN.CENTER, first=True)
        tf = textbox(s, MARGIN + 0.48, y, 5.9, 0.42)
        para(tf, txt, 11.5, False, TEXT, first=True, space_after=0)
        y += 0.80
    kpi(s, 7.30, 1.70, 2.55, 1.30, "0–1.2 mm", "稳态到位误差", "闭环判据")
    kpi(s, 10.05, 1.70, 2.55, 1.30, "3 / 10 mm/s", "位置 / 速度阈值", "连续 3 拍确认")
    rect(s, 7.30, 3.26, 5.30, 2.40, fill=RGBColor(0xF7, 0xF9, 0xFC), line=LINE)
    tf = textbox(s, 7.50, 3.40, 4.95, 3.0)
    para(tf, "三个必须写进代码的细节", 12, True, NAVY, first=True, space_after=6)
    para(tf, "① 到位 = 反馈收敛，不是剖面走完。早期按「剖面时间 + 0.15 s」报到位，实际还在追、"
             "过冲 17.7 mm，相机偏一整格。", 10.3, False, TEXT, space_after=5, line=1.2)
    para(tf, "② 每拍都下发目标。竖直轴是速度伺服，停发后重力把它拉到底（实测掉 420 mm）。",
         10.3, False, TEXT, space_after=5, line=1.2)
    para(tf, "③ 放置用「触底」判据：托盘接触后控制器仍在下压，按精确到位判会白等到超时，"
             "并连锁挤爆下一个位点的视觉窗口。", 10.3, False, TEXT, line=1.2)
    rect(s, MARGIN, 5.62, 5.95, 0.52, fill=LIGHT)
    tf = textbox(s, MARGIN + 0.18, 5.70, 5.6, 0.36)
    para(tf, "单次 HANDLE 实测 12.8 s：取料 + 搬运 + 放入三个动作", 10.5, False, NAVY,
         first=True, space_after=0)
    footer(s, 10)

    # ---------------------------------------------------------------- 11 可视化
    s = prs.slides.add_slide(blank)
    header(s, 11, "全过程可视化与实时判分", "一页看全：抓取过程、识别结果、得分——单窗口即可录制")
    picture_fit(s, os.path.join(ASSETS, "staging_camera_crop.png"), MARGIN, 1.62, 4.55, 3.40)
    tf = textbox(s, MARGIN, 5.12, 4.55, 0.6)
    para(tf, "固定相机实拍：置物台上的铁片与划痕（640×480）", 9.5, False, MUTED,
         first=True, space_after=0)
    x = MARGIN + 4.95
    bullets(s, x, 1.66, 6.95, [
        ("全景 · 自由摄像机", "世界内独立相机，看板可切 4 个预设机位并六向平移，抓取全程可见"),
        ("末端相机画中画", "识别原图 + 当前位点 / 判型 / 实时比分"),
        ("俯视态势图", "物料从供料管 → 置物台 → 吸盘（跟随龙门）→ 货架层位，全程位置可见"),
        ("实时判分", "按「放置列 == 物料类型」判绿 / 红；超时单独计数；纯旁路，不干扰主链路"),
    ], size=10.5, gap=0.62)
    rect(s, x, 4.42, 6.95, 1.70, fill=LIGHT)
    tf = textbox(s, x + 0.20, 4.56, 6.55, 1.9)
    para(tf, "为什么判分必须独立成节点", 11.5, True, NAVY, first=True, space_after=4)
    para(tf, "判分只订阅、不发布：它挂了链路照常跑，它算错也不会反向影响决策。"
             "真值只取 sim_layout.yaml 的 materials.types 一份事实，避免「视觉真值」与「判分真值」漂移。",
         10.2, False, TEXT, space_after=4, line=1.2)
    para(tf, "浏览器打开 127.0.0.1:8080，OBS 录这一个窗口即满足「单窗口合成录制」。",
         10.2, False, NAVY, line=1.2)
    footer(s, 11)

    # ---------------------------------------------------------------- 12 验证
    s = prs.slides.add_slide(blank)
    header(s, 12, "实验与验证", "所有结论都可在本机复现：一条命令跑完并自动断言")
    kpi(s, MARGIN, 1.60, 2.86, 1.28, "12 / 12", "主结果全对", "真视觉、0 超时")
    kpi(s, MARGIN + 3.05, 1.60, 2.86, 1.28, "0 误判", "60 位点泛化", "5 种子 × 12 块")
    kpi(s, MARGIN + 6.10, 1.60, 2.86, 1.28, "0.0 mm", "落位误差", "x / y 方向")
    kpi(s, MARGIN + 9.15, 1.60, 2.86, 1.28, "12 + 12", "吸附 / 释放", "无卡料、无漏放")
    data = [
        ["验证项", "方法", "结果"],
        ["识别正确性", "真视觉跑完整一轮，逐位点比对 materials.types", "12/12，0 超时"],
        ["阈值泛化", "jitter_seed 1–5 随机化缺陷几何，各跑 12 位点", "60 位点 0 误判"],
        ["物理落位", "pose/info 读 12 块物料世界坐标，对照类型列与层高", "误差 0.0 mm，L1–L3 逐层填满"],
        ["鲁棒性", "注入 60% 识别失败（drop_rate=0.6）", "巡视不中断，超时被正确计数"],
        ["关停正确性", "整轮后检查吸附 / 释放条数", "各 12 次，最后一块不卡在吸盘上"],
    ]
    table(s, data, MARGIN, 3.18, CONTENT_W, [2.35, 6.15, 3.35], row_h=0.52, font=10)
    tf = textbox(s, MARGIN, 6.50, CONTENT_W, 0.36)
    para(tf, "一键复现：./scripts/verify_sim_m2.sh（跑完自动调 check_sim_m2.py 做三组断言，失败打印具体条目）",
         10, False, MUTED, first=True, space_after=0)
    footer(s, 12)

    # ---------------------------------------------------------------- 13 总结
    s = prs.slides.add_slide(blank)
    header(s, 13, "总结与展望", "省赛要件已齐备；国赛把「能跑通」升级为「可迁移」")
    checks = [
        ("合规", "官方四项硬指标逐条实现：上料节拍、四列三位置、吸盘取放、自动计分"),
        ("可靠", "12/12、60 位点 0 误判、0.0 mm 落位；超时与迟到结果都有显式防护"),
        ("完整", "识别 → 决策 → 取放 → 判分全链路可看、可录、可复现，单条命令自验"),
    ]
    y = 1.66
    for tag, body in checks:
        rect(s, MARGIN, y, 0.86, 0.44, fill=BLUE)
        tf = s.shapes[-1].text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf, tag, 12, True, WHITE, PP_ALIGN.CENTER, first=True)
        tf = textbox(s, MARGIN + 1.05, y + 0.04, 5.6, 0.42)
        para(tf, body, 10.8, False, TEXT, first=True, space_after=0, line=1.2)
        y += 0.86
    rect(s, MARGIN + 6.95, 1.60, 4.95, 2.05, fill=RGBColor(0xF7, 0xF9, 0xFC), line=LINE)
    tf = textbox(s, MARGIN + 7.15, 1.76, 4.55, 4.1)
    para(tf, "下一步", 13, True, NAVY, first=True, space_after=8)
    for t in ["复合缺陷多标签：同一块料同时有划痕 + 孔洞时的判据优先级",
              "赛题映射表：把官方评分细则逐条映射到代码与日志字段",
              "真机迁移：控制层替换为 PLCnext，视觉与决策零改动",
              "录制与提交：按官方要求完成一镜到底录像与命名"]:
        para(tf, "·  " + t, 10.5, False, TEXT, space_after=6, line=1.25)
    tf = textbox(s, MARGIN, 5.62, 6.5, 1.0)
    para(tf, "技术文档与仿真平台均在本仓库内，可按 README 一键构建复现。",
         11, False, MUTED, first=True, space_after=0)
    footer(s, 13)

    assert os.path.isdir(os.path.dirname(OUT))
    prs.save(OUT)
    print("saved", OUT, "slides:", len(prs.slides))


if __name__ == "__main__":
    build()
