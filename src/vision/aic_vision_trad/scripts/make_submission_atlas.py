#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""生成**提交版**视觉识别图集（省赛「单独立项提交」的图集，jpg/png）。

与 `make_atlas.py`（技术文档 PPT 内嵌图版）的区别：
  - 中文标注：用 PIL + Noto Sans CJK SC 排版，评委直接读，不用回文档查表；
  - 浅色底：PPT 图版是深色（配深色幻灯片），提交版要能打印、能单独看；
  - 原图/结果对照：每个样本给出「相机原图 → 识别结果」一对，
    满足「原始图 + 检测结果图对照」的提交要求；
  - 诚实口径：原图列 = 调试帧**去掉仿真侧绿色标注环**后的相机画面
    （`cv2.inpaint` 去叠加，1542 px 量级），识别结果列 = 提交算法对
    **同一张去叠加图**重新检测/判类的输出。检测就跑在图集展示的输入上，
    不存在「展示一套、跑另一套」。

用法：
  /usr/bin/python3 src/vision/aic_vision_trad/scripts/make_submission_atlas.py \
      --frames logs/m2_debug --out docs/competition/submission/视觉识别图集.png

输入与 make_atlas.py 相同：`run_sim.sh --vision trad --debug-dir <目录>` 产出的
`vision_<stamp>_<类型名>.png`（帧上带仿真侧画的绿色盘圆，本脚本负责去掉）。

产物：
  1. 图集 PNG（默认 3 样本/类 × 4 类 = 12 张，一镜一轮的任务帧，按时间取前 3 张/类）；
  2. 特征表 CSV（每帧的五个归一化特征 + 判定结果，供答辩被问时查）。
退出码：全部判对为 0，有误判为 1（误判帧仍会画进图集，标红，不藏）。
"""
import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plate_detector import DetectorParams, detect          # noqa: E402
from defect_classifier import TYPE_NAMES, classify         # noqa: E402

# ---------- 中文口径（与赛题/文档一致：1=划痕 2=污渍 3=缺角 4=孔洞） ----------
CN_NAME = {1: "划痕", 2: "污渍", 3: "缺角", 4: "孔洞"}
EN_NAME = {1: "scratch", 2: "stain", 3: "chip", 4: "hole"}
# 每类的判据一句话（对应 defect_classifier.DEFAULT_RULES，改阈值要同步改这里）
CRITERIA = {
    4: "圆度≥0.70 且 0.008≤面积比≤0.045 且 亮度落差≥100；贴边时以「斑内近黑像素占比≥0.05」兜底",
    3: "半径比≥0.68 且 长宽比≤5.0 且 面积比≥0.020",
    1: "长宽比≥12.0 且 圆度≤0.25",
    2: "兜底判据：取「面积×圆度」最大的连通域",
}
NAME_TYPE = {v: k for k, v in TYPE_NAMES.items()}

# ---------- 版式 ----------
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_SC_IDX = 2                       # ttc 内 Noto Sans CJK SC
TILE_W, TILE_H = 300, 225             # 4:3，与 640×480 同比例
ARROW_W = 40                          # 原图与结果之间的箭头宽度
LABEL_W = 236                         # 左侧类型说明列
CELL_GAP = 22                         # 样本间距
ROW_GAP = 26                          # 行间距
CAP_H = 104                           # 每样本下方说明高
HEAD_H = 150                          # 页头
FOOT_H = 150                          # 页脚
MARGIN = 38

# 浅色底（提交版要能打印）
BG = (255, 255, 255)
PANEL = (244, 246, 248)
BORDER = (206, 213, 220)
INK = (31, 41, 51)
INK2 = (82, 96, 109)
OK_C = (14, 138, 79)
BAD_C = (192, 57, 43)
# 识别结果图上的标注色（BGR，与仿真侧绿色环区分开）
C_CIRCLE = (215, 165, 40)             # 琥珀：盘圆 + 圆心
C_BOX = (40, 70, 200)                 # 蓝：缺陷外接框


def _font(size):
    return ImageFont.truetype(FONT_PATH, size, index=FONT_SC_IDX)


def wrap(draw, text, font, max_w):
    """按像素宽度换行（中文逐字断行，英文单词不拆）。"""
    lines, line = [], ""
    for ch in text:
        if ch == "\n":
            lines.append(line)
            line = ""
            continue
        trial = line + ch
        if draw.textlength(trial, font=font) > max_w and line:
            # 行尾是半个英文单词时，把最后一个单词整体移到下一行
            if ch.isascii() and ch.isalnum() and line and line[-1].isascii() and line[-1].isalnum():
                head = line.rsplit(" ", 1)[0]
                tail = line[len(head):].lstrip(" ")
                if head:
                    lines.append(head)
                    line = tail + ch
                    continue
            lines.append(line)
            line = ch
        else:
            line = trial
    if line:
        lines.append(line)
    return lines


def pick_frames(frames_dir, per_type):
    """{类型编号: [按时间排序的前 per_type 张]}；与 make_atlas.py 同口径。"""
    buckets = {}
    for path in sorted(glob.glob(os.path.join(frames_dir, "vision_*.png"))):
        suffix = os.path.basename(path)[:-4].rsplit("_", 1)[-1]
        tid = NAME_TYPE.get(suffix)
        if tid is None:
            continue
        buckets.setdefault(tid, []).append(path)
    return {t: ps[:per_type] for t, ps in buckets.items()}


def strip_overlay(path):
    """去掉仿真侧叠加的绿色盘圆，返回去叠加后的灰度图。

    只去「G 明显高于 R/B」的像素（实测该批帧只有绿环、无红字标签），
    用邻域灰度 inpaint 回填；环宽 2 px、底层是平滑灰度渐变，回填无可见残影。
    """
    bgr = cv2.imread(path)
    if bgr is None:
        return None
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    overlay = ((g - np.maximum(r, b)) > 60).astype(np.uint8)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if overlay.any():
        gray = cv2.inpaint(gray, overlay, 3, cv2.INPAINT_TELEA)
    return gray


def annotate(gray, res):
    """在去叠加图上画本算法的识别结果：琥珀盘圆 + 圆心 + 蓝色缺陷外接框。"""
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    pl = res["plate"]
    cx, cy, rr = int(pl["cx"]), int(pl["cy"]), int(pl["r"])
    cv2.circle(vis, (cx, cy), rr, C_CIRCLE, 2)
    cv2.drawMarker(vis, (cx, cy), C_CIRCLE, cv2.MARKER_CROSS, 11, 1)
    if res["comps"]:
        x, y, w, h = max(res["comps"], key=lambda d: d["area"])["bbox"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), C_BOX, 2)
    return vis


def translate_why(why):
    """classify() 的理由串翻成中文（前缀是判据名，后面是数值）。"""
    table = (("hole frac", "孔洞判据：面积比"), ("hole core", "孔洞判据：近黑核心"),
             ("chip ", "缺角判据："), ("scratch ", "划痕判据："), ("stain ", "污渍判据："),
             ("no candidate", "无可用候选"))
    for en, cn in table:
        if why.startswith(en):
            rest = why[len(en):].strip()
            if not rest:
                return cn
            # 原文是 "hole frac=0.021 ..."，替换前缀后剩下 "=0.021 ..."：等号前不补空格
            sep = "" if rest.startswith("=") else " "
            return cn + sep + rest
    return why


def feature_line(c):
    if not c:
        return "未定位到缺陷连通域"
    return ("面积比 frac %.3f    圆度 circ %.2f    半径比 rad %.2f\n"
            "长宽比 elong %.1f    亮度落差 drop %.0f"
            % (c["frac"], c["circ"], c["rad"], c["elong"], c["drop"]))


def build(frames_dir, out_path, per_type=3, feat_csv="", team=""):
    frames = pick_frames(frames_dir, per_type)
    if not frames:
        print("在 %s 下没有找到 vision_*.png，先跑 "
              "./scripts/run_sim.sh --vision trad --debug-dir %s"
              % (frames_dir, frames_dir))
        return 1

    cell_w = TILE_W + ARROW_W + TILE_W
    row_w = LABEL_W + CELL_GAP + per_type * cell_w + (per_type - 1) * CELL_GAP
    width = MARGIN * 2 + row_w
    height = HEAD_H + len(frames) * (TILE_H + CAP_H + ROW_GAP) + FOOT_H

    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)

    # ---------- 页头 ----------
    d.text((MARGIN, 30), "视觉识别图集", font=_font(46), fill=INK)
    d.text((MARGIN, 88), "第八届全球校园人工智能算法精英大赛 · 算法应用赛 · 视觉逻辑算法应用赛"
                         "（区域赛提交）", font=_font(19), fill=INK2)
    meta = "圆形铁片四类缺陷：划痕 / 污渍 / 缺角 / 孔洞    样本来源：本队 Gazebo 仿真相机 640×480"
    if team:
        meta = team + "    " + meta
    for i, seg in enumerate(wrap(d, meta, _font(16), width - MARGIN * 2)):
        d.text((MARGIN, 114 + i * 22), seg, font=_font(16), fill=INK2)
    d.line((MARGIN, HEAD_H - 12, width - MARGIN, HEAD_H - 12), fill=BORDER, width=2)

    # ---------- 四类样本 ----------
    ok, bad, rows = 0, 0, []
    y = HEAD_H
    for tid in sorted(frames):
        # 左侧类型说明
        d.rectangle((MARGIN, y, MARGIN + LABEL_W - CELL_GAP, y + TILE_H + CAP_H),
                    fill=PANEL, outline=BORDER, width=1)
        lx = MARGIN + 18
        d.text((lx, y + 16), "类型 %d" % tid, font=_font(22), fill=INK2)
        d.text((lx, y + 48), CN_NAME[tid], font=_font(40), fill=INK)
        d.text((lx, y + 100), EN_NAME[tid], font=_font(17), fill=INK2)
        d.line((lx, y + 128, MARGIN + LABEL_W - CELL_GAP - 18, y + 128), fill=BORDER, width=1)
        # 判据自动换行
        ly = y + 140
        for seg in wrap(d, CRITERIA[tid], _font(15), LABEL_W - 56):
            d.text((lx, ly), seg, font=_font(15), fill=INK2)
            ly += 22

        # 每个样本：原图 → 识别结果
        for col, path in enumerate(frames[tid]):
            cx0 = MARGIN + LABEL_W + CELL_GAP + col * (cell_w + CELL_GAP)
            gray = strip_overlay(path)
            if gray is None:
                continue
            res = detect(gray, DetectorParams())
            rec = {"frame": os.path.basename(path), "gt": tid, "pred": "", "r": "",
                   "frac": "", "circ": "", "rad": "", "elong": "", "drop": "",
                   "why": "", "verdict": ""}
            if res is not None:
                pred, why = classify(res["comps"])
                c = max(res["comps"], key=lambda x: x["area"]) if res["comps"] else None
                good = (pred == tid)
                ok, bad = ok + good, bad + (not good)
                rec.update({"pred": pred, "r": round(res["plate"]["r"], 2),
                            "why": translate_why(why),
                            "verdict": "correct" if good else "wrong"})
                if c:
                    rec.update({"frac": round(c["frac"], 4), "circ": round(c["circ"], 3),
                                "rad": round(c["rad"], 3), "elong": round(c["elong"], 2),
                                "drop": round(c["drop"], 1)})
                vis = annotate(gray, res)
                c_main = c
            else:
                pred, why, good, c_main = None, "未定位到盘面", False, None
                bad += 1
                rec.update({"verdict": "wrong", "why": why})
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            raw_pil = Image.fromarray(cv2.cvtColor(
                cv2.resize(gray, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA),
                cv2.COLOR_GRAY2RGB))
            vis_pil = Image.fromarray(cv2.cvtColor(
                cv2.resize(vis, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA),
                cv2.COLOR_BGR2RGB))
            # 小标题 + 图
            d.text((cx0 + 2, y - 2), "相机原图", font=_font(14), fill=INK2)
            d.text((cx0 + TILE_W + ARROW_W + 2, y - 2), "识别结果", font=_font(14), fill=INK2)
            img.paste(raw_pil, (cx0, y))
            img.paste(vis_pil, (cx0 + TILE_W + ARROW_W, y))
            # 箭头
            ax = cx0 + TILE_W + 6
            ay = y + TILE_H // 2
            d.polygon([(ax + 18, ay), (ax + 4, ay - 8), (ax + 4, ay + 8)], fill=INK2)
            d.line((ax, ay, ax + 14, ay), fill=INK2, width=2)
            # 边框
            d.rectangle((cx0, y, cx0 + TILE_W, y + TILE_H), outline=BORDER, width=1)
            d.rectangle((cx0 + TILE_W + ARROW_W, y, cx0 + cell_w, y + TILE_H),
                        outline=BORDER, width=1)
            # 下方说明
            cy = y + TILE_H + 8
            tint = OK_C if good else BAD_C
            mark = "✓" if good else "✗"
            got = CN_NAME.get(pred, "未识别")
            d.text((cx0 + 2, cy), "判定：%s %s %s" % (got, mark,
                   "正确" if good else "误判"), font=_font(17), fill=tint)
            d.text((cx0 + 2, cy + 26), feature_line(c_main), font=_font(13), fill=INK2)
            d.text((cx0 + 2, cy + 68), translate_why(why) if res is not None else why,
                   font=_font(13), fill=INK)
            rows.append(rec)
        y += TILE_H + CAP_H + ROW_GAP

    # ---------- 页脚 ----------
    fy = y + 6
    d.line((MARGIN, fy, width - MARGIN, fy), fill=BORDER, width=2)
    foot_font = _font(14)
    notes = [
        "说明：① 每个样本左列为相机原图（已去除仿真侧调试标注环），右列为提交算法对同一帧重新检测与判类的输出"
        "（琥珀色=定位到的盘圆与圆心，蓝色=缺陷外接框）；② 五个特征量均按盘半径归一化，圆精修后半径抖动 0.1–0.5 px，"
        "四类判据阈值留有 2 倍以上余量；",
        "③ 本批 %d 帧取自同一轮 12/12 全部放对的分拣任务，每类按时间顺序取前 %d 帧，无人工挑选；"
        "④ 图集与特征表由脚本从调试帧自动复算生成（make_submission_atlas.py），可与答辩现场演示互相印证。"
        % (len(rows), per_type),
    ]
    ly = fy + 14
    for n in notes:
        for seg in wrap(d, n, foot_font, width - MARGIN * 2):
            d.text((MARGIN, ly), seg, font=foot_font, fill=INK2)
            ly += 22
    stat = "本批判定：%d 正确 / %d 误判" % (ok, bad)
    d.text((MARGIN, ly + 8), stat, font=_font(16),
           fill=OK_C if bad == 0 else BAD_C)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    img.save(out_path)
    if feat_csv:
        os.makedirs(os.path.dirname(feat_csv) or ".", exist_ok=True)
        with open(feat_csv, "w", encoding="utf-8-sig", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["frame", "gt", "pred", "r", "frac", "circ",
                                              "rad", "elong", "drop", "why", "verdict"])
            w.writeheader()
            w.writerows(rows)
        print("特征表：%s（%d 行）" % (feat_csv, len(rows)))
    print("图集：%s（%d×%d，%d 张，命中 %d，误判 %d）"
          % (out_path, width, height, ok + bad, ok, bad))
    return 0 if bad == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="生成提交版视觉识别图集（中文、原图/结果对照）")
    ap.add_argument("--frames", default="logs/m2_debug", help="标注帧目录")
    ap.add_argument("--out", default="docs/competition/submission/视觉识别图集.png",
                    help="输出 PNG")
    ap.add_argument("--per-type", type=int, default=3, help="每类取几张（默认 3）")
    ap.add_argument("--features-csv", default="",
                    help="特征表输出路径（默认与图集同目录同名 .csv）")
    ap.add_argument("--team", default="", help="页头队伍名，如「广西大学 GXU RobotZ」")
    args = ap.parse_args()
    csv_path = args.features_csv
    if not csv_path:
        csv_path = os.path.splitext(args.out)[0] + "_特征表.csv"
    return build(args.frames, args.out, args.per_type, csv_path, args.team)


if __name__ == "__main__":
    sys.exit(main())
