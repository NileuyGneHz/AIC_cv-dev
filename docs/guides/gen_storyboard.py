#!/usr/bin/env python3
"""把 capture_storyboard.py 抓的帧拼成手册用「全过程」图版。

用法：
    python3 docs/guides/gen_storyboard.py logs/storyboard2 docs/guides/assets/storyboard.png

布局：3 列 x 4 行，每格 640x360（16:9，非 16:9 的图 letterbox），下方 32 px 黑底标注条。
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

CELL_W, CELL_H = 640, 360
BAR_H = 34
COLS, ROWS = 3, 4
PAD = 8
TITLE_H = 54

# (帧 tag 关键字, 标注)。按时间/语义顺序排列，脚本按关键字在目录里找文件。
SHOTS = [
    ("r01_loading", "0 · 上料：物料自供料管送到置物台"),
    ("r01_arrived", "1 · 到位：龙门让开光路，相机拍照"),
    ("r01_cam",     "2 · 识别：末端相机原图（真视觉输入）"),
    ("r01_grip",    "3 · 吸附：吸盘贴住，建柔性约束"),
    ("r01_carry",   "4 · 搬运：物料随龙门横移"),
    ("r01_placed",  "5 · 放入：落到类型列最低层 L1"),
    ("r02_arrived", "6 · 第 2 轮：换料，动作完全重复"),
    ("r02_placed",  "7 · 第 2 轮放入：同列第 2 层"),
    ("r06_arrived", "8 · 第 6 轮：半程，节拍稳定"),
    ("r06_placed",  "9 · 半程：前 6 块已按类型就位"),
    ("r12_placed",  "10 · 第 12 轮：最后一格填满"),
    ("r99_final",   "11 · 终态：4 类型列 x 3 层全满"),
]

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
]


def load_font(size):
    for path in FONT_CANDIDATES:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def find_frame(frames, keyword):
    for name in frames:
        if keyword in name:
            return name
    return None


def fit(img, w, h):
    """等比缩放并居中，空余补深灰（16:9 之外的图不拉伸）。"""
    canvas = Image.new("RGB", (w, h), (26, 30, 36))
    scale = min(w / img.width, h / img.height)
    nw, nh = max(1, int(img.width * scale)), max(1, int(img.height * scale))
    canvas.paste(img.resize((nw, nh), Image.LANCZOS), ((w - nw) // 2, (h - nh) // 2))
    return canvas


def main():
    src_dir = sys.argv[1] if len(sys.argv) > 1 else "logs/storyboard2"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "docs/guides/assets/storyboard.png"

    frames = sorted(f for f in os.listdir(src_dir) if f.endswith(".png"))
    if not frames:
        sys.exit("没有找到帧：%s" % src_dir)

    title_font = load_font(26)
    label_font = load_font(19)
    width = COLS * CELL_W + (COLS + 1) * PAD
    height = TITLE_H + ROWS * (CELL_H + BAR_H) + (ROWS + 1) * PAD
    sheet = Image.new("RGB", (width, height), (244, 246, 250))
    draw = ImageDraw.Draw(sheet)
    draw.text((PAD + 6, 14), "Gazebo 仿真全过程（正俯视机位 · M3 单轮 145 s 实测）",
              font=title_font, fill=(20, 22, 28))

    missing = []
    for i, (key, label) in enumerate(SHOTS):
        name = find_frame(frames, key)
        if name is None:
            missing.append(key)
            continue
        col, row = i % COLS, i // COLS
        x = PAD + col * (CELL_W + PAD)
        y = TITLE_H + PAD + row * (CELL_H + BAR_H + PAD)
        with Image.open(os.path.join(src_dir, name)) as im:
            sheet.paste(fit(im.convert("RGB"), CELL_W, CELL_H), (x, y))
        draw.rectangle([x, y + CELL_H, x + CELL_W, y + CELL_H + BAR_H], fill=(26, 30, 36))
        draw.text((x + 8, y + CELL_H + 6), label, font=label_font, fill=(235, 238, 244))

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    sheet.save(out_path, optimize=True)
    print("已生成 %s（%d x %d）" % (out_path, sheet.width, sheet.height))
    if missing:
        print("缺失帧（关键字）：%s" % ", ".join(missing))


if __name__ == "__main__":
    main()
