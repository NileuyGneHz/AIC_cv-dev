#!/usr/bin/env bash
# 省赛提交物打包：源码 zip + 图集 + 技术文档 PPT，汇到一个目录，附自检清单。
#
# 用法：
#   ./scripts/package_submission.sh                 # 输出到 docs/competition/submission/
#   ./scripts/package_submission.sh --out /tmp/pack # 自定义输出目录
#   ./scripts/package_submission.sh --tag v1.0      # 先给当前提交打 tag 再打包（默认用 HEAD）
#
# 为什么不直接压缩整个仓库：logs/ 里有上百次运行的调试帧与日志（数百 MB），
# .venv/、__pycache__、*.mp4 同理。官方只要求「程序源代码 + 运行说明」，
# 带一堆无关产物只会让评委难找入口，也可能把大文件顶上网盘/系统上限。
#
# 实现说明：用 `git archive --format=zip` 直接出包——只含被版本跟踪的文件，
# 不落临时目录、不需要事后清理。打包前会用 git ls-files 断言没有缓存/大文件
# 被误跟踪（真被跟踪了会直接报错退出，而不是悄悄带进包）。

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

OUT="docs/competition/submission"
TAG=""
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT="$2"; shift 2 ;;
        --tag) TAG="$2"; shift 2 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done
mkdir -p "$OUT"

echo "== 0/4 检查被跟踪文件里没有缓存/大文件"
JUNK=$(git ls-files | grep -E "pycache|\.pyc$|\.mp4$|\.venv/|(^|/)install/|(^|/)build/" || true)
if [ -n "$JUNK" ]; then
    echo "   !! 以下文件被 git 跟踪，不该进源码包：" >&2
    echo "$JUNK" | sed 's/^/      /' >&2
    echo "   先 git rm --cached 它们，再跑本脚本。" >&2
    exit 3
fi
echo "   ok（$(git ls-files | wc -l) 个被跟踪文件）"

echo "== 1/4 导出源码 zip"
if [ -n "$TAG" ]; then
    git tag -f "$TAG" HEAD >/dev/null
    REF="$TAG"
else
    REF="HEAD"
fi
git archive --format=zip --prefix="AIC_cv-dev/" "$REF" -o "$OUT/AIC_cv-dev_源代码.zip"
SZ=$(du -h "$OUT/AIC_cv-dev_源代码.zip" | cut -f1)
echo "   -> $OUT/AIC_cv-dev_源代码.zip（$SZ，来自 $REF）"

echo "== 2/4 复制技术文档 PPT"
if cp "docs/competition/AIC视觉分拣技术文档/AIC视觉分拣技术文档.pptx" "$OUT/"; then
    echo "   -> $OUT/AIC视觉分拣技术文档.pptx"
else
    echo "   !! PPT 不存在，先跑 docs/competition/AIC视觉分拣技术文档/build_ppt.py"
fi

echo "== 3/4 视觉识别图集（不存在则现场生成）"
if [ ! -f "$OUT/视觉识别图集.png" ]; then
    PATH=/usr/bin:$PATH /usr/bin/python3 \
        src/vision/aic_vision_trad/scripts/make_submission_atlas.py \
        --frames logs/m2_debug --out "$OUT/视觉识别图集.png" \
        || echo "   !! 图集生成失败（缺 cv2/PIL 或无帧），需手动补"
fi
[ -f "$OUT/视觉识别图集.png" ] && echo "   -> $OUT/视觉识别图集.png"

echo "== 4/4 自检清单"
FAIL=0
check() { # check <描述> <文件>
    if [ -e "$2" ]; then echo "   [ok] $1"; else echo "   [缺] $1  -> $2"; FAIL=1; fi
}
check "程序源代码 zip"  "$OUT/AIC_cv-dev_源代码.zip"
check "技术文档 PPT"    "$OUT/AIC视觉分拣技术文档.pptx"
check "视觉识别图集"    "$OUT/视觉识别图集.png"
check "任务完成视频"    "$OUT/AIC2026_任务完成.mp4"
check "参赛视频"        "$OUT/AIC2026_参赛.mp4"

echo
echo "输出目录：$OUT"
if [ $FAIL -eq 0 ]; then
    echo "四件套齐了（视频文件名按队长群通知规范再改）"
else
    echo "还有缺口：两段视频需 OBS 实录，步骤见 docs/guides/obs_recording_playbook.md"
fi
exit $FAIL
