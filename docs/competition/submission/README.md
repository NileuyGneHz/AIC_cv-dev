# 省赛提交物目录（staging）

> 本目录是**提交前的汇集点**，不是存档库。最终传到报名系统 / 网盘时，
> 按队长群通知的命名规范重命名；本目录内的文件名只是工作名。

## 清单与现状（2026-10-25）

| # | 提交项 | 文件 | 状态 | 怎么来的 |
|---|---|---|---|---|
| 1 | 程序源代码 | `AIC_cv-dev_源代码.zip` | 已打包 | `./scripts/package_submission.sh`（`git archive`，只含被跟踪文件，约 5.0 MB） |
| 2 | 技术文档（PPT） | `AIC视觉分拣技术文档.pptx` | 已就绪 | `docs/competition/AIC视觉分拣技术文档/build_ppt.py` 生成（13 页，pdf 同名） |
| 3 | 视觉识别图集 | `视觉识别图集.png` | 已生成 | `src/vision/aic_vision_trad/scripts/make_submission_atlas.py`（12 帧，4 类 × 3，原图/结果对照） |
| 4 | 竞赛任务完成视频 | `AIC2026_任务完成.mp4` | **未录** | OBS 录看板窗口，一镜到底；`docs/guides/obs_recording_playbook.md` |
| 5 | 参赛视频（固定机位） | `AIC2026_参赛.mp4` | **未录** | 手机/相机固定三脚架，拍选手 + 屏幕全景 |
| — | 参赛承诺书 | — | **已提交**（2026-10-22） | 全员手写签字版，已交组委会 |

特征表 `视觉识别图集_特征表.csv` 是图集的附表（每帧五个归一化特征 + 判定理由），
不单独提交，答辩被问「阈值怎么定的」时直接打开。

## 重新生成

```bash
./scripts/package_submission.sh                # 全部重来（源码 zip + PPT + 图集）
./scripts/package_submission.sh --tag v1.0     # 顺手给当前提交打 tag
```

图集也可以只针对某一批调试帧重算（换一批样本核对鲁棒性时用）：

```bash
/usr/bin/python3 src/vision/aic_vision_trad/scripts/make_submission_atlas.py \
    --frames logs/m2_debug2 --out /tmp/atlas_b2.png --per-type 3
```

## 坑

1. **不要把 `logs/` 打包进去**。那里有上百次运行的调试帧与日志（数百 MB），
   官方只要「程序源代码 + 运行说明」。`package_submission.sh` 用 `git archive`
   从版本库取文件，天然不会带上；并且会先用 `git ls-files` 断言没有缓存/大文件
   被误跟踪（有就直接报错退出，不默默带进包）。
2. **源码 zip 与 PPT 不入 git**（`.gitignore` 已挡）：zip 是 git 的派生产物，
   PPT 在 `docs/competition/AIC视觉分拣技术文档/` 已有同一份，避免双份漂移。
   图集 PNG/CSV 要入 git——它是交付物，也是复现证据。
3. 视频文件名 `AIC2026_任务完成.mp4` / `AIC2026_参赛.mp4` 是工作名，
   以群通知规范为准（命名规则、是否需另附说明文档都还没最终确认）。
