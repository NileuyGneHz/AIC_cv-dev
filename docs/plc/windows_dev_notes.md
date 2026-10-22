# Windows 开发环境记录（2026-10-22 接手日）

> 读者：在这台 Windows 机器上继续 PLC 线开发的人（含 agent）。
> 配套交接文档：`docs/guides/windows_plc_handover.md`；本文只记**这台机器实测**的环境事实。

## 1. 环境现状

| 项 | 实测 | 影响 |
|---|---|---|
| Python | 3.14（默认）/ 3.13（Store）/ 3.11；**没有 3.10** | `PyPlcnextRsc` 0.1.12 导入即报错（`enum._decompose` 在 3.11 被删）→ `--dry-run/--probe/--once` 全链路本机暂时跑不了；交接文档 §1 的 `py -3.10` 尚未安装 |
| pip 直连 PyPI | **502 Bad Gateway**（代理隧道失败） | `pip install -r requirements-plc.txt` 会直接失败 |
| pip 走清华镜像 | **可用**（2026-10-22 实测装上 PyYAML / opencv-python-headless） | 命令加 `-i https://pypi.tuna.tsinghua.edu.cn/simple` |
| cv2 / PyYAML | 已装进 WorkBuddy 托管 venv（3.13），仅用于离线验证；正式环境仍按交接文档建 3.10 venv | — |

## 2. 本机 `F:\AIC_cv-dev` 仓库的 git 异常（已绕过，留档）

现象：`git fetch` 报 `56b1c70..bd265cb main -> origin/main`，但
`.git/refs/remotes/origin/main` **永远写不进去**（`update-ref` 静默失败、无 lock 残留、
`refs/heads/*` 却能正常写）。怀疑与某个文件系统过滤驱动有关，未定位根因。

绕过办法（已用）：

```bash
git fetch origin main:refs/heads/wb-upstream   # refs/heads 能写
git merge --ff-only wb-upstream                # ff 到 bd265cb
```

后果：`git status` 里 origin/main 永远显示「已最新」，别信它；以上面的 fetch 行为准。

## 3. 已完成的开发（本机验证记录）

| 交付物 | 验证方式 | 结果 |
|---|---|---|
| `src/plugins/aic_plc_gateway/config/cam_official.yaml`（P0-1） | `rsc_link.py --map ... --describe`（3.13 + PyYAML，`--describe` 不加载 RSC SDK） | 8 标签，路径全对：`program=Main 标签 8（读 3 / 写 5）` |
| `src/vision/aic_vision_trad/scripts/run_to_plc.py`（P0-2） | `py_compile` 语法检查 | 通过 |
| 同上 · 识别链路 | `vision_atlas.png` 裁 12 格帧（1140×1474，按亮度自动分行带），跑 `detect_and_classify` | **12/12**（scratch/stain/chip/hole 全对；单帧 4–114 ms） |
| 同上 · 写 PLC 时序 | FakeLink 记录 write 序列 | Data=[1,1,1,2,2,2,3,3,3,4,4,4]、Done×12；先 Data 后 Done |
| `deploy/vplc/{Dockerfile,requirements-arm64.txt,README.md}`（P0-3） | 未构建（本机 Docker 状态未知） | 构建验收见 deploy/vplc/README.md §2.3 |
| `scripts/check_vplc_link.ps1`（P1） | 未运行（无设备） | 真机接上后跑 |

**注意**：图版帧带标注（绿圈/橙框是画上去的），12/12 只能证明判据在「有干扰标注」的
帧上也稳；严格回归仍要 Ubuntu 侧 `logs/` 的原始帧（`bench_offline.py`）。

## 4. 待办（按交接文档 §6 的顺序）

1. 装 Python 3.10 → 建 `.venv` → `rsc_link.py --dry-run --watch` 26 标签全绿（交接文档 §2 自检）；
2. `run_to_plc.py --source folder --dry-run` 用真实日志帧 12/12；
3. P0-3 镜像构建验收；P0-4 真机四步（需要设备）。
