# aic_vision_trad —— 传统视觉识别链路

不训练模型，纯 OpenCV。目标是把「相机图 -> 缺陷类别」这一步做成确定性、可解释、可离线回归的模块。

## 1. 它做什么

```text
/camera/image ─┐
               ├─> 盘面定位 ─> 缺陷候选 ─> 四类判据 ─> /vision/material_type ("1".."4")
/vision/trigger┘   (凹槽 ROI)   (连通域)     (物理形态)
```

类别口径（与决策侧一致）：`1`=划痕、`2`=污渍、`3`=缺角、`4`=孔洞。识别不出就不发消息，
决策侧按超时处理（不会收到非法值）。

## 2. 算法

**第一步：定位盘面。** 不用全图 `HoughCircles`。台面与铁片灰度只差几级，全图 Hough 会锁到
台面边缘或夹爪上，实测圆心在 `(220,240)` 与 `(319,239)` 之间乱跳。改成：

1. Otsu 自动取阈，找**不接触图像边框的最大暗连通域** = 库位凹槽；
2. 只在凹槽 ROI 内做 `HoughCircles`，圆心稳定到 ±3 px；
3. 以盘半径 `r` 归一化所有特征，因此对相机高度不敏感。

**第二步：取缺陷候选。** 在 `0.93r` 的盘内圆域里，取显著暗于盘面（`level - 30`）的连通域，
算四个归一化特征：`frac`（占盘面积）、`circ`（圆度）、`elong`（minAreaRect 长宽比）、
`rad`（质心到盘心 / r）。

**第三步：判据按物理形态排序**（顺序不能改）：

| 顺序 | 类别 | 判据 | 依据 |
|---|---|---|---|
| A | 孔洞 | `circ≥0.70` 且 `0.008≤frac≤0.045` 且 `drop≥100` | 贯穿孔反射率最低，比盘面暗一大截 |
| B | 缺角 | `rad≥0.68` 且 `elong≤5.0` 且 `frac≥0.035` | 成块暗区压在盘缘上 |
| C | 划痕 | `elong≥12` 且 `circ≤0.25` | 极细长直条 |
| D | 污渍 | 兜底，取 `area×circ` 最大者 | 盘内大片圆润灰斑 |

**坑：为什么「贴边」不能放第一条。** 盘缘有一圈圆柱侧壁的暗沿（`rad≈0.85`、细长、
`frac<0.03`）。早期版本把贴边判据放在最前，侧壁暗沿抢在划痕/孔洞之前命中，
12 张样本只对 6 张。现在用 `elong>5`（细长就不是缺角）和 `frac≥0.035`（成块才有面积）
两道**独立**闸门挡掉它，两条都不满足才轮不到。

## 3. 离线回归（改判据后必跑）

```bash
cd ~/AIC_cv-dev/src/vision/aic_vision_trad
/usr/bin/python3 scripts/bench_offline.py \
    --dataset ~/AIC_cv-dev/logs/vision_ds \
    --layout ../../simulation/aic_gazebo_sim/config/sim_layout.yaml \
    --dump
```

数据集两种目录约定自动识别：`slot_NN/*.png`（按 `sim_layout.yaml` 的 `materials.types`
取真值）或 `<label>/*.png`（`label ∈ scratch|stain|chip|hole`，实拍数据用这个）。
退出码 0 = 达到 `--min-acc`（默认 95%），可直接当 CI 门禁。

当前基线：**12/12 = 100%**。

## 4. 在仿真里跑真视觉

```bash
cd ~/AIC_cv-dev
./scripts/run_sim.sh --vision trad              # 端到端一轮
./scripts/run_sim.sh --vision trad --debug-dir logs/vision_debug
```

或直接：

```bash
ros2 launch aic_gazebo_sim sim_decision.launch.py vision_backend:=trad
```

触发链路：`gantry_controller` 到位后发 `/vision/trigger`（`std_msgs/Empty`）→
本节点等到**请求之后**拍的帧才处理 → 发 `/vision/material_type`。

## 5. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `image_topic` | `/camera/image` | 图像输入 |
| `trigger_topic` | `/vision/trigger` | 采集请求 |
| `result_topic` | `/vision/material_type` | 结果输出 |
| `frame_wait_s` | `1.20` | 收到请求后最多尝试多久（期间逐帧重试）；须小于决策的 `vision_timeout_s` |
| `settle_skip_s` | `0.15` | 到位瞬间跳过这么多秒内到达的帧（那些还是运动中画面） |
| `max_stale_s` | `1.50` | 帧超过这个年龄就不判（宁缺勿错） |
| `debug_dir` | 空 | 非空则存标注图（可用于提交材料的「图像集展示」） |
| `dark_delta` | `30` | 比盘面暗多少算缺陷 |
| `hole_*` / `chip_*` / `scratch_*` | 见代码 | 判据阈值，现场可调 |

## 6. 坑

- **订阅端必须 `best_effort`。** gz→ros 的图像桥按 sensor data QoS 发；reliable 订阅者
  连 best effort 发布者是「不兼容」，一条都收不到且不报错。
- **不要用图像 `header.stamp` 算帧年龄。** 仿真里是 sim time，一旦上游没带 stamp 或时钟
  未同步，年龄会变成几十万秒，触发永远被判「帧太旧」。本节点用收到帧的本地单调时钟。
- **盘缘侧壁暗沿不是缺陷。** 见第 2 节。
- **`frac` 依赖 `r` 的估计。** Hough 的 `r` 有 ±3 px 波动（≈±7% 面积），所以判据余量要留够；
  现在的 `chip` 用 `elong` 二次确认，就是为此。
