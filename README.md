# AIC_cv-dev

Using for AIC competition 2026 (视觉逻辑算法应用赛: 视觉识别 → 坐标解算 → 运动控制).

Team workspace conventions live in [AGENTS.md](AGENTS.md) (AI 助手入口) and
[skills.md](skills.md) (技能注册表).

## 当前状态

- **decision 侧闭环已完成并自验通过**：vision 结果订阅 → 行为树分发 → control 任务下发，
  12 库位蛇形巡视（右下出发：`4,3,2,1 → 5,6,7,8 → 12,11,10,9`）。
- **Gazebo 仿真（M2）已跑通完整搬运**：12 库位货架 + 四类缺陷铁片（动态）+ 三轴龙门 +
  俯视相机 + 4 个分类码放格。夹持用 `DetachableJoint`，HANDLE 走「下探 → 夹 → 抬 →
  搬到该类型的格 → 触底放下 → 松 → 抬」。
  真视觉一轮 **12/12 四类全对**，且 **12 块铁片全部落到对应类型的格子**
  （每型 3 块叠放，层高差实测正好等于片厚 6 mm）。单点约 6.3 s，整轮约 80 s。
  详见 `docs/guides/simulation_guide.md`。
- **传统视觉基线已可用**：`src/vision/aic_vision_trad` 读 `/camera/image`，
  仿真一轮 **12/12 四类全对**（`./scripts/run_sim.sh --vision trad`）。
- vision / control 两个真机模块本体仍为**实现留白**，接口契约见 `docs/guides/`。

## 快速开始

依赖：ROS 2 Humble（宿主机 `/opt/ros/humble`，含 BehaviorTree.CPP 4.10）
+ Ignition Gazebo 6（Fortress，仿真用）。

```bash
# 1. 编译（-pipe 规避 /tmp 过小导致的编译失败，脚本里已处理）
./scripts/build.sh

# 2. 端到端跑仿真：Gazebo 场景 + 龙门 + 视觉桩 + 决策行为树（跑完一轮自动退出）
./scripts/run_sim.sh            # 加 --gui 看画面；--drop-rate 0.3 注入识别失败

# 3. 不带仿真的决策自验：假 vision + 任务话题记录 + CSV 断言，两个场景
#    A=正常 12 位点；B=每 5 条注入一条非法值（验证 TIMEOUT 兜底不中断）
./scripts/verify_decision_loop.sh        # 结果落 logs/verify_full_run.log

# 3.5 仿真搬运自验（M2）：真视觉跑一整轮，断言 12/12 + 夹放各 12 次 + 12 块料落对格
./scripts/verify_sim_m2.sh               # 约 100 s；--vision gt 可排除视觉因素

# 4. 单独起决策节点（真 vision / control 接入后同样这么起）
./scripts/run_decision.sh
```

自验工具（开发期桩，不进比赛提交物）：

| 脚本 | 用途 |
|---|---|
| `scripts/fake_vision_pub.py` | 假 vision：收到 GOTO 后回 `"1".."4"`，可注入非法值 / 静默 |
| `scripts/record_task_topic.py` | 记录 `/control/material_task`，导出 CSV 与汇总 |
| `scripts/check_decision_run.py` | 断言：位点顺序 / 类型序列 / GOTO·HANDLE 条数 / TIMEOUT 数 |
| `scripts/verify_sim_m2.sh` | 跑一整轮真视觉搬运，再调下面的断言脚本 |
| `scripts/check_sim_m2.py` | 断言：结果 12/12、夹放各 12 次、12 块料 x/y/z 落位 |

## 目录

```text
src/
├── plugins/aic_msgs/              MaterialTask.msg（决策→control 任务契约）
├── decision/aic_warehouse/        3x4 货架模型：蛇形顺序生成 + gtest + print_path
├── decision/aic_behavior_tree/    决策核心：VisionClient / ControlBridge / 8 节点 / bt_runner
├── simulation/aic_gazebo_sim/     Gazebo 场景（货架+铁片+三轴龙门+码放格）+ 控制/视觉仿真实现
├── vision/aic_vision_trad/        传统视觉：盘面定位 + 四类缺陷判据（仿真 12/12）
├── control/  ...                  其余各域留位（见 AGENTS.md）
docs/
├── guides/decision_vision_interface.md   vision ↔ decision 契约
├── guides/decision_control_interface.md  decision → control 契约（control 接入清单）
├── guides/decision_bt_architecture.md    行为树结构 / 黑板键 / 改树前必读的坑
├── guides/simulation_guide.md            仿真：场景几何 / 话题 / 夹持搬运 / 20 个坑
└── adr/0001-btcpp-decision-stack.md      选型记录
config/  data/  docker/  tools/           留位
```

## 常用参数

话题与超时都在 `src/decision/aic_behavior_tree/config/aic_decision_params.yaml` 改，
不碰代码；行为树 XML 见 `aic_behavior_tree/config/trees/aic_sort.xml`
（改树前先读 `docs/guides/decision_bt_architecture.md` 第 3 节的三个坑）。
