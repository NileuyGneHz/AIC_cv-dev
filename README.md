# AIC_cv-dev

第八届全球校园人工智能算法精英大赛 · 算法应用赛 · 视觉逻辑算法应用赛
（视觉识别 → 坐标解算 → 运动控制）。

团队工作约定见 [AGENTS.md](AGENTS.md)，技能注册表见 [skills.md](skills.md)。

## 当前状态

| 模块 | 状态 | 证据 |
|---|---|---|
| 决策（行为树） | 完成 | 12 库位蛇形巡视，`bt_runner` + 8 节点，见 `docs/guides/decision_bt_architecture.md` |
| Gazebo 仿真（M4） | 完成 | 来料架 3×4 → 置物台识别 → 吸盘取料 → 分拣货架 4 列×3 层；真视觉整轮 **12/12**、落位 **0.0 mm**、约 145 s |
| 传统视觉 | 完成 | 盘面定位 + 四类缺陷判据，仿真 12/12；`src/vision/aic_vision_trad` |
| 判分看板 | 完成 | 纯旁路 HTTP :8080，MJPEG + 态势图 + 逐位点判分 |
| PLC 网关 | 完成（Modbus 已实测 / RSC 干跑） | `aic_plc_gateway`；接管整轮 12/12、约 130 s |
| 真机执行机构 | 未接 | 需串口/CAN 驱动替换 `gantry_controller` 的执行层 |
| 海康相机 | 未接 | 需相机或 aarch64 版 MVS SDK |
| PLCnext vPLC 现场联调 | 未做 | 本机无 vPLC；版本与部署见 `docs/guides/vplc_deployment.md`，标签映射见 `plcnext_rsc_setup.md` |
| vPLC 算法镜像（arm64） | 未做 | 需 `deploy/vplc/`（Dockerfile + `run_to_plc.py`），依赖真机探明官方 `Cam` 变量路径 |

## 快速开始

依赖：ROS 2 Humble（`/opt/ros/humble`，含 BehaviorTree.CPP 4.10）
+ Ignition Gazebo 6（Fortress）。

```bash
cd ~/AIC_cv-dev
./scripts/build.sh                 # 编译（已用 -pipe 规避 /tmp 过小）

# 一键验收：构建 → 协议层 → RSC 干跑 → 仿真基线（约 3 分钟）
./scripts/verify_all.sh
./scripts/verify_all.sh --with-plc # 追加 PLC 接管整轮（约 6 分钟）

# 看画面：判分看板（浏览器 127.0.0.1:8080）
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad

# 直接看 Gazebo 窗口（需要本机有图形会话）
./scripts/run_sim.sh --gui
```

验收判据、逐级耗时与排障顺序见 **`docs/guides/acceptance_manual.md`**——想知道
「现在到底好不好、坏了先看哪里」，只看那一份就够。

## 目录

```text
src/                                 7 个可构建包，没有只放 .gitkeep 的占位目录
├── plugins/aic_msgs/                MaterialTask.msg（决策→control 任务契约）
├── plugins/aic_dashboard/           判分看板（旁路，不参与控制）
├── plugins/aic_plc_gateway/         PLC 网关：Modbus TCP 从站 / PLCnext RSC 双后端
├── decision/aic_warehouse/          3x4 货架模型：蛇形顺序生成 + gtest
├── decision/aic_behavior_tree/      决策核心：VisionClient / ControlBridge / 8 节点 / bt_runner
├── simulation/aic_gazebo_sim/       场景生成 + 龙门控制 + 视觉桩 + 抓帧/快照工具
└── vision/aic_vision_trad/          传统视觉：盘面定位 + 四类缺陷判据

docs/
├── guides/acceptance_manual.md      验收手册（五级阶梯 + 实测基线 + 排障）
├── guides/simulation_manual.md      仿真操作手册（参数、模式、产物）
├── guides/simulation_guide.md       原理与 29 个坑（改仿真前必读）
├── guides/plc_integration.md        PLC 集成指南（寄存器表、握手、排障）
├── guides/plcnext_rsc_setup.md      RSC 标签映射与现场联调五步
├── guides/vplc_deployment.md        VC 版接入：版本表、arm64 镜像、podman 部署
├── guides/competition_host.md       比赛上位机（Ubuntu）改造清单与风险
├── guides/windows_plc_handover.md   Windows 侧 PLC 开发交接（环境/自检/待办/坑）
├── guides/obs_recording_playbook.md 省赛视频录制（OBS）
├── guides/decision_*.md             三份接口契约
├── plc/main.st                      PLC 侧巡视状态机骨架（ST）
├── adr/                             选型记录（BT.CPP / PLC 契约）
└── competition/                     技术文档 PPT 与素材

scripts/                              构建 / 运行 / 自验 / PLC 测试入口
config/  data/  docker/  tools/        配置、样本、镜像、开发工具
logs/                                 所有运行产物（不入 git）
```

**后续扩展位不建空目录。** 真要加包时按 `src/<域>/<包名>` 新建并带 `package.xml`；
只有 `.gitkeep` 的目录既不能被 colcon 构建，又会让人误以为已有实现
（2026-10-21 已清理 8 个此类占位）。

## 常用入口

| 想做什么 | 命令 |
|---|---|
| 确认全部正常 | `./scripts/verify_all.sh --with-plc` |
| 看仿真跑一轮 | `./scripts/run_sim.sh --gui` |
| 看判分与画面 | `ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad` |
| 单独验 PLC 接管 | `bash scripts/test_plc_one.sh` / `_full.sh` |
| 验 RSC 后端（无 vPLC） | `bash scripts/test_plc_rsc.sh` |
| 调窗口 / 看画面 | `docs/guides/visual_walkthrough.md` |
| 录省赛视频 | `docs/guides/obs_recording_playbook.md` |

参数在 `src/decision/aic_behavior_tree/config/aic_decision_params.yaml` 与
`src/simulation/aic_gazebo_sim/config/sim_layout.yaml`（场景几何唯一数据源）；
行为树 XML 见 `aic_behavior_tree/config/trees/aic_sort.xml`。
