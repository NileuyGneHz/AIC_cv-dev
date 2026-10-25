# AIC_cv-dev
bilibili: @珐儿姐  广西大学@robotz

第八届全球校园人工智能算法精英大赛 · 算法应用赛 · 视觉逻辑算法应用赛
（视觉识别 → 坐标解算 → 运动控制）。

团队工作约定见 [AGENTS.md](AGENTS.md)，技能注册表见 [skills.md](skills.md)。

## 当前状态

| 模块 | 状态 | 证据 |
|---|---|---|
| 决策（行为树） | 完成 | 12 库位蛇形巡视，`bt_runner` + 8 节点，见 `docs/guides/decision_bt_architecture.md` |
| Gazebo 仿真（M5） | 完成 | 生成物料（底板下停料）→ 置物台识别 → 单立柱龙门吸盘取料 → 垂直货架 4 列×3 层（放料后盘侧加绿色指示环）；真视觉整轮 **12/12**、落位 **0.3 mm**、约 145 s |
| 传统视觉 | 完成 | 盘面定位 + 四类缺陷判据（划痕 / 污渍 / **破损** / 孔洞，规则称缺角），仿真 12/12；`src/vision/aic_vision_trad` |
| 判分看板 | 完成 | 纯旁路 HTTP :8080；按官方控制窗口分 6 区：全景 + 库位监视/放料位/态势图三联，右侧状态、XYZ 手动、抓取/放置/生成物料/拍照、日志、重置/退出 |
| PLC 网关 | 完成（Modbus 已实测 / RSC 干跑） | `aic_plc_gateway`；接管整轮 12/12、约 130 s |
| 真机执行机构 | 未接 | 需串口/CAN 驱动替换 `gantry_controller` 的执行层 |
| 海康相机 | 未接 | 需相机或 aarch64 版 MVS SDK |
| PLCnext vPLC 现场联调 | 未做 | 本机无 vPLC；版本与部署见 `docs/guides/vplc_deployment.md`，标签映射见 `plcnext_rsc_setup.md` |
| vPLC 算法镜像（arm64） | 已就绪未构建 | `deploy/vplc/`（Dockerfile + requirements + 手册）与无 ROS 入口 `run_to_plc.py` 已落地；需在有设备的环境 `podman build` 并探明官方 `Cam` 变量路径 |

## 省赛提交状态（截止 2026-11-02）

| 提交项 | 状态 | 说明 |
|---|---|---|
| 参赛承诺书 | **已提交**（2026-10-22 确认） | 全员手写签字版 |
| 技术文档（PPT） | 已完成 | `docs/competition/AIC视觉分拣技术文档/`，13 页，pptx + pdf |
| 程序源代码 | 就绪待打包 | 仓库即源码；`./scripts/package_submission.sh` 一键出 zip（`git archive`，只含被跟踪文件，3.4 MB） |
| 竞赛任务完成视频 | 未录 | 看板窗口一镜到底，约 80–100 s，1080P；步骤见 `docs/guides/obs_recording_playbook.md` |
| 参赛视频（固定机位） | 未录 | 手机/相机固定三脚架拍选手 + 屏幕全景 |
| 视觉识别图集 | **已生成** | `视觉识别图集.png`（4 类 × 3，原图/结果对照，中文标注）+ 特征表 CSV；脚本 `make_submission_atlas.py` 可复算 |

行动清单与录像硬要求见 `docs/competition/platform_and_video_checklist.md`。

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

## 省赛快速检验（Gazebo GUI + 判分看板）

> 目标：一条命令起齐「Gazebo 窗口 + 真视觉 + 决策 + 判分看板」，
> 浏览器看比分，30 秒判断链路好不好。整轮约 145 s（12 块料），跑完自动退出。

**0. 前置（只做一次）**

```bash
cd ~/AIC_cv-dev
./scripts/build.sh          # 编译；没有 install/ 后面全都起不来
```

**1. 起齐全部（一条命令，不要拆成两条）**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export PATH=/usr/bin:$PATH     # 坑：不设这条，视觉节点会因 python3 缺 cv2 起不来，
                              #      表现为「等待视觉结果超时」而不是导入报错
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad gui:=true
```

随后应出现：Gazebo 窗口（场景）、启动终端（日志）、浏览器 <http://127.0.0.1:8080>（看板）。
看板**左上角**应有一块 4×3 的「库位监视」小画面（12 个格口各一只隐藏相机），
每格边框颜色 = 该格物料状态（灰 空 / 蓝 台面 / 琥珀 抓取 / 绿 已放）。
**GUI 窗口会晚 3–5 s 才出来**——它是独立客户端，要先等仿真服务端把 `/world/` 发布出来。
另开一个终端 `ros2 node list`，应看到 6 个节点：`gz_bridge`、`gz_camera_bridge`、
`gantry_controller`、`vision_trad_node`、`aic_decision_bt`、`aic_score_node`。

> **为什么 GUI 要单独起（2026-10-22 修的一个重要坑）**
> 旧写法 `ign gazebo -r world.sdf` 把服务端和 GUI 塞进同一个进程，**相机传感器与 GUI 视口
> 共用同一个渲染线程**。人盯着浏览器看板时，Gazebo 窗口被挡在后面（或最小化/失焦），
> 那一路渲染被节流 → `/camera/image` 的帧又稀又不规律 → 看板上画面一抽一抽、
> 视觉 1.2 s 内等不到新帧 → 每个位点都 TIMEOUT → **判分格几乎全红**。
> 现在服务端永远 `-s --headless-rendering`，GUI 只用 `ign gazebo -g` 连过来，
> 画得慢或被遮挡都不再影响相机。细节见 `docs/guides/simulation_guide.md` 坑 30。

> **坑：不要再另开终端跑 `./scripts/run_sim.sh --gui`。** 看板这条命令**已经包含仿真**，
> 两个仿真实例会抢同一批关节指令——RTF 掉到 1/10、「到位超时 8.0 s」、
> 单轮从 10 s 变 36 s，而两边都显示正常。「GUI + 看板」只用这一条命令；
> 只想跑仿真、不要看板时，才用 `./scripts/run_sim.sh --gui`。
> `gui:=true` 需要本机有图形会话（远程 SSH 加 `-X` 能出窗口但很卡）。

**2. 看什么（30 秒判定）**

| 看哪 | 正常 | 异常意味着 |
|---|---|---|
| Gazebo 窗口 | 来料架 3×4 铺满 12 块料；龙门逐块「上料→拍照→吸取→码放」；货架 12 格被填满 | 时钟在走但物体不动 → 物理引擎插件没加载（launch 已显式补路径，重起即可） |
| 看板库位格 | 每放入一格变绿；最终 12 格全绿、比分 12 / 0、事件流水无 TIMEOUT | 出现红/琥珀 → 识别失败或超时，看流水里对应位点 |
| 终端日志 | 每轮一条 `[vision] 判定 …（r=…px）`；无 Traceback | 视觉节点没起 → 回到第 1 步的 `export PATH` |
| 耗时 | 整轮约 145 s，跑完自动退出（`once:=true` 默认） | 超过 4 分钟 → 机器慢或 RTF 掉，先 `sim_kill_stale` 再重跑 |

常用变体：`once:=false auto_restart:=true` 常驻多轮（Ctrl-C 结束）；
`vision_backend:=gt` 换真值桩（排除视觉因素，定位控制/物理问题）；
`--domain auto` 式隔离见 `scripts/run_sim.sh`（同机跑两套仿真时必加）。

**3. 收尾与排障**

- 正常收尾：`once:=true` 跑完自动关；中途停直接在启动终端 Ctrl-C。
- **判分格几乎全红**：先在日志里搜「没有等到可用新帧」。命中说明相机掉帧（不是判据错），
  按坑 30 检查 GUI 是否被遮挡/最小化；`gui:=false` 跑一遍若恢复 12/12，即可确认。
  没有这条告警才是真误判，去查 `[vision] 判定 …` 的特征值与 `defect_classifier.py` 阈值。
- 卡住 / 怀疑双实例：`source scripts/lib/sim_cleanup.sh && sim_kill_stale`
  （坑：`pkill -f gzserver` 在 Fortress 上匹配不到进程，进程名是 `ign gazebo`）。
- 权威自验（约 100 s，含 12 行全 OK + 夹持/释放各 12 次 + 落位 4 mm 核对）：
  `./scripts/verify_sim_m2.sh --domain auto`。
- 想录省赛视频：`docs/guides/obs_recording_playbook.md`（OBS 参数 + 分镜 + 双机位）。

## 目录

```text
src/                                 7 个可构建包，没有只放 .gitkeep 的占位目录
├── plugins/aic_msgs/                MaterialTask.msg（决策→control 任务契约）
├── plugins/aic_dashboard/           判分看板（旁路，不参与控制）
├── plugins/aic_plc_gateway/         PLC 网关：Modbus TCP 从站 / PLCnext RSC 双后端
├── decision/aic_warehouse/          3x4 货架模型：蛇形顺序生成 + gtest
├── decision/aic_behavior_tree/      决策核心：VisionClient / ControlBridge / 8 节点 / bt_runner
├── simulation/aic_gazebo_sim/       场景生成 + 龙门控制 + 视觉桩 + 抓帧/快照工具
└── vision/aic_vision_trad/          传统视觉：盘面定位 + 四类缺陷判据 + run_to_plc.py（真机无 ROS 入口）

deploy/vplc/                          真机算法镜像：arm64 Dockerfile + requirements + 部署手册

docs/
├── guides/acceptance_manual.md      验收手册（五级阶梯 + 实测基线 + 排障）
├── guides/simulation_manual.md      仿真操作手册（参数、模式、产物）
├── guides/simulation_guide.md       原理与 29 个坑（改仿真前必读）
├── guides/plc_integration.md        PLC 集成指南（寄存器表、握手、排障）
├── guides/plcnext_rsc_setup.md      RSC 标签映射与现场联调五步
├── guides/vplc_deployment.md        VC 版接入：版本表、arm64 镜像、podman 部署
├── guides/competition_host.md       比赛上位机（Ubuntu）改造清单与风险
├── guides/windows_plc_handover.md   Windows 侧 PLC 开发交接（环境/自检/待办/坑）
├── guides/handover_ubuntu.md        Windows → Ubuntu 仿真交接
├── guides/vn_mock_design.md         VirtualNext mock 平台设计（协议层）
├── guides/obs_recording_playbook.md 省赛视频录制（OBS）
├── guides/decision_*.md             三份接口契约
├── plc/main.st                      PLC 侧巡视状态机骨架（ST）
├── plc/windows_dev_notes.md         Windows 环境实测记录（pip 镜像 / git refs 异常）
├── adr/                             选型记录（BT.CPP / PLC 契约）
└── competition/                     技术文档 PPT、平台对标清单与素材

scripts/                              构建 / 运行 / 自验 / PLC 测试 / 提交物打包
deploy/vplc/                          真机算法镜像（arm64）
logs/                                 所有运行产物（不入 git）
```

**空目录一律不留。** 真要加包时按 `src/<域>/<包名>` 新建并带 `package.xml`；
只有 `.gitkeep` 的目录既不能被 colcon 构建，又会让人误以为已有实现
（2026-10-21 已清理 `src/` 下 8 个占位，2026-10-22 清掉根目录
`config/` `data/` `docker/` `tools/` 4 组共 7 个）。规划写在
`AGENTS.md` §5 的「后续要加的包」表里。

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
