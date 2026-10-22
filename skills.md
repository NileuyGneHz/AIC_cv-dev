# skills.md — AIC_cv-dev 技能注册表

> 与 `AGENTS.md` 配套的本区「技能」唯一索引：① 能力域分解；② 已沉淀 / 计划沉淀的 Agent 技能；
> ③ 可复用既有资产；④ 学习缺口；⑤ 维护规则。
> 单个技能的正式定义（`SKILL.md`）在技能开工时落盘（`.workbuddy/skills/<name>/`），
> 本表是其唯一登记处；状态流转：`规划` → `在建` → `可用`。
>
> **开发环境（2026-10-15 变更）**：主力开发转向 Windows——官方「视觉逻辑算法应用仿真平台」
> 是 Windows 程序，线上录制必须在 Windows 宿主跑；已自验的决策链（ROS 2 Humble +
> BehaviorTree.CPP 4.10）保留，迁入 **WSL2 Ubuntu 22.04**，与宿主仿真平台走 localhost 通信。
> 原 Ubuntu 宿主机方案降级为参考。`F:\AIC_cv-dev` 是 Windows 侧工作副本（repo：
> NileuyGneHz/AIC_cv-dev）。

## 1. 比赛能力需求 → 技能域分解

| # | 技能域 | 比赛中的用途 | 当前基础 | 优先级 | 阶段 |
|---|---|---|---|---|---|
| 1 | 缺陷检测（分类 / 分割） | 识别划痕 / 污渍 / 缺角 / 孔洞 | 样本形态已探明（2025 缺陷图纸 48 例：白底浅灰圆片，缺角=边缘缺口、孔洞=内部挖空、划痕=细线、污渍=黑色实心，**存在混合缺陷**）；训练集来源未定 | P0 | 省赛前 |
| 2 | 图像预处理与数据集 | 光照 / 定位归一、样本组织与增强 | OpenCV 可用（推断）；仿真图可从平台 `received_images/` 导出 | P0 | 省赛前 |
| 3 | 手眼标定与坐标解算 | 图像坐标 → 抓取坐标 | 坐标变换 / PnP 概念有（推断）；**仿真线无需标定**（平台坐标直读），仅国赛真机线需要 | P2 | 国赛前 |
| 4 | 三轴运动控制 | 抓取 / 码放动作编排 | 契约已定（`MaterialTask`：GOTO / HANDLE / HOME）；VirtualNext 轴参数已探明（X/Y 0–300、Z 0–200，速度 1–5，伺服 / IO 变量表见 `aic-official-sim`） | P1 | 国赛前 |
| 5 | 通信对接 | PC ↔ 平台 / 控制模块 | **官方协议已完全探明**（2026-10-16）：VirtualNext 双框架 TCP socket 字符串协议 `Camera:Trigger/Start/Result` + 错误码 −1…−4；真机线 PROFINET + PLCnext（371 页手册已到手） | P1 | 省赛前 |
| 6 | 任务调度与状态机 | 整流程编排、异常兜底、循环执行 | **已落地**：BT 决策闭环自验通过（2026-10-15，场景 A 正常 / B 抗扰动）；**待决策**：ROS 2 BT 链与官方 Python/C# 框架的对接架构（见第 4 节缺口 4） | P0 | 骨架完成 |
| 7 | 官方仿真与录制 | 平台操作、WSL2↔宿主桥接、录屏规范 | 平台使用说明书已到手，通信协议 / 变量表 / 摆放计分规则全部探明；平台软件未购买安装（550 元、有效期 2025-12-31 为**上届口径**）。**2026-10-19 官方答疑：非必须购买，同类型数字孪生平台可拍视频，但须满足比赛文件要求（待书面留证）** → 自建 Gazebo 路线转为可行，落地清单见 `aic-official-sim` §6.1 | P0 | 省赛前 |
| 8 | 提交物打包 | PPT / 源码 / 视频 / 图集整理与自检 | **已明确**：评分 20（源码）/ 60（视频）/ 20（答辩）；四件套 + 百度网盘命名规则；承诺书需手写签字 | P0 | 省赛前 |

## 2. Agent 技能

| 技能名 | 触发场景 | 覆盖内容 | 状态 |
|---|---|---|---|
| `aic-decision-loop` | 「构建 / 自验决策节点」「接入真 vision 或 control」「改行为树 / 扩子树」 | build / verify / run 命令、BT.CPP 四个实测坑（while 装饰器 haltChild、tick 重入快照、hasNext 吞尾、SubTree 黑板隔离）、vision / control 接口契约、扩展工作流、WSL2 环境要点 | **可用**（闭环自验通过；SKILL.md 已落盘） |
| `aic-official-sim` | 「官方仿真平台」「VirtualNext」「录制省赛视频」 | **平台协议事实已全部落盘**（SKILL.md 2026-10-16）：双框架 TCP 架构、字符串协议与错误码、伺服 / IO 变量表、RunCycle 流程、摆放计分规则、界面操作、录屏规范；平台安装激活待购买后补 | **在建**（SKILL.md 已落盘） |
| `aic-vn-mock` | 「没买平台也要联调」「mock VirtualNext」 | 按官方协议实现 mock 服务器（`Camera:Trigger/Start/Result` + 图像下发 + 伺服变量仿真），替代 `fake_vision_pub.py` 升级为协议级联调；**设计定稿见 `docs/guides/vn_mock_design.md`**（MockCamera / MockPlc / SimCore / OpenCV 2D Viewer，2.5–3 天工作量） | **设计定稿**（2026-10-19；SKILL.md 已落盘，实现待排期） |
| `aic-defect-vision` | 「训练 / 调优缺陷识别」「评估模型」 | 四类缺陷样本形态特征（48 例图纸分析）、传统 / 深度方案取舍、**混合缺陷判据**、对接 `process_image` 整数结果契约与 `/vision/material_type`（`"1".."4"`、QoS reliable）、视觉识别图集产出规范 | **在建**（SKILL.md 已落盘） |
| `aic-submission-pack` | 「打包提交」「整理材料」「答辩」 | 评分细则（源码 20 / 视频 60 / 答辩 20）、四件套与百度网盘命名规则、视频录制硬要求、答辩 5–10min 结构、承诺书签署清单 | **在建**（SKILL.md 已落盘） |
| `aic-handeye-calib` | 「标定相机和机械臂」 | 标定板采集、解算脚本框架、参数落盘与验证。**仿真线无需**（VirtualNext 坐标直读），仅国赛真机线启用 | 规划（国赛真机线） |
| `aic-plc-comm` | 「PC 和 PLC 通信」「PLCnext」「PyPlcnextRsc」 | **省赛线确认不需要**：VirtualNext 已内置三轴运动仿真 + C# 运动控制框架，无真实 PLC。转国赛真机线，SKILL.md 已落盘（2026-10-16）：PyPlcnextRsc 变量读写（RSC 服务）、PLCnextBase 库导入、海康相机容器化 find-no-device 排查清单、手册 / 快速入门指南章节索引 | **在建**（SKILL.md 已落盘） |
| `aic-plc-gateway` | 「接 PLC」「Modbus 不动」「RSC 联调」「握手超时」 | Modbus TCP / PLCnext RSC 双后端、`IoBackend` 切法、四层验证阶梯（协议层→进程内整轮→跨进程端到端→真机）、三类假象的定位顺序（来自上游 2026-10-21 提交） | **可用**（仿真侧；真 vPLC 未联调） |
| `aic-gazebo-sim` | 「仿真不动」「相机没图」「改 Gazebo 场景」「夹不住/放不下」 | Fortress + ROS 2 仿真骨架、世界生成、ros_gz_bridge 方向（含关节反馈桥）、到位判定闭环、DetachableJoint 夹持与码放、20 个坑（含视觉）、落位核对探针（来自上游 2026-10-21 提交） | **可用**（仿真侧） |
| `aic-motion-3axis` | 「三轴 / 抓取动作调试」 | 坐标系与行程定义、点位示教、回零 / 限位安全规范；命令语义对齐 `MaterialTask`（GOTO / HANDLE / HOME），slot→物理坐标表由 control 侧持有；VirtualNext 侧对齐伺服变量（`PositionSetting` / `Arrived` / `SuctionEnable` 等） | 规划（国赛真机线） |

> 2026-10-15 变更：原 `aic-sim-capture` 更名 `aic-official-sim`，职责从「录制规范」扩展到
> 「平台接口探测 + WSL2 桥接」；新增 `aic-decision-loop`（首个可用技能）。
> 2026-10-16 变更：第八届官方文档到手（赛题规则 / VirtualNext Smart 说明书 / 缺陷图纸 48 例 /
> PLCnext 手册 / 承诺书），协议与评分事实全部落技能；补登 `aic-vn-mock`（2026-10-15 三版
> 修订记录已提及但表内遗漏）；`aic-plc-comm` 依据说明书「平台内置运动控制框架」判定省赛线
> 不需要，降为国赛真机线；`aic-official-sim` / `aic-defect-vision` / `aic-submission-pack`
> 转「在建」并落盘 SKILL.md。

## 3. 可复用既有资产

| 来源 | 资产 | 在 AIC 的用途 |
|---|---|---|
| 本仓库（已自验） | `src/decision/aic_behavior_tree` 决策闭环 | 比赛核心骨架；`aic-decision-loop` 技能的载体 |
| 本仓库（已自验） | `src/decision/aic_warehouse` 布局生成器 + gtest | 布局变化改 YAML 不改代码（`warehouse_layout.yaml`）；12 库位 = 4 列 × 3 行，**列对应缺陷类**与官方摆放规则一致 |
| 本仓库（已自验） | `scripts/verify_decision_loop.sh` + 3 个自验桩（fake_vision / recorder / checker） | 决策侧任何改动后的回归自验；`aic-vn-mock` 是 fake_vision 的协议级升级线 |
| 本仓库 | `docs/guides/` 三份接口 / 架构文档 | vision / control 接入时的契约唯一来源 |
| **官方文档**（2026-10-16 全套归档 `docs/competition/`，共 10 件） | ①《视觉逻辑算法应用赛规则-线上》docx（第七届 2025-05 模板）②《VirtualNext Smart开发平台使用说明》pdf ③《缺陷图纸示例 2025年》pdf（48 例）④《PLCnext Engineer基础应用手册2024_V1.0》pdf（371 页）⑤《1020_关于智能控制与深度学习软件平台发布的通知》pdf ⑥《参赛选手承诺书》docx（第八届）⑦《PLCnext_使用python建立变量连接的方法》pdf（PyPlcnextRsc）⑧《PLCnext_快速入门指南2025_V1.0》pdf（413 页）⑨《第七届总决赛通知-251126》pdf（扫描版 11 页，已视读）⑩《PLCnextBase_1.6.2.0》zip（功能块库） | 协议 / 评分 / 样本 / 控制器 / 国赛流程的权威事实来源，已提炼进各技能 SKILL.md |
| 插件级技能（非用户级） | `agent-browser`（浏览器自动化）、`tencent-pptx` / `tencent-docx`（文档生成） | 查官网下载资料、省赛技术文档与答辩 PPT |
| rm_nav-dev（只读） | `bt-*`、`ros2-topic-bridge-verify` 等技能、`.agents/skills/` 与 `docs/agents/` | BT 与话题联调经验、工程化流程借鉴（引入时先拷贝再裁剪） |
| EC_ws（只读） | STM32 工具链 / 烧录调试技能 | 若国赛涉及下位机 / 板卡再引入 |

> 修正（2026-10-15）：原表登记为「用户级技能」的 `docker-pull-stall-cn`、`rosdep-cn-mirror-fix`
> 等，经核查 `~/.workbuddy/skills/` 下**并不存在**（该目录当前为空），已删除；
> `agent-browser` 实为插件级技能，登记口径已更正。

## 4. 学习缺口（补技能线）

1. **官方仿真平台**：协议已完全探明（见 `aic-official-sim`）。**2026-10-19 官方答疑推翻
   「必须购买」的旧判断**：非必须购买，同类型数字孪生平台可拍视频、但视频须满足比赛文件
   要求——该口径不在说明书内（说明书 9 页逐字核对无此内容），**必须留存截图并争取群公告**；
   **2026-10-21 江苏赛区答疑（队长群，负责人「沉寂」19:19/19:20 复读）**：问「赛区制到底
   能不能用其他仿真平台进行作答」→ 答「可以」，并提及「江苏赛区线下赛」。**两条新信息**：
   ① 自带平台口径扩展到赛区/区域赛；② 江苏赛区区域赛是**线下**——与 AGENTS.md「省赛
   （线上）」口径的关系待向队长确认（线下是否用赛场统一设备、自带平台以什么形式参与）。
   留证状态：仅有群聊文字 + 官方二维码图，**还没有完整聊天截图归档**（存 `docs/competition/`）。
   剩余未知：平台购买与激活（550 元 / 2025-12-31 为上届口径）、防火墙端口、
   `process_image` 返回整数的类别映射语义（推断 1–4 对应四列缺陷）；
2. **WSL2 环境落地**：Ubuntu 22.04 + ROS 2 Humble 安装、与宿主 localhost 互通验证；
   colcon 在 `/mnt/f`（9P 文件系统）上构建慢的坑——建议 WSL 内部 clone 构建、Windows 侧
   只做编辑，经 git 同步（**推断，待实测**）；
3. 缺陷检测数据集：四类 + **混合缺陷**形态已明确（48 例图纸）；样本来源候选——
   VirtualNext `received_images/` 导出、缺陷图纸合成增强；多缺陷共存时的分类优先级规则待定；
4. **架构对接决策（新增，最高优先）**：比赛主链是官方 Python 视觉框架（改 `process_image`）
   + C# 运动框架（改 `RunCycle()`），走 TCP 字符串协议；现有 ROS 2 BT 决策链如何接入——
   方案 A：写 TCP 桥节点（vision / control 侧各一），保留 BT 资产与自验体系；
   方案 B：调度逻辑直接进官方框架，BT 降为设计文档与答辩素材。**需定案后再写对接代码**；
5. 工业相机（**海康 MVS 已由往届报告证实**）与手眼标定完整实操；容器内部署坑已有实战记录
   （EPC 边缘控制器内 "find no device!"，排查清单见 `aic-plc-comm` §4）——国赛真实设备线；
6. Podman 容器内运行识别程序：赛题规则文档确认「软件部分基于 Podman 完成部署」
   （第七届口径），容器化基本坐实，待平台到手验证构建流程；
7. **新增提交物：视觉识别图集**（jpg / png，单独立项提交）——从第一次仿真跑通起就
   随手攒图（原始图 + 检测结果图对照），勿到赛前补做；承诺书需全体成员手写签字；
8. **国赛参考（第七届总决赛通知 251126，已视读）**：总决赛 2025-12-06/07 在**北京邮电大学
   （海淀校区）**——此前 temp.md「南京」口径有误已修正；视觉逻辑算法应用赛 12-06 线下进行
   （队长 QQ 群 831738277），报到当天交《总决赛安全承诺书》、全员需购出行保险、
   交通食宿自理。本届（2026-11 月底）时间地点以第八届通知为准。

## 5. 维护规则

1. 先改本表、再动 `SKILL.md`（本表是唯一登记处）。
2. 本区专属技能用 `aic-` 前缀 + kebab-case；通用技能不加前缀。
3. 「规划 → 可用」的判据：按 `SKILL.md` 流程完成一次真实任务且无翻车。
4. **环境边界**：技能中的可执行命令默认在 WSL2 Ubuntu 22.04 内执行；Windows 宿主只跑
   官方仿真平台与录制；两侧经 localhost 通信。技能落盘位置
   `.workbuddy/skills/<name>/SKILL.md`（已被 `.gitignore` 排除，不入仓库；如需团队共享再议）。
5. 赛季结束后：可复用技能上收用户级；赛事信息（时间线等）回写 `AGENTS.md`。

> 修订记录：2026-10-14 初版（`~/Desktop/temp.md` + rm_nav-dev 正式文件 + AIC 公开资料整理）；
> 2026-10-15 二版（决策闭环落地 → 新增 `aic-decision-loop` 可用；开发环境转向 Windows/WSL2；
> `aic-sim-capture` 更名 `aic-official-sim` 并扩展职责；修正过时的用户级技能登记）；
> 2026-10-15 三版（VirtualNext Smart 手册到手 → 协议事实写入 `aic-official-sim`；新增
> `aic-vn-mock` 平替技能规划）；
> 2026-10-16 四版（第八届官方文档全套到手并完成提取：赛题规则 / VirtualNext Smart 说明书 /
> 缺陷图纸 48 例 / PLCnext 手册 / 1020 通知 / 承诺书——通信协议、伺服 / IO 变量表、摆放
> 计分规则、评分 20/60/20、四件套与网盘命名、缺陷形态特征全部落库；补登 `aic-vn-mock`；
> `aic-plc-comm` 判定省赛线不需要、降国赛真机线；`aic-handeye-calib` 同步标注仅真机线；
> `aic-official-sim` / `aic-vn-mock` / `aic-defect-vision` / `aic-submission-pack` 转「在建」
> 并落盘 SKILL.md；新增学习缺口 4「架构对接决策」与 7「视觉识别图集」）；
> 2026-10-16 五版（官方文档全套 10 件归档 `docs/competition/`；新增文档提取——PyPlcnextRsc
> Python 变量连接（真机通信选型首选）、容器内海康相机 find-no-device 实战坑、第七届总决赛
> 通知视读（12-06/07 北邮，修正「南京」旧口径）、PLCnextBase 库与快速入门指南 413 页）；
> `aic-plc-comm` 转「在建」并落盘 SKILL.md（真机通信 + 容器部署坑）。
> 2026-10-22 八版（拉取上游 bd265cb 并合并本地改动；江苏赛区 10-21 答疑「可以用其他
> 仿真平台作答」+「江苏赛区线下赛」补入缺口 1，线下赛形式待确认）。
> 2026-10-19 七版（官方答疑：比赛非必须购买平台，同类型数字孪生平台可拍视频、但须满足
> 比赛文件要求——口径不在说明书内，需留证；`aic-official-sim` 增 §6.1 自建平台录制落地
> 清单，`aic-submission-pack` 增第三方平台录制条款与自检项；域 7 与缺口 1 同步修正；
> `aic-vn-mock` 与 `docs/guides/vn_mock_design.md` 的「mock 不可录屏」结论按新口径改为
> 「2D 俯视不构成同类型平台」）。
> 2026-10-19 六版（**决策：不购买官方仿真平台**，开发期走 mock；录屏路线赛前再议——
> 规则硬约束「官方平台或真实设备」，mock 不能录屏。`aic-vn-mock` 设计定稿落
> `docs/guides/vn_mock_design.md`：MockCamera / MockPlc / SimCore / OpenCV 2D Viewer、
> 图像传输格式与端口未定义项全部可配置、判分严格对齐官方、2.5–3 天工作量、未实现）。
