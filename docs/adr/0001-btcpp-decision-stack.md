# ADR 0001 — 决策层选型：ROS 2 Humble + BehaviorTree.CPP 4.10（C++），12 位点蛇形巡视

- 状态：已接受
- 日期：2026-10-15
- 关联：AGENTS.md 第 3 节技术栈；docs/guides/decision_bt_architecture.md

## 背景

省赛提交物需要「编程逻辑介绍」与技术文档（PPT 形式）；比赛流程是固定的
12 步机械移动 × 每步一次视觉识别 × 分类型处理，且要求数小时内搭出可自验的骨架。
宿主机已具备 ROS 2 Humble 与 BT.CPP 4.10（/opt/ros/humble），本机 docker 链路不稳定。

## 决策

1. **决策引擎用 BehaviorTree.CPP 4.10（原生 C++，不用 behaviortree_ros2 包装）**：
   - 12 个位点的「移动→等待→分发」是典型 Sequence + 循环结构，BT 的节点粒度正好
     对应「GOTO / 等识别 / 分发处理」，PPT 里也好画。
   - 不引入 `behaviortree_ros2`：减少一层依赖，桥接逻辑（订阅/发布）自己写,
     总量不到 200 行，且行为可控（seq 过滤、QoS 约定都收在自己手里）。
2. **Python 只用于开发期桩与自验脚本**（fake_vision / recorder / checker），
   决策本体 C++——与负责人的技术栈一致，比赛运行环境也以 C++ 为主。
3. **12 位点顺序由 aic_warehouse 库生成而非硬编码**：
   布局（行数/列数/出发点/首行方向）进 YAML，生成器 + gtest 单测；
   若赛题布局变化（比如改 4 行），改配置不改代码。
4. **类型分发用 `<Switch4>` + 四个 SubTree**：每个缺陷类型一个留白子树，
   control 实现接入时只改子树内部，主树与话题契约不动。

## 后果

- 正：骨架当天闭环、自验（2 场景）全绿；文档/黑板键/节点三处同源。
- 负：BT.CPP 4.10 的三个坑（装饰器复位 / tick 重入 / hasNext 语义）已写进
  architecture 文档，后续改树必须读；SubTree 黑板隔离导致 XML 里端口映射较啰嗦。
- 备选被否：py_trees（Python，性能与提交一致性弱）；纯手写状态机
  （PPT 讲解成本高、扩展子树不如 BT 直观）。
