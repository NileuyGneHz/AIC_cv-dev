// 决策层自定义行为树节点（BT.CPP v4）。
//
// 主树形状（见 config/trees/aic_sort.xml）：
//   InitRun
//   WhileSlotsRemain          <- 只要还有下一位点就重复
//     Sequence[OneSlot]
//       GotoNextSlot          <- 推进游标 + 发 GOTO 任务（control 桥接）
//       Fallback              <- 等 vision 结果；超时则记 TIMEOUT 后继续
//         WaitVisionResult
//         RecordSlotResult/TIMEOUT
//       Switch4               <- material_type 1/2/3/4 -> 对应处理子树
//         SubTree HandleMaterial1..4
//         SkipSlot            <- 非法/缺失类型的兜底（只记日志，不重复记账）
//   RunSummary
//
// 所有节点共用 DecisionContext（构造注入），黑板键统一见 bb_keys.hpp。
#pragma once

#include <cstdint>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorator_node.h>

#include "aic_behavior_tree/context.hpp"

namespace aic_behavior_tree
{

/// 一次运行的开头：重载货架布局、复位游标与结果表、补齐黑板默认值。
class InitRun : public BT::SyncActionNode
{
public:
  InitRun(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 循环装饰器：只要货架还有下一位点，就重复 tick 子树。
class WhileSlotsRemain : public BT::DecoratorNode
{
public:
  WhileSlotsRemain(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 推进到下一位点，并把 GOTO 任务交给 control 桥接；同时记住「此刻的 vision 序号」。
class GotoNextSlot : public BT::SyncActionNode
{
public:
  GotoNextSlot(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 等待 vision 返回「本次移动之后」产生的识别结果；异步（RUNNING），超时返回 FAILURE。
class WaitVisionResult : public BT::StatefulActionNode
{
public:
  WaitVisionResult(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  DecisionContext ctx_;
  std::uint64_t after_seq_{0};
  double deadline_s_{0.0};
  double timeout_s_{0.0};
};

/// 把「处理当前位点」的任务交给 control 桥接（control 侧实现留白）。
class DispatchMaterialTask : public BT::SyncActionNode
{
public:
  DispatchMaterialTask(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 兜底：物料类型非法/缺失时不触发任何处理子树，只留一条日志（不重复记账）。
class SkipSlot : public BT::SyncActionNode
{
public:
  SkipSlot(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 记录一个位点的结果（逐个位点落一行：播报 + CSV）。
class RecordSlotResult : public BT::SyncActionNode
{
public:
  RecordSlotResult(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

/// 运行结束：打印汇总。
class RunSummary : public BT::SyncActionNode
{
public:
  RunSummary(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  DecisionContext ctx_;
};

}  // namespace aic_behavior_tree
