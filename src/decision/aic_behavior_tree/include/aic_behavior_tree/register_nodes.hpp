// 把本包的自定义节点注册进工厂（带决策上下文）。
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "aic_behavior_tree/context.hpp"

namespace aic_behavior_tree
{

/// 注册全部决策节点；ctx 会被拷贝进每个节点的构造参数（内部为共享指针）。
void registerDecisionNodes(BT::BehaviorTreeFactory & factory, DecisionContext ctx);

}  // namespace aic_behavior_tree
