#include "aic_behavior_tree/register_nodes.hpp"

#include "aic_behavior_tree/nodes.hpp"

namespace aic_behavior_tree
{

void registerDecisionNodes(BT::BehaviorTreeFactory & factory, DecisionContext ctx)
{
  factory.registerNodeType<InitRun>("InitRun", ctx);
  factory.registerNodeType<WhileSlotsRemain>("WhileSlotsRemain", ctx);
  factory.registerNodeType<GotoNextSlot>("GotoNextSlot", ctx);
  factory.registerNodeType<WaitVisionResult>("WaitVisionResult", ctx);
  factory.registerNodeType<DispatchMaterialTask>("DispatchMaterialTask", ctx);
  factory.registerNodeType<SkipSlot>("SkipSlot", ctx);
  factory.registerNodeType<RecordSlotResult>("RecordSlotResult", ctx);
  factory.registerNodeType<RunSummary>("RunSummary", ctx);
}

}  // namespace aic_behavior_tree
