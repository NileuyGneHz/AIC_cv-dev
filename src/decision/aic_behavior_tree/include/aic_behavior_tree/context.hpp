// 决策侧节点共享上下文。
//
// 行为树节点由工厂实例化，构造时注入本上下文（按值拷贝，但内部全是 shared_ptr，
// 所以所有节点看到的是同一批对象——尤其是同一份「游标 + 结果表」）。
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "aic_behavior_tree/control_bridge.hpp"
#include "aic_behavior_tree/result_log.hpp"
#include "aic_behavior_tree/vision_client.hpp"
#include "aic_warehouse/warehouse_layout.hpp"

namespace aic_behavior_tree
{

struct DecisionContext
{
  /// 货架布局（几何 + 访问顺序）与巡视游标：两者由 InitRun 在每个新运行开头复位
  std::shared_ptr<aic_warehouse::WarehouseLayout> layout;
  std::shared_ptr<aic_warehouse::WarehouseCursor> cursor;

  /// vision 桥接（订阅 + 序号）
  std::shared_ptr<VisionClient> vision;
  /// control 桥接（发布任务；control 侧实现留白）
  std::shared_ptr<ControlBridge> control;
  /// 位点结果表 / CSV
  std::shared_ptr<ResultLog> results;

  /// 默认参数（runner 由 ROS 参数注入；XML 里用黑板变量覆盖）
  std::string warehouse_yaml{};
  std::string results_csv{};
  double default_vision_timeout_s{3.0};

  rclcpp::Logger logger{rclcpp::get_logger("aic_decision_bt")};

  /// 便于节点里少写一点样板
  double visionTimeoutOr(double fallback) const
  {
    return fallback > 0.0 ? fallback : default_vision_timeout_s;
  }
};

}  // namespace aic_behavior_tree
