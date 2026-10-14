// 决策 → control 的桥接：把行为树的决策结果发成 ROS 话题消息。
//
// **实现边界（本版有意留白）**：
//   决策只发布「去哪个库位（slot_id/row/col）、物料算什么类型（material_type）、做什么动作（action）」
//   三轴插补、坐标换算、串口/Modbus/PLC 交互、到位反馈全部由 control 负责，
//   见 docs/guides/decision_control_interface.md 的「control 侧待实现清单」。
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include <aic_msgs/msg/material_task.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace aic_behavior_tree
{

class ControlBridge
{
public:
  /// wait_ready=true 时订阅 ready_topic（std_msgs/Bool），用于将来「等 control 就绪/到位」的握手
  ControlBridge(
    rclcpp::Node & node, const std::string & task_topic,
    bool wait_ready = false, const std::string & ready_topic = "/control/ready");

  /// 发布一条任务；返回是否成功入列（发布器可用即视为成功）
  bool publish(
    int slot_id, int row, int col, int visit_index, int material_type,
    const std::string & action, const std::string & note);

  /// control 是否报「就绪/到位」。control 未实现时恒为 false —— 所以默认不阻塞等待。
  bool ready() const;
  /// control 是否曾上报过 ready（用于区分「没实现」与「实现了但当前未就绪」）
  bool readySeen() const;

  std::uint64_t publishedCount() const;
  std::uint64_t countByAction(const std::string & action) const;

private:
  void onReady(std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Publisher<aic_msgs::msg::MaterialTask>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;

  mutable std::mutex mtx_;
  bool ready_{false};
  bool ready_seen_{false};
  std::uint64_t published_{0};
  std::map<std::string, std::uint64_t> per_action_;
};

}  // namespace aic_behavior_tree
