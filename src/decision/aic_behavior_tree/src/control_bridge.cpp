#include "aic_behavior_tree/control_bridge.hpp"

namespace aic_behavior_tree
{

ControlBridge::ControlBridge(
  rclcpp::Node & node, const std::string & task_topic,
  bool wait_ready, const std::string & ready_topic)
: node_(node)
{
  pub_ = node_.create_publisher<aic_msgs::msg::MaterialTask>(task_topic, rclcpp::QoS(10));
  if (wait_ready) {
    // 留白：control 侧尚未实现该话题时，这里订阅到的是「无人发布」，ready() 恒为 false。
    ready_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {onReady(std::move(msg));});
  }

  const std::string ready_note =
    wait_ready ? ("并监听就绪 '" + ready_topic + "'") : "不做就绪握手（control 侧留白）";
  RCLCPP_INFO(
    node_.get_logger(), "[decision] control 桥接就绪：发布 '%s'（aic_msgs/MaterialTask）；%s",
    task_topic.c_str(), ready_note.c_str());
}

bool ControlBridge::publish(
  int slot_id, int row, int col, int visit_index, int material_type,
  const std::string & action, const std::string & note)
{
  if (!pub_) {
    return false;
  }
  aic_msgs::msg::MaterialTask msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "warehouse";
  msg.slot_id = static_cast<std::uint8_t>(slot_id);
  msg.visit_index = static_cast<std::uint8_t>(visit_index);
  msg.row = static_cast<std::uint8_t>(row);
  msg.col = static_cast<std::uint8_t>(col);
  msg.material_type = static_cast<std::uint8_t>(material_type);
  msg.action = action;
  msg.note = note;
  pub_->publish(msg);

  {
    std::lock_guard<std::mutex> lock(mtx_);
    ++published_;
    ++per_action_[action];
  }
  return true;
}

void ControlBridge::onReady(std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  ready_ = msg->data;
  ready_seen_ = true;
}

bool ControlBridge::ready() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ready_;
}

bool ControlBridge::readySeen() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ready_seen_;
}

std::uint64_t ControlBridge::publishedCount() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return published_;
}

std::uint64_t ControlBridge::countByAction(const std::string & action) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = per_action_.find(action);
  return it == per_action_.end() ? 0 : it->second;
}

}  // namespace aic_behavior_tree
