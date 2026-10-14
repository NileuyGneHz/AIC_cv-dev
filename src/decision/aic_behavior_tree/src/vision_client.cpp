#include "aic_behavior_tree/vision_client.hpp"

#include <utility>

#include "aic_behavior_tree/material_types.hpp"

namespace aic_behavior_tree
{

VisionClient::VisionClient(
  rclcpp::Node & node, const std::string & topic,
  bool enable_trigger, const std::string & trigger_topic)
: node_(node)
{
  // 坑：这里用默认 QoS（reliable + volatile，深度 10）。若 vision 侧用 best_effort 发布，
  // 两边 QoS 不兼容会「订阅成功但一条都收不到」，且不带任何报错。
  // 约定见 docs/guides/decision_vision_interface.md —— 保持默认即可互通。
  sub_ = node_.create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(10),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {onMessage(std::move(msg));});

  if (enable_trigger) {
    trigger_pub_ = node_.create_publisher<std_msgs::msg::Empty>(trigger_topic, rclcpp::QoS(10));
  }

  const std::string trigger_note =
    enable_trigger ? ("并按需发布采集请求 '" + trigger_topic + "'") : "不主动请求采集";
  RCLCPP_INFO(
    node_.get_logger(),
    "[decision] vision 桥接就绪：订阅 '%s'（std_msgs/String，取值 \"1\"..\"4\"）；%s",
    topic.c_str(), trigger_note.c_str());
}

void VisionClient::onMessage(std_msgs::msg::String::ConstSharedPtr msg)
{
  const int type = parseMaterialType(msg->data);

  std::lock_guard<std::mutex> lock(mtx_);
  ++received_;
  if (type < kMaterialTypeMin || type > kMaterialTypeMax) {
    ++invalid_;
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "[decision] vision 返回非法物料字符串 '%s'（约定仅 \"1\"..\"4\"），已丢弃；累计非法 %lu 条",
      msg->data.c_str(), static_cast<unsigned long>(invalid_));
    return;
  }

  ++seq_;
  latest_.raw = msg->data;
  latest_.material_type = type;
  latest_.seq = seq_;
  latest_.stamp = node_.now();

  RCLCPP_INFO(
    node_.get_logger(), "[decision] 收到 vision 结果 #%lu：'%s' -> 类型 %d(%s)",
    static_cast<unsigned long>(seq_), latest_.raw.c_str(), type,
    materialTypeName(type).c_str());
}

std::uint64_t VisionClient::seq() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return seq_;
}

bool VisionClient::latest(VisionSample & out) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq_ == 0) {
    return false;
  }
  out = latest_;
  return true;
}

bool VisionClient::latestNewerThan(std::uint64_t after_seq, VisionSample & out) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq_ <= after_seq) {
    return false;
  }
  out = latest_;
  return true;
}

void VisionClient::requestCapture()
{
  if (!trigger_pub_) {
    return;
  }
  trigger_pub_->publish(std_msgs::msg::Empty());
  RCLCPP_DEBUG(node_.get_logger(), "[decision] 已发出一次采集请求");
}

std::uint64_t VisionClient::received() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return received_;
}

std::uint64_t VisionClient::invalid() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return invalid_;
}

}  // namespace aic_behavior_tree
