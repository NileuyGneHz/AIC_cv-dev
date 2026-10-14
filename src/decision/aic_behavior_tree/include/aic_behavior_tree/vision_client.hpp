// 决策 ← vision 的桥接：ROS 话题 -> 行为树可用的「最新识别结果」。
//
// 契约（详见 docs/guides/decision_vision_interface.md）：
//   话题 /vision/material_type，类型 std_msgs/msg/String，data ∈ {"1","2","3","4"}
//   每完成一次位点识别，vision 发一条；决策在每个位点下发 GOTO 后等待「比 GOTO 时刻更新」的一条。
//
// 为什么要序号（seq）而不是直接用最新值：
//   若 vision 连续发布，决策可能把上一位点的残留结果当成本位点的结果；用 seq 做「只看新知」的闸门，
//   把这种错配挡在桥接层，行为树侧就不需要关心时序。
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace aic_behavior_tree
{

/// 一次识别结果的快照
struct VisionSample
{
  std::string raw;          ///< vision 发出的原始字符串
  int material_type{0};     ///< 1..4；0 = 未识别/非法
  std::uint64_t seq{0};     ///< 合法结果序号，从 1 开始；非法消息不推进
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

  bool valid() const {return material_type >= 1 && material_type <= 4;}
};

class VisionClient
{
public:
  /// enable_trigger=true 时额外开辟一个「请 vision 拍照」的话题（默认关闭，属与 vision 组的待定项）
  VisionClient(
    rclcpp::Node & node, const std::string & topic,
    bool enable_trigger = false, const std::string & trigger_topic = "/vision/capture");

  /// 已接受的合法结果数
  std::uint64_t seq() const;
  /// 最近一次合法结果；没有则 false
  bool latest(VisionSample & out) const;
  /// 最近一次「比 after_seq 更新」的合法结果；没有则 false（非阻塞，供行为树每 tick 轮询）
  bool latestNewerThan(std::uint64_t after_seq, VisionSample & out) const;

  /// 可选：通知 vision 采集一次（enable_trigger=false 时空操作）
  void requestCapture();

  /// 收到过的消息总数（含非法）
  std::uint64_t received() const;
  /// 被丢弃的非法消息数
  std::uint64_t invalid() const;

private:
  void onMessage(std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr trigger_pub_;

  mutable std::mutex mtx_;
  VisionSample latest_{};
  std::uint64_t seq_{0};
  std::uint64_t received_{0};
  std::uint64_t invalid_{0};
};

}  // namespace aic_behavior_tree
