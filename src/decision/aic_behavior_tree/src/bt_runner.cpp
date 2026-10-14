// 决策行为树执行器：一个 ROS 2 节点 = 一套「vision 桥接 + control 桥接 + 行为树」。
//
// 线程模型：
//   - 主线程：按 tick_rate_hz 推进行为树（tree.tickOnce()）
//   - 执行器线程：单线程 executor，跑订阅回调（vision 收数、control ready）
//   两个线程之间只通过 VisionClient/ControlBridge 内部的互斥锁交换数据。
//
// 用法：
//   ros2 run aic_behavior_tree bt_runner --ros-args --params-file <params.yaml>
//   ros2 run aic_behavior_tree bt_runner --ros-args -p once:=true    # 跑完一轮即退出（自验用）
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "aic_behavior_tree/bb_keys.hpp"
#include "aic_behavior_tree/context.hpp"
#include "aic_behavior_tree/control_bridge.hpp"
#include "aic_behavior_tree/register_nodes.hpp"
#include "aic_behavior_tree/result_log.hpp"
#include "aic_behavior_tree/vision_client.hpp"
#include "aic_warehouse/warehouse_layout.hpp"

namespace
{

std::atomic<bool> g_stop{false};

void handleSignal(int)
{
  g_stop = true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("aic_decision_bt");

  // ---------------------------------------------------------------- 参数
  const std::string tree_xml_arg =
    node->declare_parameter<std::string>("tree_xml", "");
  const std::string warehouse_yaml_arg =
    node->declare_parameter<std::string>("warehouse_yaml", "");
  const std::string vision_topic =
    node->declare_parameter<std::string>("vision_topic", "/vision/material_type");
  const bool vision_trigger_enable =
    node->declare_parameter<bool>("vision_trigger_enable", false);
  const std::string vision_trigger_topic =
    node->declare_parameter<std::string>("vision_trigger_topic", "/vision/capture");
  const double vision_timeout_s =
    node->declare_parameter<double>("vision_timeout_s", 3.0);
  const std::string control_task_topic =
    node->declare_parameter<std::string>("control_task_topic", "/control/material_task");
  const bool control_wait_ready =
    node->declare_parameter<bool>("control_wait_ready", false);
  const std::string control_ready_topic =
    node->declare_parameter<std::string>("control_ready_topic", "/control/ready");
  const std::string results_csv_arg =
    node->declare_parameter<std::string>("results_csv", "");
  const double tick_rate_hz = node->declare_parameter<double>("tick_rate_hz", 50.0);
  const bool once = node->declare_parameter<bool>("once", false);
  const bool auto_restart = node->declare_parameter<bool>("auto_restart", false);

  // ------------------------------------------------- 默认路径（包共享目录）
  std::string tree_xml = tree_xml_arg;
  if (tree_xml.empty()) {
    tree_xml = ament_index_cpp::get_package_share_directory("aic_behavior_tree") +
      "/config/trees/aic_sort.xml";
  }
  std::string warehouse_yaml = warehouse_yaml_arg;
  if (warehouse_yaml.empty()) {
    warehouse_yaml = ament_index_cpp::get_package_share_directory("aic_warehouse") +
      "/config/warehouse_layout.yaml";
  }

  // ------------------------------------------------------------- 上下文
  aic_behavior_tree::DecisionContext ctx;
  try {
    ctx.layout = std::make_shared<aic_warehouse::WarehouseLayout>(
      aic_warehouse::WarehouseLayout::fromYamlFile(warehouse_yaml));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "[decision] 货架布局加载失败：%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  ctx.cursor = std::make_shared<aic_warehouse::WarehouseCursor>(ctx.layout.get());
  ctx.vision = std::make_shared<aic_behavior_tree::VisionClient>(
    *node, vision_topic, vision_trigger_enable, vision_trigger_topic);
  ctx.control = std::make_shared<aic_behavior_tree::ControlBridge>(
    *node, control_task_topic, control_wait_ready, control_ready_topic);
  ctx.results = std::make_shared<aic_behavior_tree::ResultLog>(results_csv_arg);
  ctx.warehouse_yaml = warehouse_yaml;
  ctx.results_csv = results_csv_arg;
  ctx.default_vision_timeout_s = vision_timeout_s;
  ctx.logger = node->get_logger();

  // -------------------------------------------------------- 工厂与行为树
  BT::BehaviorTreeFactory factory;
  aic_behavior_tree::registerDecisionNodes(factory, ctx);

  BT::Tree tree;
  try {
    factory.registerBehaviorTreeFromFile(tree_xml);
    tree = factory.createTree("AicSort");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "[decision] 行为树加载失败（%s）：%s", tree_xml.c_str(), e.what());
    rclcpp::shutdown();
    return 2;
  }

  // XML 里用 {xxx} 引用的黑板变量在这里给默认值（ROS 参数 -> 黑板）
  auto blackboard = tree.rootBlackboard();
  blackboard->set<std::string>(aic_behavior_tree::bb::kWarehouseYaml, warehouse_yaml);
  blackboard->set<std::string>(aic_behavior_tree::bb::kResultsCsv, results_csv_arg);
  blackboard->set<double>(aic_behavior_tree::bb::kVisionTimeout, vision_timeout_s);

  RCLCPP_INFO(
    node->get_logger(),
    "[decision] 就绪：树=%s | 库位=%d | vision='%s' | control='%s' | tick=%.1fHz",
    tree_xml.c_str(), ctx.layout->slotCount(), vision_topic.c_str(),
    control_task_topic.c_str(), tick_rate_hz);
  RCLCPP_INFO(node->get_logger(), "\n%s", ctx.layout->describe().c_str());

  // ------------------------------------------------------------ 线程与主循环
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  const double rate = tick_rate_hz > 1.0 ? tick_rate_hz : 1.0;
  const auto period = std::chrono::duration<double>(1.0 / rate);

  int runs = 0;
  int exit_code = 0;
  while (rclcpp::ok() && !g_stop) {
    const BT::NodeStatus status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      ++runs;
      const bool ok = (status == BT::NodeStatus::SUCCESS);
      RCLCPP_INFO(
        node->get_logger(), "[decision] 第 %d 轮结束：%s（GOTO %lu / HANDLE %lu）", runs,
        ok ? "SUCCESS" : "FAILURE",
        static_cast<unsigned long>(ctx.control->countByAction("GOTO")),
        static_cast<unsigned long>(ctx.control->countByAction("HANDLE")));
      if (!ok) {
        exit_code = 1;
      }
      if (once || !auto_restart) {
        break;
      }
      RCLCPP_INFO(node->get_logger(), "[decision] auto_restart=true，开始新一轮巡视");
    }
    std::this_thread::sleep_for(period);
  }

  tree.haltTree();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  RCLCPP_INFO(node->get_logger(), "[decision] 退出（轮数 %d）", runs);
  rclcpp::shutdown();
  return exit_code;
}
