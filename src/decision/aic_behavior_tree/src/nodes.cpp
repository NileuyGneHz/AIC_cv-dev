// 决策层行为树节点的实现。
#include "aic_behavior_tree/nodes.hpp"

#include <behaviortree_cpp/blackboard.h>

#include <chrono>
#include <string>
#include <utility>

#include "aic_behavior_tree/bb_keys.hpp"
#include "aic_behavior_tree/material_types.hpp"

namespace aic_behavior_tree
{
namespace
{

/// 单调时钟（秒）。刻意不用 ROS 时钟：单点耗时统计不该受 use_sim_time 影响。
double steadySeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

std::string stringOr(const std::string & value, const std::string & fallback)
{
  return value.empty() ? fallback : value;
}

/// 读黑板，取不到就用兜底值（BT 的 Blackboard::get() 标了 [[nodiscard]]，这里统一消费掉）
template<typename T>
T getOr(const BT::Blackboard::Ptr & bb, const std::string & key, const T & fallback)
{
  T value{};
  if (bb != nullptr && bb->get<T>(key, value)) {
    return value;
  }
  return fallback;
}

}  // namespace

// ---------------------------------------------------------------- InitRun

InitRun::InitRun(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList InitRun::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "warehouse_yaml", "",
      "货架布局 YAML 路径；留空则沿用 runner 已加载的布局"),
    BT::InputPort<bool>(
      "reset_results", "true",
      "是否清空结果表（新一次运行置 true）"),
  };
}

BT::NodeStatus InitRun::tick()
{
  if (!ctx_.layout || !ctx_.cursor) {
    RCLCPP_ERROR(ctx_.logger, "[decision] InitRun：布局/游标未注入（runner 配置问题）");
    return BT::NodeStatus::FAILURE;
  }

  std::string yaml;
  if (!getInput("warehouse_yaml", yaml) || yaml.empty()) {
    yaml = ctx_.warehouse_yaml;
  }
  if (!yaml.empty()) {
    try {
      *ctx_.layout = aic_warehouse::WarehouseLayout::fromYamlFile(yaml);
      RCLCPP_INFO(ctx_.logger, "[decision] 已按 '%s' 加载货架布局", yaml.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(ctx_.logger, "[decision] 加载货架布局失败：%s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (ctx_.layout->slotCount() <= 0) {
    RCLCPP_ERROR(ctx_.logger, "[decision] 货架布局为空（库位数 0），无法开始巡视");
    return BT::NodeStatus::FAILURE;
  }

  ctx_.cursor->bind(ctx_.layout.get());

  bool reset_results = true;
  getInput("reset_results", reset_results);
  if (reset_results && ctx_.results) {
    ctx_.results->clear();
  }

  auto bb = config().blackboard;
  bb->set<std::string>(bb::kMaterialType, "0");
  bb->set<std::string>(bb::kMaterialRaw, "");
  bb->set<int>(bb::kSlotId, 0);
  bb->set<int>(bb::kRow, 0);
  bb->set<int>(bb::kCol, 0);
  bb->set<int>(bb::kVisitIndex, 0);
  bb->set<int>(bb::kSlotCount, ctx_.layout->slotCount());
  bb->set<std::uint64_t>(bb::kVisionSeqBeforeMove, 0);
  bb->set<double>(bb::kSlotStartTime, 0.0);

  double timeout = 0.0;
  if (!bb->get<double>(bb::kVisionTimeout, timeout) || timeout <= 0.0) {
    bb->set<double>(bb::kVisionTimeout, ctx_.default_vision_timeout_s);
  }

  RCLCPP_INFO(
    ctx_.logger, "[decision] 初始化完成：%d 个库位，出发点 %s，首次扫行 %s",
    ctx_.layout->slotCount(), ctx_.layout->config().start_corner.c_str(),
    ctx_.layout->config().first_direction.c_str());
  return BT::NodeStatus::SUCCESS;
}

// --------------------------------------------------------- WhileSlotsRemain

WhileSlotsRemain::WhileSlotsRemain(
  const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList WhileSlotsRemain::providedPorts()
{
  return {};
}

BT::NodeStatus WhileSlotsRemain::tick()
{
  if (!ctx_.cursor) {
    RCLCPP_ERROR(ctx_.logger, "[decision] WhileSlotsRemain：游标未注入");
    return BT::NodeStatus::FAILURE;
  }

  while (ctx_.cursor->hasNext()) {
    const int before = ctx_.cursor->visitIndex();
    const BT::NodeStatus child_status = child_node_->executeTick();

    if (child_status == BT::NodeStatus::RUNNING) {
      return child_status;
    }
    if (child_status != BT::NodeStatus::SUCCESS) {
      return child_status;
    }
    // 防死循环：子节点成功了却没推进游标，说明 OneSlot 里缺 GotoNextSlot
    if (ctx_.cursor->visitIndex() == before) {
      RCLCPP_ERROR(
        ctx_.logger,
        "[decision] SlotsRemain 检测到子节点未推进游标（检查 OneSlot 是否包含 GotoNextSlot）");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::SUCCESS;
}

// --------------------------------------------------------------- GotoNextSlot

GotoNextSlot::GotoNextSlot(
  const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList GotoNextSlot::providedPorts()
{
  return {};
}

BT::NodeStatus GotoNextSlot::tick()
{
  const aic_warehouse::Slot * slot = ctx_.cursor->advance();
  if (slot == nullptr) {
    RCLCPP_WARN(ctx_.logger, "[decision] 已无下一位点（巡视结束）");
    return BT::NodeStatus::FAILURE;
  }
  const int visit = ctx_.cursor->visitIndex();

  auto bb = config().blackboard;
  // 先清残留：上一位点的类型不能漏到本位点（否则 Switch4 会走错分支）
  bb->set<std::string>(bb::kMaterialType, "0");
  bb->set<std::string>(bb::kMaterialRaw, "");
  bb->set<int>(bb::kSlotId, slot->id);
  bb->set<int>(bb::kRow, slot->row);
  bb->set<int>(bb::kCol, slot->col);
  bb->set<int>(bb::kVisitIndex, visit);
  bb->set<double>(bb::kSlotStartTime, steadySeconds());

  const bool published = ctx_.control->publish(
    slot->id, slot->row, slot->col, visit, 0, "GOTO", "巡视位点");
  if (!published) {
    RCLCPP_ERROR(ctx_.logger, "[decision] 位点 #%d 的 GOTO 任务发布失败", visit);
    return BT::NodeStatus::FAILURE;
  }
  ctx_.vision->requestCapture();

  // 记录「此刻已收到的 vision 序号」：之后只认比它更新的结果
  const std::uint64_t seq_now = ctx_.vision->seq();
  bb->set<std::uint64_t>(bb::kVisionSeqBeforeMove, seq_now);

  RCLCPP_INFO(
    ctx_.logger, "[decision] 第 %d/%d 次移动 -> slot %02d (row%d,col%d)；等待 vision（基准 seq=%lu）",
    visit, ctx_.cursor->total(), slot->id, slot->row, slot->col,
    static_cast<unsigned long>(seq_now));
  return BT::NodeStatus::SUCCESS;
}

// ----------------------------------------------------------- WaitVisionResult

WaitVisionResult::WaitVisionResult(
  const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList WaitVisionResult::providedPorts()
{
  return {
    BT::InputPort<double>("timeout", "3.0", "等待 vision 结果的超时（秒）"),
    BT::InputPort<std::uint64_t>(
      "after_seq", "{vision_seq_before_move}",
      "只接受序号大于该值的识别结果（默认取 GotoNextSlot 写入的黑板值）"),
  };
}

BT::NodeStatus WaitVisionResult::onStart()
{
  double timeout = 0.0;
  if (!getInput("timeout", timeout) || timeout <= 0.0) {
    timeout = ctx_.default_vision_timeout_s;
  }
  timeout_s_ = timeout;

  std::uint64_t seq0 = 0;
  const bool from_port = getInput("after_seq", seq0).has_value();
  if (!from_port && !config().blackboard->get<std::uint64_t>(bb::kVisionSeqBeforeMove, seq0)) {
    // 兜底：拿不到基准就用当前序号（等于「从现在起算」），并留下告警
    seq0 = ctx_.vision->seq();
    RCLCPP_WARN(
      ctx_.logger, "[decision] 取不到 vision 序号基准，改用当前序号 %lu",
      static_cast<unsigned long>(seq0));
  }
  after_seq_ = seq0;
  deadline_s_ = steadySeconds() + timeout_s_;

  RCLCPP_DEBUG(
    ctx_.logger, "[decision] 等待 vision 新结果（seq > %lu，超时 %.2f s）",
    static_cast<unsigned long>(after_seq_), timeout_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitVisionResult::onRunning()
{
  VisionSample sample;
  if (ctx_.vision->latestNewerThan(after_seq_, sample)) {
    auto bb = config().blackboard;
    bb->set<std::string>(bb::kMaterialType, std::to_string(sample.material_type));
    bb->set<std::string>(bb::kMaterialRaw, sample.raw);

    const int visit = getOr<int>(bb, bb::kVisitIndex, 0);
    const int slot_id = getOr<int>(bb, bb::kSlotId, 0);
    RCLCPP_INFO(
      ctx_.logger, "[decision] 位点 #%d (slot %02d) 识别完成：'%s' -> 类型 %d(%s)，耗时 %.3f s",
      visit, slot_id, sample.raw.c_str(), sample.material_type,
      materialTypeName(sample.material_type).c_str(), timeout_s_ - (deadline_s_ - steadySeconds()));
    return BT::NodeStatus::SUCCESS;
  }

  if (steadySeconds() >= deadline_s_) {
    RCLCPP_WARN(
      ctx_.logger,
      "[decision] 等待 vision 超时（%.2f s）：已收 %lu 条、其中非法 %lu 条；本位点将记为 TIMEOUT",
      timeout_s_, static_cast<unsigned long>(ctx_.vision->received()),
      static_cast<unsigned long>(ctx_.vision->invalid()));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitVisionResult::onHalted()
{
  RCLCPP_DEBUG(ctx_.logger, "[decision] WaitVisionResult 被中止");
}

// ------------------------------------------------------- DispatchMaterialTask

DispatchMaterialTask::DispatchMaterialTask(
  const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList DispatchMaterialTask::providedPorts()
{
  return {
    BT::InputPort<std::string>("action", "HANDLE", "任务动作，见 aic_msgs/MaterialTask.msg"),
    BT::InputPort<std::string>(
      "material_type", "{material_type}",
      "物料类型（字符串）；默认取本位点识别结果，可在子树里覆盖"),
    BT::InputPort<std::string>("note", "", "备注，透传给 control"),
  };
}

BT::NodeStatus DispatchMaterialTask::tick()
{
  auto bb = config().blackboard;
  const int slot_id = getOr<int>(bb, bb::kSlotId, 0);
  const int row = getOr<int>(bb, bb::kRow, 0);
  const int col = getOr<int>(bb, bb::kCol, 0);
  const int visit = getOr<int>(bb, bb::kVisitIndex, 0);

  std::string type_str;
  getInput("material_type", type_str);
  const int material_type = parseMaterialType(type_str);

  std::string action;
  getInput("action", action);
  action = stringOr(action, "HANDLE");
  std::string note;
  getInput("note", note);

  if (!ctx_.control->publish(slot_id, row, col, visit, material_type, action, note)) {
    RCLCPP_ERROR(ctx_.logger, "[decision] 位点 #%d 的 %s 任务发布失败", visit, action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ctx_.logger, "[decision] 位点 #%d slot %02d：下发 %s（类型 %d(%s)）%s",
    visit, slot_id, action.c_str(), material_type, materialTypeName(material_type).c_str(),
    note.empty() ? "" : ("；" + note).c_str());
  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------------- SkipSlot

SkipSlot::SkipSlot(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList SkipSlot::providedPorts()
{
  return {BT::InputPort<std::string>("note", "", "说明")};
}

BT::NodeStatus SkipSlot::tick()
{
  std::string note;
  getInput("note", note);

  auto bb = config().blackboard;
  const std::string type_str = getOr<std::string>(bb, bb::kMaterialType, "0");
  const int visit = getOr<int>(bb, bb::kVisitIndex, 0);

  RCLCPP_WARN(
    ctx_.logger, "[decision] 位点 #%d 的物料类型为 '%s'（非 1..4），不触发处理子树%s",
    visit, type_str.c_str(), note.empty() ? "" : ("；" + note).c_str());
  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------ RecordSlotResult

RecordSlotResult::RecordSlotResult(
  const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList RecordSlotResult::providedPorts()
{
  return {
    BT::InputPort<std::string>("result", "OK", "本行结果：OK / TIMEOUT / UNKNOWN"),
    BT::InputPort<std::string>("note", "", "备注"),
  };
}

BT::NodeStatus RecordSlotResult::tick()
{
  auto bb = config().blackboard;

  SlotOutcome outcome;
  outcome.visit_index = getOr<int>(bb, bb::kVisitIndex, 0);
  outcome.slot_id = getOr<int>(bb, bb::kSlotId, 0);
  outcome.row = getOr<int>(bb, bb::kRow, 0);
  outcome.col = getOr<int>(bb, bb::kCol, 0);

  const std::string type_str = getOr<std::string>(bb, bb::kMaterialType, "0");
  outcome.material_type = parseMaterialType(type_str);
  outcome.material_raw = getOr<std::string>(bb, bb::kMaterialRaw, "");

  const double t0 = getOr<double>(bb, bb::kSlotStartTime, 0.0);
  outcome.elapsed_s = t0 > 0.0 ? (steadySeconds() - t0) : 0.0;

  outcome.result = "OK";
  getInput("result", outcome.result);
  getInput("note", outcome.note);

  if (ctx_.results) {
    ctx_.results->append(outcome);
  }

  // 一行一位点：现场可照着念，赛后可直接对 CSV 复盘
  RCLCPP_INFO(
    ctx_.logger, "[RESULT] %02d/%02d | slot %02d (row%d,col%d) | type %d(%s) | %s | %.3fs%s",
    outcome.visit_index, ctx_.cursor ? ctx_.cursor->total() : 0, outcome.slot_id, outcome.row,
    outcome.col, outcome.material_type, materialTypeName(outcome.material_type).c_str(),
    outcome.result.c_str(), outcome.elapsed_s,
    outcome.note.empty() ? "" : (" | " + outcome.note).c_str());
  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------------ RunSummary

RunSummary::RunSummary(const std::string & name, const BT::NodeConfig & config, DecisionContext ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList RunSummary::providedPorts()
{
  return {};
}

BT::NodeStatus RunSummary::tick()
{
  if (ctx_.results) {
    RCLCPP_INFO(ctx_.logger, "\n%s", ctx_.results->summary().c_str());
  }
  RCLCPP_INFO(
    ctx_.logger, "[decision] 已发布任务 %lu 条（GOTO %lu / HANDLE %lu）；vision 收到 %lu 条",
    static_cast<unsigned long>(ctx_.control->publishedCount()),
    static_cast<unsigned long>(ctx_.control->countByAction("GOTO")),
    static_cast<unsigned long>(ctx_.control->countByAction("HANDLE")),
    static_cast<unsigned long>(ctx_.vision->received()));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace aic_behavior_tree
