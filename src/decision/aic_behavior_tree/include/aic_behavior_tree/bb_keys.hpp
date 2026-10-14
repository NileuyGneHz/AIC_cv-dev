// 行为树黑板键的唯一登记处：XML、节点、文档三处引用同一份常量，避免拼写漂移。
#pragma once

namespace aic_behavior_tree
{
namespace bb
{

/// 当前位点的物料类型，字符串 "0".."4"（"0" = 未识别/非法）；与 vision msgs 的 string 同型，
/// 便于直接喂给 <Switch4 variable="{material_type}">
inline constexpr const char * kMaterialType = "material_type";
/// vision 发出的原始字符串（排障用）
inline constexpr const char * kMaterialRaw = "material_raw";

/// 当前位点几何信息
inline constexpr const char * kSlotId = "slot_id";
inline constexpr const char * kRow = "row";
inline constexpr const char * kCol = "col";
/// 第几次移动，1..12
inline constexpr const char * kVisitIndex = "visit_index";
/// 本次运行总位点数（=12）
inline constexpr const char * kSlotCount = "slot_count";

/// 下发 GOTO 之前已接收到的 vision 结果序号：WaitVisionResult 只认「比它新」的结果
inline constexpr const char * kVisionSeqBeforeMove = "vision_seq_before_move";
/// 本一位点开始时刻（秒，double），用于统计单点耗时
inline constexpr const char * kSlotStartTime = "slot_start_time";

/// vision 等待超时（秒，double）
inline constexpr const char * kVisionTimeout = "vision_timeout";

/// 运行期由 runner 注入的路径类参数
inline constexpr const char * kWarehouseYaml = "warehouse_yaml";
inline constexpr const char * kResultsCsv = "results_csv";

}  // namespace bb
}  // namespace aic_behavior_tree
