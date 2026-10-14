// 位点结果记录：支撑「逐一播报检测结果」与赛后可复盘的 CSV。
//
// 比赛真实设备路径要求手持机位跟进、逐一播报检测结果；因此每个位点落一行：
//   访问次序 / 库位号 / 行 / 列 / 物料类型 / 结果（OK|TIMEOUT|UNKNOWN）/ 耗时 / 备注
#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace aic_behavior_tree
{

struct SlotOutcome
{
  int visit_index{0};
  int slot_id{0};
  int row{0};
  int col{0};
  int material_type{0};
  std::string material_raw;
  std::string result;      ///< OK / TIMEOUT / UNKNOWN
  double elapsed_s{0.0};
  std::string note;
};

class ResultLog
{
public:
  /// csv_path 为空则不落盘（只留在内存里做汇总）
  explicit ResultLog(const std::string & csv_path = "");

  /// 追加一行；同时按需写入 CSV（每次 append 落盘一次，避免比赛中途崩溃丢数据）
  void append(const SlotOutcome & outcome);

  void clear();
  std::vector<SlotOutcome> rows() const;

  /// 多行文本汇总（逐位点 + 统计），用于运行结束打印
  std::string summary() const;

  const std::string & csvPath() const {return csv_path_;}

private:
  void writeHeaderIfNeeded();
  void writeRow(const SlotOutcome & outcome);

  mutable std::mutex mtx_;
  std::string csv_path_;
  std::vector<SlotOutcome> rows_;
  bool header_written_{false};
};

}  // namespace aic_behavior_tree
