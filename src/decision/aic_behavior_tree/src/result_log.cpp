#include "aic_behavior_tree/result_log.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

#include "aic_behavior_tree/material_types.hpp"

namespace aic_behavior_tree
{
namespace
{

std::string csvEscape(const std::string & field)
{
  if (field.find_first_of(",\"\n") == std::string::npos) {
    return field;
  }
  std::string out = "\"";
  for (const char c : field) {
    if (c == '"') {
      out += "\"\"";
    } else {
      out += c;
    }
  }
  out += "\"";
  return out;
}

}  // namespace

ResultLog::ResultLog(const std::string & csv_path)
: csv_path_(csv_path)
{
}

void ResultLog::clear()
{
  std::lock_guard<std::mutex> lock(mtx_);
  rows_.clear();
  header_written_ = false;
}

void ResultLog::append(const SlotOutcome & outcome)
{
  std::lock_guard<std::mutex> lock(mtx_);
  rows_.push_back(outcome);
  if (!csv_path_.empty()) {
    writeHeaderIfNeeded();
    writeRow(outcome);
  }
}

std::vector<SlotOutcome> ResultLog::rows() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return rows_;
}

void ResultLog::writeHeaderIfNeeded()
{
  if (header_written_) {
    return;
  }
  std::ofstream ofs(csv_path_, std::ios::out | std::ios::trunc);
  if (!ofs) {
    csv_path_.clear();   // 写不了就退化成「只在内存里」的模式，不影响比赛主流程
    return;
  }
  ofs << "visit_index,slot_id,row,col,material_type,material_name,material_raw,"
      << "result,elapsed_s,note\n";
  header_written_ = true;
}

void ResultLog::writeRow(const SlotOutcome & o)
{
  std::ofstream ofs(csv_path_, std::ios::out | std::ios::app);
  if (!ofs) {
    return;
  }
  ofs << o.visit_index << "," << o.slot_id << "," << o.row << "," << o.col << ","
      << o.material_type << "," << csvEscape(materialTypeName(o.material_type)) << ","
      << csvEscape(o.material_raw) << "," << csvEscape(o.result) << ","
      << std::fixed << std::setprecision(3) << o.elapsed_s << ","
      << csvEscape(o.note) << "\n";
}

std::string ResultLog::summary() const
{
  std::lock_guard<std::mutex> lock(mtx_);

  int ok = 0;
  int timeout = 0;
  int unknown = 0;
  int per_type[5] = {0, 0, 0, 0, 0};
  double elapsed_sum = 0.0;

  std::ostringstream oss;
  oss << "===== 本次运行逐位点结果 =====\n";
  for (const auto & r : rows_) {
    if (r.result == "OK") {
      ++ok;
    } else if (r.result == "TIMEOUT") {
      ++timeout;
    } else {
      ++unknown;
    }
    if (r.material_type >= 1 && r.material_type <= 4) {
      ++per_type[r.material_type];
    }
    elapsed_sum += r.elapsed_s;

    oss << "  #" << std::setw(2) << std::setfill('0') << r.visit_index << " slot"
        << std::setw(2) << std::setfill('0') << r.slot_id
        << " (row" << r.row << ",col" << r.col << ") type " << r.material_type
        << "(" << materialTypeName(r.material_type) << ")"
        << " result=" << r.result
        << " dt=" << std::fixed << std::setprecision(3) << r.elapsed_s << "s"
        << (r.note.empty() ? "" : (" note=" + r.note)) << "\n";
  }
  oss << "===== 汇总 =====\n";
  oss << "  位点数：" << rows_.size() << "（OK " << ok << " / TIMEOUT " << timeout
      << " / 其他 " << unknown << "）\n";
  oss << "  按类型：划痕 " << per_type[1] << " / 污渍 " << per_type[2]
      << " / 缺角 " << per_type[3] << " / 孔洞 " << per_type[4] << "\n";
  if (!rows_.empty()) {
    oss << "  平均单点耗时：" << std::fixed << std::setprecision(3)
        << (elapsed_sum / static_cast<double>(rows_.size())) << " s\n";
  }
  if (!csv_path_.empty()) {
    oss << "  CSV：" << csv_path_ << "\n";
  }
  return oss.str();
}

}  // namespace aic_behavior_tree
