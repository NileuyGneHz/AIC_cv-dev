#include "aic_warehouse/warehouse_layout.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aic_warehouse
{
namespace
{

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

/// 把方向修正为「指向行内」：出发点在右端时不能再往右扫。
/// 返回修正后的方向（+1 右 / -1 左）。
int normalizeDirection(int dir, int start_col, int cols, bool & corrected)
{
  corrected = false;
  if (cols <= 1) {
    return 1;
  }
  if (start_col == cols && dir > 0) {
    corrected = true;
    return -1;
  }
  if (start_col == 1 && dir < 0) {
    corrected = true;
    return 1;
  }
  return dir;
}

}  // namespace

std::string Slot::describe() const
{
  std::ostringstream oss;
  oss << "slot " << std::setw(2) << std::setfill('0') << id
      << " (row" << row << ",col" << col << ")";
  return oss.str();
}

LayoutConfig WarehouseLayout::loadConfigFile(const std::string & path)
{
  LayoutConfig cfg;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("无法读取货架布局配置 '" + path + "': " + e.what());
  }
  if (root["rows"]) {
    cfg.rows = root["rows"].as<int>();
  }
  if (root["cols"]) {
    cfg.cols = root["cols"].as<int>();
  }
  if (root["start_corner"]) {
    cfg.start_corner = root["start_corner"].as<std::string>();
  }
  if (root["first_direction"]) {
    cfg.first_direction = root["first_direction"].as<std::string>();
  }
  if (root["frame_note"]) {
    cfg.frame_note = root["frame_note"].as<std::string>();
  }
  if (cfg.rows < 1 || cfg.cols < 1) {
    throw std::runtime_error("货架布局非法：rows/cols 必须 >= 1");
  }
  return cfg;
}

WarehouseLayout WarehouseLayout::fromConfig(const LayoutConfig & cfg)
{
  if (cfg.rows < 1 || cfg.cols < 1) {
    throw std::runtime_error("货架布局非法：rows/cols 必须 >= 1");
  }

  WarehouseLayout layout;
  layout.cfg_ = cfg;

  // 1) 几何表：id = (row-1)*cols + col
  layout.slots_.reserve(static_cast<std::size_t>(cfg.rows * cfg.cols));
  for (int row = 1; row <= cfg.rows; ++row) {
    for (int col = 1; col <= cfg.cols; ++col) {
      layout.slots_.push_back(Slot{(row - 1) * cfg.cols + col, row, col});
    }
  }

  // 2) 访问顺序：蛇形（沿行扫完 -> 换行 -> 反向）
  const bool start_bottom = contains(cfg.start_corner, "bottom");
  const bool start_right = contains(cfg.start_corner, "right");
  const int start_row = start_bottom ? 1 : cfg.rows;
  const int start_col = start_right ? cfg.cols : 1;
  const int row_step = start_bottom ? 1 : -1;

  int dir = 0;
  if (cfg.first_direction == "left") {
    dir = -1;
  } else if (cfg.first_direction == "right") {
    dir = 1;
  } else {
    // auto：从出发点往「另一侧」扫
    dir = (start_col == cfg.cols) ? -1 : 1;
  }
  bool corrected = false;
  dir = normalizeDirection(dir, start_col, cfg.cols, corrected);
  if (corrected) {
    std::cerr << "[aic_warehouse] 首次扫行方向与出发点冲突，已自动改为 "
              << (dir > 0 ? "right" : "left") << "（请检查 first_direction 配置）\n";
  }

  layout.visit_order_.reserve(layout.slots_.size());
  int col_start = start_col;
  int row = start_row;
  for (int r = 0; r < cfg.rows; ++r) {
    for (int c = 0; c < cfg.cols; ++c) {
      const int col = col_start + c * dir;
      layout.visit_order_.push_back(Slot{(row - 1) * cfg.cols + col, row, col});
    }
    // 下一行从「本行的终点列」开始扫，否则会跑出行外（第一版就栽在这里）
    col_start = col_start + (cfg.cols - 1) * dir;
    dir = -dir;
    row += row_step;
  }

  return layout;
}

WarehouseLayout WarehouseLayout::fromYamlFile(const std::string & path)
{
  return fromConfig(loadConfigFile(path));
}

std::optional<Slot> WarehouseLayout::slotById(int id) const
{
  if (id < 1 || id > slotCount()) {
    return std::nullopt;
  }
  return slots_[static_cast<std::size_t>(id - 1)];
}

std::string WarehouseLayout::describe() const
{
  const int width = std::max(3, static_cast<int>(std::to_string(slotCount()).size()) + 1);

  std::ostringstream oss;
  oss << "货架布局：" << cfg_.rows << " 行 x " << cfg_.cols << " 列 = " << slotCount()
      << " 个库位 | 出发点 " << cfg_.start_corner
      << " | 首次扫行 " << cfg_.first_direction << "\n";
  if (!cfg_.frame_note.empty()) {
    oss << "视角说明：" << cfg_.frame_note << "\n";
  }

  // 网格 A：访问次序（第几次移动）
  oss << "格内数字 = 第几次移动：\n";
  for (int row = cfg_.rows; row >= 1; --row) {
    oss << "  row" << row << " |";
    for (int col = 1; col <= cfg_.cols; ++col) {
      const int id = (row - 1) * cfg_.cols + col;
      int visit = -1;
      for (std::size_t i = 0; i < visit_order_.size(); ++i) {
        if (visit_order_[i].id == id) {
          visit = static_cast<int>(i) + 1;
          break;
        }
      }
      oss << " " << std::setw(width) << std::setfill(' ') << visit;
    }
    oss << "  |\n";
  }

  // 网格 B：几何库位号
  oss << "格内数字 = 几何库位号：\n";
  for (int row = cfg_.rows; row >= 1; --row) {
    oss << "  row" << row << " |";
    for (int col = 1; col <= cfg_.cols; ++col) {
      oss << " " << std::setw(width) << std::setfill(' ')
          << ((row - 1) * cfg_.cols + col);
    }
    oss << "  |\n";
  }
  oss << "         ";
  for (int col = 1; col <= cfg_.cols; ++col) {
    oss << " " << std::setw(width) << std::setfill(' ') << ("c" + std::to_string(col));
  }
  oss << "\n";
  return oss.str();
}

std::string WarehouseLayout::describeVisitOrder() const
{
  std::ostringstream oss;
  oss << "访问顺序（共 " << visit_order_.size() << " 次移动）：\n";
  for (std::size_t i = 0; i < visit_order_.size(); ++i) {
    oss << "  " << std::setw(2) << std::setfill('0') << (i + 1) << ": "
        << visit_order_[i].describe() << "\n";
  }
  return oss.str();
}

// ---------------------------------------------------------------- cursor

WarehouseCursor::WarehouseCursor(const WarehouseLayout * layout)
: layout_(layout) {}

void WarehouseCursor::bind(const WarehouseLayout * layout)
{
  layout_ = layout;
  reset();
}

void WarehouseCursor::reset()
{
  next_ = 0;
  visited_ = 0;
}

bool WarehouseCursor::hasNext() const
{
  return layout_ != nullptr && next_ < layout_->visitOrder().size();
}

const Slot * WarehouseCursor::advance()
{
  if (!hasNext()) {
    return nullptr;
  }
  const Slot & slot = layout_->visitOrder()[next_];
  ++next_;
  ++visited_;
  return &slot;
}

int WarehouseCursor::total() const
{
  return layout_ == nullptr ? 0 : layout_->slotCount();
}

}  // namespace aic_warehouse
