// 货架 / 库位模型：几何编号 + 巡视访问顺序。
//
// 赛题口径（2026-10-15 与用户确认的初版理解，若现场不符只改 YAML）：
//   货架 3 行 x 4 列 = 12 个库位；从右下角出发，沿行扫 4 格 -> 上行 1 格 -> 反向扫 4 格
//   -> 上行 1 格 -> 再反向扫 4 格，共 12 个位点，即 12 次移动。
//
// 约定：本类只产出「几何编号（row/col/id）」与「访问顺序」，不涉及任何坐标/行程 —— 那属于 control。
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace aic_warehouse
{

/// 单个库位。行/列均 1-based：row 1 = 最下一排，col 1 = 最左一列。
struct Slot
{
  int id{0};
  int row{0};
  int col{0};

  std::string describe() const;
};

/// 货架布局配置（YAML 可覆盖）
struct LayoutConfig
{
  int rows{3};
  int cols{4};
  /// 出发点：bottom_right（默认，右下角）/ bottom_left / top_right / top_left
  std::string start_corner{"bottom_right"};
  /// 首次扫行方向：auto（自动取「与出发点相反的一侧」）/ left / right
  std::string first_direction{"auto"};
  /// 视角说明（仅存档，不参与计算）
  std::string frame_note{};
};

class WarehouseLayout
{
public:
  /// 读取 YAML 配置（缺键用默认值；文件不存在抛 std::runtime_error）
  static LayoutConfig loadConfigFile(const std::string & path);
  /// 由配置生成布局与访问顺序
  static WarehouseLayout fromConfig(const LayoutConfig & cfg);
  /// 一步到位：读文件 + 生成
  static WarehouseLayout fromYamlFile(const std::string & path);

  int rows() const {return cfg_.rows;}
  int cols() const {return cfg_.cols;}
  int slotCount() const {return static_cast<int>(slots_.size());}
  const LayoutConfig & config() const {return cfg_;}

  /// 几何表，按 id 升序
  const std::vector<Slot> & slots() const {return slots_;}
  /// 访问顺序（长度 = slotCount），第 i 项 = 第 i+1 次移动的位点
  const std::vector<Slot> & visitOrder() const {return visit_order_;}

  /// 1-based 查库位；越界返回 std::nullopt
  std::optional<Slot> slotById(int id) const;

  /// ASCII 网格：格子里打印「访问次序」，用于现场核对路径
  std::string describe() const;
  /// 单行：访问顺序明细
  std::string describeVisitOrder() const;

private:
  LayoutConfig cfg_;
  std::vector<Slot> slots_;
  std::vector<Slot> visit_order_;
};

/// 巡视游标：负责「移动 12 次」的推进与复位。
class WarehouseCursor
{
public:
  explicit WarehouseCursor(const WarehouseLayout * layout = nullptr);

  void bind(const WarehouseLayout * layout);
  void reset();
  bool hasNext() const;
  /// 返回下一个位点并推进；无剩余返回 nullptr
  const Slot * advance();
  /// 已推进次数 0..total
  int visitIndex() const {return visited_;}
  int total() const;

private:
  const WarehouseLayout * layout_{nullptr};
  std::size_t next_{0};
  int visited_{0};
};

}  // namespace aic_warehouse
