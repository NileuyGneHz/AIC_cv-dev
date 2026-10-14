// 货架布局 / 访问顺序单元测试：把「12 次移动」的形状钉死在测试里。
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "aic_warehouse/warehouse_layout.hpp"

namespace
{

std::vector<int> idsOf(const std::vector<aic_warehouse::Slot> & slots)
{
  std::vector<int> ids;
  ids.reserve(slots.size());
  for (const auto & s : slots) {
    ids.push_back(s.id);
  }
  return ids;
}

aic_warehouse::LayoutConfig makeConfig(
  const std::string & corner, const std::string & dir = "auto")
{
  aic_warehouse::LayoutConfig cfg;
  cfg.rows = 3;
  cfg.cols = 4;
  cfg.start_corner = corner;
  cfg.first_direction = dir;
  return cfg;
}

}  // namespace

TEST(WarehouseLayout, GeometryIsRowMajorTwelveSlots)
{
  const auto layout = aic_warehouse::WarehouseLayout::fromConfig(makeConfig("bottom_right"));
  ASSERT_EQ(layout.slotCount(), 12);
  EXPECT_EQ(idsOf(layout.slots()), (std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));

  const auto s = layout.slotById(7);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->row, 2);
  EXPECT_EQ(s->col, 3);

  EXPECT_FALSE(layout.slotById(0).has_value());
  EXPECT_FALSE(layout.slotById(13).has_value());
}

TEST(WarehouseLayout, SnakeFromBottomRightVisitsTwelveSlotsExactlyOnce)
{
  const auto layout = aic_warehouse::WarehouseLayout::fromConfig(makeConfig("bottom_right"));
  const auto & order = layout.visitOrder();
  ASSERT_EQ(order.size(), 12u);

  // 右下角出发 -> 向左扫 4 -> 上行 1 -> 向右扫 4 -> 上行 1 -> 向左扫 4
  EXPECT_EQ(idsOf(order), (std::vector<int>{4, 3, 2, 1, 5, 6, 7, 8, 12, 11, 10, 9}));

  // 12 个位点不重不漏
  auto ids = idsOf(order);
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(ids, (std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));

  // 相邻两次移动只差一格（同行走一步，或换行）
  for (std::size_t i = 1; i < order.size(); ++i) {
    const int drow = std::abs(order[i].row - order[i - 1].row);
    const int dcol = std::abs(order[i].col - order[i - 1].col);
    EXPECT_LE(drow, 1);
    EXPECT_LE(dcol, 3);
  }
}

TEST(WarehouseLayout, MirroredCornerFlipsThePath)
{
  const auto layout = aic_warehouse::WarehouseLayout::fromConfig(makeConfig("bottom_left"));
  EXPECT_EQ(idsOf(layout.visitOrder()),
    (std::vector<int>{1, 2, 3, 4, 8, 7, 6, 5, 9, 10, 11, 12}));
}

TEST(WarehouseLayout, ConflictingDirectionIsAutoCorrected)
{
  // 出发点在右下角却要求向右扫 -> 自动纠正为向左，不抛异常
  const auto layout = aic_warehouse::WarehouseLayout::fromConfig(
    makeConfig("bottom_right", "right"));
  EXPECT_EQ(idsOf(layout.visitOrder()),
    (std::vector<int>{4, 3, 2, 1, 5, 6, 7, 8, 12, 11, 10, 9}));
}

TEST(WarehouseCursor, AdvancesTwelveTimesThenStops)
{
  const auto layout = aic_warehouse::WarehouseLayout::fromConfig(makeConfig("bottom_right"));
  aic_warehouse::WarehouseCursor cursor(&layout);
  EXPECT_EQ(cursor.total(), 12);
  EXPECT_TRUE(cursor.hasNext());

  int count = 0;
  while (cursor.hasNext()) {
    const auto * slot = cursor.advance();
    ASSERT_NE(slot, nullptr);
    ++count;
    EXPECT_EQ(cursor.visitIndex(), count);
  }
  EXPECT_EQ(count, 12);
  EXPECT_EQ(cursor.advance(), nullptr);
  EXPECT_FALSE(cursor.hasNext());

  cursor.reset();
  EXPECT_TRUE(cursor.hasNext());
  EXPECT_EQ(cursor.visitIndex(), 0);
}
