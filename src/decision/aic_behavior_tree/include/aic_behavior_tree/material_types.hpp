// 物料类型（1..4）的解析与命名。
//
// 「1=划痕 2=污渍 3=缺角 4=孔洞」是本版**暂名**（见 config/material_types.yaml），
// 赛题/裁判口径明确后只改这里与那份 YAML，行为树与话题都不动。
#pragma once

#include <string>

namespace aic_behavior_tree
{

/// 合法物料类型范围
inline constexpr int kMaterialTypeMin = 1;
inline constexpr int kMaterialTypeMax = 4;

/// 把 vision 发来的字符串解析成物料类型；非法（含空、多字符、越界）返回 0。
/// 会先去掉首尾空白；接受 "1".."4"，不接受 "01"/"1.0" 这类写法。
int parseMaterialType(const std::string & raw);

/// 类型 -> 中文名；未知返回 "unknown"
std::string materialTypeName(int type);

}  // namespace aic_behavior_tree
