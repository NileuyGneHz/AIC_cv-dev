#include "aic_behavior_tree/material_types.hpp"

#include <algorithm>
#include <cctype>

namespace aic_behavior_tree
{
namespace
{

std::string trimCopy(const std::string & s)
{
  const auto not_space = [](unsigned char c) {return std::isspace(c) == 0;};
  auto begin = std::find_if(s.begin(), s.end(), not_space);
  auto end = std::find_if(s.rbegin(), s.rend(), not_space).base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

}  // namespace

int parseMaterialType(const std::string & raw)
{
  const std::string s = trimCopy(raw);
  // 严格单字符 "1".."4"：不接受 "01"、"1.0"、"1 "、空串
  if (s.size() != 1 || s[0] < '0' || s[0] > '9') {
    return 0;
  }
  const int v = s[0] - '0';
  return (v >= kMaterialTypeMin && v <= kMaterialTypeMax) ? v : 0;
}

std::string materialTypeName(int type)
{
  switch (type) {
    case 1: return "划痕";
    case 2: return "污渍";
    case 3: return "缺角";
    case 4: return "孔洞";
    default: return "unknown";
  }
}

}  // namespace aic_behavior_tree
