// 小工具：打印货架编号与访问顺序，用于现场核对「12 次移动」的路径。
//
// 用法：
//   ros2 run aic_warehouse print_path                 # 用包内默认 config/warehouse_layout.yaml
//   ros2 run aic_warehouse print_path /path/to.yaml   # 用指定配置
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>

#include "aic_warehouse/warehouse_layout.hpp"

int main(int argc, char ** argv)
{
  std::string yaml_path;
  if (argc > 1) {
    yaml_path = argv[1];
  } else {
    try {
      yaml_path = ament_index_cpp::get_package_share_directory("aic_warehouse") +
        "/config/warehouse_layout.yaml";
    } catch (const std::exception & e) {
      std::cerr << "[print_path] 找不到 aic_warehouse 包共享目录：" << e.what() << "\n";
      return 2;
    }
  }

  try {
    const auto layout = aic_warehouse::WarehouseLayout::fromYamlFile(yaml_path);
    std::cout << "[print_path] 配置：" << yaml_path << "\n\n";
    std::cout << layout.describe() << "\n";
    std::cout << layout.describeVisitOrder();
  } catch (const std::exception & e) {
    std::cerr << "[print_path] 失败：" << e.what() << "\n";
    return 1;
  }
  return 0;
}
