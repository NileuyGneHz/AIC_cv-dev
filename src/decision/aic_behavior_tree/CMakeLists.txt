cmake_minimum_required(VERSION 3.8)
project(aic_behavior_tree)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(aic_msgs REQUIRED)
find_package(aic_warehouse REQUIRED)

# 决策核心：桥接 + 行为树节点（库，便于将来加测试 / 复用）
add_library(aic_decision_core SHARED
  src/material_types.cpp
  src/vision_client.cpp
  src/control_bridge.cpp
  src/result_log.cpp
  src/nodes.cpp
  src/register_nodes.cpp
)
target_include_directories(aic_decision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aic_decision_core PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${aic_msgs_TARGETS}
  aic_warehouse::aic_warehouse
  behaviortree_cpp::behaviortree_cpp
)
# 行为树执行器（进程入口）
add_executable(bt_runner src/bt_runner.cpp)
target_link_libraries(bt_runner PRIVATE aic_decision_core ament_index_cpp::ament_index_cpp)

install(TARGETS aic_decision_core bt_runner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(aic_decision_core)
ament_export_dependencies(rclcpp std_msgs aic_msgs aic_warehouse behaviortree_cpp)

ament_package()
