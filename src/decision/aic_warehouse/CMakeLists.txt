cmake_minimum_required(VERSION 3.8)
project(aic_warehouse)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(aic_warehouse SHARED
  src/warehouse_layout.cpp
)
target_include_directories(aic_warehouse PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aic_warehouse PUBLIC yaml-cpp)

# 命令行小工具：打印货架编号与访问顺序，便于现场核对「移动路径」是否与赛位一致
add_executable(print_path src/print_path_main.cpp)
target_link_libraries(print_path PRIVATE aic_warehouse ament_index_cpp::ament_index_cpp)
install(TARGETS print_path DESTINATION lib/${PROJECT_NAME})

install(TARGETS aic_warehouse
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_warehouse_layout test/test_warehouse_layout.cpp)
  target_link_libraries(test_warehouse_layout aic_warehouse)
endif()

ament_package()
