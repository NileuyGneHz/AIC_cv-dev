cmake_minimum_required(VERSION 3.8)
project(aic_dashboard)

find_package(ament_cmake REQUIRED)

# Python 节点（安装成可执行，ros2 run / launch 直接调）
install(PROGRAMS
  scripts/score_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
