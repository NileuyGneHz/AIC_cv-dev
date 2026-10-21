cmake_minimum_required(VERSION 3.8)
project(aic_plc_gateway)

find_package(ament_cmake REQUIRED)

# Python 节点（安装成可执行，ros2 run / launch 直接调）
install(PROGRAMS
  scripts/plc_gateway.py
  scripts/plc_master_demo.py
  scripts/rsc_link.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# rsc_link.py 会在 share/aic_plc_gateway/config/ 下找标签表；不装 config 的话
# 只有从源码目录直接跑才碰巧能用，ros2 run 起来会找不到 map。
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
