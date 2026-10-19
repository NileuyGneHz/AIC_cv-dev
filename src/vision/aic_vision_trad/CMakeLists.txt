cmake_minimum_required(VERSION 3.8)
project(aic_vision_trad)

find_package(ament_cmake REQUIRED)

# ROS 节点与离线工具
install(PROGRAMS
  scripts/vision_node.py
  scripts/bench_offline.py
  DESTINATION lib/${PROJECT_NAME}
)

# 共用算法模块（被上面两个脚本 import，必须落在同一目录）
install(FILES
  scripts/plate_detector.py
  scripts/defect_classifier.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
