cmake_minimum_required(VERSION 3.8)
project(aic_gazebo_sim)

find_package(ament_cmake REQUIRED)

# 世界 / 模型 / 配置 / launch
install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Python 节点（安装成可执行，ros2 run / launch 直接调）
install(PROGRAMS
  scripts/gantry_controller.py
  scripts/sim_vision_node.py
  scripts/sim_snapshot.py
  scripts/gen_world.py
  scripts/capture_storyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# 共用模块（被上面三个脚本 import，必须落在同一目录）
install(FILES
  scripts/sim_layout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
