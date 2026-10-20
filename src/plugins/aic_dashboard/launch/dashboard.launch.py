"""判分看板 + 端到端仿真：一条命令起齐「仿真 + 决策 + 判分可视化」。

用法：
    ros2 launch aic_dashboard dashboard.launch.py                     # 真值桩视觉
    ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad
    ros2 launch aic_dashboard dashboard.launch.py gui:=true once:=false

浏览器打开 http://127.0.0.1:8080 —— 相机流 + 12 库位绿/红 + 计分板单窗口，
OBS 录这个浏览器窗口即可满足「单窗口合成录制」硬指标（指标 4）。

判分器是纯旁路（只订阅不发布），关掉它链路照常跑；
它自己挂了也不影响仿真 —— 与 run_sim.sh 的链路隔离，方便排查。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("aic_gazebo_sim")

    score_node = Node(
        package="aic_dashboard", executable="score_node.py",
        name="aic_score_node", output="screen",
        parameters=[{
            "http_port": LaunchConfiguration("http_port"),
            "layout_yaml": LaunchConfiguration("layout_yaml"),
        }],
    )

    return LaunchDescription([
        # 仿真 + 决策原样透传（见 sim_decision.launch.py 的同名参数）
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("once", default_value="true"),
        DeclareLaunchArgument("vision_backend", default_value="gt"),
        DeclareLaunchArgument("vision_debug_dir", default_value=""),
        DeclareLaunchArgument("drop_rate", default_value="0.0"),
        DeclareLaunchArgument("invalid_rate", default_value="0.0"),
        # 看板自己的参数
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("layout_yaml", default_value="",
                              description="sim_layout.yaml 路径；空 = 用安装的 share 版"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, "launch", "sim_decision.launch.py")),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "once": LaunchConfiguration("once"),
                "vision_backend": LaunchConfiguration("vision_backend"),
                "vision_debug_dir": LaunchConfiguration("vision_debug_dir"),
                "drop_rate": LaunchConfiguration("drop_rate"),
                "invalid_rate": LaunchConfiguration("invalid_rate"),
            }.items(),
        ),
        score_node,
    ])
