"""端到端仿真：Gazebo 场景 + 龙门控制桩 + ground-truth 视觉桩 + 决策行为树。

用法：
    ros2 launch aic_gazebo_sim sim_decision.launch.py                    # 跑一轮（once=true）
    ros2 launch aic_gazebo_sim sim_decision.launch.py once:=false        # 常驻，多轮
    ros2 launch aic_gazebo_sim sim_decision.launch.py gui:=true          # 带 GUI 看画面
    ros2 launch aic_gazebo_sim sim_decision.launch.py drop_rate:=0.15     # 注入识别失败

预期：12 个位点依次 GOTO -> 到位 -> 视觉回 type -> HANDLE，结果写 logs/sim_results_*.csv。
决策参数见 aic_behavior_tree/config/aic_decision_params.yaml（话题名两边必须一致）。
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("aic_gazebo_sim")
    bt_share = get_package_share_directory("aic_behavior_tree")
    params_file = os.path.join(bt_share, "config", "aic_decision_params.yaml")

    once = LaunchConfiguration("once")
    results_csv = LaunchConfiguration("results_csv")

    decision_node = Node(package="aic_behavior_tree", executable="bt_runner",
                         name="aic_decision_bt", output="screen",
                         parameters=[params_file, {
                             "once": once,
                             "results_csv": results_csv,
                             "vision_timeout_s": LaunchConfiguration("vision_timeout_s"),
                             # 仿真 control 会发 /control/ready；打开握手，避免首条 GOTO 丢
                             "control_wait_ready": True,
                             # 坑：决策常驻要的是 auto_restart，不是 once:=false ——
                             # auto_restart 缺省 false，只给 once:=false 时一轮结束照样退出，
                             # 触发下面的 Shutdown 把整个仿真一起关掉（实测 --loop 因此失效）。
                             "auto_restart": LaunchConfiguration("auto_restart"),
                         }])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("once", default_value="true",
                              description="true = 跑完一轮巡视即退出（自验用）"),
        DeclareLaunchArgument("auto_restart", default_value="false",
                              description="true = 一轮结束后决策自动重开巡视（常驻多轮；与 once:=false 连用）"),
        DeclareLaunchArgument(
            "results_csv",
            default_value="logs/sim_results_{}.csv".format(time.strftime("%Y%m%d_%H%M%S")),
            description="决策结果 CSV（相对工作区）"),
        DeclareLaunchArgument("drop_rate", default_value="0.0",
                              description="视觉桩模拟识别失败的概率"),
        DeclareLaunchArgument("invalid_rate", default_value="0.0",
                              description="视觉桩返回非法值（\"9\"）的概率"),
        # 坑：WaitVisionResult 的窗口是「GOTO 下发」那一刻开始计时的，不是「到位」开始。
        #     而控制器是 FIFO —— GOTO(N+1) 下发时，队列里还压着上一个位点的 HANDLE。
        #     窗口必须覆盖「HANDLE + GOTO + 视觉延迟」：
        #       只做「下探-抬升」的 HANDLE ≈ 1.1 s -> 3.0 就够但已经很紧
        #       加上「搬到码放格 + 放下」（M2）后 HANDLE ≈ 4.3 s，GOTO ≈ 1.5 s -> 5.0 不够
        #     实测（M2 完整搬运，真视觉 12 位点）：成功位点的等待时长落在 5.6~7.9 s，
        #     8.0 只剩 0.1 s 余量，于是 12 位点里稳定丢 3 个（6/10/12）。给到 12.0。
        #     窗口偏小的后果不只是「慢」，更是**迟到的结果被下一个位点误收**
        #     （类型错位，结果表还显示 OK）。所以这里留足余量。
        #     代价：真失败时要空等 12 s。真机 HANDLE 不含搬运（夹爪一开一合），
        #     实际参数应以真机节拍重定，这里只是仿真侧的自验配置。
        DeclareLaunchArgument("vision_timeout_s", default_value="12.0",
                              description="单个位点等识别结果的上限（须 > HANDLE + GOTO + 视觉延迟）"),
        DeclareLaunchArgument("vision_backend", default_value="gt",
                              description="gt = ground-truth 桩；trad = 传统视觉（看相机图）"),
        DeclareLaunchArgument("vision_debug_dir", default_value="",
                              description="trad 后端保存调试图的目录（空 = 不存）"),
        # 坑：决策发完最后一个 HANDLE 就退出，但那是 fire-and-forget ——
        #     控制器 FIFO 里还压着这条 HANDLE（含夹持 + 搬运 ≈5 s）。
        #     若 OnProcessExit 立刻 Shutdown，最后一块料会**卡在夹具里没放下**
        #     （实测 12 次 attach 只有 11 次：第 12 块铁片始终留在货架上）。
        #     所以退出后先等这段宽限期把队列排空，再关仿真。
        DeclareLaunchArgument("shutdown_grace_s", default_value="12.0",
                              description="决策退出后等控制队列排空再关仿真（秒）"),

        # 仿真本体 + 控制/视觉桩
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, "launch", "sim.launch.py")),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "drop_rate": LaunchConfiguration("drop_rate"),
                "invalid_rate": LaunchConfiguration("invalid_rate"),
                "vision_backend": LaunchConfiguration("vision_backend"),
                "vision_debug_dir": LaunchConfiguration("vision_debug_dir"),
            }.items(),
        ),

        # 决策行为树
        decision_node,
        # 决策跑完退出后一并关掉仿真（自验友好）；常驻多轮时用 once:=false，不会触发。
        # 注意这里不是立刻 Shutdown，而是先等 shutdown_grace_s —— 见参数处的注释。
        RegisterEventHandler(OnProcessExit(
            target_action=decision_node,
            on_exit=[TimerAction(
                period=LaunchConfiguration("shutdown_grace_s"),
                actions=[Shutdown(reason="decision 已退出，且控制队列已排空")])])),
    ])
