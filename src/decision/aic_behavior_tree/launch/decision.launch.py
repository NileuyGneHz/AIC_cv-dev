# 决策层启动文件：只起 bt_runner（vision / control 的实现由各自模块负责）。
#
# 用法：
#   ros2 launch aic_behavior_tree decision.launch.py
#   ros2 launch aic_behavior_tree decision.launch.py once:=true results_csv:=/tmp/run.csv
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('aic_behavior_tree'), 'config', 'aic_decision_params.yaml'])

    params_file = LaunchConfiguration('params_file')
    once = LaunchConfiguration('once')
    results_csv = LaunchConfiguration('results_csv')
    vision_timeout_s = LaunchConfiguration('vision_timeout_s')
    vision_topic = LaunchConfiguration('vision_topic')
    control_task_topic = LaunchConfiguration('control_task_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='决策参数文件（话题名、超时、CSV 等）'),
        DeclareLaunchArgument(
            'once', default_value='false',
            description='true = 跑完一轮即退出（自验用）'),
        DeclareLaunchArgument(
            'results_csv', default_value='',
            description='逐位点结果 CSV 输出路径，留空则不落盘'),
        DeclareLaunchArgument(
            'vision_timeout_s', default_value='3.0',
            description='单个位点等待 vision 结果的超时（秒）'),
        DeclareLaunchArgument(
            'vision_topic', default_value='/vision/material_type',
            description='vision 结果话题'),
        DeclareLaunchArgument(
            'control_task_topic', default_value='/control/material_task',
            description='control 任务话题'),

        Node(
            package='aic_behavior_tree',
            executable='bt_runner',
            name='aic_decision_bt',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'once': ParameterValue(once, value_type=bool),
                    'results_csv': results_csv,
                    'vision_timeout_s': ParameterValue(vision_timeout_s, value_type=float),
                    'vision_topic': vision_topic,
                    'control_task_topic': control_task_topic,
                },
            ],
        ),
    ])
