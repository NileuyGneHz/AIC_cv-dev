"""单独起传统视觉节点（不启动 Gazebo），用于对着已有图像话题调参。

用法：
    ros2 launch aic_vision_trad vision.launch.py
    ros2 launch aic_vision_trad vision.launch.py debug_dir:=logs/vision_debug
    ros2 launch aic_vision_trad vision.launch.py image_topic:=/camera/image_raw
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image"),
        DeclareLaunchArgument("trigger_topic", default_value="/vision/trigger"),
        DeclareLaunchArgument("result_topic", default_value="/vision/material_type"),
        DeclareLaunchArgument("debug_dir", default_value="",
                              description="非空则每判定一次存一张标注图"),
        Node(
            package="aic_vision_trad",
            executable="vision_node.py",
            name="vision_trad_node",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "trigger_topic": LaunchConfiguration("trigger_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "debug_dir": LaunchConfiguration("debug_dir"),
            }],
        ),
    ])
