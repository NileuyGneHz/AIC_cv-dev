#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 PLC 网关（Modbus TCP 从站）。

默认观察模式：PLC 只读状态，可与决策层 bt_runner 同时跑。
要由 PLC 下发任务：allow_control:=true，并先停掉 bt_runner。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("port", default_value="5020",
                              description="Modbus TCP 端口（502 需 root，建议 5020）"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("unit_id", default_value="1"),
        DeclareLaunchArgument("allow_control", default_value="false",
                              description="true = PLC 可下发 GOTO/HANDLE（须停 bt_runner）"),
        DeclareLaunchArgument("n_materials", default_value="12"),
    ]
    return LaunchDescription(args + [
        Node(
            package="aic_plc_gateway",
            executable="plc_gateway.py",
            name="plc_gateway",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "unit_id": LaunchConfiguration("unit_id"),
                "allow_control": LaunchConfiguration("allow_control"),
                "n_materials": LaunchConfiguration("n_materials"),
            }],
        ),
    ])
