#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 PLC 网关。

两种协议后端，状态机是同一套：

  backend:=modbus（默认）  网关当 Modbus TCP 从站，PLC 主站连 5020。
                            没有真 PLC 时用 plc_master_demo.py 当软主站。
  backend:=rsc             网关当 PLCnext RSC 客户端，连 vPLC 41100。
                            没有 vPLC 时加 rsc_dry_run:=true 走内存假 PLC。

默认观察模式：PLC 只读状态，可与决策层 bt_runner 同时跑。
要由 PLC 下发任务：allow_control:=true，并先停掉 bt_runner。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # ---- 协议选择 ----
        DeclareLaunchArgument("backend", default_value="modbus",
                              description="modbus = TCP 从站；rsc = PLCnext RSC 客户端"),
        DeclareLaunchArgument("allow_control", default_value="false",
                              description="true = PLC 可下发 GOTO/HANDLE（须停 bt_runner）"),
        DeclareLaunchArgument("n_materials", default_value="12"),

        # ---- Modbus 后端 ----
        DeclareLaunchArgument("port", default_value="5020",
                              description="Modbus TCP 端口（502 需 root，建议 5020）"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("unit_id", default_value="1"),

        # ---- RSC 后端 ----
        DeclareLaunchArgument("rsc_host", default_value="192.168.3.99",
                              description="vPLC 地址；算法在容器内时用 10.88.0.2"),
        DeclareLaunchArgument("rsc_port", default_value="41100"),
        DeclareLaunchArgument("rsc_user", default_value="admin"),
        DeclareLaunchArgument("rsc_password", default_value="plcnext"),
        DeclareLaunchArgument("rsc_map", default_value="",
                              description="标签表路径，空 = 包内默认 plc_rsc_map.yaml"),
        DeclareLaunchArgument("rsc_dry_run", default_value="false",
                              description="true = 内存假 PLC，不需要 vPLC（联调用）"),
    ]
    return LaunchDescription(args + [
        Node(
            package="aic_plc_gateway",
            executable="plc_gateway.py",
            name="plc_gateway",
            output="screen",
            parameters=[{
                "backend": LaunchConfiguration("backend"),
                "allow_control": LaunchConfiguration("allow_control"),
                "n_materials": LaunchConfiguration("n_materials"),
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "unit_id": LaunchConfiguration("unit_id"),
                "rsc_host": LaunchConfiguration("rsc_host"),
                "rsc_port": LaunchConfiguration("rsc_port"),
                "rsc_user": LaunchConfiguration("rsc_user"),
                "rsc_password": LaunchConfiguration("rsc_password"),
                "rsc_map": LaunchConfiguration("rsc_map"),
                "rsc_dry_run": LaunchConfiguration("rsc_dry_run"),
            }],
        ),
    ])
