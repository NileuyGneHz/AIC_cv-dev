#!/usr/bin/python3
"""从 config/sim_layout.yaml 生成 worlds/aic_warehouse.sdf（M3：官方平台对齐版）。

用法：
    /usr/bin/python3 scripts/gen_world.py                # 覆盖写 worlds/aic_warehouse.sdf

M3 与 M2 的区别（对齐官方 VirtualNext 平台，参考资料 docs/vnext_pdf/manual.txt）：
    场景 = 底板 + 立柱式龙门 + 放置货架（4 类型列 x 3 层，每格一块）
           + 置物台（取料源）+ 来料架（3 行 x 4 列，12 块物料开局铺在格位上）。
    物料不再烘在取料库位，而是运行时由 gantry_controller 逐块 set_pose 传送到
    置物台 —— 但 12 块物料仍必须在世界 SDF 里就存在：DetachableJoint 插件
    （挂龙门、child_model=material_NN）在世界加载时解析 child，动态 create
    的实体绑不上（M2 实测 child 找不到时插件直接失效）。

物料缺陷用几何体表达（不贴纹理）：
    1 划痕 = 细长深色薄条     2 污渍 = 扁圆深色斑
    3 缺角 = 边缘一块「缺口」  4 孔洞 = 小深色圆孔
"""
import argparse
import math
import os
import random
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

TYPE_NAMES = {1: "scratch", 2: "stain", 3: "chip", 4: "hole"}

# 缺陷几何的随机范围（只在 materials.jitter_seed > 0 时生效）。
# 尺寸是「相对 M2 整定值」的倍数，位置按下面的约束抽：
#   盘内缺陷（划痕/污渍/孔洞）整体留在 roi_frac=0.93r 之内 —— 缺陷被掩膜切掉一半
#   会让 frac 不可控（坑 18 的同类问题）；
#   缺角（chip）必须压在盘缘，径向取 0.90~0.96r，否则视觉上就不是「缺角」。
JITTER = {
    1: {"len": (0.85, 1.15), "wid": (0.85, 1.20)},   # 划痕：长宽微变
    2: {"radius": (0.85, 1.20)},                     # 污渍：斑半径微变
    3: {"size": (0.85, 1.20)},                       # 缺角：缺口边长微变
    4: {"radius": (0.85, 1.20)},                     # 孔洞：孔径微变
}

HEADER = """<?xml version="1.0" ?>
<!--
  自动生成，勿手改：由 scripts/gen_world.py 从 config/sim_layout.yaml 生成。
  AIC 视觉逻辑算法应用赛 · M3 官方平台对齐版（Ignition Fortress）：
  底板 + 分拣货架（4 类型列 x 3 层）+ 置物台 + 来料架（3 行 x 4 列）+ 立柱式三轴龙门（吸盘）
-->
<sdf version="1.6">
  <world name="aic_warehouse">
    <physics name="4ms" type="dartsim">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.60 0.60 0.63 1</ambient>
      <background>0.72 0.75 0.80 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- 相机等传感器的渲染系统必须显式加载（Fortress 与 classic 的差别之一） -->
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- 物理系统必须显式加载：本机 Fortress 在有 <physics> 段时并不会自动装 physics
         system，且引擎库位于 ign-physics-5/engine-plugins、默认搜索路径找不到。
         漏写这一段的表现极具迷惑性——世界能起、相机能出图、时钟在走，但「什么都不动」。 -->
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics">
      <engine>ignition-physics-dartsim-plugin</engine>
    </plugin>

    <!-- SceneBroadcaster：/world/<w>/state 服务与 pose/info 话题靠它；
         不加载时 `ign model -l` 查位姿会超时，TF/位姿类工具也全瞎 -->
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <!-- UserCommands：提供 /world/<w>/create、/remove、/set_pose 三个服务。
         坑：不加载它时这两个服务压根不存在（`ign service -l` 只有 26 条，
         没有 set_pose），但客户端不会立刻报「无此服务」，而是等满 timeout
         后报 "Service call timed out" —— 极易误判成"动态模型不能 set_pose"。
         本项目的用途：M4 上料（把 material_NN 从来料架格位传送到置物台）
         与看板自由摄像机机位切换。 -->
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>

    <light type="directional" name="sun">
      <pose>0 0 3 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.55 0.55 0.55 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>-0.25 0.20 -0.95</direction>
    </light>

    <!-- 货架正上方的补光，保证俯视相机下铁片表面照度均匀 -->
    <light type="point" name="rack_fill">
      <pose>0 0 1.6 0 0 0</pose>
      <diffuse>0.42 0.42 0.42 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <attenuation>
        <range>4</range>
        <constant>1.0</constant>
        <linear>0.04</linear>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

    <!-- 地面 -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry>
          <material>
            <ambient>0.28 0.29 0.31 1</ambient>
            <diffuse>0.33 0.34 0.36 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry>
        </collision>
      </link>
    </model>

"""


def slot_centers(layout):
    """放置货架格心：col 1..4 = 类型列，row/level 1..3 = 层（沿 +Z，阶梯 +Y）。

    M3 里货架是放置目标（每格一块）。id 仍按 row-major 1..N 编号，仅供
    生成器命名与核对；控制器按「类型列 + 最低空层」选格。
    """
    rack = layout["rack"]
    cols, rows = int(rack["cols"]), int(rack["rows"])
    px = float(rack["pitch_x"])
    top_z = float(rack["top_z"])
    lpz = float(rack["level_pitch_z"])
    y_front = float(rack.get("y_front", rack.get("y", 0.30)))
    step_y = float(rack.get("level_step_y", 0.0))
    out = []
    for level in range(1, rows + 1):
        for col in range(1, cols + 1):
            out.append({
                "id": (level - 1) * cols + col,
                "level": level,
                "col": col,
                "x": (col - (cols + 1) / 2.0) * px,
                "y": y_front - (level - 1) * step_y,
                "z": top_z + (level - 1) * lpz,
            })
    return out


def infeed_slot(layout, idx):
    """来料架第 idx 块（1 起）的格位 (x, y, 托盘面 z)。

    row-major：第 1 行 4 块（1..4），第 2 行（5..8），第 3 行（9..12）。
    物料编号与格位一一对应，types[idx-1] 就是这一格的缺陷类型。
    """
    inf = layout["infeed"]
    cols = int(inf["cols"])
    row, col = divmod(int(idx) - 1, cols)
    px, py = float(inf["pitch_x"]), float(inf["pitch_y"])
    x = (col + 1 - (cols + 1) / 2.0) * px
    y = float(inf["y0"]) + row * py
    return x, y, float(inf["z"])


def material_types(layout, count):
    types = layout.get("materials", {}).get("types")
    if not types:
        types = [(i % 4) + 1 for i in range(count)]
    types = [int(t) for t in types]
    if len(types) != count:
        raise SystemExit(
            "[gen_world] materials.types 长度 {} != 库位数 {}".format(len(types), count))
    if any(t not in TYPE_NAMES for t in types):
        raise SystemExit("[gen_world] materials.types 只能取 1..4")
    return types


def base_plate_model(layout):
    """整机底板（官方效果图里的青色 PHOENIX CONTACT 台面的中性版）。

    立柱式龙门、分拣货架、置物台、来料架全部「站」在这块板上。
    """
    out = [
        '    <!-- 整机底板（1.40 x 1.10 m，上表面 z=0.02；-Y 侧加深 0.20 m 用来放 3x4 来料架） -->\n',
        '    <model name="base_plate">\n',
        "      <static>true</static>\n",
        '      <link name="plate">\n',
        '        <visual name="plate">\n',
        "          <pose>0 0 0.01 0 0 0</pose>\n",
        "          <geometry><box><size>1.40 1.10 0.02</size></box></geometry>\n",
        "          <material>\n"
        "            <ambient>0.10 0.42 0.46 1</ambient>\n"
        "            <diffuse>0.13 0.52 0.58 1</diffuse>\n"
        "          </material>\n",
        "        </visual>\n",
        '        <collision name="plate_col">\n',
        "          <pose>0 0 0.01 0 0 0</pose>\n",
        "          <geometry><box><size>1.40 1.10 0.02</size></box></geometry>\n",
        "        </collision>\n",
        "      </link>\n",
        "    </model>\n\n",
    ]
    return "".join(out)


def rack_shelf_model(layout):
    """放置货架：4 类型列 x 3 层，每格一块物料（官方「智能感知货架」）。

    层位沿 +Y 阶梯错开（第 1 层最靠前，step = level_step_y）：
    从上方垂放只穿过本层正上方，不会撞到上层托盘 —— 同 (x,y) 垂直堆叠时
    实测盘撞上层托盘、放置卡死 8 s（M3 首轮联调）。
    每层 4 个独立格口（托盘带围边、有 collision），列 x 对应类型 1..4。
    """
    rack = layout["rack"]
    cols, rows = int(rack["cols"]), int(rack["rows"])
    px = float(rack["pitch_x"])
    lpz = float(rack["level_pitch_z"])
    top_z = float(rack["top_z"])
    y_front = float(rack.get("y_front", rack.get("y", 0.30)))
    step_y = float(rack.get("level_step_y", 0.0))
    tile = float(rack["tile_size"])
    rim_h = float(rack["rim_height"])

    width = cols * px + 0.10
    depth = tile + 0.05 + (rows - 1) * step_y
    y_back = y_front + tile / 2.0 + 0.02
    panel_t = 0.015

    out = [
        '    <!-- 放置货架：{} 列 x {} 层，阶梯深度（第 1 层 y={:.2f}，步距 {:.2f}） -->\n'.format(
            cols, rows, y_front, step_y),
        '    <model name="rack">\n',
        "      <static>true</static>\n",
        '      <link name="frame">\n',
    ]
    # 左右侧板 + 背板（铝型材白）
    for sx, nm in ((-1, "side_l"), (1, "side_r")):
        out.append(
            '        <visual name="{n}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {d} {h}</size></box></geometry>\n"
            "          <material><ambient>0.72 0.74 0.76 1</ambient>"
            "<diffuse>0.82 0.84 0.86 1</diffuse></material>\n"
            "        </visual>\n".format(
                n=nm, x=sx * (width / 2.0 - panel_t / 2.0),
                y=(y_front + y_back) / 2.0,
                z=(top_z + (rows - 1) * lpz + 0.10) / 2.0 + 0.01,
                t=panel_t, d=depth,
                h=top_z + (rows - 1) * lpz + 0.12))
    out.append(
        '        <visual name="back">\n'
        "          <pose>0 {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>{w} {t} {h}</size></box></geometry>\n"
        "          <material><ambient>0.62 0.64 0.67 1</ambient>"
        "<diffuse>0.72 0.74 0.77 1</diffuse></material>\n"
        "        </visual>\n".format(
            y=y_back + panel_t / 2.0,
            z=(top_z + (rows - 1) * lpz + 0.10) / 2.0 + 0.01,
            w=width, t=panel_t, h=top_z + (rows - 1) * lpz + 0.12))
    # 每层：前沿横梁 + 4 个格口（托盘 collision + 四面围边 visual）
    for s in slot_centers(layout):
        lz = s["z"]
        ly = s["y"]
        if s["col"] == 1:
            out.append(
                '        <!-- 第 {lv} 层（z={z:.2f}, y={y:.2f}） -->\n'.format(
                    lv=s["level"], z=lz, y=ly))
            out.append(
                '        <visual name="beam_{lv}">\n'
                "          <pose>0 {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>{w} 0.03 0.02</size></box></geometry>\n"
                "          <material><ambient>0.55 0.68 0.35 1</ambient>"
                "<diffuse>0.62 0.76 0.42 1</diffuse></material>\n"
                "        </visual>\n".format(lv=s["level"], y=ly - tile / 2.0 - 0.015,
                                             z=lz - 0.012, w=width))
        out.append(
            '        <collision name="tray_{id}_col">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} 0.004</size></box></geometry>\n"
            "        </collision>\n".format(
                id=s["id"], x=s["x"], y=ly, z=lz - 0.002, t=tile))
        out.append(
            '        <visual name="tray_{id}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} 0.004</size></box></geometry>\n"
            "          <material><ambient>0.24 0.26 0.29 1</ambient>"
            "<diffuse>0.30 0.32 0.35 1</diffuse></material>\n"
            "        </visual>\n".format(id=s["id"], x=s["x"], y=ly, z=lz - 0.002, t=tile))
        for name, dx, dy, sx_, sy_ in (
                ("rim_xa", 0.0, -(tile / 2.0 + 0.003), tile + 0.012, 0.006),
                ("rim_xb", 0.0, +(tile / 2.0 + 0.003), tile + 0.012, 0.006),
                ("rim_ya", -(tile / 2.0 + 0.003), 0.0, 0.006, tile + 0.012),
                ("rim_yb", +(tile / 2.0 + 0.003), 0.0, 0.006, tile + 0.012)):
            out.append(
                '        <visual name="c{id}_{n}">\n'
                "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n"
                "          <material><ambient>0.22 0.24 0.27 1</ambient>"
                "<diffuse>0.28 0.30 0.33 1</diffuse></material>\n"
                "        </visual>\n".format(
                    id=s["id"], n=name, x=s["x"] + dx, y=ly + dy, z=lz + rim_h / 2.0,
                    sx=sx_, sy=sy_, h=rim_h))
    # 四根货架立腿（站到底板上表面 0.02）
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(
                '        <visual name="leg_{sx}_{sy}">\n'
                "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>0.03 0.03 {h}</size></box></geometry>\n"
                "          <material><ambient>0.30 0.31 0.33 1</ambient>"
                "<diffuse>0.36 0.37 0.39 1</diffuse></material>\n"
                "        </visual>\n".format(
                    sx=sx, sy=sy,
                    x=sx * (width / 2.0 - 0.03), y=y_front + sy * (depth / 2.0 - 0.03),
                    z=0.02 + (top_z + (rows - 1) * lpz + 0.06) / 2.0,
                    h=top_z + (rows - 1) * lpz + 0.06))
    out.append("      </link>\n    </model>\n\n")
    return "".join(out)


def staging_table_model(layout):
    """置物台（取料源）：每轮生成一块物料于此，相机俯拍识别后吸盘取走。"""
    st = layout["staging"]
    x, y, z = float(st["x"]), float(st["y"]), float(st["z"])
    out = [
        '    <!-- 置物台（取料源，台面 z={:.2f}） -->\n'.format(z),
        '    <model name="staging_table">\n',
        "      <static>true</static>\n",
        '      <link name="table">\n',
        '        <visual name="top">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>0.16 0.16 0.01</size></box></geometry>\n"
        "          <material><ambient>0.10 0.11 0.13 1</ambient>"
        "<diffuse>0.14 0.15 0.17 1</diffuse></material>\n"
        "        </visual>\n".format(x=x, y=y, z=z - 0.005),
        '        <collision name="top_col">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>0.16 0.16 0.01</size></box></geometry>\n"
        "        </collision>\n".format(x=x, y=y, z=z - 0.005),
        '        <visual name="pedestal">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>0.10 0.10 0.08</size></box></geometry>\n"
        "          <material><ambient>0.30 0.31 0.33 1</ambient>"
        "<diffuse>0.36 0.37 0.39 1</diffuse></material>\n"
        # 坑：立柱顶面原来与台面底面共面（z 0.09 对 0.09），渲染 z-fighting，
        # 在俯视相机里表现为台面中央一块「亮方块」——把盘的高对比背景打掉一半，
        # 真视觉的环对比判据直接失效。立柱下移 2 mm 藏到台面下方。
        "        </visual>\n".format(x=x, y=y, z=0.048),
        "      </link>\n    </model>\n\n",
    ]
    return "".join(out)


def infeed_shelf_model(layout):
    """来料架：3 行 x 4 列平面料位，12 块物料开局铺在这里（类型乱序）。

    每格是一个实心托盘（有 collision，物料落在它上面）+ 一圈深色围边，
    俯视时能清楚数出 12 个格口。位置在 -Y 侧、龙门行程之外（龙门用 set_pose
    取料，不需要走过去），所以不会与运动轨迹干涉。
    """
    inf = layout["infeed"]
    cols, rows = int(inf["cols"]), int(inf["rows"])
    z = float(inf["z"])
    tile = float(inf["tile"])
    rim_h = float(inf["rim_height"])
    pedestal_h = z - 0.02          # 从底板上表面（z=0.02）长到托盘面

    out = [
        '    <!-- 来料架：{} 行 x {} 列，12 块物料开局铺在这里 -->\n'.format(rows, cols),
        '    <model name="infeed_rack">\n',
        "      <static>true</static>\n",
        '      <link name="frame">\n',
    ]
    for idx in range(1, cols * rows + 1):
        x, y, _z = infeed_slot(layout, idx)
        out.append(
            '        <visual name="tile_{i:02d}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} {h}</size></box></geometry>\n"
            "          <material><ambient>0.46 0.48 0.50 1</ambient>"
            "<diffuse>0.56 0.58 0.60 1</diffuse></material>\n"
            "        </visual>\n".format(
                i=idx, x=x, y=y, z=0.02 + pedestal_h / 2.0, t=tile, h=pedestal_h))
        out.append(
            '        <collision name="tile_col_{i:02d}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} {h}</size></box></geometry>\n"
            "        </collision>\n".format(
                i=idx, x=x, y=y, z=0.02 + pedestal_h / 2.0, t=tile, h=pedestal_h))
        # 四面围边（深色，俯视时勾出格口）
        half = tile / 2.0 - 0.006
        for nm, dx, dy, sx, sy in (
                ("n", 0.0, +half, tile, 0.012),
                ("s", 0.0, -half, tile, 0.012),
                ("e", +half, 0.0, 0.012, tile),
                ("w", -half, 0.0, 0.012, tile)):
            out.append(
                '        <visual name="rim_{i:02d}_{n}">\n'
                "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n"
                "          <material><ambient>0.16 0.17 0.19 1</ambient>"
                "<diffuse>0.20 0.21 0.23 1</diffuse></material>\n"
                "        </visual>\n".format(
                    i=idx, n=nm, x=x + dx, y=y + dy, z=z + rim_h / 2.0,
                    sx=sx, sy=sy, h=rim_h))
    out.append("      </link>\n    </model>\n\n")
    return "".join(out)


def recycle_marker(layout):
    """回收位标记（仅 visual 圆环，物料识别失败未取走时被传送到这里）。"""
    rc = layout["recycle"]
    x, y, z = float(rc["x"]), float(rc["y"]), float(rc["z"])
    out = [
        '    <!-- 回收位标记（识别失败物料的去处） -->\n',
        '    <model name="recycle_marker">\n',
        "      <static>true</static>\n",
        '      <link name="mark">\n',
        '        <visual name="ring">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><cylinder><radius>0.055</radius>"
        "<length>0.002</length></cylinder></geometry>\n"
        "          <material><ambient>0.55 0.35 0.10 1</ambient>"
        "<diffuse>0.65 0.42 0.12 1</diffuse></material>\n"
        "        </visual>\n".format(x=x, y=y, z=z + 0.001),
        "      </link>\n    </model>\n\n",
    ]
    return "".join(out)


def ee_cam_model(layout):
    """末端相机（M3 固定式）：装在置物台正上方朝下，拍照在前、下探在后。

    为什么从 z_carriage 挪到固定支架（对齐官方平台 img-006 的取景）：
    吸盘下探取料时正好悬在盘上方 —— 相机若跟在吸盘后面，盘中心会被
    吸盘遮住（盘内缺陷判据全瞎）。固定式 + GOTO 停靠偏移位（staging.
    goto_offset_y）让吸盘让出光路，识别的是完整盘面。
    高度 = 置物台盘顶 + 0.26 m（盘半径 ~83 px，与 M2 阈值标定一致）。
    """
    cam = layout.get("camera") or {}
    st = layout["staging"]
    m = cam.get("mount")
    if m and len(m) == 6:
        x, y, z, rr, p, yaw = [float(v) for v in m]
    else:
        x, y = float(st["x"]), float(st["y"])
        z = float(st["z"]) + 0.006 + 0.26
        rr, p, yaw = 0.0, 1.5708, -1.5708
    fov = float(cam.get("fov_h", 1.047))
    w = int(cam.get("width", 640))
    h = int(cam.get("height", 480))
    rate = float(cam.get("update_rate", 20.0))
    out = [
        '    <!-- 末端相机（固定式，置物台正上方朝下；话题 /camera/image 不变） -->\n',
        '    <model name="ee_cam">\n',
        "      <static>true</static>\n",
        "      <pose>{} {} {} {} {} {}</pose>\n".format(
            *["{:.4f}".format(v) for v in (x, y, z, rr, p, yaw)]),
        '      <link name="body">\n',
        # 机身 visual 必须让出传感器光路：gz 会渲染本模型的 visual（M2 立柱
        # 挪位的同款坑），机身压在 sensor 原点上的话相机拍到的全是自己的壳。
        '        <visual name="body">\n'
        "          <pose>0 0 0.035 0 0 0</pose>\n"
        "          <geometry><box><size>0.06 0.06 0.04</size></box></geometry>\n"
        "          <material><ambient>0.15 0.15 0.17 1</ambient>"
        "<diffuse>0.20 0.20 0.22 1</diffuse></material>\n"
        "        </visual>\n",
        '        <sensor name="camera" type="camera">\n',
        "          <pose>0 0 0 0 0 0</pose>\n",
        "          <always_on>1</always_on>\n",
        "          <visualize>false</visualize>\n",
        "          <update_rate>{}</update_rate>\n".format(rate),
        '          <topic>/camera/image</topic>\n',
        "          <camera>\n",
        "            <horizontal_fov>{}</horizontal_fov>\n".format(fov),
        "            <image><width>{}</width><height>{}</height>"
        "<format>R8G8B8</format></image>\n".format(w, h),
        "            <clip><near>0.005</near><far>4.0</far></clip>\n",
        "          </camera>\n",
        "        </sensor>\n",
        "      </link>\n    </model>\n\n",
    ]
    return "".join(out)


def scene_cam(layout):
    """自由摄像机（scene_cam）：看板页面的「全景/自由视角」数据源。

    默认机位 = 货架正前上方的全景俯视，整个抓取搬运过程一目了然；
    运行时可通过 /world/<w>/set_pose 服务移动（aic_dashboard 看板的
    相机控制按钮走这条路），姿态随预设机位一并设置。模型是 static 的，
    set_pose 平移/转动都合法（相当于挪机位，不参与物理）。
    """
    cam = layout.get("scene_cam") or {}
    p = [float(v) for v in cam.get(
        "pose", [0.0, -0.95, 1.15, 0.0, 0.88, 1.5708])]
    fov = float(cam.get("fov_h", 1.30))
    w = int(cam.get("width", 960))
    h = int(cam.get("height", 540))
    rate = float(cam.get("update_rate", 15.0))
    out = [
        '    <!-- 自由摄像机：全景默认机位，看板可经 set_pose 服务自由移动 -->\n',
        '    <model name="scene_cam">\n',
        "      <static>true</static>\n",
        "      <pose>{} {} {} {} {} {}</pose>\n".format(*["{:.4f}".format(v) for v in p]),
        '      <link name="body">\n',
        '        <visual name="body">\n',
        "          <geometry><box><size>0.09 0.09 0.09</size></box></geometry>\n",
        "          <material><ambient>0.9 0.3 0.3 1</ambient>"
        "<diffuse>0.9 0.3 0.3 1</diffuse></material>\n",
        "        </visual>\n",
        '        <sensor name="scene_camera" type="camera">\n',
        "          <pose>0 0 0 0 0 0</pose>\n",
        "          <topic>/scene/image</topic>\n",
        "          <update_rate>{}</update_rate>\n".format(rate),
        "          <always_on>1</always_on>\n",
        "          <visualize>false</visualize>\n",
        "          <camera>\n",
        "            <horizontal_fov>{}</horizontal_fov>\n".format(fov),
        "            <image><width>{}</width><height>{}</height>"
        "<format>R8G8B8</format></image>\n".format(w, h),
        "            <clip><near>0.05</near><far>20</far></clip>\n",
        "          </camera>\n",
        "        </sensor>\n",
        "      </link>\n",
        "    </model>\n",
    ]
    return "".join(out)


def defect_rng(layout, slot_id, seed=None):
    """每块铁片一个独立随机流：同一 (seed, slot_id) 必然抽出同一组几何。

    用 seed*1000+slot_id 而不是一路顺次抽，是为了改某一块的形状时不影响
    其它库位的随机序列（做对照实验时能只换一个变量）。
    seed=None 时读 sim_layout.yaml 的 materials.jitter_seed；0 表示关闭。
    """
    if seed is None:
        seed = int((layout.get("materials") or {}).get("jitter_seed", 0) or 0)
    if not seed:
        return None
    return random.Random(int(seed) * 1000 + int(slot_id))


def defect_visual(mtype, r, d, rng):
    """返回缺陷可视的 (name, pose 串, geometry 片段, material 片段)。

    rng=None 时走 M2 的固定几何。格式精度按原值对齐（0.004 / 0.436 / 0.0025 …），
    因此关掉随机化后生成物与 M2 逐字节一致 —— 这条本身就是回归检查。
    """
    if mtype == 1:          # 划痕：细长薄条，绕 Z 任意朝向
        if rng is None:
            cx, cy, yaw, ln, wd = 0.004, -0.006, math.radians(25.0), 0.052, 0.0025
        else:
            ln = 0.052 * rng.uniform(*JITTER[1]["len"])
            wd = 0.0025 * rng.uniform(*JITTER[1]["wid"])
            lim = max(0.0, 0.93 * r - ln / 2.0)      # 整条划痕留在判据域内
            ang = rng.uniform(0.0, 2.0 * math.pi)
            rad = rng.uniform(0.0, lim)
            cx, cy, yaw = rad * math.cos(ang), rad * math.sin(ang), rng.uniform(-math.pi, math.pi)
        return ("defect_scratch",
                "{:.3f} {:.3f} {:.5f} 0 0 {:.3f}".format(cx, cy, d + 0.0004, yaw),
                "<box><size>{:.3f} {:.4f} 0.0008</size></box>".format(ln, wd),
                "<ambient>0.20 0.20 0.21 1</ambient><diffuse>0.24 0.24 0.25 1</diffuse>")
    if mtype == 2:          # 污渍：扁圆深色斑
        if rng is None:
            cx, cy, rad2 = 0.008, 0.004, 0.012
        else:
            rad2 = 0.012 * rng.uniform(*JITTER[2]["radius"])
            lim = max(0.0, 0.93 * r - rad2)
            ang = rng.uniform(0.0, 2.0 * math.pi)
            rad = rng.uniform(0.0, lim)
            cx, cy = rad * math.cos(ang), rad * math.sin(ang)
        return ("defect_stain",
                "{:.3f} {:.3f} {:.5f} 0 0 0".format(cx, cy, d + 0.0002),
                "<cylinder><radius>{:.3f}</radius><length>0.0006</length></cylinder>".format(rad2),
                "<ambient>0.16 0.13 0.10 1</ambient><diffuse>0.20 0.16 0.12 1</diffuse>")
    if mtype == 3:          # 缺角：压在盘缘的深色方块（视觉上像被咬掉）
        if rng is None:
            cx, cy, sa, sb = 0.032, -0.020, 0.024, 0.022
        else:
            u = rng.uniform(*JITTER[3]["size"])
            sa, sb = 0.024 * u, 0.022 * u
            # 径向必须落在 0.90~0.96r：往里挪就不是缺角，往外挪整块掉出盘外
            rad = rng.uniform(0.90, 0.96) * r
            ang = rng.uniform(0.0, 2.0 * math.pi)
            cx, cy = rad * math.cos(ang), rad * math.sin(ang)
        return ("defect_chip",
                "{:.3f} {:.3f} {:.5f} 0 0 0".format(cx, cy, d + 0.0006),
                "<box><size>{:.3f} {:.3f} 0.0012</size></box>".format(sa, sb),
                "<ambient>0.10 0.11 0.13 1</ambient><diffuse>0.14 0.15 0.17 1</diffuse>")
    # 孔洞：小深色圆
    if rng is None:
        cx, cy, rad4 = -0.004, 0.012, 0.0055
    else:
        rad4 = 0.0055 * rng.uniform(*JITTER[4]["radius"])
        lim = max(0.0, 0.93 * r - rad4)
        ang = rng.uniform(0.0, 2.0 * math.pi)
        rad = rng.uniform(0.0, lim)
        cx, cy = rad * math.cos(ang), rad * math.sin(ang)
    return ("defect_hole",
            "{:.3f} {:.3f} {:.5f} 0 0 0".format(cx, cy, d + 0.0002),
            "<cylinder><radius>{:.4f}</radius><length>0.0006</length></cylinder>".format(rad4),
            "<ambient>0.03 0.03 0.03 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse>")


def plate_model(layout, slot, mtype, jitter_seed=None):
    """一块圆形物料 + 该类缺陷的几何表达（M4：开局铺在来料架的格位上）。

    动态模型（有质量/碰撞/摩擦）：运行时被 set_pose 传送到置物台，
    HANDLE 时由 DetachableJoint 吸附到 z_carriage（吸盘），放进货架格后松开。
    物料编号 = 来料架格位号（row-major），types[idx-1] 即该格类型。
    """
    plate = layout["plate"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    idx = int(slot["id"])
    cx, cy, deck_z = infeed_slot(layout, idx)
    cz = deck_z + th / 2.0 + 0.001   # 托盘面上留 1 mm，重力自然落稳
    name = "material_{:02d}".format(idx)
    d = th / 2.0  # 盘顶面在 link 内的 z
    mass = float(plate.get("mass", 0.05))
    ixx = mass * (3.0 * r * r + th * th) / 12.0
    izz = mass * r * r / 2.0

    out = [
        '    <model name="{}">\n'.format(name),
        "      <static>false</static>\n",
        "      <pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose>\n".format(cx, cy, cz),
        '      <link name="disc">\n',
        '        <inertial>\n',
        "          <mass>{}</mass>\n".format(mass),
        "          <inertia>\n",
        "            <ixx>{:.8g}</ixx><ixy>0</ixy><ixz>0</ixz>\n".format(ixx),
        "            <iyy>{:.8g}</iyy><iyz>0</iyz><izz>{:.8g}</izz>\n".format(ixx, izz),
        "          </inertia>\n",
        '        </inertial>\n',
        '        <collision name="disc_col">\n',
        "          <geometry><cylinder><radius>{}</radius><length>{}</length></cylinder></geometry>\n".format(r, th),
        "          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>\n",
        '        </collision>\n',
        '        <visual name="disc">\n',
        "          <geometry><cylinder><radius>{}</radius><length>{}</length></cylinder></geometry>\n".format(r, th),
        "          <material>\n",
        "            <ambient>0.42 0.44 0.46 1</ambient>\n",
        "            <diffuse>0.50 0.52 0.54 1</diffuse>\n",
        "            <specular>0.0 0.0 0.0 1</specular>\n",
        "          </material>\n",
        "        </visual>\n",
    ]

    ind = "        "
    rng = defect_rng(layout, slot["id"], jitter_seed)
    vname, vpose, vgeo, vmat = defect_visual(mtype, r, d, rng)
    out.append(
        ind + '<visual name="{}">\n'.format(vname)
        + ind + "  <pose>{}</pose>\n".format(vpose)
        + ind + "  <geometry>{}</geometry>\n".format(vgeo)
        + ind + "  <material>{}</material>\n".format(vmat)
        + ind + "</visual>\n")

    out.append("      </link>\n")
    out.append("    </model>\n\n")
    return "".join(out)


def gantry_model(layout, types, slots):
    """把 models/aic_gantry/model.sdf 内联进世界，并给每块铁片挂一个 DetachableJoint。

    为什么不用 <include> + 在铁片模型里挂插件（踩过）：
      DetachableJoint 的 <child_model> 只在**自己的模型范围内**找（按 parent=本模型
      过滤实体），平级模型一律报 "Child Model aic_gantry could not be found"。
      所以插件必须挂在「持有方」——也就是龙门——child 指向铁片。
      而 <include> 不能追加插件，只能内联模型。
    为什么不直接给 model.sdf 写 12 份插件：库位号是数据、几何是代码。
      让生成器拼「几何 + N 个插件」，改库位数不用碰 model.sdf。
    """
    src = os.path.join(PKG, "models", "aic_gantry", "model.sdf")
    with open(src, "r", encoding="utf-8") as f:
        text = f.read()
    start = text.index("<model")
    end = text.rindex("</model>") + len("</model>")
    block = text[start:end]

    # M3：相机已移出龙门（固定式 ee_cam，见 ee_cam_model），mount_z 替换逻辑删除。
    # model.sdf 里不再有 <sensor name="camera">，若又加回去需要重新接上替换逻辑。

    plugs = []
    for s, t in zip(slots, types):
        name = "material_{:02d}".format(s["id"])
        plugs.append(
            '    <plugin filename="ignition-gazebo-detachable-joint-system"\n'
            '            name="ignition::gazebo::systems::DetachableJoint">\n'
            "      <parent_link>z_carriage</parent_link>\n"
            "      <child_model>{}</child_model>\n"
            "      <child_link>disc</child_link>\n"
            "      <attach_topic>/sim/attach_{:02d}</attach_topic>\n"
            "      <detach_topic>/sim/detach_{:02d}</detach_topic>\n"
            "    </plugin>\n".format(name, s["id"], s["id"]))

    cut = block.rindex("</model>")
    block = block[:cut] + "".join(plugs) + block[cut:]
    # 落到 <world> 下，整体缩进 2 格，与其它顶层模型对齐
    return "\n".join("  " + ln if ln.strip() else ln for ln in block.splitlines()) + "\n\n"


def build_sdf(layout, seed=None):
    slots = slot_centers(layout)
    types = material_types(layout, len(slots))

    out = [HEADER, base_plate_model(layout), rack_shelf_model(layout),
           staging_table_model(layout), infeed_shelf_model(layout),
           recycle_marker(layout), ee_cam_model(layout), scene_cam(layout)]
    out.append("    <!-- ===== 12 块圆形物料（开局铺在来料架 3x4 格位上，运行时逐块上料；"
               "类型 = materials.types[序号-1]）===== -->\n")
    for s, t in zip(slots, types):
        out.append(plate_model(layout, s, t, seed))

    out.append("  <!-- 三轴龙门（关节零位 = 吸盘最高位 0.239 / 相机 0.365） -->\n")
    out.append(gantry_model(layout, types, slots))
    out.append("  </world>\n</sdf>\n")
    return "".join(out), types


def main(argv=None):
    ap = argparse.ArgumentParser(description="生成 aic_warehouse.sdf")
    ap.add_argument("--layout", default=os.path.join(PKG, "config", "sim_layout.yaml"))
    ap.add_argument("--out", default=os.path.join(PKG, "worlds", "aic_warehouse.sdf"))
    ap.add_argument("--seed", type=int, default=None,
                    help="缺陷几何随机化种子；不给则读 materials.jitter_seed，0=关闭")
    args = ap.parse_args(argv)

    with open(args.layout, "r", encoding="utf-8") as f:
        layout = yaml.safe_load(f)

    sdf, types = build_sdf(layout, args.seed)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(sdf)
    seed = args.seed if args.seed is not None else int(
        (layout.get("materials") or {}).get("jitter_seed", 0) or 0)
    print("[gen_world] 已生成 {}（{} 字节，jitter_seed={}）".format(args.out, len(sdf), seed))
    print("[gen_world] 物料生成顺序类型（material 1..12）：{}".format(list(types)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
