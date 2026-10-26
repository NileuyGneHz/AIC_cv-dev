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
import io
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
                "y": y_front - (rows - level) * step_y,
                "z": top_z + (level - 1) * lpz,
            })
    return out


def park_slot(layout, idx):
    """停料区第 idx 块（1 起）的位置 (x, y, 托板面 z)。

    12 块沿 +X 排成一行，藏在**底板下方**（俯视与相机都看不见）。
    等价于官方平台的「生成物料」：运行时由 gantry_controller 用 set_pose
    把第 k 块送到置物台，对观察者来说就是"凭空出现一块料"。
    """
    p = layout["park"]
    x = float(p.get("x0", -0.495)) + (int(idx) - 1) * float(p.get("pitch_x", 0.09))
    return x, float(p.get("y", 0.0)), float(p.get("z", -0.10))


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
            # 前沿绿色装饰横杠已移除（用户 2026-10-23 要求）：纯 visual、
            # 无 collision，删掉不影响放置判定。
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


def material_park_model(layout):
    """停料托板：藏在底板下方的托板，12 块物料开局停在这里（俯视不可见）。

    官方平台是「点『生成物料』在置物台生成一块」。我们没有运行时 create
    （DetachableJoint 在世界加载时就解析 child_model，动态实体绑不上），
    所以 12 块物料仍在 SDF 里，只是停在底板下面，运行时用 set_pose 逐块送到
    置物台 —— 对观察者等价于"凭空生成"。
    """
    p = layout["park"]
    z = float(p.get("z", -0.10))
    n = len(layout["materials"]["types"])
    x0 = float(p.get("x0", -0.495))
    pitch = float(p.get("pitch_x", 0.09))
    width = (n - 1) * pitch + 0.16
    cx = x0 + (n - 1) * pitch / 2.0
    cy = float(p.get("y", 0.0))
    out = [
        '    <!-- 停料托板（底板下方，俯视与相机都看不见） -->\n',
        '    <model name="material_park">\n',
        "      <static>true</static>\n",
        '      <link name="board">\n',
        '        <visual name="board_v">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>{w:.4f} 0.16 0.02</size></box></geometry>\n"
        "          <material><ambient>0.20 0.22 0.25 1</ambient>"
        "<diffuse>0.24 0.26 0.29 1</diffuse></material>\n"
        "        </visual>\n"
        '        <collision name="board_col">\n'
        "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
        "          <geometry><box><size>{w:.4f} 0.16 0.02</size></box></geometry>\n"
        "        </collision>\n"
        "      </link>\n    </model>\n\n".format(
            x=cx, y=cy, z=z - 0.01, w=width),
    ]
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

    为什么从末端连杆挪到固定支架（对齐官方平台 img-006 的取景）：
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


# ---------------------------------------------------------------------------
# 四类缺陷图案（2026-10-25 对齐官方图 8：赛题规则 §2「任务流程和物料」）
#
#   官方样本一行一类，各 4 块，形状固定、只有位置/朝向/尺寸随机：
#     缺角 = 盘缘被咬掉一个楔形缺口（盘体缺口，不是画上去的线）
#     污渍 = 一个实心圆斑
#     孔洞 = 空心三角轮廓（细线围一圈，中间还是亮盘面）
#     划痕 = 一条细弧钩（光滑开口曲线，没有折角）
# ---------------------------------------------------------------------------
CHIP_ANGLE_DEG = (30.0, 40.0)     # 缺口张角（= 缺口口部的弧长，不随深度变）
# 缺口深度（× 盘半径）：越大两腰越长。上限 0.55 -> 顶点仍在 0.45r 之外，
# 缺口不会顶到盘心（2026-10-26 用户口径：加长两腰、弧长不变、不过圆心）。
CHIP_DEPTH = (0.42, 0.52)
SCRATCH_R = (0.0115, 0.0150)      # 划痕圆弧半径
SCRATCH_SWEEP = (2.4, 3.5)        # 划痕弧张角（rad）
STAIN_R = (0.0050, 0.0070)        # 污渍斑半径
HOLE_SIDE = (0.0130, 0.0175)      # 孔洞三角边长
DEFECT_LINE_W = 0.0016            # 细线宽（盘径 80 mm 下约 1/50 盘径）

# 网格（visual-only）与 SDF 放在一起：worlds/meshes/。
# 为什么不用 model://：本包 install 的是**拷贝**（不是符号链接），新建的 model
# 目录不会自动出现在 install/share/.../models 下，实测报
#   Unable to find file with URI [model://aic_plate/meshes/...]
# 所以走 file:// + 生成时算出的绝对路径 —— 世界是生成产物，换机器重跑
# scripts/gen_world.py 即可刷新路径。缺角盘 = 被挖掉楔口的圆盘网格，
# 碰撞体仍是整圆柱：物理不关心缺口，视觉上已经是真缺口。
MESH_DIR = os.path.join(PKG, "worlds", "meshes")
PLATE_MESH_URI = "file://" + os.path.join(MESH_DIR, "plate_{:02d}.obj")
TUBE_MESH_URI = "file://" + os.path.join(MESH_DIR, "marker_tube.obj")


def _obj_text(verts, faces, note=""):
    """写 OBJ。每个三角面单独给一条法线（平面着色）。

    坑：只写 v/f 不写 vn，Ogre 会拿默认法线去算光照，盘面直接过曝成纯白
    （实测 2026-10-26：缺角盘在白底上完全看不出缺口）。法线按面的绕向算，
    所以下面的建模里**绕向必须是外法线方向**。
    """
    out = ["# {}\n".format(note)]
    out += ["v {:.6f} {:.6f} {:.6f}\n".format(x, y, z) for x, y, z in verts]
    for i, (a, b, c) in enumerate(faces):
        p0, p1, p2 = verts[a - 1], verts[b - 1], verts[c - 1]
        ux, uy, uz = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
        vx, vy, vz = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
        nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
        ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        out.append("vn {:.6f} {:.6f} {:.6f}\n".format(nx / ln, ny / ln, nz / ln))
    for i, (a, b, c) in enumerate(faces):
        out.append("f {}//{n} {}//{n} {}//{n}\n".format(a, b, c, n=i + 1))
    return "".join(out)


def notch_disc_mesh(r, th, a0, da, depth_frac, n=160):
    """圆盘挖掉一个楔形缺口后的三角网格（顶面 + 底面 + 侧壁，米制）。

    a0 = 缺口起始角，da = 缺口张角，depth_frac = 缺口深度 / 盘半径。
    缺口形状与官方样本一致：两条直边从盘缘收进到内侧一个尖点。
    """
    half = 0.5 * th
    r_apex = r * (1.0 - depth_frac)
    pts = []
    span = 2.0 * math.pi - da
    for k in range(n):
        ang = a0 + da + span * k / float(n)
        pts.append((r * math.cos(ang), r * math.sin(ang)))
    apex_ang = a0 + 0.5 * da
    pts.append((r_apex * math.cos(apex_ang), r_apex * math.sin(apex_ang)))
    m = len(pts)
    verts = [(0.0, 0.0, half), (0.0, 0.0, -half)]
    for (x, y) in pts:
        verts.append((x, y, half))
        verts.append((x, y, -half))
    faces = []
    # 顶点表：1 = 顶心，2 = 底心，从 3 起每个边界点占两个（顶、底）：
    #   p_i 顶 = 3 + 2i，p_i 底 = 4 + 2i（都是 OBJ 的 1-based 序号）
    for i in range(m):
        j = (i + 1) % m
        ti, tj = 3 + 2 * i, 3 + 2 * j
        bi, bj = ti + 1, tj + 1
        faces.append((1, ti, tj))          # 顶面（法线 +z）
        faces.append((2, bj, bi))          # 底面（法线 -z）
        faces.append((ti, bi, bj))         # 侧壁（外法线）
        faces.append((ti, bj, tj))
    return _obj_text(verts, faces, "notched plate r={:.4f} notch={:.1f}deg".format(
        r, math.degrees(da)))


def tube_mesh(r_out, r_in, th, a0=0.0, da=0.0, n=192):
    """圆环管（放料标记）：外壁 + 内壁 + 上下环面。中间是空的，不挡盘面图案。

    da > 0 时在 [a0, a0+da] 这一段**开口**（给缺角盘用）：环也在缺口处断开，
    否则管壁会横跨缺口、把咬口「补」上一道 0.5 mm 的绿边。
    开口两端各补一块径向端面，免得从缺口看进去能看穿管壁。
    """
    half = 0.5 * th
    closed = da <= 0.0
    if closed:
        angs = [2.0 * math.pi * k / n for k in range(n)]
    else:
        span = 2.0 * math.pi - da
        angs = [a0 + da + span * k / n for k in range(n + 1)]
    verts = []
    for a in angs:
        c, s = math.cos(a), math.sin(a)
        verts.append((r_out * c, r_out * s, half))    # ot
        verts.append((r_out * c, r_out * s, -half))   # ob
        verts.append((r_in * c, r_in * s, half))      # it
        verts.append((r_in * c, r_in * s, -half))     # ib
    faces = []
    m = len(angs)
    for k in range(m if closed else m - 1):
        j = (k + 1) % m
        ot_i, ob_i, it_i, ib_i = 4 * k, 4 * k + 1, 4 * k + 2, 4 * k + 3
        ot_j, ob_j, it_j, ib_j = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        faces.append((ot_i + 1, ob_i + 1, ob_j + 1))   # 外壁
        faces.append((ot_i + 1, ob_j + 1, ot_j + 1))
        faces.append((it_i + 1, it_j + 1, ib_j + 1))   # 内壁（朝轴心）
        faces.append((it_i + 1, ib_j + 1, ib_i + 1))
        faces.append((ot_i + 1, ot_j + 1, it_j + 1))   # 上环面（+z）
        faces.append((ot_i + 1, it_j + 1, it_i + 1))
        faces.append((ob_i + 1, ib_i + 1, ib_j + 1))   # 下环面（-z）
        faces.append((ob_i + 1, ib_j + 1, ob_j + 1))
    if not closed:
        faces.append((1, 3, 4))                        # 起始端面（法线 -切向）
        faces.append((1, 4, 2))
        b = 4 * (m - 1)
        faces.append((b + 1, b + 2, b + 4))            # 末端面（法线 +切向）
        faces.append((b + 1, b + 4, b + 3))
    return _obj_text(verts, faces,
                     "marker tube" if closed else "marker tube (notched)")


def write_plate_meshes(layout, seed=None):
    """写出缺角盘 + 对应开口的标记环网格。

    返回 {物料号: 开口环文件名}：只有缺角盘有开口环，其余物料用整圆环。
    """
    os.makedirs(MESH_DIR, exist_ok=True)
    plate = layout["plate"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    wall = float((layout.get("marker") or {}).get("wall_m", 0.0005))
    types = material_types(layout, len(slot_centers(layout)))
    out = {}
    for i, t in enumerate(types, start=1):
        if int(t) != 3:
            continue
        rng = defect_rng(layout, i, seed) or random.Random(3 * 1000 + i)
        a0 = rng.uniform(0.0, 2.0 * math.pi)
        da = math.radians(rng.uniform(*CHIP_ANGLE_DEG))
        depth = rng.uniform(*CHIP_DEPTH)
        name = "plate_{:02d}.obj".format(i)
        with io.open(os.path.join(MESH_DIR, name), "w", encoding="utf-8") as f:
            f.write(notch_disc_mesh(r, th, a0, da, depth))
        # 同一角度开一口的环：缺角盘套环后，环也在缺口处断开
        ring = "marker_tube_{:02d}.obj".format(i)
        with io.open(os.path.join(MESH_DIR, ring), "w", encoding="utf-8") as f:
            f.write(tube_mesh(r + wall, r, th, a0=a0, da=da))
        out[i] = ring
    with io.open(os.path.join(MESH_DIR, "marker_tube.obj"), "w", encoding="utf-8") as f:
        f.write(tube_mesh(r + wall, r, th))
    return out


def _poly_segments(pts, width):
    """把折线轮廓 pts 拆成 [(cx, cy, theta, length), ...] 的细长段（未定位、未旋转）。"""
    out = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        ln = math.hypot(dx, dy)
        if ln < 1e-6:
            continue
        # 两端各延长半个线宽：不然折角处会留缝，连通域会被切断
        out.append((0.5 * (x0 + x1), 0.5 * (y0 + y1), math.atan2(dy, dx),
                    ln + (width if width > 0 else 0.0)))
    return out


def defect_visual(mtype, r, d, rng):
    """返回该类缺陷的视觉列表 [(name, pose, geometry, material), ...]。

    对齐官方图 8（赛题规则 §2「任务流程和物料」）的四类样本，一类一种形状，
    只有位置 / 朝向 / 尺度随机（规则原文：划痕长短、污渍形状、缺角大小、孔洞大小随机）：
      1 划痕 = 一条细弧钩（光滑开口曲线，无折角）
      2 污渍 = 一个实心圆斑
      3 缺角 = 盘缘楔形缺口 —— **盘体本身的缺口**，见 notch_disc_mesh()，
               这里不返回任何 visual（缺口由网格挖出来）
      4 孔洞 = 空心正三角轮廓（1.6 mm 细线，中间仍是亮盘面）

    实现约束：Fortress 的 visual 没有折线/多边形，曲线 = 一串首尾重叠的细长 box
    （640x480 俯视下与一条连续线无差别）；三角轮廓 = 3 段首尾相接的细长 box。
    全部 visual-only、无 collision。
    """
    z = d + 0.0004
    line_mat = ("<ambient>0.18 0.18 0.19 1</ambient>"
                "<diffuse>0.22 0.22 0.23 1</diffuse>")
    solid_mat = ("<ambient>0.16 0.13 0.10 1</ambient>"
                 "<diffuse>0.20 0.16 0.12 1</diffuse>")
    wd = DEFECT_LINE_W

    def place(segs, mat, solid=False):
        """把盘内坐标的线段列表移到盘面上：抽位置 + 朝向，返回 visual 列表。"""
        hw = max(abs(sx) + 0.5 * ln for sx, _sy, _th, ln in segs) + wd
        hh = max(abs(sy) + 0.5 * ln for _sx, sy, _th, ln in segs) + wd
        if rng is None:
            cx, cy, yaw = 0.002, -0.004, math.radians(20.0)
        else:
            lim_x = max(0.0, 0.93 * r - hw)
            lim_y = max(0.0, 0.93 * r - hh)
            cx = rng.uniform(-lim_x, lim_x) if lim_x > 1e-4 else 0.0
            cy = rng.uniform(-lim_y, lim_y) if lim_y > 1e-4 else 0.0
            yaw = rng.uniform(-math.pi, math.pi)
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for i, (sx, sy, th, ln) in enumerate(segs):
            out.append((
                "defect_{}_{:02d}".format(mtype, i),
                "{:.4f} {:.4f} {:.5f} 0 0 {:.4f}".format(
                    cx + sx * c - sy * s, cy + sx * s + sy * c, z, th + yaw),
                "<box><size>{:.4f} {:.4f} 0.0008</size></box>".format(ln, wd),
                mat))
        return out

    u = 1.0 if rng is None else rng.uniform(0.90, 1.15)     # 整体尺度微变

    if mtype == 1:                                   # 划痕：细弧钩
        rad = rng.uniform(*SCRATCH_R) * u if rng else 0.0115
        sweep = rng.uniform(*SCRATCH_SWEEP) if rng else 2.9
        base = rng.uniform(0.0, 2.0 * math.pi) if rng else 0.0
        seg = 12
        pts = [(rad * math.cos(base + sweep * i / seg),
                rad * math.sin(base + sweep * i / seg)) for i in range(seg + 1)]
        pts = [(x - pts[seg // 2][0], y - pts[seg // 2][1]) for x, y in pts]
        return place(_poly_segments(pts, wd), line_mat)

    if mtype == 2:                                   # 污渍：实心圆斑
        rad = rng.uniform(*STAIN_R) * u if rng else 0.0060
        # 实心圆 = 一个扁圆柱（最简单的「实心」；不需要用细线一层层铺）
        hw = rad
        if rng is None:
            cx, cy = 0.002, -0.004
        else:
            lim = max(0.0, 0.93 * r - hw)
            ang = rng.uniform(0.0, 2.0 * math.pi)
            dist = rng.uniform(0.0, lim) if lim > 1e-4 else 0.0
            cx, cy = dist * math.cos(ang), dist * math.sin(ang)
        return [("defect_{}_00".format(mtype),
                 "{:.4f} {:.4f} {:.5f} 0 0 0".format(cx, cy, z),
                 "<cylinder><radius>{:.4f}</radius><length>0.0006</length></cylinder>".format(rad),
                 solid_mat)]

    if mtype == 3:                                   # 缺角：盘体缺口（网格）
        return []

    # 孔洞：空心正三角轮廓（闭合细线）
    side = rng.uniform(*HOLE_SIDE) * u if rng else 0.0175
    h = side * math.sqrt(3.0) / 2.0
    rot = rng.uniform(0.0, 2.0 * math.pi) if rng else 0.0
    tri = [(-side / 2, -h / 3), (side / 2, -h / 3), (0.0, 2.0 * h / 3)]
    c, s = math.cos(rot), math.sin(rot)
    tri = [(x * c - y * s, x * s + y * c) for x, y in tri]
    return place(_poly_segments(tri + [tri[0]], wd), line_mat)


def plate_model(layout, slot, mtype, jitter_seed=None):
    """一块圆形物料 + 该类缺陷的几何表达（M4：开局铺在来料架的格位上）。

    动态模型（有质量/碰撞/摩擦）：运行时被 set_pose 传送到置物台，
    HANDLE 时由 DetachableJoint 吸附到 tool_head（吸盘），放进货架格后松开。
    物料编号 = 来料架格位号（row-major），types[idx-1] 即该格类型。
    """
    plate = layout["plate"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    idx = int(slot["id"])
    cx, cy, deck_z = park_slot(layout, idx)
    cz = deck_z + th / 2.0 + 0.001   # 托盘面上留 1 mm，重力自然落稳
    name = "material_{:02d}".format(idx)
    d = th / 2.0  # 盘顶面在 link 内的 z
    mass = float(plate.get("mass", 0.05))
    ixx = mass * (3.0 * r * r + th * th) / 12.0
    izz = mass * r * r / 2.0

    # 初始朝向跟随 jitter 流（停料区在底板下看不见；真正每轮的随机朝向
    # 由控制器上料时给，见 gantry_controller._set_pose）。注意：yaw 先抽、
    # 缺陷几何后抽 —— 同一 seed 下缺陷几何与旧版本会不同，阈值有余量不受影响。
    rng0 = defect_rng(layout, slot["id"], jitter_seed)
    yaw0 = rng0.uniform(0.0, 2.0 * math.pi) if rng0 else 0.0

    out = [
        '    <model name="{}">\n'.format(name),
        "      <static>false</static>\n",
        "      <pose>{:.4f} {:.4f} {:.4f} 0 0 {:.4f}</pose>\n".format(cx, cy, cz, yaw0),
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
        # 缺角（type=3）：盘体用「挖掉楔口」的网格（碰撞仍是整圆柱，见
        # notch_disc_mesh 的说明）；其余三类是完整圆柱 + 盘面上的图案。
        ("          <geometry><mesh><uri>{}</uri></mesh></geometry>\n".format(
            PLATE_MESH_URI.format(idx)) if int(mtype) == 3 else
         "          <geometry><cylinder><radius>{}</radius><length>{}</length></cylinder></geometry>\n".format(r, th)),
        "          <material>\n",
        "            <ambient>0.42 0.44 0.46 1</ambient>\n",
        "            <diffuse>0.50 0.52 0.54 1</diffuse>\n",
        "            <specular>0.0 0.0 0.0 1</specular>\n",
        "          </material>\n",
        "        </visual>\n",
    ]

    ind = "        "
    rng = rng0          # 同一块铁片共用一条随机流（yaw 已在上面抽掉一次）
    for vname, vpose, vgeo, vmat in defect_visual(mtype, r, d, rng):
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
            "      <parent_link>tool_head</parent_link>\n"
            "      <child_model>{}</child_model>\n"
            "      <child_link>disc</child_link>\n"
            "      <attach_topic>/sim/attach_{:02d}</attach_topic>\n"
            "      <detach_topic>/sim/detach_{:02d}</detach_topic>\n"
            "    </plugin>\n".format(name, s["id"], s["id"]))

    cut = block.rindex("</model>")
    block = block[:cut] + "".join(plugs) + block[cut:]
    # 落到 <world> 下，整体缩进 2 格，与其它顶层模型对齐
    return "\n".join("  " + ln if ln.strip() else ln for ln in block.splitlines()) + "\n\n"


def camera_post_model(layout):
    """货架右前侧的**装饰性**相机立柱（对齐官方平台图，不参与成像）。

    真正的成像相机是 ee_cam（固定在置物台正上方，俯视标定用它）——
    这根立柱只补齐官方平台的外观元素，避免按图对照时觉得"少了一个相机"。
    位置在货架右前角外侧（x=0.64），横梁行程只到 x≈0.48，不干涉。
    """
    x, y = 0.64, 0.20
    out = [
        '    <!-- 装饰性相机立柱（货架右前侧；不参与成像） -->\n',
        '    <model name="camera_post">\n',
        "      <static>true</static>\n",
        '      <link name="post">\n',
        '        <visual name="foot">\n'
        "          <pose>{x:.4f} {y:.4f} 0.035 0 0 0</pose>\n"
        "          <geometry><box><size>0.10 0.10 0.03</size></box></geometry>\n"
        "          <material><ambient>0.20 0.21 0.23 1</ambient>"
        "<diffuse>0.26 0.27 0.30 1</diffuse></material>\n"
        "        </visual>\n"
        '        <visual name="pole">\n'
        "          <pose>{x:.4f} {y:.4f} 0.29 0 0 0</pose>\n"
        "          <geometry><box><size>0.035 0.035 0.54</size></box></geometry>\n"
        "          <material><ambient>0.28 0.30 0.33 1</ambient>"
        "<diffuse>0.34 0.36 0.39 1</diffuse></material>\n"
        "        </visual>\n"
        '        <visual name="head">\n'
        "          <pose>{x:.4f} {y:.4f} 0.575 0 0.7854 -2.25</pose>\n"
        "          <geometry><box><size>0.09 0.06 0.06</size></box></geometry>\n"
        "          <material><ambient>0.12 0.12 0.14 1</ambient>"
        "<diffuse>0.16 0.16 0.18 1</diffuse></material>\n"
        "        </visual>\n"
        "      </link>\n    </model>\n\n".format(x=x, y=y),
    ]
    return "".join(out)


def rack_cam_model(layout):
    """每个货架格口一只**隐藏**监视相机（只有 sensor，没有 visual，不参与物理）。

    为什么不用「正上方垂直下视」：货架是垂直立面，层间距只有 0.06 m，
    正上方会被上一层托盘整块挡住。改成「格口前沿外 + 略高」斜视格心：
    视线擦过围边上沿（余量 ~14 mm），能看到整个盘面。
    分辨率 160x120 / 5 Hz：只做占位监视（有没有料、放没放正），
    真正的识别仍走置物台上方的 ee_cam，两套互不影响。
    话题：/rack/cam_<id>/image，id 与 slot_centers 的格口号一致（层优先）。
    """
    cam = layout.get("rack_cams") or {}
    fov = float(cam.get("fov_h", 1.25))
    w = int(cam.get("width", 160))
    h = int(cam.get("height", 120))
    rate = float(cam.get("update_rate", 5.0))
    z_lift = float(cam.get("z_lift", 0.040))     # 相对本层托盘面的高度
    y_off = float(cam.get("y_offset", 0.115))    # 在格口前沿外多少
    pitch = math.atan2(z_lift - 0.006, y_off)    # 看向盘顶（盘厚 6 mm）

    out = ['    <!-- 库位监视相机墙：每格一只隐藏相机（无 visual，俯视格心） -->\n',
           '    <model name="rack_cams">\n',
           "      <static>true</static>\n"]
    for s in slot_centers(layout):
        x = s["x"]
        y = s["y"] - y_off
        z = s["z"] + z_lift
        out.append(
            '      <link name="cam_{id:02d}">\n'
            "        <pose>{x:.4f} {y:.4f} {z:.4f} 0 {p:.4f} 1.5708</pose>\n"
            '        <sensor name="rack_cam_{id:02d}" type="camera">\n'
            "          <pose>0 0 0 0 0 0</pose>\n"
            "          <always_on>1</always_on>\n"
            "          <visualize>false</visualize>\n"
            "          <update_rate>{rate}</update_rate>\n"
            "          <topic>/rack/cam_{id:02d}/image</topic>\n"
            "          <camera>\n"
            "            <horizontal_fov>{fov}</horizontal_fov>\n"
            "            <image><width>{w}</width><height>{h}</height>"
            "<format>R8G8B8</format></image>\n"
            "            <clip><near>0.01</near><far>2.0</far></clip>\n"
            "          </camera>\n"
            "        </sensor>\n"
            "      </link>\n".format(
                id=s["id"], x=x, y=y, z=z, p=pitch, rate=rate, fov=fov, w=w, h=h))
    out.append("    </model>\n\n")
    return "".join(out)


MARKER_GREEN = ((0.04, 0.42, 0.12), (0.06, 0.70, 0.20), (0.02, 0.16, 0.05))
MARKER_RED = ((0.42, 0.04, 0.04), (0.78, 0.08, 0.08), (0.20, 0.02, 0.02))
# 环外径比盘半径大多少（= 俯视露出的环宽）的缺省值；sim_layout.yaml 的
# marker.wall_m 优先（2026-10-25 用户口径：7 mm → 1.5 mm → 0.5 mm）。
MARKER_WALL_M = 0.0005


def marker_model(layout, tubes=None):
    """放料判定指示环：**无碰撞**的绿/红薄套，放料后 set_pose 到该盘位置。

    Fortress 没有「运行时改材质」的服务，所以不能直接把盘侧边的材质刷成颜色。
    这里用一个**比盘只大 0.5 mm**（`sim_layout.yaml` 的 `marker.wall_m`）、
    与盘等高的薄套贴在盘周侧：侧视整圈变色，俯视只露出一圈 0.5 mm 细边。

    每种颜色各一套、编号与物料一一对应，运行时只点亮其中一只（另一只留在底板下）：
      * `marker_NN`       绿环 —— 视觉判型 == 该轮真值（放对）
      * `marker_wrong_NN` 红环 —— 视觉判型 != 该轮真值（放错）
    两只**几何完全一致**，只有材质不同，所以「绿边/红边」宽度、高度、位置一致。

    三条硬约束：
    1. **没有 collision**：不参与物理，不会顶住盘或货架，也不影响落位判定；
    2. **gravity=false**：停在底板下方（z=-0.14）不会掉；
    3. 顶面比盘顶低 0.5 mm：不盖住盘面，两个视觉相机看到的缺陷特征不变。

    tubes：{物料号: 开口环网格文件名}，缺角盘用**同角度开口**的环（见 write_plate_meshes），
    否则整圆环会在缺口上横跨一道 0.5 mm 的色边。
    """
    tubes = tubes or {}
    plate = layout["plate"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    R = r + float((layout.get("marker") or {}).get("wall_m", MARKER_WALL_M))
    p = layout["park"]
    n = len(layout["materials"]["types"])
    x0 = float(p.get("x0", -0.495))
    pitch = float(p.get("pitch_x", 0.09))
    out = ['    <!-- 放料判定指示环（无碰撞；放料后 set_pose 到盘位，包住圆柱侧边；'
           '绿=放对 / 红=放错） -->\n']
    for name, y_park, col in (("marker", 0.16, MARKER_GREEN),
                              ("marker_wrong", 0.20, MARKER_RED)):
        amb, dif, emi = col
        for i in range(1, n + 1):
            out.append(
                '    <model name="{p}_{i:02d}">\n'
                "      <static>false</static>\n"
                "      <pose>{x:.4f} {y:.3f} -0.14 0 0 0</pose>\n"
                '      <link name="ring">\n'
                "        <gravity>false</gravity>\n"
                "        <inertial><mass>0.005</mass><inertia>"
                "<ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>\n"
                '        <visual name="ring">\n'
                # 圆环**管**（不是实心圆柱）：缺角盘被套环后缺口仍然透出来。
                # 管壁 = marker.wall_m（0.5 mm），外径 = 盘半径 + 壁厚。
                "          <geometry><mesh><uri>{}</uri></mesh></geometry>\n"
                "          <material>\n"
                "            <ambient>{a:.2f} {b:.2f} {c:.2f} 1</ambient>\n"
                "            <diffuse>{d:.2f} {e:.2f} {f:.2f} 1</diffuse>\n"
                "            <emissive>{g:.2f} {h2:.2f} {k:.2f} 1</emissive>\n"
                "          </material>\n"
                "        </visual>\n"
                "      </link>\n    </model>\n\n".format(
                    ("file://" + os.path.join(MESH_DIR, tubes[i])
                     if i in tubes else TUBE_MESH_URI),
                    p=name, i=i, x=x0 + (i - 1) * pitch, y=y_park, r=R, h=th,
                    a=amb[0], b=amb[1], c=amb[2],
                    d=dif[0], e=dif[1], f=dif[2],
                    g=emi[0], h2=emi[1], k=emi[2]))
    return "".join(out)


def build_sdf(layout, seed=None):
    slots = slot_centers(layout)
    types = material_types(layout, len(slots))
    # 先把缺角盘 / 标记环的网格写出来（file:// 引用，必须早于世界加载）
    tubes = write_plate_meshes(layout, seed)

    out = [HEADER, base_plate_model(layout), rack_shelf_model(layout),
           staging_table_model(layout), material_park_model(layout),
           recycle_marker(layout), ee_cam_model(layout),
           camera_post_model(layout), rack_cam_model(layout), marker_model(layout, tubes),
           scene_cam(layout)]
    out.append("    <!-- ===== 12 块圆形物料（开局停在底板下方的停料托板上，运行时逐块 set_pose 上料；"
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
