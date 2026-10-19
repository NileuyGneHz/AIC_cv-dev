#!/usr/bin/python3
"""从 config/sim_layout.yaml 生成 worlds/aic_warehouse.sdf（货架 + 12 块物料 + 龙门）。

用法：
    /usr/bin/python3 scripts/gen_world.py                # 覆盖写 worlds/aic_warehouse.sdf

为什么用生成而不是手写：12 个库位地贴 / 12 块缺陷物料的坐标与类型都来自
sim_layout.yaml 的同一份数据，布局或类型分配一改，重跑本脚本即可，不必手改 XML。
生成的 SDF 一并提交；运行时不依赖本脚本。

物料缺陷用几何体表达（不贴纹理）：
    1 划痕 = 细长深色薄条     2 污渍 = 扁圆深色斑
    3 缺角 = 边缘一块「缺口」  4 孔洞 = 小深色圆孔
这样渲染稳定、俯视相机一眼可分，也便于后续传统视觉做阈值/模板算法。
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

TYPE_NAMES = {1: "scratch", 2: "stain", 3: "chip", 4: "hole"}

HEADER = """<?xml version="1.0" ?>
<!--
  自动生成，勿手改：由 scripts/gen_world.py 从 config/sim_layout.yaml 生成。
  AIC 视觉逻辑算法应用赛 · 12 库位货架 + 三轴龙门 仿真世界（Ignition Fortress）
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
    """与 aic_warehouse 同一口径：id=(row-1)*cols+col，row 1 在 -Y 侧（前排）。"""
    rack = layout["rack"]
    cols, rows = int(rack["cols"]), int(rack["rows"])
    px, py = float(rack["pitch_x"]), float(rack["pitch_y"])
    cx, cy = float(rack.get("center_x", 0.0)), float(rack.get("center_y", 0.0))
    out = []
    for row in range(1, rows + 1):
        for col in range(1, cols + 1):
            out.append({
                "id": (row - 1) * cols + col,
                "row": row,
                "col": col,
                "x": cx + (col - (cols + 1) / 2.0) * px,
                "y": cy + (row - (rows + 1) / 2.0) * py,
            })
    return out


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


def rack_model(layout):
    rack = layout["rack"]
    cols, rows = int(rack["cols"]), int(rack["rows"])
    px, py = float(rack["pitch_x"]), float(rack["pitch_y"])
    top_z = float(rack["top_z"])
    tile = float(rack["tile_size"])
    tile_t = float(rack["tile_thickness"])
    deck_x = cols * px + 0.02
    deck_y = rows * py + 0.02
    cx, cy = float(rack.get("center_x", 0.0)), float(rack.get("center_y", 0.0))

    out = [
        '    <!-- 货架台面（{} x {} mm，上表面 z={} m） -->\n'.format(
            int(deck_x * 1000), int(deck_y * 1000), top_z),
        '    <model name="rack">\n',
        "      <static>true</static>\n",
        "      <pose>{} {} 0 0 0 0</pose>\n".format(cx, cy),
        '      <link name="deck">\n',
        '        <visual name="deck">\n',
        "          <pose>0 0 {} 0 0 0</pose>\n".format(top_z - 0.01),
        "          <geometry><box><size>{} {} 0.02</size></box></geometry>\n".format(
            deck_x, deck_y),
        "          <material>\n",
        "            <ambient>0.55 0.56 0.58 1</ambient>\n",
        "            <diffuse>0.62 0.63 0.65 1</diffuse>\n",
        "            <specular>0.0 0.0 0.0 1</specular>\n",
        "          </material>\n",
        "        </visual>\n",
        '        <collision name="deck_col">\n',
        "          <geometry><box><size>{} {} 0.02</size></box></geometry>\n".format(
            deck_x, deck_y),
        "        </collision>\n",
    ]

    # 库位地贴（深色，便于俯视相机与人眼对位）
    for s in slot_centers(layout):
        out.append(
            '        <visual name="tile_{id}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} {tt}</size></box></geometry>\n"
            "          <material>\n"
            "            <ambient>0.10 0.11 0.13 1</ambient>\n"
            "            <diffuse>0.14 0.15 0.17 1</diffuse>\n"
            "          </material>\n"
            "        </visual>\n".format(
                id=s["id"], x=s["x"] - cx, y=s["y"] - cy,
                z=top_z + tile_t / 2.0, t=tile, tt=tile_t))

    out.append("      </link>\n")

    legs = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            legs.append(
                '        <visual name="leg_{sx}_{sy}">\n'
                "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>0.04 0.04 {h}</size></box></geometry>\n"
                "          <material><ambient>0.30 0.31 0.33 1</ambient>"
                "<diffuse>0.36 0.37 0.39 1</diffuse></material>\n"
                "        </visual>\n".format(
                    sx=sx, sy=sy,
                    x=sx * (deck_x / 2.0 - 0.05), y=sy * (deck_y / 2.0 - 0.05),
                    z=(top_z - 0.02) / 2.0, h=top_z - 0.02))
    out.append('      <link name="legs">\n' + "".join(legs) + "      </link>\n")
    out.append("    </model>\n\n")
    return "".join(out)


def plate_model(layout, slot, mtype):
    """一块圆形铁片 + 该类缺陷的几何表达。静态模型：不会被物理扰动。"""
    plate = layout["plate"]
    rack = layout["rack"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    base_z = float(rack["top_z"]) + float(rack["tile_thickness"])
    cz = base_z + th / 2.0
    name = "material_{:02d}_{}".format(slot["id"], TYPE_NAMES[mtype])
    d = th / 2.0  # 铁片顶面在 link 内的 z

    out = [
        '    <model name="{}">\n'.format(name),
        "      <static>true</static>\n",
        "      <pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose>\n".format(slot["x"], slot["y"], cz),
        '      <link name="disc">\n',
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
    if mtype == 1:          # 划痕：细长薄条，绕 Z 斜置
        out.append(
            ind + '<visual name="defect_scratch">\n'
            + ind + "  <pose>0.004 -0.006 {:.5f} 0 0 0.436</pose>\n".format(d + 0.0004)
            + ind + "  <geometry><box><size>0.052 0.0025 0.0008</size></box></geometry>\n"
            + ind + "  <material><ambient>0.20 0.20 0.21 1</ambient>"
                     "<diffuse>0.24 0.24 0.25 1</diffuse></material>\n"
            + ind + "</visual>\n")
    elif mtype == 2:        # 污渍：扁圆深色斑
        out.append(
            ind + '<visual name="defect_stain">\n'
            + ind + "  <pose>0.008 0.004 {:.5f} 0 0 0</pose>\n".format(d + 0.0002)
            + ind + "  <geometry><cylinder><radius>0.012</radius><length>0.0006</length></cylinder></geometry>\n"
            + ind + "  <material><ambient>0.16 0.13 0.10 1</ambient>"
                     "<diffuse>0.20 0.16 0.12 1</diffuse></material>\n"
            + ind + "</visual>\n")
    elif mtype == 3:        # 缺角：边缘覆盖一块与地贴同色的方块（视觉上像被咬掉）
        out.append(
            ind + '<visual name="defect_chip">\n'
            + ind + "  <pose>0.032 -0.020 {:.5f} 0 0 0</pose>\n".format(d + 0.0006)
            + ind + "  <geometry><box><size>0.024 0.022 0.0012</size></box></geometry>\n"
            + ind + "  <material><ambient>0.10 0.11 0.13 1</ambient>"
                     "<diffuse>0.14 0.15 0.17 1</diffuse></material>\n"
            + ind + "</visual>\n")
    else:                   # 孔洞：小深色圆
        out.append(
            ind + '<visual name="defect_hole">\n'
            + ind + "  <pose>-0.004 0.012 {:.5f} 0 0 0</pose>\n".format(d + 0.0002)
            + ind + "  <geometry><cylinder><radius>0.0055</radius><length>0.0006</length></cylinder></geometry>\n"
            + ind + "  <material><ambient>0.03 0.03 0.03 1</ambient>"
                     "<diffuse>0.05 0.05 0.05 1</diffuse></material>\n"
            + ind + "</visual>\n")

    out.append("      </link>\n")
    out.append("    </model>\n\n")
    return "".join(out)


def build_sdf(layout):
    slots = slot_centers(layout)
    types = material_types(layout, len(slots))

    out = [HEADER, rack_model(layout)]
    out.append("    <!-- ===== 12 块圆形铁片（静态；类型见 sim_layout.yaml materials.types）===== -->\n")
    for s, t in zip(slots, types):
        out.append(plate_model(layout, s, t))

    out.append(
        "    <!-- 三轴龙门（关节零位 = 相机在货架中心正上方、末端最高位） -->\n"
        "    <include>\n"
        "      <uri>model://aic_gantry</uri>\n"
        "      <name>aic_gantry</name>\n"
        "      <pose>0 0 0 0 0 0</pose>\n"
        "    </include>\n"
        "  </world>\n"
        "</sdf>\n")
    return "".join(out), types


def main(argv=None):
    ap = argparse.ArgumentParser(description="生成 aic_warehouse.sdf")
    ap.add_argument("--layout", default=os.path.join(PKG, "config", "sim_layout.yaml"))
    ap.add_argument("--out", default=os.path.join(PKG, "worlds", "aic_warehouse.sdf"))
    args = ap.parse_args(argv)

    with open(args.layout, "r", encoding="utf-8") as f:
        layout = yaml.safe_load(f)

    sdf, types = build_sdf(layout)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(sdf)
    print("[gen_world] 已生成 {}（{} 字节）".format(args.out, len(sdf)))
    print("[gen_world] 12 库位类型（slot 1..12）：{}".format(list(types)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
