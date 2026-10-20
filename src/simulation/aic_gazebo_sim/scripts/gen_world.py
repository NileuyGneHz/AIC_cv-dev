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
        # 坑：这里必须和上面的 visual 同 pose。漏写时 collision 落在 link 原点
        #     （z∈[-0.01,0.01]），与可视台面差 90 mm。铁片还是 static 时看不出来，
        #     一改成动态就整批掉到这块「看不见的碰撞板」上（实测停在 z=0.013）。
        "          <pose>0 0 {} 0 0 0</pose>\n".format(top_z - 0.01),
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


def place_table(layout):
    """分类码放条（+Y 侧）：一块台面 + 4 个格口（地贴 + 围边）。

    真机台面 1000x500 已被 12 库位占满，仿真为跑通「搬运到按类型的目标格」才加这条。
    坐标全部来自 sim_layout.yaml 的 place_zones 段。
    """
    pz = layout.get("place_zones")
    if not pz:
        return ""
    rack = layout["rack"]
    top_z = float(rack["top_z"])
    cz = float(pz["z"])
    py = float(pz["y"])
    tile = float(pz["tile_size"])
    rim_h = float(pz["rim_height"])
    xs = [float(v) for v in pz["xs"]]

    depth = tile + 0.02
    width = (xs[-1] - xs[0]) + tile + 0.06
    out = [
        '    <!-- 分类码放条（{} x {} mm，上表面 z={}） -->\n'.format(
            int(width * 1000), int(depth * 1000), cz),
        '    <model name="place_table">\n',
        "      <static>true</static>\n",
        '      <link name="deck">\n',
        '        <visual name="deck">\n',
        "          <pose>0 {:.4f} {:.4f} 0 0 0</pose>\n".format(py, cz - 0.01),
        "          <geometry><box><size>{} {} 0.02</size></box></geometry>\n".format(width, depth),
        "          <material>\n",
        "            <ambient>0.55 0.56 0.58 1</ambient>\n",
        "            <diffuse>0.62 0.63 0.65 1</diffuse>\n",
        "            <specular>0.0 0.0 0.0 1</specular>\n",
        "          </material>\n",
        "        </visual>\n",
        '        <collision name="deck_col">\n',
        # 同 rack：collision 必须跟 visual 同 pose。这里连 Y 也要给，
        # 否则碰撞板落在 y=0（货架下方），码放到 y=0.30 的铁片会直接穿到地面。
        "          <pose>0 {:.4f} {:.4f} 0 0 0</pose>\n".format(py, cz - 0.01),
        "          <geometry><box><size>{} {} 0.02</size></box></geometry>\n".format(width, depth),
        "        </collision>\n",
    ]
    # 4 个格口：浅色地贴 + 四边围边，方便人眼/回放区分类型
    for i, x in enumerate(xs):
        out.append(
            '        <visual name="zone_{i}">\n'
            "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
            "          <geometry><box><size>{t} {t} 0.002</size></box></geometry>\n"
            "          <material><ambient>0.28 0.30 0.33 1</ambient>"
            "<diffuse>0.34 0.36 0.39 1</diffuse></material>\n"
            "        </visual>\n".format(
                i=i + 1, x=x, y=py, z=cz + 0.001, t=tile))
        # 围边：两根沿 X、两根沿 Y，压在地贴四边
        for name, dx, dy, sx, sy in (
                ("rim_xa", 0.0, -(tile / 2.0), tile, 0.006),
                ("rim_xb", 0.0, +(tile / 2.0), tile, 0.006),
                ("rim_ya", -(tile / 2.0), 0.0, 0.006, tile),
                ("rim_yb", +(tile / 2.0), 0.0, 0.006, tile)):
            out.append(
                '        <visual name="z{i}_{n}">\n'
                "          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n"
                "          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n"
                "          <material><ambient>0.20 0.22 0.24 1</ambient>"
                "<diffuse>0.26 0.28 0.30 1</diffuse></material>\n"
                "        </visual>\n".format(
                    i=i + 1, n=name, x=x + dx, y=py + dy, z=cz + rim_h / 2.0,
                    sx=sx, sy=sy, h=rim_h))
    out.append("      </link>\n")
    out.append("    </model>\n\n")
    return "".join(out)


def plate_model(layout, slot, mtype):
    """一块圆形铁片 + 该类缺陷的几何表达。

    动态模型（有质量/碰撞/摩擦）：HANDLE 时由 DetachableJoint 吸附到 z_carriage，
    抬升后跟着走，放到目标格再松开。早前是 static，做不了抓取。
    """
    plate = layout["plate"]
    rack = layout["rack"]
    r = float(plate["diameter"]) / 2.0
    th = float(plate["thickness"])
    # 库位地贴只有 visual、没有 collision，铁片实际落在台面板上表面 top_z。
    base_z = float(rack["top_z"])
    # 抬高 0.2 mm：避免出生瞬间与地贴互相穿透，物理会把它弹一下
    cz = base_z + th / 2.0 + 0.0002
    name = "material_{:02d}_{}".format(slot["id"], TYPE_NAMES[mtype])
    d = th / 2.0  # 铁片顶面在 link 内的 z
    mass = float(plate.get("mass", 0.05))
    ixx = mass * (3.0 * r * r + th * th) / 12.0
    izz = mass * r * r / 2.0

    out = [
        '    <model name="{}">\n'.format(name),
        "      <static>false</static>\n",
        "      <pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose>\n".format(slot["x"], slot["y"], cz),
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

    plugs = []
    for s, t in zip(slots, types):
        name = "material_{:02d}_{}".format(s["id"], TYPE_NAMES[t])
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


def build_sdf(layout):
    slots = slot_centers(layout)
    types = material_types(layout, len(slots))

    out = [HEADER, rack_model(layout), place_table(layout)]
    out.append("    <!-- ===== 12 块圆形铁片（动态；类型见 sim_layout.yaml materials.types）===== -->\n")
    for s, t in zip(slots, types):
        out.append(plate_model(layout, s, t))

    out.append("  <!-- 三轴龙门（关节零位 = 相机在货架中心正上方、末端最高位） -->\n")
    out.append(gantry_model(layout, types, slots))
    out.append("  </world>\n</sdf>\n")
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
