"""仿真侧共用的布局读取（gantry_controller / sim_vision_node 都用它）。

单一口径：库位号 <-> 世界坐标 <-> 缺陷类型，全部来自
share/aic_gazebo_sim/config/sim_layout.yaml（源文件在包内 config/）。
"""
import os

import yaml


def default_layout_path():
    """优先用安装后的 share 路径，源码树里直接跑时回退到包内 config/。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory("aic_gazebo_sim"),
                            "config", "sim_layout.yaml")
        if os.path.isfile(cand):
            return cand
    except Exception:  # noqa: BLE001 - 未 source 环境时必须能回退
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), "config", "sim_layout.yaml")


class SimLayout:
    def __init__(self, doc):
        self.doc = doc
        self.frame_id = doc.get("frame_id", "warehouse")
        self.rack = doc["rack"]
        self.plate = doc["plate"]
        self.gantry = doc.get("gantry", {})
        self.staging = doc.get("staging", {})
        self.infeed = doc.get("infeed", {})
        self.recycle = doc.get("recycle", {})
        self.types = [int(t) for t in doc.get("materials", {}).get("types", [])]

    @classmethod
    def load(cls, path=""):
        with open(path or default_layout_path(), "r", encoding="utf-8") as f:
            return cls(yaml.safe_load(f))

    @property
    def cols(self):
        return int(self.rack["cols"])

    @property
    def rows(self):
        return int(self.rack["rows"])

    def staging_xy(self):
        """置物台（取料源）格心 (x, y) 与台面高 z。"""
        return (float(self.staging["x"]), float(self.staging["y"]),
                float(self.staging["z"]))

    def infeed_slot(self, idx):
        """来料架第 idx 块（1 起）的格位 (x, y, 托盘面 z)；row-major。

        与 gen_world.infeed_slot 同一公式（世界生成与运行时取料必须一致）。
        """
        cols = int(self.infeed.get("cols", 4))
        row, col = divmod(int(idx) - 1, cols)
        px = float(self.infeed.get("pitch_x", 0.25))
        py = float(self.infeed.get("pitch_y", 0.09))
        x = (col + 1 - (cols + 1) / 2.0) * px
        y = float(self.infeed.get("y0", -0.44)) + row * py
        return x, y, float(self.infeed.get("z", 0.10))

    def recycle_xyz(self):
        """回收位 (x, y, z)。"""
        return (float(self.recycle["x"]), float(self.recycle["y"]),
                float(self.recycle["z"]))

    def rack_cell(self, mtype, level):
        """类型列 mtype 1..4 的第 level 层（1 起）格心 (x, y) 与托盘面高 z。

        层位沿 +Y 阶梯错开：y = y_front - (level-1)*level_step_y，
        保证从上方垂放时不会撞到上层的托盘。
        """
        col = int(mtype)
        if not 1 <= col <= self.cols:
            return None
        if not 1 <= int(level) <= self.rows:
            return None
        x = (col - (self.cols + 1) / 2.0) * float(self.rack["pitch_x"])
        z = float(self.rack["top_z"]) + (int(level) - 1) * float(
            self.rack["level_pitch_z"])
        y = (float(self.rack.get("y_front", self.rack.get("y", 0.30)))
             - (int(level) - 1) * float(self.rack.get("level_step_y", 0.0)))
        return x, y, z

    def type_of_material(self, idx):
        """第 idx 块物料（1 起，生成顺序）的类型真值。"""
        i = int(idx) - 1
        if 0 <= i < len(self.types):
            return self.types[i]
        return 0

    def slot_type(self, slot_id):
        """slot_id（=物料编号，1 起）的缺陷类型真值 1..4；越界返回 0。"""
        return self.type_of_material(slot_id)

    def pad_z0(self):
        """吸盘工具基准：z_joint=0 时吸口下沿的世界高度。"""
        return float(self.gantry.get("pad_z0", 0.239))

    def summary(self):
        return ("frame={} 货架={}列x{}层 types={} 置物台=({},{})").format(
            self.frame_id, self.cols, self.rows, self.types,
            self.staging.get("x"), self.staging.get("y"))


def declare_number(node, name, default, cast=float):
    """声明数字参数，允许命令行给 int 或 float。

    坑：declare_parameter(name, 0.5) 会把类型钉成 double；命令行
    `-p interval_s:=5` 传进来是 INTEGER，rclpy 直接抛
    InvalidParameterTypeException 把节点打挂（表现为「节点启动即退出」）。
    用 dynamic_typing 声明后自己 cast 一次即可。
    """
    from rcl_interfaces.msg import ParameterDescriptor
    try:
        return cast(node.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value)
    except Exception:  # noqa: BLE001 - 拿不到就用默认值，不要因为参数类型挂掉仿真
        return cast(default)
