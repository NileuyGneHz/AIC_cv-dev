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
        self.place = doc.get("place_zones", {})
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

    def slot_xy(self, slot_id):
        """slot_id 1..N（row-major）-> 世界坐标 (x, y)。row 1 在 -Y 侧。"""
        slot_id = int(slot_id)
        if not 1 <= slot_id <= self.cols * self.rows:
            raise ValueError("slot_id 越界：{}".format(slot_id))
        row = (slot_id - 1) // self.cols + 1
        col = (slot_id - 1) % self.cols + 1
        px = float(self.rack["pitch_x"])
        py = float(self.rack["pitch_y"])
        cx = float(self.rack.get("center_x", 0.0))
        cy = float(self.rack.get("center_y", 0.0))
        x = cx + (col - (self.cols + 1) / 2.0) * px
        y = cy + (row - (self.rows + 1) / 2.0) * py
        return x, y

    def slot_type(self, slot_id):
        """缺陷类型 1..4；未配置时返回 0。"""
        idx = int(slot_id) - 1
        if 0 <= idx < len(self.types):
            return self.types[idx]
        return 0

    def place_xy(self, mtype):
        """缺陷类型 1..4 -> 码放格的格心 (x, y)；未配置时返回 None。

        码放条在 +Y 侧，4 格沿 +X 排列，顺序与类型号一致（见 place_zones.xs）。
        """
        xs = self.place.get("xs") or []
        idx = int(mtype) - 1
        if not 0 <= idx < len(xs):
            return None
        return float(xs[idx]), float(self.place["y"])

    def summary(self):
        return "frame={} cols={} rows={} pitch=({}, {}) types={}".format(
            self.frame_id, self.cols, self.rows,
            self.rack["pitch_x"], self.rack["pitch_y"], self.types)


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
