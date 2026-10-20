"""启动 Gazebo 仿真（Ignition Fortress）：世界 + 龙门 + ROS 桥 + 控制/视觉。

用法：
    ros2 launch aic_gazebo_sim sim.launch.py                 # 无头（默认，适合 SSH / 自验）
    ros2 launch aic_gazebo_sim sim.launch.py gui:=true       # 带 GUI（本机看画面）
    ros2 launch aic_gazebo_sim sim.launch.py bridge_camera:=false   # 不看图、省带宽
    ros2 launch aic_gazebo_sim sim.launch.py vision_backend:=trad   # 换真视觉（反查相机图）

只起仿真和控制/视觉，不起决策；端到端请用 sim_decision.launch.py。

视觉后端（vision_backend）：
    gt   —— aic_gazebo_sim/sim_vision_node.py：读 sim_layout.yaml 直接发真值（默认，链路自验用）
    trad —— aic_vision_trad/vision_node.py：订阅 /camera/image，真做盘面定位 + 缺陷分类

话题（全部经 ros_gz_bridge 转成 ROS 2）：
    /aic/joint/{x,y,z}/cmd_pos   std_msgs/Float64   (ROS -> GZ，位置指令 m)
    /world/aic_warehouse/model/aic_gantry/joint_state  sensor_msgs/JointState (GZ -> ROS，真实关节反馈)
    /camera/image                sensor_msgs/Image  (GZ -> ROS)
    /camera/camera_info          sensor_msgs/CameraInfo
    /clock                       rosgraph_msgs/Clock
    /vision/trigger              std_msgs/Empty     (gantry_controller -> vision)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRIDGE_TOPICS = [
    # ROS 2 <- Gazebo
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    # 关节反馈：JointStatePublisher 发的是 ignition.msgs.Model，桥成 sensor_msgs/JointState。
    # prismatic 关节的位移/速度就装在 axis1.position / axis1.velocity 里，实测可用
    # （早前把它当成「不回传」是错的，那是 pose/info 的行为）。控制侧靠这条判「真到位」。
    "/world/aic_warehouse/model/aic_gantry/joint_state"
    "@sensor_msgs/msg/JointState[ignition.msgs.Model",
    # ROS 2 -> Gazebo（关节位置指令）
    "/aic/joint/x/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/aic/joint/y/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/aic/joint/z/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
]

CAMERA_TOPICS = [
    "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    # 自由摄像机（scene_cam，看板「全景/自由视角」用；模型在 gen_world.py 里生成）
    "/scene/image@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/scene/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
]


def grip_topics(n_slots):
    """抓取/释放信号桥：每块铁片一对，话题名与 gen_world.py 生成的 DetachableJoint 一致。

    坑：DetachableJoint 是「一个插件绑一个 child」，不能所有铁片共用一条话题
    （那样一按就 12 块全吸附）。所以这里是 2*N 条，按 slot 号编号。
    """
    out = []
    for i in range(1, int(n_slots) + 1):
        # 编号直接接词尾：ROS 话题名不允许「以数字开头的 token」（/sim/attach/01 非法）
        out.append("/sim/attach_{:02d}@std_msgs/msg/Empty]ignition.msgs.Empty".format(i))
        out.append("/sim/detach_{:02d}@std_msgs/msg/Empty]ignition.msgs.Empty".format(i))
    return out


def generate_launch_description():
    share = get_package_share_directory("aic_gazebo_sim")
    world_path = os.path.join(share, "worlds", "aic_warehouse.sdf")
    layout_path = os.path.join(share, "config", "sim_layout.yaml")
    models_path = os.path.join(share, "models")

    # 抓取信号桥的条数由布局决定（cols * rows），不写死 12
    import yaml
    with open(layout_path, "r", encoding="utf-8") as f:
        _layout = yaml.safe_load(f)
    n_slots = int(_layout["rack"]["cols"]) * int(_layout["rack"]["rows"])

    gui = LaunchConfiguration("gui")
    bridge_camera = LaunchConfiguration("bridge_camera")
    verbose = LaunchConfiguration("verbose")

    # 坑：ign gazebo 在 HOME 缺失时会把日志/缓存目录退化成 /.ignition 并反复报警，
    #     极端情况下直接无法写日志。容器/CI 里 HOME 常常是空的，这里兜一个。
    home = os.environ.get("HOME") or "/tmp"
    resource_path = models_path
    if os.environ.get("IGN_GAZEBO_RESOURCE_PATH"):
        resource_path += os.pathsep + os.environ["IGN_GAZEBO_RESOURCE_PATH"]

    # 坑：本机 ign-physics-5 的引擎库放在独立目录，Fortress 不会自动加进搜索路径，
    #     结果是 physics system 静默不加载（时钟在走、物体不动）。显式补上。
    physics_engine_path = os.environ.get("IGN_PHYSICS_ENGINE_PATH", "")
    if not physics_engine_path:
        import glob
        cands = glob.glob("/usr/lib/x86_64-linux-gnu/ign-physics-*/engine-plugins")
        physics_engine_path = cands[0] if cands else ""

    gz_headless_cmd = ["ign", "gazebo", "-s", "-r", "--headless-rendering",
                       "-v", verbose, world_path]
    gz_gui_cmd = ["ign", "gazebo", "-r", "-v", verbose, world_path]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="true = 起 GUI（本机看画面）；false = 只起服务端"),
        DeclareLaunchArgument("bridge_camera", default_value="true",
                              description="是否把 /camera/image 桥到 ROS 2"),
        DeclareLaunchArgument("verbose", default_value="2",
                              description="ign gazebo 日志级别 1..4"),
        DeclareLaunchArgument("drop_rate", default_value="0.0",
                              description="视觉桩模拟识别失败的概率（不发结果）"),
        DeclareLaunchArgument("invalid_rate", default_value="0.0",
                              description="视觉桩返回非法值（\"9\"）的概率"),
        DeclareLaunchArgument("vision_delay_s", default_value="0.0",
                              description="视觉桩在到位后额外延时再回复"),
        DeclareLaunchArgument("vision_backend", default_value="gt",
                              description="gt = ground-truth 桩；trad = 传统视觉（看相机图）"),
        DeclareLaunchArgument("vision_debug_dir", default_value="",
                              description="trad 后端保存调试图的目录（空 = 不存）"),

        SetEnvironmentVariable("HOME", home),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_PHYSICS_ENGINE_PATH", physics_engine_path),
        # 本机自验只在回环上通信，避免多网卡/多机环境下的发现失败干扰结论
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", os.environ.get("ROS_LOCALHOST_ONLY", "0")),

        LogInfo(msg=["[sim] world = ", world_path]),
        LogInfo(msg=["[sim] models = ", models_path]),

        # 仿真服务端：无 GUI 时用 EGL 无头渲染（相机仍然出图）
        ExecuteProcess(cmd=gz_headless_cmd, output="screen",
                       condition=UnlessCondition(gui), name="gzserver"),
        ExecuteProcess(cmd=gz_gui_cmd, output="screen",
                       condition=IfCondition(gui), name="gz_gui"),

        # gz <-> ROS 2 桥
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="gz_bridge", output="screen",
             arguments=BRIDGE_TOPICS + grip_topics(n_slots)),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="gz_camera_bridge", output="screen",
             condition=IfCondition(bridge_camera),
             arguments=CAMERA_TOPICS),

        # 控制侧（仿真实现）：/control/material_task -> 关节指令
        Node(package="aic_gazebo_sim", executable="gantry_controller.py",
             name="gantry_controller", output="screen",
             parameters=[{"layout_yaml": layout_path}]),

        # 视觉侧 A（ground truth 桩）：到位 -> /vision/material_type
        Node(package="aic_gazebo_sim", executable="sim_vision_node.py",
             name="sim_vision_node", output="screen",
             condition=LaunchConfigurationEquals("vision_backend", "gt"),
             parameters=[{
                 "layout_yaml": layout_path,
                 "drop_rate": LaunchConfiguration("drop_rate"),
                 "invalid_rate": LaunchConfiguration("invalid_rate"),
                 "delay_s": LaunchConfiguration("vision_delay_s"),
             }]),

        # 视觉侧 B（传统视觉）：/camera/image + /vision/trigger -> /vision/material_type
        # 两者只能起一个，否则同一个位点会收到两条结果（决策按 seq 只认最新，但日志会乱）。
        Node(package="aic_vision_trad", executable="vision_node.py",
             name="vision_trad_node", output="screen",
             condition=LaunchConfigurationEquals("vision_backend", "trad"),
             parameters=[{"debug_dir": LaunchConfiguration("vision_debug_dir")}]),
    ])
