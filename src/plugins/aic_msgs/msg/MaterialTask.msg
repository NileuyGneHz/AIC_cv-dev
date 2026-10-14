# 决策（decision）→ 控制桥接（control）的任务指令。
# 契约见 docs/guides/decision_control_interface.md；control 侧实现留白。
#
# 设计边界：决策只回答「去哪个库位、这块物料算什么类型、做什么动作」，
# 不做坐标换算、不定义行程与速度——那些属于 control（见 control 侧文档）。

std_msgs/Header header

# ---- 库位标识（1-based）----
# slot_id：几何库位号 1..12，row-major，id = (row-1)*cols + col
# visit_index：本次运行中的第几次移动 1..12（顺序号，便于复盘与播报）
uint8 slot_id
uint8 visit_index
uint8 row
uint8 col

# ---- 物料类型 ----
# 1..4 对应四类缺陷物料（暂名：1=划痕 2=污渍 3=缺角 4=孔洞，待赛题确认）
uint8 material_type

# ---- 动作 ----
# "GOTO"   ：移动到该库位（decision 在每个位点识别前下发）
# "HANDLE" ：对该库位物料执行类型相关处理（抓取/码放，具体动作由 control 决定）
# "HOME"   ：回到原点/起始位（预留）
string action

# 调试备注；control 可忽略
string note

# 常量
uint8 MATERIAL_UNKNOWN = 0
uint8 MATERIAL_SCRATCH = 1
uint8 MATERIAL_STAIN = 2
uint8 MATERIAL_CHIP = 3
uint8 MATERIAL_HOLE = 4

string ACTION_GOTO = "GOTO"
string ACTION_HANDLE = "HANDLE"
string ACTION_HOME = "HOME"
