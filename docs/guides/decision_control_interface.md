# decision → control 接口约定（control 侧实现留白）

> 版本：2026-10-15（决策侧已实现并自验通过；control 侧按此文档接即可）
> 决策侧实现：`src/decision/aic_behavior_tree/src/control_bridge.cpp`
> 消息定义：`src/plugins/aic_msgs/msg/MaterialTask.msg`

## 1. 话题

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| decision → control | `/control/material_task` | `aic_msgs/MaterialTask` | reliable，深度 10（默认） |
| control → decision（预留） | `/control/task_result` | `aic_msgs/MaterialTaskResult`（未定义） | 未定 |

就绪握手（`/control/ready`）**当前未启用**：决策侧 `wait_ready=false`，不等 control 起来就开跑。
control 侧若需要握手机制，先提，再在 `aic_decision_params.yaml` 的 `control.wait_ready` 打开。

## 2. MaterialTask 消息

```text
uint8 COMMAND_GOTO=1
uint8 COMMAND_HANDLE=2
uint8 COMMAND_HOME=3

uint8 command          # GOTO=移动到位 / HANDLE=对该物料执行处理 / HOME=回原点（预留）
int32 visit_index      # 第几次移动，1..12
int32 slot_id          # 几何库位号 1..12（row1 从左到右为 1..4，见 warehouse_layout）
int32 row              # 行 1..3
int32 col              # 列 1..4
string material_type   # "0".."4"；GOTO 任务恒为 "0"，HANDLE 任务为该位点识别结果
string note            # 备注透传（决策侧的调试/播报信息，control 可忽略）
```

注意 `material_type` 是**字符串**——与 vision 上行 string 同型，便于日志对照；
control 解析时按 int 转一次即可（非法值决策侧已兜底，HANDLE 只会发 "1".."4"）。

## 3. 决策侧已验证的任务流（自验脚本 logs/verify_*_tasks.csv）

```text
visit 1..12：
  GOTO  (slot_id = 蛇形顺序 4,3,2,1,5,6,7,8,12,11,10,9)
  HANDLE(同位点, material_type = "1".."4" 或超时跳过)
总计：GOTO 12 条 + HANDLE 10~12 条，全部为 fire-and-forget（不等待回执）
```

每条 GOTO 与 HANDLE 间隔 = 该位点识别耗时（假桩下约 20 ms；真 vision 下 = 识别时间，
上限 `vision_timeout_s`）。**当前决策不等 control 回执**，control 侧按队列消化即可。

## 4. control 侧需要实现/决策的事项（占位清单）

1. 三轴运动的执行器抽象：收到 GOTO 后移动到 `slot_id` 对应物理坐标
   （坐标标定值属现场知识，建议 control 侧自己持有一份 slot→坐标 表，不要塞进决策）。
2. HANDLE 的分类型动作差异（划痕/污渍/缺角/孔洞 → 抓取与码放策略）。
3. 回执机制：如果控制层会失败（丢步、夹取失败），需要一条回执话题 + 决策侧重试子树；
   这会改动主树（`HandleMaterial*` 子树里加 WaitForControl 节点），接口先在这里挂账。
4. 急停/归位：HOME 命令已留字面量，触发条件（比赛结束 / 异常）待定。
