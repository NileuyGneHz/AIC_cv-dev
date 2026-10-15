# 决策行为树架构说明（aic_sort.xml）

> 版本：2026-10-15，端到端自验 PASS（场景 A 正常 / 场景 B 抗扰动）。
> 本文对应 `src/decision/aic_behavior_tree/config/trees/aic_sort.xml`。

## 1. 主树形状

```text
Sequence ROOT
  InitRun                       加载货架布局、复位游标/结果表、初始化黑板默认值
  WhileSlotsRemain  (装饰器)    自定义循环：还有位点就重复 OneSlot
    Sequence OneSlot
      GotoNextSlot              游标推进 + 发 GOTO 任务 + 采样 vision seq 基准
      Fallback VisionOrTimeout
        WaitVisionResult        StatefulActionNode：等本位点的新识别结果（RUNNING）
        RecordSlotResult        超时兜底：记一行 TIMEOUT，巡视继续
      Switch4 variable={material_type}
        SubTree HandleMaterial1..4   按类型分发（端口映射见下）
        SkipSlot                     "0"/非法值兜底（只留日志，不重复记账）
  RunSummary                    汇总：逐位点结果 + GOTO/HANDLE 计数
```

12 位点蛇形顺序（右下出发）：`4,3,2,1 → 5,6,7,8 → 12,11,10,9`，
由 `aic_warehouse` 包生成并配置于 `config/warehouse_layout.yaml`。

## 2. 黑板键（唯一登记处 bb_keys.hpp）

| 键 | 类型 | 写入者 | 读者 |
|---|---|---|---|
| `material_type` | string "0".."4" | WaitVisionResult（GotoNextSlot 先清 "0"） | Switch4 / 子树 |
| `material_raw` | string | WaitVisionResult | RecordSlotResult |
| `slot_id` / `row` / `col` | int | GotoNextSlot | Dispatch / Record |
| `visit_index` | int 1..12 | GotoNextSlot | 全部 |
| `vision_seq_before_move` | uint64 | GotoNextSlot | WaitVisionResult |
| `slot_start_time` | double | GotoNextSlot | RecordSlotResult |
| `vision_timeout` | double | InitRun / ROS 参数 | WaitVisionResult |

## 3. 三个踩过的坑（改树前必读）

1. **while 型装饰器必须每轮 `haltChild()`**：v4 的 Sequence 返回 SUCCESS 后停在
   「已完成」状态，不复位则下一轮 `executeTick()` 秒回 SUCCESS——12 个位点只走 1 个。
2. **`tick()` 在子树 RUNNING 期间被反复重入**：本轮起点游标只能在迭代首次进入时快照
   （`child_started_` 标志），每次重入都重照会让推进检测必然误报。
3. **`while(hasNext())` 会吞掉最后一个位点**：`hasNext()` 语义是「当前之后还有没有」，
   visit 12 的等待发生在它已为 false 之后。必须无条件循环、在迭代开头才检查 `hasNext()`。

三个坑都体现在 `WhileSlotsRemain::tick()` 的实现与注释里（`src/nodes.cpp`）。

## 4. SubTree 的黑板隔离（4.10 实测）

BT.CPP 4.10 的 `<SubTree>` **黑板是隔离的**，库内不存在 `__shared_blackboard` 属性。
所以四个 `HandleMaterial*` 在主树里全部显式映射了位点端口：

```xml
<SubTree ID="HandleMaterial1"
         visit_index="{visit_index}" slot_id="{slot_id}" row="{row}" col="{col}"
         material_type="{material_type}" material_raw="{material_raw}"
         slot_start_time="{slot_start_time}"/>
```

子树里新增要读的黑板键时，**四处映射都要同步加**（漏加的表现：HANDLE 发到
slot 00 / type 0 的假任务）。

## 5. 怎么扩（control 实现接入时）

- 四个 `HandleMaterial*` 子树内部就是留白区：加 `WaitControlDone`（等回执）、
  分支码放、重试等节点，主树不用动。
- 主树形状变化后，`TreeNodesModel` 段要同步（Groot2 靠它渲染）。
- tick 频率 50 Hz（`bt_runner` 参数 `tick_rate_hz`），单轮 tree tick 内做一次识别等待。
