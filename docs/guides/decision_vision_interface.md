# vision ↔ decision 接口约定

> 版本：2026-10-15（决策侧已实现并自验通过）
> 决策侧实现：`src/decision/aic_behavior_tree/src/vision_client.cpp`

## 1. 话题

| 方向 | 话题 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| vision → decision | `/vision/material_type` | `std_msgs/String` | reliable + volatile，深度 10（双方默认即可互通） | 每个位点一次识别结果 |
| vision → dashboard（诊断，可选） | `/vision/detail` | `std_msgs/String`（JSON） | 默认（reliable，深度 10） | **只给看板写「运行日志」用**，决策不订阅；每次结果一条，超时也发一条（`type=0`） |
| decision → vision（可选） | `/vision/trigger` | `std_msgs/Empty` | 默认 | 采集请求。**当前决策侧未启用**（`vision_trigger_enable=false`）；vision 若为「来图即识别」模式可忽略 |

`/vision/detail` 的 payload 是单行 JSON（`ensure_ascii=false`），字段固定：

```json
{"type": 4, "name": "hole", "cn": "孔洞",
 "reason": "hole enclosed=1.22 frac=0.016",
 "ms": 316, "retry": 1, "r": 85.1, "refined": true, "n_comp": 1,
 "feat": {"frac": 0.0157, "circ": 0.273, "elong": 1.1, "rad": 0.11, "fill": 0.25,
          "solidity": 0.45, "enclosed": 1.218, "drop": 60.1, "area": 356.0}}
```

| 字段 | 含义 |
|---|---|
| `type` / `name` / `cn` | 类型号 1..4（失败为 0）/ 内部名 / 中文名 |
| `reason` | 命中的那条判据（含它的实测值），排障时先看这里 |
| `ms` | 从 `/vision/trigger` 到发布结果的耗时 |
| `retry` | 这一拍试了几帧才判出（帧停稳需要几帧，正常 1–3） |
| `r` / `refined` | 盘半径（px）/ 圆精修是否被采纳（`false` 时会退回 Hough 值，特征会略糙） |
| `n_comp` | 盘内暗连通域个数（>1 说明有干扰，判据按优先级取用） |
| `feat` | 判据实际读的归一化特征：`frac` 面积占比、`circ` 圆度、`elong` 长宽比、`rad` 重心半径、`fill` 填充率、`solidity` 凸包填充、`enclosed` 闭合轮廓围住比、`drop` 亮度落差 |

> 契约上这条话题**可以不要**：`aic_vision_trad` 侧由参数 `detail_topic` 控制（置空即关闭），
> 决策与 PLC 链路只读 `/vision/material_type`，不受影响。

参数化入口：`aic_decision_params.yaml` 的 `vision_topic` / `vision_trigger_topic` / `vision_trigger_enable`
（参数名就是这个下划线形式，没有 `vision.` 这种分组前缀）。

## 2. 消息语义

- payload 是一个字符串，取值约定 `"1".."4"`：
  - `1` = 划痕、`2` = 污渍、`3` = 缺角、`4` = 孔洞
- **一个位点只发一条**（一拍结果）。决策按「GOTO 之后的新结果」过滤：`VisionClient`
  内部维护有效结果序号 `seq`（非法值不递增），`WaitVisionResult` 只接受 `seq > GOTO 时基准` 的消息。
- 非法字符串（非 `"1".."4"`）会被决策**丢弃并计数**（`invalid`），不会触发处理子树；
  对应位点最终记 `TIMEOUT`。vision 侧无需为失败场景发明特殊值，不发即可。

## 3. 时序

```text
decision                          vision
  |-- GOTO (MaterialTask) -------->|       （经 /control/material_task，见 control 接口文档）
  |    [可选] trigger(Empty) ----->|
  |                                |-- 拍照 + 识别
  |<------ String "1".."4" --------|
  |  （超时 vision_timeout_s 未收到 -> 该位点记 TIMEOUT，巡视继续）
```

- 决策等待超时由 ROS 参数 `vision_timeout_s` 控制（默认 3.0 s，自验场景 B 用 0.6 s 验证过超时路径）。
- vision 慢一点没关系：决策在结果到达前一直处于 `RUNNING`，不会丢位点。

## 4. 给 vision 侧的实现提示（未实现，占位）

1. 发布频率约束：**每次 GOTO 后最多一条**；重复发会被 `seq` 过滤掉旧的、只认最新一条，
   多发不会崩但没意义。
2. QoS 别改 best_effort：决策订阅端是 reliable，`best_effort` 发布者与 reliable 订阅者
   **不兼容**，表现为订阅成功但一条都收不到且无报错（坑点，QoS 表见
   https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html）。
3. 世界坐标 / 相机参数等不属于本话题；后续 PnP 解算结果如果要上行，另开话题，
   不要往这条 string 里塞复合信息。
