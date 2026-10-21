(*
 * AIC 三轴分拣 —— PLC 侧主程序（骨架）
 *
 * 角色：本程序是「PLC」这一端，通过 RSC 或 Modbus 与 aic_plc_gateway 握手。
 *       网关那端的实现在 src/plugins/aic_plc_gateway/scripts/plc_gateway.py。
 *
 * 变量声明：见 docs/guides/plcnext_rsc_setup.md §2.1（udt_Camera / udt_Axis /
 *           udt_Job / udt_Shelf / udt_Diag / udt_Cmd / udt_Di），本文件只放 PROGRAM。
 *           变量名必须与 config/plc_rsc_map.yaml 的 path 逐字一致，否则 RSC 读回 NotExists。
 *
 * 适用范围：本文件是**可编译的骨架**，不含伺服使能、回零、安全门等真机逻辑。
 *           上机前必须补：急停链路、软限位、伺服 Ready 检查。
 *
 * 与网关的握手（时序细节见 docs/guides/plc_integration.md §7）
 *   本程序写：Pos、Task.Cmd、Cmd.Reset
 *   本程序读：Di.InPos、Camera.State、Camera.Data、Task.State、Shelf.*、Diag.*
 *
 * 两个不能省的设计点
 * -----------------
 * 1) 执行位要「保持到被接受」，不能发一个扫描周期就撤。
 *    网关轮询 50 Hz，一个扫描周期的脉冲可能整好落在两次轮询之间被漏掉。
 *    所以 Task.Cmd 保持，直到 Task.State 变成预期值（1=GOTO / 2=HANDLE）才清 0。
 *    Modbus 下网关自己会把 Cmd 清 0；RSC 下网关写不动这个变量，必须由本程序清。
 *    两种情况用同一段逻辑都能覆盖。
 *
 * 2) Camera.Request 不在这里用。
 *    本项目的网关在「轴到位」时自动触发视觉（_on_arrived -> /vision/trigger），
 *    再由网关把结果写进 Camera.State / Camera.Data。
 *    PLC 如果再拉 Camera.Request 会重复触发一次识别。
 *)

PROGRAM Main
VAR
    (* ---- 网关写的（本程序只读）---- *)
    Camera : udt_Camera;
    Axis   : udt_Axis;
    Task   : udt_Job;
    Shelf  : udt_Shelf;
    Diag   : udt_Diag;
    Di     : udt_Di;

    (* ---- 本程序写的（网关读）---- *)
    Pos : INT;              (* 目标库位 1..12，对应官方变量 Pos *)
    Cmd : udt_Cmd;          (* Trigger/Attach/Detach/Reset，宏动作流程下只用 Reset *)

    (* ---- 内部状态 ---- *)
    Start   : BOOL;         (* 接 HMI 的绿色启动按钮 *)
    Stop    : BOOL;         (* 接 HMI 的红色停止按钮 *)
    state   : INT;          (* 见下方 CASE 的注释 *)
    slot    : INT;          (* 当前库位 1..12 *)
    hbLast  : DINT;         (* 上一次看到的心跳值 *)
    wdTimer : TON;          (* 看门狗：心跳 3 s 不变 -> FAULT *)
    errCode : INT;          (* 0=无；10=心跳丢失；20=识别失败；30=抓放超时 *)
    tStep   : TON;          (* 单步超时 *)
END_VAR

VAR CONSTANT
    N_SLOT : INT := 12;
    T_STEP : TIME := T#40S;     (* 单步超时：抓放最慢约 13 s，留足余量 *)
    T_WD   : TIME := T#3S;      (* 心跳看门狗 *)
END_VAR

(* 状态编号
     0  IDLE            等启动
    10  SEND_GOTO       写 Pos + Task.Cmd=1
    20  WAIT_GOTO_ACK   等 Task.State=1（或 Cmd 被网关清 0），然后清 Cmd
    30  WAIT_INPOS      等 Di.InPos（轴到位，网关同时已触发拍照）
    40  WAIT_CAM        等 Camera.State=2(DONE)；3(FAIL) -> 故障
    50  SEND_HANDLE     写 Task.Cmd=2
    60  WAIT_HANDLE_ACK 等 Task.State=2，然后清 Cmd
    70  WAIT_DONE       等 Task.State=5(DONE)
    80  FINISHED        12 格放满
    90  FAULT           停在这里，等复位
*)

(* ---------------- 看门狗：心跳 3 s 不变就停 ---------------- *)
wdTimer(IN := (Diag.Heartbeat = hbLast), PT := T_WD);
IF Diag.Heartbeat <> hbLast THEN
    hbLast := Diag.Heartbeat;
END_IF

IF wdTimer.Q AND state <> 90 THEN
    errCode := 10;
    state   := 90;
END_IF

(* ---------------- 主状态机 ---------------- *)
CASE state OF

0:  (* IDLE *)
    Task.Cmd := 0;
    Cmd.Reset := FALSE;
    IF Start AND NOT Stop THEN
        slot    := 1;
        errCode := 0;
        tStep(IN := FALSE);
        state   := 10;
    END_IF

10: (* SEND_GOTO *)
    Pos      := slot;
    Task.Cmd := 1;              (* 1 = GOTO *)
    tStep(IN := FALSE);
    state    := 20;

20: (* WAIT_GOTO_ACK：保持 Cmd 直到网关接受 *)
    IF (Task.Cmd = 0) OR (Task.State = 1) THEN
        Task.Cmd := 0;          (* Modbus 下网关已清；RSC 下由这里清 *)
        tStep(IN := FALSE);
        state    := 30;
    ELSIF tStep.Q THEN
        errCode := 30;
        state   := 90;
    ELSE
        tStep(IN := TRUE, PT := T_STEP);
    END_IF

30: (* WAIT_INPOS *)
    IF Di.InPos THEN
        tStep(IN := FALSE);
        state := 40;
    ELSIF tStep.Q THEN
        errCode := 30;
        state   := 90;
    ELSE
        tStep(IN := TRUE, PT := T_STEP);
    END_IF

40: (* WAIT_CAM *)
    CASE Camera.State OF
        2:  (* DONE：Camera.Data 就是物料编号 1..4 *)
            tStep(IN := FALSE);
            state := 50;
        3:  (* FAIL *)
            errCode := 20;
            state   := 90;
    ELSE
        IF tStep.Q THEN
            errCode := 20;
            state   := 90;
        ELSE
            tStep(IN := TRUE, PT := T_STEP);
        END_IF
    END_CASE

50: (* SEND_HANDLE：类型由网关按 Camera.Data 带进 GOTO/HANDLE 任务 *)
    Task.Cmd := 2;              (* 2 = HANDLE *)
    tStep(IN := FALSE);
    state    := 60;

60: (* WAIT_HANDLE_ACK *)
    IF (Task.Cmd = 0) OR (Task.State = 2) THEN
        Task.Cmd := 0;
        tStep(IN := FALSE);
        state    := 70;
    ELSIF tStep.Q THEN
        errCode := 30;
        state   := 90;
    ELSE
        tStep(IN := TRUE, PT := T_STEP);
    END_IF

70: (* WAIT_DONE *)
    IF Task.State = 5 THEN
        tStep(IN := FALSE);
        IF slot >= N_SLOT THEN
            state := 80;
        ELSE
            slot  := slot + 1;
            state := 10;
        END_IF
    ELSIF Task.State = 9 THEN
        errCode := 30;
        state   := 90;
    ELSIF tStep.Q THEN
        errCode := 30;
        state   := 90;
    ELSE
        tStep(IN := TRUE, PT := T_STEP);
    END_IF

80: (* FINISHED：12 格放满，等人工按停止/复位 *)
    Task.Cmd := 0;
    IF Stop THEN
        state := 0;
    END_IF

90: (* FAULT：停住不放行。Shelf 与 errCode 留给 eHMI / 看板显示 *)
    Task.Cmd := 0;
    IF Start AND NOT Stop THEN     (* 人工确认后重新启动 *)
        slot    := 1;
        errCode := 0;
        state   := 10;
    END_IF

ELSE
    state := 0;
END_CASE
