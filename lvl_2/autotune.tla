-------------------------------- MODULE autotune --------------------------------
EXTENDS Integers, Sequences, TLC

CONSTANTS
    N,
    INPUT_DATA_SIZE,
    GLOBAL_MEMORY_ACCESS,
    LOCAL_MEMORY_ACCESS,
    LOCAL_MEMORY_SIZE,
    UNITS_PER_DEVICE,
    DEVICES,
    PES_PER_UNIT

(*--algorithm Main {
    variables
              (* MSGS *)
              GO            = "GO",
              STOP          = "STOP",
              DONE          = "DONE",
              GOWG          = "GOWG",
              GOWARP        = "GOWARP",
              DONEWARP      = "DONEWARP",
              STOPWARPS     = "STOPWARPS",

              (* flags *)
              hostFlag      = FALSE,
              clockFlag     = FALSE,
              final         = FALSE,

              (* Host <-> Device channels *)
              hst_d = <<>>,
              d_hst = <<>>,

              (* Device <-> WarpScheduler channels *)
              dev_sch = [d \in 0..(DEVICES-1) |->
                        [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],
              sch_dev = [d \in 0..(DEVICES-1) |->
                        [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],

              (* WarpScheduler <-> Unit channels *)
              sch_u = [d \in 0..(DEVICES-1) |->
                      [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],
              u_sch = [d \in 0..(DEVICES-1) |->
                      [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],

              (* time *)
              globalTime = 0,
              Tmin = 60,

              (* memory *)
              globalMemory = [i \in 0..(INPUT_DATA_SIZE - 1) |-> 0],

              (* tuning params *)
              workGroupSize          = 0,
              nWorkGroups            = 0,
              tileSize               = 0,

              (* service variables *)
              nWorkingDevices        = 0,
              nWorkingUnitsPerDevice = 0,
              nWorkingPEsPerUnit     = 0,
              allWorkingUnits        = 0,
              nRunningUnits          = 0,
              nWaitingUnits          = 0,
              nWarpsPerUnit          = 0,
              nWarps                 = 0,
              nInstructions          = 7,

              aoutput                = 100000,
              unitsFinal             = 0,

              (* workgroups queue: sequence of <<wgId, readyToRun>> *)
              workgroups = <<>>,

              (* barrierIn[d][u * nWarpsPerUnit + w] — flat per device *)
              (* max size = UNITS_PER_DEVICE * (INPUT_DATA_SIZE \div PES_PER_UNIT) *)
              barrierIn = [d \in 0..(DEVICES-1) |->
                          [i \in 0..(UNITS_PER_DEVICE * INPUT_DATA_SIZE - 1) |-> 0]],

              (* isWarpReadyToRun — same indexing *)
              isWarpReadyToRun = [d \in 0..(DEVICES-1) |->
                                 [i \in 0..(UNITS_PER_DEVICE * INPUT_DATA_SIZE - 1) |-> 0]];

    (* ========== MACROS ========== *)

    macro Send1(m, chan) {
        chan := Append(chan, m);
    }
    macro Rcv1(v, chan) {
        await chan # <<>>;
        v := Head(chan);
        chan := Tail(chan);
    }
    macro Send2(m, chan, d, u) {
        chan[d][u] := Append(chan[d][u], m);
    }
    macro Rcv2(v, chan, d, u) {
        await chan[d][u] # <<>>;
        v := Head(chan[d][u]);
        chan[d][u] := Tail(chan[d][u]);
    }

    (* ========== CLOCK ========== *)

    fair process (Clock = <<0,0,0>>)
    {
        c0: await clockFlag = TRUE;
        loop_cl: while (~final) {
        c1:         if ((nRunningUnits = (allWorkingUnits - nWaitingUnits))
                       /\ (allWorkingUnits # 0)
                       /\ (nWaitingUnits # allWorkingUnits)) {
        c2:            nRunningUnits := 0;
                       globalTime := globalTime + 1;
                    };
                 };
    }

    (* ========== UNIT ========== *)

    fair process (UnitProc \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) })
    variable dId = 0, uId = 0,
             msg = <<>>,
             wgId = 0, warpId = 0, instrId = 0,
             pesIdx = 0, u_i = 0,
             t = 0, bd = 0, bu = 0, w = 0,
             longWorkFlag = FALSE,
             startTime = 0, curTime = 0,
             localMemory_u = [lm \in 0..(LOCAL_MEMORY_SIZE - 1) |-> 100000],
             localId_u     = [li \in 0..(INPUT_DATA_SIZE - 1) |-> 0],
             globalOffset_u = [go2 \in 0..(INPUT_DATA_SIZE - 1) |-> 0],
             tileIdx_u     = [ti \in 0..(INPUT_DATA_SIZE - 1) |-> 0],
             readyToMin_u  = [rm \in 0..(INPUT_DATA_SIZE - 1) |-> 0],
             minVal = 0;
    {
        unit_init:
            await self[2] < nWorkingDevices
               /\ self[3] < nWorkingUnitsPerDevice;
            dId := self[2];
            uId := self[3];

        (* ---- outer loop: wait for GO or STOP ---- *)
        unit_outer:
            Rcv2(msg, sch_u, dId, uId);

        unit_outer_check:
            if (msg[1] = GO) {
                wgId := msg[2];
                goto unit_init_local;
            } else if (msg[1] = STOP) {
                unitsFinal := unitsFinal + 1;
                goto unit_done;
            } else {
                goto unit_outer;
            };

        (* ---- init local memory and registers ---- *)
        unit_init_local:
            u_i := 0;

        unit_init_lm_loop:
            if (u_i >= LOCAL_MEMORY_SIZE) {
                u_i := 0;
                goto unit_init_reg_loop;
            };

        unit_init_lm_set:
            localMemory_u[u_i] := 100000;
            u_i := u_i + 1;
            goto unit_init_lm_loop;

        unit_init_reg_loop:
            if (u_i >= INPUT_DATA_SIZE) {
                goto unit_warp_loop;
            };

        unit_init_reg_set:
            localId_u[u_i] := 0;
            globalOffset_u[u_i] := 0;
            tileIdx_u[u_i] := 0;
            readyToMin_u[u_i] := 0;
            u_i := u_i + 1;
            goto unit_init_reg_loop;

        (* ---- inner loop: receive warp instructions ---- *)
        unit_warp_loop:
            Rcv2(msg, sch_u, dId, uId);

        unit_warp_check:
            if (msg[1] = GOWARP) {
                warpId := msg[2];
                instrId := msg[3];
                goto unit_set_time;
            } else if (msg[1] = STOPWARPS) {
                goto unit_outer;
            } else {
                goto unit_warp_loop;
            };

        unit_set_time:
            startTime := globalTime;
            curTime := globalTime;

        (* ---- dispatch by instrId ---- *)
        unit_dispatch:
            if (instrId = 0) {
                goto unit_instr0;
            } else if (instrId = 1) {
                goto unit_instr1;
            } else if (instrId = 2) {
                goto unit_instr2;
            } else if (instrId = 3) {
                goto unit_instr3;
            } else if (instrId = 4) {
                goto unit_instr4;
            } else if (instrId = 5) {
                goto unit_instr5;
            } else if (instrId = 6) {
                goto unit_instr6;
            } else {
                goto unit_send_done_warp;
            };

        (* ==== instrId 0: compute localId ==== *)
        unit_instr0:
            pesIdx := 0;

        unit_instr0_loop:
            if (pesIdx >= nWorkingPEsPerUnit) {
                goto unit_send_done_warp;
            };

        unit_instr0_body:
            if (workGroupSize > nWorkingPEsPerUnit) {
                localId_u[pesIdx * nWarpsPerUnit + warpId] :=
                    pesIdx + warpId * nWorkingPEsPerUnit;
            } else {
                localId_u[pesIdx * nWarpsPerUnit + warpId] := pesIdx;
            };
            pesIdx := pesIdx + 1;
            goto unit_instr0_loop;

        (* ==== instrId 1: compute globalOffset ==== *)
        unit_instr1:
            pesIdx := 0;

        unit_instr1_loop:
            if (pesIdx >= nWorkingPEsPerUnit) {
                goto unit_send_done_warp;
            };

        unit_instr1_body:
            globalOffset_u[pesIdx * nWarpsPerUnit + warpId] :=
                tileSize * (wgId * workGroupSize
                    + localId_u[pesIdx * nWarpsPerUnit + warpId]);
            pesIdx := pesIdx + 1;
            goto unit_instr1_loop;

        (* ==== instrId 2: check bounds, set predicate register ==== *)
        unit_instr2:
            pesIdx := 0;

        unit_instr2_loop:
            if (pesIdx >= nWorkingPEsPerUnit) {
                goto unit_send_done_warp;
            };

        unit_instr2_body:
            if (tileIdx_u[warpId]
                + globalOffset_u[pesIdx * nWarpsPerUnit + warpId]
                < INPUT_DATA_SIZE) {
                readyToMin_u[pesIdx * nWarpsPerUnit + warpId] := 1;
            };
            pesIdx := pesIdx + 1;
            goto unit_instr2_loop;

        (* ==== instrId 3: min + long_work ==== *)
        unit_instr3:
            pesIdx := 0;
            longWorkFlag := FALSE;

        unit_instr3_loop:
            if (pesIdx >= nWorkingPEsPerUnit) {
                goto unit_instr3_after;
            };

        unit_instr3_body:
            if (readyToMin_u[pesIdx * nWarpsPerUnit + warpId] = 1) {
                minVal := globalMemory[
                    tileIdx_u[warpId]
                    + globalOffset_u[pesIdx * nWarpsPerUnit + warpId]];
                if (localMemory_u[localId_u[pesIdx * nWarpsPerUnit + warpId]] > minVal) {
                    localMemory_u[localId_u[pesIdx * nWarpsPerUnit + warpId]] := minVal;
                };
                longWorkFlag := TRUE;
                readyToMin_u[pesIdx * nWarpsPerUnit + warpId] := 0;
            };
            pesIdx := pesIdx + 1;
            goto unit_instr3_loop;

        unit_instr3_after:
            if (longWorkFlag) {
                goto unit_long_work;
            } else {
                goto unit_send_done_warp;
            };

        (* ---- long_work subroutine ---- *)
        unit_long_work:
            if (globalTime >= startTime + GLOBAL_MEMORY_ACCESS) {
                goto unit_send_done_warp;
            };

        unit_work_step:
            curTime := globalTime;
            nRunningUnits := nRunningUnits + 1;

        unit_work_await:
            await globalTime >= curTime + 1;
            goto unit_long_work;

        (* ==== instrId 4: tileIdx++ and loop back ==== *)
        unit_instr4:
            tileIdx_u[warpId] := tileIdx_u[warpId] + 1;

        unit_instr4_check:
            if (tileIdx_u[warpId] < tileSize) {
                instrId := instrId - 3;
                goto unit_send_done_warp;
            } else {
                goto unit_send_done_warp;
            };

        (* ==== instrId 5: barrier ==== *)
        unit_instr5:
            barrierIn[dId][uId * nWarpsPerUnit + warpId] := 1;
            t := 0;
            bd := 0;

                unit_barrier_d_loop:
            if (bd >= nWorkingDevices) {
                goto unit_barrier_check;
            };

        unit_barrier_d_body:
            bu := 0;

        unit_barrier_u_loop:
            if (bu >= nWorkingUnitsPerDevice) {
                bd := bd + 1;
                goto unit_barrier_d_loop;
            };

        unit_barrier_u_body:
            w := 0;

        unit_barrier_w_loop:
            if (w >= nWarpsPerUnit) {
                bu := bu + 1;
                goto unit_barrier_u_loop;
            };

        unit_barrier_w_body:
            t := t + barrierIn[bd][bu * nWarpsPerUnit + w];
            nWaitingUnits := t;
            w := w + 1;
            goto unit_barrier_w_loop;

        unit_barrier_check:
            if (t = nWarps) {
                bd := 0;
                goto unit_barrier_reset_d;
            } else {
                goto unit_send_done_warp;
            };

        unit_barrier_reset_d:
            if (bd >= nWorkingDevices) {
                goto unit_send_done_warp;
            };

        unit_barrier_reset_d_body:
            bu := 0;

        unit_barrier_reset_u:
            if (bu >= nWorkingUnitsPerDevice) {
                bd := bd + 1;
                goto unit_barrier_reset_d;
            };

        unit_barrier_reset_u_body:
            w := 0;

        unit_barrier_reset_w:
            if (w >= nWarpsPerUnit) {
                bu := bu + 1;
                goto unit_barrier_reset_u;
            };

        unit_barrier_reset_body:
            barrierIn[bd][bu * nWarpsPerUnit + w] := 0;
            nWaitingUnits := 0;
            w := w + 1;
            goto unit_barrier_reset_w;

        (* ==== instrId 6: final reduction ==== *)
        unit_instr6:
            if (localId_u[0 * nWarpsPerUnit + warpId] = 0) {
                u_i := 0;
                goto unit_instr6_reduce_loop;
            } else {
                goto unit_send_done_warp;
            };

        unit_instr6_reduce_loop:
            if (u_i >= nWarpsPerUnit * nWorkingPEsPerUnit) {
                goto unit_instr6_global_min;
            };

        unit_instr6_reduce_body:
            if (localMemory_u[localId_u[0]] > localMemory_u[u_i]) {
                localMemory_u[localId_u[0]] := localMemory_u[u_i];
            };
            startTime := curTime;

        unit_instr6_reduce_lw:
            if (globalTime >= startTime + LOCAL_MEMORY_ACCESS) {
                u_i := u_i + 1;
                goto unit_instr6_reduce_loop;
            };

        unit_instr6_reduce_ws:
            curTime := globalTime;
            nRunningUnits := nRunningUnits + 1;

        unit_instr6_reduce_wa:
            await globalTime >= curTime + 1;
            goto unit_instr6_reduce_lw;

        unit_instr6_global_min:
            if (aoutput > localMemory_u[localId_u[0]]) {
                aoutput := localMemory_u[localId_u[0]];
            };
            startTime := curTime;

        unit_instr6_global_lw:
            if (globalTime >= startTime + GLOBAL_MEMORY_ACCESS) {
                goto unit_send_done_warp;
            };

        unit_instr6_global_ws:
            curTime := globalTime;
            nRunningUnits := nRunningUnits + 1;

        unit_instr6_global_wa:
            await globalTime >= curTime + 1;
            goto unit_instr6_global_lw;

        (* ---- send donewarp back to scheduler ---- *)
        unit_send_done_warp:
            Send2(<<DONEWARP, instrId>>, u_sch, dId, uId);
            goto unit_warp_loop;

        unit_done:
            skip;
    }

    (* ========== WARP SCHEDULER ========== *)

    fair process (WarpSch \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1) })
    variable dId = 0, uId = 0,
             msg = <<>>,
             wgId = 0, warpId = 0, instrId = 0,
             warps = <<>>,    \* local queue of <<warpId, instrId>>
             warpEntry = <<>>;
    {
        ws_init:
            await self[2] < nWorkingDevices
               /\ self[3] < nWorkingUnitsPerDevice;
            dId := self[2];
            uId := self[3];

        (* ---- outer loop ---- *)
        ws_outer:
            Rcv2(msg, dev_sch, dId, uId);

        ws_outer_check:
            if (msg[1] = GO) {
                wgId := msg[2];
                goto ws_send_go_to_unit;
            } else if (msg[1] = STOP) {
                \* forward stop to unit
                Send2(<<STOP, 0>>, sch_u, dId, uId);
                goto ws_done;
            } else {
                goto ws_outer;
            };

        (* send GO wgId to unit *)
        ws_send_go_to_unit:
            Send2(<<GO, wgId>>, sch_u, dId, uId);

        (* fill warp queue *)
        ws_fill_warps:
            warpId := 0;
            warps := <<>>;

        ws_fill_loop:
            if (warpId >= nWarpsPerUnit) {
                goto ws_schedule_loop;
            };

        ws_fill_body:
            isWarpReadyToRun[dId][uId * nWarpsPerUnit + warpId] := 1;
            warps := Append(warps, <<warpId, 0>>);
            warpId := warpId + 1;
            goto ws_fill_loop;

        (* ---- scheduling loop ---- *)
        ws_schedule_loop:
            if (warps = <<>>) {
                goto ws_all_warps_done;
            };

        ws_take_warp:
            warpEntry := Head(warps);
            warps := Tail(warps);
            warpId := warpEntry[1];
            instrId := warpEntry[2];

        ws_check_ready:
            isWarpReadyToRun[dId][uId * nWarpsPerUnit + warpId] :=
                1 - barrierIn[dId][uId * nWarpsPerUnit + warpId];

        ws_dispatch:
            if (isWarpReadyToRun[dId][uId * nWarpsPerUnit + warpId] = 1) {
                goto ws_send_to_unit;
            } else {
                \* not ready, put back
                warps := Append(warps, <<warpId, instrId>>);
                goto ws_schedule_loop;
            };

        ws_send_to_unit:
            Send2(<<GOWARP, warpId, instrId>>, sch_u, dId, uId);

        ws_wait_done:
            Rcv2(msg, u_sch, dId, uId);

        ws_after_done:
            instrId := msg[2] + 1;

        ws_requeue:
            if (instrId < nInstructions) {
                warps := Append(warps, <<warpId, instrId>>);
            };
            goto ws_schedule_loop;

        (* all warps finished *)
        ws_all_warps_done:
            Send2(<<STOPWARPS, 0>>, sch_u, dId, uId);

        ws_report_done:
            Send2(<<DONE, uId>>, sch_dev, dId, uId);
            goto ws_outer;

        ws_done:
            skip;
    }

    (* ========== DEVICE ========== *)

    fair process (DeviceProc \in { <<4,d,0>> : d \in 0..(DEVICES-1) })
    variable dId = 0, uId = 0,
             msgFromHost = "",
             msgFromSch = <<>>,
             wgId = 0,
             readyToRun = TRUE,
             wgEntry = <<>>,
             wgWaiting = TRUE;
    {
        dev_init:
            dId := self[2];

        dev_loop:
            Rcv1(msgFromHost, hst_d);

        dev_check:
            if (msgFromHost = GO) {
                goto dev_distribute;
            } else if (msgFromHost = STOP) {
                goto dev_send_stop;
            } else {
                goto dev_loop;
            };

        (* ---- distribute initial WGs to schedulers ---- *)
        dev_distribute:
            uId := 0;

        dev_dist_loop:
            if (uId >= nWorkingUnitsPerDevice) {
                goto dev_after_dist;
            };

        dev_dist_take:
            \* take from workgroups queue
            await workgroups # <<>>;
            wgEntry := Head(workgroups);
            workgroups := Tail(workgroups);
            wgId := wgEntry[1];
            readyToRun := wgEntry[2];

        dev_dist_check:
            if (readyToRun) {
                Send2(<<GO, wgId>>, dev_sch, dId, uId);
                uId := uId + 1;
                goto dev_dist_loop;
            } else {
                \* put back and retry
                workgroups := Append(workgroups, <<wgId, TRUE>>);
                goto dev_dist_take;
            };

        (* ---- after initial distribution ---- *)
        dev_after_dist:
            if (nWorkGroups <= nWorkingUnitsPerDevice) {
                uId := 0;
                goto dev_simple_wait;
            } else {
                uId := 0;
                goto dev_overflow_wait;
            };

        (* ---- simple: wait for all schedulers ---- *)
        dev_simple_wait:
            if (uId >= nWorkingUnitsPerDevice) {
                goto dev_report_done;
            };

        dev_simple_recv:
            with (u2 \in 0..(nWorkingUnitsPerDevice-1)) {
                await sch_dev[dId][u2] # <<>>;
                msgFromSch := Head(sch_dev[dId][u2]);
                sch_dev[dId][u2] := Tail(sch_dev[dId][u2]);
            };

        dev_simple_dec:
            allWorkingUnits := allWorkingUnits - 1;
            uId := uId + 1;
            goto dev_simple_wait;

        (* ---- overflow: more WGs than units ---- *)
        dev_overflow_wait:
            if (uId >= nWorkGroups - nWorkingUnitsPerDevice) {
                goto dev_overflow_final;
            };

        dev_overflow_recv:
            with (u2 \in 0..(nWorkingUnitsPerDevice-1)) {
                await sch_dev[dId][u2] # <<>>;
                msgFromSch := Head(sch_dev[dId][u2]);
                sch_dev[dId][u2] := Tail(sch_dev[dId][u2]);
            };

        dev_overflow_requeue:
            wgWaiting := TRUE;

        dev_overflow_take:
            if (~wgWaiting) {
                uId := uId + 1;
                goto dev_overflow_wait;
            };

        dev_overflow_take2:
            await workgroups # <<>>;
            wgEntry := Head(workgroups);
            workgroups := Tail(workgroups);
            wgId := wgEntry[1];
            readyToRun := wgEntry[2];

        dev_overflow_take_check:
            if (readyToRun) {
                \* find which unit sent done and send new WG to it
                with (u2 \in 0..(nWorkingUnitsPerDevice-1)) {
                    Send2(<<GO, wgId>>, dev_sch, dId, u2);
                };
                wgWaiting := FALSE;
                goto dev_overflow_take;
            } else {
                workgroups := Append(workgroups, <<wgId, TRUE>>);
                goto dev_overflow_take2;
            };

        dev_overflow_final:
            uId := 0;

        dev_overflow_final_loop:
            if (uId >= nWorkingUnitsPerDevice) {
                goto dev_report_done;
            };

        dev_overflow_final_recv:
            with (u2 \in 0..(nWorkingUnitsPerDevice-1)) {
                await sch_dev[dId][u2] # <<>>;
                msgFromSch := Head(sch_dev[dId][u2]);
                sch_dev[dId][u2] := Tail(sch_dev[dId][u2]);
            };

        dev_overflow_final_dec:
            allWorkingUnits := allWorkingUnits - 1;
            uId := uId + 1;
            goto dev_overflow_final_loop;

        dev_report_done:
            Send1(DONE, d_hst);
            goto dev_loop;

        (* ---- STOP ---- *)
        dev_send_stop:
            uId := 0;

        dev_stop_loop:
            if (uId >= nWorkingUnitsPerDevice) {
                goto dev_done;
            };

        dev_stop_send:
            Send2(<<STOP, 0>>, dev_sch, dId, uId);
            uId := uId + 1;
            goto dev_stop_loop;

        dev_done:
            skip;
    }

    (* ========== HOST ========== *)

    fair process (Host = <<5,0,0>>)
    variable msgFromDevice = "",
             deviceIdx = 0, wgIdx = 0;
    {
        h0: await hostFlag = TRUE;
        h1: final := FALSE;

        (* fill workgroups queue *)
        h_fill_wg:
            wgIdx := 0;

        h_fill_loop:
            if (wgIdx >= nWorkGroups) {
                goto h_send_go;
            };

        h_fill_body:
            workgroups := Append(workgroups, <<wgIdx, TRUE>>);
            wgIdx := wgIdx + 1;
            goto h_fill_loop;

        (* send GO to all devices *)
        h_send_go:
            deviceIdx := 0;

        h_send_go_loop:
            if (deviceIdx >= nWorkingDevices) {
                goto h_wait_done;
            };

        h_send_go_body:
            Send1(GO, hst_d);
            deviceIdx := deviceIdx + 1;
            goto h_send_go_loop;

        (* wait for DONE from all devices, then send STOP *)
        h_wait_done:
            deviceIdx := 0;

        h_wait_loop:
            if (deviceIdx >= nWorkingDevices) {
                goto h_wait_units;
            };

        h_wait_recv:
            Rcv1(msgFromDevice, d_hst);

        h_wait_stop:
            Send1(STOP, hst_d);
            deviceIdx := deviceIdx + 1;
            goto h_wait_loop;

        (* wait until all units finished *)
        h_wait_units:
            await unitsFinal = nWorkingUnitsPerDevice * nWorkingDevices;

        h_set_final:
            final := TRUE;

        h_done:
            skip;
    }

    (* ========== MAIN ========== *)

    fair process (Main = <<1,0,0>>)
    variable i_1 = 0, j_1 = 0;
    {
        m_loop_gm: while (i_1 < INPUT_DATA_SIZE) {
        m1:           globalMemory[i_1] := INPUT_DATA_SIZE - i_1;
        m2:           i_1 := i_1 + 1;
                   };
        m_WG:      with (k \in 2..(N - 1)) {
                      workGroupSize := INPUT_DATA_SIZE \div (2^(N - k))
                   };
        m_TS:      with (l \in 1..(N - 2)) {
                      tileSize := INPUT_DATA_SIZE \div (2^(N - l))
                   };
        m5:        if (workGroupSize * tileSize > INPUT_DATA_SIZE) {
        m6:           tileSize := INPUT_DATA_SIZE \div workGroupSize;
                   };
        m7:        nWorkGroups := INPUT_DATA_SIZE \div (workGroupSize * tileSize);
        m8:        nWorkingDevices := DEVICES;
        m9:        if (nWorkGroups <= UNITS_PER_DEVICE * DEVICES) {
        m10:          nWorkingDevices := nWorkGroups \div UNITS_PER_DEVICE;
                   };
        m11:       if ((nWorkGroups \div UNITS_PER_DEVICE) # 0) {
        m12:          nWorkingDevices := 1;
                   };
        m13:       nWorkingUnitsPerDevice := UNITS_PER_DEVICE;
        m14:       if (nWorkGroups <= UNITS_PER_DEVICE) {
        m15:          nWorkingUnitsPerDevice := nWorkGroups;
                   };
        m16:       nWorkingPEsPerUnit := PES_PER_UNIT;
        m17:       if (workGroupSize <= PES_PER_UNIT) {
        m18:          nWorkingPEsPerUnit := workGroupSize;
                   };
        m19:       allWorkingUnits := nWorkingDevices * nWorkingUnitsPerDevice;
        m20:       nWarpsPerUnit := workGroupSize \div nWorkingPEsPerUnit;
        m21:       nWarps := nWarpsPerUnit * nWorkingUnitsPerDevice * nWorkingDevices;
        m22:       hostFlag := TRUE;
                   clockFlag := TRUE;
    }
}
*)
\* BEGIN TRANSLATION (chksum(pcal) = "310b0fd9" /\ chksum(tla) = "ff1cff82")
\* Process variable dId of process UnitProc at line 122 col 14 changed to dId_
\* Process variable uId of process UnitProc at line 122 col 23 changed to uId_
\* Process variable msg of process UnitProc at line 123 col 14 changed to msg_
\* Process variable wgId of process UnitProc at line 124 col 14 changed to wgId_
\* Process variable warpId of process UnitProc at line 124 col 24 changed to warpId_
\* Process variable instrId of process UnitProc at line 124 col 36 changed to instrId_
\* Process variable dId of process WarpSch at line 472 col 14 changed to dId_W
\* Process variable uId of process WarpSch at line 472 col 23 changed to uId_W
\* Process variable wgId of process WarpSch at line 474 col 14 changed to wgId_W
VARIABLES GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, hostFlag, 
          clockFlag, final, hst_d, d_hst, dev_sch, sch_dev, sch_u, u_sch, 
          globalTime, Tmin, globalMemory, workGroupSize, nWorkGroups, 
          tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
          nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, nWaitingUnits, 
          nWarpsPerUnit, nWarps, nInstructions, aoutput, unitsFinal, 
          workgroups, barrierIn, isWarpReadyToRun, pc, dId_, uId_, msg_, 
          wgId_, warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
          startTime, curTime, localMemory_u, localId_u, globalOffset_u, 
          tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
          instrId, warps, warpEntry, dId, uId, msgFromHost, msgFromSch, wgId, 
          readyToRun, wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
          i_1, j_1

vars == << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, hostFlag, 
           clockFlag, final, hst_d, d_hst, dev_sch, sch_dev, sch_u, u_sch, 
           globalTime, Tmin, globalMemory, workGroupSize, nWorkGroups, 
           tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
           nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, nWaitingUnits, 
           nWarpsPerUnit, nWarps, nInstructions, aoutput, unitsFinal, 
           workgroups, barrierIn, isWarpReadyToRun, pc, dId_, uId_, msg_, 
           wgId_, warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
           startTime, curTime, localMemory_u, localId_u, globalOffset_u, 
           tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
           instrId, warps, warpEntry, dId, uId, msgFromHost, msgFromSch, wgId, 
           readyToRun, wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
           i_1, j_1 >>

ProcSet == {<<0,0,0>>} \cup ({ <<3,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) }) \cup ({ <<2,d,u>> : d \in 0..(DEVICES-1),
                                                                                                  u \in 0..(UNITS_PER_DEVICE-1) }) \cup ({ <<4,d,0>> : d \in 0..(DEVICES-1) }) \cup {<<5,0,0>>} \cup {<<1,0,0>>}

Init == (* Global variables *)
        /\ GO = "GO"
        /\ STOP = "STOP"
        /\ DONE = "DONE"
        /\ GOWG = "GOWG"
        /\ GOWARP = "GOWARP"
        /\ DONEWARP = "DONEWARP"
        /\ STOPWARPS = "STOPWARPS"
        /\ hostFlag = FALSE
        /\ clockFlag = FALSE
        /\ final = FALSE
        /\ hst_d = <<>>
        /\ d_hst = <<>>
        /\ dev_sch = [d \in 0..(DEVICES-1) |->
                     [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ sch_dev = [d \in 0..(DEVICES-1) |->
                     [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ sch_u = [d \in 0..(DEVICES-1) |->
                   [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ u_sch = [d \in 0..(DEVICES-1) |->
                   [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ globalTime = 0
        /\ Tmin = 60
        /\ globalMemory = [i \in 0..(INPUT_DATA_SIZE - 1) |-> 0]
        /\ workGroupSize = 0
        /\ nWorkGroups = 0
        /\ tileSize = 0
        /\ nWorkingDevices = 0
        /\ nWorkingUnitsPerDevice = 0
        /\ nWorkingPEsPerUnit = 0
        /\ allWorkingUnits = 0
        /\ nRunningUnits = 0
        /\ nWaitingUnits = 0
        /\ nWarpsPerUnit = 0
        /\ nWarps = 0
        /\ nInstructions = 7
        /\ aoutput = 100000
        /\ unitsFinal = 0
        /\ workgroups = <<>>
        /\ barrierIn = [d \in 0..(DEVICES-1) |->
                       [i \in 0..(UNITS_PER_DEVICE * INPUT_DATA_SIZE - 1) |-> 0]]
        /\ isWarpReadyToRun = [d \in 0..(DEVICES-1) |->
                              [i \in 0..(UNITS_PER_DEVICE * INPUT_DATA_SIZE - 1) |-> 0]]
        (* Process UnitProc *)
        /\ dId_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                           u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ uId_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                           u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ msg_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                           u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        /\ wgId_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ warpId_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ instrId_ = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                               u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ pesIdx = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ u_i = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                          u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ t = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                        u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ bd = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                         u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ bu = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                         u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ w = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                        u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ longWorkFlag = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                   u \in 0..(UNITS_PER_DEVICE-1) } |-> FALSE]
        /\ startTime = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ curTime = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ localMemory_u = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                    u \in 0..(UNITS_PER_DEVICE-1) } |-> [lm \in 0..(LOCAL_MEMORY_SIZE - 1) |-> 100000]]
        /\ localId_u = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1) } |-> [li \in 0..(INPUT_DATA_SIZE - 1) |-> 0]]
        /\ globalOffset_u = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                     u \in 0..(UNITS_PER_DEVICE-1) } |-> [go2 \in 0..(INPUT_DATA_SIZE - 1) |-> 0]]
        /\ tileIdx_u = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1) } |-> [ti \in 0..(INPUT_DATA_SIZE - 1) |-> 0]]
        /\ readyToMin_u = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                   u \in 0..(UNITS_PER_DEVICE-1) } |-> [rm \in 0..(INPUT_DATA_SIZE - 1) |-> 0]]
        /\ minVal = [self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        (* Process WarpSch *)
        /\ dId_W = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ uId_W = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ msg = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                          u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        /\ wgId_W = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ warpId = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ instrId = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ warps = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        /\ warpEntry = [self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        (* Process DeviceProc *)
        /\ dId = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ uId = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ msgFromHost = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> ""]
        /\ msgFromSch = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> <<>>]
        /\ wgId = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ readyToRun = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> TRUE]
        /\ wgEntry = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> <<>>]
        /\ wgWaiting = [self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } |-> TRUE]
        (* Process Host *)
        /\ msgFromDevice = ""
        /\ deviceIdx = 0
        /\ wgIdx = 0
        (* Process Main *)
        /\ i_1 = 0
        /\ j_1 = 0
        /\ pc = [self \in ProcSet |-> CASE self = <<0,0,0>> -> "c0"
                                        [] self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                                                   u \in 0..(UNITS_PER_DEVICE-1) } -> "unit_init"
                                        [] self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                                                   u \in 0..(UNITS_PER_DEVICE-1) } -> "ws_init"
                                        [] self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } -> "dev_init"
                                        [] self = <<5,0,0>> -> "h0"
                                        [] self = <<1,0,0>> -> "m_loop_gm"]

c0 == /\ pc[<<0,0,0>>] = "c0"
      /\ clockFlag = TRUE
      /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

loop_cl == /\ pc[<<0,0,0>>] = "loop_cl"
           /\ IF ~final
                 THEN /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "c1"]
                 ELSE /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "Done"]
           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                           hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                           sch_dev, sch_u, u_sch, globalTime, Tmin, 
                           globalMemory, workGroupSize, nWorkGroups, tileSize, 
                           nWorkingDevices, nWorkingUnitsPerDevice, 
                           nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, 
                           nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                           aoutput, unitsFinal, workgroups, barrierIn, 
                           isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                           instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                           startTime, curTime, localMemory_u, localId_u, 
                           globalOffset_u, tileIdx_u, readyToMin_u, minVal, 
                           dId_W, uId_W, msg, wgId_W, warpId, instrId, warps, 
                           warpEntry, dId, uId, msgFromHost, msgFromSch, wgId, 
                           readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                           deviceIdx, wgIdx, i_1, j_1 >>

c1 == /\ pc[<<0,0,0>>] = "c1"
      /\ IF  (nRunningUnits = (allWorkingUnits - nWaitingUnits))
            /\ (allWorkingUnits # 0)
            /\ (nWaitingUnits # allWorkingUnits)
            THEN /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "c2"]
            ELSE /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

c2 == /\ pc[<<0,0,0>>] = "c2"
      /\ nRunningUnits' = 0
      /\ globalTime' = globalTime + 1
      /\ pc' = [pc EXCEPT ![<<0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, Tmin, globalMemory, workGroupSize, 
                      nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nWaitingUnits, nWarpsPerUnit, nWarps, 
                      nInstructions, aoutput, unitsFinal, workgroups, 
                      barrierIn, isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                      warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                      longWorkFlag, startTime, curTime, localMemory_u, 
                      localId_u, globalOffset_u, tileIdx_u, readyToMin_u, 
                      minVal, dId_W, uId_W, msg, wgId_W, warpId, instrId, 
                      warps, warpEntry, dId, uId, msgFromHost, msgFromSch, 
                      wgId, readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                      deviceIdx, wgIdx, i_1, j_1 >>

Clock == c0 \/ loop_cl \/ c1 \/ c2

unit_init(self) == /\ pc[self] = "unit_init"
                   /\    self[2] < nWorkingDevices
                      /\ self[3] < nWorkingUnitsPerDevice
                   /\ dId_' = [dId_ EXCEPT ![self] = self[2]]
                   /\ uId_' = [uId_ EXCEPT ![self] = self[3]]
                   /\ pc' = [pc EXCEPT ![self] = "unit_outer"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                   STOPWARPS, hostFlag, clockFlag, final, 
                                   hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                   u_sch, globalTime, Tmin, globalMemory, 
                                   workGroupSize, nWorkGroups, tileSize, 
                                   nWorkingDevices, nWorkingUnitsPerDevice, 
                                   nWorkingPEsPerUnit, allWorkingUnits, 
                                   nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                   nWarps, nInstructions, aoutput, unitsFinal, 
                                   workgroups, barrierIn, isWarpReadyToRun, 
                                   msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                                   t, bd, bu, w, longWorkFlag, startTime, 
                                   curTime, localMemory_u, localId_u, 
                                   globalOffset_u, tileIdx_u, readyToMin_u, 
                                   minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                   instrId, warps, warpEntry, dId, uId, 
                                   msgFromHost, msgFromSch, wgId, readyToRun, 
                                   wgEntry, wgWaiting, msgFromDevice, 
                                   deviceIdx, wgIdx, i_1, j_1 >>

unit_outer(self) == /\ pc[self] = "unit_outer"
                    /\ sch_u[dId_[self]][uId_[self]] # <<>>
                    /\ msg_' = [msg_ EXCEPT ![self] = Head(sch_u[dId_[self]][uId_[self]])]
                    /\ sch_u' = [sch_u EXCEPT ![dId_[self]][uId_[self]] = Tail(sch_u[dId_[self]][uId_[self]])]
                    /\ pc' = [pc EXCEPT ![self] = "unit_outer_check"]
                    /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                    STOPWARPS, hostFlag, clockFlag, final, 
                                    hst_d, d_hst, dev_sch, sch_dev, u_sch, 
                                    globalTime, Tmin, globalMemory, 
                                    workGroupSize, nWorkGroups, tileSize, 
                                    nWorkingDevices, nWorkingUnitsPerDevice, 
                                    nWorkingPEsPerUnit, allWorkingUnits, 
                                    nRunningUnits, nWaitingUnits, 
                                    nWarpsPerUnit, nWarps, nInstructions, 
                                    aoutput, unitsFinal, workgroups, barrierIn, 
                                    isWarpReadyToRun, dId_, uId_, wgId_, 
                                    warpId_, instrId_, pesIdx, u_i, t, bd, bu, 
                                    w, longWorkFlag, startTime, curTime, 
                                    localMemory_u, localId_u, globalOffset_u, 
                                    tileIdx_u, readyToMin_u, minVal, dId_W, 
                                    uId_W, msg, wgId_W, warpId, instrId, warps, 
                                    warpEntry, dId, uId, msgFromHost, 
                                    msgFromSch, wgId, readyToRun, wgEntry, 
                                    wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                                    i_1, j_1 >>

unit_outer_check(self) == /\ pc[self] = "unit_outer_check"
                          /\ IF msg_[self][1] = GO
                                THEN /\ wgId_' = [wgId_ EXCEPT ![self] = msg_[self][2]]
                                     /\ pc' = [pc EXCEPT ![self] = "unit_init_local"]
                                     /\ UNCHANGED unitsFinal
                                ELSE /\ IF msg_[self][1] = STOP
                                           THEN /\ unitsFinal' = unitsFinal + 1
                                                /\ pc' = [pc EXCEPT ![self] = "unit_done"]
                                           ELSE /\ pc' = [pc EXCEPT ![self] = "unit_outer"]
                                                /\ UNCHANGED unitsFinal
                                     /\ wgId_' = wgId_
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, workgroups, barrierIn, 
                                          isWarpReadyToRun, dId_, uId_, msg_, 
                                          warpId_, instrId_, pesIdx, u_i, t, 
                                          bd, bu, w, longWorkFlag, startTime, 
                                          curTime, localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_init_local(self) == /\ pc[self] = "unit_init_local"
                         /\ u_i' = [u_i EXCEPT ![self] = 0]
                         /\ pc' = [pc EXCEPT ![self] = "unit_init_lm_loop"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_dev, sch_u, u_sch, 
                                         globalTime, Tmin, globalMemory, 
                                         workGroupSize, nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, t, bd, bu, w, longWorkFlag, 
                                         startTime, curTime, localMemory_u, 
                                         localId_u, globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

unit_init_lm_loop(self) == /\ pc[self] = "unit_init_lm_loop"
                           /\ IF u_i[self] >= LOCAL_MEMORY_SIZE
                                 THEN /\ u_i' = [u_i EXCEPT ![self] = 0]
                                      /\ pc' = [pc EXCEPT ![self] = "unit_init_reg_loop"]
                                 ELSE /\ pc' = [pc EXCEPT ![self] = "unit_init_lm_set"]
                                      /\ u_i' = u_i
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, t, 
                                           bd, bu, w, longWorkFlag, startTime, 
                                           curTime, localMemory_u, localId_u, 
                                           globalOffset_u, tileIdx_u, 
                                           readyToMin_u, minVal, dId_W, uId_W, 
                                           msg, wgId_W, warpId, instrId, warps, 
                                           warpEntry, dId, uId, msgFromHost, 
                                           msgFromSch, wgId, readyToRun, 
                                           wgEntry, wgWaiting, msgFromDevice, 
                                           deviceIdx, wgIdx, i_1, j_1 >>

unit_init_lm_set(self) == /\ pc[self] = "unit_init_lm_set"
                          /\ localMemory_u' = [localMemory_u EXCEPT ![self][u_i[self]] = 100000]
                          /\ u_i' = [u_i EXCEPT ![self] = u_i[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "unit_init_lm_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, t, bd, bu, w, longWorkFlag, 
                                          startTime, curTime, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_init_reg_loop(self) == /\ pc[self] = "unit_init_reg_loop"
                            /\ IF u_i[self] >= INPUT_DATA_SIZE
                                  THEN /\ pc' = [pc EXCEPT ![self] = "unit_warp_loop"]
                                  ELSE /\ pc' = [pc EXCEPT ![self] = "unit_init_reg_set"]
                            /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                            DONEWARP, STOPWARPS, hostFlag, 
                                            clockFlag, final, hst_d, d_hst, 
                                            dev_sch, sch_dev, sch_u, u_sch, 
                                            globalTime, Tmin, globalMemory, 
                                            workGroupSize, nWorkGroups, 
                                            tileSize, nWorkingDevices, 
                                            nWorkingUnitsPerDevice, 
                                            nWorkingPEsPerUnit, 
                                            allWorkingUnits, nRunningUnits, 
                                            nWaitingUnits, nWarpsPerUnit, 
                                            nWarps, nInstructions, aoutput, 
                                            unitsFinal, workgroups, barrierIn, 
                                            isWarpReadyToRun, dId_, uId_, msg_, 
                                            wgId_, warpId_, instrId_, pesIdx, 
                                            u_i, t, bd, bu, w, longWorkFlag, 
                                            startTime, curTime, localMemory_u, 
                                            localId_u, globalOffset_u, 
                                            tileIdx_u, readyToMin_u, minVal, 
                                            dId_W, uId_W, msg, wgId_W, warpId, 
                                            instrId, warps, warpEntry, dId, 
                                            uId, msgFromHost, msgFromSch, wgId, 
                                            readyToRun, wgEntry, wgWaiting, 
                                            msgFromDevice, deviceIdx, wgIdx, 
                                            i_1, j_1 >>

unit_init_reg_set(self) == /\ pc[self] = "unit_init_reg_set"
                           /\ localId_u' = [localId_u EXCEPT ![self][u_i[self]] = 0]
                           /\ globalOffset_u' = [globalOffset_u EXCEPT ![self][u_i[self]] = 0]
                           /\ tileIdx_u' = [tileIdx_u EXCEPT ![self][u_i[self]] = 0]
                           /\ readyToMin_u' = [readyToMin_u EXCEPT ![self][u_i[self]] = 0]
                           /\ u_i' = [u_i EXCEPT ![self] = u_i[self] + 1]
                           /\ pc' = [pc EXCEPT ![self] = "unit_init_reg_loop"]
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, t, 
                                           bd, bu, w, longWorkFlag, startTime, 
                                           curTime, localMemory_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, uId, 
                                           msgFromHost, msgFromSch, wgId, 
                                           readyToRun, wgEntry, wgWaiting, 
                                           msgFromDevice, deviceIdx, wgIdx, 
                                           i_1, j_1 >>

unit_warp_loop(self) == /\ pc[self] = "unit_warp_loop"
                        /\ sch_u[dId_[self]][uId_[self]] # <<>>
                        /\ msg_' = [msg_ EXCEPT ![self] = Head(sch_u[dId_[self]][uId_[self]])]
                        /\ sch_u' = [sch_u EXCEPT ![dId_[self]][uId_[self]] = Tail(sch_u[dId_[self]][uId_[self]])]
                        /\ pc' = [pc EXCEPT ![self] = "unit_warp_check"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, u_sch, 
                                        globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, wgId_, warpId_, instrId_, pesIdx, 
                                        u_i, t, bd, bu, w, longWorkFlag, 
                                        startTime, curTime, localMemory_u, 
                                        localId_u, globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, uId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

unit_warp_check(self) == /\ pc[self] = "unit_warp_check"
                         /\ IF msg_[self][1] = GOWARP
                               THEN /\ warpId_' = [warpId_ EXCEPT ![self] = msg_[self][2]]
                                    /\ instrId_' = [instrId_ EXCEPT ![self] = msg_[self][3]]
                                    /\ pc' = [pc EXCEPT ![self] = "unit_set_time"]
                               ELSE /\ IF msg_[self][1] = STOPWARPS
                                          THEN /\ pc' = [pc EXCEPT ![self] = "unit_outer"]
                                          ELSE /\ pc' = [pc EXCEPT ![self] = "unit_warp_loop"]
                                    /\ UNCHANGED << warpId_, instrId_ >>
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_dev, sch_u, u_sch, 
                                         globalTime, Tmin, globalMemory, 
                                         workGroupSize, nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, pesIdx, u_i, t, bd, 
                                         bu, w, longWorkFlag, startTime, 
                                         curTime, localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

unit_set_time(self) == /\ pc[self] = "unit_set_time"
                       /\ startTime' = [startTime EXCEPT ![self] = globalTime]
                       /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                       /\ pc' = [pc EXCEPT ![self] = "unit_dispatch"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       localMemory_u, localId_u, 
                                       globalOffset_u, tileIdx_u, readyToMin_u, 
                                       minVal, dId_W, uId_W, msg, wgId_W, 
                                       warpId, instrId, warps, warpEntry, dId, 
                                       uId, msgFromHost, msgFromSch, wgId, 
                                       readyToRun, wgEntry, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

unit_dispatch(self) == /\ pc[self] = "unit_dispatch"
                       /\ IF instrId_[self] = 0
                             THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr0"]
                             ELSE /\ IF instrId_[self] = 1
                                        THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr1"]
                                        ELSE /\ IF instrId_[self] = 2
                                                   THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr2"]
                                                   ELSE /\ IF instrId_[self] = 3
                                                              THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr3"]
                                                              ELSE /\ IF instrId_[self] = 4
                                                                         THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr4"]
                                                                         ELSE /\ IF instrId_[self] = 5
                                                                                    THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr5"]
                                                                                    ELSE /\ IF instrId_[self] = 6
                                                                                               THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr6"]
                                                                                               ELSE /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, instrId, warps, 
                                       warpEntry, dId, uId, msgFromHost, 
                                       msgFromSch, wgId, readyToRun, wgEntry, 
                                       wgWaiting, msgFromDevice, deviceIdx, 
                                       wgIdx, i_1, j_1 >>

unit_instr0(self) == /\ pc[self] = "unit_instr0"
                     /\ pesIdx' = [pesIdx EXCEPT ![self] = 0]
                     /\ pc' = [pc EXCEPT ![self] = "unit_instr0_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, u_i, t, 
                                     bd, bu, w, longWorkFlag, startTime, 
                                     curTime, localMemory_u, localId_u, 
                                     globalOffset_u, tileIdx_u, readyToMin_u, 
                                     minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                     instrId, warps, warpEntry, dId, uId, 
                                     msgFromHost, msgFromSch, wgId, readyToRun, 
                                     wgEntry, wgWaiting, msgFromDevice, 
                                     deviceIdx, wgIdx, i_1, j_1 >>

unit_instr0_loop(self) == /\ pc[self] = "unit_instr0_loop"
                          /\ IF pesIdx[self] >= nWorkingPEsPerUnit
                                THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr0_body"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, u_i, t, bd, bu, w, 
                                          longWorkFlag, startTime, curTime, 
                                          localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_instr0_body(self) == /\ pc[self] = "unit_instr0_body"
                          /\ IF workGroupSize > nWorkingPEsPerUnit
                                THEN /\ localId_u' = [localId_u EXCEPT ![self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = pesIdx[self] + warpId_[self] * nWorkingPEsPerUnit]
                                ELSE /\ localId_u' = [localId_u EXCEPT ![self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = pesIdx[self]]
                          /\ pesIdx' = [pesIdx EXCEPT ![self] = pesIdx[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "unit_instr0_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          u_i, t, bd, bu, w, longWorkFlag, 
                                          startTime, curTime, localMemory_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_instr1(self) == /\ pc[self] = "unit_instr1"
                     /\ pesIdx' = [pesIdx EXCEPT ![self] = 0]
                     /\ pc' = [pc EXCEPT ![self] = "unit_instr1_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, u_i, t, 
                                     bd, bu, w, longWorkFlag, startTime, 
                                     curTime, localMemory_u, localId_u, 
                                     globalOffset_u, tileIdx_u, readyToMin_u, 
                                     minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                     instrId, warps, warpEntry, dId, uId, 
                                     msgFromHost, msgFromSch, wgId, readyToRun, 
                                     wgEntry, wgWaiting, msgFromDevice, 
                                     deviceIdx, wgIdx, i_1, j_1 >>

unit_instr1_loop(self) == /\ pc[self] = "unit_instr1_loop"
                          /\ IF pesIdx[self] >= nWorkingPEsPerUnit
                                THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr1_body"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, u_i, t, bd, bu, w, 
                                          longWorkFlag, startTime, curTime, 
                                          localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_instr1_body(self) == /\ pc[self] = "unit_instr1_body"
                          /\ globalOffset_u' = [globalOffset_u EXCEPT ![self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = tileSize * (wgId_[self] * workGroupSize
                                                                                                                                  + localId_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]])]
                          /\ pesIdx' = [pesIdx EXCEPT ![self] = pesIdx[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "unit_instr1_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          u_i, t, bd, bu, w, longWorkFlag, 
                                          startTime, curTime, localMemory_u, 
                                          localId_u, tileIdx_u, readyToMin_u, 
                                          minVal, dId_W, uId_W, msg, wgId_W, 
                                          warpId, instrId, warps, warpEntry, 
                                          dId, uId, msgFromHost, msgFromSch, 
                                          wgId, readyToRun, wgEntry, wgWaiting, 
                                          msgFromDevice, deviceIdx, wgIdx, i_1, 
                                          j_1 >>

unit_instr2(self) == /\ pc[self] = "unit_instr2"
                     /\ pesIdx' = [pesIdx EXCEPT ![self] = 0]
                     /\ pc' = [pc EXCEPT ![self] = "unit_instr2_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, u_i, t, 
                                     bd, bu, w, longWorkFlag, startTime, 
                                     curTime, localMemory_u, localId_u, 
                                     globalOffset_u, tileIdx_u, readyToMin_u, 
                                     minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                     instrId, warps, warpEntry, dId, uId, 
                                     msgFromHost, msgFromSch, wgId, readyToRun, 
                                     wgEntry, wgWaiting, msgFromDevice, 
                                     deviceIdx, wgIdx, i_1, j_1 >>

unit_instr2_loop(self) == /\ pc[self] = "unit_instr2_loop"
                          /\ IF pesIdx[self] >= nWorkingPEsPerUnit
                                THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr2_body"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, u_i, t, bd, bu, w, 
                                          longWorkFlag, startTime, curTime, 
                                          localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_instr2_body(self) == /\ pc[self] = "unit_instr2_body"
                          /\ IF tileIdx_u[self][warpId_[self]]
                                + globalOffset_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]]
                                < INPUT_DATA_SIZE
                                THEN /\ readyToMin_u' = [readyToMin_u EXCEPT ![self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = 1]
                                ELSE /\ TRUE
                                     /\ UNCHANGED readyToMin_u
                          /\ pesIdx' = [pesIdx EXCEPT ![self] = pesIdx[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "unit_instr2_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          u_i, t, bd, bu, w, longWorkFlag, 
                                          startTime, curTime, localMemory_u, 
                                          localId_u, globalOffset_u, tileIdx_u, 
                                          minVal, dId_W, uId_W, msg, wgId_W, 
                                          warpId, instrId, warps, warpEntry, 
                                          dId, uId, msgFromHost, msgFromSch, 
                                          wgId, readyToRun, wgEntry, wgWaiting, 
                                          msgFromDevice, deviceIdx, wgIdx, i_1, 
                                          j_1 >>

unit_instr3(self) == /\ pc[self] = "unit_instr3"
                     /\ pesIdx' = [pesIdx EXCEPT ![self] = 0]
                     /\ longWorkFlag' = [longWorkFlag EXCEPT ![self] = FALSE]
                     /\ pc' = [pc EXCEPT ![self] = "unit_instr3_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, u_i, t, 
                                     bd, bu, w, startTime, curTime, 
                                     localMemory_u, localId_u, globalOffset_u, 
                                     tileIdx_u, readyToMin_u, minVal, dId_W, 
                                     uId_W, msg, wgId_W, warpId, instrId, 
                                     warps, warpEntry, dId, uId, msgFromHost, 
                                     msgFromSch, wgId, readyToRun, wgEntry, 
                                     wgWaiting, msgFromDevice, deviceIdx, 
                                     wgIdx, i_1, j_1 >>

unit_instr3_loop(self) == /\ pc[self] = "unit_instr3_loop"
                          /\ IF pesIdx[self] >= nWorkingPEsPerUnit
                                THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr3_after"]
                                ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr3_body"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, u_i, t, bd, bu, w, 
                                          longWorkFlag, startTime, curTime, 
                                          localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

unit_instr3_body(self) == /\ pc[self] = "unit_instr3_body"
                          /\ IF readyToMin_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = 1
                                THEN /\ minVal' = [minVal EXCEPT ![self] =       globalMemory[
                                                                           tileIdx_u[self][warpId_[self]]
                                                                           + globalOffset_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]]]]
                                     /\ IF localMemory_u[self][localId_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]]] > minVal'[self]
                                           THEN /\ localMemory_u' = [localMemory_u EXCEPT ![self][localId_u[self][pesIdx[self] * nWarpsPerUnit + warpId_[self]]] = minVal'[self]]
                                           ELSE /\ TRUE
                                                /\ UNCHANGED localMemory_u
                                     /\ longWorkFlag' = [longWorkFlag EXCEPT ![self] = TRUE]
                                     /\ readyToMin_u' = [readyToMin_u EXCEPT ![self][pesIdx[self] * nWarpsPerUnit + warpId_[self]] = 0]
                                ELSE /\ TRUE
                                     /\ UNCHANGED << longWorkFlag, 
                                                     localMemory_u, 
                                                     readyToMin_u, minVal >>
                          /\ pesIdx' = [pesIdx EXCEPT ![self] = pesIdx[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "unit_instr3_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          u_i, t, bd, bu, w, startTime, 
                                          curTime, localId_u, globalOffset_u, 
                                          tileIdx_u, dId_W, uId_W, msg, wgId_W, 
                                          warpId, instrId, warps, warpEntry, 
                                          dId, uId, msgFromHost, msgFromSch, 
                                          wgId, readyToRun, wgEntry, wgWaiting, 
                                          msgFromDevice, deviceIdx, wgIdx, i_1, 
                                          j_1 >>

unit_instr3_after(self) == /\ pc[self] = "unit_instr3_after"
                           /\ IF longWorkFlag[self]
                                 THEN /\ pc' = [pc EXCEPT ![self] = "unit_long_work"]
                                 ELSE /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, 
                                           u_i, t, bd, bu, w, longWorkFlag, 
                                           startTime, curTime, localMemory_u, 
                                           localId_u, globalOffset_u, 
                                           tileIdx_u, readyToMin_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, uId, 
                                           msgFromHost, msgFromSch, wgId, 
                                           readyToRun, wgEntry, wgWaiting, 
                                           msgFromDevice, deviceIdx, wgIdx, 
                                           i_1, j_1 >>

unit_long_work(self) == /\ pc[self] = "unit_long_work"
                        /\ IF globalTime >= startTime[self] + GLOBAL_MEMORY_ACCESS
                              THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                              ELSE /\ pc' = [pc EXCEPT ![self] = "unit_work_step"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, msg_, wgId_, warpId_, instrId_, 
                                        pesIdx, u_i, t, bd, bu, w, 
                                        longWorkFlag, startTime, curTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, uId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

unit_work_step(self) == /\ pc[self] = "unit_work_step"
                        /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                        /\ nRunningUnits' = nRunningUnits + 1
                        /\ pc' = [pc EXCEPT ![self] = "unit_work_await"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nWaitingUnits, nWarpsPerUnit, nWarps, 
                                        nInstructions, aoutput, unitsFinal, 
                                        workgroups, barrierIn, 
                                        isWarpReadyToRun, dId_, uId_, msg_, 
                                        wgId_, warpId_, instrId_, pesIdx, u_i, 
                                        t, bd, bu, w, longWorkFlag, startTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, uId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

unit_work_await(self) == /\ pc[self] = "unit_work_await"
                         /\ globalTime >= curTime[self] + 1
                         /\ pc' = [pc EXCEPT ![self] = "unit_long_work"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_dev, sch_u, u_sch, 
                                         globalTime, Tmin, globalMemory, 
                                         workGroupSize, nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, u_i, t, bd, bu, w, 
                                         longWorkFlag, startTime, curTime, 
                                         localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

unit_instr4(self) == /\ pc[self] = "unit_instr4"
                     /\ tileIdx_u' = [tileIdx_u EXCEPT ![self][warpId_[self]] = tileIdx_u[self][warpId_[self]] + 1]
                     /\ pc' = [pc EXCEPT ![self] = "unit_instr4_check"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, pesIdx, 
                                     u_i, t, bd, bu, w, longWorkFlag, 
                                     startTime, curTime, localMemory_u, 
                                     localId_u, globalOffset_u, readyToMin_u, 
                                     minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                     instrId, warps, warpEntry, dId, uId, 
                                     msgFromHost, msgFromSch, wgId, readyToRun, 
                                     wgEntry, wgWaiting, msgFromDevice, 
                                     deviceIdx, wgIdx, i_1, j_1 >>

unit_instr4_check(self) == /\ pc[self] = "unit_instr4_check"
                           /\ IF tileIdx_u[self][warpId_[self]] < tileSize
                                 THEN /\ instrId_' = [instrId_ EXCEPT ![self] = instrId_[self] - 3]
                                      /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                 ELSE /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                      /\ UNCHANGED instrId_
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, pesIdx, u_i, t, bd, 
                                           bu, w, longWorkFlag, startTime, 
                                           curTime, localMemory_u, localId_u, 
                                           globalOffset_u, tileIdx_u, 
                                           readyToMin_u, minVal, dId_W, uId_W, 
                                           msg, wgId_W, warpId, instrId, warps, 
                                           warpEntry, dId, uId, msgFromHost, 
                                           msgFromSch, wgId, readyToRun, 
                                           wgEntry, wgWaiting, msgFromDevice, 
                                           deviceIdx, wgIdx, i_1, j_1 >>

unit_instr5(self) == /\ pc[self] = "unit_instr5"
                     /\ barrierIn' = [barrierIn EXCEPT ![dId_[self]][uId_[self] * nWarpsPerUnit + warpId_[self]] = 1]
                     /\ t' = [t EXCEPT ![self] = 0]
                     /\ bd' = [bd EXCEPT ![self] = 0]
                     /\ pc' = [pc EXCEPT ![self] = "unit_barrier_d_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                                     warpId_, instrId_, pesIdx, u_i, bu, w, 
                                     longWorkFlag, startTime, curTime, 
                                     localMemory_u, localId_u, globalOffset_u, 
                                     tileIdx_u, readyToMin_u, minVal, dId_W, 
                                     uId_W, msg, wgId_W, warpId, instrId, 
                                     warps, warpEntry, dId, uId, msgFromHost, 
                                     msgFromSch, wgId, readyToRun, wgEntry, 
                                     wgWaiting, msgFromDevice, deviceIdx, 
                                     wgIdx, i_1, j_1 >>

unit_barrier_d_loop(self) == /\ pc[self] = "unit_barrier_d_loop"
                             /\ IF bd[self] >= nWorkingDevices
                                   THEN /\ pc' = [pc EXCEPT ![self] = "unit_barrier_check"]
                                   ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_d_body"]
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bd, bu, w, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_d_body(self) == /\ pc[self] = "unit_barrier_d_body"
                             /\ bu' = [bu EXCEPT ![self] = 0]
                             /\ pc' = [pc EXCEPT ![self] = "unit_barrier_u_loop"]
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bd, w, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_u_loop(self) == /\ pc[self] = "unit_barrier_u_loop"
                             /\ IF bu[self] >= nWorkingUnitsPerDevice
                                   THEN /\ bd' = [bd EXCEPT ![self] = bd[self] + 1]
                                        /\ pc' = [pc EXCEPT ![self] = "unit_barrier_d_loop"]
                                   ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_u_body"]
                                        /\ bd' = bd
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bu, w, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_u_body(self) == /\ pc[self] = "unit_barrier_u_body"
                             /\ w' = [w EXCEPT ![self] = 0]
                             /\ pc' = [pc EXCEPT ![self] = "unit_barrier_w_loop"]
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bd, bu, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_w_loop(self) == /\ pc[self] = "unit_barrier_w_loop"
                             /\ IF w[self] >= nWarpsPerUnit
                                   THEN /\ bu' = [bu EXCEPT ![self] = bu[self] + 1]
                                        /\ pc' = [pc EXCEPT ![self] = "unit_barrier_u_loop"]
                                   ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_w_body"]
                                        /\ bu' = bu
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bd, w, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_w_body(self) == /\ pc[self] = "unit_barrier_w_body"
                             /\ t' = [t EXCEPT ![self] = t[self] + barrierIn[bd[self]][bu[self] * nWarpsPerUnit + w[self]]]
                             /\ nWaitingUnits' = t'[self]
                             /\ w' = [w EXCEPT ![self] = w[self] + 1]
                             /\ pc' = [pc EXCEPT ![self] = "unit_barrier_w_loop"]
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, u_sch, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWarpsPerUnit, nWarps, 
                                             nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, bd, bu, longWorkFlag, 
                                             startTime, curTime, localMemory_u, 
                                             localId_u, globalOffset_u, 
                                             tileIdx_u, readyToMin_u, minVal, 
                                             dId_W, uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_check(self) == /\ pc[self] = "unit_barrier_check"
                            /\ IF t[self] = nWarps
                                  THEN /\ bd' = [bd EXCEPT ![self] = 0]
                                       /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_d"]
                                  ELSE /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                       /\ bd' = bd
                            /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                            DONEWARP, STOPWARPS, hostFlag, 
                                            clockFlag, final, hst_d, d_hst, 
                                            dev_sch, sch_dev, sch_u, u_sch, 
                                            globalTime, Tmin, globalMemory, 
                                            workGroupSize, nWorkGroups, 
                                            tileSize, nWorkingDevices, 
                                            nWorkingUnitsPerDevice, 
                                            nWorkingPEsPerUnit, 
                                            allWorkingUnits, nRunningUnits, 
                                            nWaitingUnits, nWarpsPerUnit, 
                                            nWarps, nInstructions, aoutput, 
                                            unitsFinal, workgroups, barrierIn, 
                                            isWarpReadyToRun, dId_, uId_, msg_, 
                                            wgId_, warpId_, instrId_, pesIdx, 
                                            u_i, t, bu, w, longWorkFlag, 
                                            startTime, curTime, localMemory_u, 
                                            localId_u, globalOffset_u, 
                                            tileIdx_u, readyToMin_u, minVal, 
                                            dId_W, uId_W, msg, wgId_W, warpId, 
                                            instrId, warps, warpEntry, dId, 
                                            uId, msgFromHost, msgFromSch, wgId, 
                                            readyToRun, wgEntry, wgWaiting, 
                                            msgFromDevice, deviceIdx, wgIdx, 
                                            i_1, j_1 >>

unit_barrier_reset_d(self) == /\ pc[self] = "unit_barrier_reset_d"
                              /\ IF bd[self] >= nWorkingDevices
                                    THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                    ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_d_body"]
                              /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                              DONEWARP, STOPWARPS, hostFlag, 
                                              clockFlag, final, hst_d, d_hst, 
                                              dev_sch, sch_dev, sch_u, u_sch, 
                                              globalTime, Tmin, globalMemory, 
                                              workGroupSize, nWorkGroups, 
                                              tileSize, nWorkingDevices, 
                                              nWorkingUnitsPerDevice, 
                                              nWorkingPEsPerUnit, 
                                              allWorkingUnits, nRunningUnits, 
                                              nWaitingUnits, nWarpsPerUnit, 
                                              nWarps, nInstructions, aoutput, 
                                              unitsFinal, workgroups, 
                                              barrierIn, isWarpReadyToRun, 
                                              dId_, uId_, msg_, wgId_, warpId_, 
                                              instrId_, pesIdx, u_i, t, bd, bu, 
                                              w, longWorkFlag, startTime, 
                                              curTime, localMemory_u, 
                                              localId_u, globalOffset_u, 
                                              tileIdx_u, readyToMin_u, minVal, 
                                              dId_W, uId_W, msg, wgId_W, 
                                              warpId, instrId, warps, 
                                              warpEntry, dId, uId, msgFromHost, 
                                              msgFromSch, wgId, readyToRun, 
                                              wgEntry, wgWaiting, 
                                              msgFromDevice, deviceIdx, wgIdx, 
                                              i_1, j_1 >>

unit_barrier_reset_d_body(self) == /\ pc[self] = "unit_barrier_reset_d_body"
                                   /\ bu' = [bu EXCEPT ![self] = 0]
                                   /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_u"]
                                   /\ UNCHANGED << GO, STOP, DONE, GOWG, 
                                                   GOWARP, DONEWARP, STOPWARPS, 
                                                   hostFlag, clockFlag, final, 
                                                   hst_d, d_hst, dev_sch, 
                                                   sch_dev, sch_u, u_sch, 
                                                   globalTime, Tmin, 
                                                   globalMemory, workGroupSize, 
                                                   nWorkGroups, tileSize, 
                                                   nWorkingDevices, 
                                                   nWorkingUnitsPerDevice, 
                                                   nWorkingPEsPerUnit, 
                                                   allWorkingUnits, 
                                                   nRunningUnits, 
                                                   nWaitingUnits, 
                                                   nWarpsPerUnit, nWarps, 
                                                   nInstructions, aoutput, 
                                                   unitsFinal, workgroups, 
                                                   barrierIn, isWarpReadyToRun, 
                                                   dId_, uId_, msg_, wgId_, 
                                                   warpId_, instrId_, pesIdx, 
                                                   u_i, t, bd, w, longWorkFlag, 
                                                   startTime, curTime, 
                                                   localMemory_u, localId_u, 
                                                   globalOffset_u, tileIdx_u, 
                                                   readyToMin_u, minVal, dId_W, 
                                                   uId_W, msg, wgId_W, warpId, 
                                                   instrId, warps, warpEntry, 
                                                   dId, uId, msgFromHost, 
                                                   msgFromSch, wgId, 
                                                   readyToRun, wgEntry, 
                                                   wgWaiting, msgFromDevice, 
                                                   deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_reset_u(self) == /\ pc[self] = "unit_barrier_reset_u"
                              /\ IF bu[self] >= nWorkingUnitsPerDevice
                                    THEN /\ bd' = [bd EXCEPT ![self] = bd[self] + 1]
                                         /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_d"]
                                    ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_u_body"]
                                         /\ bd' = bd
                              /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                              DONEWARP, STOPWARPS, hostFlag, 
                                              clockFlag, final, hst_d, d_hst, 
                                              dev_sch, sch_dev, sch_u, u_sch, 
                                              globalTime, Tmin, globalMemory, 
                                              workGroupSize, nWorkGroups, 
                                              tileSize, nWorkingDevices, 
                                              nWorkingUnitsPerDevice, 
                                              nWorkingPEsPerUnit, 
                                              allWorkingUnits, nRunningUnits, 
                                              nWaitingUnits, nWarpsPerUnit, 
                                              nWarps, nInstructions, aoutput, 
                                              unitsFinal, workgroups, 
                                              barrierIn, isWarpReadyToRun, 
                                              dId_, uId_, msg_, wgId_, warpId_, 
                                              instrId_, pesIdx, u_i, t, bu, w, 
                                              longWorkFlag, startTime, curTime, 
                                              localMemory_u, localId_u, 
                                              globalOffset_u, tileIdx_u, 
                                              readyToMin_u, minVal, dId_W, 
                                              uId_W, msg, wgId_W, warpId, 
                                              instrId, warps, warpEntry, dId, 
                                              uId, msgFromHost, msgFromSch, 
                                              wgId, readyToRun, wgEntry, 
                                              wgWaiting, msgFromDevice, 
                                              deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_reset_u_body(self) == /\ pc[self] = "unit_barrier_reset_u_body"
                                   /\ w' = [w EXCEPT ![self] = 0]
                                   /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_w"]
                                   /\ UNCHANGED << GO, STOP, DONE, GOWG, 
                                                   GOWARP, DONEWARP, STOPWARPS, 
                                                   hostFlag, clockFlag, final, 
                                                   hst_d, d_hst, dev_sch, 
                                                   sch_dev, sch_u, u_sch, 
                                                   globalTime, Tmin, 
                                                   globalMemory, workGroupSize, 
                                                   nWorkGroups, tileSize, 
                                                   nWorkingDevices, 
                                                   nWorkingUnitsPerDevice, 
                                                   nWorkingPEsPerUnit, 
                                                   allWorkingUnits, 
                                                   nRunningUnits, 
                                                   nWaitingUnits, 
                                                   nWarpsPerUnit, nWarps, 
                                                   nInstructions, aoutput, 
                                                   unitsFinal, workgroups, 
                                                   barrierIn, isWarpReadyToRun, 
                                                   dId_, uId_, msg_, wgId_, 
                                                   warpId_, instrId_, pesIdx, 
                                                   u_i, t, bd, bu, 
                                                   longWorkFlag, startTime, 
                                                   curTime, localMemory_u, 
                                                   localId_u, globalOffset_u, 
                                                   tileIdx_u, readyToMin_u, 
                                                   minVal, dId_W, uId_W, msg, 
                                                   wgId_W, warpId, instrId, 
                                                   warps, warpEntry, dId, uId, 
                                                   msgFromHost, msgFromSch, 
                                                   wgId, readyToRun, wgEntry, 
                                                   wgWaiting, msgFromDevice, 
                                                   deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_reset_w(self) == /\ pc[self] = "unit_barrier_reset_w"
                              /\ IF w[self] >= nWarpsPerUnit
                                    THEN /\ bu' = [bu EXCEPT ![self] = bu[self] + 1]
                                         /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_u"]
                                    ELSE /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_body"]
                                         /\ bu' = bu
                              /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                              DONEWARP, STOPWARPS, hostFlag, 
                                              clockFlag, final, hst_d, d_hst, 
                                              dev_sch, sch_dev, sch_u, u_sch, 
                                              globalTime, Tmin, globalMemory, 
                                              workGroupSize, nWorkGroups, 
                                              tileSize, nWorkingDevices, 
                                              nWorkingUnitsPerDevice, 
                                              nWorkingPEsPerUnit, 
                                              allWorkingUnits, nRunningUnits, 
                                              nWaitingUnits, nWarpsPerUnit, 
                                              nWarps, nInstructions, aoutput, 
                                              unitsFinal, workgroups, 
                                              barrierIn, isWarpReadyToRun, 
                                              dId_, uId_, msg_, wgId_, warpId_, 
                                              instrId_, pesIdx, u_i, t, bd, w, 
                                              longWorkFlag, startTime, curTime, 
                                              localMemory_u, localId_u, 
                                              globalOffset_u, tileIdx_u, 
                                              readyToMin_u, minVal, dId_W, 
                                              uId_W, msg, wgId_W, warpId, 
                                              instrId, warps, warpEntry, dId, 
                                              uId, msgFromHost, msgFromSch, 
                                              wgId, readyToRun, wgEntry, 
                                              wgWaiting, msgFromDevice, 
                                              deviceIdx, wgIdx, i_1, j_1 >>

unit_barrier_reset_body(self) == /\ pc[self] = "unit_barrier_reset_body"
                                 /\ barrierIn' = [barrierIn EXCEPT ![bd[self]][bu[self] * nWarpsPerUnit + w[self]] = 0]
                                 /\ nWaitingUnits' = 0
                                 /\ w' = [w EXCEPT ![self] = w[self] + 1]
                                 /\ pc' = [pc EXCEPT ![self] = "unit_barrier_reset_w"]
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, dev_sch, sch_dev, 
                                                 sch_u, u_sch, globalTime, 
                                                 Tmin, globalMemory, 
                                                 workGroupSize, nWorkGroups, 
                                                 tileSize, nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWarpsPerUnit, 
                                                 nWarps, nInstructions, 
                                                 aoutput, unitsFinal, 
                                                 workgroups, isWarpReadyToRun, 
                                                 dId_, uId_, msg_, wgId_, 
                                                 warpId_, instrId_, pesIdx, 
                                                 u_i, t, bd, bu, longWorkFlag, 
                                                 startTime, curTime, 
                                                 localMemory_u, localId_u, 
                                                 globalOffset_u, tileIdx_u, 
                                                 readyToMin_u, minVal, dId_W, 
                                                 uId_W, msg, wgId_W, warpId, 
                                                 instrId, warps, warpEntry, 
                                                 dId, uId, msgFromHost, 
                                                 msgFromSch, wgId, readyToRun, 
                                                 wgEntry, wgWaiting, 
                                                 msgFromDevice, deviceIdx, 
                                                 wgIdx, i_1, j_1 >>

unit_instr6(self) == /\ pc[self] = "unit_instr6"
                     /\ IF localId_u[self][0 * nWarpsPerUnit + warpId_[self]] = 0
                           THEN /\ u_i' = [u_i EXCEPT ![self] = 0]
                                /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_loop"]
                           ELSE /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                /\ u_i' = u_i
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, pesIdx, t, 
                                     bd, bu, w, longWorkFlag, startTime, 
                                     curTime, localMemory_u, localId_u, 
                                     globalOffset_u, tileIdx_u, readyToMin_u, 
                                     minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                     instrId, warps, warpEntry, dId, uId, 
                                     msgFromHost, msgFromSch, wgId, readyToRun, 
                                     wgEntry, wgWaiting, msgFromDevice, 
                                     deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_reduce_loop(self) == /\ pc[self] = "unit_instr6_reduce_loop"
                                 /\ IF u_i[self] >= nWarpsPerUnit * nWorkingPEsPerUnit
                                       THEN /\ pc' = [pc EXCEPT ![self] = "unit_instr6_global_min"]
                                       ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_body"]
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, dev_sch, sch_dev, 
                                                 sch_u, u_sch, globalTime, 
                                                 Tmin, globalMemory, 
                                                 workGroupSize, nWorkGroups, 
                                                 tileSize, nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWaitingUnits, 
                                                 nWarpsPerUnit, nWarps, 
                                                 nInstructions, aoutput, 
                                                 unitsFinal, workgroups, 
                                                 barrierIn, isWarpReadyToRun, 
                                                 dId_, uId_, msg_, wgId_, 
                                                 warpId_, instrId_, pesIdx, 
                                                 u_i, t, bd, bu, w, 
                                                 longWorkFlag, startTime, 
                                                 curTime, localMemory_u, 
                                                 localId_u, globalOffset_u, 
                                                 tileIdx_u, readyToMin_u, 
                                                 minVal, dId_W, uId_W, msg, 
                                                 wgId_W, warpId, instrId, 
                                                 warps, warpEntry, dId, uId, 
                                                 msgFromHost, msgFromSch, wgId, 
                                                 readyToRun, wgEntry, 
                                                 wgWaiting, msgFromDevice, 
                                                 deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_reduce_body(self) == /\ pc[self] = "unit_instr6_reduce_body"
                                 /\ IF localMemory_u[self][localId_u[self][0]] > localMemory_u[self][u_i[self]]
                                       THEN /\ localMemory_u' = [localMemory_u EXCEPT ![self][localId_u[self][0]] = localMemory_u[self][u_i[self]]]
                                       ELSE /\ TRUE
                                            /\ UNCHANGED localMemory_u
                                 /\ startTime' = [startTime EXCEPT ![self] = curTime[self]]
                                 /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_lw"]
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, dev_sch, sch_dev, 
                                                 sch_u, u_sch, globalTime, 
                                                 Tmin, globalMemory, 
                                                 workGroupSize, nWorkGroups, 
                                                 tileSize, nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWaitingUnits, 
                                                 nWarpsPerUnit, nWarps, 
                                                 nInstructions, aoutput, 
                                                 unitsFinal, workgroups, 
                                                 barrierIn, isWarpReadyToRun, 
                                                 dId_, uId_, msg_, wgId_, 
                                                 warpId_, instrId_, pesIdx, 
                                                 u_i, t, bd, bu, w, 
                                                 longWorkFlag, curTime, 
                                                 localId_u, globalOffset_u, 
                                                 tileIdx_u, readyToMin_u, 
                                                 minVal, dId_W, uId_W, msg, 
                                                 wgId_W, warpId, instrId, 
                                                 warps, warpEntry, dId, uId, 
                                                 msgFromHost, msgFromSch, wgId, 
                                                 readyToRun, wgEntry, 
                                                 wgWaiting, msgFromDevice, 
                                                 deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_reduce_lw(self) == /\ pc[self] = "unit_instr6_reduce_lw"
                               /\ IF globalTime >= startTime[self] + LOCAL_MEMORY_ACCESS
                                     THEN /\ u_i' = [u_i EXCEPT ![self] = u_i[self] + 1]
                                          /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_loop"]
                                     ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_ws"]
                                          /\ u_i' = u_i
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nRunningUnits, 
                                               nWaitingUnits, nWarpsPerUnit, 
                                               nWarps, nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, t, 
                                               bd, bu, w, longWorkFlag, 
                                               startTime, curTime, 
                                               localMemory_u, localId_u, 
                                               globalOffset_u, tileIdx_u, 
                                               readyToMin_u, minVal, dId_W, 
                                               uId_W, msg, wgId_W, warpId, 
                                               instrId, warps, warpEntry, dId, 
                                               uId, msgFromHost, msgFromSch, 
                                               wgId, readyToRun, wgEntry, 
                                               wgWaiting, msgFromDevice, 
                                               deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_reduce_ws(self) == /\ pc[self] = "unit_instr6_reduce_ws"
                               /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                               /\ nRunningUnits' = nRunningUnits + 1
                               /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_wa"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nWaitingUnits, 
                                               nWarpsPerUnit, nWarps, 
                                               nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, u_i, 
                                               t, bd, bu, w, longWorkFlag, 
                                               startTime, localMemory_u, 
                                               localId_u, globalOffset_u, 
                                               tileIdx_u, readyToMin_u, minVal, 
                                               dId_W, uId_W, msg, wgId_W, 
                                               warpId, instrId, warps, 
                                               warpEntry, dId, uId, 
                                               msgFromHost, msgFromSch, wgId, 
                                               readyToRun, wgEntry, wgWaiting, 
                                               msgFromDevice, deviceIdx, wgIdx, 
                                               i_1, j_1 >>

unit_instr6_reduce_wa(self) == /\ pc[self] = "unit_instr6_reduce_wa"
                               /\ globalTime >= curTime[self] + 1
                               /\ pc' = [pc EXCEPT ![self] = "unit_instr6_reduce_lw"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nRunningUnits, 
                                               nWaitingUnits, nWarpsPerUnit, 
                                               nWarps, nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, u_i, 
                                               t, bd, bu, w, longWorkFlag, 
                                               startTime, curTime, 
                                               localMemory_u, localId_u, 
                                               globalOffset_u, tileIdx_u, 
                                               readyToMin_u, minVal, dId_W, 
                                               uId_W, msg, wgId_W, warpId, 
                                               instrId, warps, warpEntry, dId, 
                                               uId, msgFromHost, msgFromSch, 
                                               wgId, readyToRun, wgEntry, 
                                               wgWaiting, msgFromDevice, 
                                               deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_global_min(self) == /\ pc[self] = "unit_instr6_global_min"
                                /\ IF aoutput > localMemory_u[self][localId_u[self][0]]
                                      THEN /\ aoutput' = localMemory_u[self][localId_u[self][0]]
                                      ELSE /\ TRUE
                                           /\ UNCHANGED aoutput
                                /\ startTime' = [startTime EXCEPT ![self] = curTime[self]]
                                /\ pc' = [pc EXCEPT ![self] = "unit_instr6_global_lw"]
                                /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                DONEWARP, STOPWARPS, hostFlag, 
                                                clockFlag, final, hst_d, d_hst, 
                                                dev_sch, sch_dev, sch_u, u_sch, 
                                                globalTime, Tmin, globalMemory, 
                                                workGroupSize, nWorkGroups, 
                                                tileSize, nWorkingDevices, 
                                                nWorkingUnitsPerDevice, 
                                                nWorkingPEsPerUnit, 
                                                allWorkingUnits, nRunningUnits, 
                                                nWaitingUnits, nWarpsPerUnit, 
                                                nWarps, nInstructions, 
                                                unitsFinal, workgroups, 
                                                barrierIn, isWarpReadyToRun, 
                                                dId_, uId_, msg_, wgId_, 
                                                warpId_, instrId_, pesIdx, u_i, 
                                                t, bd, bu, w, longWorkFlag, 
                                                curTime, localMemory_u, 
                                                localId_u, globalOffset_u, 
                                                tileIdx_u, readyToMin_u, 
                                                minVal, dId_W, uId_W, msg, 
                                                wgId_W, warpId, instrId, warps, 
                                                warpEntry, dId, uId, 
                                                msgFromHost, msgFromSch, wgId, 
                                                readyToRun, wgEntry, wgWaiting, 
                                                msgFromDevice, deviceIdx, 
                                                wgIdx, i_1, j_1 >>

unit_instr6_global_lw(self) == /\ pc[self] = "unit_instr6_global_lw"
                               /\ IF globalTime >= startTime[self] + GLOBAL_MEMORY_ACCESS
                                     THEN /\ pc' = [pc EXCEPT ![self] = "unit_send_done_warp"]
                                     ELSE /\ pc' = [pc EXCEPT ![self] = "unit_instr6_global_ws"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nRunningUnits, 
                                               nWaitingUnits, nWarpsPerUnit, 
                                               nWarps, nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, u_i, 
                                               t, bd, bu, w, longWorkFlag, 
                                               startTime, curTime, 
                                               localMemory_u, localId_u, 
                                               globalOffset_u, tileIdx_u, 
                                               readyToMin_u, minVal, dId_W, 
                                               uId_W, msg, wgId_W, warpId, 
                                               instrId, warps, warpEntry, dId, 
                                               uId, msgFromHost, msgFromSch, 
                                               wgId, readyToRun, wgEntry, 
                                               wgWaiting, msgFromDevice, 
                                               deviceIdx, wgIdx, i_1, j_1 >>

unit_instr6_global_ws(self) == /\ pc[self] = "unit_instr6_global_ws"
                               /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                               /\ nRunningUnits' = nRunningUnits + 1
                               /\ pc' = [pc EXCEPT ![self] = "unit_instr6_global_wa"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nWaitingUnits, 
                                               nWarpsPerUnit, nWarps, 
                                               nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, u_i, 
                                               t, bd, bu, w, longWorkFlag, 
                                               startTime, localMemory_u, 
                                               localId_u, globalOffset_u, 
                                               tileIdx_u, readyToMin_u, minVal, 
                                               dId_W, uId_W, msg, wgId_W, 
                                               warpId, instrId, warps, 
                                               warpEntry, dId, uId, 
                                               msgFromHost, msgFromSch, wgId, 
                                               readyToRun, wgEntry, wgWaiting, 
                                               msgFromDevice, deviceIdx, wgIdx, 
                                               i_1, j_1 >>

unit_instr6_global_wa(self) == /\ pc[self] = "unit_instr6_global_wa"
                               /\ globalTime >= curTime[self] + 1
                               /\ pc' = [pc EXCEPT ![self] = "unit_instr6_global_lw"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                               DONEWARP, STOPWARPS, hostFlag, 
                                               clockFlag, final, hst_d, d_hst, 
                                               dev_sch, sch_dev, sch_u, u_sch, 
                                               globalTime, Tmin, globalMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingUnits, nRunningUnits, 
                                               nWaitingUnits, nWarpsPerUnit, 
                                               nWarps, nInstructions, aoutput, 
                                               unitsFinal, workgroups, 
                                               barrierIn, isWarpReadyToRun, 
                                               dId_, uId_, msg_, wgId_, 
                                               warpId_, instrId_, pesIdx, u_i, 
                                               t, bd, bu, w, longWorkFlag, 
                                               startTime, curTime, 
                                               localMemory_u, localId_u, 
                                               globalOffset_u, tileIdx_u, 
                                               readyToMin_u, minVal, dId_W, 
                                               uId_W, msg, wgId_W, warpId, 
                                               instrId, warps, warpEntry, dId, 
                                               uId, msgFromHost, msgFromSch, 
                                               wgId, readyToRun, wgEntry, 
                                               wgWaiting, msgFromDevice, 
                                               deviceIdx, wgIdx, i_1, j_1 >>

unit_send_done_warp(self) == /\ pc[self] = "unit_send_done_warp"
                             /\ u_sch' = [u_sch EXCEPT ![dId_[self]][uId_[self]] = Append(u_sch[dId_[self]][uId_[self]], (<<DONEWARP, instrId_[self]>>))]
                             /\ pc' = [pc EXCEPT ![self] = "unit_warp_loop"]
                             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                             DONEWARP, STOPWARPS, hostFlag, 
                                             clockFlag, final, hst_d, d_hst, 
                                             dev_sch, sch_dev, sch_u, 
                                             globalTime, Tmin, globalMemory, 
                                             workGroupSize, nWorkGroups, 
                                             tileSize, nWorkingDevices, 
                                             nWorkingUnitsPerDevice, 
                                             nWorkingPEsPerUnit, 
                                             allWorkingUnits, nRunningUnits, 
                                             nWaitingUnits, nWarpsPerUnit, 
                                             nWarps, nInstructions, aoutput, 
                                             unitsFinal, workgroups, barrierIn, 
                                             isWarpReadyToRun, dId_, uId_, 
                                             msg_, wgId_, warpId_, instrId_, 
                                             pesIdx, u_i, t, bd, bu, w, 
                                             longWorkFlag, startTime, curTime, 
                                             localMemory_u, localId_u, 
                                             globalOffset_u, tileIdx_u, 
                                             readyToMin_u, minVal, dId_W, 
                                             uId_W, msg, wgId_W, warpId, 
                                             instrId, warps, warpEntry, dId, 
                                             uId, msgFromHost, msgFromSch, 
                                             wgId, readyToRun, wgEntry, 
                                             wgWaiting, msgFromDevice, 
                                             deviceIdx, wgIdx, i_1, j_1 >>

unit_done(self) == /\ pc[self] = "unit_done"
                   /\ TRUE
                   /\ pc' = [pc EXCEPT ![self] = "Done"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                   STOPWARPS, hostFlag, clockFlag, final, 
                                   hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                   u_sch, globalTime, Tmin, globalMemory, 
                                   workGroupSize, nWorkGroups, tileSize, 
                                   nWorkingDevices, nWorkingUnitsPerDevice, 
                                   nWorkingPEsPerUnit, allWorkingUnits, 
                                   nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                   nWarps, nInstructions, aoutput, unitsFinal, 
                                   workgroups, barrierIn, isWarpReadyToRun, 
                                   dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                   pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                   startTime, curTime, localMemory_u, 
                                   localId_u, globalOffset_u, tileIdx_u, 
                                   readyToMin_u, minVal, dId_W, uId_W, msg, 
                                   wgId_W, warpId, instrId, warps, warpEntry, 
                                   dId, uId, msgFromHost, msgFromSch, wgId, 
                                   readyToRun, wgEntry, wgWaiting, 
                                   msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

UnitProc(self) == unit_init(self) \/ unit_outer(self)
                     \/ unit_outer_check(self) \/ unit_init_local(self)
                     \/ unit_init_lm_loop(self) \/ unit_init_lm_set(self)
                     \/ unit_init_reg_loop(self) \/ unit_init_reg_set(self)
                     \/ unit_warp_loop(self) \/ unit_warp_check(self)
                     \/ unit_set_time(self) \/ unit_dispatch(self)
                     \/ unit_instr0(self) \/ unit_instr0_loop(self)
                     \/ unit_instr0_body(self) \/ unit_instr1(self)
                     \/ unit_instr1_loop(self) \/ unit_instr1_body(self)
                     \/ unit_instr2(self) \/ unit_instr2_loop(self)
                     \/ unit_instr2_body(self) \/ unit_instr3(self)
                     \/ unit_instr3_loop(self) \/ unit_instr3_body(self)
                     \/ unit_instr3_after(self) \/ unit_long_work(self)
                     \/ unit_work_step(self) \/ unit_work_await(self)
                     \/ unit_instr4(self) \/ unit_instr4_check(self)
                     \/ unit_instr5(self) \/ unit_barrier_d_loop(self)
                     \/ unit_barrier_d_body(self)
                     \/ unit_barrier_u_loop(self)
                     \/ unit_barrier_u_body(self)
                     \/ unit_barrier_w_loop(self)
                     \/ unit_barrier_w_body(self)
                     \/ unit_barrier_check(self)
                     \/ unit_barrier_reset_d(self)
                     \/ unit_barrier_reset_d_body(self)
                     \/ unit_barrier_reset_u(self)
                     \/ unit_barrier_reset_u_body(self)
                     \/ unit_barrier_reset_w(self)
                     \/ unit_barrier_reset_body(self) \/ unit_instr6(self)
                     \/ unit_instr6_reduce_loop(self)
                     \/ unit_instr6_reduce_body(self)
                     \/ unit_instr6_reduce_lw(self)
                     \/ unit_instr6_reduce_ws(self)
                     \/ unit_instr6_reduce_wa(self)
                     \/ unit_instr6_global_min(self)
                     \/ unit_instr6_global_lw(self)
                     \/ unit_instr6_global_ws(self)
                     \/ unit_instr6_global_wa(self)
                     \/ unit_send_done_warp(self) \/ unit_done(self)

ws_init(self) == /\ pc[self] = "ws_init"
                 /\    self[2] < nWorkingDevices
                    /\ self[3] < nWorkingUnitsPerDevice
                 /\ dId_W' = [dId_W EXCEPT ![self] = self[2]]
                 /\ uId_W' = [uId_W EXCEPT ![self] = self[3]]
                 /\ pc' = [pc EXCEPT ![self] = "ws_outer"]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                 STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                 d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                 globalTime, Tmin, globalMemory, workGroupSize, 
                                 nWorkGroups, tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingUnits, nRunningUnits, nWaitingUnits, 
                                 nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                                 unitsFinal, workgroups, barrierIn, 
                                 isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                                 warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                                 longWorkFlag, startTime, curTime, 
                                 localMemory_u, localId_u, globalOffset_u, 
                                 tileIdx_u, readyToMin_u, minVal, msg, wgId_W, 
                                 warpId, instrId, warps, warpEntry, dId, uId, 
                                 msgFromHost, msgFromSch, wgId, readyToRun, 
                                 wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                                 wgIdx, i_1, j_1 >>

ws_outer(self) == /\ pc[self] = "ws_outer"
                  /\ dev_sch[dId_W[self]][uId_W[self]] # <<>>
                  /\ msg' = [msg EXCEPT ![self] = Head(dev_sch[dId_W[self]][uId_W[self]])]
                  /\ dev_sch' = [dev_sch EXCEPT ![dId_W[self]][uId_W[self]] = Tail(dev_sch[dId_W[self]][uId_W[self]])]
                  /\ pc' = [pc EXCEPT ![self] = "ws_outer_check"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                  d_hst, sch_dev, sch_u, u_sch, globalTime, 
                                  Tmin, globalMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingUnits, nRunningUnits, 
                                  nWaitingUnits, nWarpsPerUnit, nWarps, 
                                  nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, wgId_W, warpId, 
                                  instrId, warps, warpEntry, dId, uId, 
                                  msgFromHost, msgFromSch, wgId, readyToRun, 
                                  wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                                  wgIdx, i_1, j_1 >>

ws_outer_check(self) == /\ pc[self] = "ws_outer_check"
                        /\ IF msg[self][1] = GO
                              THEN /\ wgId_W' = [wgId_W EXCEPT ![self] = msg[self][2]]
                                   /\ pc' = [pc EXCEPT ![self] = "ws_send_go_to_unit"]
                                   /\ sch_u' = sch_u
                              ELSE /\ IF msg[self][1] = STOP
                                         THEN /\ sch_u' = [sch_u EXCEPT ![dId_W[self]][uId_W[self]] = Append(sch_u[dId_W[self]][uId_W[self]], (<<STOP, 0>>))]
                                              /\ pc' = [pc EXCEPT ![self] = "ws_done"]
                                         ELSE /\ pc' = [pc EXCEPT ![self] = "ws_outer"]
                                              /\ sch_u' = sch_u
                                   /\ UNCHANGED wgId_W
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, u_sch, 
                                        globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, msg_, wgId_, warpId_, instrId_, 
                                        pesIdx, u_i, t, bd, bu, w, 
                                        longWorkFlag, startTime, curTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, warpId, instrId, warps, warpEntry, 
                                        dId, uId, msgFromHost, msgFromSch, 
                                        wgId, readyToRun, wgEntry, wgWaiting, 
                                        msgFromDevice, deviceIdx, wgIdx, i_1, 
                                        j_1 >>

ws_send_go_to_unit(self) == /\ pc[self] = "ws_send_go_to_unit"
                            /\ sch_u' = [sch_u EXCEPT ![dId_W[self]][uId_W[self]] = Append(sch_u[dId_W[self]][uId_W[self]], (<<GO, wgId_W[self]>>))]
                            /\ pc' = [pc EXCEPT ![self] = "ws_fill_warps"]
                            /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                            DONEWARP, STOPWARPS, hostFlag, 
                                            clockFlag, final, hst_d, d_hst, 
                                            dev_sch, sch_dev, u_sch, 
                                            globalTime, Tmin, globalMemory, 
                                            workGroupSize, nWorkGroups, 
                                            tileSize, nWorkingDevices, 
                                            nWorkingUnitsPerDevice, 
                                            nWorkingPEsPerUnit, 
                                            allWorkingUnits, nRunningUnits, 
                                            nWaitingUnits, nWarpsPerUnit, 
                                            nWarps, nInstructions, aoutput, 
                                            unitsFinal, workgroups, barrierIn, 
                                            isWarpReadyToRun, dId_, uId_, msg_, 
                                            wgId_, warpId_, instrId_, pesIdx, 
                                            u_i, t, bd, bu, w, longWorkFlag, 
                                            startTime, curTime, localMemory_u, 
                                            localId_u, globalOffset_u, 
                                            tileIdx_u, readyToMin_u, minVal, 
                                            dId_W, uId_W, msg, wgId_W, warpId, 
                                            instrId, warps, warpEntry, dId, 
                                            uId, msgFromHost, msgFromSch, wgId, 
                                            readyToRun, wgEntry, wgWaiting, 
                                            msgFromDevice, deviceIdx, wgIdx, 
                                            i_1, j_1 >>

ws_fill_warps(self) == /\ pc[self] = "ws_fill_warps"
                       /\ warpId' = [warpId EXCEPT ![self] = 0]
                       /\ warps' = [warps EXCEPT ![self] = <<>>]
                       /\ pc' = [pc EXCEPT ![self] = "ws_fill_loop"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, instrId, warpEntry, dId, uId, 
                                       msgFromHost, msgFromSch, wgId, 
                                       readyToRun, wgEntry, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

ws_fill_loop(self) == /\ pc[self] = "ws_fill_loop"
                      /\ IF warpId[self] >= nWarpsPerUnit
                            THEN /\ pc' = [pc EXCEPT ![self] = "ws_schedule_loop"]
                            ELSE /\ pc' = [pc EXCEPT ![self] = "ws_fill_body"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                      STOPWARPS, hostFlag, clockFlag, final, 
                                      hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                      u_sch, globalTime, Tmin, globalMemory, 
                                      workGroupSize, nWorkGroups, tileSize, 
                                      nWorkingDevices, nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingUnits, 
                                      nRunningUnits, nWaitingUnits, 
                                      nWarpsPerUnit, nWarps, nInstructions, 
                                      aoutput, unitsFinal, workgroups, 
                                      barrierIn, isWarpReadyToRun, dId_, uId_, 
                                      msg_, wgId_, warpId_, instrId_, pesIdx, 
                                      u_i, t, bd, bu, w, longWorkFlag, 
                                      startTime, curTime, localMemory_u, 
                                      localId_u, globalOffset_u, tileIdx_u, 
                                      readyToMin_u, minVal, dId_W, uId_W, msg, 
                                      wgId_W, warpId, instrId, warps, 
                                      warpEntry, dId, uId, msgFromHost, 
                                      msgFromSch, wgId, readyToRun, wgEntry, 
                                      wgWaiting, msgFromDevice, deviceIdx, 
                                      wgIdx, i_1, j_1 >>

ws_fill_body(self) == /\ pc[self] = "ws_fill_body"
                      /\ isWarpReadyToRun' = [isWarpReadyToRun EXCEPT ![dId_W[self]][uId_W[self] * nWarpsPerUnit + warpId[self]] = 1]
                      /\ warps' = [warps EXCEPT ![self] = Append(warps[self], <<warpId[self], 0>>)]
                      /\ warpId' = [warpId EXCEPT ![self] = warpId[self] + 1]
                      /\ pc' = [pc EXCEPT ![self] = "ws_fill_loop"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                      STOPWARPS, hostFlag, clockFlag, final, 
                                      hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                      u_sch, globalTime, Tmin, globalMemory, 
                                      workGroupSize, nWorkGroups, tileSize, 
                                      nWorkingDevices, nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingUnits, 
                                      nRunningUnits, nWaitingUnits, 
                                      nWarpsPerUnit, nWarps, nInstructions, 
                                      aoutput, unitsFinal, workgroups, 
                                      barrierIn, dId_, uId_, msg_, wgId_, 
                                      warpId_, instrId_, pesIdx, u_i, t, bd, 
                                      bu, w, longWorkFlag, startTime, curTime, 
                                      localMemory_u, localId_u, globalOffset_u, 
                                      tileIdx_u, readyToMin_u, minVal, dId_W, 
                                      uId_W, msg, wgId_W, instrId, warpEntry, 
                                      dId, uId, msgFromHost, msgFromSch, wgId, 
                                      readyToRun, wgEntry, wgWaiting, 
                                      msgFromDevice, deviceIdx, wgIdx, i_1, 
                                      j_1 >>

ws_schedule_loop(self) == /\ pc[self] = "ws_schedule_loop"
                          /\ IF warps[self] = <<>>
                                THEN /\ pc' = [pc EXCEPT ![self] = "ws_all_warps_done"]
                                ELSE /\ pc' = [pc EXCEPT ![self] = "ws_take_warp"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                          DONEWARP, STOPWARPS, hostFlag, 
                                          clockFlag, final, hst_d, d_hst, 
                                          dev_sch, sch_dev, sch_u, u_sch, 
                                          globalTime, Tmin, globalMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingUnits, 
                                          nRunningUnits, nWaitingUnits, 
                                          nWarpsPerUnit, nWarps, nInstructions, 
                                          aoutput, unitsFinal, workgroups, 
                                          barrierIn, isWarpReadyToRun, dId_, 
                                          uId_, msg_, wgId_, warpId_, instrId_, 
                                          pesIdx, u_i, t, bd, bu, w, 
                                          longWorkFlag, startTime, curTime, 
                                          localMemory_u, localId_u, 
                                          globalOffset_u, tileIdx_u, 
                                          readyToMin_u, minVal, dId_W, uId_W, 
                                          msg, wgId_W, warpId, instrId, warps, 
                                          warpEntry, dId, uId, msgFromHost, 
                                          msgFromSch, wgId, readyToRun, 
                                          wgEntry, wgWaiting, msgFromDevice, 
                                          deviceIdx, wgIdx, i_1, j_1 >>

ws_take_warp(self) == /\ pc[self] = "ws_take_warp"
                      /\ warpEntry' = [warpEntry EXCEPT ![self] = Head(warps[self])]
                      /\ warps' = [warps EXCEPT ![self] = Tail(warps[self])]
                      /\ warpId' = [warpId EXCEPT ![self] = warpEntry'[self][1]]
                      /\ instrId' = [instrId EXCEPT ![self] = warpEntry'[self][2]]
                      /\ pc' = [pc EXCEPT ![self] = "ws_check_ready"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                      STOPWARPS, hostFlag, clockFlag, final, 
                                      hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                      u_sch, globalTime, Tmin, globalMemory, 
                                      workGroupSize, nWorkGroups, tileSize, 
                                      nWorkingDevices, nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingUnits, 
                                      nRunningUnits, nWaitingUnits, 
                                      nWarpsPerUnit, nWarps, nInstructions, 
                                      aoutput, unitsFinal, workgroups, 
                                      barrierIn, isWarpReadyToRun, dId_, uId_, 
                                      msg_, wgId_, warpId_, instrId_, pesIdx, 
                                      u_i, t, bd, bu, w, longWorkFlag, 
                                      startTime, curTime, localMemory_u, 
                                      localId_u, globalOffset_u, tileIdx_u, 
                                      readyToMin_u, minVal, dId_W, uId_W, msg, 
                                      wgId_W, dId, uId, msgFromHost, 
                                      msgFromSch, wgId, readyToRun, wgEntry, 
                                      wgWaiting, msgFromDevice, deviceIdx, 
                                      wgIdx, i_1, j_1 >>

ws_check_ready(self) == /\ pc[self] = "ws_check_ready"
                        /\ isWarpReadyToRun' = [isWarpReadyToRun EXCEPT ![dId_W[self]][uId_W[self] * nWarpsPerUnit + warpId[self]] = 1 - barrierIn[dId_W[self]][uId_W[self] * nWarpsPerUnit + warpId[self]]]
                        /\ pc' = [pc EXCEPT ![self] = "ws_dispatch"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, dId_, uId_, msg_, wgId_, 
                                        warpId_, instrId_, pesIdx, u_i, t, bd, 
                                        bu, w, longWorkFlag, startTime, 
                                        curTime, localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, uId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

ws_dispatch(self) == /\ pc[self] = "ws_dispatch"
                     /\ IF isWarpReadyToRun[dId_W[self]][uId_W[self] * nWarpsPerUnit + warpId[self]] = 1
                           THEN /\ pc' = [pc EXCEPT ![self] = "ws_send_to_unit"]
                                /\ warps' = warps
                           ELSE /\ warps' = [warps EXCEPT ![self] = Append(warps[self], <<warpId[self], instrId[self]>>)]
                                /\ pc' = [pc EXCEPT ![self] = "ws_schedule_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                     STOPWARPS, hostFlag, clockFlag, final, 
                                     hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                     u_sch, globalTime, Tmin, globalMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingUnits, 
                                     nRunningUnits, nWaitingUnits, 
                                     nWarpsPerUnit, nWarps, nInstructions, 
                                     aoutput, unitsFinal, workgroups, 
                                     barrierIn, isWarpReadyToRun, dId_, uId_, 
                                     msg_, wgId_, warpId_, instrId_, pesIdx, 
                                     u_i, t, bd, bu, w, longWorkFlag, 
                                     startTime, curTime, localMemory_u, 
                                     localId_u, globalOffset_u, tileIdx_u, 
                                     readyToMin_u, minVal, dId_W, uId_W, msg, 
                                     wgId_W, warpId, instrId, warpEntry, dId, 
                                     uId, msgFromHost, msgFromSch, wgId, 
                                     readyToRun, wgEntry, wgWaiting, 
                                     msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

ws_send_to_unit(self) == /\ pc[self] = "ws_send_to_unit"
                         /\ sch_u' = [sch_u EXCEPT ![dId_W[self]][uId_W[self]] = Append(sch_u[dId_W[self]][uId_W[self]], (<<GOWARP, warpId[self], instrId[self]>>))]
                         /\ pc' = [pc EXCEPT ![self] = "ws_wait_done"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_dev, u_sch, globalTime, 
                                         Tmin, globalMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, u_i, t, bd, bu, w, 
                                         longWorkFlag, startTime, curTime, 
                                         localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

ws_wait_done(self) == /\ pc[self] = "ws_wait_done"
                      /\ u_sch[dId_W[self]][uId_W[self]] # <<>>
                      /\ msg' = [msg EXCEPT ![self] = Head(u_sch[dId_W[self]][uId_W[self]])]
                      /\ u_sch' = [u_sch EXCEPT ![dId_W[self]][uId_W[self]] = Tail(u_sch[dId_W[self]][uId_W[self]])]
                      /\ pc' = [pc EXCEPT ![self] = "ws_after_done"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                      STOPWARPS, hostFlag, clockFlag, final, 
                                      hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                      globalTime, Tmin, globalMemory, 
                                      workGroupSize, nWorkGroups, tileSize, 
                                      nWorkingDevices, nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingUnits, 
                                      nRunningUnits, nWaitingUnits, 
                                      nWarpsPerUnit, nWarps, nInstructions, 
                                      aoutput, unitsFinal, workgroups, 
                                      barrierIn, isWarpReadyToRun, dId_, uId_, 
                                      msg_, wgId_, warpId_, instrId_, pesIdx, 
                                      u_i, t, bd, bu, w, longWorkFlag, 
                                      startTime, curTime, localMemory_u, 
                                      localId_u, globalOffset_u, tileIdx_u, 
                                      readyToMin_u, minVal, dId_W, uId_W, 
                                      wgId_W, warpId, instrId, warps, 
                                      warpEntry, dId, uId, msgFromHost, 
                                      msgFromSch, wgId, readyToRun, wgEntry, 
                                      wgWaiting, msgFromDevice, deviceIdx, 
                                      wgIdx, i_1, j_1 >>

ws_after_done(self) == /\ pc[self] = "ws_after_done"
                       /\ instrId' = [instrId EXCEPT ![self] = msg[self][2] + 1]
                       /\ pc' = [pc EXCEPT ![self] = "ws_requeue"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, warps, warpEntry, dId, 
                                       uId, msgFromHost, msgFromSch, wgId, 
                                       readyToRun, wgEntry, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

ws_requeue(self) == /\ pc[self] = "ws_requeue"
                    /\ IF instrId[self] < nInstructions
                          THEN /\ warps' = [warps EXCEPT ![self] = Append(warps[self], <<warpId[self], instrId[self]>>)]
                          ELSE /\ TRUE
                               /\ warps' = warps
                    /\ pc' = [pc EXCEPT ![self] = "ws_schedule_loop"]
                    /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                    STOPWARPS, hostFlag, clockFlag, final, 
                                    hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                    u_sch, globalTime, Tmin, globalMemory, 
                                    workGroupSize, nWorkGroups, tileSize, 
                                    nWorkingDevices, nWorkingUnitsPerDevice, 
                                    nWorkingPEsPerUnit, allWorkingUnits, 
                                    nRunningUnits, nWaitingUnits, 
                                    nWarpsPerUnit, nWarps, nInstructions, 
                                    aoutput, unitsFinal, workgroups, barrierIn, 
                                    isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                                    warpId_, instrId_, pesIdx, u_i, t, bd, bu, 
                                    w, longWorkFlag, startTime, curTime, 
                                    localMemory_u, localId_u, globalOffset_u, 
                                    tileIdx_u, readyToMin_u, minVal, dId_W, 
                                    uId_W, msg, wgId_W, warpId, instrId, 
                                    warpEntry, dId, uId, msgFromHost, 
                                    msgFromSch, wgId, readyToRun, wgEntry, 
                                    wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                                    i_1, j_1 >>

ws_all_warps_done(self) == /\ pc[self] = "ws_all_warps_done"
                           /\ sch_u' = [sch_u EXCEPT ![dId_W[self]][uId_W[self]] = Append(sch_u[dId_W[self]][uId_W[self]], (<<STOPWARPS, 0>>))]
                           /\ pc' = [pc EXCEPT ![self] = "ws_report_done"]
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, u_sch, globalTime, 
                                           Tmin, globalMemory, workGroupSize, 
                                           nWorkGroups, tileSize, 
                                           nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, 
                                           u_i, t, bd, bu, w, longWorkFlag, 
                                           startTime, curTime, localMemory_u, 
                                           localId_u, globalOffset_u, 
                                           tileIdx_u, readyToMin_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, uId, 
                                           msgFromHost, msgFromSch, wgId, 
                                           readyToRun, wgEntry, wgWaiting, 
                                           msgFromDevice, deviceIdx, wgIdx, 
                                           i_1, j_1 >>

ws_report_done(self) == /\ pc[self] = "ws_report_done"
                        /\ sch_dev' = [sch_dev EXCEPT ![dId_W[self]][uId_W[self]] = Append(sch_dev[dId_W[self]][uId_W[self]], (<<DONE, uId_W[self]>>))]
                        /\ pc' = [pc EXCEPT ![self] = "ws_outer"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_u, u_sch, 
                                        globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, msg_, wgId_, warpId_, instrId_, 
                                        pesIdx, u_i, t, bd, bu, w, 
                                        longWorkFlag, startTime, curTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, uId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

ws_done(self) == /\ pc[self] = "ws_done"
                 /\ TRUE
                 /\ pc' = [pc EXCEPT ![self] = "Done"]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                 STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                 d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                 globalTime, Tmin, globalMemory, workGroupSize, 
                                 nWorkGroups, tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingUnits, nRunningUnits, nWaitingUnits, 
                                 nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                                 unitsFinal, workgroups, barrierIn, 
                                 isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                                 warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                                 longWorkFlag, startTime, curTime, 
                                 localMemory_u, localId_u, globalOffset_u, 
                                 tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, 
                                 msg, wgId_W, warpId, instrId, warps, 
                                 warpEntry, dId, uId, msgFromHost, msgFromSch, 
                                 wgId, readyToRun, wgEntry, wgWaiting, 
                                 msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

WarpSch(self) == ws_init(self) \/ ws_outer(self) \/ ws_outer_check(self)
                    \/ ws_send_go_to_unit(self) \/ ws_fill_warps(self)
                    \/ ws_fill_loop(self) \/ ws_fill_body(self)
                    \/ ws_schedule_loop(self) \/ ws_take_warp(self)
                    \/ ws_check_ready(self) \/ ws_dispatch(self)
                    \/ ws_send_to_unit(self) \/ ws_wait_done(self)
                    \/ ws_after_done(self) \/ ws_requeue(self)
                    \/ ws_all_warps_done(self) \/ ws_report_done(self)
                    \/ ws_done(self)

dev_init(self) == /\ pc[self] = "dev_init"
                  /\ dId' = [dId EXCEPT ![self] = self[2]]
                  /\ pc' = [pc EXCEPT ![self] = "dev_loop"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                  d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                  globalTime, Tmin, globalMemory, 
                                  workGroupSize, nWorkGroups, tileSize, 
                                  nWorkingDevices, nWorkingUnitsPerDevice, 
                                  nWorkingPEsPerUnit, allWorkingUnits, 
                                  nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                  nWarps, nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                  instrId, warps, warpEntry, uId, msgFromHost, 
                                  msgFromSch, wgId, readyToRun, wgEntry, 
                                  wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                                  i_1, j_1 >>

dev_loop(self) == /\ pc[self] = "dev_loop"
                  /\ hst_d # <<>>
                  /\ msgFromHost' = [msgFromHost EXCEPT ![self] = Head(hst_d)]
                  /\ hst_d' = Tail(hst_d)
                  /\ pc' = [pc EXCEPT ![self] = "dev_check"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, d_hst, 
                                  dev_sch, sch_dev, sch_u, u_sch, globalTime, 
                                  Tmin, globalMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingUnits, nRunningUnits, 
                                  nWaitingUnits, nWarpsPerUnit, nWarps, 
                                  nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                  instrId, warps, warpEntry, dId, uId, 
                                  msgFromSch, wgId, readyToRun, wgEntry, 
                                  wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                                  i_1, j_1 >>

dev_check(self) == /\ pc[self] = "dev_check"
                   /\ IF msgFromHost[self] = GO
                         THEN /\ pc' = [pc EXCEPT ![self] = "dev_distribute"]
                         ELSE /\ IF msgFromHost[self] = STOP
                                    THEN /\ pc' = [pc EXCEPT ![self] = "dev_send_stop"]
                                    ELSE /\ pc' = [pc EXCEPT ![self] = "dev_loop"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                   STOPWARPS, hostFlag, clockFlag, final, 
                                   hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                   u_sch, globalTime, Tmin, globalMemory, 
                                   workGroupSize, nWorkGroups, tileSize, 
                                   nWorkingDevices, nWorkingUnitsPerDevice, 
                                   nWorkingPEsPerUnit, allWorkingUnits, 
                                   nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                   nWarps, nInstructions, aoutput, unitsFinal, 
                                   workgroups, barrierIn, isWarpReadyToRun, 
                                   dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                   pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                   startTime, curTime, localMemory_u, 
                                   localId_u, globalOffset_u, tileIdx_u, 
                                   readyToMin_u, minVal, dId_W, uId_W, msg, 
                                   wgId_W, warpId, instrId, warps, warpEntry, 
                                   dId, uId, msgFromHost, msgFromSch, wgId, 
                                   readyToRun, wgEntry, wgWaiting, 
                                   msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

dev_distribute(self) == /\ pc[self] = "dev_distribute"
                        /\ uId' = [uId EXCEPT ![self] = 0]
                        /\ pc' = [pc EXCEPT ![self] = "dev_dist_loop"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, msg_, wgId_, warpId_, instrId_, 
                                        pesIdx, u_i, t, bd, bu, w, 
                                        longWorkFlag, startTime, curTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

dev_dist_loop(self) == /\ pc[self] = "dev_dist_loop"
                       /\ IF uId[self] >= nWorkingUnitsPerDevice
                             THEN /\ pc' = [pc EXCEPT ![self] = "dev_after_dist"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "dev_dist_take"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, instrId, warps, 
                                       warpEntry, dId, uId, msgFromHost, 
                                       msgFromSch, wgId, readyToRun, wgEntry, 
                                       wgWaiting, msgFromDevice, deviceIdx, 
                                       wgIdx, i_1, j_1 >>

dev_dist_take(self) == /\ pc[self] = "dev_dist_take"
                       /\ workgroups # <<>>
                       /\ wgEntry' = [wgEntry EXCEPT ![self] = Head(workgroups)]
                       /\ workgroups' = Tail(workgroups)
                       /\ wgId' = [wgId EXCEPT ![self] = wgEntry'[self][1]]
                       /\ readyToRun' = [readyToRun EXCEPT ![self] = wgEntry'[self][2]]
                       /\ pc' = [pc EXCEPT ![self] = "dev_dist_check"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, barrierIn, 
                                       isWarpReadyToRun, dId_, uId_, msg_, 
                                       wgId_, warpId_, instrId_, pesIdx, u_i, 
                                       t, bd, bu, w, longWorkFlag, startTime, 
                                       curTime, localMemory_u, localId_u, 
                                       globalOffset_u, tileIdx_u, readyToMin_u, 
                                       minVal, dId_W, uId_W, msg, wgId_W, 
                                       warpId, instrId, warps, warpEntry, dId, 
                                       uId, msgFromHost, msgFromSch, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

dev_dist_check(self) == /\ pc[self] = "dev_dist_check"
                        /\ IF readyToRun[self]
                              THEN /\ dev_sch' = [dev_sch EXCEPT ![dId[self]][uId[self]] = Append(dev_sch[dId[self]][uId[self]], (<<GO, wgId[self]>>))]
                                   /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                                   /\ pc' = [pc EXCEPT ![self] = "dev_dist_loop"]
                                   /\ UNCHANGED workgroups
                              ELSE /\ workgroups' = Append(workgroups, <<wgId[self], TRUE>>)
                                   /\ pc' = [pc EXCEPT ![self] = "dev_dist_take"]
                                   /\ UNCHANGED << dev_sch, uId >>
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, sch_dev, sch_u, u_sch, 
                                        globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, barrierIn, 
                                        isWarpReadyToRun, dId_, uId_, msg_, 
                                        wgId_, warpId_, instrId_, pesIdx, u_i, 
                                        t, bd, bu, w, longWorkFlag, startTime, 
                                        curTime, localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

dev_after_dist(self) == /\ pc[self] = "dev_after_dist"
                        /\ IF nWorkGroups <= nWorkingUnitsPerDevice
                              THEN /\ uId' = [uId EXCEPT ![self] = 0]
                                   /\ pc' = [pc EXCEPT ![self] = "dev_simple_wait"]
                              ELSE /\ uId' = [uId EXCEPT ![self] = 0]
                                   /\ pc' = [pc EXCEPT ![self] = "dev_overflow_wait"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingUnits, 
                                        nRunningUnits, nWaitingUnits, 
                                        nWarpsPerUnit, nWarps, nInstructions, 
                                        aoutput, unitsFinal, workgroups, 
                                        barrierIn, isWarpReadyToRun, dId_, 
                                        uId_, msg_, wgId_, warpId_, instrId_, 
                                        pesIdx, u_i, t, bd, bu, w, 
                                        longWorkFlag, startTime, curTime, 
                                        localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

dev_simple_wait(self) == /\ pc[self] = "dev_simple_wait"
                         /\ IF uId[self] >= nWorkingUnitsPerDevice
                               THEN /\ pc' = [pc EXCEPT ![self] = "dev_report_done"]
                               ELSE /\ pc' = [pc EXCEPT ![self] = "dev_simple_recv"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_dev, sch_u, u_sch, 
                                         globalTime, Tmin, globalMemory, 
                                         workGroupSize, nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, u_i, t, bd, bu, w, 
                                         longWorkFlag, startTime, curTime, 
                                         localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

dev_simple_recv(self) == /\ pc[self] = "dev_simple_recv"
                         /\ \E u2 \in 0..(nWorkingUnitsPerDevice-1):
                              /\ sch_dev[dId[self]][u2] # <<>>
                              /\ msgFromSch' = [msgFromSch EXCEPT ![self] = Head(sch_dev[dId[self]][u2])]
                              /\ sch_dev' = [sch_dev EXCEPT ![dId[self]][u2] = Tail(sch_dev[dId[self]][u2])]
                         /\ pc' = [pc EXCEPT ![self] = "dev_simple_dec"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, d_hst, 
                                         dev_sch, sch_u, u_sch, globalTime, 
                                         Tmin, globalMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, u_i, t, bd, bu, w, 
                                         longWorkFlag, startTime, curTime, 
                                         localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         wgId, readyToRun, wgEntry, wgWaiting, 
                                         msgFromDevice, deviceIdx, wgIdx, i_1, 
                                         j_1 >>

dev_simple_dec(self) == /\ pc[self] = "dev_simple_dec"
                        /\ allWorkingUnits' = allWorkingUnits - 1
                        /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                        /\ pc' = [pc EXCEPT ![self] = "dev_simple_wait"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                        STOPWARPS, hostFlag, clockFlag, final, 
                                        hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                        u_sch, globalTime, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, nRunningUnits, 
                                        nWaitingUnits, nWarpsPerUnit, nWarps, 
                                        nInstructions, aoutput, unitsFinal, 
                                        workgroups, barrierIn, 
                                        isWarpReadyToRun, dId_, uId_, msg_, 
                                        wgId_, warpId_, instrId_, pesIdx, u_i, 
                                        t, bd, bu, w, longWorkFlag, startTime, 
                                        curTime, localMemory_u, localId_u, 
                                        globalOffset_u, tileIdx_u, 
                                        readyToMin_u, minVal, dId_W, uId_W, 
                                        msg, wgId_W, warpId, instrId, warps, 
                                        warpEntry, dId, msgFromHost, 
                                        msgFromSch, wgId, readyToRun, wgEntry, 
                                        wgWaiting, msgFromDevice, deviceIdx, 
                                        wgIdx, i_1, j_1 >>

dev_overflow_wait(self) == /\ pc[self] = "dev_overflow_wait"
                           /\ IF uId[self] >= nWorkGroups - nWorkingUnitsPerDevice
                                 THEN /\ pc' = [pc EXCEPT ![self] = "dev_overflow_final"]
                                 ELSE /\ pc' = [pc EXCEPT ![self] = "dev_overflow_recv"]
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, 
                                           u_i, t, bd, bu, w, longWorkFlag, 
                                           startTime, curTime, localMemory_u, 
                                           localId_u, globalOffset_u, 
                                           tileIdx_u, readyToMin_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, uId, 
                                           msgFromHost, msgFromSch, wgId, 
                                           readyToRun, wgEntry, wgWaiting, 
                                           msgFromDevice, deviceIdx, wgIdx, 
                                           i_1, j_1 >>

dev_overflow_recv(self) == /\ pc[self] = "dev_overflow_recv"
                           /\ \E u2 \in 0..(nWorkingUnitsPerDevice-1):
                                /\ sch_dev[dId[self]][u2] # <<>>
                                /\ msgFromSch' = [msgFromSch EXCEPT ![self] = Head(sch_dev[dId[self]][u2])]
                                /\ sch_dev' = [sch_dev EXCEPT ![dId[self]][u2] = Tail(sch_dev[dId[self]][u2])]
                           /\ pc' = [pc EXCEPT ![self] = "dev_overflow_requeue"]
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_u, u_sch, globalTime, 
                                           Tmin, globalMemory, workGroupSize, 
                                           nWorkGroups, tileSize, 
                                           nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, 
                                           u_i, t, bd, bu, w, longWorkFlag, 
                                           startTime, curTime, localMemory_u, 
                                           localId_u, globalOffset_u, 
                                           tileIdx_u, readyToMin_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, uId, 
                                           msgFromHost, wgId, readyToRun, 
                                           wgEntry, wgWaiting, msgFromDevice, 
                                           deviceIdx, wgIdx, i_1, j_1 >>

dev_overflow_requeue(self) == /\ pc[self] = "dev_overflow_requeue"
                              /\ wgWaiting' = [wgWaiting EXCEPT ![self] = TRUE]
                              /\ pc' = [pc EXCEPT ![self] = "dev_overflow_take"]
                              /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                              DONEWARP, STOPWARPS, hostFlag, 
                                              clockFlag, final, hst_d, d_hst, 
                                              dev_sch, sch_dev, sch_u, u_sch, 
                                              globalTime, Tmin, globalMemory, 
                                              workGroupSize, nWorkGroups, 
                                              tileSize, nWorkingDevices, 
                                              nWorkingUnitsPerDevice, 
                                              nWorkingPEsPerUnit, 
                                              allWorkingUnits, nRunningUnits, 
                                              nWaitingUnits, nWarpsPerUnit, 
                                              nWarps, nInstructions, aoutput, 
                                              unitsFinal, workgroups, 
                                              barrierIn, isWarpReadyToRun, 
                                              dId_, uId_, msg_, wgId_, warpId_, 
                                              instrId_, pesIdx, u_i, t, bd, bu, 
                                              w, longWorkFlag, startTime, 
                                              curTime, localMemory_u, 
                                              localId_u, globalOffset_u, 
                                              tileIdx_u, readyToMin_u, minVal, 
                                              dId_W, uId_W, msg, wgId_W, 
                                              warpId, instrId, warps, 
                                              warpEntry, dId, uId, msgFromHost, 
                                              msgFromSch, wgId, readyToRun, 
                                              wgEntry, msgFromDevice, 
                                              deviceIdx, wgIdx, i_1, j_1 >>

dev_overflow_take(self) == /\ pc[self] = "dev_overflow_take"
                           /\ IF ~wgWaiting[self]
                                 THEN /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                                      /\ pc' = [pc EXCEPT ![self] = "dev_overflow_wait"]
                                 ELSE /\ pc' = [pc EXCEPT ![self] = "dev_overflow_take2"]
                                      /\ uId' = uId
                           /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                           DONEWARP, STOPWARPS, hostFlag, 
                                           clockFlag, final, hst_d, d_hst, 
                                           dev_sch, sch_dev, sch_u, u_sch, 
                                           globalTime, Tmin, globalMemory, 
                                           workGroupSize, nWorkGroups, 
                                           tileSize, nWorkingDevices, 
                                           nWorkingUnitsPerDevice, 
                                           nWorkingPEsPerUnit, allWorkingUnits, 
                                           nRunningUnits, nWaitingUnits, 
                                           nWarpsPerUnit, nWarps, 
                                           nInstructions, aoutput, unitsFinal, 
                                           workgroups, barrierIn, 
                                           isWarpReadyToRun, dId_, uId_, msg_, 
                                           wgId_, warpId_, instrId_, pesIdx, 
                                           u_i, t, bd, bu, w, longWorkFlag, 
                                           startTime, curTime, localMemory_u, 
                                           localId_u, globalOffset_u, 
                                           tileIdx_u, readyToMin_u, minVal, 
                                           dId_W, uId_W, msg, wgId_W, warpId, 
                                           instrId, warps, warpEntry, dId, 
                                           msgFromHost, msgFromSch, wgId, 
                                           readyToRun, wgEntry, wgWaiting, 
                                           msgFromDevice, deviceIdx, wgIdx, 
                                           i_1, j_1 >>

dev_overflow_take2(self) == /\ pc[self] = "dev_overflow_take2"
                            /\ workgroups # <<>>
                            /\ wgEntry' = [wgEntry EXCEPT ![self] = Head(workgroups)]
                            /\ workgroups' = Tail(workgroups)
                            /\ wgId' = [wgId EXCEPT ![self] = wgEntry'[self][1]]
                            /\ readyToRun' = [readyToRun EXCEPT ![self] = wgEntry'[self][2]]
                            /\ pc' = [pc EXCEPT ![self] = "dev_overflow_take_check"]
                            /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                            DONEWARP, STOPWARPS, hostFlag, 
                                            clockFlag, final, hst_d, d_hst, 
                                            dev_sch, sch_dev, sch_u, u_sch, 
                                            globalTime, Tmin, globalMemory, 
                                            workGroupSize, nWorkGroups, 
                                            tileSize, nWorkingDevices, 
                                            nWorkingUnitsPerDevice, 
                                            nWorkingPEsPerUnit, 
                                            allWorkingUnits, nRunningUnits, 
                                            nWaitingUnits, nWarpsPerUnit, 
                                            nWarps, nInstructions, aoutput, 
                                            unitsFinal, barrierIn, 
                                            isWarpReadyToRun, dId_, uId_, msg_, 
                                            wgId_, warpId_, instrId_, pesIdx, 
                                            u_i, t, bd, bu, w, longWorkFlag, 
                                            startTime, curTime, localMemory_u, 
                                            localId_u, globalOffset_u, 
                                            tileIdx_u, readyToMin_u, minVal, 
                                            dId_W, uId_W, msg, wgId_W, warpId, 
                                            instrId, warps, warpEntry, dId, 
                                            uId, msgFromHost, msgFromSch, 
                                            wgWaiting, msgFromDevice, 
                                            deviceIdx, wgIdx, i_1, j_1 >>

dev_overflow_take_check(self) == /\ pc[self] = "dev_overflow_take_check"
                                 /\ IF readyToRun[self]
                                       THEN /\ \E u2 \in 0..(nWorkingUnitsPerDevice-1):
                                                 dev_sch' = [dev_sch EXCEPT ![dId[self]][u2] = Append(dev_sch[dId[self]][u2], (<<GO, wgId[self]>>))]
                                            /\ wgWaiting' = [wgWaiting EXCEPT ![self] = FALSE]
                                            /\ pc' = [pc EXCEPT ![self] = "dev_overflow_take"]
                                            /\ UNCHANGED workgroups
                                       ELSE /\ workgroups' = Append(workgroups, <<wgId[self], TRUE>>)
                                            /\ pc' = [pc EXCEPT ![self] = "dev_overflow_take2"]
                                            /\ UNCHANGED << dev_sch, wgWaiting >>
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, sch_dev, sch_u, u_sch, 
                                                 globalTime, Tmin, 
                                                 globalMemory, workGroupSize, 
                                                 nWorkGroups, tileSize, 
                                                 nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWaitingUnits, 
                                                 nWarpsPerUnit, nWarps, 
                                                 nInstructions, aoutput, 
                                                 unitsFinal, barrierIn, 
                                                 isWarpReadyToRun, dId_, uId_, 
                                                 msg_, wgId_, warpId_, 
                                                 instrId_, pesIdx, u_i, t, bd, 
                                                 bu, w, longWorkFlag, 
                                                 startTime, curTime, 
                                                 localMemory_u, localId_u, 
                                                 globalOffset_u, tileIdx_u, 
                                                 readyToMin_u, minVal, dId_W, 
                                                 uId_W, msg, wgId_W, warpId, 
                                                 instrId, warps, warpEntry, 
                                                 dId, uId, msgFromHost, 
                                                 msgFromSch, wgId, readyToRun, 
                                                 wgEntry, msgFromDevice, 
                                                 deviceIdx, wgIdx, i_1, j_1 >>

dev_overflow_final(self) == /\ pc[self] = "dev_overflow_final"
                            /\ uId' = [uId EXCEPT ![self] = 0]
                            /\ pc' = [pc EXCEPT ![self] = "dev_overflow_final_loop"]
                            /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                            DONEWARP, STOPWARPS, hostFlag, 
                                            clockFlag, final, hst_d, d_hst, 
                                            dev_sch, sch_dev, sch_u, u_sch, 
                                            globalTime, Tmin, globalMemory, 
                                            workGroupSize, nWorkGroups, 
                                            tileSize, nWorkingDevices, 
                                            nWorkingUnitsPerDevice, 
                                            nWorkingPEsPerUnit, 
                                            allWorkingUnits, nRunningUnits, 
                                            nWaitingUnits, nWarpsPerUnit, 
                                            nWarps, nInstructions, aoutput, 
                                            unitsFinal, workgroups, barrierIn, 
                                            isWarpReadyToRun, dId_, uId_, msg_, 
                                            wgId_, warpId_, instrId_, pesIdx, 
                                            u_i, t, bd, bu, w, longWorkFlag, 
                                            startTime, curTime, localMemory_u, 
                                            localId_u, globalOffset_u, 
                                            tileIdx_u, readyToMin_u, minVal, 
                                            dId_W, uId_W, msg, wgId_W, warpId, 
                                            instrId, warps, warpEntry, dId, 
                                            msgFromHost, msgFromSch, wgId, 
                                            readyToRun, wgEntry, wgWaiting, 
                                            msgFromDevice, deviceIdx, wgIdx, 
                                            i_1, j_1 >>

dev_overflow_final_loop(self) == /\ pc[self] = "dev_overflow_final_loop"
                                 /\ IF uId[self] >= nWorkingUnitsPerDevice
                                       THEN /\ pc' = [pc EXCEPT ![self] = "dev_report_done"]
                                       ELSE /\ pc' = [pc EXCEPT ![self] = "dev_overflow_final_recv"]
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, dev_sch, sch_dev, 
                                                 sch_u, u_sch, globalTime, 
                                                 Tmin, globalMemory, 
                                                 workGroupSize, nWorkGroups, 
                                                 tileSize, nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWaitingUnits, 
                                                 nWarpsPerUnit, nWarps, 
                                                 nInstructions, aoutput, 
                                                 unitsFinal, workgroups, 
                                                 barrierIn, isWarpReadyToRun, 
                                                 dId_, uId_, msg_, wgId_, 
                                                 warpId_, instrId_, pesIdx, 
                                                 u_i, t, bd, bu, w, 
                                                 longWorkFlag, startTime, 
                                                 curTime, localMemory_u, 
                                                 localId_u, globalOffset_u, 
                                                 tileIdx_u, readyToMin_u, 
                                                 minVal, dId_W, uId_W, msg, 
                                                 wgId_W, warpId, instrId, 
                                                 warps, warpEntry, dId, uId, 
                                                 msgFromHost, msgFromSch, wgId, 
                                                 readyToRun, wgEntry, 
                                                 wgWaiting, msgFromDevice, 
                                                 deviceIdx, wgIdx, i_1, j_1 >>

dev_overflow_final_recv(self) == /\ pc[self] = "dev_overflow_final_recv"
                                 /\ \E u2 \in 0..(nWorkingUnitsPerDevice-1):
                                      /\ sch_dev[dId[self]][u2] # <<>>
                                      /\ msgFromSch' = [msgFromSch EXCEPT ![self] = Head(sch_dev[dId[self]][u2])]
                                      /\ sch_dev' = [sch_dev EXCEPT ![dId[self]][u2] = Tail(sch_dev[dId[self]][u2])]
                                 /\ pc' = [pc EXCEPT ![self] = "dev_overflow_final_dec"]
                                 /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                 DONEWARP, STOPWARPS, hostFlag, 
                                                 clockFlag, final, hst_d, 
                                                 d_hst, dev_sch, sch_u, u_sch, 
                                                 globalTime, Tmin, 
                                                 globalMemory, workGroupSize, 
                                                 nWorkGroups, tileSize, 
                                                 nWorkingDevices, 
                                                 nWorkingUnitsPerDevice, 
                                                 nWorkingPEsPerUnit, 
                                                 allWorkingUnits, 
                                                 nRunningUnits, nWaitingUnits, 
                                                 nWarpsPerUnit, nWarps, 
                                                 nInstructions, aoutput, 
                                                 unitsFinal, workgroups, 
                                                 barrierIn, isWarpReadyToRun, 
                                                 dId_, uId_, msg_, wgId_, 
                                                 warpId_, instrId_, pesIdx, 
                                                 u_i, t, bd, bu, w, 
                                                 longWorkFlag, startTime, 
                                                 curTime, localMemory_u, 
                                                 localId_u, globalOffset_u, 
                                                 tileIdx_u, readyToMin_u, 
                                                 minVal, dId_W, uId_W, msg, 
                                                 wgId_W, warpId, instrId, 
                                                 warps, warpEntry, dId, uId, 
                                                 msgFromHost, wgId, readyToRun, 
                                                 wgEntry, wgWaiting, 
                                                 msgFromDevice, deviceIdx, 
                                                 wgIdx, i_1, j_1 >>

dev_overflow_final_dec(self) == /\ pc[self] = "dev_overflow_final_dec"
                                /\ allWorkingUnits' = allWorkingUnits - 1
                                /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                                /\ pc' = [pc EXCEPT ![self] = "dev_overflow_final_loop"]
                                /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                                DONEWARP, STOPWARPS, hostFlag, 
                                                clockFlag, final, hst_d, d_hst, 
                                                dev_sch, sch_dev, sch_u, u_sch, 
                                                globalTime, Tmin, globalMemory, 
                                                workGroupSize, nWorkGroups, 
                                                tileSize, nWorkingDevices, 
                                                nWorkingUnitsPerDevice, 
                                                nWorkingPEsPerUnit, 
                                                nRunningUnits, nWaitingUnits, 
                                                nWarpsPerUnit, nWarps, 
                                                nInstructions, aoutput, 
                                                unitsFinal, workgroups, 
                                                barrierIn, isWarpReadyToRun, 
                                                dId_, uId_, msg_, wgId_, 
                                                warpId_, instrId_, pesIdx, u_i, 
                                                t, bd, bu, w, longWorkFlag, 
                                                startTime, curTime, 
                                                localMemory_u, localId_u, 
                                                globalOffset_u, tileIdx_u, 
                                                readyToMin_u, minVal, dId_W, 
                                                uId_W, msg, wgId_W, warpId, 
                                                instrId, warps, warpEntry, dId, 
                                                msgFromHost, msgFromSch, wgId, 
                                                readyToRun, wgEntry, wgWaiting, 
                                                msgFromDevice, deviceIdx, 
                                                wgIdx, i_1, j_1 >>

dev_report_done(self) == /\ pc[self] = "dev_report_done"
                         /\ d_hst' = Append(d_hst, DONE)
                         /\ pc' = [pc EXCEPT ![self] = "dev_loop"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, 
                                         DONEWARP, STOPWARPS, hostFlag, 
                                         clockFlag, final, hst_d, dev_sch, 
                                         sch_dev, sch_u, u_sch, globalTime, 
                                         Tmin, globalMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingUnits, 
                                         nRunningUnits, nWaitingUnits, 
                                         nWarpsPerUnit, nWarps, nInstructions, 
                                         aoutput, unitsFinal, workgroups, 
                                         barrierIn, isWarpReadyToRun, dId_, 
                                         uId_, msg_, wgId_, warpId_, instrId_, 
                                         pesIdx, u_i, t, bd, bu, w, 
                                         longWorkFlag, startTime, curTime, 
                                         localMemory_u, localId_u, 
                                         globalOffset_u, tileIdx_u, 
                                         readyToMin_u, minVal, dId_W, uId_W, 
                                         msg, wgId_W, warpId, instrId, warps, 
                                         warpEntry, dId, uId, msgFromHost, 
                                         msgFromSch, wgId, readyToRun, wgEntry, 
                                         wgWaiting, msgFromDevice, deviceIdx, 
                                         wgIdx, i_1, j_1 >>

dev_send_stop(self) == /\ pc[self] = "dev_send_stop"
                       /\ uId' = [uId EXCEPT ![self] = 0]
                       /\ pc' = [pc EXCEPT ![self] = "dev_stop_loop"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, instrId, warps, 
                                       warpEntry, dId, msgFromHost, msgFromSch, 
                                       wgId, readyToRun, wgEntry, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

dev_stop_loop(self) == /\ pc[self] = "dev_stop_loop"
                       /\ IF uId[self] >= nWorkingUnitsPerDevice
                             THEN /\ pc' = [pc EXCEPT ![self] = "dev_done"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "dev_stop_send"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, dev_sch, sch_dev, sch_u, 
                                       u_sch, globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, instrId, warps, 
                                       warpEntry, dId, uId, msgFromHost, 
                                       msgFromSch, wgId, readyToRun, wgEntry, 
                                       wgWaiting, msgFromDevice, deviceIdx, 
                                       wgIdx, i_1, j_1 >>

dev_stop_send(self) == /\ pc[self] = "dev_stop_send"
                       /\ dev_sch' = [dev_sch EXCEPT ![dId[self]][uId[self]] = Append(dev_sch[dId[self]][uId[self]], (<<STOP, 0>>))]
                       /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                       /\ pc' = [pc EXCEPT ![self] = "dev_stop_loop"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                       STOPWARPS, hostFlag, clockFlag, final, 
                                       hst_d, d_hst, sch_dev, sch_u, u_sch, 
                                       globalTime, Tmin, globalMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingUnits, 
                                       nRunningUnits, nWaitingUnits, 
                                       nWarpsPerUnit, nWarps, nInstructions, 
                                       aoutput, unitsFinal, workgroups, 
                                       barrierIn, isWarpReadyToRun, dId_, uId_, 
                                       msg_, wgId_, warpId_, instrId_, pesIdx, 
                                       u_i, t, bd, bu, w, longWorkFlag, 
                                       startTime, curTime, localMemory_u, 
                                       localId_u, globalOffset_u, tileIdx_u, 
                                       readyToMin_u, minVal, dId_W, uId_W, msg, 
                                       wgId_W, warpId, instrId, warps, 
                                       warpEntry, dId, msgFromHost, msgFromSch, 
                                       wgId, readyToRun, wgEntry, wgWaiting, 
                                       msgFromDevice, deviceIdx, wgIdx, i_1, 
                                       j_1 >>

dev_done(self) == /\ pc[self] = "dev_done"
                  /\ TRUE
                  /\ pc' = [pc EXCEPT ![self] = "Done"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                  d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                  globalTime, Tmin, globalMemory, 
                                  workGroupSize, nWorkGroups, tileSize, 
                                  nWorkingDevices, nWorkingUnitsPerDevice, 
                                  nWorkingPEsPerUnit, allWorkingUnits, 
                                  nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                  nWarps, nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                  instrId, warps, warpEntry, dId, uId, 
                                  msgFromHost, msgFromSch, wgId, readyToRun, 
                                  wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                                  wgIdx, i_1, j_1 >>

DeviceProc(self) == dev_init(self) \/ dev_loop(self) \/ dev_check(self)
                       \/ dev_distribute(self) \/ dev_dist_loop(self)
                       \/ dev_dist_take(self) \/ dev_dist_check(self)
                       \/ dev_after_dist(self) \/ dev_simple_wait(self)
                       \/ dev_simple_recv(self) \/ dev_simple_dec(self)
                       \/ dev_overflow_wait(self)
                       \/ dev_overflow_recv(self)
                       \/ dev_overflow_requeue(self)
                       \/ dev_overflow_take(self)
                       \/ dev_overflow_take2(self)
                       \/ dev_overflow_take_check(self)
                       \/ dev_overflow_final(self)
                       \/ dev_overflow_final_loop(self)
                       \/ dev_overflow_final_recv(self)
                       \/ dev_overflow_final_dec(self)
                       \/ dev_report_done(self) \/ dev_send_stop(self)
                       \/ dev_stop_loop(self) \/ dev_stop_send(self)
                       \/ dev_done(self)

h0 == /\ pc[<<5,0,0>>] = "h0"
      /\ hostFlag = TRUE
      /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h1"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

h1 == /\ pc[<<5,0,0>>] = "h1"
      /\ final' = FALSE
      /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_fill_wg"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, hst_d, d_hst, dev_sch, sch_dev, 
                      sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

h_fill_wg == /\ pc[<<5,0,0>>] = "h_fill_wg"
             /\ wgIdx' = 0
             /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_fill_loop"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                             hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                             sch_dev, sch_u, u_sch, globalTime, Tmin, 
                             globalMemory, workGroupSize, nWorkGroups, 
                             tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
                             nWorkingPEsPerUnit, allWorkingUnits, 
                             nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                             nWarps, nInstructions, aoutput, unitsFinal, 
                             workgroups, barrierIn, isWarpReadyToRun, dId_, 
                             uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                             t, bd, bu, w, longWorkFlag, startTime, curTime, 
                             localMemory_u, localId_u, globalOffset_u, 
                             tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, 
                             msg, wgId_W, warpId, instrId, warps, warpEntry, 
                             dId, uId, msgFromHost, msgFromSch, wgId, 
                             readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                             deviceIdx, i_1, j_1 >>

h_fill_loop == /\ pc[<<5,0,0>>] = "h_fill_loop"
               /\ IF wgIdx >= nWorkGroups
                     THEN /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_send_go"]
                     ELSE /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_fill_body"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                               d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                               globalTime, Tmin, globalMemory, workGroupSize, 
                               nWorkGroups, tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                               wgIdx, i_1, j_1 >>

h_fill_body == /\ pc[<<5,0,0>>] = "h_fill_body"
               /\ workgroups' = Append(workgroups, <<wgIdx, TRUE>>)
               /\ wgIdx' = wgIdx + 1
               /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_fill_loop"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                               d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                               globalTime, Tmin, globalMemory, workGroupSize, 
                               nWorkGroups, tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, barrierIn, isWarpReadyToRun, dId_, 
                               uId_, msg_, wgId_, warpId_, instrId_, pesIdx, 
                               u_i, t, bd, bu, w, longWorkFlag, startTime, 
                               curTime, localMemory_u, localId_u, 
                               globalOffset_u, tileIdx_u, readyToMin_u, minVal, 
                               dId_W, uId_W, msg, wgId_W, warpId, instrId, 
                               warps, warpEntry, dId, uId, msgFromHost, 
                               msgFromSch, wgId, readyToRun, wgEntry, 
                               wgWaiting, msgFromDevice, deviceIdx, i_1, j_1 >>

h_send_go == /\ pc[<<5,0,0>>] = "h_send_go"
             /\ deviceIdx' = 0
             /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_send_go_loop"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                             hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                             sch_dev, sch_u, u_sch, globalTime, Tmin, 
                             globalMemory, workGroupSize, nWorkGroups, 
                             tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
                             nWorkingPEsPerUnit, allWorkingUnits, 
                             nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                             nWarps, nInstructions, aoutput, unitsFinal, 
                             workgroups, barrierIn, isWarpReadyToRun, dId_, 
                             uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                             t, bd, bu, w, longWorkFlag, startTime, curTime, 
                             localMemory_u, localId_u, globalOffset_u, 
                             tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, 
                             msg, wgId_W, warpId, instrId, warps, warpEntry, 
                             dId, uId, msgFromHost, msgFromSch, wgId, 
                             readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                             wgIdx, i_1, j_1 >>

h_send_go_loop == /\ pc[<<5,0,0>>] = "h_send_go_loop"
                  /\ IF deviceIdx >= nWorkingDevices
                        THEN /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_done"]
                        ELSE /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_send_go_body"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                  d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                  globalTime, Tmin, globalMemory, 
                                  workGroupSize, nWorkGroups, tileSize, 
                                  nWorkingDevices, nWorkingUnitsPerDevice, 
                                  nWorkingPEsPerUnit, allWorkingUnits, 
                                  nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                                  nWarps, nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                  instrId, warps, warpEntry, dId, uId, 
                                  msgFromHost, msgFromSch, wgId, readyToRun, 
                                  wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                                  wgIdx, i_1, j_1 >>

h_send_go_body == /\ pc[<<5,0,0>>] = "h_send_go_body"
                  /\ hst_d' = Append(hst_d, GO)
                  /\ deviceIdx' = deviceIdx + 1
                  /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_send_go_loop"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                  STOPWARPS, hostFlag, clockFlag, final, d_hst, 
                                  dev_sch, sch_dev, sch_u, u_sch, globalTime, 
                                  Tmin, globalMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingUnits, nRunningUnits, 
                                  nWaitingUnits, nWarpsPerUnit, nWarps, 
                                  nInstructions, aoutput, unitsFinal, 
                                  workgroups, barrierIn, isWarpReadyToRun, 
                                  dId_, uId_, msg_, wgId_, warpId_, instrId_, 
                                  pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                                  startTime, curTime, localMemory_u, localId_u, 
                                  globalOffset_u, tileIdx_u, readyToMin_u, 
                                  minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                                  instrId, warps, warpEntry, dId, uId, 
                                  msgFromHost, msgFromSch, wgId, readyToRun, 
                                  wgEntry, wgWaiting, msgFromDevice, wgIdx, 
                                  i_1, j_1 >>

h_wait_done == /\ pc[<<5,0,0>>] = "h_wait_done"
               /\ deviceIdx' = 0
               /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_loop"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                               d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                               globalTime, Tmin, globalMemory, workGroupSize, 
                               nWorkGroups, tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, msgFromDevice, wgIdx, i_1, 
                               j_1 >>

h_wait_loop == /\ pc[<<5,0,0>>] = "h_wait_loop"
               /\ IF deviceIdx >= nWorkingDevices
                     THEN /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_units"]
                     ELSE /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_recv"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                               d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                               globalTime, Tmin, globalMemory, workGroupSize, 
                               nWorkGroups, tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                               wgIdx, i_1, j_1 >>

h_wait_recv == /\ pc[<<5,0,0>>] = "h_wait_recv"
               /\ d_hst # <<>>
               /\ msgFromDevice' = Head(d_hst)
               /\ d_hst' = Tail(d_hst)
               /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_stop"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                               dev_sch, sch_dev, sch_u, u_sch, globalTime, 
                               Tmin, globalMemory, workGroupSize, nWorkGroups, 
                               tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, deviceIdx, wgIdx, i_1, j_1 >>

h_wait_stop == /\ pc[<<5,0,0>>] = "h_wait_stop"
               /\ hst_d' = Append(hst_d, STOP)
               /\ deviceIdx' = deviceIdx + 1
               /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_wait_loop"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, final, d_hst, 
                               dev_sch, sch_dev, sch_u, u_sch, globalTime, 
                               Tmin, globalMemory, workGroupSize, nWorkGroups, 
                               tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, msgFromDevice, wgIdx, i_1, 
                               j_1 >>

h_wait_units == /\ pc[<<5,0,0>>] = "h_wait_units"
                /\ unitsFinal = nWorkingUnitsPerDevice * nWorkingDevices
                /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_set_final"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                                STOPWARPS, hostFlag, clockFlag, final, hst_d, 
                                d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                                globalTime, Tmin, globalMemory, workGroupSize, 
                                nWorkGroups, tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingUnits, nRunningUnits, nWaitingUnits, 
                                nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                                unitsFinal, workgroups, barrierIn, 
                                isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                                warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                                longWorkFlag, startTime, curTime, 
                                localMemory_u, localId_u, globalOffset_u, 
                                tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, 
                                msg, wgId_W, warpId, instrId, warps, warpEntry, 
                                dId, uId, msgFromHost, msgFromSch, wgId, 
                                readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                                deviceIdx, wgIdx, i_1, j_1 >>

h_set_final == /\ pc[<<5,0,0>>] = "h_set_final"
               /\ final' = TRUE
               /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "h_done"]
               /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, 
                               STOPWARPS, hostFlag, clockFlag, hst_d, d_hst, 
                               dev_sch, sch_dev, sch_u, u_sch, globalTime, 
                               Tmin, globalMemory, workGroupSize, nWorkGroups, 
                               tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               allWorkingUnits, nRunningUnits, nWaitingUnits, 
                               nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                               unitsFinal, workgroups, barrierIn, 
                               isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                               warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                               longWorkFlag, startTime, curTime, localMemory_u, 
                               localId_u, globalOffset_u, tileIdx_u, 
                               readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                               warpId, instrId, warps, warpEntry, dId, uId, 
                               msgFromHost, msgFromSch, wgId, readyToRun, 
                               wgEntry, wgWaiting, msgFromDevice, deviceIdx, 
                               wgIdx, i_1, j_1 >>

h_done == /\ pc[<<5,0,0>>] = "h_done"
          /\ TRUE
          /\ pc' = [pc EXCEPT ![<<5,0,0>>] = "Done"]
          /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                          hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                          sch_dev, sch_u, u_sch, globalTime, Tmin, 
                          globalMemory, workGroupSize, nWorkGroups, tileSize, 
                          nWorkingDevices, nWorkingUnitsPerDevice, 
                          nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, 
                          nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                          aoutput, unitsFinal, workgroups, barrierIn, 
                          isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                          instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                          startTime, curTime, localMemory_u, localId_u, 
                          globalOffset_u, tileIdx_u, readyToMin_u, minVal, 
                          dId_W, uId_W, msg, wgId_W, warpId, instrId, warps, 
                          warpEntry, dId, uId, msgFromHost, msgFromSch, wgId, 
                          readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                          deviceIdx, wgIdx, i_1, j_1 >>

Host == h0 \/ h1 \/ h_fill_wg \/ h_fill_loop \/ h_fill_body \/ h_send_go
           \/ h_send_go_loop \/ h_send_go_body \/ h_wait_done
           \/ h_wait_loop \/ h_wait_recv \/ h_wait_stop \/ h_wait_units
           \/ h_set_final \/ h_done

m_loop_gm == /\ pc[<<1,0,0>>] = "m_loop_gm"
             /\ IF i_1 < INPUT_DATA_SIZE
                   THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m1"]
                   ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m_WG"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                             hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                             sch_dev, sch_u, u_sch, globalTime, Tmin, 
                             globalMemory, workGroupSize, nWorkGroups, 
                             tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
                             nWorkingPEsPerUnit, allWorkingUnits, 
                             nRunningUnits, nWaitingUnits, nWarpsPerUnit, 
                             nWarps, nInstructions, aoutput, unitsFinal, 
                             workgroups, barrierIn, isWarpReadyToRun, dId_, 
                             uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                             t, bd, bu, w, longWorkFlag, startTime, curTime, 
                             localMemory_u, localId_u, globalOffset_u, 
                             tileIdx_u, readyToMin_u, minVal, dId_W, uId_W, 
                             msg, wgId_W, warpId, instrId, warps, warpEntry, 
                             dId, uId, msgFromHost, msgFromSch, wgId, 
                             readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                             deviceIdx, wgIdx, i_1, j_1 >>

m1 == /\ pc[<<1,0,0>>] = "m1"
      /\ globalMemory' = [globalMemory EXCEPT ![i_1] = INPUT_DATA_SIZE - i_1]
      /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m2"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, workGroupSize, 
                      nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m2 == /\ pc[<<1,0,0>>] = "m2"
      /\ i_1' = i_1 + 1
      /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m_loop_gm"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, j_1 >>

m_WG == /\ pc[<<1,0,0>>] = "m_WG"
        /\ \E k \in 2..(N - 1):
             workGroupSize' = (INPUT_DATA_SIZE \div (2^(N - k)))
        /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m_TS"]
        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                        hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                        sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                        nWorkGroups, tileSize, nWorkingDevices, 
                        nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                        allWorkingUnits, nRunningUnits, nWaitingUnits, 
                        nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                        unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                        dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, 
                        u_i, t, bd, bu, w, longWorkFlag, startTime, curTime, 
                        localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                        readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                        warpId, instrId, warps, warpEntry, dId, uId, 
                        msgFromHost, msgFromSch, wgId, readyToRun, wgEntry, 
                        wgWaiting, msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m_TS == /\ pc[<<1,0,0>>] = "m_TS"
        /\ \E l \in 1..(N - 2):
             tileSize' = (INPUT_DATA_SIZE \div (2^(N - l)))
        /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m5"]
        /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                        hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                        sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                        workGroupSize, nWorkGroups, nWorkingDevices, 
                        nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                        allWorkingUnits, nRunningUnits, nWaitingUnits, 
                        nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                        unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                        dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, 
                        u_i, t, bd, bu, w, longWorkFlag, startTime, curTime, 
                        localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                        readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, 
                        warpId, instrId, warps, warpEntry, dId, uId, 
                        msgFromHost, msgFromSch, wgId, readyToRun, wgEntry, 
                        wgWaiting, msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m5 == /\ pc[<<1,0,0>>] = "m5"
      /\ IF workGroupSize * tileSize > INPUT_DATA_SIZE
            THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m6"]
            ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m7"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m6 == /\ pc[<<1,0,0>>] = "m6"
      /\ tileSize' = (INPUT_DATA_SIZE \div workGroupSize)
      /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m7"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m7 == /\ pc[<<1,0,0>>] = "m7"
      /\ nWorkGroups' = (INPUT_DATA_SIZE \div (workGroupSize * tileSize))
      /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m8"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m8 == /\ pc[<<1,0,0>>] = "m8"
      /\ nWorkingDevices' = DEVICES
      /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m9"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m9 == /\ pc[<<1,0,0>>] = "m9"
      /\ IF nWorkGroups <= UNITS_PER_DEVICE * DEVICES
            THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m10"]
            ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m11"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                      hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                      sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingUnits, nRunningUnits, nWaitingUnits, 
                      nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                      unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                      dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                      t, bd, bu, w, longWorkFlag, startTime, curTime, 
                      localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                      readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                      instrId, warps, warpEntry, dId, uId, msgFromHost, 
                      msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                      msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m10 == /\ pc[<<1,0,0>>] = "m10"
       /\ nWorkingDevices' = (nWorkGroups \div UNITS_PER_DEVICE)
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m11"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m11 == /\ pc[<<1,0,0>>] = "m11"
       /\ IF (nWorkGroups \div UNITS_PER_DEVICE) # 0
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m12"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m13"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m12 == /\ pc[<<1,0,0>>] = "m12"
       /\ nWorkingDevices' = 1
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m13"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m13 == /\ pc[<<1,0,0>>] = "m13"
       /\ nWorkingUnitsPerDevice' = UNITS_PER_DEVICE
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m14"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, 
                       nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                       aoutput, unitsFinal, workgroups, barrierIn, 
                       isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                       instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                       startTime, curTime, localMemory_u, localId_u, 
                       globalOffset_u, tileIdx_u, readyToMin_u, minVal, dId_W, 
                       uId_W, msg, wgId_W, warpId, instrId, warps, warpEntry, 
                       dId, uId, msgFromHost, msgFromSch, wgId, readyToRun, 
                       wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                       i_1, j_1 >>

m14 == /\ pc[<<1,0,0>>] = "m14"
       /\ IF nWorkGroups <= UNITS_PER_DEVICE
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m15"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m16"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m15 == /\ pc[<<1,0,0>>] = "m15"
       /\ nWorkingUnitsPerDevice' = nWorkGroups
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m16"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingPEsPerUnit, allWorkingUnits, nRunningUnits, 
                       nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                       aoutput, unitsFinal, workgroups, barrierIn, 
                       isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                       instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                       startTime, curTime, localMemory_u, localId_u, 
                       globalOffset_u, tileIdx_u, readyToMin_u, minVal, dId_W, 
                       uId_W, msg, wgId_W, warpId, instrId, warps, warpEntry, 
                       dId, uId, msgFromHost, msgFromSch, wgId, readyToRun, 
                       wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                       i_1, j_1 >>

m16 == /\ pc[<<1,0,0>>] = "m16"
       /\ nWorkingPEsPerUnit' = PES_PER_UNIT
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m17"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, allWorkingUnits, nRunningUnits, 
                       nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                       aoutput, unitsFinal, workgroups, barrierIn, 
                       isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                       instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                       startTime, curTime, localMemory_u, localId_u, 
                       globalOffset_u, tileIdx_u, readyToMin_u, minVal, dId_W, 
                       uId_W, msg, wgId_W, warpId, instrId, warps, warpEntry, 
                       dId, uId, msgFromHost, msgFromSch, wgId, readyToRun, 
                       wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                       i_1, j_1 >>

m17 == /\ pc[<<1,0,0>>] = "m17"
       /\ IF workGroupSize <= PES_PER_UNIT
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m18"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m19"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

m18 == /\ pc[<<1,0,0>>] = "m18"
       /\ nWorkingPEsPerUnit' = workGroupSize
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m19"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, allWorkingUnits, nRunningUnits, 
                       nWaitingUnits, nWarpsPerUnit, nWarps, nInstructions, 
                       aoutput, unitsFinal, workgroups, barrierIn, 
                       isWarpReadyToRun, dId_, uId_, msg_, wgId_, warpId_, 
                       instrId_, pesIdx, u_i, t, bd, bu, w, longWorkFlag, 
                       startTime, curTime, localMemory_u, localId_u, 
                       globalOffset_u, tileIdx_u, readyToMin_u, minVal, dId_W, 
                       uId_W, msg, wgId_W, warpId, instrId, warps, warpEntry, 
                       dId, uId, msgFromHost, msgFromSch, wgId, readyToRun, 
                       wgEntry, wgWaiting, msgFromDevice, deviceIdx, wgIdx, 
                       i_1, j_1 >>

m19 == /\ pc[<<1,0,0>>] = "m19"
       /\ allWorkingUnits' = nWorkingDevices * nWorkingUnitsPerDevice
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m20"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       nRunningUnits, nWaitingUnits, nWarpsPerUnit, nWarps, 
                       nInstructions, aoutput, unitsFinal, workgroups, 
                       barrierIn, isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                       warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                       longWorkFlag, startTime, curTime, localMemory_u, 
                       localId_u, globalOffset_u, tileIdx_u, readyToMin_u, 
                       minVal, dId_W, uId_W, msg, wgId_W, warpId, instrId, 
                       warps, warpEntry, dId, uId, msgFromHost, msgFromSch, 
                       wgId, readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                       deviceIdx, wgIdx, i_1, j_1 >>

m20 == /\ pc[<<1,0,0>>] = "m20"
       /\ nWarpsPerUnit' = (workGroupSize \div nWorkingPEsPerUnit)
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m21"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, nWarps, 
                       nInstructions, aoutput, unitsFinal, workgroups, 
                       barrierIn, isWarpReadyToRun, dId_, uId_, msg_, wgId_, 
                       warpId_, instrId_, pesIdx, u_i, t, bd, bu, w, 
                       longWorkFlag, startTime, curTime, localMemory_u, 
                       localId_u, globalOffset_u, tileIdx_u, readyToMin_u, 
                       minVal, dId_W, uId_W, msg, wgId_W, warpId, instrId, 
                       warps, warpEntry, dId, uId, msgFromHost, msgFromSch, 
                       wgId, readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                       deviceIdx, wgIdx, i_1, j_1 >>

m21 == /\ pc[<<1,0,0>>] = "m21"
       /\ nWarps' = nWarpsPerUnit * nWorkingUnitsPerDevice * nWorkingDevices
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "m22"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       hostFlag, clockFlag, final, hst_d, d_hst, dev_sch, 
                       sch_dev, sch_u, u_sch, globalTime, Tmin, globalMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nInstructions, aoutput, unitsFinal, 
                       workgroups, barrierIn, isWarpReadyToRun, dId_, uId_, 
                       msg_, wgId_, warpId_, instrId_, pesIdx, u_i, t, bd, bu, 
                       w, longWorkFlag, startTime, curTime, localMemory_u, 
                       localId_u, globalOffset_u, tileIdx_u, readyToMin_u, 
                       minVal, dId_W, uId_W, msg, wgId_W, warpId, instrId, 
                       warps, warpEntry, dId, uId, msgFromHost, msgFromSch, 
                       wgId, readyToRun, wgEntry, wgWaiting, msgFromDevice, 
                       deviceIdx, wgIdx, i_1, j_1 >>

m22 == /\ pc[<<1,0,0>>] = "m22"
       /\ hostFlag' = TRUE
       /\ clockFlag' = TRUE
       /\ pc' = [pc EXCEPT ![<<1,0,0>>] = "Done"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, GOWARP, DONEWARP, STOPWARPS, 
                       final, hst_d, d_hst, dev_sch, sch_dev, sch_u, u_sch, 
                       globalTime, Tmin, globalMemory, workGroupSize, 
                       nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingUnits, nRunningUnits, nWaitingUnits, 
                       nWarpsPerUnit, nWarps, nInstructions, aoutput, 
                       unitsFinal, workgroups, barrierIn, isWarpReadyToRun, 
                       dId_, uId_, msg_, wgId_, warpId_, instrId_, pesIdx, u_i, 
                       t, bd, bu, w, longWorkFlag, startTime, curTime, 
                       localMemory_u, localId_u, globalOffset_u, tileIdx_u, 
                       readyToMin_u, minVal, dId_W, uId_W, msg, wgId_W, warpId, 
                       instrId, warps, warpEntry, dId, uId, msgFromHost, 
                       msgFromSch, wgId, readyToRun, wgEntry, wgWaiting, 
                       msgFromDevice, deviceIdx, wgIdx, i_1, j_1 >>

Main == m_loop_gm \/ m1 \/ m2 \/ m_WG \/ m_TS \/ m5 \/ m6 \/ m7 \/ m8 \/ m9
           \/ m10 \/ m11 \/ m12 \/ m13 \/ m14 \/ m15 \/ m16 \/ m17 \/ m18
           \/ m19 \/ m20 \/ m21 \/ m22

(* Allow infinite stuttering to prevent deadlock on termination. *)
Terminating == /\ \A self \in ProcSet: pc[self] = "Done"
               /\ UNCHANGED vars

Next == Clock \/ Host \/ Main
           \/ (\E self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                          u \in 0..(UNITS_PER_DEVICE-1) }: UnitProc(self))
           \/ (\E self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                          u \in 0..(UNITS_PER_DEVICE-1) }: WarpSch(self))
           \/ (\E self \in { <<4,d,0>> : d \in 0..(DEVICES-1) }: DeviceProc(self))
           \/ Terminating

Spec == /\ Init /\ [][Next]_vars
        /\ WF_vars(Clock)
        /\ \A self \in { <<3,d,u>> : d \in 0..(DEVICES-1),
                                      u \in 0..(UNITS_PER_DEVICE-1) } : WF_vars(UnitProc(self))
        /\ \A self \in { <<2,d,u>> : d \in 0..(DEVICES-1),
                                      u \in 0..(UNITS_PER_DEVICE-1) } : WF_vars(WarpSch(self))
        /\ \A self \in { <<4,d,0>> : d \in 0..(DEVICES-1) } : WF_vars(DeviceProc(self))
        /\ WF_vars(Host)
        /\ WF_vars(Main)

Termination == <>(\A self \in ProcSet: pc[self] = "Done")

\* END TRANSLATION 
OverTime == [](final => (globalTime > Tmin))

=============================================================================
\* Modification History
\* Last modified Tue Apr 21 15:18:45 MSK 2026 by s.flusova
\* Created Fri Apr 17 12:21:01 MSK 2026 by s.flusova