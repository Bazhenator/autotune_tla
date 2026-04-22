-------------------------------- MODULE autotune --------------------------------
EXTENDS Integers, Sequences, TLC

CONSTANTS
    N,
    INPUT_DATA_SIZE,
    GLOBAL_MEMORY_ACCESS,
    LOCAL_MEMORY_SIZE,
    UNITS_PER_DEVICE,
    DEVICES,
    PES_PER_UNIT

(*--algorithm Main {
    variables
              (* MSGS *)
              GO          = "GO",
              STOP        = "STOP",
              DONE        = "DONE",
              GOWG        = "GOWG",
              EOI         = "EOI",
              NEOI        = "NEOI",
              
              (* flags *)
              hostFlag    = FALSE,
              clockFlag   = FALSE,
              flag        = FALSE,
              final       = FALSE,
              
              (* processes chans *)
              hst_d = <<>>,
              d_hst = <<>>,
              dev_u = [d \in 0..(DEVICES-1) |-> [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],
              u_dev = [d \in 0..(DEVICES-1) |-> [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]],
              u_pex = [d \in 0..(DEVICES-1) |->
                      [u \in 0..(UNITS_PER_DEVICE-1) |->
                      [p \in 0..(PES_PER_UNIT-1) |-> <<>>]]],
              pex_u = [d \in 0..(DEVICES-1) |->
                      [u \in 0..(UNITS_PER_DEVICE-1) |->
                      [p \in 0..(PES_PER_UNIT-1) |-> <<>>]]],
              
              (* time variables *)
              globalTime = 0,
              Tmin = 24, (* configured param *)
              
              (* memory *)
              globalMemory = [i \in 0..(INPUT_DATA_SIZE - 1) |-> 0],
              localMemory  = [j \in 0..(LOCAL_MEMORY_SIZE * UNITS_PER_DEVICE - 1) |-> 0],
              
              (* tuning params *)
              workGroupSize = 0,
              nWorkGroups   = 0,
              tileSize      = 0,
              
              (* service variables *)
              nWorkingDevices        = 0,
              nWorkingUnitsPerDevice = 0,
              nWorkingPEsPerUnit     = 0,
              allWorkingPEs          = 0,
              nRunningPEs            = 0,
              aoutput                = 0,
              
              (* FAA registers *)
              nWaitingPEs    = 0,
              nWaitingPEsOut = 0;

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
    macro Send3(m, chan, d, u, p) {
        chan[d][u][p] := Append(chan[d][u][p], m);
    }
    macro Rcv3(v, chan, d, u, p) {
        await chan[d][u][p] # <<>>;
        v := Head(chan[d][u][p]);
        chan[d][u][p] := Tail(chan[d][u][p]);
    }

    (* ========== BARRIER ========== *)

    procedure Barrier()
    variable bt = 0;
    {
        bar_faa1:
            bt := nWaitingPEs;
            nWaitingPEs := nWaitingPEs + 1;

        bar_wait1:
            if (bt < (nWorkingPEsPerUnit * nWorkingUnitsPerDevice - 1)) {
                await nWaitingPEs = 0;
            } else {
                nWaitingPEs := 0;
            };

        bar_faa2:
            bt := nWaitingPEsOut;
            nWaitingPEsOut := nWaitingPEsOut + 1;

        bar_wait2:
            if (bt < (nWorkingPEsPerUnit * nWorkingUnitsPerDevice - 1)) {
                await nWaitingPEsOut = 0;
            } else {
                nWaitingPEsOut := 0;
            };

        bar_ret:
            return;
    }

    
    fair process (PEX \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                           u \in 0..(UNITS_PER_DEVICE-1),
                                           p \in 0..(PES_PER_UNIT-1) })
    variable dId = 0, uId = 0, pId = 0,
             msg1 = <<>>,
             startTime = 0, curTime = 0,
             globalOffset = 0, tileIdx = 0, localMemIdx = 0,
             wgIter = 0, wgId = 0, localId = 0, t = 0;
    {
        p_init:
            await self[2] < nWorkingDevices
               /\ self[3] < nWorkingUnitsPerDevice
               /\ self[4] < nWorkingPEsPerUnit;
            dId := self[2];
            uId := self[3];
            pId := self[4];

        p_lmem:
            localMemIdx := pId + uId * nWorkingPEsPerUnit;

        (* ======== OUTER LOOP: wait for GOWG or STOP ======== *)
        pex_outer:
            Rcv3(msg1, u_pex, dId, uId, pId);

        pex_outer_check:
            if (msg1[1] = STOP) {
                goto pex_stop_phase;
            } else if (msg1[1] = GOWG) {
                wgId := msg1[2];
                goto pex_inner;
            } else {
                goto pex_outer;
            };

        (* ======== INNER LOOP: wait for GO iterations ======== *)
        pex_inner:
            Rcv3(msg1, u_pex, dId, uId, pId);

        pex_inner_check:
            if (msg1[1] = GO) {
                wgIter := msg1[2];
                goto pex_compute;
            } else if (msg1[1] = STOP) {
                goto pex_stop_phase;
            } else {
                goto pex_inner;
            };

        (* ---- compute phase: one GO iteration ---- *)
        pex_compute:
            startTime := globalTime;
            curTime   := globalTime;

        pex_local_id:
            if (workGroupSize > nWorkingPEsPerUnit) {
                localId := pId + wgIter * nWorkingPEsPerUnit;
            } else {
                localId := pId;
            };

        pex_offset:
            globalOffset := tileSize * (wgId * workGroupSize + localId);
            tileIdx := 0;

        (* ---- tile loop ---- *)
        pex_tile_loop:
            if (tileIdx >= tileSize) {
                goto pex_call_barrier;
            };

        pex_tile_bound:
            if (tileIdx + globalOffset >= INPUT_DATA_SIZE) {
                goto pex_call_barrier;
            };

        (* even_sum *)
        pex_even_sum:
            if (globalMemory[tileIdx + globalOffset] % 2 = 0) {
                localMemory[localMemIdx] :=
                    localMemory[localMemIdx]
                  + globalMemory[tileIdx + globalOffset];
            };

        (* long_work(GLOBAL_MEMORY_ACCESS) *)
        pex_long_work:
            if (globalTime >= startTime + GLOBAL_MEMORY_ACCESS) {
                startTime := curTime;
                tileIdx := tileIdx + 1;
                goto pex_tile_loop;
            };

        (* work_step *)
        pex_work_step:
            curTime := globalTime;
            nRunningPEs := nRunningPEs + 1;

        pex_work_await:
            await globalTime >= curTime + 1;
            goto pex_long_work;

        (* ---- barrier call ---- *)
        pex_call_barrier:
            call Barrier();

        (* ---- after barrier: decide EOI / NEOI ---- *)
        pex_decide:
            if ((wgIter + 1) >= (workGroupSize \div nWorkingPEsPerUnit)) {
                Send3(<<EOI, pId>>, pex_u, dId, uId, pId);
                goto pex_outer;
            } else {
                Send3(<<NEOI, pId>>, pex_u, dId, uId, pId);
                goto pex_inner;
            };

        (* ======== STOP PHASE: final reduction ======== *)
        pex_stop_phase:
            if (pId = 0) {
                tileIdx := 1;
                goto pex_reduce_loop;
            } else {
                goto pex_done;
            };

        pex_reduce_loop:
            if (tileIdx >= nWorkingPEsPerUnit) {
                goto pex_final_write;
            };

        pex_reduce_sum:
            localMemory[uId * nWorkingPEsPerUnit] :=
                localMemory[uId * nWorkingPEsPerUnit]
              + localMemory[tileIdx + uId * nWorkingPEsPerUnit];
            globalTime := globalTime + 1;
            tileIdx := tileIdx + 1;
            goto pex_reduce_loop;

        pex_final_write:
            globalTime := globalTime + GLOBAL_MEMORY_ACCESS;
            aoutput := aoutput + localMemory[uId * nWorkingPEsPerUnit];

        pex_final_write2:
            globalTime := globalTime + GLOBAL_MEMORY_ACCESS;
            final := TRUE;

        pex_done:
            skip;
    }

    (* ========== UNIT PROCESS ========== *)

    fair process (Unit \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) })
    variable dId = 0, uId = 0, pId = 0,
             wgIter = 0, wgId = 0, nProc = 0,
             msgFromDev = <<>>, msgFromPEX = <<>>;
    {
        u0:
            dId := self[3];
            uId := self[4];

        u_loop:
            Rcv2(msgFromDev, dev_u, dId, uId);

        u1:
            if (msgFromDev[1] = GO) {
                wgIter := 0;
                wgId   := msgFromDev[2];
                pId    := 0;
                goto u_send_gowg;
            } else if (msgFromDev[1] = STOP) {
                pId := 0;
                goto u_send_stop;
            } else {
                goto u_done;
            };

        (* ---- send GOWG + first GO to each PE ---- *)
        u_send_gowg:
            if (pId >= nWorkingPEsPerUnit) {
                goto u_after_send;
            };

        u_send_gowg_msg:
            Send3(<<GOWG, wgId>>, u_pex, dId, uId, pId);

        u_send_go_msg:
            Send3(<<GO, wgIter>>, u_pex, dId, uId, pId);
            pId := pId + 1;
            goto u_send_gowg;

        (* ---- branch: WG fits in PEs or not ---- *)
        u_after_send:
            if (workGroupSize <= nWorkingPEsPerUnit) {
                pId := 0;
                goto u_recv_eoi;
            } else {
                wgIter := 1;
                nProc  := 0;
                pId    := 0;
                goto u_overflow_loop;
            };

        (* ---- simple case: read EOI from all PEs ---- *)
        u_recv_eoi:
            if (pId >= nWorkingPEsPerUnit) {
                goto u_send_done;
            };

        u_recv_eoi_one:
            Rcv3(msgFromPEX, pex_u, dId, uId, pId);
            pId := pId + 1;
            goto u_recv_eoi;

        (* ---- overflow case: workGroupSize > nWorkingPEsPerUnit ---- *)
        u_overflow_loop:
            if (pId >= workGroupSize - nWorkingPEsPerUnit) {
                goto u_overflow_final;
            };

        u_overflow_recv:
            with (peIdx = pId % nWorkingPEsPerUnit) {
                Rcv3(msgFromPEX, pex_u, dId, uId, peIdx);
            };

        u_overflow_check:
            if (msgFromPEX[1] = NEOI) {
                with (peIdx = pId % nWorkingPEsPerUnit) {
                    Send3(<<GO, wgIter>>, u_pex, dId, uId, peIdx);
                };
            };
            nProc := nProc + 1;

        u_overflow_iter:
            if (nProc >= nWorkingPEsPerUnit) {
                wgIter := wgIter + 1;
                nProc := 0;
            };
            pId := pId + 1;
            goto u_overflow_loop;

        u_overflow_final:
            pId := 0;

        u_overflow_final_loop:
            if (pId >= nWorkingPEsPerUnit) {
                goto u_send_done;
            };

        u_overflow_final_recv:
            Rcv3(msgFromPEX, pex_u, dId, uId, pId);
            pId := pId + 1;
            goto u_overflow_final_loop;

        u_send_done:
            Send2(<<DONE, uId>>, u_dev, dId, uId);
            goto u_loop;

        (* ---- STOP: forward to all PEs ---- *)
        u_send_stop:
            if (pId >= nWorkingPEsPerUnit) {
                goto u_done;
            };

        u_send_stop_one:
            Send3(<<STOP, 0>>, u_pex, dId, uId, pId);
            pId := pId + 1;
            goto u_send_stop;

        u_done:
            skip;
    }

    fair process (Device \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) })
    variable dId = 0, uId = 0, wgId = 0, msgFromHost = "", msgFromUnit = <<>>;
    {
        d0:       dId := self[3];
        d_loop:
                  Rcv1(msgFromHost, hst_d);
        d1:       if (msgFromHost = GO) {
        d2:           uId := 0;
        d_send:       while (uId < nWorkingUnitsPerDevice) {
                        Send2(<<GO, uId>>, dev_u, dId, uId);
                        uId := uId + 1;
                      };
        d3:           if (nWorkGroups <= nWorkingUnitsPerDevice) {
        d4:              uId := 0;
        d_rcv:           while (uId < nWorkGroups) {
                             Rcv2(msgFromUnit, u_dev, dId, uId);
                             allWorkingPEs := allWorkingPEs - nWorkingPEsPerUnit;
                             uId := uId + 1;
                         };
                      } else {
        d6:              wgId := uId;
        d7:              uId := 0;
        d_wait1:         while (uId < nWorkGroups - nWorkingUnitsPerDevice) {
        d_choose1:         with (u \in 0..(nWorkingUnitsPerDevice-1)) {
                               await u_dev[dId][u] # <<>>;
                               msgFromUnit := Head(u_dev[dId][u]);
                               u_dev[dId][u] := Tail(u_dev[dId][u]);
                               Send2(<<GO, wgId>>, dev_u, dId, u);
                           };
                           wgId := wgId + 1;
                           uId := uId + 1;
                         };
        d8:              uId := 0;
        d_wait2:         while (uId < nWorkingUnitsPerDevice) {
        d_choose2:         with (u \in 0..(nWorkingUnitsPerDevice-1)) {
                               await u_dev[dId][u] # <<>>;
                               msgFromUnit := Head(u_dev[dId][u]);
                               u_dev[dId][u] := Tail(u_dev[dId][u]);
                           };
                           allWorkingPEs := allWorkingPEs - nWorkingPEsPerUnit;
                           uId := uId + 1;
                         };
                      };
        d_write:       Send1(DONE, d_hst);
                       goto d_loop;
                  } else if (msgFromHost = STOP) {
        d10:           uId := 0;
        d_stop:        while (uId < nWorkingUnitsPerDevice) {
                           Send2(<<STOP, uId>>, dev_u, dId, uId);
                           uId := uId + 1;
                       };
                       goto d_done;
                   };
        d_done:    skip;
    }

    fair process (Host = <<2,0,0,0>>)
    variable msgFromDevice = "";
    {
        h0: await hostFlag = TRUE;
        h1: final := FALSE;
        h2: Send1(GO, hst_d);
        h_loop:
            Rcv1(msgFromDevice, d_hst);
        h3: if (msgFromDevice = DONE) {
        h4:    Send1(STOP, hst_d);
        h5:    goto h7;
            };
        h6: goto h_loop;
        h7: skip;
    }

    fair process (Clock = <<0,0,0,0>>)
    {
        c0: await clockFlag = TRUE;
        loop_cl: while (~final) {
        c1:         if ((nRunningPEs = (allWorkingPEs - nWaitingPEs))
                       /\ (allWorkingPEs # 0)
                       /\ (nWaitingPEs # allWorkingPEs)) {
        c2:            nRunningPEs := 0;
                       globalTime := globalTime + 1;
                    };
                 };
    }

    fair process (Main = <<1,0,0,0>>)
    variable i_1 = 0, j_1 = 0;
    {
        m_loop_gm: while (i_1 < INPUT_DATA_SIZE) {
        m1:           globalMemory[i_1] := INPUT_DATA_SIZE - i_1 - 1;
        m2:           i_1 := i_1 + 1;
                   };
        m_loop_lm: while (j_1 < LOCAL_MEMORY_SIZE * UNITS_PER_DEVICE) {
        m3:           localMemory[j_1] := 0;
        m4:           j_1 := j_1 + 1;
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
        m19:       allWorkingPEs := nWorkingDevices * nWorkingUnitsPerDevice * nWorkingPEsPerUnit;
        m20:       hostFlag := TRUE;
                   clockFlag := TRUE;
    }
}
*)
\* BEGIN TRANSLATION (chksum(pcal) = "d12ac0ab" /\ chksum(tla) = "a698a6a6")
\* Process variable dId of process PEX at line 128 col 14 changed to dId_
\* Process variable uId of process PEX at line 128 col 23 changed to uId_
\* Process variable pId of process PEX at line 128 col 32 changed to pId_
\* Process variable wgIter of process PEX at line 132 col 14 changed to wgIter_
\* Process variable wgId of process PEX at line 132 col 26 changed to wgId_
\* Process variable dId of process Unit at line 278 col 14 changed to dId_U
\* Process variable uId of process Unit at line 278 col 23 changed to uId_U
\* Process variable wgId of process Unit at line 279 col 26 changed to wgId_U
VARIABLES GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, flag, final, 
          hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, globalTime, Tmin, 
          globalMemory, localMemory, workGroupSize, nWorkGroups, tileSize, 
          nWorkingDevices, nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
          allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, nWaitingPEsOut, 
          pc, stack, bt, dId_, uId_, pId_, msg1, startTime, curTime, 
          globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, localId, t, 
          dId_U, uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, msgFromPEX, 
          dId, uId, wgId, msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1

vars == << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, flag, final, 
           hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, globalTime, Tmin, 
           globalMemory, localMemory, workGroupSize, nWorkGroups, tileSize, 
           nWorkingDevices, nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
           allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, nWaitingPEsOut, 
           pc, stack, bt, dId_, uId_, pId_, msg1, startTime, curTime, 
           globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, localId, t, 
           dId_U, uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, msgFromPEX, 
           dId, uId, wgId, msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1
        >>

ProcSet == ({ <<5,d,u,p>> : d \in 0..(DEVICES-1),
                             u \in 0..(UNITS_PER_DEVICE-1),
                             p \in 0..(PES_PER_UNIT-1) }) \cup ({ <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                                                                 u \in 0..(UNITS_PER_DEVICE-1) }) \cup ({ <<3,0,d,0>> : d \in 0..(DEVICES-1) }) \cup {<<2,0,0,0>>} \cup {<<0,0,0,0>>} \cup {<<1,0,0,0>>}

Init == (* Global variables *)
        /\ GO = "GO"
        /\ STOP = "STOP"
        /\ DONE = "DONE"
        /\ GOWG = "GOWG"
        /\ EOI = "EOI"
        /\ NEOI = "NEOI"
        /\ hostFlag = FALSE
        /\ clockFlag = FALSE
        /\ flag = FALSE
        /\ final = FALSE
        /\ hst_d = <<>>
        /\ d_hst = <<>>
        /\ dev_u = [d \in 0..(DEVICES-1) |-> [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ u_dev = [d \in 0..(DEVICES-1) |-> [u \in 0..(UNITS_PER_DEVICE-1) |-> <<>>]]
        /\ u_pex = [d \in 0..(DEVICES-1) |->
                   [u \in 0..(UNITS_PER_DEVICE-1) |->
                   [p \in 0..(PES_PER_UNIT-1) |-> <<>>]]]
        /\ pex_u = [d \in 0..(DEVICES-1) |->
                   [u \in 0..(UNITS_PER_DEVICE-1) |->
                   [p \in 0..(PES_PER_UNIT-1) |-> <<>>]]]
        /\ globalTime = 0
        /\ Tmin = 24
        /\ globalMemory = [i \in 0..(INPUT_DATA_SIZE - 1) |-> 0]
        /\ localMemory = [j \in 0..(LOCAL_MEMORY_SIZE * UNITS_PER_DEVICE - 1) |-> 0]
        /\ workGroupSize = 0
        /\ nWorkGroups = 0
        /\ tileSize = 0
        /\ nWorkingDevices = 0
        /\ nWorkingUnitsPerDevice = 0
        /\ nWorkingPEsPerUnit = 0
        /\ allWorkingPEs = 0
        /\ nRunningPEs = 0
        /\ aoutput = 0
        /\ nWaitingPEs = 0
        /\ nWaitingPEsOut = 0
        (* Procedure Barrier *)
        /\ bt = [ self \in ProcSet |-> 0]
        (* Process PEX *)
        /\ dId_ = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1),
                                             p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ uId_ = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1),
                                             p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ pId_ = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1),
                                             p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ msg1 = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                             u \in 0..(UNITS_PER_DEVICE-1),
                                             p \in 0..(PES_PER_UNIT-1) } |-> <<>>]
        /\ startTime = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                  u \in 0..(UNITS_PER_DEVICE-1),
                                                  p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ curTime = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1),
                                                p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ globalOffset = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                     u \in 0..(UNITS_PER_DEVICE-1),
                                                     p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ tileIdx = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1),
                                                p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ localMemIdx = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                    u \in 0..(UNITS_PER_DEVICE-1),
                                                    p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ wgIter_ = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1),
                                                p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ wgId_ = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1),
                                              p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ localId = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                u \in 0..(UNITS_PER_DEVICE-1),
                                                p \in 0..(PES_PER_UNIT-1) } |-> 0]
        /\ t = [self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                          u \in 0..(UNITS_PER_DEVICE-1),
                                          p \in 0..(PES_PER_UNIT-1) } |-> 0]
        (* Process Unit *)
        /\ dId_U = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ uId_U = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ pId = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ wgIter = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                               u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ wgId_U = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                               u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ nProc = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                              u \in 0..(UNITS_PER_DEVICE-1) } |-> 0]
        /\ msgFromDev = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                                   u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        /\ msgFromPEX = [self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                                   u \in 0..(UNITS_PER_DEVICE-1) } |-> <<>>]
        (* Process Device *)
        /\ dId = [self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ uId = [self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ wgId = [self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } |-> 0]
        /\ msgFromHost = [self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } |-> ""]
        /\ msgFromUnit = [self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } |-> <<>>]
        (* Process Host *)
        /\ msgFromDevice = ""
        (* Process Main *)
        /\ i_1 = 0
        /\ j_1 = 0
        /\ stack = [self \in ProcSet |-> << >>]
        /\ pc = [self \in ProcSet |-> CASE self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                                                     u \in 0..(UNITS_PER_DEVICE-1),
                                                                     p \in 0..(PES_PER_UNIT-1) } -> "p_init"
                                        [] self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                                                     u \in 0..(UNITS_PER_DEVICE-1) } -> "u0"
                                        [] self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } -> "d0"
                                        [] self = <<2,0,0,0>> -> "h0"
                                        [] self = <<0,0,0,0>> -> "c0"
                                        [] self = <<1,0,0,0>> -> "m_loop_gm"]

bar_faa1(self) == /\ pc[self] = "bar_faa1"
                  /\ bt' = [bt EXCEPT ![self] = nWaitingPEs]
                  /\ nWaitingPEs' = nWaitingPEs + 1
                  /\ pc' = [pc EXCEPT ![self] = "bar_wait1"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                  clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                  u_dev, u_pex, pex_u, globalTime, Tmin, 
                                  globalMemory, localMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingPEs, nRunningPEs, aoutput, 
                                  nWaitingPEsOut, stack, dId_, uId_, pId_, 
                                  msg1, startTime, curTime, globalOffset, 
                                  tileIdx, localMemIdx, wgIter_, wgId_, 
                                  localId, t, dId_U, uId_U, pId, wgIter, 
                                  wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                  uId, wgId, msgFromHost, msgFromUnit, 
                                  msgFromDevice, i_1, j_1 >>

bar_wait1(self) == /\ pc[self] = "bar_wait1"
                   /\ IF bt[self] < (nWorkingPEsPerUnit * nWorkingUnitsPerDevice - 1)
                         THEN /\ nWaitingPEs = 0
                              /\ UNCHANGED nWaitingPEs
                         ELSE /\ nWaitingPEs' = 0
                   /\ pc' = [pc EXCEPT ![self] = "bar_faa2"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                   u_dev, u_pex, pex_u, globalTime, Tmin, 
                                   globalMemory, localMemory, workGroupSize, 
                                   nWorkGroups, tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   allWorkingPEs, nRunningPEs, aoutput, 
                                   nWaitingPEsOut, stack, bt, dId_, uId_, pId_, 
                                   msg1, startTime, curTime, globalOffset, 
                                   tileIdx, localMemIdx, wgIter_, wgId_, 
                                   localId, t, dId_U, uId_U, pId, wgIter, 
                                   wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                   uId, wgId, msgFromHost, msgFromUnit, 
                                   msgFromDevice, i_1, j_1 >>

bar_faa2(self) == /\ pc[self] = "bar_faa2"
                  /\ bt' = [bt EXCEPT ![self] = nWaitingPEsOut]
                  /\ nWaitingPEsOut' = nWaitingPEsOut + 1
                  /\ pc' = [pc EXCEPT ![self] = "bar_wait2"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                  clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                  u_dev, u_pex, pex_u, globalTime, Tmin, 
                                  globalMemory, localMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingPEs, nRunningPEs, aoutput, 
                                  nWaitingPEs, stack, dId_, uId_, pId_, msg1, 
                                  startTime, curTime, globalOffset, tileIdx, 
                                  localMemIdx, wgIter_, wgId_, localId, t, 
                                  dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                                  msgFromDev, msgFromPEX, dId, uId, wgId, 
                                  msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                                  j_1 >>

bar_wait2(self) == /\ pc[self] = "bar_wait2"
                   /\ IF bt[self] < (nWorkingPEsPerUnit * nWorkingUnitsPerDevice - 1)
                         THEN /\ nWaitingPEsOut = 0
                              /\ UNCHANGED nWaitingPEsOut
                         ELSE /\ nWaitingPEsOut' = 0
                   /\ pc' = [pc EXCEPT ![self] = "bar_ret"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                   u_dev, u_pex, pex_u, globalTime, Tmin, 
                                   globalMemory, localMemory, workGroupSize, 
                                   nWorkGroups, tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   allWorkingPEs, nRunningPEs, aoutput, 
                                   nWaitingPEs, stack, bt, dId_, uId_, pId_, 
                                   msg1, startTime, curTime, globalOffset, 
                                   tileIdx, localMemIdx, wgIter_, wgId_, 
                                   localId, t, dId_U, uId_U, pId, wgIter, 
                                   wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                   uId, wgId, msgFromHost, msgFromUnit, 
                                   msgFromDevice, i_1, j_1 >>

bar_ret(self) == /\ pc[self] = "bar_ret"
                 /\ pc' = [pc EXCEPT ![self] = Head(stack[self]).pc]
                 /\ bt' = [bt EXCEPT ![self] = Head(stack[self]).bt]
                 /\ stack' = [stack EXCEPT ![self] = Tail(stack[self])]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                 clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                 u_dev, u_pex, pex_u, globalTime, Tmin, 
                                 globalMemory, localMemory, workGroupSize, 
                                 nWorkGroups, tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingPEs, nRunningPEs, aoutput, 
                                 nWaitingPEs, nWaitingPEsOut, dId_, uId_, pId_, 
                                 msg1, startTime, curTime, globalOffset, 
                                 tileIdx, localMemIdx, wgIter_, wgId_, localId, 
                                 t, dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                                 msgFromDev, msgFromPEX, dId, uId, wgId, 
                                 msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                                 j_1 >>

Barrier(self) == bar_faa1(self) \/ bar_wait1(self) \/ bar_faa2(self)
                    \/ bar_wait2(self) \/ bar_ret(self)

p_init(self) == /\ pc[self] = "p_init"
                /\    self[2] < nWorkingDevices
                   /\ self[3] < nWorkingUnitsPerDevice
                   /\ self[4] < nWorkingPEsPerUnit
                /\ dId_' = [dId_ EXCEPT ![self] = self[2]]
                /\ uId_' = [uId_ EXCEPT ![self] = self[3]]
                /\ pId_' = [pId_ EXCEPT ![self] = self[4]]
                /\ pc' = [pc EXCEPT ![self] = "p_lmem"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                u_dev, u_pex, pex_u, globalTime, Tmin, 
                                globalMemory, localMemory, workGroupSize, 
                                nWorkGroups, tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, msg1, 
                                startTime, curTime, globalOffset, tileIdx, 
                                localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                                uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                                msgFromPEX, dId, uId, wgId, msgFromHost, 
                                msgFromUnit, msgFromDevice, i_1, j_1 >>

p_lmem(self) == /\ pc[self] = "p_lmem"
                /\ localMemIdx' = [localMemIdx EXCEPT ![self] = pId_[self] + uId_[self] * nWorkingPEsPerUnit]
                /\ pc' = [pc EXCEPT ![self] = "pex_outer"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                u_dev, u_pex, pex_u, globalTime, Tmin, 
                                globalMemory, localMemory, workGroupSize, 
                                nWorkGroups, tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, wgIter_, wgId_, localId, 
                                t, dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                                msgFromDev, msgFromPEX, dId, uId, wgId, 
                                msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                                j_1 >>

pex_outer(self) == /\ pc[self] = "pex_outer"
                   /\ u_pex[dId_[self]][uId_[self]][pId_[self]] # <<>>
                   /\ msg1' = [msg1 EXCEPT ![self] = Head(u_pex[dId_[self]][uId_[self]][pId_[self]])]
                   /\ u_pex' = [u_pex EXCEPT ![dId_[self]][uId_[self]][pId_[self]] = Tail(u_pex[dId_[self]][uId_[self]][pId_[self]])]
                   /\ pc' = [pc EXCEPT ![self] = "pex_outer_check"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                   u_dev, pex_u, globalTime, Tmin, 
                                   globalMemory, localMemory, workGroupSize, 
                                   nWorkGroups, tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   allWorkingPEs, nRunningPEs, aoutput, 
                                   nWaitingPEs, nWaitingPEsOut, stack, bt, 
                                   dId_, uId_, pId_, startTime, curTime, 
                                   globalOffset, tileIdx, localMemIdx, wgIter_, 
                                   wgId_, localId, t, dId_U, uId_U, pId, 
                                   wgIter, wgId_U, nProc, msgFromDev, 
                                   msgFromPEX, dId, uId, wgId, msgFromHost, 
                                   msgFromUnit, msgFromDevice, i_1, j_1 >>

pex_outer_check(self) == /\ pc[self] = "pex_outer_check"
                         /\ IF msg1[self][1] = STOP
                               THEN /\ pc' = [pc EXCEPT ![self] = "pex_stop_phase"]
                                    /\ wgId_' = wgId_
                               ELSE /\ IF msg1[self][1] = GOWG
                                          THEN /\ wgId_' = [wgId_ EXCEPT ![self] = msg1[self][2]]
                                               /\ pc' = [pc EXCEPT ![self] = "pex_inner"]
                                          ELSE /\ pc' = [pc EXCEPT ![self] = "pex_outer"]
                                               /\ wgId_' = wgId_
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, localId, t, dId_U, uId_U, 
                                         pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

pex_inner(self) == /\ pc[self] = "pex_inner"
                   /\ u_pex[dId_[self]][uId_[self]][pId_[self]] # <<>>
                   /\ msg1' = [msg1 EXCEPT ![self] = Head(u_pex[dId_[self]][uId_[self]][pId_[self]])]
                   /\ u_pex' = [u_pex EXCEPT ![dId_[self]][uId_[self]][pId_[self]] = Tail(u_pex[dId_[self]][uId_[self]][pId_[self]])]
                   /\ pc' = [pc EXCEPT ![self] = "pex_inner_check"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                   u_dev, pex_u, globalTime, Tmin, 
                                   globalMemory, localMemory, workGroupSize, 
                                   nWorkGroups, tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   allWorkingPEs, nRunningPEs, aoutput, 
                                   nWaitingPEs, nWaitingPEsOut, stack, bt, 
                                   dId_, uId_, pId_, startTime, curTime, 
                                   globalOffset, tileIdx, localMemIdx, wgIter_, 
                                   wgId_, localId, t, dId_U, uId_U, pId, 
                                   wgIter, wgId_U, nProc, msgFromDev, 
                                   msgFromPEX, dId, uId, wgId, msgFromHost, 
                                   msgFromUnit, msgFromDevice, i_1, j_1 >>

pex_inner_check(self) == /\ pc[self] = "pex_inner_check"
                         /\ IF msg1[self][1] = GO
                               THEN /\ wgIter_' = [wgIter_ EXCEPT ![self] = msg1[self][2]]
                                    /\ pc' = [pc EXCEPT ![self] = "pex_compute"]
                               ELSE /\ IF msg1[self][1] = STOP
                                          THEN /\ pc' = [pc EXCEPT ![self] = "pex_stop_phase"]
                                          ELSE /\ pc' = [pc EXCEPT ![self] = "pex_inner"]
                                    /\ UNCHANGED wgIter_
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgId_, localId, t, dId_U, uId_U, pId, 
                                         wgIter, wgId_U, nProc, msgFromDev, 
                                         msgFromPEX, dId, uId, wgId, 
                                         msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

pex_compute(self) == /\ pc[self] = "pex_compute"
                     /\ startTime' = [startTime EXCEPT ![self] = globalTime]
                     /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                     /\ pc' = [pc EXCEPT ![self] = "pex_local_id"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                     clockFlag, flag, final, hst_d, d_hst, 
                                     dev_u, u_dev, u_pex, pex_u, globalTime, 
                                     Tmin, globalMemory, localMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingPEs, 
                                     nRunningPEs, aoutput, nWaitingPEs, 
                                     nWaitingPEsOut, stack, bt, dId_, uId_, 
                                     pId_, msg1, globalOffset, tileIdx, 
                                     localMemIdx, wgIter_, wgId_, localId, t, 
                                     dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                                     msgFromDev, msgFromPEX, dId, uId, wgId, 
                                     msgFromHost, msgFromUnit, msgFromDevice, 
                                     i_1, j_1 >>

pex_local_id(self) == /\ pc[self] = "pex_local_id"
                      /\ IF workGroupSize > nWorkingPEsPerUnit
                            THEN /\ localId' = [localId EXCEPT ![self] = pId_[self] + wgIter_[self] * nWorkingPEsPerUnit]
                            ELSE /\ localId' = [localId EXCEPT ![self] = pId_[self]]
                      /\ pc' = [pc EXCEPT ![self] = "pex_offset"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                      hostFlag, clockFlag, flag, final, hst_d, 
                                      d_hst, dev_u, u_dev, u_pex, pex_u, 
                                      globalTime, Tmin, globalMemory, 
                                      localMemory, workGroupSize, nWorkGroups, 
                                      tileSize, nWorkingDevices, 
                                      nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingPEs, 
                                      nRunningPEs, aoutput, nWaitingPEs, 
                                      nWaitingPEsOut, stack, bt, dId_, uId_, 
                                      pId_, msg1, startTime, curTime, 
                                      globalOffset, tileIdx, localMemIdx, 
                                      wgIter_, wgId_, t, dId_U, uId_U, pId, 
                                      wgIter, wgId_U, nProc, msgFromDev, 
                                      msgFromPEX, dId, uId, wgId, msgFromHost, 
                                      msgFromUnit, msgFromDevice, i_1, j_1 >>

pex_offset(self) == /\ pc[self] = "pex_offset"
                    /\ globalOffset' = [globalOffset EXCEPT ![self] = tileSize * (wgId_[self] * workGroupSize + localId[self])]
                    /\ tileIdx' = [tileIdx EXCEPT ![self] = 0]
                    /\ pc' = [pc EXCEPT ![self] = "pex_tile_loop"]
                    /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                    clockFlag, flag, final, hst_d, d_hst, 
                                    dev_u, u_dev, u_pex, pex_u, globalTime, 
                                    Tmin, globalMemory, localMemory, 
                                    workGroupSize, nWorkGroups, tileSize, 
                                    nWorkingDevices, nWorkingUnitsPerDevice, 
                                    nWorkingPEsPerUnit, allWorkingPEs, 
                                    nRunningPEs, aoutput, nWaitingPEs, 
                                    nWaitingPEsOut, stack, bt, dId_, uId_, 
                                    pId_, msg1, startTime, curTime, 
                                    localMemIdx, wgIter_, wgId_, localId, t, 
                                    dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                                    msgFromDev, msgFromPEX, dId, uId, wgId, 
                                    msgFromHost, msgFromUnit, msgFromDevice, 
                                    i_1, j_1 >>

pex_tile_loop(self) == /\ pc[self] = "pex_tile_loop"
                       /\ IF tileIdx[self] >= tileSize
                             THEN /\ pc' = [pc EXCEPT ![self] = "pex_call_barrier"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "pex_tile_bound"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                       hostFlag, clockFlag, flag, final, hst_d, 
                                       d_hst, dev_u, u_dev, u_pex, pex_u, 
                                       globalTime, Tmin, globalMemory, 
                                       localMemory, workGroupSize, nWorkGroups, 
                                       tileSize, nWorkingDevices, 
                                       nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingPEs, 
                                       nRunningPEs, aoutput, nWaitingPEs, 
                                       nWaitingPEsOut, stack, bt, dId_, uId_, 
                                       pId_, msg1, startTime, curTime, 
                                       globalOffset, tileIdx, localMemIdx, 
                                       wgIter_, wgId_, localId, t, dId_U, 
                                       uId_U, pId, wgIter, wgId_U, nProc, 
                                       msgFromDev, msgFromPEX, dId, uId, wgId, 
                                       msgFromHost, msgFromUnit, msgFromDevice, 
                                       i_1, j_1 >>

pex_tile_bound(self) == /\ pc[self] = "pex_tile_bound"
                        /\ IF tileIdx[self] + globalOffset[self] >= INPUT_DATA_SIZE
                              THEN /\ pc' = [pc EXCEPT ![self] = "pex_call_barrier"]
                              ELSE /\ pc' = [pc EXCEPT ![self] = "pex_even_sum"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                        hostFlag, clockFlag, flag, final, 
                                        hst_d, d_hst, dev_u, u_dev, u_pex, 
                                        pex_u, globalTime, Tmin, globalMemory, 
                                        localMemory, workGroupSize, 
                                        nWorkGroups, tileSize, nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingPEs, 
                                        nRunningPEs, aoutput, nWaitingPEs, 
                                        nWaitingPEsOut, stack, bt, dId_, uId_, 
                                        pId_, msg1, startTime, curTime, 
                                        globalOffset, tileIdx, localMemIdx, 
                                        wgIter_, wgId_, localId, t, dId_U, 
                                        uId_U, pId, wgIter, wgId_U, nProc, 
                                        msgFromDev, msgFromPEX, dId, uId, wgId, 
                                        msgFromHost, msgFromUnit, 
                                        msgFromDevice, i_1, j_1 >>

pex_even_sum(self) == /\ pc[self] = "pex_even_sum"
                      /\ IF globalMemory[tileIdx[self] + globalOffset[self]] % 2 = 0
                            THEN /\ localMemory' = [localMemory EXCEPT ![localMemIdx[self]] =   localMemory[localMemIdx[self]]
                                                                                              + globalMemory[tileIdx[self] + globalOffset[self]]]
                            ELSE /\ TRUE
                                 /\ UNCHANGED localMemory
                      /\ pc' = [pc EXCEPT ![self] = "pex_long_work"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                      hostFlag, clockFlag, flag, final, hst_d, 
                                      d_hst, dev_u, u_dev, u_pex, pex_u, 
                                      globalTime, Tmin, globalMemory, 
                                      workGroupSize, nWorkGroups, tileSize, 
                                      nWorkingDevices, nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingPEs, 
                                      nRunningPEs, aoutput, nWaitingPEs, 
                                      nWaitingPEsOut, stack, bt, dId_, uId_, 
                                      pId_, msg1, startTime, curTime, 
                                      globalOffset, tileIdx, localMemIdx, 
                                      wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                      pId, wgIter, wgId_U, nProc, msgFromDev, 
                                      msgFromPEX, dId, uId, wgId, msgFromHost, 
                                      msgFromUnit, msgFromDevice, i_1, j_1 >>

pex_long_work(self) == /\ pc[self] = "pex_long_work"
                       /\ IF globalTime >= startTime[self] + GLOBAL_MEMORY_ACCESS
                             THEN /\ startTime' = [startTime EXCEPT ![self] = curTime[self]]
                                  /\ tileIdx' = [tileIdx EXCEPT ![self] = tileIdx[self] + 1]
                                  /\ pc' = [pc EXCEPT ![self] = "pex_tile_loop"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "pex_work_step"]
                                  /\ UNCHANGED << startTime, tileIdx >>
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                       hostFlag, clockFlag, flag, final, hst_d, 
                                       d_hst, dev_u, u_dev, u_pex, pex_u, 
                                       globalTime, Tmin, globalMemory, 
                                       localMemory, workGroupSize, nWorkGroups, 
                                       tileSize, nWorkingDevices, 
                                       nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingPEs, 
                                       nRunningPEs, aoutput, nWaitingPEs, 
                                       nWaitingPEsOut, stack, bt, dId_, uId_, 
                                       pId_, msg1, curTime, globalOffset, 
                                       localMemIdx, wgIter_, wgId_, localId, t, 
                                       dId_U, uId_U, pId, wgIter, wgId_U, 
                                       nProc, msgFromDev, msgFromPEX, dId, uId, 
                                       wgId, msgFromHost, msgFromUnit, 
                                       msgFromDevice, i_1, j_1 >>

pex_work_step(self) == /\ pc[self] = "pex_work_step"
                       /\ curTime' = [curTime EXCEPT ![self] = globalTime]
                       /\ nRunningPEs' = nRunningPEs + 1
                       /\ pc' = [pc EXCEPT ![self] = "pex_work_await"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                       hostFlag, clockFlag, flag, final, hst_d, 
                                       d_hst, dev_u, u_dev, u_pex, pex_u, 
                                       globalTime, Tmin, globalMemory, 
                                       localMemory, workGroupSize, nWorkGroups, 
                                       tileSize, nWorkingDevices, 
                                       nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingPEs, 
                                       aoutput, nWaitingPEs, nWaitingPEsOut, 
                                       stack, bt, dId_, uId_, pId_, msg1, 
                                       startTime, globalOffset, tileIdx, 
                                       localMemIdx, wgIter_, wgId_, localId, t, 
                                       dId_U, uId_U, pId, wgIter, wgId_U, 
                                       nProc, msgFromDev, msgFromPEX, dId, uId, 
                                       wgId, msgFromHost, msgFromUnit, 
                                       msgFromDevice, i_1, j_1 >>

pex_work_await(self) == /\ pc[self] = "pex_work_await"
                        /\ globalTime >= curTime[self] + 1
                        /\ pc' = [pc EXCEPT ![self] = "pex_long_work"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                        hostFlag, clockFlag, flag, final, 
                                        hst_d, d_hst, dev_u, u_dev, u_pex, 
                                        pex_u, globalTime, Tmin, globalMemory, 
                                        localMemory, workGroupSize, 
                                        nWorkGroups, tileSize, nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingPEs, 
                                        nRunningPEs, aoutput, nWaitingPEs, 
                                        nWaitingPEsOut, stack, bt, dId_, uId_, 
                                        pId_, msg1, startTime, curTime, 
                                        globalOffset, tileIdx, localMemIdx, 
                                        wgIter_, wgId_, localId, t, dId_U, 
                                        uId_U, pId, wgIter, wgId_U, nProc, 
                                        msgFromDev, msgFromPEX, dId, uId, wgId, 
                                        msgFromHost, msgFromUnit, 
                                        msgFromDevice, i_1, j_1 >>

pex_call_barrier(self) == /\ pc[self] = "pex_call_barrier"
                          /\ stack' = [stack EXCEPT ![self] = << [ procedure |->  "Barrier",
                                                                   pc        |->  "pex_decide",
                                                                   bt        |->  bt[self] ] >>
                                                               \o stack[self]]
                          /\ bt' = [bt EXCEPT ![self] = 0]
                          /\ pc' = [pc EXCEPT ![self] = "bar_faa1"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                          hostFlag, clockFlag, flag, final, 
                                          hst_d, d_hst, dev_u, u_dev, u_pex, 
                                          pex_u, globalTime, Tmin, 
                                          globalMemory, localMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingPEs, 
                                          nRunningPEs, aoutput, nWaitingPEs, 
                                          nWaitingPEsOut, dId_, uId_, pId_, 
                                          msg1, startTime, curTime, 
                                          globalOffset, tileIdx, localMemIdx, 
                                          wgIter_, wgId_, localId, t, dId_U, 
                                          uId_U, pId, wgIter, wgId_U, nProc, 
                                          msgFromDev, msgFromPEX, dId, uId, 
                                          wgId, msgFromHost, msgFromUnit, 
                                          msgFromDevice, i_1, j_1 >>

pex_decide(self) == /\ pc[self] = "pex_decide"
                    /\ IF (wgIter_[self] + 1) >= (workGroupSize \div nWorkingPEsPerUnit)
                          THEN /\ pex_u' = [pex_u EXCEPT ![dId_[self]][uId_[self]][pId_[self]] = Append(pex_u[dId_[self]][uId_[self]][pId_[self]], (<<EOI, pId_[self]>>))]
                               /\ pc' = [pc EXCEPT ![self] = "pex_outer"]
                          ELSE /\ pex_u' = [pex_u EXCEPT ![dId_[self]][uId_[self]][pId_[self]] = Append(pex_u[dId_[self]][uId_[self]][pId_[self]], (<<NEOI, pId_[self]>>))]
                               /\ pc' = [pc EXCEPT ![self] = "pex_inner"]
                    /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                    clockFlag, flag, final, hst_d, d_hst, 
                                    dev_u, u_dev, u_pex, globalTime, Tmin, 
                                    globalMemory, localMemory, workGroupSize, 
                                    nWorkGroups, tileSize, nWorkingDevices, 
                                    nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                    allWorkingPEs, nRunningPEs, aoutput, 
                                    nWaitingPEs, nWaitingPEsOut, stack, bt, 
                                    dId_, uId_, pId_, msg1, startTime, curTime, 
                                    globalOffset, tileIdx, localMemIdx, 
                                    wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                    pId, wgIter, wgId_U, nProc, msgFromDev, 
                                    msgFromPEX, dId, uId, wgId, msgFromHost, 
                                    msgFromUnit, msgFromDevice, i_1, j_1 >>

pex_stop_phase(self) == /\ pc[self] = "pex_stop_phase"
                        /\ IF pId_[self] = 0
                              THEN /\ tileIdx' = [tileIdx EXCEPT ![self] = 1]
                                   /\ pc' = [pc EXCEPT ![self] = "pex_reduce_loop"]
                              ELSE /\ pc' = [pc EXCEPT ![self] = "pex_done"]
                                   /\ UNCHANGED tileIdx
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                        hostFlag, clockFlag, flag, final, 
                                        hst_d, d_hst, dev_u, u_dev, u_pex, 
                                        pex_u, globalTime, Tmin, globalMemory, 
                                        localMemory, workGroupSize, 
                                        nWorkGroups, tileSize, nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingPEs, 
                                        nRunningPEs, aoutput, nWaitingPEs, 
                                        nWaitingPEsOut, stack, bt, dId_, uId_, 
                                        pId_, msg1, startTime, curTime, 
                                        globalOffset, localMemIdx, wgIter_, 
                                        wgId_, localId, t, dId_U, uId_U, pId, 
                                        wgIter, wgId_U, nProc, msgFromDev, 
                                        msgFromPEX, dId, uId, wgId, 
                                        msgFromHost, msgFromUnit, 
                                        msgFromDevice, i_1, j_1 >>

pex_reduce_loop(self) == /\ pc[self] = "pex_reduce_loop"
                         /\ IF tileIdx[self] >= nWorkingPEsPerUnit
                               THEN /\ pc' = [pc EXCEPT ![self] = "pex_final_write"]
                               ELSE /\ pc' = [pc EXCEPT ![self] = "pex_reduce_sum"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

pex_reduce_sum(self) == /\ pc[self] = "pex_reduce_sum"
                        /\ localMemory' = [localMemory EXCEPT ![uId_[self] * nWorkingPEsPerUnit] =   localMemory[uId_[self] * nWorkingPEsPerUnit]
                                                                                                   + localMemory[tileIdx[self] + uId_[self] * nWorkingPEsPerUnit]]
                        /\ globalTime' = globalTime + 1
                        /\ tileIdx' = [tileIdx EXCEPT ![self] = tileIdx[self] + 1]
                        /\ pc' = [pc EXCEPT ![self] = "pex_reduce_loop"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                        hostFlag, clockFlag, flag, final, 
                                        hst_d, d_hst, dev_u, u_dev, u_pex, 
                                        pex_u, Tmin, globalMemory, 
                                        workGroupSize, nWorkGroups, tileSize, 
                                        nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingPEs, 
                                        nRunningPEs, aoutput, nWaitingPEs, 
                                        nWaitingPEsOut, stack, bt, dId_, uId_, 
                                        pId_, msg1, startTime, curTime, 
                                        globalOffset, localMemIdx, wgIter_, 
                                        wgId_, localId, t, dId_U, uId_U, pId, 
                                        wgIter, wgId_U, nProc, msgFromDev, 
                                        msgFromPEX, dId, uId, wgId, 
                                        msgFromHost, msgFromUnit, 
                                        msgFromDevice, i_1, j_1 >>

pex_final_write(self) == /\ pc[self] = "pex_final_write"
                         /\ globalTime' = globalTime + GLOBAL_MEMORY_ACCESS
                         /\ aoutput' = aoutput + localMemory[uId_[self] * nWorkingPEsPerUnit]
                         /\ pc' = [pc EXCEPT ![self] = "pex_final_write2"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

pex_final_write2(self) == /\ pc[self] = "pex_final_write2"
                          /\ globalTime' = globalTime + GLOBAL_MEMORY_ACCESS
                          /\ final' = TRUE
                          /\ pc' = [pc EXCEPT ![self] = "pex_done"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                          hostFlag, clockFlag, flag, hst_d, 
                                          d_hst, dev_u, u_dev, u_pex, pex_u, 
                                          Tmin, globalMemory, localMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingPEs, 
                                          nRunningPEs, aoutput, nWaitingPEs, 
                                          nWaitingPEsOut, stack, bt, dId_, 
                                          uId_, pId_, msg1, startTime, curTime, 
                                          globalOffset, tileIdx, localMemIdx, 
                                          wgIter_, wgId_, localId, t, dId_U, 
                                          uId_U, pId, wgIter, wgId_U, nProc, 
                                          msgFromDev, msgFromPEX, dId, uId, 
                                          wgId, msgFromHost, msgFromUnit, 
                                          msgFromDevice, i_1, j_1 >>

pex_done(self) == /\ pc[self] = "pex_done"
                  /\ TRUE
                  /\ pc' = [pc EXCEPT ![self] = "Done"]
                  /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                  clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                  u_dev, u_pex, pex_u, globalTime, Tmin, 
                                  globalMemory, localMemory, workGroupSize, 
                                  nWorkGroups, tileSize, nWorkingDevices, 
                                  nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                  allWorkingPEs, nRunningPEs, aoutput, 
                                  nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                  uId_, pId_, msg1, startTime, curTime, 
                                  globalOffset, tileIdx, localMemIdx, wgIter_, 
                                  wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                  wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                  uId, wgId, msgFromHost, msgFromUnit, 
                                  msgFromDevice, i_1, j_1 >>

PEX(self) == p_init(self) \/ p_lmem(self) \/ pex_outer(self)
                \/ pex_outer_check(self) \/ pex_inner(self)
                \/ pex_inner_check(self) \/ pex_compute(self)
                \/ pex_local_id(self) \/ pex_offset(self)
                \/ pex_tile_loop(self) \/ pex_tile_bound(self)
                \/ pex_even_sum(self) \/ pex_long_work(self)
                \/ pex_work_step(self) \/ pex_work_await(self)
                \/ pex_call_barrier(self) \/ pex_decide(self)
                \/ pex_stop_phase(self) \/ pex_reduce_loop(self)
                \/ pex_reduce_sum(self) \/ pex_final_write(self)
                \/ pex_final_write2(self) \/ pex_done(self)

u0(self) == /\ pc[self] = "u0"
            /\ dId_U' = [dId_U EXCEPT ![self] = self[3]]
            /\ uId_U' = [uId_U EXCEPT ![self] = self[4]]
            /\ pc' = [pc EXCEPT ![self] = "u_loop"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, pId, wgIter, wgId_U, nProc, msgFromDev, 
                            msgFromPEX, dId, uId, wgId, msgFromHost, 
                            msgFromUnit, msgFromDevice, i_1, j_1 >>

u_loop(self) == /\ pc[self] = "u_loop"
                /\ dev_u[dId_U[self]][uId_U[self]] # <<>>
                /\ msgFromDev' = [msgFromDev EXCEPT ![self] = Head(dev_u[dId_U[self]][uId_U[self]])]
                /\ dev_u' = [dev_u EXCEPT ![dId_U[self]][uId_U[self]] = Tail(dev_u[dId_U[self]][uId_U[self]])]
                /\ pc' = [pc EXCEPT ![self] = "u1"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, u_dev, 
                                u_pex, pex_u, globalTime, Tmin, globalMemory, 
                                localMemory, workGroupSize, nWorkGroups, 
                                tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromPEX, dId, uId, wgId, 
                                msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                                j_1 >>

u1(self) == /\ pc[self] = "u1"
            /\ IF msgFromDev[self][1] = GO
                  THEN /\ wgIter' = [wgIter EXCEPT ![self] = 0]
                       /\ wgId_U' = [wgId_U EXCEPT ![self] = msgFromDev[self][2]]
                       /\ pId' = [pId EXCEPT ![self] = 0]
                       /\ pc' = [pc EXCEPT ![self] = "u_send_gowg"]
                  ELSE /\ IF msgFromDev[self][1] = STOP
                             THEN /\ pId' = [pId EXCEPT ![self] = 0]
                                  /\ pc' = [pc EXCEPT ![self] = "u_send_stop"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "u_done"]
                                  /\ pId' = pId
                       /\ UNCHANGED << wgIter, wgId_U >>
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, nProc, msgFromDev, 
                            msgFromPEX, dId, uId, wgId, msgFromHost, 
                            msgFromUnit, msgFromDevice, i_1, j_1 >>

u_send_gowg(self) == /\ pc[self] = "u_send_gowg"
                     /\ IF pId[self] >= nWorkingPEsPerUnit
                           THEN /\ pc' = [pc EXCEPT ![self] = "u_after_send"]
                           ELSE /\ pc' = [pc EXCEPT ![self] = "u_send_gowg_msg"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                     clockFlag, flag, final, hst_d, d_hst, 
                                     dev_u, u_dev, u_pex, pex_u, globalTime, 
                                     Tmin, globalMemory, localMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingPEs, 
                                     nRunningPEs, aoutput, nWaitingPEs, 
                                     nWaitingPEsOut, stack, bt, dId_, uId_, 
                                     pId_, msg1, startTime, curTime, 
                                     globalOffset, tileIdx, localMemIdx, 
                                     wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                     pId, wgIter, wgId_U, nProc, msgFromDev, 
                                     msgFromPEX, dId, uId, wgId, msgFromHost, 
                                     msgFromUnit, msgFromDevice, i_1, j_1 >>

u_send_gowg_msg(self) == /\ pc[self] = "u_send_gowg_msg"
                         /\ u_pex' = [u_pex EXCEPT ![dId_U[self]][uId_U[self]][pId[self]] = Append(u_pex[dId_U[self]][uId_U[self]][pId[self]], (<<GOWG, wgId_U[self]>>))]
                         /\ pc' = [pc EXCEPT ![self] = "u_send_go_msg"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, pex_u, 
                                         globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

u_send_go_msg(self) == /\ pc[self] = "u_send_go_msg"
                       /\ u_pex' = [u_pex EXCEPT ![dId_U[self]][uId_U[self]][pId[self]] = Append(u_pex[dId_U[self]][uId_U[self]][pId[self]], (<<GO, wgIter[self]>>))]
                       /\ pId' = [pId EXCEPT ![self] = pId[self] + 1]
                       /\ pc' = [pc EXCEPT ![self] = "u_send_gowg"]
                       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                       hostFlag, clockFlag, flag, final, hst_d, 
                                       d_hst, dev_u, u_dev, pex_u, globalTime, 
                                       Tmin, globalMemory, localMemory, 
                                       workGroupSize, nWorkGroups, tileSize, 
                                       nWorkingDevices, nWorkingUnitsPerDevice, 
                                       nWorkingPEsPerUnit, allWorkingPEs, 
                                       nRunningPEs, aoutput, nWaitingPEs, 
                                       nWaitingPEsOut, stack, bt, dId_, uId_, 
                                       pId_, msg1, startTime, curTime, 
                                       globalOffset, tileIdx, localMemIdx, 
                                       wgIter_, wgId_, localId, t, dId_U, 
                                       uId_U, wgIter, wgId_U, nProc, 
                                       msgFromDev, msgFromPEX, dId, uId, wgId, 
                                       msgFromHost, msgFromUnit, msgFromDevice, 
                                       i_1, j_1 >>

u_after_send(self) == /\ pc[self] = "u_after_send"
                      /\ IF workGroupSize <= nWorkingPEsPerUnit
                            THEN /\ pId' = [pId EXCEPT ![self] = 0]
                                 /\ pc' = [pc EXCEPT ![self] = "u_recv_eoi"]
                                 /\ UNCHANGED << wgIter, nProc >>
                            ELSE /\ wgIter' = [wgIter EXCEPT ![self] = 1]
                                 /\ nProc' = [nProc EXCEPT ![self] = 0]
                                 /\ pId' = [pId EXCEPT ![self] = 0]
                                 /\ pc' = [pc EXCEPT ![self] = "u_overflow_loop"]
                      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                      hostFlag, clockFlag, flag, final, hst_d, 
                                      d_hst, dev_u, u_dev, u_pex, pex_u, 
                                      globalTime, Tmin, globalMemory, 
                                      localMemory, workGroupSize, nWorkGroups, 
                                      tileSize, nWorkingDevices, 
                                      nWorkingUnitsPerDevice, 
                                      nWorkingPEsPerUnit, allWorkingPEs, 
                                      nRunningPEs, aoutput, nWaitingPEs, 
                                      nWaitingPEsOut, stack, bt, dId_, uId_, 
                                      pId_, msg1, startTime, curTime, 
                                      globalOffset, tileIdx, localMemIdx, 
                                      wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                      wgId_U, msgFromDev, msgFromPEX, dId, uId, 
                                      wgId, msgFromHost, msgFromUnit, 
                                      msgFromDevice, i_1, j_1 >>

u_recv_eoi(self) == /\ pc[self] = "u_recv_eoi"
                    /\ IF pId[self] >= nWorkingPEsPerUnit
                          THEN /\ pc' = [pc EXCEPT ![self] = "u_send_done"]
                          ELSE /\ pc' = [pc EXCEPT ![self] = "u_recv_eoi_one"]
                    /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                    clockFlag, flag, final, hst_d, d_hst, 
                                    dev_u, u_dev, u_pex, pex_u, globalTime, 
                                    Tmin, globalMemory, localMemory, 
                                    workGroupSize, nWorkGroups, tileSize, 
                                    nWorkingDevices, nWorkingUnitsPerDevice, 
                                    nWorkingPEsPerUnit, allWorkingPEs, 
                                    nRunningPEs, aoutput, nWaitingPEs, 
                                    nWaitingPEsOut, stack, bt, dId_, uId_, 
                                    pId_, msg1, startTime, curTime, 
                                    globalOffset, tileIdx, localMemIdx, 
                                    wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                    pId, wgIter, wgId_U, nProc, msgFromDev, 
                                    msgFromPEX, dId, uId, wgId, msgFromHost, 
                                    msgFromUnit, msgFromDevice, i_1, j_1 >>

u_recv_eoi_one(self) == /\ pc[self] = "u_recv_eoi_one"
                        /\ pex_u[dId_U[self]][uId_U[self]][pId[self]] # <<>>
                        /\ msgFromPEX' = [msgFromPEX EXCEPT ![self] = Head(pex_u[dId_U[self]][uId_U[self]][pId[self]])]
                        /\ pex_u' = [pex_u EXCEPT ![dId_U[self]][uId_U[self]][pId[self]] = Tail(pex_u[dId_U[self]][uId_U[self]][pId[self]])]
                        /\ pId' = [pId EXCEPT ![self] = pId[self] + 1]
                        /\ pc' = [pc EXCEPT ![self] = "u_recv_eoi"]
                        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                        hostFlag, clockFlag, flag, final, 
                                        hst_d, d_hst, dev_u, u_dev, u_pex, 
                                        globalTime, Tmin, globalMemory, 
                                        localMemory, workGroupSize, 
                                        nWorkGroups, tileSize, nWorkingDevices, 
                                        nWorkingUnitsPerDevice, 
                                        nWorkingPEsPerUnit, allWorkingPEs, 
                                        nRunningPEs, aoutput, nWaitingPEs, 
                                        nWaitingPEsOut, stack, bt, dId_, uId_, 
                                        pId_, msg1, startTime, curTime, 
                                        globalOffset, tileIdx, localMemIdx, 
                                        wgIter_, wgId_, localId, t, dId_U, 
                                        uId_U, wgIter, wgId_U, nProc, 
                                        msgFromDev, dId, uId, wgId, 
                                        msgFromHost, msgFromUnit, 
                                        msgFromDevice, i_1, j_1 >>

u_overflow_loop(self) == /\ pc[self] = "u_overflow_loop"
                         /\ IF pId[self] >= workGroupSize - nWorkingPEsPerUnit
                               THEN /\ pc' = [pc EXCEPT ![self] = "u_overflow_final"]
                               ELSE /\ pc' = [pc EXCEPT ![self] = "u_overflow_recv"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

u_overflow_recv(self) == /\ pc[self] = "u_overflow_recv"
                         /\ LET peIdx == pId[self] % nWorkingPEsPerUnit IN
                              /\ pex_u[dId_U[self]][uId_U[self]][peIdx] # <<>>
                              /\ msgFromPEX' = [msgFromPEX EXCEPT ![self] = Head(pex_u[dId_U[self]][uId_U[self]][peIdx])]
                              /\ pex_u' = [pex_u EXCEPT ![dId_U[self]][uId_U[self]][peIdx] = Tail(pex_u[dId_U[self]][uId_U[self]][peIdx])]
                         /\ pc' = [pc EXCEPT ![self] = "u_overflow_check"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, pId, wgIter, wgId_U, nProc, 
                                         msgFromDev, dId, uId, wgId, 
                                         msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

u_overflow_check(self) == /\ pc[self] = "u_overflow_check"
                          /\ IF msgFromPEX[self][1] = NEOI
                                THEN /\ LET peIdx == pId[self] % nWorkingPEsPerUnit IN
                                          u_pex' = [u_pex EXCEPT ![dId_U[self]][uId_U[self]][peIdx] = Append(u_pex[dId_U[self]][uId_U[self]][peIdx], (<<GO, wgIter[self]>>))]
                                ELSE /\ TRUE
                                     /\ u_pex' = u_pex
                          /\ nProc' = [nProc EXCEPT ![self] = nProc[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "u_overflow_iter"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                          hostFlag, clockFlag, flag, final, 
                                          hst_d, d_hst, dev_u, u_dev, pex_u, 
                                          globalTime, Tmin, globalMemory, 
                                          localMemory, workGroupSize, 
                                          nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingPEs, 
                                          nRunningPEs, aoutput, nWaitingPEs, 
                                          nWaitingPEsOut, stack, bt, dId_, 
                                          uId_, pId_, msg1, startTime, curTime, 
                                          globalOffset, tileIdx, localMemIdx, 
                                          wgIter_, wgId_, localId, t, dId_U, 
                                          uId_U, pId, wgIter, wgId_U, 
                                          msgFromDev, msgFromPEX, dId, uId, 
                                          wgId, msgFromHost, msgFromUnit, 
                                          msgFromDevice, i_1, j_1 >>

u_overflow_iter(self) == /\ pc[self] = "u_overflow_iter"
                         /\ IF nProc[self] >= nWorkingPEsPerUnit
                               THEN /\ wgIter' = [wgIter EXCEPT ![self] = wgIter[self] + 1]
                                    /\ nProc' = [nProc EXCEPT ![self] = 0]
                               ELSE /\ TRUE
                                    /\ UNCHANGED << wgIter, nProc >>
                         /\ pId' = [pId EXCEPT ![self] = pId[self] + 1]
                         /\ pc' = [pc EXCEPT ![self] = "u_overflow_loop"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, u_pex, 
                                         pex_u, globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, wgId_U, msgFromDev, msgFromPEX, 
                                         dId, uId, wgId, msgFromHost, 
                                         msgFromUnit, msgFromDevice, i_1, j_1 >>

u_overflow_final(self) == /\ pc[self] = "u_overflow_final"
                          /\ pId' = [pId EXCEPT ![self] = 0]
                          /\ pc' = [pc EXCEPT ![self] = "u_overflow_final_loop"]
                          /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                          hostFlag, clockFlag, flag, final, 
                                          hst_d, d_hst, dev_u, u_dev, u_pex, 
                                          pex_u, globalTime, Tmin, 
                                          globalMemory, localMemory, 
                                          workGroupSize, nWorkGroups, tileSize, 
                                          nWorkingDevices, 
                                          nWorkingUnitsPerDevice, 
                                          nWorkingPEsPerUnit, allWorkingPEs, 
                                          nRunningPEs, aoutput, nWaitingPEs, 
                                          nWaitingPEsOut, stack, bt, dId_, 
                                          uId_, pId_, msg1, startTime, curTime, 
                                          globalOffset, tileIdx, localMemIdx, 
                                          wgIter_, wgId_, localId, t, dId_U, 
                                          uId_U, wgIter, wgId_U, nProc, 
                                          msgFromDev, msgFromPEX, dId, uId, 
                                          wgId, msgFromHost, msgFromUnit, 
                                          msgFromDevice, i_1, j_1 >>

u_overflow_final_loop(self) == /\ pc[self] = "u_overflow_final_loop"
                               /\ IF pId[self] >= nWorkingPEsPerUnit
                                     THEN /\ pc' = [pc EXCEPT ![self] = "u_send_done"]
                                     ELSE /\ pc' = [pc EXCEPT ![self] = "u_overflow_final_recv"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                               hostFlag, clockFlag, flag, 
                                               final, hst_d, d_hst, dev_u, 
                                               u_dev, u_pex, pex_u, globalTime, 
                                               Tmin, globalMemory, localMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingPEs, nRunningPEs, 
                                               aoutput, nWaitingPEs, 
                                               nWaitingPEsOut, stack, bt, dId_, 
                                               uId_, pId_, msg1, startTime, 
                                               curTime, globalOffset, tileIdx, 
                                               localMemIdx, wgIter_, wgId_, 
                                               localId, t, dId_U, uId_U, pId, 
                                               wgIter, wgId_U, nProc, 
                                               msgFromDev, msgFromPEX, dId, 
                                               uId, wgId, msgFromHost, 
                                               msgFromUnit, msgFromDevice, i_1, 
                                               j_1 >>

u_overflow_final_recv(self) == /\ pc[self] = "u_overflow_final_recv"
                               /\ pex_u[dId_U[self]][uId_U[self]][pId[self]] # <<>>
                               /\ msgFromPEX' = [msgFromPEX EXCEPT ![self] = Head(pex_u[dId_U[self]][uId_U[self]][pId[self]])]
                               /\ pex_u' = [pex_u EXCEPT ![dId_U[self]][uId_U[self]][pId[self]] = Tail(pex_u[dId_U[self]][uId_U[self]][pId[self]])]
                               /\ pId' = [pId EXCEPT ![self] = pId[self] + 1]
                               /\ pc' = [pc EXCEPT ![self] = "u_overflow_final_loop"]
                               /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                               hostFlag, clockFlag, flag, 
                                               final, hst_d, d_hst, dev_u, 
                                               u_dev, u_pex, globalTime, Tmin, 
                                               globalMemory, localMemory, 
                                               workGroupSize, nWorkGroups, 
                                               tileSize, nWorkingDevices, 
                                               nWorkingUnitsPerDevice, 
                                               nWorkingPEsPerUnit, 
                                               allWorkingPEs, nRunningPEs, 
                                               aoutput, nWaitingPEs, 
                                               nWaitingPEsOut, stack, bt, dId_, 
                                               uId_, pId_, msg1, startTime, 
                                               curTime, globalOffset, tileIdx, 
                                               localMemIdx, wgIter_, wgId_, 
                                               localId, t, dId_U, uId_U, 
                                               wgIter, wgId_U, nProc, 
                                               msgFromDev, dId, uId, wgId, 
                                               msgFromHost, msgFromUnit, 
                                               msgFromDevice, i_1, j_1 >>

u_send_done(self) == /\ pc[self] = "u_send_done"
                     /\ u_dev' = [u_dev EXCEPT ![dId_U[self]][uId_U[self]] = Append(u_dev[dId_U[self]][uId_U[self]], (<<DONE, uId_U[self]>>))]
                     /\ pc' = [pc EXCEPT ![self] = "u_loop"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                     clockFlag, flag, final, hst_d, d_hst, 
                                     dev_u, u_pex, pex_u, globalTime, Tmin, 
                                     globalMemory, localMemory, workGroupSize, 
                                     nWorkGroups, tileSize, nWorkingDevices, 
                                     nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingPEs, 
                                     nRunningPEs, aoutput, nWaitingPEs, 
                                     nWaitingPEsOut, stack, bt, dId_, uId_, 
                                     pId_, msg1, startTime, curTime, 
                                     globalOffset, tileIdx, localMemIdx, 
                                     wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                     pId, wgIter, wgId_U, nProc, msgFromDev, 
                                     msgFromPEX, dId, uId, wgId, msgFromHost, 
                                     msgFromUnit, msgFromDevice, i_1, j_1 >>

u_send_stop(self) == /\ pc[self] = "u_send_stop"
                     /\ IF pId[self] >= nWorkingPEsPerUnit
                           THEN /\ pc' = [pc EXCEPT ![self] = "u_done"]
                           ELSE /\ pc' = [pc EXCEPT ![self] = "u_send_stop_one"]
                     /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                     clockFlag, flag, final, hst_d, d_hst, 
                                     dev_u, u_dev, u_pex, pex_u, globalTime, 
                                     Tmin, globalMemory, localMemory, 
                                     workGroupSize, nWorkGroups, tileSize, 
                                     nWorkingDevices, nWorkingUnitsPerDevice, 
                                     nWorkingPEsPerUnit, allWorkingPEs, 
                                     nRunningPEs, aoutput, nWaitingPEs, 
                                     nWaitingPEsOut, stack, bt, dId_, uId_, 
                                     pId_, msg1, startTime, curTime, 
                                     globalOffset, tileIdx, localMemIdx, 
                                     wgIter_, wgId_, localId, t, dId_U, uId_U, 
                                     pId, wgIter, wgId_U, nProc, msgFromDev, 
                                     msgFromPEX, dId, uId, wgId, msgFromHost, 
                                     msgFromUnit, msgFromDevice, i_1, j_1 >>

u_send_stop_one(self) == /\ pc[self] = "u_send_stop_one"
                         /\ u_pex' = [u_pex EXCEPT ![dId_U[self]][uId_U[self]][pId[self]] = Append(u_pex[dId_U[self]][uId_U[self]][pId[self]], (<<STOP, 0>>))]
                         /\ pId' = [pId EXCEPT ![self] = pId[self] + 1]
                         /\ pc' = [pc EXCEPT ![self] = "u_send_stop"]
                         /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, 
                                         hostFlag, clockFlag, flag, final, 
                                         hst_d, d_hst, dev_u, u_dev, pex_u, 
                                         globalTime, Tmin, globalMemory, 
                                         localMemory, workGroupSize, 
                                         nWorkGroups, tileSize, 
                                         nWorkingDevices, 
                                         nWorkingUnitsPerDevice, 
                                         nWorkingPEsPerUnit, allWorkingPEs, 
                                         nRunningPEs, aoutput, nWaitingPEs, 
                                         nWaitingPEsOut, stack, bt, dId_, uId_, 
                                         pId_, msg1, startTime, curTime, 
                                         globalOffset, tileIdx, localMemIdx, 
                                         wgIter_, wgId_, localId, t, dId_U, 
                                         uId_U, wgIter, wgId_U, nProc, 
                                         msgFromDev, msgFromPEX, dId, uId, 
                                         wgId, msgFromHost, msgFromUnit, 
                                         msgFromDevice, i_1, j_1 >>

u_done(self) == /\ pc[self] = "u_done"
                /\ TRUE
                /\ pc' = [pc EXCEPT ![self] = "Done"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                u_dev, u_pex, pex_u, globalTime, Tmin, 
                                globalMemory, localMemory, workGroupSize, 
                                nWorkGroups, tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                uId, wgId, msgFromHost, msgFromUnit, 
                                msgFromDevice, i_1, j_1 >>

Unit(self) == u0(self) \/ u_loop(self) \/ u1(self) \/ u_send_gowg(self)
                 \/ u_send_gowg_msg(self) \/ u_send_go_msg(self)
                 \/ u_after_send(self) \/ u_recv_eoi(self)
                 \/ u_recv_eoi_one(self) \/ u_overflow_loop(self)
                 \/ u_overflow_recv(self) \/ u_overflow_check(self)
                 \/ u_overflow_iter(self) \/ u_overflow_final(self)
                 \/ u_overflow_final_loop(self)
                 \/ u_overflow_final_recv(self) \/ u_send_done(self)
                 \/ u_send_stop(self) \/ u_send_stop_one(self)
                 \/ u_done(self)

d0(self) == /\ pc[self] = "d0"
            /\ dId' = [dId EXCEPT ![self] = self[3]]
            /\ pc' = [pc EXCEPT ![self] = "d_loop"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, uId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d_loop(self) == /\ pc[self] = "d_loop"
                /\ hst_d # <<>>
                /\ msgFromHost' = [msgFromHost EXCEPT ![self] = Head(hst_d)]
                /\ hst_d' = Tail(hst_d)
                /\ pc' = [pc EXCEPT ![self] = "d1"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, d_hst, dev_u, u_dev, 
                                u_pex, pex_u, globalTime, Tmin, globalMemory, 
                                localMemory, workGroupSize, nWorkGroups, 
                                tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                uId, wgId, msgFromUnit, msgFromDevice, i_1, 
                                j_1 >>

d1(self) == /\ pc[self] = "d1"
            /\ IF msgFromHost[self] = GO
                  THEN /\ pc' = [pc EXCEPT ![self] = "d2"]
                  ELSE /\ IF msgFromHost[self] = STOP
                             THEN /\ pc' = [pc EXCEPT ![self] = "d10"]
                             ELSE /\ pc' = [pc EXCEPT ![self] = "d_done"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d2(self) == /\ pc[self] = "d2"
            /\ uId' = [uId EXCEPT ![self] = 0]
            /\ pc' = [pc EXCEPT ![self] = "d_send"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d_send(self) == /\ pc[self] = "d_send"
                /\ IF uId[self] < nWorkingUnitsPerDevice
                      THEN /\ dev_u' = [dev_u EXCEPT ![dId[self]][uId[self]] = Append(dev_u[dId[self]][uId[self]], (<<GO, uId[self]>>))]
                           /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                           /\ pc' = [pc EXCEPT ![self] = "d_send"]
                      ELSE /\ pc' = [pc EXCEPT ![self] = "d3"]
                           /\ UNCHANGED << dev_u, uId >>
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, u_dev, 
                                u_pex, pex_u, globalTime, Tmin, globalMemory, 
                                localMemory, workGroupSize, nWorkGroups, 
                                tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                wgId, msgFromHost, msgFromUnit, msgFromDevice, 
                                i_1, j_1 >>

d3(self) == /\ pc[self] = "d3"
            /\ IF nWorkGroups <= nWorkingUnitsPerDevice
                  THEN /\ pc' = [pc EXCEPT ![self] = "d4"]
                  ELSE /\ pc' = [pc EXCEPT ![self] = "d6"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d4(self) == /\ pc[self] = "d4"
            /\ uId' = [uId EXCEPT ![self] = 0]
            /\ pc' = [pc EXCEPT ![self] = "d_rcv"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d_rcv(self) == /\ pc[self] = "d_rcv"
               /\ IF uId[self] < nWorkGroups
                     THEN /\ u_dev[dId[self]][uId[self]] # <<>>
                          /\ msgFromUnit' = [msgFromUnit EXCEPT ![self] = Head(u_dev[dId[self]][uId[self]])]
                          /\ u_dev' = [u_dev EXCEPT ![dId[self]][uId[self]] = Tail(u_dev[dId[self]][uId[self]])]
                          /\ allWorkingPEs' = allWorkingPEs - nWorkingPEsPerUnit
                          /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                          /\ pc' = [pc EXCEPT ![self] = "d_rcv"]
                     ELSE /\ pc' = [pc EXCEPT ![self] = "d_write"]
                          /\ UNCHANGED << u_dev, allWorkingPEs, uId, 
                                          msgFromUnit >>
               /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                               clockFlag, flag, final, hst_d, d_hst, dev_u, 
                               u_pex, pex_u, globalTime, Tmin, globalMemory, 
                               localMemory, workGroupSize, nWorkGroups, 
                               tileSize, nWorkingDevices, 
                               nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                               nRunningPEs, aoutput, nWaitingPEs, 
                               nWaitingPEsOut, stack, bt, dId_, uId_, pId_, 
                               msg1, startTime, curTime, globalOffset, tileIdx, 
                               localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                               uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                               msgFromPEX, dId, wgId, msgFromHost, 
                               msgFromDevice, i_1, j_1 >>

d6(self) == /\ pc[self] = "d6"
            /\ wgId' = [wgId EXCEPT ![self] = uId[self]]
            /\ pc' = [pc EXCEPT ![self] = "d7"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, uId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d7(self) == /\ pc[self] = "d7"
            /\ uId' = [uId EXCEPT ![self] = 0]
            /\ pc' = [pc EXCEPT ![self] = "d_wait1"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d_wait1(self) == /\ pc[self] = "d_wait1"
                 /\ IF uId[self] < nWorkGroups - nWorkingUnitsPerDevice
                       THEN /\ pc' = [pc EXCEPT ![self] = "d_choose1"]
                       ELSE /\ pc' = [pc EXCEPT ![self] = "d8"]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                 clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                 u_dev, u_pex, pex_u, globalTime, Tmin, 
                                 globalMemory, localMemory, workGroupSize, 
                                 nWorkGroups, tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingPEs, nRunningPEs, aoutput, 
                                 nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                 uId_, pId_, msg1, startTime, curTime, 
                                 globalOffset, tileIdx, localMemIdx, wgIter_, 
                                 wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                 wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                 uId, wgId, msgFromHost, msgFromUnit, 
                                 msgFromDevice, i_1, j_1 >>

d_choose1(self) == /\ pc[self] = "d_choose1"
                   /\ \E u \in 0..(nWorkingUnitsPerDevice-1):
                        /\ u_dev[dId[self]][u] # <<>>
                        /\ msgFromUnit' = [msgFromUnit EXCEPT ![self] = Head(u_dev[dId[self]][u])]
                        /\ u_dev' = [u_dev EXCEPT ![dId[self]][u] = Tail(u_dev[dId[self]][u])]
                        /\ dev_u' = [dev_u EXCEPT ![dId[self]][u] = Append(dev_u[dId[self]][u], (<<GO, wgId[self]>>))]
                   /\ wgId' = [wgId EXCEPT ![self] = wgId[self] + 1]
                   /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                   /\ pc' = [pc EXCEPT ![self] = "d_wait1"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, u_pex, 
                                   pex_u, globalTime, Tmin, globalMemory, 
                                   localMemory, workGroupSize, nWorkGroups, 
                                   tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   allWorkingPEs, nRunningPEs, aoutput, 
                                   nWaitingPEs, nWaitingPEsOut, stack, bt, 
                                   dId_, uId_, pId_, msg1, startTime, curTime, 
                                   globalOffset, tileIdx, localMemIdx, wgIter_, 
                                   wgId_, localId, t, dId_U, uId_U, pId, 
                                   wgIter, wgId_U, nProc, msgFromDev, 
                                   msgFromPEX, dId, msgFromHost, msgFromDevice, 
                                   i_1, j_1 >>

d8(self) == /\ pc[self] = "d8"
            /\ uId' = [uId EXCEPT ![self] = 0]
            /\ pc' = [pc EXCEPT ![self] = "d_wait2"]
            /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                            clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                            u_pex, pex_u, globalTime, Tmin, globalMemory, 
                            localMemory, workGroupSize, nWorkGroups, tileSize, 
                            nWorkingDevices, nWorkingUnitsPerDevice, 
                            nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                            aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                            dId_, uId_, pId_, msg1, startTime, curTime, 
                            globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                            localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                            nProc, msgFromDev, msgFromPEX, dId, wgId, 
                            msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

d_wait2(self) == /\ pc[self] = "d_wait2"
                 /\ IF uId[self] < nWorkingUnitsPerDevice
                       THEN /\ pc' = [pc EXCEPT ![self] = "d_choose2"]
                       ELSE /\ pc' = [pc EXCEPT ![self] = "d_write"]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                 clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                 u_dev, u_pex, pex_u, globalTime, Tmin, 
                                 globalMemory, localMemory, workGroupSize, 
                                 nWorkGroups, tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingPEs, nRunningPEs, aoutput, 
                                 nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                 uId_, pId_, msg1, startTime, curTime, 
                                 globalOffset, tileIdx, localMemIdx, wgIter_, 
                                 wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                 wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                 uId, wgId, msgFromHost, msgFromUnit, 
                                 msgFromDevice, i_1, j_1 >>

d_choose2(self) == /\ pc[self] = "d_choose2"
                   /\ \E u \in 0..(nWorkingUnitsPerDevice-1):
                        /\ u_dev[dId[self]][u] # <<>>
                        /\ msgFromUnit' = [msgFromUnit EXCEPT ![self] = Head(u_dev[dId[self]][u])]
                        /\ u_dev' = [u_dev EXCEPT ![dId[self]][u] = Tail(u_dev[dId[self]][u])]
                   /\ allWorkingPEs' = allWorkingPEs - nWorkingPEsPerUnit
                   /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                   /\ pc' = [pc EXCEPT ![self] = "d_wait2"]
                   /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                   clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                   u_pex, pex_u, globalTime, Tmin, 
                                   globalMemory, localMemory, workGroupSize, 
                                   nWorkGroups, tileSize, nWorkingDevices, 
                                   nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                   nRunningPEs, aoutput, nWaitingPEs, 
                                   nWaitingPEsOut, stack, bt, dId_, uId_, pId_, 
                                   msg1, startTime, curTime, globalOffset, 
                                   tileIdx, localMemIdx, wgIter_, wgId_, 
                                   localId, t, dId_U, uId_U, pId, wgIter, 
                                   wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                   wgId, msgFromHost, msgFromDevice, i_1, j_1 >>

d_write(self) == /\ pc[self] = "d_write"
                 /\ d_hst' = Append(d_hst, DONE)
                 /\ pc' = [pc EXCEPT ![self] = "d_loop"]
                 /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                 clockFlag, flag, final, hst_d, dev_u, u_dev, 
                                 u_pex, pex_u, globalTime, Tmin, globalMemory, 
                                 localMemory, workGroupSize, nWorkGroups, 
                                 tileSize, nWorkingDevices, 
                                 nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                 allWorkingPEs, nRunningPEs, aoutput, 
                                 nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                 uId_, pId_, msg1, startTime, curTime, 
                                 globalOffset, tileIdx, localMemIdx, wgIter_, 
                                 wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                 wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                 uId, wgId, msgFromHost, msgFromUnit, 
                                 msgFromDevice, i_1, j_1 >>

d10(self) == /\ pc[self] = "d10"
             /\ uId' = [uId EXCEPT ![self] = 0]
             /\ pc' = [pc EXCEPT ![self] = "d_stop"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                             clockFlag, flag, final, hst_d, d_hst, dev_u, 
                             u_dev, u_pex, pex_u, globalTime, Tmin, 
                             globalMemory, localMemory, workGroupSize, 
                             nWorkGroups, tileSize, nWorkingDevices, 
                             nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                             allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                             nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                             startTime, curTime, globalOffset, tileIdx, 
                             localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                             uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                             msgFromPEX, dId, wgId, msgFromHost, msgFromUnit, 
                             msgFromDevice, i_1, j_1 >>

d_stop(self) == /\ pc[self] = "d_stop"
                /\ IF uId[self] < nWorkingUnitsPerDevice
                      THEN /\ dev_u' = [dev_u EXCEPT ![dId[self]][uId[self]] = Append(dev_u[dId[self]][uId[self]], (<<STOP, uId[self]>>))]
                           /\ uId' = [uId EXCEPT ![self] = uId[self] + 1]
                           /\ pc' = [pc EXCEPT ![self] = "d_stop"]
                      ELSE /\ pc' = [pc EXCEPT ![self] = "d_done"]
                           /\ UNCHANGED << dev_u, uId >>
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, u_dev, 
                                u_pex, pex_u, globalTime, Tmin, globalMemory, 
                                localMemory, workGroupSize, nWorkGroups, 
                                tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                wgId, msgFromHost, msgFromUnit, msgFromDevice, 
                                i_1, j_1 >>

d_done(self) == /\ pc[self] = "d_done"
                /\ TRUE
                /\ pc' = [pc EXCEPT ![self] = "Done"]
                /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                                clockFlag, flag, final, hst_d, d_hst, dev_u, 
                                u_dev, u_pex, pex_u, globalTime, Tmin, 
                                globalMemory, localMemory, workGroupSize, 
                                nWorkGroups, tileSize, nWorkingDevices, 
                                nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                                allWorkingPEs, nRunningPEs, aoutput, 
                                nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                                uId_, pId_, msg1, startTime, curTime, 
                                globalOffset, tileIdx, localMemIdx, wgIter_, 
                                wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                                wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                                uId, wgId, msgFromHost, msgFromUnit, 
                                msgFromDevice, i_1, j_1 >>

Device(self) == d0(self) \/ d_loop(self) \/ d1(self) \/ d2(self)
                   \/ d_send(self) \/ d3(self) \/ d4(self) \/ d_rcv(self)
                   \/ d6(self) \/ d7(self) \/ d_wait1(self)
                   \/ d_choose1(self) \/ d8(self) \/ d_wait2(self)
                   \/ d_choose2(self) \/ d_write(self) \/ d10(self)
                   \/ d_stop(self) \/ d_done(self)

h0 == /\ pc[<<2,0,0,0>>] = "h0"
      /\ hostFlag = TRUE
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h1"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h1 == /\ pc[<<2,0,0,0>>] = "h1"
      /\ final' = FALSE
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h2"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h2 == /\ pc[<<2,0,0,0>>] = "h2"
      /\ hst_d' = Append(hst_d, GO)
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h_loop"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h_loop == /\ pc[<<2,0,0,0>>] = "h_loop"
          /\ d_hst # <<>>
          /\ msgFromDevice' = Head(d_hst)
          /\ d_hst' = Tail(d_hst)
          /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h3"]
          /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                          flag, final, hst_d, dev_u, u_dev, u_pex, pex_u, 
                          globalTime, Tmin, globalMemory, localMemory, 
                          workGroupSize, nWorkGroups, tileSize, 
                          nWorkingDevices, nWorkingUnitsPerDevice, 
                          nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                          aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                          dId_, uId_, pId_, msg1, startTime, curTime, 
                          globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                          localId, t, dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                          msgFromDev, msgFromPEX, dId, uId, wgId, msgFromHost, 
                          msgFromUnit, i_1, j_1 >>

h3 == /\ pc[<<2,0,0,0>>] = "h3"
      /\ IF msgFromDevice = DONE
            THEN /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h4"]
            ELSE /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h6"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h4 == /\ pc[<<2,0,0,0>>] = "h4"
      /\ hst_d' = Append(hst_d, STOP)
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h5"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h5 == /\ pc[<<2,0,0,0>>] = "h5"
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h7"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h6 == /\ pc[<<2,0,0,0>>] = "h6"
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "h_loop"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

h7 == /\ pc[<<2,0,0,0>>] = "h7"
      /\ TRUE
      /\ pc' = [pc EXCEPT ![<<2,0,0,0>>] = "Done"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

Host == h0 \/ h1 \/ h2 \/ h_loop \/ h3 \/ h4 \/ h5 \/ h6 \/ h7

c0 == /\ pc[<<0,0,0,0>>] = "c0"
      /\ clockFlag = TRUE
      /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

loop_cl == /\ pc[<<0,0,0,0>>] = "loop_cl"
           /\ IF ~final
                 THEN /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "c1"]
                 ELSE /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "Done"]
           /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                           clockFlag, flag, final, hst_d, d_hst, dev_u, u_dev, 
                           u_pex, pex_u, globalTime, Tmin, globalMemory, 
                           localMemory, workGroupSize, nWorkGroups, tileSize, 
                           nWorkingDevices, nWorkingUnitsPerDevice, 
                           nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, 
                           aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, 
                           dId_, uId_, pId_, msg1, startTime, curTime, 
                           globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                           localId, t, dId_U, uId_U, pId, wgIter, wgId_U, 
                           nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                           msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

c1 == /\ pc[<<0,0,0,0>>] = "c1"
      /\ IF  (nRunningPEs = (allWorkingPEs - nWaitingPEs))
            /\ (allWorkingPEs # 0)
            /\ (nWaitingPEs # allWorkingPEs)
            THEN /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "c2"]
            ELSE /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

c2 == /\ pc[<<0,0,0,0>>] = "c2"
      /\ nRunningPEs' = 0
      /\ globalTime' = globalTime + 1
      /\ pc' = [pc EXCEPT ![<<0,0,0,0>>] = "loop_cl"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      Tmin, globalMemory, localMemory, workGroupSize, 
                      nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, aoutput, nWaitingPEs, nWaitingPEsOut, 
                      stack, bt, dId_, uId_, pId_, msg1, startTime, curTime, 
                      globalOffset, tileIdx, localMemIdx, wgIter_, wgId_, 
                      localId, t, dId_U, uId_U, pId, wgIter, wgId_U, nProc, 
                      msgFromDev, msgFromPEX, dId, uId, wgId, msgFromHost, 
                      msgFromUnit, msgFromDevice, i_1, j_1 >>

Clock == c0 \/ loop_cl \/ c1 \/ c2

m_loop_gm == /\ pc[<<1,0,0,0>>] = "m_loop_gm"
             /\ IF i_1 < INPUT_DATA_SIZE
                   THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m1"]
                   ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m_loop_lm"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                             clockFlag, flag, final, hst_d, d_hst, dev_u, 
                             u_dev, u_pex, pex_u, globalTime, Tmin, 
                             globalMemory, localMemory, workGroupSize, 
                             nWorkGroups, tileSize, nWorkingDevices, 
                             nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                             allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                             nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                             startTime, curTime, globalOffset, tileIdx, 
                             localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                             uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                             msgFromPEX, dId, uId, wgId, msgFromHost, 
                             msgFromUnit, msgFromDevice, i_1, j_1 >>

m1 == /\ pc[<<1,0,0,0>>] = "m1"
      /\ globalMemory' = [globalMemory EXCEPT ![i_1] = INPUT_DATA_SIZE - i_1 - 1]
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m2"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, localMemory, workGroupSize, 
                      nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m2 == /\ pc[<<1,0,0,0>>] = "m2"
      /\ i_1' = i_1 + 1
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m_loop_gm"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, j_1 >>

m_loop_lm == /\ pc[<<1,0,0,0>>] = "m_loop_lm"
             /\ IF j_1 < LOCAL_MEMORY_SIZE * UNITS_PER_DEVICE
                   THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m3"]
                   ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m_WG"]
             /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, 
                             clockFlag, flag, final, hst_d, d_hst, dev_u, 
                             u_dev, u_pex, pex_u, globalTime, Tmin, 
                             globalMemory, localMemory, workGroupSize, 
                             nWorkGroups, tileSize, nWorkingDevices, 
                             nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                             allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                             nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                             startTime, curTime, globalOffset, tileIdx, 
                             localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                             uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                             msgFromPEX, dId, uId, wgId, msgFromHost, 
                             msgFromUnit, msgFromDevice, i_1, j_1 >>

m3 == /\ pc[<<1,0,0,0>>] = "m3"
      /\ localMemory' = [localMemory EXCEPT ![j_1] = 0]
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m4"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, workGroupSize, 
                      nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m4 == /\ pc[<<1,0,0,0>>] = "m4"
      /\ j_1' = j_1 + 1
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m_loop_lm"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1 >>

m_WG == /\ pc[<<1,0,0,0>>] = "m_WG"
        /\ \E k \in 2..(N - 1):
             workGroupSize' = (INPUT_DATA_SIZE \div (2^(N - k)))
        /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m_TS"]
        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                        flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                        globalTime, Tmin, globalMemory, localMemory, 
                        nWorkGroups, tileSize, nWorkingDevices, 
                        nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                        allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                        nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                        startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                        wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                        wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                        msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m_TS == /\ pc[<<1,0,0,0>>] = "m_TS"
        /\ \E l \in 1..(N - 2):
             tileSize' = (INPUT_DATA_SIZE \div (2^(N - l)))
        /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m5"]
        /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                        flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                        globalTime, Tmin, globalMemory, localMemory, 
                        workGroupSize, nWorkGroups, nWorkingDevices, 
                        nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                        allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                        nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                        startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                        wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                        wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                        msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m5 == /\ pc[<<1,0,0,0>>] = "m5"
      /\ IF workGroupSize * tileSize > INPUT_DATA_SIZE
            THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m6"]
            ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m7"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m6 == /\ pc[<<1,0,0,0>>] = "m6"
      /\ tileSize' = (INPUT_DATA_SIZE \div workGroupSize)
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m7"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m7 == /\ pc[<<1,0,0,0>>] = "m7"
      /\ nWorkGroups' = (INPUT_DATA_SIZE \div (workGroupSize * tileSize))
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m8"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m8 == /\ pc[<<1,0,0,0>>] = "m8"
      /\ nWorkingDevices' = DEVICES
      /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m9"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m9 == /\ pc[<<1,0,0,0>>] = "m9"
      /\ IF nWorkGroups <= UNITS_PER_DEVICE * DEVICES
            THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m10"]
            ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m11"]
      /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                      flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                      globalTime, Tmin, globalMemory, localMemory, 
                      workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                      nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                      allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                      nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                      startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                      wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                      wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                      msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m10 == /\ pc[<<1,0,0,0>>] = "m10"
       /\ nWorkingDevices' = (nWorkGroups \div UNITS_PER_DEVICE)
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m11"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                       nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                       startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                       wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                       wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                       msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m11 == /\ pc[<<1,0,0,0>>] = "m11"
       /\ IF (nWorkGroups \div UNITS_PER_DEVICE) # 0
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m12"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m13"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                       nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                       startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                       wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                       wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                       msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m12 == /\ pc[<<1,0,0,0>>] = "m12"
       /\ nWorkingDevices' = 1
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m13"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                       nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                       startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                       wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                       wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                       msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m13 == /\ pc[<<1,0,0,0>>] = "m13"
       /\ nWorkingUnitsPerDevice' = UNITS_PER_DEVICE
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m14"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, aoutput, 
                       nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, uId_, 
                       pId_, msg1, startTime, curTime, globalOffset, tileIdx, 
                       localMemIdx, wgIter_, wgId_, localId, t, dId_U, uId_U, 
                       pId, wgIter, wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                       uId, wgId, msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                       j_1 >>

m14 == /\ pc[<<1,0,0,0>>] = "m14"
       /\ IF nWorkGroups <= UNITS_PER_DEVICE
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m15"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m16"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                       nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                       startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                       wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                       wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                       msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m15 == /\ pc[<<1,0,0,0>>] = "m15"
       /\ nWorkingUnitsPerDevice' = nWorkGroups
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m16"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, aoutput, 
                       nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, uId_, 
                       pId_, msg1, startTime, curTime, globalOffset, tileIdx, 
                       localMemIdx, wgIter_, wgId_, localId, t, dId_U, uId_U, 
                       pId, wgIter, wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                       uId, wgId, msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                       j_1 >>

m16 == /\ pc[<<1,0,0,0>>] = "m16"
       /\ nWorkingPEsPerUnit' = PES_PER_UNIT
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m17"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, allWorkingPEs, nRunningPEs, 
                       aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                       uId_, pId_, msg1, startTime, curTime, globalOffset, 
                       tileIdx, localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                       uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                       msgFromPEX, dId, uId, wgId, msgFromHost, msgFromUnit, 
                       msgFromDevice, i_1, j_1 >>

m17 == /\ pc[<<1,0,0,0>>] = "m17"
       /\ IF workGroupSize <= PES_PER_UNIT
             THEN /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m18"]
             ELSE /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m19"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, 
                       allWorkingPEs, nRunningPEs, aoutput, nWaitingPEs, 
                       nWaitingPEsOut, stack, bt, dId_, uId_, pId_, msg1, 
                       startTime, curTime, globalOffset, tileIdx, localMemIdx, 
                       wgIter_, wgId_, localId, t, dId_U, uId_U, pId, wgIter, 
                       wgId_U, nProc, msgFromDev, msgFromPEX, dId, uId, wgId, 
                       msgFromHost, msgFromUnit, msgFromDevice, i_1, j_1 >>

m18 == /\ pc[<<1,0,0,0>>] = "m18"
       /\ nWorkingPEsPerUnit' = workGroupSize
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m19"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, allWorkingPEs, nRunningPEs, 
                       aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                       uId_, pId_, msg1, startTime, curTime, globalOffset, 
                       tileIdx, localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                       uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                       msgFromPEX, dId, uId, wgId, msgFromHost, msgFromUnit, 
                       msgFromDevice, i_1, j_1 >>

m19 == /\ pc[<<1,0,0,0>>] = "m19"
       /\ allWorkingPEs' = nWorkingDevices * nWorkingUnitsPerDevice * nWorkingPEsPerUnit
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "m20"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, hostFlag, clockFlag, 
                       flag, final, hst_d, d_hst, dev_u, u_dev, u_pex, pex_u, 
                       globalTime, Tmin, globalMemory, localMemory, 
                       workGroupSize, nWorkGroups, tileSize, nWorkingDevices, 
                       nWorkingUnitsPerDevice, nWorkingPEsPerUnit, nRunningPEs, 
                       aoutput, nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, 
                       uId_, pId_, msg1, startTime, curTime, globalOffset, 
                       tileIdx, localMemIdx, wgIter_, wgId_, localId, t, dId_U, 
                       uId_U, pId, wgIter, wgId_U, nProc, msgFromDev, 
                       msgFromPEX, dId, uId, wgId, msgFromHost, msgFromUnit, 
                       msgFromDevice, i_1, j_1 >>

m20 == /\ pc[<<1,0,0,0>>] = "m20"
       /\ hostFlag' = TRUE
       /\ clockFlag' = TRUE
       /\ pc' = [pc EXCEPT ![<<1,0,0,0>>] = "Done"]
       /\ UNCHANGED << GO, STOP, DONE, GOWG, EOI, NEOI, flag, final, hst_d, 
                       d_hst, dev_u, u_dev, u_pex, pex_u, globalTime, Tmin, 
                       globalMemory, localMemory, workGroupSize, nWorkGroups, 
                       tileSize, nWorkingDevices, nWorkingUnitsPerDevice, 
                       nWorkingPEsPerUnit, allWorkingPEs, nRunningPEs, aoutput, 
                       nWaitingPEs, nWaitingPEsOut, stack, bt, dId_, uId_, 
                       pId_, msg1, startTime, curTime, globalOffset, tileIdx, 
                       localMemIdx, wgIter_, wgId_, localId, t, dId_U, uId_U, 
                       pId, wgIter, wgId_U, nProc, msgFromDev, msgFromPEX, dId, 
                       uId, wgId, msgFromHost, msgFromUnit, msgFromDevice, i_1, 
                       j_1 >>

Main == m_loop_gm \/ m1 \/ m2 \/ m_loop_lm \/ m3 \/ m4 \/ m_WG \/ m_TS
           \/ m5 \/ m6 \/ m7 \/ m8 \/ m9 \/ m10 \/ m11 \/ m12 \/ m13 \/ m14
           \/ m15 \/ m16 \/ m17 \/ m18 \/ m19 \/ m20

(* Allow infinite stuttering to prevent deadlock on termination. *)
Terminating == /\ \A self \in ProcSet: pc[self] = "Done"
               /\ UNCHANGED vars

Next == Host \/ Clock \/ Main
           \/ (\E self \in ProcSet: Barrier(self))
           \/ (\E self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1),
                                            p \in 0..(PES_PER_UNIT-1) }: PEX(self))
           \/ (\E self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                            u \in 0..(UNITS_PER_DEVICE-1) }: Unit(self))
           \/ (\E self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) }: Device(self))
           \/ Terminating

Spec == /\ Init /\ [][Next]_vars
        /\ \A self \in { <<5,d,u,p>> : d \in 0..(DEVICES-1),
                                        u \in 0..(UNITS_PER_DEVICE-1),
                                        p \in 0..(PES_PER_UNIT-1) } : /\ WF_vars(PEX(self))
                                                                      /\ WF_vars(Barrier(self))
        /\ \A self \in { <<4,0,d,u>> : d \in 0..(DEVICES-1),
                                        u \in 0..(UNITS_PER_DEVICE-1) } : WF_vars(Unit(self))
        /\ \A self \in { <<3,0,d,0>> : d \in 0..(DEVICES-1) } : WF_vars(Device(self))
        /\ WF_vars(Host)
        /\ WF_vars(Clock)
        /\ WF_vars(Main)

Termination == <>(\A self \in ProcSet: pc[self] = "Done")

\* END TRANSLATION 

OverTime == [](final => (globalTime > Tmin))
DebugInv == final => FALSE
=============================================================================
\* Modification History
\* Last modified Fri Apr 17 12:11:20 MSK 2026 by s.flusova
\* Created Wed Apr 15 11:03:11 MSK 2026 by s.flusova
