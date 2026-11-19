`timescale 1ns/1ps

module tb_frontend;

    import rv32_ooo_pkg::*;
    import instruction_memory_pkg::*;
    import frontend_internal_pkg::*;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    instruction_memory_if imem_if ();
    frontend_backend_if   be_if (.clk_i(clk), .rst_ni(rst_n));

    logic [31:0] stat_branches;
    logic [31:0] stat_mispredicts;

    frontend_top #(
        .NUM_CHECKPOINTS (4)
    ) dut (
        .clk_i              (clk),
        .rst_ni             (rst_n),
        .imem_if            (imem_if),
        .be_if              (be_if),
        .stat_branches_o    (stat_branches),
        .stat_mispredicts_o (stat_mispredicts)
    );

    // -------------------------------------------------------------------------
    // Compliant 1-Cycle Synchronous Instruction ROM Model (disabled edges HOLD)
    // -------------------------------------------------------------------------
    localparam int ROM_WORDS = 1024;
    logic [31:0] rom_mem [ROM_WORDS];

    always_ff @(posedge clk) begin
        if (imem_if.read_enable) begin
            int widx;
            widx = int'(imem_if.address[31:2]);
            if (imem_if.address[1:0] != 2'b00 || widx >= ROM_WORDS) begin
                imem_if.data[31:0] <= 32'h0000_0013;
                imem_if.error[0]   <= 1'b1;
            end else begin
                imem_if.data[31:0] <= rom_mem[widx];
                imem_if.error[0]   <= 1'b0;
            end

            if (imem_if.address[1:0] != 2'b00 || (widx + 1) >= ROM_WORDS) begin
                imem_if.data[63:32] <= 32'h0000_0013;
                imem_if.error[1]    <= 1'b1;
            end else begin
                imem_if.data[63:32] <= rom_mem[widx + 1];
                imem_if.error[1]    <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // RV32I Instruction Encoders
    // -------------------------------------------------------------------------
    function automatic logic [31:0] enc_r(
        input logic [6:0] f7, input logic [4:0] rs2, input logic [4:0] rs1,
        input logic [2:0] f3, input logic [4:0] rd
    );
        return {f7, rs2, rs1, f3, rd, 7'b0110011};
    endfunction

    function automatic logic [31:0] enc_i(
        input logic [11:0] imm, input logic [4:0] rs1, input logic [2:0] f3, input logic [4:0] rd
    );
        return {imm, rs1, f3, rd, 7'b0010011};
    endfunction

    function automatic logic [31:0] enc_b(
        input logic signed [12:0] offset, input logic [4:0] rs2, input logic [4:0] rs1, input logic [2:0] f3
    );
        return {offset[12], offset[10:5], rs2, rs1, f3, offset[4:1], offset[11], 7'b1100011};
    endfunction

    function automatic logic [31:0] enc_jal(input logic signed [20:0] offset, input logic [4:0] rd);
        return {offset[20], offset[10:1], offset[11], offset[19:12], rd, 7'b1101111};
    endfunction

    // -------------------------------------------------------------------------
    // Continuous Physical-Register Conservation Checker (Catches leaks/dups!)
    // Exact law: For every p in 0..63:
    //   count_in_RAT(p) + count_in_FreeList(p) + count_in_ROB_old_pdst(p) == 1
    // -------------------------------------------------------------------------
    int errors = 0;

    always_ff @(posedge clk) if (rst_n) begin
        int seen [PHYS_REG_COUNT];
        int fl_cnt;
        for (int p = 0; p < PHYS_REG_COUNT; p++) seen[p] = 0;

        // 1. RAT
        for (int r = 0; r < ARCH_REG_COUNT; r++) begin
            seen[dut.u_rename.rat_q[r]]++;
        end
        // 2. Free List
        fl_cnt = int'(dut.u_rename.fl_count_c);
        for (int k = 0; k < fl_cnt; k++) begin
            logic [FL_IDX_W-1:0] idx;
            idx = dut.u_rename.fl_head_q[FL_IDX_W-1:0] + FL_IDX_W'(k);
            seen[dut.u_rename.fl_mem_q[idx]]++;
        end
        // 3. Uncommitted ROB old_pdst
        for (int i = 0; i < ROB_ENTRY_COUNT; i++) begin
            if (dut.u_rob.rob_q[i].valid && dut.u_rob.rob_q[i].uop.has_destination) begin
                seen[dut.u_rob.rob_q[i].old_pdst]++;
            end
        end

        for (int p = 0; p < PHYS_REG_COUNT; p++) begin
            if (seen[p] != 1) begin
                $display("FAIL [t=%0t] Phys reg p%0d count=%0d (expected 1)!", $time, p, seen[p]);
                errors++;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    task automatic clear_rom();
        for (int i = 0; i < ROM_WORDS; i++) rom_mem[i] = enc_i(12'd0, 5'd0, 3'b000, 5'd0); // NOP
    endtask

    task automatic idle_backend();
        be_if.dispatch_accept_count = 2'd0;
        be_if.completion_valid_0    = 1'b0;
        be_if.completion_valid_1    = 1'b0;
        be_if.completion_0          = '0;
        be_if.completion_1          = '0;
        be_if.branch_resolve_valid  = 1'b0;
        be_if.branch_mispredict     = 1'b0;
        be_if.branch_result_tag     = '0;
        be_if.branch_result_pc      = '0;
    endtask

    task automatic do_reset();
        idle_backend();
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
    endtask

    // -------------------------------------------------------------------------
    // Test Suite
    // -------------------------------------------------------------------------
    initial begin
        $display("=== UCLA ECE 180 Frontend Verification Suite ===");

        // =====================================================================
        // UT-03: Intra-Bundle RAW & WAW Bypass + Partial Backend Acceptance
        // =====================================================================
        clear_rom();
        // Bundle 0 (PC 0x0, 0x4): RAW hazard
        //   Lane 0: ADD x1, x2, x3
        //   Lane 1: SUB x4, x1, x5  -> psrc1 MUST equal Lane 0's pdst!
        rom_mem[0] = enc_r(7'b0000000, 5'd3, 5'd2, 3'b000, 5'd1);
        rom_mem[1] = enc_r(7'b0100000, 5'd5, 5'd1, 3'b000, 5'd4);
        // Bundle 1 (PC 0x8, 0xC): WAW hazard
        //   Lane 0: ADD x1, x2, x3
        //   Lane 1: OR  x1, x4, x5  -> ROB old_pdst of Lane 1 MUST equal Lane 0's pdst!
        rom_mem[2] = enc_r(7'b0000000, 5'd3, 5'd2, 3'b000, 5'd1);
        rom_mem[3] = enc_r(7'b0000000, 5'd5, 5'd4, 3'b110, 5'd1);

        do_reset();

        // Wait for Bundle 0 to appear at dispatch
        while (be_if.dispatch_count < 2'd2) @(posedge clk);
        if (be_if.dispatch_bundle.lane1.psrc1 !== be_if.dispatch_bundle.lane0.pdst) begin
            $display("FAIL UT-03 RAW: lane1.psrc1=%0d != lane0.pdst=%0d",
                     be_if.dispatch_bundle.lane1.psrc1, be_if.dispatch_bundle.lane0.pdst);
            errors++;
        end else begin
            $display("  ok   UT-03a RAW bypass (p%0d forwarded to lane1.psrc1)",
                     be_if.dispatch_bundle.lane0.pdst);
        end

        // Test Partial Dispatch Acceptance: accept ONLY Lane 0 (count=1)
        begin
            rob_tag_t saved_lane1_tag;
            saved_lane1_tag = be_if.dispatch_bundle.lane1.rob_tag;
            @(negedge clk);
            be_if.dispatch_accept_count = 2'd1;
            @(posedge clk);
            @(negedge clk);
            be_if.dispatch_accept_count = 2'd0;
            #1;
            if (be_if.dispatch_bundle.lane0.rob_tag !== saved_lane1_tag) begin
                $display("FAIL Partial Dispatch: lane0 tag=%0h != old lane1 tag=%0h",
                         be_if.dispatch_bundle.lane0.rob_tag, saved_lane1_tag);
                errors++;
            end else begin
                $display("  ok   Partial dispatch (accept=1 shifted lane1 -> lane0 cleanly)");
            end
        end

        // Accept remaining SUB (lane0) + first ADD of Bundle 1 (lane1)
        @(negedge clk); be_if.dispatch_accept_count = 2'd2;
        @(posedge clk);
        @(negedge clk); be_if.dispatch_accept_count = 2'd1; // Accept OR of Bundle 1
        @(posedge clk);
        @(negedge clk); idle_backend();

        // Check WAW in ROB entries 2 and 3
        if (dut.u_rob.rob_q[3].old_pdst !== dut.u_rob.rob_q[2].uop.pdst) begin
            $display("FAIL UT-03 WAW: rob[3].old_pdst=%0d != rob[2].pdst=%0d",
                     dut.u_rob.rob_q[3].old_pdst, dut.u_rob.rob_q[2].uop.pdst);
            errors++;
        end else begin
            $display("  ok   UT-03b WAW bypass (rob[3].old_pdst=p%0d == rob[2].pdst)",
                     dut.u_rob.rob_q[3].old_pdst);
        end

        // =====================================================================
        // UT-01 & UT-04: Sequential Fetch with Random Stalls + ROB Wrap-Around
        //                + Out-of-Order Completion + In-Order Retirement
        // =====================================================================
        clear_rom();
        for (int i = 0; i < 64; i++) begin
            rom_mem[i] = enc_i(12'(i), 5'd0, 3'b000, 5'((i % 31) + 1)); // ADDI x(1..31), x0, i
        end
        do_reset();

        begin
            rob_tag_t dispatched_q[$];
            int       retired_cnt = 0;
            rob_tag_t exp_retire  = '0;
            logic [31:0] exp_pc   = 32'h0;

            for (int cyc = 0; cyc < 180 && retired_cnt < 40; cyc++) begin
                @(negedge clk);
                idle_backend();

                // Random partial/full/stall acceptance (0, 1, or 2)
                if (be_if.dispatch_count > 0) begin
                    int req;
                    req = cyc % 3; // exercises 0, 1, 2
                    if (req > int'(be_if.dispatch_count)) req = int'(be_if.dispatch_count);
                    be_if.dispatch_accept_count = lane_count_t'(req);

                    if (req >= 1) begin
                        if (be_if.dispatch_bundle.lane0.decoded.pc !== exp_pc) begin
                            $display("FAIL UT-01 PC mismatch: got %08h exp %08h",
                                     be_if.dispatch_bundle.lane0.decoded.pc, exp_pc);
                            errors++;
                        end
                        exp_pc += 32'd4;
                        dispatched_q.push_back(be_if.dispatch_bundle.lane0.rob_tag);
                    end
                    if (req == 2) begin
                        if (be_if.dispatch_bundle.lane1.decoded.pc !== exp_pc) begin
                            $display("FAIL UT-01 PC mismatch lane1: got %08h exp %08h",
                                     be_if.dispatch_bundle.lane1.decoded.pc, exp_pc);
                            errors++;
                        end
                        exp_pc += 32'd4;
                        dispatched_q.push_back(be_if.dispatch_bundle.lane1.rob_tag);
                    end
                end

                // Complete out-of-order: pop from back of queue every other cycle
                if (dispatched_q.size() >= 2 && (cyc % 2 == 0)) begin
                    be_if.completion_valid_0   = 1'b1;
                    be_if.completion_0.rob_tag = dispatched_q.pop_back();
                    be_if.completion_valid_1   = 1'b1;
                    be_if.completion_1.rob_tag = dispatched_q.pop_front();
                end else if (dispatched_q.size() >= 1) begin
                    be_if.completion_valid_0   = 1'b1;
                    be_if.completion_0.rob_tag = dispatched_q.pop_back();
                end

                @(posedge clk);
                if (be_if.retire_valid) begin
                    if (be_if.retire_tag !== exp_retire) begin
                        $display("FAIL UT-04 retire order: got %0h exp %0h",
                                 be_if.retire_tag, exp_retire);
                        errors++;
                    end
                    exp_retire = increment_rob_tag(exp_retire, 2'd1);
                    retired_cnt++;
                end
            end

            if (retired_cnt >= 40)
                $display("  ok   UT-01 & UT-04 (%0d instructions fetched/stalled/retired across ROB wraparound)", retired_cnt);
            else begin
                $display("FAIL UT-01/UT-04 only retired %0d instructions", retired_cnt);
                errors++;
            end
        end

        // =====================================================================
        // IT-05 + OOO Branch Resolution
        // =====================================================================
        clear_rom();
        // PC 0x00: ADDI x1, x0, 10   (I0 - older inst that will retire ON mispredict cycle!)
        // PC 0x04: BEQ  x0, x0, +28  (B1 - jumps to 0x20; initially BTB miss -> mispredicts!)
        // PC 0x08: ADDI x2, x0, 20   (wrong path)
        // PC 0x0C: BEQ  x1, x2, +16  (B2 - younger nested branch -> resolves OOO before B1!)
        // PC 0x10: BEQ  x1, x2, +12  (B3 - second nested branch in dual-branch stress)
        // PC 0x14: ADDI x3, x0, 30   (wrong path)
        // PC 0x20 (word 8): ADDI x6, x0, 99  (correct target of B1!)
        // PC 0x24 (word 9): JAL  x0, -36
        rom_mem[0] = enc_i(12'd10, 5'd0, 3'b000, 5'd1);
        rom_mem[1] = enc_b(13'sd28, 5'd0, 5'd0, 3'b000);
        rom_mem[2] = enc_i(12'd20, 5'd0, 3'b000, 5'd2);
        rom_mem[3] = enc_b(13'sd16, 5'd2, 5'd1, 3'b000);
        rom_mem[4] = enc_b(13'sd12, 5'd2, 5'd1, 3'b000);
        rom_mem[5] = enc_i(12'd30, 5'd0, 3'b000, 5'd3);
        rom_mem[8] = enc_i(12'd99, 5'd0, 3'b000, 5'd6);
        rom_mem[9] = enc_jal(-21'sd36, 5'd0);

        do_reset();

        begin
            rob_tag_t tag_i0, tag_b1, tag_b2;
            int       accepted = 0;

            // Dispatch first 6 instructions (I0, B1, wrong-path ADDI, B2, B3, wrong-path ADDI)
            while (accepted < 6) begin
                @(negedge clk);
                idle_backend();
                if (be_if.dispatch_count > 0) begin
                    int take = int'(be_if.dispatch_count);
                    if (accepted + take > 6) take = 6 - accepted;
                    be_if.dispatch_accept_count = lane_count_t'(take);
                    for (int k = 0; k < take; k++) begin
                        renamed_uop_t u = (k == 0) ? be_if.dispatch_bundle.lane0
                                                   : be_if.dispatch_bundle.lane1;
                        if (accepted + k == 0) tag_i0 = u.rob_tag;
                        if (accepted + k == 1) tag_b1 = u.rob_tag;
                        if (accepted + k == 3) tag_b2 = u.rob_tag;
                    end
                    accepted += take;
                end
                @(posedge clk);
            end

            // Step A: Resolve YOUNGER branch B2 correctly BEFORE B1 (OOO branch resolution!)
            @(negedge clk);
            idle_backend();
            be_if.branch_resolve_valid = 1'b1;
            be_if.branch_mispredict    = 1'b0;
            be_if.branch_result_tag    = tag_b2;
            be_if.branch_result_pc     = 32'h0000_0010;
            be_if.completion_valid_0   = 1'b1;
            be_if.completion_0.rob_tag = tag_i0; // Complete I0 so it retires on NEXT cycle!
            @(posedge clk);

            // Step B (IT-05): On THIS exact cycle, I0 is retiring at head_q AND B1 mispredicts!
            @(negedge clk);
            idle_backend();
            if (!be_if.retire_valid || be_if.retire_tag !== tag_i0) begin
                $display("FAIL IT-05 setup: expected I0 retiring during B1 mispredict");
                errors++;
            end
            be_if.branch_resolve_valid = 1'b1;
            be_if.branch_mispredict    = 1'b1;
            be_if.branch_result_tag    = tag_b1;
            be_if.branch_result_pc     = 32'h0000_0020; // Redirect to word 8
            be_if.completion_valid_0   = 1'b1;
            be_if.completion_0.rob_tag = tag_b1;
            @(posedge clk);
            @(negedge clk);
            idle_backend();

            // Wait for redirected instruction at PC 0x20 to arrive at dispatch
            repeat (10) begin
                @(posedge clk);
                if (be_if.dispatch_count >= 2'd1) begin
                    if (be_if.dispatch_bundle.lane0.decoded.pc === 32'h0000_0020) begin
                        $display("  ok   IT-05 (simultaneous I0 retire + OOO B2 resolve + B1 mispredict -> PC 0x20)");
                        break;
                    end else begin
                        $display("FAIL IT-05 redirect PC=%08h (expected 00000020)",
                                 be_if.dispatch_bundle.lane0.decoded.pc);
                        errors++;
                        break;
                    end
                end
            end
        end

        // =====================================================================
        // UT-02: Train GShare + BTB on a 2-instruction loop (BEQ -> JAL -> BEQ)
        //        and confirm Lane 0 predicted-taken masks Lane 1 (PC 0x04)
        // =====================================================================
        clear_rom();
        // PC 0x00: BEQ  x0, x0, +16  (taken -> 0x10)
        // PC 0x04: ADDI x7, x0, 55   (fall-through -> MUST be masked once trained!)
        // PC 0x10: JAL  x0, -16      (unconditional -> 0x00)
        // PC 0x14: ADDI x8, x0, 77   (fall-through -> masked once JAL is in BTB!)
        rom_mem[0] = enc_b(13'sd16, 5'd0, 5'd0, 3'b000);
        rom_mem[1] = enc_i(12'd55, 5'd0, 3'b000, 5'd7);
        rom_mem[4] = enc_jal(-21'sd16, 5'd0);
        rom_mem[5] = enc_i(12'd77, 5'd0, 3'b000, 5'd8);
        do_reset();

        begin
            logic ut02_pass = 1'b0;
            for (int iter = 0; iter < 40 && !ut02_pass; iter++) begin
                renamed_uop_t u;
                while (be_if.dispatch_count == 2'd0) @(posedge clk);

                // If PC 0x00 predicted taken to 0x10, wait for 0x10 to enter Lane 1
                if (be_if.dispatch_bundle.lane0.decoded.pc === 32'h0000_0000 &&
                    be_if.dispatch_bundle.lane0.decoded.predicted_next_pc === 32'h0000_0010) begin
                    while (be_if.dispatch_count < 2'd2) @(posedge clk);
                    if (be_if.dispatch_bundle.lane1.decoded.pc === 32'h0000_0010) begin
                        $display("  ok   UT-02 (GShare+BTB converged on iter %0d: masked PC 0x04, jumped straight to PC 0x10)", iter);
                        ut02_pass = 1'b1;
                        break;
                    end
                end

                // Accept Lane 0 and resolve it in the backend
                u = be_if.dispatch_bundle.lane0;
                @(negedge clk);
                idle_backend();
                be_if.dispatch_accept_count = 2'd1;
                @(posedge clk);

                @(negedge clk);
                idle_backend();
                be_if.completion_valid_0   = 1'b1;
                be_if.completion_0.rob_tag = u.rob_tag;
                if (u.decoded.pc === 32'h0000_0000) begin
                    be_if.branch_resolve_valid = 1'b1;
                    be_if.branch_result_tag    = u.rob_tag;
                    be_if.branch_result_pc     = 32'h0000_0010;
                    be_if.branch_mispredict    = (u.decoded.predicted_next_pc !== 32'h0000_0010);
                end else if (u.decoded.pc === 32'h0000_0010) begin
                    be_if.branch_resolve_valid = 1'b1;
                    be_if.branch_result_tag    = u.rob_tag;
                    be_if.branch_result_pc     = 32'h0000_0000;
                    be_if.branch_mispredict    = (u.decoded.predicted_next_pc !== 32'h0000_0000);
                end
                @(posedge clk);
                @(negedge clk);
                idle_backend();
            end

            if (!ut02_pass) begin
                $display("FAIL UT-02: GShare+BTB did not converge to mask PC 0x04");
                errors++;
            end
        end

        $display("  info BPU stats: branches=%0d mispredicts=%0d", stat_branches, stat_mispredicts);
        $display("");
        if (errors == 0) $display("FRONTEND PASS (0 errors)");
        else             $display("FRONTEND FAIL (%0d errors)", errors);
        $finish;
    end

endmodule