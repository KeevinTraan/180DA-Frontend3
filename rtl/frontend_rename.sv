// Stage 3: Register Rename
// - 6R2W Register Alias Table (RAT)
// - 32-entry Circular Free List FIFO (up to 2 pops/cycle, 1 retire push/cycle)
// - Intra-bundle RAW & WAW bypass network
// - Parameterizable Bitmask Branch Checkpoint Table (default 4 slots)
//   supporting out-of-order branch resolution via ROB-tag age comparison
// - Partial-bundle rename (prevents deadlock when only 1 resource slot is free)
module frontend_rename #(
    parameter int unsigned NUM_CHECKPOINTS = 4
) (
    input  logic                                    clk_i,
    input  logic                                    rst_ni,

    // From Stage 2->3 Elastic Buffer
    input  logic                                    in_valid_i,
    output logic                                    in_ready_o,
    input  frontend_internal_pkg::decode_bundle_t   in_bundle_i,

    // To Stage 4 ROB Allocation
    input  logic [rv32_ooo_pkg::ROB_TAG_W-1:0]      rob_free_count_i,
    input  rv32_ooo_pkg::rob_tag_t                  rob_tail_tag_i,
    input  rv32_ooo_pkg::rob_tag_t                  rob_head_tag_i,
    output rv32_ooo_pkg::lane_count_t               alloc_count_o,
    output frontend_internal_pkg::rob_alloc_entry_t alloc_entry0_o,
    output frontend_internal_pkg::rob_alloc_entry_t alloc_entry1_o,

    // From Stage 4 Retirement (Free List reclamation)
    input  logic                                    retire_free_valid_i,
    input  rv32_ooo_pkg::phys_reg_t                 retire_old_pdst_i,

    // Branch Resolution / Misprediction Recovery
    input  logic                                    branch_resolve_valid_i,
    input  logic                                    branch_mispredict_i,
    input  rv32_ooo_pkg::rob_tag_t                  branch_result_tag_i,

    // Checkpoint visibility for Stage 4 under_unresolved_branch evaluation
    output logic [NUM_CHECKPOINTS-1:0]              ckpt_valid_o,
    output rv32_ooo_pkg::rob_tag_t                  ckpt_rob_tag_o [NUM_CHECKPOINTS],

    // Debug / Verification visibility
    output rv32_ooo_pkg::free_count_t               free_reg_count_o
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    localparam int unsigned CKPT_IDX_W = (NUM_CHECKPOINTS > 1) ? $clog2(NUM_CHECKPOINTS) : 1;

    // -------------------------------------------------------------------------
    // Architectural State: RAT, Free List, Branch Checkpoints, Partial Hold
    // -------------------------------------------------------------------------
    phys_reg_t rat_q [ARCH_REG_COUNT];

    phys_reg_t fl_mem_q [FL_DEPTH];
    fl_ptr_t   fl_head_q; // Pop pointer
    fl_ptr_t   fl_tail_q; // Push pointer

    logic [NUM_CHECKPOINTS-1:0] ckpt_valid_q;
    rob_tag_t                   ckpt_rob_tag_q [NUM_CHECKPOINTS];
    phys_reg_t                  ckpt_rat_q     [NUM_CHECKPOINTS][ARCH_REG_COUNT];
    fl_ptr_t                    ckpt_fl_head_q [NUM_CHECKPOINTS];

    // If Lane 0 was accepted without Lane 1, remember so Lane 1 shifts into effective Lane 0
    logic lane0_consumed_q;

    assign ckpt_valid_o   = ckpt_valid_q;
    assign ckpt_rob_tag_o = ckpt_rob_tag_q;

    // Free register count (0..32)
    fl_ptr_t fl_count_c;
    assign fl_count_c       = fl_tail_q - fl_head_q;
    assign free_reg_count_o = free_count_t'(fl_count_c);

    // -------------------------------------------------------------------------
    // Effective Input Lanes (supports partial-bundle rename cleanly)
    // -------------------------------------------------------------------------
    logic         eff_v0, eff_v1;
    decoded_uop_t eff_uop0, eff_uop1;
    pred_meta_t   eff_pred0, eff_pred1;

    always_comb begin
        if (in_valid_i) begin
            if (!lane0_consumed_q) begin
                eff_v0    = in_bundle_i.valid0;
                eff_v1    = in_bundle_i.valid1;
                eff_uop0  = in_bundle_i.uop0;
                eff_uop1  = in_bundle_i.uop1;
                eff_pred0 = in_bundle_i.pred0;
                eff_pred1 = in_bundle_i.pred1;
            end else begin
                eff_v0    = in_bundle_i.valid1;
                eff_v1    = 1'b0;
                eff_uop0  = in_bundle_i.uop1;
                eff_uop1  = '0;
                eff_pred0 = in_bundle_i.pred1;
                eff_pred1 = '0;
            end
        end else begin
            eff_v0    = 1'b0;
            eff_v1    = 1'b0;
            eff_uop0  = '0;
            eff_uop1  = '0;
            eff_pred0 = '0;
            eff_pred1 = '0;
        end
    end

    // -------------------------------------------------------------------------
    // Free Checkpoint Slot Finder (up to 2 free slots for dual-branch bundles)
    // -------------------------------------------------------------------------
    logic                  has_free_ckpt0, has_free_ckpt1;
    logic [CKPT_IDX_W-1:0] free_ckpt0_idx, free_ckpt1_idx;

    always_comb begin
        has_free_ckpt0 = 1'b0;
        has_free_ckpt1 = 1'b0;
        free_ckpt0_idx = '0;
        free_ckpt1_idx = '0;
        for (int i = 0; i < NUM_CHECKPOINTS; i++) begin
            if (!ckpt_valid_q[i]) begin
                if (!has_free_ckpt0) begin
                    has_free_ckpt0 = 1'b1;
                    free_ckpt0_idx = CKPT_IDX_W'(i);
                end else if (!has_free_ckpt1) begin
                    has_free_ckpt1 = 1'b1;
                    free_ckpt1_idx = CKPT_IDX_W'(i);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Resource Check & Allocation Decision
    // -------------------------------------------------------------------------
    logic need_reg0, need_reg1;
    logic is_br0, is_br1;
    logic can_alloc0, can_alloc1;
    logic [1:0] total_regs_needed;

    assign need_reg0 = eff_v0 && eff_uop0.writes_rd && (eff_uop0.rd != 5'd0);
    assign need_reg1 = eff_v1 && eff_uop1.writes_rd && (eff_uop1.rd != 5'd0);
    assign is_br0    = eff_v0 && is_control_flow(eff_uop0);
    assign is_br1    = eff_v1 && is_control_flow(eff_uop1);

    assign total_regs_needed = {1'b0, need_reg0} + {1'b0, need_reg1};

    always_comb begin
        can_alloc0 = rst_ni && !branch_mispredict_i && eff_v0 &&
                     (rob_free_count_i >= ROB_TAG_W'(1)) &&
                     (!need_reg0 || (fl_count_c >= FL_PTR_W'(1))) &&
                     (!is_br0 || has_free_ckpt0);

        can_alloc1 = can_alloc0 && eff_v1 &&
                     (rob_free_count_i >= ROB_TAG_W'(2)) &&
                     (fl_count_c >= FL_PTR_W'(total_regs_needed)) &&
                     ((is_br0 && is_br1) ? has_free_ckpt1 : (!is_br1 || has_free_ckpt0));

        if (can_alloc1)      alloc_count_o = 2'd2;
        else if (can_alloc0) alloc_count_o = 2'd1;
        else                 alloc_count_o = 2'd0;

        // Pop the Stage 2->3 elastic buffer only once all valid lanes in it are consumed
        if (!in_valid_i) begin
            in_ready_o = !branch_mispredict_i;
        end else if (can_alloc1) begin
            in_ready_o = 1'b1;
        end else if (can_alloc0 && !eff_v1) begin
            in_ready_o = 1'b1;
        end else begin
            in_ready_o = 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // RAT Reads, Free List Pops, and Intra-Bundle RAW / WAW Bypass Network
    // -------------------------------------------------------------------------
    phys_reg_t fl_pop0, fl_pop1;
    phys_reg_t psrc1_0, psrc2_0, pdst_0, old_pdst_0;
    phys_reg_t psrc1_1, psrc2_1, pdst_1, old_pdst_1;
    rob_tag_t  rob_tag0, rob_tag1;

    phys_reg_t rat_after_0 [ARCH_REG_COUNT];
    phys_reg_t rat_after_1 [ARCH_REG_COUNT];
    fl_ptr_t   fl_head_after_0, fl_head_after_1;

    assign fl_pop0  = fl_mem_q[fl_head_q[FL_IDX_W-1:0]];
    assign fl_pop1  = fl_mem_q[fl_head_q[FL_IDX_W-1:0] + FL_IDX_W'(1)];
    assign rob_tag0 = rob_tail_tag_i;
    assign rob_tag1 = increment_rob_tag(rob_tail_tag_i, 2'd1);

    always_comb begin
        // Lane 0 RAT reads
        psrc1_0    = (eff_uop0.rs1 == 5'd0) ? '0 : rat_q[eff_uop0.rs1];
        psrc2_0    = (eff_uop0.rs2 == 5'd0) ? '0 : rat_q[eff_uop0.rs2];
        old_pdst_0 = need_reg0 ? rat_q[eff_uop0.rd] : '0;
        pdst_0     = need_reg0 ? fl_pop0 : '0;

        // Lane 1 RAT reads with combinational RAW & WAW intra-bundle bypass
        if (eff_uop1.rs1 == 5'd0)
            psrc1_1 = '0;
        else if (need_reg0 && (eff_uop1.rs1 == eff_uop0.rd))
            psrc1_1 = pdst_0; // RAW bypass
        else
            psrc1_1 = rat_q[eff_uop1.rs1];

        if (eff_uop1.rs2 == 5'd0)
            psrc2_1 = '0;
        else if (need_reg0 && (eff_uop1.rs2 == eff_uop0.rd))
            psrc2_1 = pdst_0; // RAW bypass
        else
            psrc2_1 = rat_q[eff_uop1.rs2];

        if (!need_reg1)
            old_pdst_1 = '0;
        else if (need_reg0 && (eff_uop1.rd == eff_uop0.rd))
            old_pdst_1 = pdst_0; // WAW bypass
        else
            old_pdst_1 = rat_q[eff_uop1.rd];

        pdst_1 = need_reg1 ? (need_reg0 ? fl_pop1 : fl_pop0) : '0;

        // Post-Lane-0 and Post-Lane-1 RAT & Free List snapshots (for checkpoints & RAT commit)
        for (int i = 0; i < ARCH_REG_COUNT; i++) begin
            rat_after_0[i] = rat_q[i];
        end
        if (need_reg0) begin
            rat_after_0[eff_uop0.rd] = pdst_0;
        end

        for (int i = 0; i < ARCH_REG_COUNT; i++) begin
            rat_after_1[i] = rat_after_0[i];
        end
        if (need_reg1) begin
            rat_after_1[eff_uop1.rd] = pdst_1; // WAW: Lane 1 wins
        end

        fl_head_after_0 = fl_head_q + (need_reg0 ? FL_PTR_W'(1) : FL_PTR_W'(0));
        fl_head_after_1 = fl_head_after_0 + (need_reg1 ? FL_PTR_W'(1) : FL_PTR_W'(0));

        // Drive Stage 4 ROB allocation payloads
        alloc_entry0_o                             = '0;
        alloc_entry0_o.uop.decoded                 = eff_uop0;
        alloc_entry0_o.uop.rob_tag                 = rob_tag0;
        alloc_entry0_o.uop.psrc1                   = psrc1_0;
        alloc_entry0_o.uop.psrc2                   = psrc2_0;
        alloc_entry0_o.uop.pdst                    = pdst_0;
        alloc_entry0_o.uop.has_destination         = need_reg0;
        alloc_entry0_o.uop.under_unresolved_branch = (ckpt_valid_q != '0);
        alloc_entry0_o.old_pdst                    = old_pdst_0;
        alloc_entry0_o.pred                        = eff_pred0;

        alloc_entry1_o                             = '0;
        alloc_entry1_o.uop.decoded                 = eff_uop1;
        alloc_entry1_o.uop.rob_tag                 = rob_tag1;
        alloc_entry1_o.uop.psrc1                   = psrc1_1;
        alloc_entry1_o.uop.psrc2                   = psrc2_1;
        alloc_entry1_o.uop.pdst                    = pdst_1;
        alloc_entry1_o.uop.has_destination         = need_reg1;
        alloc_entry1_o.uop.under_unresolved_branch = (ckpt_valid_q != '0) || is_br0;
        alloc_entry1_o.old_pdst                    = old_pdst_1;
        alloc_entry1_o.pred                        = eff_pred1;
    end

    // -------------------------------------------------------------------------
    // Branch Checkpoint Match & Age Comparison for OOO Resolution / Rollback
    // -------------------------------------------------------------------------
    logic                  ckpt_match_found;
    logic [CKPT_IDX_W-1:0] ckpt_match_idx;
    rob_tag_flat_t         resolved_age;

    always_comb begin
        ckpt_match_found = 1'b0;
        ckpt_match_idx   = '0;
        resolved_age     = rob_tag_distance(branch_result_tag_i, rob_head_tag_i);
        for (int i = 0; i < NUM_CHECKPOINTS; i++) begin
            if (ckpt_valid_q[i] && (ckpt_rob_tag_q[i] == branch_result_tag_i)) begin
                ckpt_match_found = 1'b1;
                ckpt_match_idx   = CKPT_IDX_W'(i);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Sequential State Update (IT-05 Safe: Free List push runs even on mispredict)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (int i = 0; i < ARCH_REG_COUNT; i++) begin
                rat_q[i] <= phys_reg_t'(i);
            end
            for (int i = 0; i < FL_DEPTH; i++) begin
                fl_mem_q[i] <= phys_reg_t'(ARCH_REG_COUNT + i);
            end
            fl_head_q        <= '0;
            fl_tail_q        <= FL_PTR_W'(FL_DEPTH); // Starts full (32 free regs)
            ckpt_valid_q     <= '0;
            lane0_consumed_q <= 1'b0;
            for (int c = 0; c < NUM_CHECKPOINTS; c++) begin
                ckpt_rob_tag_q[c] <= '0;
                ckpt_fl_head_q[c] <= '0;
                for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                    ckpt_rat_q[c][r] <= '0;
                end
            end
        end else begin
            // 1. Free List Retirement Push (ALWAYS active, even on branch misprediction!)
            if (retire_free_valid_i && (retire_old_pdst_i != '0)) begin
                fl_mem_q[fl_tail_q[FL_IDX_W-1:0]] <= retire_old_pdst_i;
                fl_tail_q                         <= fl_tail_q + FL_PTR_W'(1);
            end

            // 2. Branch Misprediction Recovery vs. Normal Rename
            if (branch_resolve_valid_i && branch_mispredict_i) begin
                lane0_consumed_q <= 1'b0;

                // Restore RAT and Free List pop pointer from matching checkpoint
                if (ckpt_match_found) begin
                    for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                        rat_q[r] <= ckpt_rat_q[ckpt_match_idx][r];
                    end
                    fl_head_q <= ckpt_fl_head_q[ckpt_match_idx];
                end

                // Clear matching checkpoint and ALL younger checkpoints (by ROB age)
                for (int c = 0; c < NUM_CHECKPOINTS; c++) begin
                    if (ckpt_valid_q[c]) begin
                        if (rob_tag_distance(ckpt_rob_tag_q[c], rob_head_tag_i) >= resolved_age) begin
                            ckpt_valid_q[c] <= 1'b0;
                        end
                    end
                end
            end else begin
                // Free checkpoint slot on correctly predicted branch resolution (OOO-safe)
                if (branch_resolve_valid_i && !branch_mispredict_i && ckpt_match_found) begin
                    ckpt_valid_q[ckpt_match_idx] <= 1'b0;
                end

                // Partial-bundle tracking
                if (can_alloc0 && eff_v1 && !can_alloc1) begin
                    lane0_consumed_q <= 1'b1;
                end else if (in_ready_o) begin
                    lane0_consumed_q <= 1'b0;
                end

                // Commit RAT & Free List pops
                if (can_alloc1) begin
                    for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                        rat_q[r] <= rat_after_1[r];
                    end
                    fl_head_q <= fl_head_after_1;
                end else if (can_alloc0) begin
                    for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                        rat_q[r] <= rat_after_0[r];
                    end
                    fl_head_q <= fl_head_after_0;
                end

                // Allocate Branch Checkpoints
                if (can_alloc0 && is_br0) begin
                    ckpt_valid_q[free_ckpt0_idx]   <= 1'b1;
                    ckpt_rob_tag_q[free_ckpt0_idx] <= rob_tag0;
                    ckpt_fl_head_q[free_ckpt0_idx] <= fl_head_after_0;
                    for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                        ckpt_rat_q[free_ckpt0_idx][r] <= rat_after_0[r];
                    end
                end

                if (can_alloc1 && is_br1) begin
                    logic [CKPT_IDX_W-1:0] slot1;
                    slot1 = is_br0 ? free_ckpt1_idx : free_ckpt0_idx;
                    ckpt_valid_q[slot1]   <= 1'b1;
                    ckpt_rob_tag_q[slot1] <= rob_tag1;
                    ckpt_fl_head_q[slot1] <= fl_head_after_1;
                    for (int r = 0; r < ARCH_REG_COUNT; r++) begin
                        ckpt_rat_q[slot1][r] <= rat_after_1[r];
                    end
                end
            end
        end
    end

endmodule