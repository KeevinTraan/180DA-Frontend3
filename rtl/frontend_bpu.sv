// 2-Port GShare Direction Predictor + Direct-Mapped BTB + Accuracy Counters
module frontend_bpu (
    input  logic                           clk_i,
    input  logic                           rst_ni,

    // 2-Port Lookup from Stage 1 (pc0 = PC, pc1 = PC + 4)
    input  logic [rv32_ooo_pkg::XLEN-1:0]  pc0_i,
    input  logic [rv32_ooo_pkg::XLEN-1:0]  pc1_i,
    input  logic                           advance_spec_i, // High when Stage 1 issues a fetch

    output frontend_internal_pkg::pred_meta_t pred0_o,
    output frontend_internal_pkg::pred_meta_t pred1_o,

    // Resolution & Recovery Update from Stage 4
    input  frontend_internal_pkg::bpu_update_t update_i,

    // Predictor Accuracy Evaluation Outputs
    output logic [31:0]                    stat_branches_o,
    output logic [31:0]                    stat_mispredicts_o
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    // Global History Register (speculative)
    logic [GHR_W-1:0] ghr_q;

    // Pattern History Table (2-bit saturating counters)
    logic [1:0] pht_q [PHT_ENTRIES];

    // Direct-Mapped Branch Target Buffer
    typedef struct packed {
        logic                 valid;
        logic                 is_uncond;
        logic [BTB_TAG_W-1:0] tag;
        logic [XLEN-1:0]      target;
    } btb_entry_t;

    btb_entry_t btb_q [BTB_ENTRIES];

    // Accuracy counters
    logic [31:0] stat_branches_q;
    logic [31:0] stat_mispredicts_q;

    assign stat_branches_o    = stat_branches_q;
    assign stat_mispredicts_o = stat_mispredicts_q;

    // -------------------------------------------------------------------------
    // Combinational 2-Port Lookup
    // -------------------------------------------------------------------------
    logic [BTB_IDX_W-1:0] btb_idx0, btb_idx1;
    logic [BTB_TAG_W-1:0] btb_tag0, btb_tag1;
    btb_entry_t           btb_e0, btb_e1;
    logic                 btb_hit0, btb_hit1;
    logic                 btb_cond0, btb_cond1;

    logic [GHR_W-1:0]     ghr_lane0, ghr_lane1;
    logic [GHR_W-1:0]     pht_idx0, pht_idx1;
    logic                 dir0, dir1;
    logic                 taken0, taken1;

    always_comb begin
        btb_idx0 = pc0_i[BTB_IDX_W+1:2];
        btb_tag0 = pc0_i[XLEN-1:BTB_IDX_W+2];
        btb_e0   = btb_q[btb_idx0];
        btb_hit0 = btb_e0.valid && (btb_e0.tag == btb_tag0);
        btb_cond0 = btb_hit0 && !btb_e0.is_uncond;

        ghr_lane0 = ghr_q;
        pht_idx0  = pc0_i[GHR_W+1:2] ^ ghr_lane0;
        dir0      = pht_q[pht_idx0][1];
        taken0    = btb_hit0 && (btb_e0.is_uncond || dir0);

        // If lane 0 is a conditional branch predicted not-taken, shift 0 into lane 1's GHR view
        ghr_lane1 = btb_cond0 ? {ghr_q[GHR_W-2:0], 1'b0} : ghr_q;

        btb_idx1 = pc1_i[BTB_IDX_W+1:2];
        btb_tag1 = pc1_i[XLEN-1:BTB_IDX_W+2];
        btb_e1   = btb_q[btb_idx1];
        btb_hit1 = btb_e1.valid && (btb_e1.tag == btb_tag1);
        btb_cond1 = btb_hit1 && !btb_e1.is_uncond;

        pht_idx1  = pc1_i[GHR_W+1:2] ^ ghr_lane1;
        dir1      = pht_q[pht_idx1][1];
        taken1    = btb_hit1 && (btb_e1.is_uncond || dir1);

        pred0_o.pred_taken   = taken0;
        pred0_o.pred_next_pc = taken0 ? btb_e0.target : (pc0_i + 32'd4);
        pred0_o.pred_ghr     = ghr_lane0;

        pred1_o.pred_taken   = taken1;
        pred1_o.pred_next_pc = taken1 ? btb_e1.target : (pc1_i + 32'd4);
        pred1_o.pred_ghr     = ghr_lane1;
    end

    // -------------------------------------------------------------------------
    // Sequential Updates (Speculative GHR, Resolution PHT/BTB, Recovery)
    // -------------------------------------------------------------------------
    logic [GHR_W-1:0]     upd_pht_idx;
    logic [BTB_IDX_W-1:0] upd_btb_idx;
    logic [BTB_TAG_W-1:0] upd_btb_tag;

    assign upd_pht_idx = update_i.pc[GHR_W+1:2] ^ update_i.pred_ghr;
    assign upd_btb_idx = update_i.pc[BTB_IDX_W+1:2];
    assign upd_btb_tag = update_i.pc[XLEN-1:BTB_IDX_W+2];

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            ghr_q              <= '0;
            stat_branches_q    <= '0;
            stat_mispredicts_q <= '0;
            for (int i = 0; i < PHT_ENTRIES; i++) begin
                pht_q[i] <= 2'b01; // Weakly not-taken
            end
            for (int i = 0; i < BTB_ENTRIES; i++) begin
                btb_q[i] <= '0;
            end
        end else begin
            // 1. Update accuracy counters and PHT/BTB on branch resolution
            if (update_i.valid) begin
                stat_branches_q <= stat_branches_q + 32'd1;
                if (update_i.mispredict) begin
                    stat_mispredicts_q <= stat_mispredicts_q + 32'd1;
                end

                // Update GShare PHT for conditional branches
                if (update_i.is_cond) begin
                    if (update_i.actual_taken) begin
                        if (pht_q[upd_pht_idx] != 2'b11)
                            pht_q[upd_pht_idx] <= pht_q[upd_pht_idx] + 2'b01;
                    end else begin
                        if (pht_q[upd_pht_idx] != 2'b00)
                            pht_q[upd_pht_idx] <= pht_q[upd_pht_idx] - 2'b01;
                    end
                end

                // Allocate/update BTB on taken conditional branches or jumps
                if (update_i.actual_taken || update_i.is_jump) begin
                    btb_q[upd_btb_idx].valid     <= 1'b1;
                    btb_q[upd_btb_idx].is_uncond <= update_i.is_jump;
                    btb_q[upd_btb_idx].tag       <= upd_btb_tag;
                    btb_q[upd_btb_idx].target    <= update_i.actual_target;
                end
            end

            // 2. GHR update: mispredict recovery wins over speculative fetch advance
            if (update_i.valid && update_i.mispredict) begin
                ghr_q <= update_i.is_cond
                         ? {update_i.pred_ghr[GHR_W-2:0], update_i.actual_taken}
                         : update_i.pred_ghr;
            end else if (advance_spec_i) begin
                if (btb_cond0 && taken0) begin
                    // Lane 0 taken conditional branch (lane 1 masked)
                    ghr_q <= {ghr_q[GHR_W-2:0], 1'b1};
                end else if (btb_cond0 && !taken0 && btb_cond1) begin
                    // Two conditional branches in same bundle, lane 0 not-taken
                    ghr_q <= {ghr_q[GHR_W-3:0], 1'b0, taken1};
                end else if (btb_cond0 && !taken0) begin
                    ghr_q <= {ghr_q[GHR_W-2:0], 1'b0};
                end else if (!taken0 && btb_cond1) begin
                    ghr_q <= {ghr_q[GHR_W-2:0], taken1};
                end
            end
        end
    end

endmodule