// Stage 4: 16-Entry Circular Reorder Buffer (ROB), Ordered Dispatch, & Retirement
// Uses a decoupled dispatch_q pointer so partial backend acceptance (0/1/2)
// shifts Lane 1 -> Lane 0 cleanly with zero combinational feedback to Stage 3.
module frontend_rob #(
    parameter int unsigned NUM_CHECKPOINTS = 4
) (
    input  logic                                    clk_i,
    input  logic                                    rst_ni,

    // From Stage 3 Rename Allocation
    input  rv32_ooo_pkg::lane_count_t               alloc_count_i,
    input  frontend_internal_pkg::rob_alloc_entry_t alloc_entry0_i,
    input  frontend_internal_pkg::rob_alloc_entry_t alloc_entry1_i,
    output logic [rv32_ooo_pkg::ROB_TAG_W-1:0]      rob_free_count_o,
    output rv32_ooo_pkg::rob_tag_t                  rob_tail_tag_o,

    // Active Checkpoints from Stage 3 (for accurate under_unresolved_branch at dispatch)
    input  logic [NUM_CHECKPOINTS-1:0]              ckpt_valid_i,
    input  rv32_ooo_pkg::rob_tag_t                  ckpt_rob_tag_i [NUM_CHECKPOINTS],

    // To Stage 3 Free List Reclamation
    output logic                                    retire_free_valid_o,
    output rv32_ooo_pkg::phys_reg_t                 retire_old_pdst_o,

    // To Stage 1/BPU Recovery & Training
    output frontend_internal_pkg::bpu_update_t      bpu_update_o,

    // Backend Interface
    frontend_backend_if.frontend                    be_if
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    typedef struct packed {
        logic            valid;
        logic            completed;
        logic            branch_resolved;
        renamed_uop_t    uop;
        phys_reg_t       old_pdst;
        logic [XLEN-1:0] value;
        pred_meta_t      pred;
    } rob_slot_t;

    rob_slot_t rob_q [ROB_ENTRY_COUNT];

    rob_tag_t head_q;     // Next to retire
    rob_tag_t dispatch_q; // Next to dispatch to backend
    rob_tag_t tail_q;     // Next to allocate from Stage 3

    // Occupancy Calculations (all exact under 6-bit modular arithmetic for <=16 live entries)
    rob_tag_flat_t used_count_c;
    rob_tag_flat_t undispatched_count_c;

    assign used_count_c         = rob_tag_distance(tail_q, head_q);
    assign undispatched_count_c = rob_tag_distance(tail_q, dispatch_q);
    assign rob_free_count_o     = ROB_TAG_W'(ROB_ENTRY_COUNT) - used_count_c;
    assign rob_tail_tag_o       = tail_q;
    assign be_if.rob_head_tag   = head_q;

    // Helper: Is a given ROB tag strictly younger than any currently unresolved branch?
    function automatic logic is_under_branch(input rob_tag_t tag);
        logic          under;
        rob_tag_flat_t tag_age;
        begin
            under   = 1'b0;
            tag_age = rob_tag_distance(tag, head_q);
            for (int c = 0; c < NUM_CHECKPOINTS; c++) begin
                if (ckpt_valid_i[c] &&
                    (tag_age > rob_tag_distance(ckpt_rob_tag_i[c], head_q))) begin
                    under = 1'b1;
                end
            end
            return under;
        end
    endfunction

    // -------------------------------------------------------------------------
    // Ordered Dispatch Output to Backend
    // -------------------------------------------------------------------------
    rob_tag_t disp_tag0, disp_tag1;
    assign disp_tag0 = dispatch_q;
    assign disp_tag1 = increment_rob_tag(dispatch_q, 2'd1);

    always_comb begin
        be_if.dispatch_count  = 2'd0;
        be_if.dispatch_bundle = '0;

        if (rst_ni) begin
            if (undispatched_count_c >= ROB_TAG_W'(2))
                be_if.dispatch_count = 2'd2;
            else
                be_if.dispatch_count = undispatched_count_c[1:0];

            if (be_if.dispatch_count >= 2'd1) begin
                be_if.dispatch_bundle.lane0                         = rob_q[disp_tag0.index].uop;
                be_if.dispatch_bundle.lane0.under_unresolved_branch = is_under_branch(disp_tag0);
            end
            if (be_if.dispatch_count == 2'd2) begin
                be_if.dispatch_bundle.lane1                         = rob_q[disp_tag1.index].uop;
                be_if.dispatch_bundle.lane1.under_unresolved_branch = is_under_branch(disp_tag1);
            end
        end
    end

    // -------------------------------------------------------------------------
    // In-Order Retirement (up to 1/cycle per frontend_backend_if)
    // -------------------------------------------------------------------------
    logic can_retire_c;
    assign can_retire_c = rst_ni && (used_count_c != '0) &&
                          rob_q[head_q.index].valid &&
                          rob_q[head_q.index].completed &&
                          rob_q[head_q.index].branch_resolved;

    assign be_if.retire_valid  = can_retire_c;
    assign be_if.retire_tag    = head_q;
    assign retire_free_valid_o = can_retire_c && rob_q[head_q.index].uop.has_destination;
    assign retire_old_pdst_o   = rob_q[head_q.index].old_pdst;

    // -------------------------------------------------------------------------
    // BPU Update Extraction from Resolving Branch's ROB Entry
    // -------------------------------------------------------------------------
    rob_slot_t br_slot_c;
    always_comb begin
        br_slot_c                = rob_q[be_if.branch_result_tag.index];
        bpu_update_o             = '0;
        bpu_update_o.valid       = rst_ni && be_if.branch_resolve_valid &&
                                   br_slot_c.valid &&
                                   (br_slot_c.uop.rob_tag == be_if.branch_result_tag);
        bpu_update_o.mispredict  = be_if.branch_mispredict;
        bpu_update_o.is_cond     = (br_slot_c.uop.decoded.uop_class == UOP_BRANCH);
        bpu_update_o.is_jump     = (br_slot_c.uop.decoded.uop_class == UOP_JUMP);
        bpu_update_o.pc          = br_slot_c.uop.decoded.pc;
        bpu_update_o.actual_target = be_if.branch_result_pc;
        bpu_update_o.pred_ghr    = br_slot_c.pred.pred_ghr;
        bpu_update_o.actual_taken = bpu_update_o.is_jump ||
                                    (be_if.branch_result_pc != (br_slot_c.uop.decoded.pc + 32'd4));
    end

    // -------------------------------------------------------------------------
    // Sequential ROB State Update
    // -------------------------------------------------------------------------
    rob_tag_t      next_tail_rollback;
    rob_tag_flat_t br_age_c;

    assign next_tail_rollback = increment_rob_tag(be_if.branch_result_tag, 2'd1);
    assign br_age_c           = rob_tag_distance(be_if.branch_result_tag, head_q);

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            head_q     <= '0;
            dispatch_q <= '0;
            tail_q     <= '0;
            for (int i = 0; i < ROB_ENTRY_COUNT; i++) begin
                rob_q[i] <= '0;
            end
        end else begin
            // 1. Completion Channel 0
            if (be_if.completion_valid_0) begin
                logic [ROB_INDEX_W-1:0] idx0;
                idx0 = be_if.completion_0.rob_tag.index;
                if (rob_q[idx0].valid && (rob_q[idx0].uop.rob_tag == be_if.completion_0.rob_tag)) begin
                    rob_q[idx0].completed <= 1'b1;
                    rob_q[idx0].value     <= be_if.completion_0.value;
                end
            end

            // 2. Completion Channel 1
            if (be_if.completion_valid_1) begin
                logic [ROB_INDEX_W-1:0] idx1;
                idx1 = be_if.completion_1.rob_tag.index;
                if (rob_q[idx1].valid && (rob_q[idx1].uop.rob_tag == be_if.completion_1.rob_tag)) begin
                    rob_q[idx1].completed <= 1'b1;
                    rob_q[idx1].value     <= be_if.completion_1.value;
                end
            end

            // 3. Mark Branch Resolved
            if (be_if.branch_resolve_valid) begin
                logic [ROB_INDEX_W-1:0] bidx;
                bidx = be_if.branch_result_tag.index;
                if (rob_q[bidx].valid && (rob_q[bidx].uop.rob_tag == be_if.branch_result_tag)) begin
                    rob_q[bidx].branch_resolved <= 1'b1;
                end
            end

            // 4. In-Order Retirement (runs even on simultaneous mispredict for older head!)
            if (can_retire_c) begin
                rob_q[head_q.index].valid <= 1'b0;
                head_q                    <= increment_rob_tag(head_q, 2'd1);
            end

            // 5. Misprediction Rollback vs. Normal Allocation & Dispatch
            if (be_if.branch_resolve_valid && be_if.branch_mispredict) begin
                // Invalidate all entries strictly younger than branch_result_tag
                for (int i = 0; i < ROB_ENTRY_COUNT; i++) begin
                    if (rob_q[i].valid &&
                        (rob_tag_distance(rob_q[i].uop.rob_tag, head_q) > br_age_c)) begin
                        rob_q[i].valid           <= 1'b0;
                        rob_q[i].completed       <= 1'b0;
                        rob_q[i].branch_resolved <= 1'b0;
                    end
                end
                tail_q     <= next_tail_rollback;
                dispatch_q <= next_tail_rollback;
            end else begin
                // Advance dispatch pointer by backend acceptance (0, 1, or 2)
                if (be_if.dispatch_accept_count != 2'd0) begin
                    dispatch_q <= increment_rob_tag(dispatch_q, be_if.dispatch_accept_count);
                end

                // Allocate up to 2 renamed entries from Stage 3
                if (alloc_count_i >= 2'd1) begin
                    logic [ROB_INDEX_W-1:0] aidx0;
                    aidx0 = tail_q.index;
                    rob_q[aidx0].valid           <= 1'b1;
                    rob_q[aidx0].completed       <= 1'b0;
                    rob_q[aidx0].branch_resolved <= !is_control_flow(alloc_entry0_i.uop.decoded);
                    rob_q[aidx0].uop             <= alloc_entry0_i.uop;
                    rob_q[aidx0].old_pdst        <= alloc_entry0_i.old_pdst;
                    rob_q[aidx0].value           <= '0;
                    rob_q[aidx0].pred            <= alloc_entry0_i.pred;
                end
                if (alloc_count_i == 2'd2) begin
                    rob_tag_t               t1;
                    logic [ROB_INDEX_W-1:0] aidx1;
                    t1    = increment_rob_tag(tail_q, 2'd1);
                    aidx1 = t1.index;
                    rob_q[aidx1].valid           <= 1'b1;
                    rob_q[aidx1].completed       <= 1'b0;
                    rob_q[aidx1].branch_resolved <= !is_control_flow(alloc_entry1_i.uop.decoded);
                    rob_q[aidx1].uop             <= alloc_entry1_i.uop;
                    rob_q[aidx1].old_pdst        <= alloc_entry1_i.old_pdst;
                    rob_q[aidx1].value           <= '0;
                    rob_q[aidx1].pred            <= alloc_entry1_i.pred;
                end
                if (alloc_count_i != 2'd0) begin
                    tail_q <= increment_rob_tag(tail_q, alloc_count_i);
                end
            end
        end
    end

endmodule