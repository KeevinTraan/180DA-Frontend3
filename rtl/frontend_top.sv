// Top-Level 4-Stage 2-Wide Out-of-Order RISC-V Frontend
module frontend_top #(
    parameter int unsigned NUM_CHECKPOINTS = 4
) (
    input  logic                    clk_i,
    input  logic                    rst_ni,
    instruction_memory_if.frontend  imem_if,
    frontend_backend_if.frontend    be_if,

    // Optional Branch Predictor Accuracy Counters
    output logic [31:0]             stat_branches_o,
    output logic [31:0]             stat_mispredicts_o
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    // Global frontend flush on misprediction
    logic            flush_c;
    logic [XLEN-1:0] flush_pc_c;

    assign flush_c    = be_if.branch_resolve_valid && be_if.branch_mispredict;
    assign flush_pc_c = be_if.branch_result_pc;

    // BPU <-> Stage 1 / Stage 4
    logic [XLEN-1:0] bpu_pc0, bpu_pc1;
    logic            bpu_advance;
    pred_meta_t      bpu_pred0, bpu_pred1;
    bpu_update_t     bpu_update;

    // Stage 1 -> Elastic Buffer 1 -> Stage 2
    logic          s1_valid, s1_ready;
    fetch_bundle_t s1_bundle;
    logic          buf1_valid, buf1_ready;
    fetch_bundle_t buf1_bundle;

    // Stage 2 -> Elastic Buffer 2 -> Stage 3
    decode_bundle_t s2_bundle;
    logic           buf2_valid, buf2_ready;
    decode_bundle_t buf2_bundle;

    // Stage 3 <-> Stage 4
    logic [ROB_TAG_W-1:0]       rob_free_count;
    rob_tag_t                   rob_tail_tag;
    lane_count_t                alloc_count;
    rob_alloc_entry_t           alloc_entry0, alloc_entry1;
    logic                       retire_free_valid;
    phys_reg_t                  retire_old_pdst;
    logic [NUM_CHECKPOINTS-1:0] ckpt_valid;
    rob_tag_t                   ckpt_rob_tag [NUM_CHECKPOINTS];
    free_count_t                free_reg_count_unused;

    // -------------------------------------------------------------------------
    // Branch Prediction Unit (GShare + BTB)
    // -------------------------------------------------------------------------
    frontend_bpu u_bpu (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .pc0_i              (bpu_pc0),
        .pc1_i              (bpu_pc1),
        .advance_spec_i     (bpu_advance),
        .pred0_o            (bpu_pred0),
        .pred1_o            (bpu_pred1),
        .update_i           (bpu_update),
        .stat_branches_o    (stat_branches_o),
        .stat_mispredicts_o (stat_mispredicts_o)
    );

    // -------------------------------------------------------------------------
    // Stage 1: Fetch
    // -------------------------------------------------------------------------
    frontend_fetch u_fetch (
        .clk_i         (clk_i),
        .rst_ni        (rst_ni),
        .flush_i       (flush_c),
        .flush_pc_i    (flush_pc_c),
        .imem          (imem_if),
        .bpu_pc0_o     (bpu_pc0),
        .bpu_pc1_o     (bpu_pc1),
        .bpu_advance_o (bpu_advance),
        .bpu_pred0_i   (bpu_pred0),
        .bpu_pred1_i   (bpu_pred1),
        .out_valid_o   (s1_valid),
        .out_ready_i   (s1_ready),
        .out_bundle_o  (s1_bundle)
    );

    // -------------------------------------------------------------------------
    // Elastic Skid Buffer 1 (Fetch -> Decode)
    // -------------------------------------------------------------------------
    elastic_buffer #(
        .DATA_WIDTH ($bits(fetch_bundle_t))
    ) u_buf_f2d (
        .clk_i          (clk_i),
        .rst_ni         (rst_ni),
        .flush_i        (flush_c),
        .input_valid_i  (s1_valid),
        .input_ready_o  (s1_ready),
        .input_data_i   (s1_bundle),
        .output_valid_o (buf1_valid),
        .output_ready_i (buf1_ready),
        .output_data_o  (buf1_bundle)
    );

    // -------------------------------------------------------------------------
    // Stage 2: Parallel RV32I + Zicsr Decode
    // -------------------------------------------------------------------------
    frontend_decode u_decode (
        .in_bundle_i  (buf1_bundle),
        .out_bundle_o (s2_bundle)
    );

    // -------------------------------------------------------------------------
    // Elastic Skid Buffer 2 (Decode -> Rename)
    // -------------------------------------------------------------------------
    elastic_buffer #(
        .DATA_WIDTH ($bits(decode_bundle_t))
    ) u_buf_d2r (
        .clk_i          (clk_i),
        .rst_ni         (rst_ni),
        .flush_i        (flush_c),
        .input_valid_i  (buf1_valid),
        .input_ready_o  (buf1_ready),
        .input_data_i   (s2_bundle),
        .output_valid_o (buf2_valid),
        .output_ready_i (buf2_ready),
        .output_data_o  (buf2_bundle)
    );

    // -------------------------------------------------------------------------
    // Stage 3: Register Rename + 4-Branch Checkpoint Table
    // -------------------------------------------------------------------------
    frontend_rename #(
        .NUM_CHECKPOINTS (NUM_CHECKPOINTS)
    ) u_rename (
        .clk_i                  (clk_i),
        .rst_ni                 (rst_ni),
        .in_valid_i             (buf2_valid),
        .in_ready_o             (buf2_ready),
        .in_bundle_i            (buf2_bundle),
        .rob_free_count_i       (rob_free_count),
        .rob_tail_tag_i         (rob_tail_tag),
        .rob_head_tag_i         (be_if.rob_head_tag),
        .alloc_count_o          (alloc_count),
        .alloc_entry0_o         (alloc_entry0),
        .alloc_entry1_o         (alloc_entry1),
        .retire_free_valid_i    (retire_free_valid),
        .retire_old_pdst_i      (retire_old_pdst),
        .branch_resolve_valid_i (be_if.branch_resolve_valid),
        .branch_mispredict_i    (be_if.branch_mispredict),
        .branch_result_tag_i    (be_if.branch_result_tag),
        .ckpt_valid_o           (ckpt_valid),
        .ckpt_rob_tag_o         (ckpt_rob_tag),
        .free_reg_count_o       (free_reg_count_unused)
    );

    // -------------------------------------------------------------------------
    // Stage 4: 16-Entry Circular ROB + Ordered Dispatch + Retirement
    // -------------------------------------------------------------------------
    frontend_rob #(
        .NUM_CHECKPOINTS (NUM_CHECKPOINTS)
    ) u_rob (
        .clk_i               (clk_i),
        .rst_ni              (rst_ni),
        .alloc_count_i       (alloc_count),
        .alloc_entry0_i      (alloc_entry0),
        .alloc_entry1_i      (alloc_entry1),
        .rob_free_count_o    (rob_free_count),
        .rob_tail_tag_o      (rob_tail_tag),
        .ckpt_valid_i        (ckpt_valid),
        .ckpt_rob_tag_i      (ckpt_rob_tag),
        .retire_free_valid_o (retire_free_valid),
        .retire_old_pdst_o   (retire_old_pdst),
        .bpu_update_o        (bpu_update),
        .be_if               (be_if)
    );

endmodule