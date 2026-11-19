package frontend_internal_pkg;

    import rv32_ooo_pkg::*;

    // Branch Predictor Sizing (Group 3: GShare + BTB)
    localparam int unsigned GHR_W       = 6;
    localparam int unsigned PHT_ENTRIES = (1 << GHR_W); // 64 2-bit saturating counters
    localparam int unsigned BTB_ENTRIES = 32;
    localparam int unsigned BTB_IDX_W   = $clog2(BTB_ENTRIES);
    localparam int unsigned BTB_TAG_W   = XLEN - BTB_IDX_W - 2;

    // Free List sizing: holds PHYS_REG_COUNT - ARCH_REG_COUNT = 32 free regs
    localparam int unsigned FL_DEPTH = PHYS_REG_COUNT - ARCH_REG_COUNT; // 32
    localparam int unsigned FL_IDX_W = $clog2(FL_DEPTH);                // 5
    localparam int unsigned FL_PTR_W = FL_IDX_W + 1;                    // 6 (includes wrap bit)
    typedef logic [FL_PTR_W-1:0] fl_ptr_t;

    // Prediction metadata carried per lane through Decode -> Rename -> ROB
    typedef struct packed {
        logic             pred_taken;
        logic [XLEN-1:0]  pred_next_pc;
        logic [GHR_W-1:0] pred_ghr;
    } pred_meta_t;

    // Stage 1 -> Stage 2 bundle (stored in elastic_buffer #1)
    typedef struct packed {
        logic            valid0;
        logic            valid1;
        logic [XLEN-1:0] pc0;
        logic [XLEN-1:0] pc1;
        logic [31:0]     instr0;
        logic [31:0]     instr1;
        logic            error0;
        logic            error1;
        pred_meta_t      pred0;
        pred_meta_t      pred1;
    } fetch_bundle_t;

    // Stage 2 -> Stage 3 bundle (stored in elastic_buffer #2)
    typedef struct packed {
        logic         valid0;
        logic         valid1;
        decoded_uop_t uop0;
        decoded_uop_t uop1;
        pred_meta_t   pred0;
        pred_meta_t   pred1;
    } decode_bundle_t;

    // Stage 3 -> Stage 4 allocation payload
    typedef struct packed {
        renamed_uop_t uop;
        phys_reg_t    old_pdst;
        pred_meta_t   pred;
    } rob_alloc_entry_t;

    // ROB -> BPU update payload on branch_resolve_valid
    typedef struct packed {
        logic             valid;
        logic             mispredict;
        logic             is_cond;
        logic             is_jump;
        logic             actual_taken;
        logic [XLEN-1:0]  pc;
        logic [XLEN-1:0]  actual_target;
        logic [GHR_W-1:0] pred_ghr;
    } bpu_update_t;

endpackage