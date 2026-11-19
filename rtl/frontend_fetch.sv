// Stage 1: Instruction Fetch (Fixed 1-cycle synchronous ROM contract)
// Disabled ROM edges hold previous data/error; flush cleanly squashes in-flight data.
module frontend_fetch (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,

    // Misprediction redirect from Stage 4 / Backend
    input  logic                                 flush_i,
    input  logic [rv32_ooo_pkg::XLEN-1:0]        flush_pc_i,

    // Instruction ROM Interface
    instruction_memory_if.frontend               imem,

    // BPU Lookup Interface
    output logic [rv32_ooo_pkg::XLEN-1:0]        bpu_pc0_o,
    output logic [rv32_ooo_pkg::XLEN-1:0]        bpu_pc1_o,
    output logic                                 bpu_advance_o,
    input  frontend_internal_pkg::pred_meta_t    bpu_pred0_i,
    input  frontend_internal_pkg::pred_meta_t    bpu_pred1_i,

    // To Stage 1->2 Elastic Buffer
    output logic                                 out_valid_o,
    input  logic                                 out_ready_i,
    output frontend_internal_pkg::fetch_bundle_t out_bundle_o
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    // Current fetch PC (address to send to ROM next)
    logic [XLEN-1:0] pc_q;
    logic [XLEN-1:0] next_pc_c;

    // 1-cycle delayed metadata aligned with synchronous imem.data / imem.error
    logic            rom_valid_q;
    logic [XLEN-1:0] rom_pc_q;
    pred_meta_t      rom_pred0_q;
    pred_meta_t      rom_pred1_q;

    // Issue a new ROM read whenever the ROM output slot is empty or being consumed now.
    // On flush_i, hold read_enable=0 so no stale word enters; start fetching flush_pc_i next cycle.
    logic issue_fetch_c;
    assign issue_fetch_c = rst_ni && !flush_i && (!rom_valid_q || out_ready_i);

    assign bpu_pc0_o     = pc_q;
    assign bpu_pc1_o     = pc_q + 32'd4;
    assign bpu_advance_o = issue_fetch_c;

    assign imem.address     = pc_q;
    assign imem.read_enable = issue_fetch_c;

    // Next-PC selection priority:
    //   1. Misprediction redirect (handled in sequential block)
    //   2. Lane 0 predicted taken
    //   3. Lane 1 predicted taken
    //   4. Sequential PC + 8
    always_comb begin
        if (bpu_pred0_i.pred_taken) begin
            next_pc_c = bpu_pred0_i.pred_next_pc;
        end else if (bpu_pred1_i.pred_taken) begin
            next_pc_c = bpu_pred1_i.pred_next_pc;
        end else begin
            next_pc_c = pc_q + 32'd8;
        end
    end

    // Drive Stage 1 -> Elastic Buffer 1 output
    assign out_valid_o = rst_ni && !flush_i && rom_valid_q;

    always_comb begin
        out_bundle_o        = '0;
        out_bundle_o.valid0 = rom_valid_q;
        // Mask Lane 1 when Lane 0 is predicted taken
        out_bundle_o.valid1 = rom_valid_q && !rom_pred0_q.pred_taken;
        out_bundle_o.pc0    = rom_pc_q;
        out_bundle_o.pc1    = rom_pc_q + 32'd4;
        out_bundle_o.instr0 = imem.data[31:0];
        out_bundle_o.instr1 = imem.data[63:32];
        out_bundle_o.error0 = imem.error[0];
        out_bundle_o.error1 = imem.error[1];
        out_bundle_o.pred0  = rom_pred0_q;
        out_bundle_o.pred1  = rom_pred1_q;
    end

    // Sequential state update
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pc_q        <= RESET_PC;
            rom_valid_q <= 1'b0;
            rom_pc_q    <= RESET_PC;
            rom_pred0_q <= '0;
            rom_pred1_q <= '0;
        end else if (flush_i) begin
            pc_q        <= flush_pc_i;
            rom_valid_q <= 1'b0;
        end else if (issue_fetch_c) begin
            pc_q        <= next_pc_c;
            rom_valid_q <= 1'b1;
            rom_pc_q    <= pc_q;
            rom_pred0_q <= bpu_pred0_i;
            rom_pred1_q <= bpu_pred1_i;
        end
        // When !issue_fetch_c and !flush_i, ROM holds imem.data/error and we hold metadata.
    end

endmodule