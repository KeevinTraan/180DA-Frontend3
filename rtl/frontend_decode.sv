// Stage 2: Parallel Dual RV32I + Zicsr Combinational Decoder
module frontend_decode (
    input  frontend_internal_pkg::fetch_bundle_t  in_bundle_i,
    output frontend_internal_pkg::decode_bundle_t out_bundle_o
);

    import rv32_ooo_pkg::*;
    import frontend_internal_pkg::*;

    function automatic decoded_uop_t decode_rv32(
        input logic [XLEN-1:0] pc,
        input logic [31:0]     instr,
        input logic            fetch_err,
        input logic [XLEN-1:0] pred_next_pc
    );
        decoded_uop_t uop;
        logic [6:0]   opcode;
        logic [2:0]   funct3;
        logic [6:0]   funct7;
        logic [4:0]   rs1, rs2, rd;
        logic [31:0]  imm_i, imm_s, imm_b, imm_u, imm_j;

        begin
            opcode = instr[6:0];
            rd     = instr[11:7];
            funct3 = instr[14:12];
            rs1    = instr[19:15];
            rs2    = instr[24:20];
            funct7 = instr[31:25];

            imm_i = {{20{instr[31]}}, instr[31:20]};
            imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            imm_u = {instr[31:12], 12'b0};
            imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

            // Defaults
            uop                    = '0;
            uop.pc                 = pc;
            uop.instruction        = instr;
            uop.predicted_next_pc  = pred_next_pc;
            uop.rs1                = rs1;
            uop.rs2                = rs2;
            uop.rd                 = rd;
            uop.uop_class          = UOP_INVALID;
            uop.alu_op             = ALU_ADD;
            uop.operand_a_sel      = OPA_ZERO;
            uop.operand_b_sel      = OPB_ZERO;
            uop.branch_op          = BR_NONE;
            uop.mem_cmd            = MEM_NONE;
            uop.mem_size           = MEM_WORD;
            uop.csr_cmd            = CSR_NONE;
            uop.fetch_error        = fetch_err;

            if (fetch_err) begin
                uop.illegal = 1'b0;
            end else begin
                unique case (opcode)
                    // LUI
                    7'b0110111: begin
                        uop.uop_class     = UOP_ALU;
                        uop.alu_op        = ALU_PASS_B;
                        uop.operand_a_sel = OPA_ZERO;
                        uop.operand_b_sel = OPB_IMM;
                        uop.immediate     = imm_u;
                        uop.writes_rd     = (rd != 5'd0);
                    end

                    // AUIPC
                    7'b0010111: begin
                        uop.uop_class     = UOP_ALU;
                        uop.alu_op        = ALU_ADD;
                        uop.operand_a_sel = OPA_PC;
                        uop.operand_b_sel = OPB_IMM;
                        uop.immediate     = imm_u;
                        uop.writes_rd     = (rd != 5'd0);
                    end

                    // JAL
                    7'b1101111: begin
                        uop.uop_class     = UOP_JUMP;
                        uop.alu_op        = ALU_ADD;
                        uop.operand_a_sel = OPA_PC;
                        uop.operand_b_sel = OPB_FOUR;
                        uop.branch_op     = BR_JAL;
                        uop.immediate     = imm_j;
                        uop.writes_rd     = (rd != 5'd0);
                    end

                    // JALR
                    7'b1100111: begin
                        if (funct3 == 3'b000) begin
                            uop.uop_class     = UOP_JUMP;
                            uop.alu_op        = ALU_ADD;
                            uop.operand_a_sel = OPA_PC;
                            uop.operand_b_sel = OPB_FOUR;
                            uop.branch_op     = BR_JALR;
                            uop.uses_rs1      = 1'b1;
                            uop.immediate     = imm_i;
                            uop.writes_rd     = (rd != 5'd0);
                        end else begin
                            uop.illegal = 1'b1;
                        end
                    end

                    // Conditional Branches
                    7'b1100011: begin
                        uop.uop_class     = UOP_BRANCH;
                        uop.operand_a_sel = OPA_RS1;
                        uop.operand_b_sel = OPB_RS2;
                        uop.uses_rs1      = 1'b1;
                        uop.uses_rs2      = 1'b1;
                        uop.immediate     = imm_b;
                        unique case (funct3)
                            3'b000: uop.branch_op = BR_BEQ;
                            3'b001: uop.branch_op = BR_BNE;
                            3'b100: uop.branch_op = BR_BLT;
                            3'b101: uop.branch_op = BR_BGE;
                            3'b110: uop.branch_op = BR_BLTU;
                            3'b111: uop.branch_op = BR_BGEU;
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.uses_rs1  = 1'b0;
                                uop.uses_rs2  = 1'b0;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    // Loads
                    7'b0000011: begin
                        uop.uop_class     = UOP_LOAD;
                        uop.alu_op        = ALU_ADD;
                        uop.operand_a_sel = OPA_RS1;
                        uop.operand_b_sel = OPB_IMM;
                        uop.mem_cmd       = MEM_LOAD;
                        uop.uses_rs1      = 1'b1;
                        uop.writes_rd     = (rd != 5'd0);
                        uop.immediate     = imm_i;
                        unique case (funct3)
                            3'b000: begin uop.mem_size = MEM_BYTE; uop.load_unsigned = 1'b0; end
                            3'b001: begin uop.mem_size = MEM_HALF; uop.load_unsigned = 1'b0; end
                            3'b010: begin uop.mem_size = MEM_WORD; uop.load_unsigned = 1'b0; end
                            3'b100: begin uop.mem_size = MEM_BYTE; uop.load_unsigned = 1'b1; end
                            3'b101: begin uop.mem_size = MEM_HALF; uop.load_unsigned = 1'b1; end
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.mem_cmd   = MEM_NONE;
                                uop.uses_rs1  = 1'b0;
                                uop.writes_rd = 1'b0;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    // Stores
                    7'b0100011: begin
                        uop.uop_class     = UOP_STORE;
                        uop.alu_op        = ALU_ADD;
                        uop.operand_a_sel = OPA_RS1;
                        uop.operand_b_sel = OPB_IMM;
                        uop.mem_cmd       = MEM_STORE;
                        uop.uses_rs1      = 1'b1;
                        uop.uses_rs2      = 1'b1;
                        uop.immediate     = imm_s;
                        unique case (funct3)
                            3'b000: uop.mem_size = MEM_BYTE;
                            3'b001: uop.mem_size = MEM_HALF;
                            3'b010: uop.mem_size = MEM_WORD;
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.mem_cmd   = MEM_NONE;
                                uop.uses_rs1  = 1'b0;
                                uop.uses_rs2  = 1'b0;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    // Integer Register-Immediate (OP-IMM)
                    7'b0010011: begin
                        uop.uop_class     = UOP_ALU;
                        uop.operand_a_sel = OPA_RS1;
                        uop.operand_b_sel = OPB_IMM;
                        uop.uses_rs1      = 1'b1;
                        uop.writes_rd     = (rd != 5'd0);
                        uop.immediate     = imm_i;
                        unique case (funct3)
                            3'b000: uop.alu_op = ALU_ADD;
                            3'b010: uop.alu_op = ALU_SLT;
                            3'b011: uop.alu_op = ALU_SLTU;
                            3'b100: uop.alu_op = ALU_XOR;
                            3'b110: uop.alu_op = ALU_OR;
                            3'b111: uop.alu_op = ALU_AND;
                            3'b001: begin
                                if (funct7 == 7'b0000000) begin
                                    uop.alu_op    = ALU_SLL;
                                    uop.immediate = {27'b0, instr[24:20]};
                                end else begin
                                    uop.uop_class = UOP_INVALID;
                                    uop.uses_rs1  = 1'b0;
                                    uop.writes_rd = 1'b0;
                                    uop.illegal   = 1'b1;
                                end
                            end
                            3'b101: begin
                                if (funct7 == 7'b0000000) begin
                                    uop.alu_op    = ALU_SRL;
                                    uop.immediate = {27'b0, instr[24:20]};
                                end else if (funct7 == 7'b0100000) begin
                                    uop.alu_op    = ALU_SRA;
                                    uop.immediate = {27'b0, instr[24:20]};
                                end else begin
                                    uop.uop_class = UOP_INVALID;
                                    uop.uses_rs1  = 1'b0;
                                    uop.writes_rd = 1'b0;
                                    uop.illegal   = 1'b1;
                                end
                            end
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.uses_rs1  = 1'b0;
                                uop.writes_rd = 1'b0;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    // Integer Register-Register (OP)
                    7'b0110011: begin
                        uop.uop_class     = UOP_ALU;
                        uop.operand_a_sel = OPA_RS1;
                        uop.operand_b_sel = OPB_RS2;
                        uop.uses_rs1      = 1'b1;
                        uop.uses_rs2      = 1'b1;
                        uop.writes_rd     = (rd != 5'd0);
                        unique case ({funct7, funct3})
                            {7'b0000000, 3'b000}: uop.alu_op = ALU_ADD;
                            {7'b0100000, 3'b000}: uop.alu_op = ALU_SUB;
                            {7'b0000000, 3'b001}: uop.alu_op = ALU_SLL;
                            {7'b0000000, 3'b010}: uop.alu_op = ALU_SLT;
                            {7'b0000000, 3'b011}: uop.alu_op = ALU_SLTU;
                            {7'b0000000, 3'b100}: uop.alu_op = ALU_XOR;
                            {7'b0000000, 3'b101}: uop.alu_op = ALU_SRL;
                            {7'b0100000, 3'b101}: uop.alu_op = ALU_SRA;
                            {7'b0000000, 3'b110}: uop.alu_op = ALU_OR;
                            {7'b0000000, 3'b111}: uop.alu_op = ALU_AND;
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.uses_rs1  = 1'b0;
                                uop.uses_rs2  = 1'b0;
                                uop.writes_rd = 1'b0;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    // MISC-MEM (FENCE -> treat as ALU NOP)
                    7'b0001111: begin
                        if (funct3 == 3'b000) begin
                            uop.uop_class     = UOP_ALU;
                            uop.alu_op        = ALU_ADD;
                            uop.operand_a_sel = OPA_ZERO;
                            uop.operand_b_sel = OPB_ZERO;
                        end else begin
                            uop.illegal = 1'b1;
                        end
                    end

                    // SYSTEM / Zicsr
                    7'b1110011: begin
                        uop.csr_address   = instr[31:20];
                        uop.csr_immediate = rs1;
                        unique case (funct3)
                            3'b001: begin // CSRRW
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RW;
                                uop.csr_uses_immediate = 1'b0;
                                uop.uses_rs1           = 1'b1;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            3'b010: begin // CSRRS
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RS;
                                uop.csr_uses_immediate = 1'b0;
                                uop.uses_rs1           = 1'b1;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            3'b011: begin // CSRRC
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RC;
                                uop.csr_uses_immediate = 1'b0;
                                uop.uses_rs1           = 1'b1;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            3'b101: begin // CSRRWI
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RW;
                                uop.csr_uses_immediate = 1'b1;
                                uop.uses_rs1           = 1'b0;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            3'b110: begin // CSRRSI
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RS;
                                uop.csr_uses_immediate = 1'b1;
                                uop.uses_rs1           = 1'b0;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            3'b111: begin // CSRRCI
                                uop.uop_class          = UOP_CSR;
                                uop.csr_cmd            = CSR_RC;
                                uop.csr_uses_immediate = 1'b1;
                                uop.uses_rs1           = 1'b0;
                                uop.writes_rd          = (rd != 5'd0);
                            end
                            default: begin
                                uop.uop_class = UOP_INVALID;
                                uop.illegal   = 1'b1;
                            end
                        endcase
                    end

                    default: begin
                        uop.uop_class = UOP_INVALID;
                        uop.illegal   = 1'b1;
                    end
                endcase
            end

            return uop;
        end
    endfunction

    always_comb begin
        out_bundle_o.valid0 = in_bundle_i.valid0;
        out_bundle_o.valid1 = in_bundle_i.valid1;
        out_bundle_o.pred0  = in_bundle_i.pred0;
        out_bundle_o.pred1  = in_bundle_i.pred1;
        out_bundle_o.uop0   = decode_rv32(
            in_bundle_i.pc0, in_bundle_i.instr0, in_bundle_i.error0, in_bundle_i.pred0.pred_next_pc
        );
        out_bundle_o.uop1   = decode_rv32(
            in_bundle_i.pc1, in_bundle_i.instr1, in_bundle_i.error1, in_bundle_i.pred1.pred_next_pc
        );
    end

endmodule