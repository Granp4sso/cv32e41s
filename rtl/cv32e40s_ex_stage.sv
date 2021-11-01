// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                 Halfdan Bechmann - halfdan.bechmann@silabs.com             //
//                 Michael Platzer - michael.platzer@tuwien.ac.at             //
//                                                                            //
// Design Name:    Execute stage                                              //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: Hosts ALU and MAC unit                    //
//                 ALU: computes additions/subtractions/comparisons           //
//                 MULT: computes normal multiplications                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40s_ex_stage import cv32e40s_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // ID/EX pipeline
  input id_ex_pipe_t  id_ex_pipe_i,

  // CSR interface
  input  logic [31:0] csr_rdata_i,
  input  logic        csr_illegal_i,

  // EX/WB pipeline
  output ex_wb_pipe_t ex_wb_pipe_o,

  // From controller FSM
  input  ctrl_fsm_t   ctrl_fsm_i,

  // Register file forwarding signals (to ID)
  output logic [31:0] rf_wdata_o,

  // To IF: Jump and branch target and decision
  output logic        branch_decision_o,
  output logic [31:0] branch_target_o,

  // LSU handshake interface
  input  logic        lsu_valid_i,
  output logic        lsu_ready_o,
  output logic        lsu_valid_o,
  input  logic        lsu_ready_i,
  input  logic        lsu_split_i,      // LSU is performing first part of a misaligned/split instruction

  // Stage ready/valid
  output logic        ex_ready_o,       // EX stage is ready for new data
  output logic        ex_valid_o,       // EX stage has valid (non-bubble) data for next stage
  input  logic        wb_ready_i        // WB stage is ready for new data
);

  logic [31:0]    alu_result;
  logic           alu_cmp_result;
  logic [31:0]    mul_result;

  logic           alu_ready;
  logic           alu_valid;
  logic           csr_ready;
  logic           csr_valid;
  logic           mul_ready;
  logic           mul_valid;

  logic           instr_valid;

  // Local signals after evaluating with instr_valid
  logic           mul_en_gated;
  logic           div_en_gated;
  logic           lsu_en_gated;
  logic           previous_exception;

  // div_en not affected by kill/halt
  logic           div_en;

  // Divider signals
  logic           div_ready;
  logic           div_valid;
  logic [31:0]    div_result;

  logic           div_clz_en;
  logic [31:0]    div_clz_data_rev;
  logic [5:0]     div_clz_result;

  logic           div_shift_en;
  logic [5:0]     div_shift_amt;
  logic [31:0]    div_op_b_shifted;

  // eXtension interface signals
  logic           xif_ready;
  logic           xif_valid;

  assign instr_valid = id_ex_pipe_i.instr_valid && !ctrl_fsm_i.kill_ex && !ctrl_fsm_i.halt_ex;

  assign mul_en_gated = id_ex_pipe_i.mul_en && instr_valid; // Factoring in instr_valid to kill mul instructions on kill/halt
  assign div_en_gated = id_ex_pipe_i.div_en && instr_valid; // Factoring in instr_valid to kill div instructions on kill/halt
  assign lsu_en_gated = id_ex_pipe_i.lsu_en && instr_valid; // Factoring in instr_valid to suppress bus transactions on kill/halt

  assign div_en = id_ex_pipe_i.div_en && id_ex_pipe_i.instr_valid; // Valid DIV in EX, not affected by kill/halt


  // Exception happened during IF or ID, or trigger match in ID (converted to NOP).
  // signal needed for ex_valid to go high in such cases
  assign previous_exception = (id_ex_pipe_i.illegal_insn                 ||
                               id_ex_pipe_i.instr.bus_resp.err           ||
                               (id_ex_pipe_i.instr.mpu_status != MPU_OK) ||
                               id_ex_pipe_i.trigger_match)               &&
                              id_ex_pipe_i.instr_valid;

  // ALU write port mux
  always_comb
  begin
    // There is no need to use gated versions of alu_en, mul_en, etc. as rf_wdata_o will be ignored
    // for invalid instructions (as the register file write enable will be suppressed).
    unique case (1'b1)
      id_ex_pipe_i.mul_en : rf_wdata_o = mul_result;
      id_ex_pipe_i.div_en : rf_wdata_o = div_result;
      id_ex_pipe_i.csr_en : rf_wdata_o = csr_rdata_i;                           // alu_en = 1 here as well
      default             : rf_wdata_o = alu_result;                            // Default on purpose
    endcase
  end

  // Branch handling
  assign branch_decision_o = alu_cmp_result;
  assign branch_target_o   = id_ex_pipe_i.operand_c;

  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////


  cv32e40s_alu alu_i
  (
    .operator_i          ( id_ex_pipe_i.alu_operator  ),
    .operand_a_i         ( id_ex_pipe_i.alu_operand_a ),
    .operand_b_i         ( id_ex_pipe_i.alu_operand_b ),

    // ALU CLZ interface
    .div_clz_en_i        ( div_clz_en                 ),
    .div_clz_data_rev_i  ( div_clz_data_rev           ),
    .div_clz_result_o    ( div_clz_result             ),

    // ALU shifter interface
    .div_shift_en_i      ( div_shift_en               ),
    .div_shift_amt_i     ( div_shift_amt              ),
    .div_op_b_shifted_o  ( div_op_b_shifted           ),

    // Result(s)
    .result_o            ( alu_result                 ),
    .cmp_result_o        ( alu_cmp_result             )
  );

  ////////////////////////////////////////////////////
  //  ____ _____     __     __  ____  _____ __  __  //
  // |  _ \_ _\ \   / /    / / |  _ \| ____|  \/  | //
  // | | | | | \ \ / /    / /  | |_) |  _| | |\/| | //
  // | |_| | |  \ V /    / /   |  _ <| |___| |  | | //
  // |____/___|  \_/    /_/    |_| \_\_____|_|  |_| //
  //                                                //
  ////////////////////////////////////////////////////

  // TODO:low COCO analysis. is it okay from a leakage perspective to use the ALU at all for DIV/REM instructions?

  cv32e40s_div div_i
  (
    .clk                ( clk                        ),
    .rst_n              ( rst_n                      ),

    // Input IF
    .data_ind_timing_i  ( 1'b0                       ), // TODO:OE:low connect to CSR
    .operator_i         ( id_ex_pipe_i.div_operator  ),
    .op_a_i             ( id_ex_pipe_i.alu_operand_a ),
    .op_b_i             ( id_ex_pipe_i.alu_operand_b ),

    // ALU CLZ interface
    .alu_clz_result_i   ( div_clz_result             ),
    .alu_clz_en_o       ( div_clz_en                 ),
    .alu_clz_data_rev_o ( div_clz_data_rev           ),

    // ALU shifter interface
    .alu_op_b_shifted_i ( div_op_b_shifted           ),
    .alu_shift_en_o     ( div_shift_en               ),
    .alu_shift_amt_o    ( div_shift_amt              ),

    // Result
    .result_o           ( div_result                 ),

    // divider enable, not affected by kill/halt
    .div_en_i           ( div_en                     ),

    // Handshakes
    .valid_i            ( div_en_gated               ),
    .ready_o            ( div_ready                  ),
    .valid_o            ( div_valid                  ),
    .ready_i            ( wb_ready_i                 )
  );

  ////////////////////////////////////////////////////////////////
  //  __  __ _   _ _   _____ ___ ____  _     ___ _____ ____     //
  // |  \/  | | | | | |_   _|_ _|  _ \| |   |_ _| ____|  _ \    //
  // | |\/| | | | | |   | |  | || |_) | |    | ||  _| | |_) |   //
  // | |  | | |_| | |___| |  | ||  __/| |___ | || |___|  _ <    //
  // |_|  |_|\___/|_____|_| |___|_|   |_____|___|_____|_| \_\   //
  //                                                            //
  ////////////////////////////////////////////////////////////////

  cv32e40s_mult mult_i
  (
    .clk             ( clk                           ),
    .rst_n           ( rst_n                         ),

    .operator_i      ( id_ex_pipe_i.mul_operator     ),
    .signed_mode_i   ( id_ex_pipe_i.mul_signed_mode  ),
    .op_a_i          ( id_ex_pipe_i.mul_operand_a    ),
    .op_b_i          ( id_ex_pipe_i.mul_operand_b    ),

    // Result
    .result_o        ( mul_result                    ),

    // Handshakes
    .valid_i         ( mul_en_gated                  ),
    .ready_o         ( mul_ready                     ),
    .valid_o         ( mul_valid                     ),
    .ready_i         ( wb_ready_i                    )
  );

  // Populate instruction meta data
  instr_meta_t instr_meta_n; 
  always_comb begin
    instr_meta_n              = id_ex_pipe_i.instr_meta;
    instr_meta_n.branch_taken = branch_decision_o;
  end
  
  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : EX_WB_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      ex_wb_pipe_o.instr_valid    <= 1'b0;
      ex_wb_pipe_o.rf_we          <= 1'b0;
      ex_wb_pipe_o.rf_waddr       <= '0;
      ex_wb_pipe_o.rf_wdata       <= 32'b0;
      ex_wb_pipe_o.pc             <= 32'h0;
      ex_wb_pipe_o.instr          <= INST_RESP_RESET_VAL;
      ex_wb_pipe_o.instr_meta     <= '0;
      ex_wb_pipe_o.illegal_insn   <= 1'b0;
      ex_wb_pipe_o.ebrk_insn      <= 1'b0;
      ex_wb_pipe_o.wfi_insn       <= 1'b0;
      ex_wb_pipe_o.ecall_insn     <= 1'b0;
      ex_wb_pipe_o.fencei_insn    <= 1'b0;
      ex_wb_pipe_o.mret_insn      <= 1'b0;
      ex_wb_pipe_o.dret_insn      <= 1'b0;
      ex_wb_pipe_o.lsu_en         <= 1'b0;
      ex_wb_pipe_o.csr_en         <= 1'b0;
      ex_wb_pipe_o.csr_op         <= CSR_OP_READ;
      ex_wb_pipe_o.csr_addr       <= 12'h000;
      ex_wb_pipe_o.csr_wdata      <= 32'h00000000;
      ex_wb_pipe_o.trigger_match  <= 1'b0;
      ex_wb_pipe_o.xif_en         <= 1'b0;
      ex_wb_pipe_o.xif_id         <= '0;
    end
    else
    begin
      if (ex_valid_o && wb_ready_i) begin
        ex_wb_pipe_o.instr_valid <= 1'b1;
        // Deassert rf_we in case of illegal csr instruction or
        // when the first half of a misaligned/split LSU goes to WB.
        ex_wb_pipe_o.rf_we       <= (csr_illegal_i     ||
                                    lsu_split_i)       ? 1'b0 : id_ex_pipe_i.rf_we;
        ex_wb_pipe_o.lsu_en      <= id_ex_pipe_i.lsu_en;

        if (id_ex_pipe_i.rf_we) begin
          ex_wb_pipe_o.rf_waddr <= id_ex_pipe_i.rf_waddr;
          if (!id_ex_pipe_i.lsu_en) begin
            ex_wb_pipe_o.rf_wdata <= rf_wdata_o;
          end
        end

        // Update signals for CSR access in WB
        // deassert csr_en in case of illegal csr instruction
        ex_wb_pipe_o.csr_en     <= csr_illegal_i ? 1'b0 : id_ex_pipe_i.csr_en;
        if (id_ex_pipe_i.csr_en) begin
          ex_wb_pipe_o.csr_addr  <= id_ex_pipe_i.alu_operand_b[11:0];
          ex_wb_pipe_o.csr_wdata <= id_ex_pipe_i.alu_operand_a;
          ex_wb_pipe_o.csr_op     <= id_ex_pipe_i.csr_op;
        end

        // Propagate signals needed for exception handling in WB
        // TODO:OK:low Clock gating of pc if no existing exceptions
        //          and LSU it not in use
        ex_wb_pipe_o.pc             <= id_ex_pipe_i.pc;
        ex_wb_pipe_o.instr          <= id_ex_pipe_i.instr;
        ex_wb_pipe_o.instr_meta     <= instr_meta_n;

        // CSR illegal instruction detected in this stage, OR'ing in the status
        ex_wb_pipe_o.illegal_insn   <= id_ex_pipe_i.illegal_insn || csr_illegal_i;

        ex_wb_pipe_o.ebrk_insn      <= id_ex_pipe_i.ebrk_insn;
        ex_wb_pipe_o.wfi_insn       <= id_ex_pipe_i.wfi_insn;
        ex_wb_pipe_o.ecall_insn     <= id_ex_pipe_i.ecall_insn;
        ex_wb_pipe_o.fencei_insn    <= id_ex_pipe_i.fencei_insn;
        ex_wb_pipe_o.mret_insn      <= id_ex_pipe_i.mret_insn;
        ex_wb_pipe_o.dret_insn      <= id_ex_pipe_i.dret_insn;
        ex_wb_pipe_o.trigger_match  <= id_ex_pipe_i.trigger_match;

        // eXtension interface
        ex_wb_pipe_o.xif_en         <= id_ex_pipe_i.xif_en;
        ex_wb_pipe_o.xif_id         <= id_ex_pipe_i.xif_id;
      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we introduce a bubble
        ex_wb_pipe_o.instr_valid <= 1'b0;
      end
    end
  end

  // LSU inputs are valid when LSU is enabled; LSU outputs need to remain valid until downstream stage is ready
  assign lsu_valid_o = lsu_en_gated;
  assign lsu_ready_o = wb_ready_i;

  // ALU is single-cycle and output is therefore immediately valid (no handshake to optimize timing)
  assign alu_valid = 1'b1;
  assign alu_ready = wb_ready_i;

  // CSR is single-cycle and output is therefore immediately valid (no handshake to optimize timing)
  assign csr_valid = 1'b1;
  assign csr_ready = wb_ready_i;

  // EX stage is ready immediately when killed and otherwise when its functional units are ready,
  // unless the stage is being halted. The late (data_rvalid_i based) downstream wb_ready_i signal
  // fans into the ready signals of all functional units.

  assign ex_ready_o = ctrl_fsm_i.kill_ex || (alu_ready && csr_ready && mul_ready && div_ready && lsu_ready_i && xif_ready && !ctrl_fsm_i.halt_ex);

  // TODO:ab Reconsider setting alu_en for exception/trigger instead of using 'previous_exception'
  assign ex_valid_o = ((id_ex_pipe_i.alu_en && alu_valid)   ||
                       (id_ex_pipe_i.lsu_en && lsu_valid_i) ||
                       (id_ex_pipe_i.mul_en && mul_valid)   ||
                       (id_ex_pipe_i.div_en && div_valid)   ||
                       (id_ex_pipe_i.csr_en && csr_valid)   ||
                       (id_ex_pipe_i.xif_en && xif_valid)   ||
                       previous_exception // todo:ab:remove
                      ) && instr_valid;

  //---------------------------------------------------------------------------
  // eXtension interface
  //---------------------------------------------------------------------------

  // XIF is modeled as a functional unit that occupies EX for a single cycle no matter whether the
  // result handshake is received in a single cycle or not.
  assign xif_valid = 1'b1;
  assign xif_ready = wb_ready_i;

  // TODO: The EX stage needs to be ready to receive a result from a single cycle offloaded
  // instruction. In such case the result can be written into ex_wb_pipe_i.rf_wdata (as if the XIF
  // is a functional unit living in EX) and then typically a cycle later the result would get
  // written from ex_wb_pipe_i.rf_wdata into the registerfile.

endmodule // cv32e40s_ex_stage