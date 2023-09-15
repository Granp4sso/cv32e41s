// Copyright 2021 Silicon Labs, Inc.
//
// This file, and derivatives thereof are licensed under the
// Solderpad License, Version 2.0 (the "License");
// Use of this file means you agree to the terms and conditions
// of the license and are in full compliance with the License.
// You may obtain a copy of the License at
//
//     https://solderpad.org/licenses/SHL-2.0/
//
// Unless required by applicable law or agreed to in writing, software
// and hardware implementations thereof
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, EITHER EXPRESSED OR IMPLIED.
// See the License for the specific language governing permissions and
// limitations under the License.

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Authors:        Oivind Ekelund - oivind.ekelund@silabs.com                 //
//                                                                            //
// Additional contributions by:                                               //
//                 Stefano Mercogliano - stefano.mercogliano@unina.it         //
//                 Giovanni Celentano  - gio.celentano@hotmail.it             //
//                 Maurizio Gaudino    - gaudinomaurizio4@gmail.com           //
//                                                                            //
// Description:    MPU (Memory Protection Unit)                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e41s_mpu_test import cv32e41s_pkg::*;
  #(  parameter bit          IF_STAGE                     = 1,
      parameter type         CORE_REQ_TYPE                = obi_inst_req_t,
      parameter type         CORE_RESP_TYPE               = inst_resp_t,
      parameter type         BUS_RESP_TYPE                = obi_inst_resp_t,
      parameter int unsigned PMP_GRANULARITY              = 0,
      parameter int          PMP_NUM_REGIONS              = 0,
      parameter int          PMA_NUM_REGIONS              = 0,
      parameter pma_cfg_t    PMA_CFG[PMA_NUM_REGIONS-1:0] = '{default:PMA_R_DEFAULT},
      parameter bit          DEBUG                        = 1,
      parameter logic [31:0] DM_REGION_START              = 32'hF0000000,
      parameter logic [31:0] DM_REGION_END                = 32'hF0003FFF,
      // Custom XPMP extension
      parameter xpmp_pmr_e        XPMP_PMR_ENABLE   = PMR_NONE,
      parameter xpmp_pmr_enc_e    XPMP_PMR_ENCODING = PMR_ENC_LIN,
      parameter xpmp_trie_e       XPMP_TRIE         = PMP_TRIE_NONE,
      parameter xpmp_trie_cmp_e   XPMP_TRIE_CMP     = PMP_TRIE_NOTCPM
  )
  (
   input logic  clk,
   input logic  rst_n,

   input logic  misaligned_access_i, // Indicate that ongoing access is part of a misaligned access

   // Interface towards bus interface
   input logic  bus_trans_ready_i,
   output logic bus_trans_valid_o,
   output       CORE_REQ_TYPE bus_trans_o,

   input logic  bus_resp_valid_i,
   input        BUS_RESP_TYPE bus_resp_i,

   // Interface towards core
   input logic  core_trans_valid_i,
   input logic  core_trans_pushpop_i,
   output logic core_trans_ready_o,
   input        CORE_REQ_TYPE core_trans_i,

   output logic core_resp_valid_o,
   output       CORE_RESP_TYPE core_resp_o,

   // PMP CSR's
   input        pmp_csr_t csr_pmp_i,
   
   // RPM implicit accesses
    output logic			       							pmp_imp_req_o, 
    output logic [31:0]			 							pmp_imp_addr_o,
    input  logic        									pmp_imp_rvalid_i,
    input  logic [31:0] 									pmp_imp_rdata_b0_i,
    input  logic [31:0] 									pmp_imp_rdata_b1_i,

   // Indication from the core that there will be one pending transaction in the next cycle
   input logic  core_one_txn_pend_n,

   // Indication from the core that MPU errors should be reported after all in flight transactions
   // are complete
   input logic  core_mpu_err_wait_i,

   // Report MPU errors to the core immediatly (used in case core_mpu_err_wait_i is not asserted)
   output logic core_mpu_err_o
   );

  localparam bit PMP = SECURE;

  logic        pma_err;
  logic        pmp_err;
  logic        pmp_busy;
  logic        mpu_err;
  logic        mpu_block_core;
  logic        mpu_block_bus;
  logic        mpu_err_trans_valid;
  logic        mpu_err_trans_ready;
  mpu_status_e mpu_status;
  mpu_state_e  state_q, state_n;
  logic        bus_trans_cacheable;
  logic        bus_trans_bufferable;
  logic        bus_trans_integrity;
  logic        core_trans_we;
  pmp_req_e    pmp_req_type;
  logic [33:0] pmp_req_addr;
  privlvl_t    pmp_priv_lvl;
  logic        instr_fetch_access;
  logic        load_access;
  logic        wpt_match;
  logic        core_trans_debug_region;
  logic [31:0] pmr_reloc_address;
  logic [31:0] checked_address;

  // PMP trie signals
  logic         pmp_bus_req;
  CORE_REQ_TYPE pmp_trans;
  logic         mpu_pmp_block;
  logic         mpu_fault_flag;
  logic         pmp_trie_en;

  // Detect a debug mode transaction to the Debug Module region
  assign core_trans_debug_region = (pmp_trans.addr >= DM_REGION_START) && (pmp_trans.addr <= DM_REGION_END) && pmp_trans.dbg;

  // PMP-trie depending conditions
  // If the PMP trie is active and we are not running in M-mode, the request signal
  // will be high only if no exception occurs, therefore the mpu_err alone is enough.
  // Otherwise we need also the pmp request, which in this case would be exactly the core request
  assign pmp_trie_en = csr_pmp_i.xpmpcfg.trie_support && csr_pmp_i.xpmpcfg.trie_en && pmp_priv_lvl != PRIV_LVL_M;
  assign mpu_fault_flag = ( pmp_trie_en ) ? mpu_err : mpu_err && pmp_bus_req;

  // If the PMP is busy, stall the core. This happens only if the PMP trie is supported,
  // otherwise pmp_busy will always be 0.
  assign mpu_pmp_block = ~mpu_err && pmp_busy;

  // FSM that will "consume" transfers failing PMA or PMP checks.
  // Upon failing checks, this FSM will prevent the transfer from going out on the bus
  // and wait for all in flight bus transactions to complete while blocking new transfers.
  // When all in flight transactions are complete, it will respond with the correct status before
  // allowing new transfers to go through.
  // The input signal core_one_txn_pend_n indicates that there, from the core's point of view,
  // will be one pending transaction in the next cycle. Upon MPU error, this transaction
  // will be completed by this FSM
  always_comb begin

    state_n             = state_q;
    mpu_status          = MPU_OK;
    mpu_block_core      = 1'b0;
    mpu_block_bus       = 1'b0;
    mpu_err_trans_valid = 1'b0;
    mpu_err_trans_ready = 1'b0;

    case(state_q)
      MPU_IDLE: begin

        if(mpu_pmp_block) begin

          mpu_block_core      = 1'b1;
          mpu_block_bus       = 1'b1;

        end else if (mpu_fault_flag) begin
          // Block transfer from going out on the bus.
          mpu_block_bus  = 1'b1;

          // Signal to the core that the transfer was accepted (but will be consumed by the MPU)
          mpu_err_trans_ready = 1'b1;

          if (core_mpu_err_wait_i) begin
            if(core_trans_we) begin
              // MPU error on write
              state_n = (core_one_txn_pend_n | pmp_trie_en) ? MPU_WR_ERR_RESP : MPU_WR_ERR_WAIT;
            end
            else begin
              // MPU error on read
              state_n = (core_one_txn_pend_n | pmp_trie_en) ? MPU_RE_ERR_RESP : MPU_RE_ERR_WAIT;
            end
          end

        end
      end
      MPU_RE_ERR_WAIT, MPU_WR_ERR_WAIT: begin

        // Block new transfers while waiting for in flight transfers to complete
        mpu_block_bus  = 1'b1;
        mpu_block_core = 1'b1;

        if (core_one_txn_pend_n) begin
          state_n = (state_q == MPU_RE_ERR_WAIT) ? MPU_RE_ERR_RESP : MPU_WR_ERR_RESP;
        end
      end
      MPU_RE_ERR_RESP, MPU_WR_ERR_RESP: begin

        // Keep blocking new transfers
        mpu_block_bus  = 1'b1;
        mpu_block_core = 1'b1;

        // Set up MPU error response towards the core
        mpu_err_trans_valid = 1'b1;
        mpu_status = (state_q == MPU_RE_ERR_RESP) ? MPU_RE_FAULT : MPU_WR_FAULT;

        // Go back to IDLE uncoditionally.
        // The core is expected to always be ready for the response
        state_n = MPU_IDLE;

      end
      default: ;
    endcase
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if (rst_n == 1'b0) begin
      state_q <= MPU_IDLE;
    end
    else begin
      state_q <= state_n;
    end
  end

  // Forward transaction request towards bus interface
  assign bus_trans_valid_o = pmp_bus_req && !mpu_block_bus; //core_trans_valid_i && !mpu_block_bus;

  always_comb begin
    bus_trans_o             = pmp_trans;
    bus_trans_o.addr        = checked_address;
    bus_trans_o.memtype[0]  = bus_trans_bufferable;
    bus_trans_o.memtype[1]  = bus_trans_cacheable;
    bus_trans_o.integrity   = bus_trans_integrity;
  end

  // Forward transaction response towards core
  assign core_resp_valid_o      = bus_resp_valid_i || mpu_err_trans_valid;
  assign core_resp_o.mpu_status = mpu_status;


  // Report MPU errors to the core immediately
  assign core_mpu_err_o = mpu_err & pmp_trie_en;

  // Signal ready towards core
  assign core_trans_ready_o = (bus_trans_ready_i && !mpu_block_core) || mpu_err_trans_ready;

  // PMA - Physical Memory Attribution
  cv32e41s_pma
    #(.PMA_NUM_REGIONS          ( PMA_NUM_REGIONS         ),
      .PMA_CFG                  ( PMA_CFG                 )
  )
  pma_i
    (
    .trans_addr_i               ( pmp_trans.addr       ),
    .trans_debug_region_i       ( core_trans_debug_region ),
    .trans_pushpop_i            ( core_trans_pushpop_i    ),
    .instr_fetch_access_i       ( instr_fetch_access      ),
    .misaligned_access_i        ( misaligned_access_i     ),
    .load_access_i              ( load_access             ),
    .pma_err_o                  ( pma_err                 ),
    .pma_integrity_o            ( bus_trans_integrity     ),
    .pma_bufferable_o           ( bus_trans_bufferable    ),
    .pma_cacheable_o            ( bus_trans_cacheable     )
  );

  assign pmp_req_addr = {2'b00, core_trans_i.addr};
  assign pmp_priv_lvl = privlvl_t'(core_trans_i.prot[2:1]);

  generate
    if (PMP) begin: xpmp
      cv32e41s_xpmp #(
        // Parameters
        .PMP_GRANULARITY                    ( PMP_GRANULARITY         ),
        .PMP_NUM_REGIONS                    ( PMP_NUM_REGIONS         ),
        .XPMP_PMR_ENABLE                    ( XPMP_PMR_ENABLE         ),              
        .XPMP_PMR_ENCODING                  ( XPMP_PMR_ENCODING       ),   
        .XPMP_TRIE                          ( XPMP_TRIE               ),   
        .XPMP_TRIE_CMP                      ( XPMP_TRIE_CMP           ),  
        .CORE_REQ_TYPE                      ( CORE_REQ_TYPE           )   
      ) xpmp_i (

        .pmp_req_err_o                      ( pmp_err                 ),
        .clk                                ( clk                     ),
        .rst_n                              ( rst_n                   ),
        .pmp_req_type_i                     ( pmp_req_type            ),
        .csr_pmp_i                          ( csr_pmp_i               ),
        .priv_lvl_i                         ( pmp_priv_lvl            ),
        .pmp_req_addr_i                     ( pmp_req_addr            ),
        .pmr_reloc_addr_o                   ( pmr_reloc_address       ),   
        .pmp_req_debug_region_i             ( core_trans_debug_region ),

        // pmp-trie signals
        .pmp_busy_o                         ( pmp_busy                ),
        .pmp_imp_req_o                      ( pmp_imp_req_o           ), 
        .pmp_imp_addr_o                     ( pmp_imp_addr_o          ),
        .pmp_imp_rvalid_i                   ( pmp_imp_rvalid_i        ),
        .pmp_imp_rdata_b0_i                 ( pmp_imp_rdata_b0_i      ),
        .pmp_imp_rdata_b1_i                 ( pmp_imp_rdata_b1_i      ),

        // pmp-trie transaction signals
        .core_trans_i                       ( core_trans_i            ),
        .core_req_i                         ( core_trans_valid_i      ),
        .pmp_trans_o                        ( pmp_trans               ),
        .pmp_req_o                          ( pmp_bus_req             )
         
         );


    end
    else begin: no_pmp
      assign pmp_err = 1'b0;
      assign pmr_reloc_address = 32'b0; 
    end
  endgenerate

  assign mpu_err = pmp_err || pma_err;
  assign checked_address = pmr_reloc_address;

  // Writes are only supported on the data interface
  // Tie to 1'b0 if this MPU is instantiatied in the IF stage
  generate
    if (IF_STAGE) begin: mpu_if
      assign instr_fetch_access     = 1'b1;
      assign load_access            = 1'b0;
      assign core_trans_we          = 1'b0;
      assign core_resp_o.bus_resp   = bus_resp_i;
      assign pmp_req_type           = PMP_ACC_EXEC;
    end
    else begin: mpu_lsu
      assign instr_fetch_access       = 1'b0;
      assign load_access              = !pmp_trans.we;
      assign core_trans_we            = pmp_trans.we;
      assign core_resp_o.wpt_match    = '0; // Will be set by upstream wpt-module within load_store_unit
      assign core_resp_o.bus_resp     = bus_resp_i;
      assign pmp_req_type             = core_trans_we ? PMP_ACC_WRITE : PMP_ACC_READ;

    end
  endgenerate

// TODO:OE any way to check that the 2nd access of a failed misalgn will not reach the MPU?

endmodule
