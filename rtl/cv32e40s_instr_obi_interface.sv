// Copyright 2020 Silicon Labs, Inc.
//
// This file, and derivatives thereof are licensed under the
// Solderpad License, Version 2.0 (the "License").
//
// Use of this file means you agree to the terms and conditions
// of the license and are in full compliance with the License.
//
// You may obtain a copy of the License at:
//
//     https://solderpad.org/licenses/SHL-2.0/
//
// Unless required by applicable law or agreed to in writing, software
// and hardware implementations thereof distributed under the License
// is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS
// OF ANY KIND, EITHER EXPRESSED OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Arjan Bink - arjan.bink@silabs.com                         //
//                                                                            //
// Design Name:    OBI (Open Bus Interface)                                   //
// Project Name:   CV32E40P                                                   //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Open Bus Interface adapter. Translates transaction request //
//                 on the trans_* interface into an OBI A channel transfer.   //
//                 The OBI R channel transfer translated (i.e. passed on) as  //
//                 a transaction response on the resp_* interface.            //
//                                                                            //
//                 This adapter does not limit the number of outstanding      //
//                 OBI transactions in any way.                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40s_instr_obi_interface import cv32e40s_pkg::*;
#(  parameter int          MAX_OUTSTANDING = 2
 )
(
  input  logic           clk,
  input  logic           rst_n,

  // Transaction request interface
  input  logic           trans_valid_i,
  output logic           trans_ready_o,
  input  obi_inst_req_t  trans_i,

  // Transaction response interface
  output logic           resp_valid_o,          // Note: Consumer is assumed to be 'ready' whenever resp_valid_o = 1
  output obi_inst_resp_t resp_o,

  output logic           integrity_err_o,       // parity error or rchk error, fans into alert_major_o

  input xsecure_ctrl_t   xsecure_ctrl_i,

  // OBI interface
  if_c_obi.master        m_c_obi_instr_if
);


  typedef struct packed {
    logic        integrity;
    logic        gnterr;
  } int_gnt_fifo_t;

  // FIFO is 1 bit deeper than the maximum value of cnt_q
  // Index 0 is tied low to enable direct use of cnt_q to pick correct FIFO index.
  int_gnt_fifo_t [MAX_OUTSTANDING:0] parity_fifo_q;
  int_gnt_fifo_t parity_fifo_input;

  obi_if_state_e state_q, next_state;

  logic [11:0] achk;                            // Address phase checksum

  logic gntpar_err;                             // gnt parity error (immediate)
  logic gntpar_err_q;                           // gnt parity error (sticky for waited grants)
  logic rvalidpar_err;                          // rvalid parity error (immediate during response phase)
  logic gntpar_err_resp;                        // grant error with reponse timing (output of fifo)

  // Outstanding counter signals
  logic [1:0]     cnt_q;                        // Transaction counter
  logic [1:0]     next_cnt;                     // Next value for cnt_q
  logic           count_up;
  logic           count_down;

  logic           rchk_en;                      // Enable rchk (rvalid && integrity)
  logic           rchk_err;                     // Local rchk error signal

  //////////////////////////////////////////////////////////////////////////////
  // OBI R Channel
  //////////////////////////////////////////////////////////////////////////////

  // The OBI R channel signals are passed on directly on the transaction response
  // interface (resp_*). It is assumed that the consumer of the transaction response
  // is always receptive when resp_valid_o = 1 (otherwise a response would get dropped)
  // OBI response signals for parity error, integrity from PMA and rchk error are appended.

  assign resp_valid_o       = m_c_obi_instr_if.s_rvalid.rvalid;

  always_comb begin
    resp_o  = m_c_obi_instr_if.resp_payload;
    resp_o.parity_err  = rvalidpar_err || gntpar_err_resp;
    resp_o.rchk_err    = rchk_err;
    resp_o.integrity   = parity_fifo_q[cnt_q].integrity;
  end

  //////////////////////////////////////////////////////////////////////////////
  // OBI A Channel
  //////////////////////////////////////////////////////////////////////////////

  // OBI A channel registers (to keep A channel stable)
  obi_inst_req_t        obi_a_req_q;

  // If the incoming transaction itself is not stable; use an FSM to make sure that
  // the OBI address phase signals are kept stable during non-granted requests.

  //////////////////////////////////////////////////////////////////////////////
  // OBI FSM
  //////////////////////////////////////////////////////////////////////////////

  // FSM (state_q, next_state) to control OBI A channel signals.

  always_comb
  begin
    next_state = state_q;

    case(state_q)

      // Default (transparent) state. Transaction requests are passed directly onto the OBI A channel.
      TRANSPARENT:
      begin
        if (m_c_obi_instr_if.s_req.req && !m_c_obi_instr_if.s_gnt.gnt) begin
          // OBI request not immediately granted. Move to REGISTERED state such that OBI address phase
          // signals can be kept stable while the transaction request (trans_*) can possibly change.
          next_state = REGISTERED;
        end
      end // case: TRANSPARENT

      // Registered state. OBI address phase signals are kept stable (driven from registers).
      REGISTERED:
      begin
        if (m_c_obi_instr_if.s_gnt.gnt) begin
          // Received grant. Move back to TRANSPARENT state such that next transaction request can be passed on.
          next_state = TRANSPARENT;
        end
      end // case: REGISTERED
      default: ;
    endcase
  end

  always_comb
  begin
    if (state_q == TRANSPARENT) begin
      m_c_obi_instr_if.s_req.req        = trans_valid_i;                // Do not limit number of outstanding transactions
      m_c_obi_instr_if.req_payload      = trans_i;
      m_c_obi_instr_if.req_payload.achk = achk;
    end else begin
      // state_q == REGISTERED
      m_c_obi_instr_if.s_req.req   = 1'b1;                              // Never retract request
      m_c_obi_instr_if.req_payload = obi_a_req_q;
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // Integrity // todo: ensure this will not get optimized away
  //////////////////////////////////////////////////////////////////////////////

  always_comb begin
    achk = {
      ^{8'b0},                                         // wdata[31:24] = 8'b0
      ^{8'b0},                                         // wdata[23:16] = 8'b0
      ^{8'b0},                                         // wdata[15:8] = 8'b0
      ^{8'b0},                                         // wdata[7:0] = 8'b0
      ^{6'b0},                                         // atop[5:0] = 6'b0
      ~^{m_c_obi_instr_if.req_payload.dbg},
      ~^{4'b1111, 1'b0},                                // be[3:0] = 4'b1111, we = 1'b0
      ~^{m_c_obi_instr_if.req_payload.prot[2:0], m_c_obi_instr_if.req_payload.memtype[1:0]},
      ^{m_c_obi_instr_if.req_payload.addr[31:24]},
      ^{m_c_obi_instr_if.req_payload.addr[23:16]},
      ^{m_c_obi_instr_if.req_payload.addr[15:8]},
      ^{m_c_obi_instr_if.req_payload.addr[7:2], 2'b00} // Bits 1:0 are tied to zero in the core level.
    };
  end

 assign  m_c_obi_instr_if.s_req.reqpar = !m_c_obi_instr_if.s_req.req;

  //////////////////////////////////////////////////////////////////////////////
  // Registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      state_q       <= TRANSPARENT;
      obi_a_req_q   <= OBI_INST_REQ_RESET_VAL;
    end
    else
    begin
      state_q       <= next_state;
      if ((state_q == TRANSPARENT) && (next_state == REGISTERED)) begin
        // Keep OBI A channel signals stable throughout REGISTERED state
        obi_a_req_q <= m_c_obi_instr_if.req_payload;
      end
    end
  end

  // Always ready to accept a new transfer requests when previous A channel
  // transfer has been granted. Note that cv32e40s_obi_interface does not limit
  // the number of outstanding transactions in any way.
  assign trans_ready_o = (state_q == TRANSPARENT);

  /////////////////////////////////////////////////////////////
  // Outstanding transactions counter
  // Used for tracking parity errors and integrity attribute
  /////////////////////////////////////////////////////////////
  assign count_up = m_c_obi_instr_if.s_req.req && m_c_obi_instr_if.s_gnt.gnt;  // Increment upon accepted transfer request
  assign count_down = m_c_obi_instr_if.s_rvalid.rvalid;                        // Decrement upon accepted transfer response

  always_comb begin
    case ({count_up, count_down})
      2'b00 : begin
        next_cnt = cnt_q;
      end
      2'b01 : begin
        next_cnt = cnt_q - 1'b1;
      end
      2'b10 : begin
        next_cnt = cnt_q + 1'b1;
      end
      2'b11 : begin
        next_cnt = cnt_q;
      end
      default:;
    endcase
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      cnt_q <= '0;
    end else begin
      cnt_q <= next_cnt;
    end
  end

  //////////////////////////////////////
  // Track parity errors
  //////////////////////////////////////

  // Always check gnt parity
  // alert_major will not update when in reset
  assign gntpar_err = (m_c_obi_instr_if.s_gnt.gnt == m_c_obi_instr_if.s_gnt.gntpar);


  // gntpar_err_q is a sticky gnt error bit.
  // Any gnt parity error detected during req will be remembered and propagated
  // to the fifo when the address phase ends.
  always_ff @ (posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      gntpar_err_q <= '0;
    end
    else begin
      if (m_c_obi_instr_if.s_req.req) begin
        // Address phase active, set sticky gntpar_err if not granted
        // When granted, sticky bit will be cleared for the next address phase
        if (!m_c_obi_instr_if.s_gnt.gnt) begin
          gntpar_err_q <= gntpar_err || gntpar_err_q;
        end else begin
          gntpar_err_q <= 1'b0;
        end
      end
    end
  end

  // FIFO to keep track of gnt parity errors for outstanding transactions

  assign parity_fifo_input.integrity = trans_i.integrity;
  assign parity_fifo_input.gnterr    = (gntpar_err || gntpar_err_q);

  always_ff @ (posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      parity_fifo_q <= '0;
    end
    else begin
      if (m_c_obi_instr_if.s_req.req && m_c_obi_instr_if.s_gnt.gnt) begin
        // Accepted address phase, populate FIFO with gnt parity error and PMA integrity bit
        parity_fifo_q <= {parity_fifo_q[MAX_OUTSTANDING-1:1], parity_fifo_input, 2'b00};
      end
    end
  end


  // Enable rchk when in response phase and cpuctrl.integrity is set
  assign rchk_en = m_c_obi_instr_if.s_rvalid.rvalid && xsecure_ctrl_i.cpuctrl.integrity;
  cv32e40s_rchk_check
  #(
      .RESP_TYPE (obi_inst_resp_t)
  )
  rchk_i
  (
    .resp_i (resp_o),  // Using local output, as is has the PMA integrity bit appended from the fifo. Otherwise inputs from bus.
    .enable (rchk_en),
    .err    (rchk_err)
  );

  // grant parity for response is read from the fifo
  assign gntpar_err_resp = parity_fifo_q[cnt_q].gnterr;

  // Checking rvalid parity
  // integrity_err_o will go high immediately, while the rvalidpar_err for the instruction
  // will only propagate when rvalid==1.
  assign rvalidpar_err = (m_c_obi_instr_if.s_rvalid.rvalid == m_c_obi_instr_if.s_rvalid.rvalidpar);

  // Set integrity error outputs.
  // rchk_err: recomputed checksum mismatch when rvalid=1 and PMA has integrity set for the transaction
  // rvalidpar_err: mismatch on rvalid parity bit at any time
  // gntpar_err: mismatch on gnt parity bit at any time
  assign integrity_err_o = rchk_err || rvalidpar_err || gntpar_err;




endmodule
