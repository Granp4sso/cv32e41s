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

module cv32e41s_data_obi_interface import cv32e41s_pkg::*;
#(
    parameter int unsigned  MAX_OUTSTANDING   = 2
 )
(
  input  logic        clk,
  input  logic        rst_n,

  // Transaction request interface
  input  logic         trans_valid_i,
  output logic         trans_ready_o,
  input obi_data_req_t trans_i,

  // Transaction response interface
  output logic           resp_valid_o,          // Note: Consumer is assumed to be 'ready' whenever resp_valid_o = 1
  output obi_data_resp_t resp_o,

  output logic           integrity_err_o,
  output logic           protocol_err_o,

  input xsecure_ctrl_t   xsecure_ctrl_i,

  // OBI interface
  input  cv32e41s_if_c_obi_data_master_i   c_obi_data_if_ma_i,
  output cv32e41s_if_c_obi_data_master_o   c_obi_data_if_ma_o
);


  // Parity and rchk error signals
  logic       gntpar_err;
  logic       rvalidpar_err_resp;                          // rvalid parity error (immediate during response phase)
  logic       gntpar_err_resp;                        // grant error with reponse timing (output of fifo)
  logic       rchk_err_resp;

  logic       integrity_resp;

  logic       protocol_err;                 // Set if rvalid arrives when no outstanding transactions are active


  //////////////////////////////////////////////////////////////////////////////
  // OBI R Channel
  //////////////////////////////////////////////////////////////////////////////

  // The OBI R channel signals are passed on directly on the transaction response
  // interface (resp_*). It is assumed that the consumer of the transaction response
  // is always receptive when resp_valid_o = 1 (otherwise a response would get dropped)

  assign resp_valid_o = c_obi_data_if_ma_i.s_rvalid.rvalid;

  always_comb begin
    resp_o               = c_obi_data_if_ma_i.resp_payload;
    resp_o.integrity_err = rvalidpar_err_resp || gntpar_err_resp || rchk_err_resp;
    resp_o.integrity     = integrity_resp;
  end

  //////////////////////////////////////////////////////////////////////////////
  // OBI A Channel
  //////////////////////////////////////////////////////////////////////////////

  // If the incoming transaction itself is stable, then it satisfies the OBI protocol
  // and signals can be passed to/from OBI directly.
  always_comb
  begin
    c_obi_data_if_ma_o.s_req.req        = trans_valid_i;
    c_obi_data_if_ma_o.req_payload      = trans_i;

    // Integrity // todo: ensure this will not get optimized away
    c_obi_data_if_ma_o.req_payload.achk = {
                                        ^{c_obi_data_if_ma_o.req_payload.wdata[31:24]},
                                        ^{c_obi_data_if_ma_o.req_payload.wdata[23:16]},
                                        ^{c_obi_data_if_ma_o.req_payload.wdata[15:8]},
                                        ^{c_obi_data_if_ma_o.req_payload.wdata[7:0]},
                                        ~^{c_obi_data_if_ma_o.req_payload.dbg},
                                        ^{6'b0},                                         // atop[5:0] = 6'b0
                                        ^{8'b0},                                         // mid[7:0]  = 8'b0
                                        ~^{c_obi_data_if_ma_o.req_payload.be[3:0], c_obi_data_if_ma_o.req_payload.we},
                                        ~^{c_obi_data_if_ma_o.req_payload.prot[2:0], c_obi_data_if_ma_o.req_payload.memtype[1:0]},
                                        ^{c_obi_data_if_ma_o.req_payload.addr[31:24]},
                                        ^{c_obi_data_if_ma_o.req_payload.addr[23:16]},
                                        ^{c_obi_data_if_ma_o.req_payload.addr[15:8]},
                                        ^{c_obi_data_if_ma_o.req_payload.addr[7:0]}
                                        };

  end

  assign trans_ready_o = c_obi_data_if_ma_i.s_gnt.gnt;

  assign c_obi_data_if_ma_o.s_req.reqpar = !c_obi_data_if_ma_o.s_req.req;


  /////////////////
  // Integrity
  /////////////////

  // Always check gnt parity
  // alert_major will not update when in reset
  assign gntpar_err = (c_obi_data_if_ma_i.s_gnt.gnt == c_obi_data_if_ma_i.s_gnt.gntpar);

  cv32e41s_obi_integrity_fifo
    #(
        .MAX_OUTSTANDING   (MAX_OUTSTANDING  ),
        .RESP_TYPE         (obi_data_resp_t  )
     )
    integrity_fifo_i
    (
      .clk                (clk                ),
      .rst_n              (rst_n              ),

      // gnt parity error
      .gntpar_err_i       (gntpar_err         ),

      // Transaction inputs
      .trans_integrity_i  (trans_i.integrity  ),
      .trans_we_i         (trans_i.we         ),

      // Xsecure
      .xsecure_ctrl_i     (xsecure_ctrl_i     ),

      // Response phase properties
      .gntpar_err_resp_o  (gntpar_err_resp    ),
      .integrity_resp_o   (integrity_resp     ),
      .rchk_err_resp_o    (rchk_err_resp      ),

      .protocol_err_o     (protocol_err       ),

      // OBI interface
      .obi_req_i          (c_obi_data_if_ma_o.s_req.req       ),
      .obi_gnt_i          (c_obi_data_if_ma_i.s_gnt.gnt       ),
      .obi_rvalid_i       (c_obi_data_if_ma_i.s_rvalid.rvalid ),
      .obi_resp_i         (resp_o                          )
    );

  // Checking rvalid parity
  // alert_major_o will go high immediately
  assign rvalidpar_err_resp = (c_obi_data_if_ma_i.s_rvalid.rvalid == c_obi_data_if_ma_i.s_rvalid.rvalidpar);

  assign integrity_err_o = rchk_err_resp || rvalidpar_err_resp || gntpar_err;
  assign protocol_err_o  = protocol_err;

endmodule
