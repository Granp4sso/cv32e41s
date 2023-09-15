
// Copyright 2023 University of Naples, Federico II

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Antonio Emmanuele                                          //
//                                                                            //
// Additional contributions by:                                               //
//                 Stefano Mercogliano - stefano.mercogliano@unina.it         //
//                                                                            //
//                                                                            //
// Description:   The Tightly Couple Memory implementation. It can be         //
//                used as a data or instruction memory and can be addressed   //
//                only by the core itself. It is not shared nor coherent.     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e41s_tcm 
#(
  parameter A_WID     = 32,
  parameter MEM_SIZE  = 4096,
  parameter D_WID     = 32

) (
    input               clk_i,
    input               rst_ni,
    // TCM is basically a dual port memory, the first post can be a data port used 
    // to access data ( and to write instructions ) and the second an instruction port
    // or maybe both.
    input               a_req_i, 
    input               a_we_i,
    input        [ 3:0] a_be_i,
    input        [31:0] a_addr_i,
    input        [31:0] a_wdata_i,
    output logic        a_rvalid_o,
    output logic [31:0] a_rdata_o,

    input               b_req_i,
    input               b_we_i,
    input        [ 3:0] b_be_i,
    input        [31:0] b_addr_i,
    input        [31:0] b_wdata_i,
    output logic        b_rvalid_o,
    output logic [31:0] b_rdata_o
);

  // Convert byte mask to SRAM bit mask.
  // be is the mask of bytes to write,the for loop converts it into a 
  // mask of bits.
  logic [31:0] a_wmask;
  logic [31:0] b_wmask;
  always_comb begin
    for (int i = 0 ; i < 4 ; i++) begin
      // mask for read data
      a_wmask[8*i+:8] = {8{a_be_i[i]}};
      b_wmask[8*i+:8] = {8{b_be_i[i]}};
    end
  end
  // Manages the handshake, one clock cycle after the valid signal becomes high 
  // it is basically in this implementation a delayed version of the request 
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      a_rvalid_o <= '0;
      b_rvalid_o <= '0;
    end else begin
      a_rvalid_o <= a_req_i;
      b_rvalid_o <= b_req_i;
    end
  end
  typedef logic [31:0] memPkt;
  cv32e41s_ram_struct #(
    .A_WID(A_WID),
    .MEM_SIZE(MEM_SIZE),
    .D_WID(D_WID)
  ) ram
  (
    // common clocks
    .clka_i(clk_i), 
    .clkb_i(clk_i), 
    // in the obi protocol if req==1 then we must read, if we==0, else if we==1  write
    // Mapping this to internal mem means that the enable is the req and the 
    // we is exactly the we .
    .wea_i(a_we_i),  
    .web_i(b_we_i), 
    .ena_i(a_req_i), 
    .enb_i(b_req_i),   
    .addra_i(a_addr_i),    // Write/Read Address on port A 
    .addrb_i(b_addr_i),    // Write/Read Address on port B 
    .da_i(a_wdata_i),
    .db_i(b_wdata_i),      // Data Inputs on ports a and b
    .douta_o(a_rdata_o),
    .doutb_o(b_rdata_o)    // Data outputs on port a and b
  );

endmodule
