// Copyright 2023 University of Naples, Federico II

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Antonio Emmanuele                                          //
//                                                                            //
// Additional contributions by:                                               //
//                 Stefano Mercogliano - stefano.mercogliano@unina.it         //
//                                                                            //
//                                                                            //
// Description:   this is an internal bus made for communication              //
//                of the internal core with the external environment.         //
//                It handles two different types of memory requests :         //
//                instruction fetch requests and standard memory operations.  //
//                The term data refers to the data memory and Inst to the     //
//                instruction memory.                                         //
//                The implementation mostly resembles demo system bus.        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e41s_int_bus #(
  parameter int NrDataMems    = 1,  // Number of memories containing data
  parameter int NrInstMems    = 1,  // Number of memories containing instructions
  parameter int NrHosts      = 1,   // Number of cores handled from the bus
  parameter int DataWidth    = 32,
  parameter int AddressWidth = 32
) (
  input                           clk_i,
  input                           rst_ni,
  // Data Hosts 
  input                           host_req_data_i    [NrHosts],
  output logic                    host_gnt_data_o         [NrHosts],

  input        [AddressWidth-1:0] host_addr_data_i   [NrHosts],
  input                           host_we_data_i     [NrHosts],
  input        [ DataWidth/8-1:0] host_be_data_i     [NrHosts],
  input        [   DataWidth-1:0] host_wdata_data_i  [NrHosts],
  output logic                    host_rvalid_data_o [NrHosts],
  output logic [   DataWidth-1:0] host_rdata_data_o  [NrHosts],
  output logic                    host_err_data_o    [NrHosts],
  // Inst Hosts
  input                           host_req_inst_i    [NrHosts],
  output logic                    host_gnt_inst_o    [NrHosts],
  input        [AddressWidth-1:0] host_addr_inst_i   [NrHosts],
  input                           host_we_inst_i     [NrHosts],
  input        [ DataWidth/8-1:0] host_be_inst_i     [NrHosts],
  input        [   DataWidth-1:0] host_wdata_inst_i  [NrHosts],
  output logic                    host_rvalid_inst_o [NrHosts],
  output logic [   DataWidth-1:0] host_rdata_inst_o  [NrHosts],
  output logic                    host_err_inst_o    [NrHosts],

  // Data memories 
  output logic                    mem_req_data_o    [NrDataMems],
  output logic [AddressWidth-1:0] mem_addr_data_o        [NrDataMems],
  output logic                    mem_we_data_o          [NrDataMems],
  output logic [ DataWidth/8-1:0] mem_be_data_o     [NrDataMems],
  output logic [   DataWidth-1:0] mem_wdata_data_o  [NrDataMems],
  input                           mem_rvalid_data_i [NrDataMems],
  input        [   DataWidth-1:0] mem_rdata_data_i  [NrDataMems],
  input                           mem_err_data_i    [NrDataMems],

  //Inst Memories
  output logic                    mem_req_inst_o    [NrInstMems],
  output logic [AddressWidth-1:0] mem_addr_inst_o        [NrInstMems],
  output logic                    mem_we_inst_o          [NrInstMems],
  output logic [ DataWidth/8-1:0] mem_be_inst_o          [NrInstMems],
  output logic [   DataWidth-1:0] mem_wdata_inst_o       [NrInstMems],
  input                           mem_rvalid_inst_i      [NrInstMems],
  input        [   DataWidth-1:0] mem_rdata_inst_i       [NrInstMems],
  input                           mem_err_inst_i         [NrInstMems],
  // mem address map for Data
  input        [AddressWidth-1:0] cfg_mem_data_addr_base [NrDataMems],
  input        [AddressWidth-1:0] cfg_mem_data_addr_mask [NrDataMems],
  // mem address map for Inst
  input        [AddressWidth-1:0] cfg_mem_inst_addr_base [NrInstMems],
  input        [AddressWidth-1:0] cfg_mem_inst_addr_mask [NrInstMems]
);
  // DATA BUS CONTROLLER

  localparam int unsigned NumBitsHostSelData = NrHosts > 1 ? $clog2(NrHosts) : 1;
  localparam int unsigned NumBitsMemSelData = NrDataMems > 1 ? $clog2(NrDataMems) : 1;

  logic [NumBitsHostSelData-1:0] host_sel_data_req, host_sel_data_resp;
  logic [NumBitsMemSelData-1:0] mem_sel_data_req, mem_sel_data_resp;

  // Master select prio arbiter
  // Select the last master in decreasing order
  always_comb begin
    host_sel_data_req = '0;
    for (integer host = NrHosts - 1; host >= 0; host = host - 1) begin
      if (host_req_data_i[host]) begin
        host_sel_data_req = NumBitsHostSelData'(host); 
      end
    end
  end

  // mem select
  // A mem is selected if the selected host request address is in the dev
  // space if addr and mask==base 
  always_comb begin
    mem_sel_data_req = '0;
    for (integer mem = 1; mem < NrDataMems; mem = mem + 1) begin
      if ((host_addr_data_i[host_sel_data_req] & cfg_mem_data_addr_mask[mem])
          == cfg_mem_data_addr_base[mem]) begin
        mem_sel_data_req = NumBitsMemSelData'(mem);
      end
    end
  end

  // The main purpose of this is to delay the valid, data and err signal index
  // The new host in fact must receive the output of the selected mem one 
  // clock cycle after.
  always_ff @(posedge clk_i or negedge rst_ni) begin
     if (!rst_ni) begin
        host_sel_data_resp <= '0;
        mem_sel_data_resp <= '0;
     end else begin
        // Responses are always expected 1 cycle after the request
        mem_sel_data_resp <= mem_sel_data_req;
        host_sel_data_resp <= host_sel_data_req;
     end
  end
  // Even if the host must accept mem response ( valid, data, err) one clock cycle 
  // after the host input signals must be given to the newly selected mem 
  always_comb begin
    for (integer mem = 0; mem < NrDataMems; mem = mem + 1) begin
      if (NumBitsMemSelData'(mem) == mem_sel_data_req) begin
        mem_req_data_o[mem]   = host_req_data_i[host_sel_data_req];
        mem_we_data_o[mem]    = host_we_data_i[host_sel_data_req];
        mem_addr_data_o[mem]  = host_addr_data_i[host_sel_data_req];
        mem_wdata_data_o[mem] = host_wdata_data_i[host_sel_data_req];
        mem_be_data_o[mem]    = host_be_data_i[host_sel_data_req];
      end else begin
        mem_req_data_o[mem]   = 1'b0;
        mem_we_data_o[mem]    = 1'b0;
        mem_addr_data_o[mem]  = 'b0;
        mem_wdata_data_o[mem] = 'b0;
        mem_be_data_o[mem]    = 'b0;
      end
    end
  end
  
  // After one clock cycle the new host is communicating with the newly selected mem 
  // this happens one clock cycle after the last signal  
  // If the previous block drives the input of the dev ( instantaneus ) this one drives the 
  // input of the host( out of the dev one clock cycle after.)
  always_comb begin
    for (integer host = 0; host < NrHosts; host = host + 1) begin
      host_gnt_data_o[host] = 1'b0;
      if (NumBitsHostSelData'(host) == host_sel_data_resp) begin  
        host_rvalid_data_o[host] = mem_rvalid_data_i[mem_sel_data_resp];
        host_err_data_o[host]    = mem_err_data_i[mem_sel_data_resp];
        host_rdata_data_o[host]  = mem_rdata_data_i[mem_sel_data_resp];
      end else begin
        host_rvalid_data_o[host] = 1'b0;
        host_err_data_o[host]    = 1'b0;
        host_rdata_data_o[host]  = 'b0;
      end
    end
    host_gnt_data_o[host_sel_data_req] = host_req_data_i[host_sel_data_req];
  end

  // INSTR BUS CONTROLLER

  localparam int unsigned NumBitsHostSelInst = NrHosts > 1 ? $clog2(NrHosts) : 1;
  localparam int unsigned NumBitsMemSelInst = NrInstMems > 1 ? $clog2(NrInstMems) : 1;

  logic [NumBitsHostSelInst-1:0] host_sel_inst_req, host_sel_inst_resp;
  logic [NumBitsMemSelInst-1:0] mem_sel_inst_req, mem_sel_inst_resp;

  // Master select prio arbiter
  // Select the last master in decreasing order
  always_comb begin
    host_sel_inst_req = '0;
    for (integer host = NrHosts - 1; host >= 0; host = host - 1) begin
      if (host_req_inst_i[host]) begin
        host_sel_inst_req = NumBitsHostSelInst'(host); 
      end
    end
  end

  // mem select
  // A mem is selected if the selected host request address is in the dev
  // space if addr and mask==base 
  always_comb begin
    mem_sel_inst_req = '0;
    for (integer mem = 1; mem < NrInstMems; mem = mem + 1) begin
      if ((host_addr_inst_i[host_sel_inst_req] & cfg_mem_inst_addr_mask[mem])
          == cfg_mem_inst_addr_base[mem]) begin
        mem_sel_inst_req = NumBitsMemSelInst'(mem);
      end
    end
  end

  // The main purpose of this is to delay the valid, inst and err signal index
  // The new host in fact must receive the output of the selected mem one 
  // clock cycle after.
  always_ff @(posedge clk_i or negedge rst_ni) begin
     if (!rst_ni) begin
        host_sel_inst_resp <= '0;
        mem_sel_inst_resp <= '0;
     end else begin
        // Responses are always expected 1 cycle after the request
        mem_sel_inst_resp <= mem_sel_inst_req;
        host_sel_inst_resp <= host_sel_inst_req;
     end
  end
  // Even if the host must accept mem response ( valid, inst, err) one clock cycle 
  // after the host input signals must be given to the newly selected mem 
  always_comb begin
    for (integer mem = 0; mem < NrInstMems; mem = mem + 1) begin
      if (NumBitsMemSelInst'(mem) == mem_sel_inst_req) begin
        mem_req_inst_o[mem]   = host_req_inst_i[host_sel_inst_req];
        mem_we_inst_o[mem]    = host_we_inst_i[host_sel_inst_req];
        mem_addr_inst_o[mem]  = host_addr_inst_i[host_sel_inst_req];
        mem_wdata_inst_o[mem] = host_wdata_inst_i[host_sel_inst_req];
        mem_be_inst_o[mem]    = host_be_inst_i[host_sel_inst_req];
      end else begin
        mem_req_inst_o[mem]   = 1'b0;
        mem_we_inst_o[mem]    = 1'b0;
        mem_addr_inst_o[mem]  = 'b0;
        mem_wdata_inst_o[mem] = 'b0;
        mem_be_inst_o[mem]    = 'b0;
      end
    end
  end
  
  // After one clock cycle the new host is communicating with the newly selected mem 
  // this happens one clock cycle after the last signal  
  // If the previous block drives the input of the dev ( instantaneus ) this one drives the 
  // input of the host( out of the dev one clock cycle after.)
  always_comb begin
    for (integer host = 0; host < NrHosts; host = host + 1) begin
      host_gnt_inst_o[host] = 1'b0;
      if (NumBitsHostSelInst'(host) == host_sel_inst_resp) begin  
        host_rvalid_inst_o[host] = mem_rvalid_inst_i[mem_sel_inst_resp];
        host_err_inst_o[host]    = mem_err_inst_i[mem_sel_inst_resp];
        host_rdata_inst_o[host]  = mem_rdata_inst_i[mem_sel_inst_resp];
      end else begin
        host_rvalid_inst_o[host] = 1'b0;
        host_err_inst_o[host]    = 1'b0;
        host_rdata_inst_o[host]  = 'b0;
      end
    end
    host_gnt_inst_o[host_sel_inst_req] = host_req_inst_i[host_sel_inst_req];
  end
endmodule
