
// Copyright 2023 University of Naples, Federico II

//////////////////////////////////////////////////////////////////////////////////
// Engineer:        Stefano Mercogliano - stefano.mercogliano@unina.it          //
//                                                                              //
//                                                                              //
//                                                                              //
// Description:     The Region Protection Memory is a core-private memory       //
//                  used by a PMP-walker to access a radix-trie structure       //
//                  with minimum latency. It provides a single write port       //
//                  and two read ports which are only used by the core by       //	
//                  means of implicit accesses. It can be written only by       //
//                  a code running on a background region in M-mode. This       //
//                  implies the absence of mutual exclusion and improves        //
//                  both performance and implementation simplicity. It          //
//                  can also support the Physical Memory Relocation             //
//                  extension.                                                  //      
//                  Implicit accesses are 64 bits wide, that's why we have      //
//                  two banks                                                   //          
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

module cv32e41s_rpm 
#(
	parameter MemSize  				= 1024,
  parameter AddressWidth   	= $clog2(MemSize),
  parameter DataWidth		    = 32

) (
    input               clk_i,
    input               rst_ni,

		// This port is only accessible to M-mode, when implicit trie memory probing is inhibited
    input               						m_req_i, 
    input               						m_we_i,
    input        [ 3:0] 						m_be_i,
    input        [AddressWidth-1:0] m_addr_i,
    input        [31:0] 						m_wdata_i,
    output logic        						m_rvalid_o,
    output logic [DataWidth-1:0] 		m_rdata_o,

		// the following ports are only accessible by non M-mode as read ports
		input              							imp_if_req_i, 
    input        [AddressWidth-1:0] imp_if_addr_i,
    output logic        						imp_if_rvalid_o,
    output logic [DataWidth-1:0] 		imp_if_rdata_b0_o,
    output logic [DataWidth-1:0] 		imp_if_rdata_b1_o,

    input              							imp_lsu_req_i, 
    input        [AddressWidth-1:0] imp_lsu_addr_i,
    output logic        						imp_lsu_rvalid_o,
    output logic [DataWidth-1:0] 		imp_lsu_rdata_b0_o,
    output logic [DataWidth-1:0] 		imp_lsu_rdata_b1_o
);

  localparam unsigned BankZeroStart = 0;
  localparam unsigned BankOneStart = BankZeroStart + MemSize/2;
  localparam unsigned BankMask = MemSize/2 - 1;

	logic 										int_req_b0, int_req_b1;
  logic [AddressWidth-1:0] 	int_addr;
  logic        							int_rvalid_b0, int_rvalid_b1;
  logic [DataWidth-1:0] 	  int_rdata_b0, int_rdata_b1;

  logic                     imp_if_rvalid_b0, imp_if_rvalid_b1;

  // Bank choosing (only one bank per time can be addressed)
  always_comb begin
    if(m_req_i) begin
      // Lower half is used for bank 0, while upper hald is used for bank 1
      int_addr = m_addr_i;
      int_addr[AddressWidth-1] = 1'b0;
      int_req_b0 = ~m_addr_i[AddressWidth-1] & m_req_i;
      int_req_b1 = m_addr_i[AddressWidth-1] & m_req_i;
    end else begin
      // For implicit accesses, both banks are enabled
      int_addr = imp_lsu_addr_i; 
      int_req_b0 = imp_lsu_req_i; 
      int_req_b1 = imp_lsu_req_i;
    end
  end

  // Implementation is carried out by hosting two small TCMs
	cv32e41s_tcm #(
    .A_WID		(AddressWidth-1	),
		.MEM_SIZE	(MemSize/2		  ),
    .D_WID		(DataWidth		  )
  )rpm_bank_zero(
    .clk_i				(clk_i),
    .rst_ni				(rst_ni),

    // Port A: shared between Machine mode interface and implicit reads interface 0
    .a_req_i			(int_req_b0		), 
    .a_we_i				(m_we_i			),
    .a_be_i				(m_be_i			),
    .a_addr_i			(int_addr		),
    .a_wdata_i		(m_wdata_i	),
    .a_rvalid_o		(int_rvalid_b0	),
    .a_rdata_o		(int_rdata_b0	),

    // Port B: always connected to implicit reads interface 1
    .b_req_i			(imp_if_req_i			),
    .b_we_i				(1'b0							), 
    .b_be_i				(4'b0							), 
    .b_addr_i			(imp_if_addr_i		),
    .b_wdata_i		(32'b0						), 
    .b_rvalid_o		(imp_if_rvalid_b0	),
    .b_rdata_o		(imp_if_rdata_b0_o		)
  );

  cv32e41s_tcm #(
    .A_WID		(AddressWidth-1	),
		.MEM_SIZE	(MemSize/2		  ),
    .D_WID		(DataWidth		  )
  )rpm_bank_one(
    .clk_i				(clk_i),
    .rst_ni				(rst_ni),

    // Port A: shared between Machine mode interface and implicit reads interface 0
    .a_req_i			(int_req_b1		), 
    .a_we_i				(m_we_i			),
    .a_be_i				(m_be_i			),
    .a_addr_i			(int_addr		),
    .a_wdata_i		(m_wdata_i	),
    .a_rvalid_o		(int_rvalid_b1	),
    .a_rdata_o		(int_rdata_b1	),

    // Port B: always connected to implicit reads interface 1
    .b_req_i			(imp_if_req_i			),
    .b_we_i				(1'b0							), 
    .b_be_i				(4'b0							), 
    .b_addr_i			(imp_if_addr_i		),
    .b_wdata_i		(32'b0						), 
    .b_rvalid_o		(imp_if_rvalid_b1	),
    .b_rdata_o		(imp_if_rdata_b1_o		)
  );

	// assign the same output signal: under the assumption of mutual exclusion no problem should occurr

  always_comb begin

    if(int_rvalid_b0 ^  int_rvalid_b1) begin
      m_rdata_o = (int_rvalid_b0) ? int_rdata_b0 : int_rdata_b1; 
    end else begin
      m_rdata_o = '0;
    end

    m_rvalid_o = (int_rvalid_b0 | int_rvalid_b1) & ~imp_lsu_req_i;

  end

	assign imp_lsu_rvalid_o = int_rvalid_b0 & int_rvalid_b1;
	assign imp_lsu_rdata_b0_o = int_rdata_b0;
  assign imp_lsu_rdata_b1_o = int_rdata_b1;

  assign imp_if_rvalid_o = imp_if_rvalid_b0 & imp_if_rvalid_b1;


endmodule
