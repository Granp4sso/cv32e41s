// Copyright 2023 University of Naples, Federico II
// Date: 27 July 2023

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Engineer:        Stefano Mercogliano - stefano.mercogliano@unina.it        //
//                                                                            //
//                                                                            //
// Description:    PMP-Walker                     														//
//	
//
//
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e41s_pmp_walker_64 import cv32e41s_pkg::*;
  #(
    // Granularity of NAPOT access,
    // 0 = No restriction, 1 = 8 byte, 2 = 16 byte, 3 = 32 byte, etc.
    parameter int unsigned    PMP_GRANULARITY    = 0,
    parameter int unsigned    PMP_UTLB           = 0,
    // Custom PMR extension
    parameter xpmp_pmr_e      XPMP_PMR_ENABLE   = PMR_NONE,
    parameter xpmp_pmr_enc_e  XPMP_PMR_ENCODING = PMR_ENC_LIN,
    // Core request type to delay the memory transaction
    parameter type            CORE_REQ_TYPE       = obi_inst_req_t
  )
  (
   	// Clock and Reset
   	input	 logic        clk,
   	input  logic        rst_n,

   	// Interface to CSRs
   	input  mseccfg_t		mseccfg_i,
		input  xpmpcfg_t		xpmpcfg_i,
    input privlvl_t     priv_lvl_i,

		// Interface to Region Protection Memory (RPM)
		output logic			       							imp_req_o, 
    output logic [31:0]			 							imp_addr_o,
    input  logic        									imp_rvalid_i,
    input  logic [31:0] 									imp_rdata_b0_i,
    input  logic [31:0] 									imp_rdata_b1_i,

   	// Access checking
   	input  logic [33:0] 									pmp_req_addr_i,
   	input  logic        									pmp_req_debug_region_i,
   	input  pmp_req_e    									pmp_req_type_i,
   	output logic[33:0] 										pmr_reloc_addr_o,           
   	output logic       										pmp_req_err_o,
    output logic                          pmp_busy_o,

    // Core transaction signals
    input  logic                          pmp_start_i,     // The core requests a memory transaction that must be checked
    output logic                          pmp_stop_o
   );

	typedef enum int {
    IDLE 									= 0,
		PREFIX_PROBING		    = 1,
		LIMIT_BINDING   			= 2
	} walker_fsm_e;

	
	localparam unsigned CTRL_HINT_END = 11;		localparam unsigned CTRL_HINT_BEGIN = 0;
	localparam unsigned CTRL_GSIZE_END = 16;	localparam unsigned CTRL_GSIZE_BEGIN = 12;
	localparam unsigned CTRL_LIMIT_BIT = 17;
	localparam unsigned CTRL_VALID_BIT = 18;
	localparam unsigned CTRL_NPTR_END = 31;		localparam unsigned CTRL_NPTR_BEGIN = 19;

  localparam unsigned CFG_PERM_END = 2;      localparam unsigned CFG_PERM_BEGIN = 0;
  localparam unsigned CFG_REGSIZE_END = 31;  localparam unsigned CFG_REGSIZE_BEGIN = 3; 

  localparam unsigned H = CTRL_HINT_END + 1;
  localparam unsigned N = $clog2(H);

	walker_fsm_e      fsm_state;
  logic [12:0]	    imp_addr_w;
  logic             pmp_stop_w;

  // Prefix probing signals
  logic [31:0]  ctrl_w;
	logic [31:0]  guard_w;
	logic [11:0] 	hint_w;
	logic [4:0] 	gsize_w;
	logic					limit_w;
	logic					valid_w;
	logic	[12:0]  nptr_w;
	logic [31:0] 	prefix_w;
	logic [11:0] 	prefix_len_w;
	logic					prefix_bit_w;
	logic	[11:0]	envid_w;
	logic [31:0]	entry_guard_w;
	logic [31:0]	prefix_guard_w;
	logic [31:0]	req_guard_w;
	logic [5:0]		guard_upper_w;
	logic [5:0]		guard_lower_w;
  logic [11:0]  prefix_len_r;

  // Compressed offset signals

  logic [N*H*N-1:0] hint_sum_w;
  logic [11:0]      masked_hint_w;
  logic [3:0]       nptr_offset_cmp_w;
  logic [3:0]       nptr_offset_w;

  // Limit binding signals
  logic [31:0]  cfg_w;
  logic [31:0]  regsize_w;
  logic [31:0]  suffix_w;
  logic [2:0]   perm_w;

  // Relocation signals
  logic [31:0]  addroff_w;
  logic [31:0]  reloc_addr;

  // Exception signals
  logic         overflow_exception_w;
  logic         access_exception_w;
  logic         limit_exception_w;
  logic         prefix_exception_w;
	logic					req_exception_w;
	logic					guard_exception_w;
  logic         exception_r;          // Latched version exception signals

  logic         pmp_busy_w;
  logic [31:0]  pmp_addr_r;
  pmp_req_e     pmp_type_r;

  // Assignments
  assign limit_exception_w = access_exception_w | overflow_exception_w;
  assign prefix_exception_w = req_exception_w | guard_exception_w;

  assign imp_req_o = 1'b1;
	assign imp_addr_o = imp_addr_w;
	assign pmp_req_err_o =  prefix_exception_w | limit_exception_w | exception_r; 
  assign pmr_reloc_addr_o = reloc_addr;
  assign pmp_busy_o = pmp_busy_w;
  assign pmp_stop_o = pmp_stop_w;

  //////////////////////////
  //     ___  __          //
  //    / __\/ _\  /\/\   //
  //   / _\  \ \  /    \  //
  //  / /    _\ \/ /\/\ \ //
  //  \/     \__/\/    \/ //
  //                      //
  //////////////////////////

  assign pmp_busy_w = (fsm_state == PREFIX_PROBING); 

	always_ff@(posedge clk) begin: walker_fsm
    if (~rst_n) begin
			prefix_len_r	<= '0;
      pmp_addr_r    <= '0;
      pmp_type_r    <= PMP_ACC_EXEC;
      exception_r   <= '0;
    end else begin 

			case(fsm_state) 

				IDLE: begin
          pmp_addr_r    <= pmp_req_addr_i;
          pmp_type_r    <= pmp_req_type_i;
          
          // Once an exception occurs wait for M mode to be triggered
          exception_r   <= (priv_lvl_i == PRIV_LVL_M) ? '0 : exception_r;

					if(pmp_start_i && !exception_r) begin
						// Request the first entry in RPM
						fsm_state 		<= PREFIX_PROBING;
            //exception_r   <= '0;
					end else begin
						fsm_state 	<= IDLE;
            prefix_len_r	<= '0;
					end
				end

				PREFIX_PROBING: begin

          exception_r <= prefix_exception_w;

					if(prefix_exception_w) begin		// Check for exception -> come back to IDLE
						fsm_state 	<= IDLE;
            prefix_len_r	<= '0;
					end else if(ctrl_w[CTRL_LIMIT_BIT]) begin
						// Probing phase is over, next level is the limit one
            prefix_len_r  <= prefix_len_w;
						fsm_state 	<= LIMIT_BINDING;
					end else begin
						// Next level is not the last one, so read the next prefix entry
            prefix_len_r  <= prefix_len_w;
						fsm_state 	  <= PREFIX_PROBING;
					end

				end
				
				LIMIT_BINDING: begin
					// Clean prefix len counter and latch exception, addr and request type.
					prefix_len_r 		<= '0;
          exception_r     <= limit_exception_w;
          pmp_addr_r      <= pmp_req_addr_i;
          pmp_type_r      <= pmp_req_type_i;
					// Trie explored, go back to IDLE or PREFIX PROBING
          if(limit_exception_w) begin
            fsm_state 	<= IDLE;
          end else if(pmp_start_i) begin
            fsm_state 	  <= PREFIX_PROBING;
            exception_r <= '0;
					end else begin
						fsm_state 	<= IDLE;
					end
				end

			endcase

      if(priv_lvl_i == PRIV_LVL_M) fsm_state 	<= IDLE;
		end
	end

//////////////////////////////////////
//   __    __      _ _              //  
//  / / /\ \ \__ _| | | _____ _ __  //
//  \ \/  \/ / _` | | |/ / _ \ '__| //
//   \  /\  / (_| | |   <  __/ |    //
//    \/  \/ \__,_|_|_|\_\___|_|    //
//                                  //
//////////////////////////////////////

	always_comb begin: walker_logic

    ////////////////////////////////////////////////////////
    //  Read CTRL and GUARD registers from the next entry //
    ////////////////////////////////////////////////////////

    ctrl_w =    (fsm_state == PREFIX_PROBING)     ? imp_rdata_b0_i  : '0;
    guard_w =   (fsm_state == PREFIX_PROBING)     ? imp_rdata_b1_i  : '0;

    // Reading fields
		gsize_w = ctrl_w[CTRL_GSIZE_END:CTRL_GSIZE_BEGIN];
		hint_w	= ctrl_w[CTRL_HINT_END:CTRL_HINT_BEGIN];
		limit_w = ctrl_w[CTRL_LIMIT_BIT];
		valid_w = ctrl_w[CTRL_VALID_BIT];
		nptr_w	= ctrl_w[CTRL_NPTR_END:CTRL_NPTR_BEGIN];

    prefix_len_w = (|gsize_w) ? prefix_len_r + gsize_w + 1 : prefix_len_r + 1;
    prefix_bit_w = pmp_addr_r[31 - prefix_len_w];

    nptr_offset_cmp_w = hint_sum_w[(N-1)*H*N + N - 1: (N-1)*H*N];

    if(limit_w) begin
      nptr_offset_w = (xpmpcfg_i.trie_compression[1]) ? nptr_offset_cmp_w << 2 : envid_w << 2;        // Limit is compressed
    end else begin
      nptr_offset_w = (xpmpcfg_i.trie_compression[0]) ? nptr_offset_cmp_w << 2 : prefix_bit_w << 2;    // Segment is compressed
    end

    ////////////////////////////////////////////////////////
    //  Compute next implicit address and exceptions      //
    ////////////////////////////////////////////////////////

    // Initialize signals to 0
    guard_lower_w = 0;
    guard_upper_w = 0;
    entry_guard_w = 0;
    prefix_guard_w = 0;
    guard_exception_w = 0;
    req_exception_w = '0;
    prefix_w = '0;
    req_guard_w = '0;

    // Prefix Probing signals
    if(fsm_state == PREFIX_PROBING) begin
      // Jump to limit level or to prefix level
      imp_addr_w = nptr_w + nptr_offset_w; //(limit_w) ? nptr_w + (envid_w << 2) : nptr_w + (prefix_bit_w << 2); //       nptr_w + nptr_offset_w; 

      // If a guard has to be computed, let's do it
      if(|gsize_w) begin

        // Compute the guards signals
        guard_lower_w = 31 - prefix_len_r - gsize_w;
        guard_upper_w = 31 - prefix_len_r - 1;
        req_guard_w = (((1 << guard_upper_w)-1) & pmp_req_addr_i) >> guard_lower_w;

        // Read entry guard
        entry_guard_w = guard_w & ((1 << gsize_w) - 1);

        // If the prefix doesn't match exactly the guard, then raise a guard exception
        guard_exception_w = |(entry_guard_w ^ req_guard_w);

      end

      req_exception_w = 
											~valid_w 																| 
											(~limit_w & ~hint_w[prefix_bit_w])			|
											(limit_w & ~hint_w[envid_w]);

    end else begin
      // During IDLE state address the first or b1 entry in the RPM
			imp_addr_w = {pmp_req_addr_i[31],2'b00};
      prefix_w = pmp_addr_r & ~((1 << (31 - prefix_len_r + 1)) - 1);
    end
		
	end

  //////////////////////////////////////////////////////////////
  //  Compute the compressed offset to address the next entry //
  //////////////////////////////////////////////////////////////

  // If the RPM-GRT is not compressed we can use prefixbit/envid as offset.
  // Otherwise the hint field can be used to count how many valid offset
  // exists for that level and compute the correct offset
  // Compression can be applied to Segment Trie levels and Limit Table independently

  // envid is fixed to 0 at the moment
  // Mask all the upper bits starting from the found position
  assign masked_hint_w = (limit_w) ? (( 1 << envid_w ) - 1) & hint_w : (( 1 << prefix_bit_w ) - 1) & hint_w;

  // Use a tree adder to compute the position (sum all bits)
  // First let's assign hint_w to level 0
  for(genvar i = 0; i < N*H*N; i++) begin
    assign hint_sum_w[i] = 1'b0;
  end

  for(genvar j = 0; j < H; j++) begin
    assign hint_sum_w[j*N] = masked_hint_w[j];
  end

  // We have N = log(H) levels, for each level we have H/2^i sums and each sum is made of N bits.
  for(genvar i = 1; i < N-1; i++) begin
    for(genvar j = 0; j < H; j+= 2**i) begin
      assign hint_sum_w[i*H*N + j*N + N - 1: i*H*N + j*N] = 
                                              hint_sum_w[(i-1)*H*N + j*N + N - 1: (i-1)*H*N + j*N] + 
                                              hint_sum_w[(i-1)*H*N + (j + 2**(i-1))*N + N - 1: (i-1)*H*N + (j + 2**(i-1))*N];
    end
  end 
  // assign last level
  assign hint_sum_w[(N-1)*H*N + N - 1: (N-1)*H*N] = (H == 2**N) ?
                                              hint_sum_w[(N-2)*H*N + N - 1: (N-2)*H*N] + 
                                              hint_sum_w[(N-2)*H*N + 2**(N-2)*N + N - 1: (N-2)*H*N + 2**(N-2)*N] :

                                              hint_sum_w[(N-2)*H*N + N - 1: (N-2)*H*N] + 
                                              hint_sum_w[(N-2)*H*N + 2**(N-2)*N + N - 1: (N-2)*H*N + 2**(N-2)*N] +
                                              hint_sum_w[(N-2)*H*N + 2**(N-1)*N + N - 1: (N-2)*H*N + 2**(N-1)*N];


  ////////////////////////////////////////
  //     ___ _               _          //
  //    / __\ |__   ___  ___| | _____   //
  //   / /  | '_ \ / _ \/ __| |/ / __|  //
  //  / /___| | | |  __/ (__|   <\__ \  //
  //  \____/|_| |_|\___|\___|_|\_\___/  //
  //                                    //
  ////////////////////////////////////////

  always_comb begin: pmp_checking_logic

    // Read Limit entry registers
    cfg_w =     (fsm_state != LIMIT_BINDING)     ? '0              : imp_rdata_b0_i ;

    // Reading cfg fields
    perm_w        = cfg_w[CFG_PERM_END:CFG_PERM_BEGIN];
    regsize_w     = {cfg_w[CFG_REGSIZE_END:CFG_REGSIZE_BEGIN], 3'b000}; // A region can be as small as 8 bytes

    suffix_w = '0;
    overflow_exception_w = '0;
    access_exception_w = '0;

    if(fsm_state == LIMIT_BINDING) begin
      // Region size check
      suffix_w = pmp_addr_r - prefix_w;
      overflow_exception_w = (suffix_w >= regsize_w);

      // Access permissions check
      // Fields L and A are not used (NAPOT is the only possible encoding), so we only require the first 3 bits of pmpcfg
      access_exception_w =
                            (~perm_w[0] & pmp_type_r[1]) |     // If the R permission is low and the access is Read, fault
                            (~perm_w[1] & pmp_type_r[0]) |     // If the W permission is low and the access is Write, fault
                            (~perm_w[2] & ~(|pmp_type_r));     // If the X permission is low and the access is Exec, fault

    end

  end

  //////////////////////////////////////////////////////
  //     __      _                 _   _              //
  //    /__\ ___| | ___   ___ __ _| |_(_) ___  _ __   //
  //   / \/// _ \ |/ _ \ / __/ _` | __| |/ _ \| '_ \  //
  //  / _  \  __/ | (_) | (_| (_| | |_| | (_) | | | | //
  //  \/ \_/\___|_|\___/ \___\__,_|\__|_|\___/|_| |_| //
  //                                                  //
  //////////////////////////////////////////////////////

  // Linear Relocation is the only one reasonable for a 
  // Trie approach because we always need to read the 32 addroff bits.
  // However I am leaving the other encodings for now.

   always_comb begin: relocation_logic

    pmp_stop_w = (fsm_state != LIMIT_BINDING)     ? 1'b0            : 1'b1;

    addroff_w = (fsm_state != LIMIT_BINDING)     ? '0              : imp_rdata_b1_i ;
    reloc_addr = pmp_addr_r;

    if(xpmpcfg_i.pmr_support != PMR_NONE && fsm_state == LIMIT_BINDING) begin
      case(xpmpcfg_i.pmr_encoding)
        PMR_ENC_LIN:  reloc_addr =  (addroff_w[31]) ? 
                                    pmp_addr_r - {1'b0, addroff_w[30:0]} : 
                                    pmp_addr_r + addroff_w;

        PMR_ENC_POT:  reloc_addr =  (addroff_w[6]) ? 
                                    pmp_addr_r - (1 << addroff_w[4:0]) : 
                                    pmp_addr_r + (1 << addroff_w[4:0]);

        PMR_ENC_MIX:  reloc_addr =  (addroff_w[8]) ? 
                                    pmp_addr_r + ((1 << addroff_w[4:0])+(4'b0100 << addroff_w[7:5])) : 
                                    pmp_addr_r - ((1 << addroff_w[4:0])+(4'b0100 << addroff_w[7:5]));
                                  
        default       reloc_addr =  pmp_addr_r;
      endcase
    end

  end



endmodule