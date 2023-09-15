// Copyright 2023 University of Naples, Federico II

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

// NOTES:
//				Output PMP signals should be computed in the last stage. How long should they stay on?

module cv32e41s_pmp_walker import cv32e41s_pkg::*;
  #(
    // Granularity of NAPOT access,
    // 0 = No restriction, 1 = 8 byte, 2 = 16 byte, 3 = 32 byte, etc.
    parameter int unsigned  PMP_GRANULARITY    = 0,
    parameter int unsigned  PMP_UTLB           = 0,
    // Custom PMR extension
    parameter pmr_en_e      PMR_ENABLE         = PMR_EN_NONE,
    parameter pmr_enc_e     PMR_ENCODING       = PMR_ENC_LIN
  )
  (
   	// Clock and Reset
   	input	 logic        clk,
   	input  logic        rst_n,

   	// Interface to CSRs
   	input  mseccfg_t		mseccfg_i,
		input  pmrcfg_t			pmrcfg_i,

		// Interface to Region Protection Memory (RPM)
		input  logic													core_req_i,			// The core requests a memory transaction that must be checked
		output logic			       							imp_req_o, 
    output logic [31:0]			 							imp_addr_o,
    input  logic        									imp_rvalid_i,
    input  logic [31:0] 									imp_rdata_i,

   	// Access checking
   	input  logic [33:0] 									pmp_req_addr_i,
   	input  logic        									pmp_req_debug_region_i,
   	input  pmp_req_e    									pmp_req_type_i,
   	output logic[33:0] 										pmr_reloc_addr_o,           
   	output logic       										pmp_req_err_o
   );

	typedef enum int {
    IDLE 									= 0,
		PREFIX_PROBING_CTRL		= 1,
		PREFIX_PROBING_GUARD	= 2,
		LIMIT_BINDING_CFG			= 3,
		LIMIT_BINDING_ADDROFF = 4
	} walker_fsm_e;

	
	localparam unsigned CTRL_HINT_END = 11;		localparam unsigned CTRL_HINT_BEGIN = 0;
	localparam unsigned CTRL_GSIZE_END = 16;	localparam unsigned CTRL_GSIZE_BEGIN = 12;
	localparam unsigned CTRL_LIMIT_BIT = 17;
	localparam unsigned CTRL_VALID_BIT = 18;
	localparam unsigned CTRL_NPTR_END = 31;		localparam unsigned CTRL_NPTR_BEGIN = 19;

	walker_fsm_e  fsm_state;
	logic [31:0]  prefix_addr_r;
	logic [31:0]  ctrl_r;
	logic [31:0]  guard_r;
	logic [31:0]  cfg_r;
	logic [31:0]  addroff_r;

	logic [11:0]  prefix_len_r;
	logic [12:0]	currptr_r;
	logic [12:0]	prefix_r; 
	logic [31:0]	prefix_guard_r;	

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

	logic [12:0]	imp_addr_w;

	logic					req_exception_w;
	logic					guard_exception_w;

	logic [5:0]		guard_upper_w;
	logic [5:0]		guard_lower_w;

	assign imp_req_o = 1'b1;

	always_ff@(posedge clk) begin: walker_fsm
    if (~rst_n) begin

			ctrl_r 					<= '0;
			guard_r 				<= '0;
			prefix_len_r 		<= '0;
			prefix_addr_r		<= '0;
			currptr_r				<= '0;
			cfg_r						<= '0;
			prefix_r				<= '0;
			prefix_guard_r	<= '0;

    end else begin 

			case(fsm_state) 

				IDLE: begin

					if(core_req_i) begin

						// Request the first entry in RPM
						fsm_state 		<= PREFIX_PROBING_CTRL;
						ctrl_r 				<= imp_rdata_i;
						currptr_r 		<= imp_addr_w;

					end else begin

						fsm_state 	<= IDLE;

					end

				end

				PREFIX_PROBING_CTRL: begin

					currptr_r <= imp_addr_w;
					
					if(req_exception_w) begin		// Check for exception -> come back to IDLE

						fsm_state 	<= IDLE;

					end else if(|gsize_w) begin	// Check for guard -> go to GUARD reading

						// Read the guard; it is also possible to be limit 	
						guard_r <= imp_rdata_i;
						prefix_guard_r <= req_guard_w;
						fsm_state 	<= PREFIX_PROBING_GUARD;
						prefix_len_r <= prefix_len_r + gsize_w + 1;

					end else if(ctrl_r[CTRL_LIMIT_BIT]) begin

						// Probing phase is over, next level is the limit one
						cfg_r <= imp_rdata_i;
						fsm_state 	<= LIMIT_BINDING_CFG;

					end else begin

						// Next level is not the last one, so read the next control
						ctrl_r <= imp_rdata_i;
						fsm_state 	<= PREFIX_PROBING_CTRL;
						prefix_len_r <= prefix_len_r + 1;

					end

				end

				PREFIX_PROBING_GUARD: begin

					// Wait for the next rdata
					prefix_len_r <= prefix_len_r + gsize_w;
					currptr_r <= imp_addr_w;
					ctrl_r <= imp_rdata_i;
					prefix_len_r <= prefix_len_r + 1;

					if(guard_exception_w) begin

						fsm_state 	<= IDLE;

					end else if(ctrl_r[CTRL_LIMIT_BIT]) begin
						// Probing phase is over, next level is the limit one
						cfg_r <= imp_rdata_i;
						fsm_state 	<= LIMIT_BINDING_CFG;

					end else begin
						// Next level is not the last one, so read the next control
						ctrl_r <= imp_rdata_i;
						fsm_state 	<= PREFIX_PROBING_CTRL;

					end

				end

				LIMIT_BINDING_CFG: begin
					
					if(PMR_ENABLE != PMR_EN_NONE && PMR_ENCODING == PMR_ENC_LIN) begin
						// Read the relocation entry
						addroff_r <= imp_rdata_i;
						currptr_r <= imp_addr_w;

						fsm_state <= LIMIT_BINDING_ADDROFF;
					end else begin

						// Clean all registers used
						ctrl_r 					<= '0;
						guard_r 				<= '0;
						prefix_len_r 		<= '0;
						prefix_addr_r		<= '0;
						currptr_r				<= '0;
						cfg_r						<= '0;
						prefix_r				<= '0;
						prefix_guard_r	<= '0;

						// Trie explored, go back to IDLE or PREFIX PROBING
						if(core_req_i) begin
							fsm_state 		<= PREFIX_PROBING_CTRL;
							ctrl_r 				<= imp_rdata_i;
							currptr_r 		<= imp_addr_w;
						end else begin
							fsm_state 	<= IDLE;
						end
					end

				end
				
				LIMIT_BINDING_ADDROFF: begin

					// Clean all registers used
					ctrl_r 					<= '0;
					guard_r 				<= '0;
					prefix_len_r 		<= '0;
					prefix_addr_r		<= '0;
					currptr_r				<= '0;
					cfg_r						<= '0;
					prefix_r				<= '0;
					prefix_guard_r	<= '0;

					// Trie explored, go back to IDLE or PREFIX PROBING
					if(core_req_i) begin
						fsm_state 		<= PREFIX_PROBING_CTRL;
						ctrl_r 				<= imp_rdata_i;
						currptr_r 		<= imp_addr_w;
					end else begin
						fsm_state 	<= IDLE;
					end
				end

			endcase
		end
	end


	always_comb begin: walker_logic

		gsize_w = ctrl_r[CTRL_GSIZE_END:CTRL_GSIZE_BEGIN];
		hint_w	= ctrl_r[CTRL_HINT_END:CTRL_HINT_BEGIN];
		limit_w = ctrl_r[CTRL_LIMIT_BIT];
		valid_w = ctrl_r[CTRL_VALID_BIT];
		nptr_w	= ctrl_r[CTRL_NPTR_END:CTRL_NPTR_BEGIN];
		prefix_len_w = prefix_len_r;

		envid_w = '0; 					// Unsupported multiple evironment id atm

		// Let's check for the next bit
		prefix_bit_w = (|gsize_w) ? pmp_req_addr_i[31 - prefix_len_w - gsize_w - 1] : pmp_req_addr_i[31 - prefix_len_w - 1];		

		// Assign the address for the next implicit memory access
		if(fsm_state == IDLE) begin

			// During IDLE state address the first or second entry in the RPM
			imp_addr_w = {pmp_req_addr_i[31],3'b000};

		end else if(fsm_state == PREFIX_PROBING_CTRL) begin
			if(|gsize_w) begin
				// Read the entry guard word
				imp_addr_w = currptr_r + 4;
			end else begin
				// Jump to another level without reading the guard
				imp_addr_w = (prefix_bit_w) ? nptr_w + 8 : nptr_w;
			end
		end else if(fsm_state == PREFIX_PROBING_GUARD) begin
			// Jump to another level after reading the guard
			imp_addr_w = (prefix_bit_w) ? nptr_w + 8 : nptr_w;

		end else if(fsm_state == LIMIT_BINDING_CFG) begin

			imp_addr_w = (PMR_ENABLE != PMR_EN_NONE && PMR_ENCODING == PMR_ENC_LIN) ? currptr_r + 4: {pmp_req_addr_i[31],3'b000};

		end else if(fsm_state == LIMIT_BINDING_ADDROFF) begin

			imp_addr_w = {pmp_req_addr_i[31],3'b000};

		end 
		
		// If a guard has to be computed, let's do it
		if(|gsize_w) begin

			// Compute the guards signals
			guard_lower_w = 31 - prefix_len_w - gsize_w;
			guard_upper_w = 31 - prefix_len_w - 1;
			req_guard_w = (((1 << guard_upper_w)-1) & pmp_req_addr_i) >> guard_lower_w;

			prefix_guard_w = prefix_guard_r;

			// Read entry guard
			entry_guard_w = guard_r & ((1 << gsize_w) - 1);

			// If the prefix doesn't match exactly the guard, then raise a guard exception
			guard_exception_w = |(entry_guard_w ^ prefix_guard_w);

		end else begin

			guard_lower_w = 0;
			guard_upper_w = 0;
			entry_guard_w = 0;
			prefix_guard_w = 0;
			guard_exception_w = 0;

		end

		req_exception_w = 
											~valid_w 																| 
											(~limit_w & ~hint_w[prefix_bit_w])			|
											(limit_w & ~hint_w[envid_w]);

		
		// Compute the prefix
		prefix_w = pmp_req_addr_i & ~((1 << (31 - prefix_len_w)) - 1);
		
	end

	assign imp_addr_o = imp_addr_w;
	assign pmp_req_err_o = req_exception_w | guard_exception_w;


endmodule