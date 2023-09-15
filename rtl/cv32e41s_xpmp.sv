
// Copyright 2023 University of Naples, Federico II
// Date: 27 July 2023

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Engineer:       Stefano Mercogliano - stefano.mercogliano@unina.it         //
//                                                                            //
//                                                                            //
// Description:    XPMP (eXtended Physical Memory Protection Unit)            //        
//                 It is implemented into the MPU and instantiates            //
//                 the PMP, which supports both the Trie                      //
//                 extension (for segmentation, not paging) and the memory    //
//                 relocation extension. Both extensions are custom.          //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module cv32e41s_xpmp import cv32e41s_pkg::*;
  #(      
    // Granularity of NAPOT access,
    // 0 = No restriction, 1 = 8 byte, 2 = 16 byte, 3 = 32 byte, etc.
    parameter int unsigned  PMP_GRANULARITY    = 0,
    // Number of implemented regions
    parameter int           PMP_NUM_REGIONS    = 1,

    // Custom XPMP extension
    parameter xpmp_pmr_e        XPMP_PMR_ENABLE   = PMR_NONE,
    parameter xpmp_pmr_enc_e    XPMP_PMR_ENCODING = PMR_ENC_LIN,
    parameter xpmp_trie_e       XPMP_TRIE         = PMP_TRIE_NONE,
    parameter xpmp_trie_cmp_e   XPMP_TRIE_CMP     = PMP_TRIE_NOTCPM,

    // Core request type to delay the memory transaction
    parameter type         CORE_REQ_TYPE       = obi_inst_req_t
  )(
    // Clock and Reset
    input logic         clk,
    input logic         rst_n,

    // Interface to CSRs
    input pmp_csr_t       csr_pmp_i, 

    // Privilege mode
    input privlvl_t       priv_lvl_i,
    // Access checking
    input logic [33:0]    pmp_req_addr_i,
    input logic           pmp_req_debug_region_i,
    input pmp_req_e       pmp_req_type_i,
    output logic[33:0]    pmr_reloc_addr_o,           
    output logic          pmp_req_err_o,

    // RPM implicit accesses
    output logic			    pmp_imp_req_o, 
    output logic [31:0]	  pmp_imp_addr_o,
    input  logic          pmp_imp_rvalid_i,
    input  logic [31:0]   pmp_imp_rdata_b0_i,
    input  logic [31:0]   pmp_imp_rdata_b1_i,

    // Core transaction delayed if used by the PMP Trie
    input  CORE_REQ_TYPE  core_trans_i,
    input  logic          core_req_i,
    output CORE_REQ_TYPE  pmp_trans_o,
    output logic          pmp_req_o,
    output logic          pmp_busy_o

  );

  logic         core_req; 
  logic         pmp_imp_req;
  logic         tpmp_req_err;
  logic [33:0]  tpmr_reloc_addr;
  logic         lpmp_req_err;
  logic [33:0]  lpmr_reloc_addr;
  CORE_REQ_TYPE pmp_trans;
  logic         pmp_req;
  CORE_REQ_TYPE pmp_trans_r;

  xpmpcfg_t      xpmpcfg;

  assign xpmpcfg = csr_pmp_i.xpmpcfg;

  if (XPMP_TRIE) begin: pmp_trie

      // If PMP trie is enabled, the pmp-walker is implemented. a small PMP only applicable
      // to machine mode still remains; M-mode code will not pass to the walker and will
      // instead use a locked-only entries pmp. (M-mode only entries are never relocated)

      always_comb begin: signal_mux
        if(priv_lvl_i == PRIV_LVL_M || xpmpcfg.trie_en == PMP_TRIE_NONE) begin
          core_req          = 1'b0; 
          pmp_imp_req_o     = 1'b0;
          pmp_req_err_o     = lpmp_req_err;
          pmr_reloc_addr_o  = lpmr_reloc_addr;
          pmp_trans_o       = core_trans_i;
          pmp_req_o         = core_req_i;

        end else begin
          core_req          = core_req_i;    
          pmp_imp_req_o     = pmp_imp_req;
          pmp_req_err_o     = tpmp_req_err;
          pmr_reloc_addr_o  = tpmr_reloc_addr;
          pmp_trans_o       = pmp_trans_r;
          pmp_req_o         = pmp_req;
        end
      end

      // This process is used to delay the assignment of memory transaction if the pmp trie is enabled
      always_ff @(posedge clk) begin
        if (~rst_n) begin
          pmp_trans_r   <= '0;
        end else begin
          if(priv_lvl_i != PRIV_LVL_M) begin
            if(~pmp_busy_o & core_req) begin
              pmp_trans_r   <= core_trans_i;
            end
          end
        end
      end

      cv32e41s_pmp_walker_64
      #(
          // Custom PMR extension
          .XPMP_PMR_ENABLE                  ( XPMP_PMR_ENABLE             ),              
          .XPMP_PMR_ENCODING                ( XPMP_PMR_ENCODING           )   
      )
      pmp_walker_i
      (
          .clk                              ( clk                   ),
          .rst_n                            ( rst_n                 ),

          .mseccfg_i                        ('0                     ),  // ePMP trie not supported atm
          .xpmpcfg_i                        ( xpmpcfg               ),
          .priv_lvl_i                       ( priv_lvl_i            ),

          .imp_req_o                        ( pmp_imp_req           ), 
          .imp_addr_o                       ( pmp_imp_addr_o        ),
          .imp_rvalid_i                     ( pmp_imp_rvalid_i      ),
          .imp_rdata_b0_i                   ( pmp_imp_rdata_b0_i    ),
          .imp_rdata_b1_i                   ( pmp_imp_rdata_b1_i    ),

          .pmp_req_addr_i                   ( pmp_req_addr_i        ),
          .pmp_req_debug_region_i           ('0                     ),
          .pmp_req_type_i                   ( pmp_req_type_i        ),
          .pmr_reloc_addr_o                 ( tpmr_reloc_addr       ),          
          .pmp_req_err_o                    ( tpmp_req_err          ),
          .pmp_busy_o                       ( pmp_busy_o            ),

          .pmp_start_i                      ( core_req              ),  
          .pmp_stop_o                       ( pmp_req               )     
      );

      /* INSTANTIATE LOCKED PMP */

      cv32e41s_pmp
        #(// Parameters
          .PMP_GRANULARITY                  (PMP_GRANULARITY        ),
          .PMP_NUM_REGIONS                  (PMP_NUM_REGIONS        ),
          .XPMP_PMR_ENABLE                  (PMR_NONE               ),              
          .XPMP_PMR_ENCODING                (PMR_ENC_LIN            )   
        )
      locked_pmp_i
        (// Outputs
         .pmp_req_err_o                     ( lpmp_req_err          ),
         // Inputs
         .clk                               (clk                    ),
         .rst_n                             (rst_n                  ),
         .pmp_req_type_i                    (pmp_req_type_i         ),
         .csr_pmp_i                         (csr_pmp_i              ),
         .priv_lvl_i                        (priv_lvl_i             ),
         .pmp_req_addr_i                    (pmp_req_addr_i          ),
         .pmr_reloc_addr_o                  (lpmr_reloc_addr       ),   
         .pmp_req_debug_region_i            (pmp_req_debug_region_i)
        );      

    end
    else begin: pmp_tab

      // If PMP trie is disabled use the base PMP. If PMR is enabled
      // Relocation will be performed inside the PMP, exception made
      // for M-mode code.

      cv32e41s_pmp
        #(// Parameters
          .PMP_GRANULARITY                  (PMP_GRANULARITY        ),
          .PMP_NUM_REGIONS                  (PMP_NUM_REGIONS        ),
          .XPMP_PMR_ENABLE                  (XPMP_PMR_ENABLE        ),              
          .XPMP_PMR_ENCODING                (XPMP_PMR_ENCODING      )   
        )
      pmp_i
        (// Outputs
         .pmp_req_err_o                     (pmp_req_err_o          ),
         // Inputs
         .clk                               (clk                    ),
         .rst_n                             (rst_n                  ),
         .pmp_req_type_i                    (pmp_req_type_i         ),
         .csr_pmp_i                         (csr_pmp_i              ),
         .priv_lvl_i                        (priv_lvl_i             ),
         .pmp_req_addr_i                    (pmp_req_addr_i         ),
         .pmr_reloc_addr_o                  (pmr_reloc_addr_o       ),   
         .pmp_req_debug_region_i            (pmp_req_debug_region_i)
      );

      // Walker signals are goin to be 0
      assign pmp_imp_req_o = '0;
      assign pmp_imp_addr_o = '0;
      assign pmp_busy_o = '0;
      assign pmp_req_o = core_req_i;
      assign pmp_trans_o = core_trans_i;

  end



endmodule