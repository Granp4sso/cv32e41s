
// Copyright 2023 University of Naples, Federico II

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Antonio Emmanuele                                          //
//                                                                            //
// Additional contributions by:                                               //
//                 Stefano Mercogliano - stefano.mercogliano@unina.it         //
//                                                                            //
//                                                                            //
// Description:   Top level for cv32e41s. It hosts the incore and the         //
//                internal bus for private addressing peripherals such        //
//                as TCMs and security related devices                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module cv32e41s_core import cv32e41s_pkg::*;
#(
  parameter                             LIB                                     = 0,
  parameter rv32_e                      RV32                                    = RV32I,
  parameter b_ext_e                     B_EXT                                   = B_NONE,
  parameter m_ext_e                     M_EXT                                   = M,
  parameter bit                         DEBUG                                   = 1,
  parameter logic [31:0]                DM_REGION_START                         = 32'hF0000000,
  parameter logic [31:0]                DM_REGION_END                           = 32'hF0003FFF,
  parameter int                         DBG_NUM_TRIGGERS                        = 1,
  parameter int                         PMA_NUM_REGIONS                         = 0,
  parameter pma_cfg_t                   PMA_CFG[PMA_NUM_REGIONS-1:0]            = '{default:PMA_R_DEFAULT},
  parameter bit                         CLIC                                    = 0,
  parameter int unsigned                CLIC_ID_WIDTH                           = 5,
  parameter int unsigned                CLIC_INTTHRESHBITS                      = 8,
  parameter int unsigned                PMP_GRANULARITY                         = 0,
  parameter int                         PMP_NUM_REGIONS                         = 0,
  parameter pmpncfg_t                   PMP_PMPNCFG_RV[PMP_NUM_REGIONS-1:0]     = '{default:PMPNCFG_DEFAULT},
  parameter logic [31:0]                PMP_PMPADDR_RV[PMP_NUM_REGIONS-1:0]     = '{default:32'h0},
  parameter mseccfg_t                   PMP_MSECCFG_RV                          = MSECCFG_DEFAULT,  

  parameter lfsr_cfg_t                  LFSR0_CFG                               = LFSR_CFG_DEFAULT,   // Do not use default value for LFSR configuration
  parameter lfsr_cfg_t                  LFSR1_CFG                               = LFSR_CFG_DEFAULT,   // Do not use default value for LFSR configuration
  parameter lfsr_cfg_t                  LFSR2_CFG                               = LFSR_CFG_DEFAULT,   // Do not use default value for LFSR configuration

  // Custom XPMP extension
  parameter int                         XPMP_ENABLE       = 1,
  parameter xpmp_pmr_e                  XPMP_PMR_ENABLE   = PMR_SUPPORT,
  parameter xpmp_pmr_enc_e              XPMP_PMR_ENCODING = PMR_ENC_LIN,
  parameter xpmp_trie_e                 XPMP_TRIE         = PMP_TRIE_NONE,
  parameter xpmp_trie_cmp_e             XPMP_TRIE_CMP     = PMP_TRIE_NOTCPM,

  // Internal address space parameters 
  parameter logic [31:0] INT_BUS_START   = 32'h70000000,   // Starting address of the incore bus
  parameter logic [31:0] INT_BUS_DIM     = 12* 1024,       // Size of the internal bus address space 
  // TCM Parameters, the tcm is placed into the internal address space
  // Instruction TCM Parameters
  parameter logic [31:0] TCM_INST_SIZE     =  4 * 1024,    // Size of the instruction TCM
  parameter logic [31:0] TCM_INST_START    = INT_BUS_START, // Starting address of the inst tcm 
  // Data TCM Parameters
  parameter logic [31:0] TCM_DATA_SIZE     =  4 * 1024,      //  Size of the data TCM
  parameter logic [31:0] TCM_DATA_START    = INT_BUS_START + 32'h2000,   //  Starting address of the data tcm  
  // Region Protection Memory Parameters
  parameter logic [31:0] RPM_SIZE     =  1 * 1024,      
  parameter logic [31:0] RPM_START    = TCM_DATA_START + TCM_DATA_SIZE     
)
(
  // Clock and reset
  input  logic                          clk_i,
  input  logic                          rst_ni,
  input  logic                          scan_cg_en_i,   // Enable all clock gates for testing

  // Static configuration
  input  logic [31:0]                   boot_addr_i,
  input  logic [31:0]                   dm_exception_addr_i,
  input  logic [31:0]                   dm_halt_addr_i,
  input  logic [31:0]                   mhartid_i,
  input  logic  [3:0]                   mimpid_patch_i,
  input  logic [31:0]                   mtvec_addr_i,

  // Instruction memory interface
  output logic                          instr_req_o, 
  input  logic                          instr_gnt_i,
  input  logic                          instr_rvalid_i,
  output logic [31:0]                   instr_addr_o,
  output logic [1:0]                    instr_memtype_o,
  output logic [2:0]                    instr_prot_o,
  output logic                          instr_dbg_o,
  input  logic [31:0]                   instr_rdata_i,
  input  logic                          instr_err_i,

  output logic                          instr_reqpar_o,         // secure
  input  logic                          instr_gntpar_i,         // secure
  input  logic                          instr_rvalidpar_i,      // secure
  output logic [12:0]                   instr_achk_o,           // secure
  input  logic [4:0]                    instr_rchk_i,           // secure

  // Data memory interface
  output logic                          data_req_o,
  input  logic                          data_gnt_i,
  input  logic                          data_rvalid_i,
  output logic [31:0]                   data_addr_o,
  output logic [3:0]                    data_be_o,
  output logic                          data_we_o,
  output logic [31:0]                   data_wdata_o,
  output logic [1:0]                    data_memtype_o,
  output logic [2:0]                    data_prot_o,
  output logic                          data_dbg_o,
  input  logic [31:0]                   data_rdata_i,
  input  logic                          data_err_i,

  output logic                          data_reqpar_o,          // secure
  input  logic                          data_gntpar_i,          // secure
  input  logic                          data_rvalidpar_i,       // secure
  output logic [12:0]                   data_achk_o,            // secure
  input  logic [4:0]                    data_rchk_i,            // secure

  // Cycle count
  output logic [63:0]                   mcycle_o,

  // Basic interrupt architecture
  input  logic [31:0]                   irq_i,

  // Event wakeup signals
  input  logic                          wu_wfe_i,   // Wait-for-event wakeup

  // CLIC interrupt architecture
  input  logic                          clic_irq_i,
  input  logic [CLIC_ID_WIDTH-1:0]      clic_irq_id_i,
  input  logic [ 7:0]                   clic_irq_level_i,
  input  logic [ 1:0]                   clic_irq_priv_i,
  input  logic                          clic_irq_shv_i,

  // Fence.i flush handshake
  output logic                          fencei_flush_req_o,
  input  logic                          fencei_flush_ack_i,

    // Security Alerts
  output logic                          alert_minor_o,          // secure
  output logic                          alert_major_o,          // secure

  // Debug interface
  input  logic                          debug_req_i,
  output logic                          debug_havereset_o,
  output logic                          debug_running_o,
  output logic                          debug_halted_o,
  output logic                          debug_pc_valid_o,
  output logic [31:0]                   debug_pc_o,

  // CPU control signals
  input  logic                          fetch_enable_i,
  output logic                          core_sleep_o
);
  // Masks for tcm memories
  localparam logic [31:0] TCM_INST_MASK     = ~(TCM_INST_SIZE-1); 
  localparam logic [31:0] TCM_DATA_MASK     = ~(TCM_DATA_SIZE-1); 
  localparam logic [31:0] RPM_MASK          = ~(RPM_SIZE-1);
  
  // Number of internal memories containing instructions
  localparam NrInstMems = 2;
  // Number of internal memories containing data 
  localparam NrDataMems = 4; 

  localparam logic [31:0] data_tcm_mask = TCM_INST_SIZE-1;
  localparam logic [31:0] i_tcm_mask    = TCM_DATA_SIZE-1;
  localparam logic [31:0] rpm_mask      = RPM_SIZE-1;
  

  logic           core_instr_req      [1];   // instruction req, from the core to the mem/tcm
  logic           core_instr_gnt      [1];   // instruction grant, from the mem/tcm to the core
  logic [31:0]    core_instr_addr     [1];   // requeste address for IF, from the core to the mem/tcm
  logic           core_instr_rvalid   [1];   // tcm/memory signals that the data on the bus is valid
  logic [31:0]    core_instr_rdata    [1];   // core input Instruction from tcm/mem
  logic           core_instr_err      [1];   // Always 0

  // Not used because instruction doesn't
  logic           core_instr_we       [1];
  logic [ 3:0]    core_instr_be       [1];
  logic [31:0]    core_instr_wdata    [1];
  // Memories for instruction fetch
  logic           if_mem_instr_req    [NrInstMems]; // Input request from the internal core
  logic [31:0]    if_mem_instr_addr   [NrInstMems]; // Input address from the internal core
  logic           if_mem_instr_rvalid [NrInstMems]; // TCM/Mem confirms that the response is valid
  logic [31:0]    if_mem_instr_rdata  [NrInstMems]; // TCM/Mem out data 
  logic           if_mem_instr_err    [NrInstMems]; // TCM/Mem error 
  // Not writing in IF
  logic           if_mem_instr_we     [NrInstMems];
  logic [ 3:0]    if_mem_instr_be     [NrInstMems];
  logic [31:0]    if_mem_instr_wdata  [NrInstMems];

  //  IF address mapping
  logic [31:0] if_mem_addr [NrInstMems];
  logic [31:0] if_mem_addr_mask [NrInstMems];

  // Base addresses for IF mem and ram 
  //assign if_mem_addr[Out]         = MEM_OUT_START;
  //assign if_mem_addr_mask[Out]    = MEM_OUT_MASK;
  assign if_mem_addr[TcmInst]     = TCM_INST_START; 
  assign if_mem_addr_mask[TcmInst]   = TCM_INST_MASK;
  // Tie-off unused error signals
  //assign if_mem_instr_err[Out]  = 1'b0;
  assign if_mem_instr_err[TcmInst] = 1'b0;

  logic           core_data_req      [1];   // data req, from the core to the mem/tcm
  logic           core_data_gnt      [1];   // data grant, from the mem/tcm to the core
  logic [31:0]    core_data_addr     [1];   // requeste address from the core LSU to the mem/tcm
  logic           core_data_rvalid   [1];   // tcm/memory signals that the data on the bus is valid
  logic [31:0]    core_data_rdata    [1];   // core input Data from tcm/mem
  logic           core_data_err      [1];   // Always 0

  // Not used because instruction doesn't
  logic           core_data_we       [1];
  logic [ 3:0]    core_data_be       [1];
  logic [31:0]    core_data_wdata    [1];
  // Memories for instruction fetch
  logic           data_mem_req    [NrDataMems]; // Input request from the internal core
  logic [31:0]    data_mem_addr   [NrDataMems]; // Input address from the internal core
  logic           data_mem_rvalid [NrDataMems]; // TCM/Mem confirms that the response is valid
  logic [31:0]    data_mem_rdata  [NrDataMems]; // TCM/Mem out data 
  logic           data_mem_err    [NrDataMems]; // TCM/Mem error 
  logic           data_mem_we     [NrDataMems];
  logic [ 3:0]    data_mem_be     [NrDataMems];
  logic [31:0]    data_mem_wdata  [NrDataMems];

  // Device address mapping
  logic [31:0] data_mem_addr_base [NrDataMems];
  logic [31:0] data_mem_addr_mask [NrDataMems];

  logic [31:0] data_tcm_a_addr;
  logic [31:0] i_tcm_a_addr;
  logic [31:0] i_tcm_b_addr;
  logic [31:0] rpm_addr;

  // Base addresses for IF mem and ram 
  //assign data_mem_addr_base[Out]    = MEM_OUT_START;
  //assign data_mem_addr_mask[Out]    = MEM_OUT_MASK;
  assign data_mem_addr_base[TcmInst]    = TCM_INST_START; // to change into tcm start
  assign data_mem_addr_mask[TcmInst]    = TCM_INST_MASK;
  assign data_mem_addr_base[TcmData]    = TCM_DATA_START; // to change into tcm start
  assign data_mem_addr_mask[TcmData]    = TCM_DATA_MASK;
  assign data_mem_addr_base[RpmMem]     = RPM_START; 
  assign data_mem_addr_mask[RpmMem]     = RPM_MASK;

  // Tie-off unused error signals
  //assign device_err[Out]  = 1'b0;
  assign data_mem_err[TcmInst] = 1'b0;

  assign data_tcm_a_addr = data_tcm_mask & data_mem_addr[TcmData];
  assign i_tcm_a_addr = i_tcm_mask & data_mem_addr[TcmInst];
  assign i_tcm_b_addr = i_tcm_mask & if_mem_instr_addr[TcmInst];
  assign rpm_addr = rpm_mask & data_mem_addr[RpmMem];

  // RPM optional signals

  logic			       							pmp_lsu_req, pmp_if_req;
  logic [31:0]			 						pmp_lsu_addr, pmp_if_addr;
  logic        									pmp_lsu_rvalid, pmp_if_rvalid;
  logic [31:0] 									pmp_lsu_rdata_b0, pmp_if_rdata_b0;
  logic [31:0] 									pmp_lsu_rdata_b1, pmp_if_rdata_b1;

  //////////////////////////////////////////////
  //    _____       _         ___             // 
  //    \_   \_ __ | |_      / __\_   _ ___   //
  //     / /\/ '_ \| __|    /__\// | | / __|  //
  //  /\/ /_ | | | | |_ _  / \/  \ |_| \__ \  //  
  //  \____/ |_| |_|\__(_) \_____/\__,_|___/  //
  //                                          //
  //////////////////////////////////////////////

  // Creating the internal bus similarly to demo system bus.
  cv32e41s_int_bus #(
    .NrDataMems    ( NrDataMems ),
    .NrInstMems    (NrInstMems),
    .NrHosts      ( 1   ),
    .DataWidth    ( 32        ),
    .AddressWidth ( 32        )
  ) u_int_bus (
    // Standard clock signals
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    
    // Data Hosts
    .host_req_data_i          (core_data_req     ),
    .host_gnt_data_o          (core_data_gnt     ),
    .host_addr_data_i         (core_data_addr    ),
    .host_we_data_i           (core_data_we      ),
    .host_be_data_i           (core_data_be      ),
    .host_wdata_data_i        (core_data_wdata   ),
    .host_rvalid_data_o       (core_data_rvalid  ),
    .host_rdata_data_o        (core_data_rdata   ),
    .host_err_data_o          (core_data_err     ),
    // Inst hosts
  
    .host_req_inst_i          (core_instr_req     ),
    .host_gnt_inst_o          (core_instr_gnt     ),
    .host_addr_inst_i         (core_instr_addr    ),
    .host_we_inst_i           (core_instr_we      ),
    .host_be_inst_i           (core_instr_be      ),
    .host_wdata_inst_i        (core_instr_wdata   ),
    .host_rvalid_inst_o       (core_instr_rvalid  ),
    .host_rdata_inst_o        (core_instr_rdata   ),
    .host_err_inst_o          (core_instr_err     ),
    // Data memories
    .mem_req_data_o        (data_mem_req   ),
    .mem_addr_data_o       (data_mem_addr  ),
    .mem_we_data_o         (data_mem_we    ),
    .mem_be_data_o         (data_mem_be    ),
    .mem_wdata_data_o      (data_mem_wdata ),
    .mem_rvalid_data_i     (data_mem_rvalid),
    .mem_rdata_data_i      (data_mem_rdata ),
    .mem_err_data_i        (data_mem_err   ),
    // Instr memories
    .mem_req_inst_o        (if_mem_instr_req   ),
    .mem_addr_inst_o       (if_mem_instr_addr  ),
    .mem_we_inst_o         (if_mem_instr_we    ),
    .mem_be_inst_o         (if_mem_instr_be    ),
    .mem_wdata_inst_o      (if_mem_instr_wdata ),
    .mem_rvalid_inst_i     (if_mem_instr_rvalid),
    .mem_rdata_inst_i      (if_mem_instr_rdata ),
    .mem_err_inst_i        (if_mem_instr_err   ),
    // Data mem base and mask
    .cfg_mem_data_addr_base(data_mem_addr_base),
    .cfg_mem_data_addr_mask(data_mem_addr_mask),
    // Inst mem base and mask
    .cfg_mem_inst_addr_base(if_mem_addr),
    .cfg_mem_inst_addr_mask(if_mem_addr_mask)
  );

////////////////////////////////////
//   _                            //
//  (_)_ __   ___ ___  _ __ ___   //
//  | | '_ \ / __/ _ \| '__/ _ \  //
//  | | | | | (_| (_) | | |  __/  //
//  |_|_| |_|\___\___/|_|  \___|  //
//                                //
////////////////////////////////////

  // Istantiate the incore, cv32e41s
  cv32e41s_incore #(
      .LIB                                     ( LIB ),
      .RV32                                    ( RV32 ),
      .B_EXT                                   ( B_EXT ),
      .M_EXT                                   ( M_EXT ),
      .DEBUG                                   ( DEBUG ),
      .DM_REGION_START                         ( DM_REGION_START ),
      .DM_REGION_END                           ( DM_REGION_END ),
      .DBG_NUM_TRIGGERS                        ( DBG_NUM_TRIGGERS ),
      .PMA_NUM_REGIONS                         ( PMA_NUM_REGIONS ),
      .CLIC                                    ( CLIC ),
      .CLIC_ID_WIDTH                           ( CLIC_ID_WIDTH ),
      .CLIC_INTTHRESHBITS                      ( CLIC_INTTHRESHBITS ),
      .PMP_GRANULARITY                         ( PMP_GRANULARITY ),
      .PMP_NUM_REGIONS                         ( PMP_NUM_REGIONS ),
      .PMP_MSECCFG_RV                          ( PMP_MSECCFG_RV ),
      .LFSR0_CFG                               ( LFSR0_CFG ),
      .LFSR1_CFG                               ( LFSR1_CFG ), // Do not use default value for LFSR configuration
      .LFSR2_CFG                               ( LFSR2_CFG ),
      
      .XPMP_ENABLE                             ( XPMP_ENABLE ),
      .XPMP_PMR_ENABLE                         ( XPMP_PMR_ENABLE ),              
      .XPMP_PMR_ENCODING                       ( XPMP_PMR_ENCODING ),   
      .XPMP_TRIE                               ( XPMP_TRIE ),   
      .XPMP_TRIE_CMP                           ( XPMP_TRIE_CMP )  

  ) u_incore (

  .clk_i (clk_i),
  .rst_ni (rst_ni),
  .scan_cg_en_i (scan_cg_en_i),   

  .boot_addr_i (boot_addr_i),
  .dm_exception_addr_i (dm_exception_addr_i),
  .dm_halt_addr_i (dm_halt_addr_i),
  .mhartid_i (mhartid_i),
  .mimpid_patch_i (mimpid_patch_i),
  .mtvec_addr_i (mtvec_addr_i),

  // Instruction memory interface
  .instr_req_o (core_instr_req[0]),
  .instr_gnt_i (1'b1),
  .instr_rvalid_i (core_instr_rvalid[0]),
  .instr_addr_o (core_instr_addr[0]),
  .instr_memtype_o (),
  .instr_prot_o (),
  .instr_dbg_o (),
  .instr_rdata_i (core_instr_rdata[0]),
  .instr_err_i ('0),

  .instr_reqpar_o (),           // secure
  .instr_gntpar_i ('0),         // secure
  .instr_rvalidpar_i ('0),      // secure
  .instr_achk_o (),             // secure
  .instr_rchk_i ('0),           // secure

  // Data memory interface
  .data_req_o (core_data_req[0]),
  .data_gnt_i (1'b1),
  .data_rvalid_i (core_data_rvalid[0]),
  .data_addr_o (core_data_addr[0]),
  .data_be_o (core_data_be[0]),
  .data_we_o (core_data_we[0]),
  .data_wdata_o (core_data_wdata[0]),
  .data_memtype_o (),
  .data_prot_o (),
  .data_dbg_o (),
  .data_rdata_i (core_data_rdata[0]),
  .data_err_i (core_data_err[0]),

  // RPM LSU implicit interface
  .pmp_lsu_req_o        ( pmp_lsu_req           ), 
  .pmp_lsu_addr_o       ( pmp_lsu_addr          ),
  .pmp_lsu_rvalid_i     ( pmp_lsu_rvalid        ),
  .pmp_lsu_rdata_b0_i   ( pmp_lsu_rdata_b0         ),
  .pmp_lsu_rdata_b1_i   ( pmp_lsu_rdata_b1         ),

  // RPM IF implicit interface
  .pmp_if_req_o        ( pmp_if_req             ), 
  .pmp_if_addr_o       ( pmp_if_addr            ),
  .pmp_if_rvalid_i     ( pmp_if_rvalid          ),
  .pmp_if_rdata_b0_i   ( pmp_if_rdata_b0           ),
  .pmp_if_rdata_b1_i   ( pmp_if_rdata_b1           ),

  .data_reqpar_o (),            // secure
  .data_gntpar_i ('0),          // secure
  .data_rvalidpar_i (),         // secure
  .data_achk_o (),              // secure
  .data_rchk_i (),              // secure

  // Cycle count
  .mcycle_o (mcycle_o),                

  // Basic interrupt architecture
  .irq_i (irq_i),

  // Event wakeup signals
  .wu_wfe_i (wu_wfe_i),   // Wait-for-event wakeup

  // CLIC interrupt architecture
  .clic_irq_i (clic_irq_i),
  .clic_irq_id_i (clic_irq_id_i),
  .clic_irq_level_i (clic_irq_level_i),
  .clic_irq_priv_i (clic_irq_priv_i),
  .clic_irq_shv_i (clic_irq_shv_i),

  // Fence.i flush handshake
  .fencei_flush_req_o (fencei_flush_req_o),
  .fencei_flush_ack_i (fencei_flush_ack_i),

  // Security Alerts
  .alert_minor_o (alert_minor_o),          
  .alert_major_o (alert_major_o),         
  // Debug interface
  .debug_req_i (debug_req_i),
  .debug_havereset_o (debug_havereset_o),
  .debug_running_o (debug_running_o),
  .debug_halted_o (debug_running_o),
  .debug_pc_valid_o (debug_pc_valid_o),
  .debug_pc_o (debug_pc_o),

  // CPU control signals
  .fetch_enable_i (fetch_enable_i),
  .core_sleep_o (core_sleep_o)
  );

  //////////////////////////////////////////////////////////////////////////////
  //   _____ _       _     _   _           ___                  _          _  //
  //  /__   (_) __ _| |__ | |_| |_   _    / __\___  _   _ _ __ | | ___  __| | //
  //    / /\/ |/ _` | '_ \| __| | | | |  / /  / _ \| | | | '_ \| |/ _ \/ _` | //  
  //   / /  | | (_| | | | | |_| | |_| | / /__| (_) | |_| | |_) | |  __/ (_| | //
  //   \/   |_|\__, |_| |_|\__|_|\__, | \____/\___/ \__,_| .__/|_|\___|\__,_| //
  //           |___/             |___/                   |_|                  //
  //                                                _                         //
  //                /\/\   ___ _ __ ___   ___  _ __(_) ___  ___               //  
  //               /    \ / _ \ '_ ` _ \ / _ \| '__| |/ _ \/ __|              //
  //              / /\/\ \  __/ | | | | | (_) | |  | |  __/\__ \              //
  //              \/    \/\___|_| |_| |_|\___/|_|  |_|\___||___/              //
  //                                                                          //
  //////////////////////////////////////////////////////////////////////////////
  
  // instantiating the Instr TCM
  /*cv32e41s_tcm #(
    .A_WID(32),
    .D_WID(32)
  )inst_tcm(
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    // Data port
    .a_req_i(data_mem_req[TcmInst]), 
    .a_we_i(data_mem_we[TcmInst]),
    .a_be_i(data_mem_be[TcmInst]),
    .a_addr_i(i_tcm_a_addr),
    .a_wdata_i(data_mem_wdata[TcmInst]),
    .a_rvalid_o(data_mem_rvalid[TcmInst]),
    .a_rdata_o(data_mem_rdata[TcmInst]),
    //IF port
    .b_req_i(if_mem_instr_req[TcmInst]),
    .b_we_i(1'b0), // not writing
    .b_be_i(4'b0), // 0 mask, because we're not writing
    .b_addr_i(i_tcm_b_addr),
    .b_wdata_i(32'b0), // not writing in the if port 
    .b_rvalid_o(if_mem_instr_rvalid[TcmInst]),
    .b_rdata_o(if_mem_instr_rdata[TcmInst])
  );*/

  cv32e41s_tcm #(
    .Depth(TCM_INST_SIZE / 4)
  )inst_tcm(
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    // Data port
    .a_req_i(data_mem_req[TcmInst]), 
    .a_we_i(data_mem_we[TcmInst]),
    .a_be_i(data_mem_be[TcmInst]),
    .a_addr_i(i_tcm_a_addr),
    .a_wdata_i(data_mem_wdata[TcmInst]),
    .a_rvalid_o(data_mem_rvalid[TcmInst]),
    .a_rdata_o(data_mem_rdata[TcmInst]),
    //IF port
    .b_req_i(if_mem_instr_req[TcmInst]),
    .b_we_i(1'b0), // not writing
    .b_be_i(4'b0), // 0 mask, because we're not writing
    .b_addr_i(i_tcm_b_addr),
    .b_wdata_i(32'b0), // not writing in the if port 
    .b_rvalid_o(if_mem_instr_rvalid[TcmInst]),
    .b_rdata_o(if_mem_instr_rdata[TcmInst])
  );

  // instantiating the Data TCM
  cv32e41s_tcm #(
    .Depth(TCM_DATA_SIZE / 4)
  )data_tcm(
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    // Data port
    .a_req_i(data_mem_req[TcmData]), 
    .a_we_i(data_mem_we[TcmData]),
    .a_be_i(data_mem_be[TcmData]),
    .a_addr_i(data_tcm_a_addr),
    .a_wdata_i(data_mem_wdata[TcmData]),
    .a_rvalid_o(data_mem_rvalid[TcmData]),
    .a_rdata_o(data_mem_rdata[TcmData]),
    //IF port, pending 
    .b_req_i(),
    .b_we_i(), 
    .b_be_i(), 
    .b_addr_i(),
    .b_wdata_i(), 
    .b_rvalid_o(),
    .b_rdata_o()
  );

////////////////////////////////////////////////////////////////////////////////////////
//     __            _                 ___           _            _   _               //
//    /__\ ___  __ _(_) ___  _ __     / _ \_ __ ___ | |_ ___  ___| |_(_) ___  _ __    //
//   / \/// _ \/ _` | |/ _ \| '_ \   / /_)/ '__/ _ \| __/ _ \/ __| __| |/ _ \| '_ \   //
//  / _  \  __/ (_| | | (_) | | | | / ___/| | | (_) | ||  __/ (__| |_| | (_) | | | |  //
//  \/ \_/\___|\__, |_|\___/|_| |_| \/    |_|  \___/ \__\___|\___|\__|_|\___/|_| |_|  //
//             |___/                                                                  //
//                                                                                    //
//                      /\/\   ___ _ __ ___   ___  _ __ _   _                         //
//                     /    \ / _ \ '_ ` _ \ / _ \| '__| | | |                        //
//                    / /\/\ \  __/ | | | | | (_) | |  | |_| |                        //
//                    \/    \/\___|_| |_| |_|\___/|_|   \__, |                        //
//                                                      |___/                         //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////

  // Optional: Instantiate the Region Protection Memory

  if(XPMP_ENABLE && XPMP_TRIE == PMP_TRIE_SUPPORT) begin: rpm_gen

    cv32e41s_rpm 
    #(
        .MemSize    (1024),
        .AddressWidth (10),
        .DataWidth  (32)

    ) rpm (
        .clk_i      (clk_i  ),
        .rst_ni     (rst_ni ),

        // This port is only accessible to M-mode, when implicit trie memory probing is inhibited
        .m_req_i    (data_mem_req[RpmMem]     ), 
        .m_we_i     (data_mem_we[RpmMem]      ),
        .m_be_i     (data_mem_be[RpmMem]      ),
        .m_addr_i   (rpm_addr                 ),
        .m_wdata_i  (data_mem_wdata[RpmMem]   ),
        .m_rvalid_o (data_mem_rvalid[RpmMem]  ),
        .m_rdata_o  (data_mem_rdata[RpmMem]   ),

        // the following ports are only accessible by non M-mode as read ports
        .imp_if_req_i    (pmp_if_req), 
        .imp_if_addr_i   (pmp_if_addr),
        .imp_if_rvalid_o (pmp_if_rvalid),
        .imp_if_rdata_b0_o  (pmp_if_rdata_b0),
        .imp_if_rdata_b1_o  (pmp_if_rdata_b1),

        .imp_lsu_req_i    (pmp_lsu_req), 
        .imp_lsu_addr_i   (pmp_lsu_addr),
        .imp_lsu_rvalid_o (pmp_lsu_rvalid),
        .imp_lsu_rdata_b0_o  (pmp_lsu_rdata_b0),
        .imp_lsu_rdata_b1_o  (pmp_lsu_rdata_b1)


    );

  end
  
  // Map out signals for IF
  assign  instr_req_o=if_mem_instr_req[Out]; 
  //assign  instr_gnt_i=core_instr_gnt[0]; // This signal is always one 
  assign  if_mem_instr_rvalid[Out]=instr_rvalid_i;
  assign  instr_addr_o=if_mem_instr_addr[Out];
  assign  if_mem_instr_rdata[Out]=instr_rdata_i;
  // Ram has an error signal differently from tcm that is always 0 
  assign  if_mem_instr_err[Out]=instr_err_i; 
  // Pending
  //assign  instr_memtype_o=
  //assign  instr_prot_o=,
  //assign  instr_dbg_o=,

  // Map Out signal for data 
  assign data_req_o=data_mem_req[Out]; 
  //assign host_gnt=data_mem_req[Out];
  assign data_mem_rvalid[Out]=data_rvalid_i;
  assign data_addr_o=data_mem_addr[Out];
  assign data_be_o=data_mem_be[Out];
  assign data_we_o=data_mem_we[Out];
  assign data_wdata_o = data_mem_wdata[Out];
  assign data_mem_rdata[Out]=data_rdata_i;
  assign data_mem_err[Out]=data_err_i;
  // Pending 
  //.data_memtype_o (),
  //.data_prot_o (),
  //.data_dbg_o (),

endmodule