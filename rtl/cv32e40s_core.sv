// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                 Halfdan Bechmann - halfdan.bechmann@silabs.com             //
//                 Øystein Knauserud - oystein.knauserud@silabs.com           //
//                 Michael Platzer - michael.platzer@tuwien.ac.at             //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40s_core import cv32e40s_pkg::*;
#(
  parameter NUM_MHPMCOUNTERS             =  1,
  parameter LIB                          =  0,
  parameter int PMP_GRANULARITY          =  0,
  parameter int PMP_NUM_REGIONS          =  0,
  parameter bit     A_EXT                =  0,
  parameter b_ext_e B_EXT                =  NONE,
  parameter bit     X_EXT                =  0,
  parameter int          PMA_NUM_REGIONS =  0,
  parameter pma_region_t PMA_CFG[PMA_NUM_REGIONS-1:0] = '{default:PMA_R_DEFAULT}
)
(
  // Clock and Reset
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        scan_cg_en_i,                     // Enable all clock gates for testing

  // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
  input  logic [31:0] boot_addr_i,
  input  logic [31:0] mtvec_addr_i,
  input  logic [31:0] dm_halt_addr_i,
  input  logic [31:0] hart_id_i,
  input  logic [31:0] dm_exception_addr_i,
  input  logic [31:0] nmi_addr_i,               // TODO:OK:low use

  // Instruction memory interface
  output logic        instr_req_o,
  input  logic        instr_gnt_i,
  input  logic        instr_rvalid_i,
  output logic [31:0] instr_addr_o,
  output logic [1:0]  instr_memtype_o,
  output logic [2:0]  instr_prot_o,
  input  logic [31:0] instr_rdata_i,
  input  logic        instr_err_i,

  // Data memory interface
  output logic        data_req_o,
  input  logic        data_gnt_i,
  input  logic        data_rvalid_i,
  output logic        data_we_o,
  output logic [3:0]  data_be_o,
  output logic [31:0] data_addr_o,
  output logic [1:0]  data_memtype_o,
  output logic [2:0]  data_prot_o,
  output logic [31:0] data_wdata_o,
  input  logic [31:0] data_rdata_i,
  input  logic        data_err_i,
  output logic [5:0]  data_atop_o,
  input  logic        data_exokay_i,

  // eXtension interface
  if_xif.cpu_compressed xif_compressed_if,
  if_xif.cpu_issue      xif_issue_if,
  if_xif.cpu_commit     xif_commit_if,
  if_xif.cpu_mem        xif_mem_if,
  if_xif.cpu_mem_result xif_mem_result_if,
  if_xif.cpu_result     xif_result_if,

  // Interrupt inputs
  input  logic [31:0] irq_i,                    // CLINT interrupts + CLINT extension interrupts

  // Fencei flush handshake
  output logic        fencei_flush_req_o,
  input logic         fencei_flush_ack_i,

  // Security Alerts
  output logic        alert_minor_o,
  output logic        alert_major_o,

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_havereset_o,
  output logic        debug_running_o,
  output logic        debug_halted_o,

  // CPU Control Signals
  input  logic        fetch_enable_i,
  output logic        core_sleep_o
);

  logic [31:0]       pc_if;             // Program counter in IF stage

  // Jump and branch target and decision (EX->IF)
  logic [31:0] jump_target_id;
  logic [31:0] branch_target_ex;
  logic        branch_decision_ex;

  // Busy signals
  logic        if_busy;
  logic        lsu_busy;

  // ID/EX pipeline
  id_ex_pipe_t id_ex_pipe;

  // EX/WB pipeline
  ex_wb_pipe_t ex_wb_pipe;

  // IF/ID pipeline
  if_id_pipe_t if_id_pipe;

  // Controller
  ctrl_byp_t   ctrl_byp;
  ctrl_fsm_t   ctrl_fsm;

  // Register File Write Back
  logic        rf_we_wb;
  rf_addr_t    rf_waddr_wb;
  logic [31:0] rf_wdata_wb;

  // Forwarding RF from EX
  logic [31:0] rf_wdata_ex;

  // Register file signals from ID/decoder to controller
  logic [REGFILE_NUM_READ_PORTS-1:0] rf_re_id;
  rf_addr_t    rf_raddr_id[REGFILE_NUM_READ_PORTS];
  rf_addr_t    rf_waddr_id;

  // Register file read data
  rf_data_t    regfile_rdata_id[REGFILE_NUM_READ_PORTS];

  // Register file write interface
  rf_addr_t    regfile_waddr_wb[REGFILE_NUM_WRITE_PORTS];
  rf_data_t    regfile_wdata_wb[REGFILE_NUM_WRITE_PORTS];
  logic        regfile_we_wb   [REGFILE_NUM_WRITE_PORTS];

  // Register file write enable for ALU insn in ID
  logic regfile_alu_we_id;

    // CSR control
  logic [23:0] mtvec_addr;
  logic [1:0]  mtvec_mode;

  logic [31:0] csr_rdata;
  logic csr_counter_read;

  privlvl_t     priv_lvl_lsu, priv_lvl;
  privlvlctrl_t priv_lvl_if_ctrl;

  mstatus_t     mstatus;

  // LSU
  logic        lsu_split_ex;
  mpu_status_e lsu_mpu_status_wb;
  logic [31:0] lsu_rdata_wb;
  logic        lsu_err_wb;
  logic [31:0] lsu_addr_wb;

  logic        lsu_valid_0;             // Handshake with EX
  logic        lsu_ready_ex;
  logic        lsu_valid_ex;
  logic        lsu_ready_0;

  logic        lsu_valid_1;             // Handshake with WB
  logic        lsu_ready_wb;
  logic        lsu_valid_wb;
  logic        lsu_ready_1;

  logic        data_stall_wb;

  // Stage ready signals
  logic        id_ready;
  logic        ex_ready;
  logic        wb_ready;

  // Stage valid signals
  logic        if_valid;
  logic        id_valid;
  logic        ex_valid;
  logic        wb_valid;

  // Interrupts
  logic        m_irq_enable; // interrupt_controller
  logic [31:0] mepc, dpc;    // from cs_registers
  logic [31:0] mie;          // from cs_registers
  logic [31:0] mip;          // from cs_registers

  // Signal from IF to init mtvec at boot time
  logic        csr_mtvec_init_if;

  // Major Alert Triggers
  logic        rf_ecc_err;
  logic        pc_err;
  logic        csr_err;
  logic        itf_int_err;

  // debug mode and dcsr configuration
  // From cs_registers
  dcsr_t       dcsr;

  // trigger match detected in cs_registers (using ID timing)
  logic        trigger_match_if;

  // Controller <-> decoder
  logic       mret_insn_id;
  logic       dret_insn_id;
  logic       wfi_insn_id;
  logic [1:0] ctrl_transfer_insn_id;
  logic [1:0] ctrl_transfer_insn_raw_id;

  logic        csr_en_id;
  csr_opcode_e csr_op_id;
  logic        csr_illegal;

  // irq signals
  // TODO:AB Should find a proper suffix for signals from interrupt_controller
  logic        irq_req_ctrl;
  logic [4:0]  irq_id_ctrl;
  logic        irq_wu_ctrl;

  // PMP CSR's
  pmp_csr_t csr_pmp;

  // Used (only) by verification environment
  logic        irq_ack;
  logic [4:0]  irq_id;
  logic        dbg_ack;
  
  // Internal OBI interfaces
  if_c_obi #(.REQ_TYPE(obi_inst_req_t), .RESP_TYPE(obi_inst_resp_t))  m_c_obi_instr_if();
  if_c_obi #(.REQ_TYPE(obi_data_req_t), .RESP_TYPE(obi_data_resp_t))  m_c_obi_data_if();

  // Connect toplevel OBI signals to internal interfaces
  assign instr_req_o                         = m_c_obi_instr_if.s_req.req;
  assign instr_addr_o                        = m_c_obi_instr_if.req_payload.addr;
  assign instr_memtype_o                     = m_c_obi_instr_if.req_payload.memtype;
  assign instr_prot_o                        = m_c_obi_instr_if.req_payload.prot;
  assign m_c_obi_instr_if.s_gnt.gnt          = instr_gnt_i;
  assign m_c_obi_instr_if.s_rvalid.rvalid    = instr_rvalid_i;
  assign m_c_obi_instr_if.resp_payload.rdata = instr_rdata_i;
  assign m_c_obi_instr_if.resp_payload.err   = instr_err_i;

  assign data_req_o                          = m_c_obi_data_if.s_req.req;
  assign data_we_o                           = m_c_obi_data_if.req_payload.we;
  assign data_be_o                           = m_c_obi_data_if.req_payload.be;
  assign data_addr_o                         = m_c_obi_data_if.req_payload.addr;
  assign data_memtype_o                      = m_c_obi_data_if.req_payload.memtype;
  assign data_prot_o                         = m_c_obi_data_if.req_payload.prot;
  assign data_wdata_o                        = m_c_obi_data_if.req_payload.wdata;
  assign data_atop_o                         = m_c_obi_data_if.req_payload.atop;
  assign m_c_obi_data_if.s_gnt.gnt           = data_gnt_i;
  assign m_c_obi_data_if.s_rvalid.rvalid     = data_rvalid_i;
  assign m_c_obi_data_if.resp_payload.rdata  = data_rdata_i;
  assign m_c_obi_data_if.resp_payload.err    = data_err_i;
  assign m_c_obi_data_if.resp_payload.exokay = data_exokay_i;

  assign debug_havereset_o = ctrl_fsm.debug_havereset;
  assign debug_halted_o    = ctrl_fsm.debug_halted;
  assign debug_running_o   = ctrl_fsm.debug_running;

  // Used (only) by verification environment
  assign irq_ack = ctrl_fsm.irq_ack;
  assign irq_id  = ctrl_fsm.irq_id;
  assign dbg_ack = ctrl_fsm.dbg_ack;

  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  logic        clk;
  logic        fetch_enable;

  cv32e40s_sleep_unit
  #(
    .LIB                        ( LIB                  )
  )
  sleep_unit_i
  (
    // Clock, reset interface
    .clk_ungated_i              ( clk_i                ),       // Ungated clock
    .rst_n                      ( rst_ni               ),
    .clk_gated_o                ( clk                  ),       // Gated clock
    .scan_cg_en_i               ( scan_cg_en_i         ),

    // Core sleep
    .core_sleep_o               ( core_sleep_o         ),

    // Fetch enable
    .fetch_enable_i             ( fetch_enable_i       ),
    .fetch_enable_o             ( fetch_enable         ),

    // Core status
    .if_busy_i                  ( if_busy              ),
    .lsu_busy_i                 ( lsu_busy             ),

    // Inputs from controller (including busy)
    .ctrl_fsm_i                 ( ctrl_fsm             )
  );

  /////////////////////////////////////
  //      _    _           _         //
  //     / \  | | ___ _ __| |_ ___   //
  //    / _ \ | |/ _ \ '__| __/ __|  //
  //   / ___ \| |  __/ |  | |_\__ \  //
  //  /_/   \_\_|\___|_|   \__|___/  //
  //                                 //
  /////////////////////////////////////

  assign pc_err          = 1'b0; // todo: connect when hardened pc implemented
  assign itf_int_err     = 1'b0; // todo: connect when interface integrity implemented

  cv32e40s_alert
    alert_i
      (.clk                 ( clk               ),
       .rst_n               ( rst_ni            ),

       // Alert Triggers
       .ctrl_fsm_i          ( ctrl_fsm          ),
       .rf_ecc_err_i        ( rf_ecc_err        ),
       .pc_err_i            ( pc_err            ),
       .csr_err_i           ( csr_err           ),
       .itf_int_err_i       ( itf_int_err       ),

       // Trigger Outputs
       .alert_minor_o       ( alert_minor_o     ),
       .alert_major_o       ( alert_major_o     )
       );


  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  cv32e40s_if_stage
    #(.A_EXT(A_EXT),
      .PMP_GRANULARITY(PMP_GRANULARITY),
      .PMP_NUM_REGIONS(PMP_NUM_REGIONS),
      .X_EXT      ( X_EXT ),
      .PMA_NUM_REGIONS(PMA_NUM_REGIONS),
      .PMA_CFG(PMA_CFG))
  if_stage_i
  (
    .clk                 ( clk                       ),
    .rst_n               ( rst_ni                    ),

    // boot address
    .boot_addr_i         ( boot_addr_i[31:0]         ),
    .dm_exception_addr_i ( dm_exception_addr_i[31:0] ),

    // NMI address
    .nmi_addr_i          ( nmi_addr_i                ),

    // debug mode halt address
    .dm_halt_addr_i      ( dm_halt_addr_i[31:0]      ),

    // trap vector location
    .mtvec_addr          ( mtvec_addr                ),

    // instruction cache interface
    .m_c_obi_instr_if    ( m_c_obi_instr_if          ),

    // CSR registers
    .csr_pmp_i           ( csr_pmp                   ),

    // Privilege level
    .priv_lvl_ctrl_i     ( priv_lvl_if_ctrl          ),

    // IF/ID pipeline
    .if_id_pipe_o        ( if_id_pipe                ),

    .ex_wb_pipe_i        ( ex_wb_pipe                ),

    .ctrl_fsm_i          ( ctrl_fsm                  ),

    .mepc_i              ( mepc                      ), // exception return address

    .dpc_i               ( dpc                       ), // debug return address

    .trigger_match_i     ( trigger_match_if          ),

    .pc_if_o             ( pc_if                     ),

    .csr_mtvec_init_o    ( csr_mtvec_init_if         ),

    // Jump targets
    .jump_target_id_i    ( jump_target_id            ),
    .branch_target_ex_i  ( branch_target_ex          ),

    .if_busy_o           ( if_busy                   ),

    // Pipeline handshakes
    .if_valid_o          ( if_valid                  ),
    .id_ready_i          ( id_ready                  ),

    // eXtension interface
    .xif_compressed_if   ( xif_compressed_if         ),
    .xif_issue_valid_i   ( xif_issue_if.issue_valid  )
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  cv32e40s_id_stage
  #(
    .A_EXT                        ( A_EXT                     ),
    .B_EXT                        ( B_EXT                     ),
    .X_EXT                        ( X_EXT                     )
  )
  id_stage_i
  (
    .clk                          ( clk                       ),     // Gated clock
    .clk_ungated_i                ( clk_i                     ),     // Ungated clock
    .rst_n                        ( rst_ni                    ),

    // Jumps and branches
    .jmp_target_o                 ( jump_target_id            ),

    // IF/ID pipeline
    .if_id_pipe_i                 ( if_id_pipe                ),

    // ID/EX pipeline
    .id_ex_pipe_o                 ( id_ex_pipe                ),

    // EX/WB pipeline
    .ex_wb_pipe_i                 ( ex_wb_pipe                ),

    // Controller
    .ctrl_byp_i                   ( ctrl_byp                  ),
    .ctrl_fsm_i                   ( ctrl_fsm                  ),

    // CSR ID/EX
    .mstatus_i                    ( mstatus                   ),

    // Register file write back and forwards
    .rf_wdata_ex_i                ( rf_wdata_ex               ),
    .rf_wdata_wb_i                ( rf_wdata_wb               ),

    .mret_insn_o                  ( mret_insn_id              ),
    .dret_insn_o                  ( dret_insn_id              ),
    .wfi_insn_o                   ( wfi_insn_id               ),

    .csr_en_o                     ( csr_en_id                 ),
    .csr_op_o                     ( csr_op_id                 ),

    .ctrl_transfer_insn_o         ( ctrl_transfer_insn_id     ),
    .ctrl_transfer_insn_raw_o     ( ctrl_transfer_insn_raw_id ),

    .rf_re_o                      ( rf_re_id                  ),
    .rf_raddr_o                   ( rf_raddr_id               ),
    .rf_waddr_o                   ( rf_waddr_id               ),

    .regfile_alu_we_id_o          ( regfile_alu_we_id         ),
    .regfile_rdata_i              ( regfile_rdata_id          ),

    // Pipeline handshakes
    .id_ready_o                   ( id_ready                  ),
    .id_valid_o                   ( id_valid                  ),
    .ex_ready_i                   ( ex_ready                  ),

    // eXtension interface
    .xif_issue_if                 ( xif_issue_if              )
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////

  cv32e40s_ex_stage ex_stage_i
  (
    .clk                        ( clk                          ),
    .rst_n                      ( rst_ni                       ),

    // ID/EX pipeline
    .id_ex_pipe_i               ( id_ex_pipe                   ),

    // EX/WB pipeline
    .ex_wb_pipe_o               ( ex_wb_pipe                   ),

    // From controller FSM
    .ctrl_fsm_i                 ( ctrl_fsm                     ),

    // CSR interface
    .csr_rdata_i                ( csr_rdata                    ),
    .csr_illegal_i              ( csr_illegal                  ),

    // Branch decision
    .branch_decision_o          ( branch_decision_ex           ),
    .branch_target_o            ( branch_target_ex             ),

    // Register file forwarding
    .rf_wdata_o                 ( rf_wdata_ex                  ),

    // LSU interface
    .lsu_valid_i                ( lsu_valid_0                  ),
    .lsu_ready_o                ( lsu_ready_ex                 ),
    .lsu_valid_o                ( lsu_valid_ex                 ),
    .lsu_ready_i                ( lsu_ready_0                  ),
    .lsu_split_i                ( lsu_split_ex                 ),

    // Pipeline handshakes
    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     ),
    .wb_ready_i                 ( wb_ready                     )
  );

  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  cv32e40s_load_store_unit
    #(.A_EXT(A_EXT),
      .PMP_GRANULARITY(PMP_GRANULARITY),
      .PMP_NUM_REGIONS(PMP_NUM_REGIONS),
      .PMA_NUM_REGIONS(PMA_NUM_REGIONS),
      .PMA_CFG(PMA_CFG))
  load_store_unit_i
  (
    .clk                   ( clk                ),
    .rst_n                 ( rst_ni             ),

    // ID/EX pipeline
    .id_ex_pipe_i          ( id_ex_pipe         ),

    // Controller
    .ctrl_fsm_i            ( ctrl_fsm           ),

    // Data OBI interface
    .m_c_obi_data_if       ( m_c_obi_data_if    ),

    // Control signals
    .busy_o                ( lsu_busy           ),

    // Stage 0 outputs (EX)
    .lsu_split_0_o         ( lsu_split_ex       ),
    .lsu_mpu_status_1_o    ( lsu_mpu_status_wb  ),

    // Stage 1 outputs (WB)
    .lsu_addr_1_o          ( lsu_addr_wb        ), // To controller (has WB timing, but does not pass through WB stage)
    .lsu_err_1_o           ( lsu_err_wb         ), // To controller (has WB timing, but does not pass through WB stage)
    .lsu_rdata_1_o         ( lsu_rdata_wb       ),

    // CSR registers
    .csr_pmp_i             ( csr_pmp            ),

    // Privilege level
    .priv_lvl_lsu_i        ( priv_lvl_lsu       ),

    // Valid/ready
    .valid_0_i             ( lsu_valid_ex       ), // First LSU stage (EX)
    .ready_0_o             ( lsu_ready_0        ),
    .valid_0_o             ( lsu_valid_0        ),
    .ready_0_i             ( lsu_ready_ex       ),

    .valid_1_i             ( lsu_valid_wb       ), // Second LSU stage (WB)
    .ready_1_o             ( lsu_ready_1        ),
    .valid_1_o             ( lsu_valid_1        ),
    .ready_1_i             ( lsu_ready_wb       ),

    // eXtension interface
    .xif_mem_if            ( xif_mem_if         ),
    .xif_mem_result_if     ( xif_mem_result_if  )
  );

  ////////////////////////////////////////////////////////////////////////////////////////
  // Write back stage                                                                   //
  ////////////////////////////////////////////////////////////////////////////////////////

  cv32e40s_wb_stage
  wb_stage_i
  (
    .clk                        ( clk                          ), // Not used in RTL; only used by assertions
    .rst_n                      ( rst_ni                       ), // Not used in RTL; only used by assertions

    // EX/WB pipeline
    .ex_wb_pipe_i               ( ex_wb_pipe                   ),

    // Controller
    .ctrl_fsm_i                 ( ctrl_fsm                     ),

    // LSU
    .lsu_rdata_i                ( lsu_rdata_wb                 ),
    .lsu_mpu_status_i           ( lsu_mpu_status_wb            ),

    // Write back to register file
    .rf_we_wb_o                 ( rf_we_wb                     ),
    .rf_waddr_wb_o              ( rf_waddr_wb                  ),
    .rf_wdata_wb_o              ( rf_wdata_wb                  ),

    // LSU handshakes
    .lsu_valid_i                ( lsu_valid_1                  ),
    .lsu_ready_o                ( lsu_ready_wb                 ),
    .lsu_valid_o                ( lsu_valid_wb                 ),
    .lsu_ready_i                ( lsu_ready_1                  ),

    .data_stall_o               ( data_stall_wb                ),

    // Valid/ready
    .wb_ready_o                 ( wb_ready                     ),
    .wb_valid_o                 ( wb_valid                     ),

    // eXtension interface
    .xif_result_if              ( xif_result_if                )
  );

  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  cv32e40s_cs_registers
  #(
    .A_EXT            ( A_EXT                 ),
    .PMP_NUM_REGIONS  ( PMP_NUM_REGIONS       ),
    .PMP_GRANULARITY  ( PMP_GRANULARITY       ),
    .NUM_MHPMCOUNTERS ( NUM_MHPMCOUNTERS      )
  )
  cs_registers_i
  (
    .clk                        ( clk                    ),
    .rst_n                      ( rst_ni                 ),

    // Hart ID from outside
    .hart_id_i                  ( hart_id_i              ),

    .mtvec_addr_o               ( mtvec_addr             ),
    .mtvec_mode_o               ( mtvec_mode             ),

    // mtvec address
    .mtvec_addr_i               ( mtvec_addr_i[31:0]     ),
    .csr_mtvec_init_i           ( csr_mtvec_init_if      ),

    // IF/ID pipeline
    .if_id_pipe_i               ( if_id_pipe             ),
    .mret_id_i                  ( mret_insn_id           ),
    // ID/EX pipeline
    .id_ex_pipe_i               ( id_ex_pipe             ),

    // EX/WB pipeline
    .ex_wb_pipe_i               ( ex_wb_pipe             ),

    // From controller FSM
    .ctrl_fsm_i                 ( ctrl_fsm               ),

    // Interface to CSRs (SRAM like)
    .csr_rdata_o                ( csr_rdata              ),

    .csr_illegal_o              (csr_illegal             ),

    // Raddr from first stage (EX)
    .csr_counter_read_o         ( csr_counter_read       ),

    // Interrupt related control signals
    .mie_o                      ( mie                    ),
    .mip_i                      ( mip                    ),
    .m_irq_enable_o             ( m_irq_enable           ),
    .mepc_o                     ( mepc                   ),
    .mstatus_o                  ( mstatus                ),
 
    // PMP CSR's    
    .csr_pmp_o                  ( csr_pmp                ),

    // CSR Parity Error
    .csr_err_o                  ( csr_err                ),

    // debug
    .dpc_o                      ( dpc                    ),
    .dcsr_o                     ( dcsr                   ),
    .trigger_match_o            ( trigger_match_if       ),

    .priv_lvl_if_ctrl_o         ( priv_lvl_if_ctrl       ),
    .priv_lvl_lsu_o             ( priv_lvl_lsu           ),    
    .priv_lvl_o                 ( priv_lvl               ),
   
    .pc_if_i                    ( pc_if                  )
  );

  ////////////////////////////////////////////////////////////////////
  //    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //   / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  //  | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  //  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //   \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                //
  ////////////////////////////////////////////////////////////////////

  cv32e40s_controller
  #(
    .X_EXT                          ( X_EXT                  )
  )
  controller_i
  (
    .clk                            ( clk                    ),         // Gated clock
    .clk_ungated_i                  ( clk_i                  ),         // Ungated clock
    .rst_n                          ( rst_ni                 ),

    .fetch_enable_i                 ( fetch_enable           ),

    // From ID/EX pipeline
    .id_ex_pipe_i                   ( id_ex_pipe             ),

    .csr_counter_read_i             ( csr_counter_read       ),

    // From EX/WB pipeline
    .ex_wb_pipe_i                   ( ex_wb_pipe             ),

    .if_valid_i                     ( if_valid               ),

    // from IF/ID pipeline
    .if_id_pipe_i                   ( if_id_pipe             ),
    .mret_id_i                      ( mret_insn_id           ),
    .dret_id_i                      ( dret_insn_id           ),
    .csr_en_id_i                    ( csr_en_id              ),
    .csr_op_id_i                    ( csr_op_id              ),
    .wfi_id_i                       ( wfi_insn_id            ),

    // LSU
    .lsu_split_ex_i                 ( lsu_split_ex           ),
    .lsu_mpu_status_wb_i            ( lsu_mpu_status_wb      ),
    .data_stall_wb_i                ( data_stall_wb          ),
    .lsu_addr_wb_i                  ( lsu_addr_wb            ),
    .lsu_err_wb_i                   ( lsu_err_wb             ),

    // jump/branch control
    .branch_decision_ex_i           ( branch_decision_ex     ),
    .ctrl_transfer_insn_i           ( ctrl_transfer_insn_id  ),
    .ctrl_transfer_insn_raw_i       ( ctrl_transfer_insn_raw_id ),

    // Interrupt signals
    .irq_wu_ctrl_i                  ( irq_wu_ctrl            ),
    .irq_req_ctrl_i                 ( irq_req_ctrl           ),
    .irq_id_ctrl_i                  ( irq_id_ctrl            ),

    // Priviledge level
    .priv_lvl_i                     ( priv_lvl               ),

    // From CSR registers
    .mtvec_mode_i                   ( mtvec_mode             ),

    // Debug signals
    .debug_req_i                    ( debug_req_i            ),
    .dcsr_i                         ( dcsr                   ),

    // Register File read, write back and forwards
    .rf_re_i                        ( rf_re_id               ),
    .rf_raddr_i                     ( rf_raddr_id            ),
    .rf_waddr_i                     ( rf_waddr_id            ),

    // Write targets from ID
    .regfile_alu_we_id_i            ( regfile_alu_we_id      ),
    
    // Fencei flush handshake
    .fencei_flush_ack_i             ( fencei_flush_ack_i     ),
    .fencei_flush_req_o             ( fencei_flush_req_o     ),

    // Data OBI interface
    .m_c_obi_data_if                ( m_c_obi_data_if        ),

    .id_ready_i                     ( id_ready               ),
    .id_valid_i                     ( id_valid               ),
    .ex_ready_i                     ( ex_ready               ),
    .ex_valid_i                     ( ex_valid               ),
    .wb_ready_i                     ( wb_ready               ),
    .wb_valid_i                     ( wb_valid               ),

    .ctrl_byp_o                     ( ctrl_byp               ),
    .ctrl_fsm_o                     ( ctrl_fsm               ),

    // eXtension interface
    .xif_commit_if                  ( xif_commit_if          )
 );

////////////////////////////////////////////////////////////////////////
//  _____      _       _____             _             _ _            //
// |_   _|    | |     /  __ \           | |           | | |           //
//   | | _ __ | |_    | /  \/ ___  _ __ | |_ _ __ ___ | | | ___ _ __  //
//   | || '_ \| __|   | |    / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| //
//  _| || | | | |_ _  | \__/\ (_) | | | | |_| | | (_) | | |  __/ |    //
//  \___/_| |_|\__(_)  \____/\___/|_| |_|\__|_|  \___/|_|_|\___|_|    //
//                                                                    //
////////////////////////////////////////////////////////////////////////


  cv32e40s_int_controller
  int_controller_i
  (
    .clk                  ( clk                ),
    .rst_n                ( rst_ni             ),

    // External interrupt lines
    .irq_i                ( irq_i              ),

    // To cv32e40s_controller
    .irq_req_ctrl_o       ( irq_req_ctrl       ),
    .irq_id_ctrl_o        ( irq_id_ctrl        ),
    .irq_wu_ctrl_o        ( irq_wu_ctrl        ),

    // To/from with cv32e40s_cs_registers
    .mie_i                ( mie                ),
    .mip_o                ( mip                ),
    .m_irq_enable_i       ( m_irq_enable       )
  );

    /////////////////////////////////////////////////////////
  //  ____  _____ ____ ___ ____ _____ _____ ____  ____   //
  // |  _ \| ____/ ___|_ _/ ___|_   _| ____|  _ \/ ___|  //
  // | |_) |  _|| |  _ | |\___ \ | | |  _| | |_) \___ \  //
  // |  _ <| |__| |_| || | ___) || | | |___|  _ < ___) | //
  // |_| \_\_____\____|___|____/ |_| |_____|_| \_\____/  //
  //                                                     //
  /////////////////////////////////////////////////////////

  // Connect register file write port(s) to regfile inputs
  assign regfile_we_wb[0]    = rf_we_wb;
  assign regfile_waddr_wb[0] = rf_waddr_wb;
  assign regfile_wdata_wb[0] = rf_wdata_wb;

  cv32e40s_register_file_wrapper
  register_file_wrapper_i
  (
    .clk                ( clk                ),
    .rst_n              ( rst_ni             ),

    // ECC error output
    .ecc_err_o          ( rf_ecc_err         ),

    // Read ports
    .raddr_i            ( rf_raddr_id        ),
    .rdata_o            ( regfile_rdata_id   ), // todo:AB get consistent naming

    // Write ports
    .waddr_i            ( regfile_waddr_wb   ),
    .wdata_i            ( regfile_wdata_wb   ),
    .we_i               ( regfile_we_wb      )
  );

endmodule