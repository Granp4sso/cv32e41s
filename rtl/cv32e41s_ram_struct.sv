//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.06.2023 18:28:19
// Design Name: 
// Module Name: ram_dual_port
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// RAM Inference using Struct in SV(True Dual port)
// File:rams_tdp_struct.sv
typedef logic [31:0] memPkt;

module cv32e41s_ram_struct #(
    parameter A_WID     = 32,
    parameter MEM_SIZE  = 4096,
    parameter D_WID     = 32
)
(
    input clka_i,                   // Clock for port A
    input clkb_i,                   // Clock for port B 
    input wea_i,                    // Write enable for port A
    input web_i,                    // Write enable for port B 
    input ena_i,                    // Enable port A
    input enb_i,                    // Enable Port B 
    input [A_WID-1:0] addra_i,      // Write/Read Address on port A 
    input [A_WID-1:0] addrb_i,      // Write/Read Address on port B 
    input memPkt da_i,db_i,         // Data Inputs on ports a and b
    output memPkt douta_o, doutb_o  // Data outputs on port a and b
);

memPkt mem [MEM_SIZE-1:0];

always @ (posedge clka_i) begin
    if (ena_i) begin
        douta_o <= mem[addra_i];
        if(wea_i) begin 
            mem[addra_i] <= da_i;
        end
    end
end

always @ (posedge clkb_i) begin
    if (enb_i) begin
        doutb_o <= mem[addrb_i]; 
        if(web_i) begin
            mem[addrb_i] <= db_i;
        end
    end
end

endmodule