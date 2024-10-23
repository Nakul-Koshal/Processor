`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.09.2024 16:27:19
// Design Name: 
// Module Name: Processor_tb
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


module Processor_tb();
reg clk = 0,sys_rst = 0;
reg [15:0] din = 0;
wire [15:0] dout;

integer i = 0;
  
Processor dut(.clk(clk),.sys_rst(sys_rst),.din(din),.dout(dout));
 
always #5 clk = ~clk;
 
initial begin
sys_rst = 1'b1;
repeat(5) @(posedge clk);
sys_rst = 1'b0;
end
 
endmodule
