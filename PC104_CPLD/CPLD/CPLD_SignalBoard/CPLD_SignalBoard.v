`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11/25/2013 
// Design Name: 
// Module Name:    信号板
// Project Name:   天大AFM
// Target Devices: AFM光隔控制系统
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module CPLD_SignalBoard(clk,OUT_SD,IN_SD,IO_A,IO_B,PID_PR,PID_INR,CPLD_STM_I,CPLD_AFM_TPI,CPLD_AFM_CNI);
	input clk; //1M
	input [7:0] IO_A;
	input [7:0] IO_B;
	input [15:8] OUT_SD;

	output [15:8] IN_SD;
   output [7:0] PID_PR;
   output [7:0] PID_INR;
	 
	 output CPLD_STM_I;
	 output CPLD_AFM_TPI;
	 output CPLD_AFM_CNI;
	
	assign CPLD_STM_I = OUT_SD[8];
	assign CPLD_AFM_TPI = OUT_SD[9];
	assign CPLD_AFM_CNI = OUT_SD[10];
	
	assign PID_PR[7:0] =	IO_A[7:0]; //光隔板已经将数据所存，此处只需直连即可
	assign PID_INR[7:0] =	IO_B[7:0];

endmodule
