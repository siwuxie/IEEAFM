`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    03/31/2010 
// Design Name: 
// Module Name:    AD669 4Chips
// Project Name:   FAST_AFM
// Target Devices: 实验室AFM系统
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
module CPLD_AD7671_4Chips(clk,OUT_SD,IN_SD,IO_A,IO_B,CPLD_SD,CPLD_408_A0,CPLD_408_A1,CPLD_408_A2,
CPLD_U1_RD,CPLD_U2_RD,CPLD_U3_RD,CPLD_U4_RD,CPLD_U1234_WR,CPLD_U1_BUSY,CPLD_U2_BUSY,CPLD_U3_BUSY,CPLD_U4_BUSY);
	input clk; 
	output [7:0] IO_A;
	output [7:0] IO_B;
	input [15:8] OUT_SD;
	output [15:8] IN_SD;
   input [15:0] CPLD_SD;
	
   output CPLD_408_A0;
   output CPLD_408_A1;
	output CPLD_408_A2;
	output CPLD_U1_RD;
	output CPLD_U2_RD;
	output CPLD_U3_RD;
	output CPLD_U4_RD;
	output CPLD_U1234_WR;
	input CPLD_U1_BUSY;
	input CPLD_U2_BUSY;
	input CPLD_U3_BUSY;
	input CPLD_U4_BUSY;
	
	assign IO_B[0] = CPLD_U1_BUSY;
	assign IO_B[1] = CPLD_U2_BUSY;
	assign IO_B[2] = CPLD_U3_BUSY;
	assign IO_B[3] = CPLD_U4_BUSY;
	
	assign CPLD_408_A0 = OUT_SD[8];
	assign CPLD_408_A1 = OUT_SD[9];
	assign CPLD_408_A2 = OUT_SD[10];
	assign CPLD_U1234_WR = OUT_SD[11];
	assign CPLD_U1_RD = OUT_SD[12];
	assign CPLD_U2_RD = OUT_SD[13];
	assign CPLD_U3_RD = OUT_SD[14];
	assign CPLD_U4_RD = OUT_SD[15];
	
	assign {IN_SD[15:8],IO_A[7:0]} = CPLD_SD[15:0]; //光隔板丫数据所存,此处只需直连即可
//	assign IO_A[7:0] = 8'hA5;
//	assign IN_SD[15:8] = 8'h5A;

endmodule
