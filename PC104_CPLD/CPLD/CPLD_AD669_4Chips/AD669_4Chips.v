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
module AD669_4Chips(clk,OUT_SD,IN_SD,IO_A,IO_B,CPLD_SD,DX_WR,DY_WR,DZ_WR,SET_WR,
				X_GAIN_WR,X_OFFSET_WR,Y_GAIN_WR,Y_OFFSET_WR);
	input clk; //1M
	input [7:0] IO_A;
	input [7:0] IO_B;
	input [15:8] OUT_SD;

	output [15:8] IN_SD;
   output [15:0] CPLD_SD;
   output DX_WR;
   output DY_WR;
	output DZ_WR;
	output SET_WR;
	output X_GAIN_WR;
	output X_OFFSET_WR;
	output Y_GAIN_WR;
	output Y_OFFSET_WR;
	
	assign DX_WR = OUT_SD[8];
	assign DY_WR = OUT_SD[9];
	assign DZ_WR = OUT_SD[10];
	assign SET_WR = OUT_SD[11];
	assign X_GAIN_WR = OUT_SD[12];
	assign X_OFFSET_WR = OUT_SD[13];
	assign Y_GAIN_WR = OUT_SD[14];
	assign Y_OFFSET_WR = OUT_SD[15];
	
	assign CPLD_SD[15:0] =	{IO_B[7:0],IO_A[7:0]}; //光隔板已经将数据所存，此处只需直连即可

/*	
	wire DX_WR_Adr,DY_WR_Adr,DZ_WR_Adr,SET_WR_Adr;
	assign DX_WR_Adr = !(SA[11:0]==12'h210);  //320
	assign DY_WR_Adr = !(SA[11:0]==12'h212);
	assign DZ_WR_Adr = !(SA[11:0]==12'h214);
	assign SET_WR_Adr = !(SA[11:0]==12'h216);
	
	assign DX_WR = DX_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD = (DX_WR_Adr)?16'bz:SD; //启动DA前准备好数据
	
	assign DY_WR = DY_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD = (DY_WR_Adr)?16'bz:SD; //启动DA前准备好数据
	
	assign DZ_WR = DZ_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD = (DZ_WR_Adr)?16'bz:SD; //启动DA前准备好数据
	
	assign SET_WR = SET_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD = (SET_WR_Adr)?16'bz:SD; //启动DA前准备好数据
	
	wire X_GAIN_WR_Adr,X_OFFSET_WR_Adr,Y_GAIN_WR_Adr,Y_OFFSET_WR_Adr;
	assign X_GAIN_WR_Adr = !(SA[11:0]==12'h218);  
	assign X_OFFSET_WR_Adr = !(SA[11:0]==12'h219);
	assign Y_GAIN_WR_Adr = !(SA[11:0]==12'h21A);
	assign Y_OFFSET_WR_Adr = !(SA[11:0]==12'h21B);
	
	assign X_GAIN_WR = X_GAIN_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD[7:0] = (X_GAIN_WR_Adr)?8'bz:SD[7:0]; //启动DA前准备好数据
	
	assign X_OFFSET_WR = X_OFFSET_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD[7:0] = (X_OFFSET_WR_Adr)?8'bz:SD[7:0]; //启动DA前准备好数据
	
	assign Y_GAIN_WR = Y_GAIN_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD[7:0] = (Y_GAIN_WR_Adr)?8'bz:SD[7:0]; //启动DA前准备好数据
	
	assign Y_OFFSET_WR = Y_OFFSET_WR_Adr|IOW;  //作为启动DA转换信号
	assign CPLD_SD[7:0] = (Y_OFFSET_WR_Adr)?8'bz:SD[7:0]; //启动DA前准备好数据
*/

//16bit传输启动在AD板上完成	
//	assign IOCS16 = (DX_WR_Adr & DY_WR_Adr & DZ_WR_Adr & SET_WR_Adr)?1'b1:1'b0; //16bit数据传输,地址选通有效

endmodule
