`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:21:18 12/15/2008 
// Design Name: 
// Module Name:    SPM_PC104_AD 
// Project Name: 
// Target Devices: 
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
module CPLD_32Channel_forAD669(IOW,IOR,SA,SD,IO_A,IO_B,OUT_SD,IN_SD,IOCS16);
	
	input IOW;
   input IOR;

   input [11:0] SA;
	inout [15:0] SD;
	input [15:8] IN_SD;
	output [15:8] OUT_SD;	
	output [7:0] IO_A;
	output [7:0] IO_B;	
	output IOCS16;
	
	reg [7:0] IO_A;
	reg [7:0] IO_B;
		
//	assign CPLD_CPLD_SD_A_Adr = !(SA[11:0]==12'h200);  
//	assign CPLD_CPLD_SD_B_Adr = !(SA[11:0]==12'h201);
	assign CPLD_CPLD_SD_Adr = !(SA[11:0]==12'h200);
	assign CPLD_IN_Adr = !(SA[11:0]==12'h202);
	assign CPLD_OUT_Adr = !(SA[11:0]==12'h203);
	
	wire DX_WR_Adr,DY_WR_Adr,DZ_WR_Adr,SET_WR_Adr,X_GAIN_WR_Adr,X_OFFSET_WR_Adr,Y_GAIN_WR_Adr,Y_OFFSET_WR_Adr,AD669_Trigger;
	assign DX_WR_Adr = !(SA[11:0]==12'h210);  //320
	assign DY_WR_Adr = !(SA[11:0]==12'h212);
	assign DZ_WR_Adr = !(SA[11:0]==12'h214);
	assign SET_WR_Adr = !(SA[11:0]==12'h216);
	assign X_GAIN_WR_Adr = !(SA[11:0]==12'h218);  
	assign X_OFFSET_WR_Adr = !(SA[11:0]==12'h219);
	assign Y_GAIN_WR_Adr = !(SA[11:0]==12'h21A);
	assign Y_OFFSET_WR_Adr = !(SA[11:0]==12'h21B);
	assign AD669_Trigger = DX_WR_Adr & DY_WR_Adr & DZ_WR_Adr & SET_WR_Adr & X_GAIN_WR_Adr & X_OFFSET_WR_Adr & Y_GAIN_WR_Adr & Y_OFFSET_WR_Adr;
	
//	assign IO_A[7:0] = (AD669_Trigger) ? 16'bz : ~SD[7:0]; //启动DA前准备好数据
//	assign IO_B[7:0] = (AD669_Trigger) ? 16'bz : ~SD[15:8];
	always @ (negedge IOW)		
	begin
		if(!AD669_Trigger)
		begin
			IO_A[7:0] <= ~SD[7:0];
			IO_B[7:0] <= ~SD[15:8];
		end
	end
	
	assign OUT_SD[8] = ~(DX_WR_Adr|IOW); 
	assign OUT_SD[9] = ~(DY_WR_Adr|IOW); 
	assign OUT_SD[10] = ~(DZ_WR_Adr|IOW); 
	assign OUT_SD[11] = ~(SET_WR_Adr|IOW); 
	assign OUT_SD[12] = ~(X_GAIN_WR_Adr|IOW); 
	assign OUT_SD[13] = ~(X_OFFSET_WR_Adr|IOW); 
	assign OUT_SD[14] = ~(Y_GAIN_WR_Adr|IOW); 
	assign OUT_SD[15] = ~(Y_OFFSET_WR_Adr|IOW); 
	
/*	
	always @ (negedge IOW)		
	begin
		if(!CPLD_OUT_Adr)
		begin
			OUT_SD <= SD[7:0];
		end
		
		if(!CPLD_CPLD_SD_Adr)
		begin
			CPLD_SD[15:0] <= SD[15:0];
		end
	end	
*/

//	assign SD[7:0] = (IOR | CPLD_CPLD_SD_A_Adr ) ? 8'bz : CPLD_SD[7:0];
//	assign SD[7:0] = (IOR | CPLD_CPLD_SD_B_Adr ) ? 8'bz : CPLD_SD[15:8];
	assign SD[7:0] = (IOR | CPLD_IN_Adr ) ? 8'bz : ~IN_SD;
	
//	assign IOCS16 = (DX_WR_Adr & DY_WR_Adr & DZ_WR_Adr & SET_WR_Adr) ? 1'b1 : 1'b0; //16bit数据?地址选?
	assign IOCS16 = 1'bz;

endmodule

