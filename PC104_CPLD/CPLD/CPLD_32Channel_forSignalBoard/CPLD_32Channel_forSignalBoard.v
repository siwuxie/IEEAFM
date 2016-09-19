`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:21:18 12/15/2008 
// Design Name: 
// Module Name:    SPM_PC104_SIGNAL
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
module CPLD_32Channel_forSignalBoard(IOW,IOR,SA,SD,IO_A,IO_B,OUT_SD,IN_SD,IOCS16);
	input IOW;
   input IOR;

   input [11:0] SA;
	inout [15:0] SD;
	input [15:8] IN_SD;
	output [15:8] OUT_SD;	
	output [7:0] IO_A;
	output [7:0] IO_B;	
	output IOCS16;	 
	 
	 wire SPM_MODE_ADR,PID_PR_ADR,PID_INR_ADR;
	 assign SPM_MODE_ADR = !(SA[11:0]==12'h240);  //控制字	
	 assign PID_PR_ADR = !(SA[11:0]==12'h241);
	 assign PID_INR_ADR = !(SA[11:0]==12'h242);	 	 
	
	reg [15:8] OUT_SD;
	reg[7:0] IO_A; 	//比例
	reg[7:0] IO_B;	//积分
	always @ (negedge IOW)
	begin		
		if(!SPM_MODE_ADR)
		begin
			OUT_SD[8] <= ~SD[0];
			OUT_SD[9] <= ~SD[1];
			OUT_SD[10] <= ~SD[2];
		end
		
		if(!PID_PR_ADR)
		begin
			IO_A[7:0] <= ~SD[7:0];
		end
		
		if(!PID_INR_ADR)
		begin
			IO_B[7:0] <= ~SD[7:0];
		end
	end 
	
	assign IOCS16 = 1'bz; //16bit数据?地址选ㄓ?
endmodule

