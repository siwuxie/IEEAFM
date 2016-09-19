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
module CPLD_32Channel_forAD7671(IOW,IOR,SA,SD,IO_A,IO_B,OUT_SD,OUT_SD_S,IN_SD,IOCS16);
	
	input IOW;
   input IOR;

   input [11:0] SA;
	inout [15:0] SD;
	input [15:8] IN_SD;
	output [10:8] OUT_SD_S;	
	output [15:11] OUT_SD;
	input [7:0] IO_A;
	input [7:0] IO_B;	 //busy signal
	output IOCS16;
	
	wire S_408_Adr,U1234_WR_Adr;
	wire U1_RD_Adr,U2_RD_Adr,U3_RD_Adr,U4_RD_Adr;
	wire U1_BUSY_Adr,U2_BUSY_Adr,U3_BUSY_Adr,U4_BUSY_Adr;
	
	assign S_408_Adr = !(SA[11:0]==12'h220);
	assign U1234_WR_Adr = !(SA[11:0]==12'h221);  
	assign U1_RD_Adr = !(SA[11:0]==12'h222);
	assign U2_RD_Adr = !(SA[11:0]==12'h224);
	assign U3_RD_Adr = !(SA[11:0]==12'h226);
	assign U4_RD_Adr = !(SA[11:0]==12'h228);  
	assign U1_BUSY_Adr = !(SA[11:0]==12'h22A);
	assign U2_BUSY_Adr = !(SA[11:0]==12'h22B);
	assign U3_BUSY_Adr = !(SA[11:0]==12'h22C);
	assign U4_BUSY_Adr = !(SA[11:0]==12'h22D);

	reg [10:8] OUT_SD_S;
//	reg [15:11] OUT_SD;
	always @ (negedge IOW)		
	begin
			if(!S_408_Adr)
			begin
				OUT_SD_S[10:8] <= ~SD[3:0];
			end			
	end
/*
	always @ (negedge IOW)		
	begin
		if(!U1234_WR_Adr)
			begin
				OUT_SD[11] <= 1'b1;
			end
			else
			begin
				OUT_SD[11] <= 1'b0;
			end
	end
	
	always @ (negedge IOR)		
	begin
		if(!U1_RD_Adr)
			begin
				OUT_SD[12] <= 1'b1;
			end
			else
			begin
				OUT_SD[12] <= 1'b0;
			end
	end
	always @ (negedge IOR)		
	begin
		if(!U2_RD_Adr)
			begin
				OUT_SD[13] <= 1'b1;
			end
			else
			begin
				OUT_SD[13] <= 1'b0;
			end
	end
	always @ (negedge IOR)		
	begin
		if(!U3_RD_Adr)
			begin
				OUT_SD[14] <= 1'b1;
			end
			else
			begin
				OUT_SD[14] <= 1'b0;
			end
	end
	always @ (negedge IOR)		
	begin
		if(!U4_RD_Adr)
			begin
				OUT_SD[15] <= 1'b1; //此通道反转异常
			end
			else                                                              
			begin
				OUT_SD[15] <= 1'b0;
			end
	end
*/

	assign OUT_SD[11] = (IOW | U1234_WR_Adr)?1'b0:1'b1;
	assign OUT_SD[12] = (IOR | U1_RD_Adr)?1'b0:1'b1;
	assign OUT_SD[13] = (IOR | U2_RD_Adr)?1'b0:1'b1;
	assign OUT_SD[14] = (IOR | U3_RD_Adr)?1'b0:1'b1;
	assign OUT_SD[15] = (IOR | U4_RD_Adr)?1'b0:1'b1;
	
/*
	assign OUT_SD[11] = (IOW | U1234_WR_Adr);
	assign OUT_SD[12] = (U1_RD_Adr | IOR);
	assign OUT_SD[13] = (U2_RD_Adr | IOR);
	assign OUT_SD[14] = (U3_RD_Adr | IOR);
	assign OUT_SD[15] = (U4_RD_Adr | IOR);
*/	
	assign SD[0] = (U1_BUSY_Adr|IOR)?1'bz:~IO_B[0];
	assign SD[1] = (U2_BUSY_Adr|IOR)?1'bz:~IO_B[1];
	assign SD[2] = (U3_BUSY_Adr|IOR)?1'bz:~IO_B[2];
	assign SD[3] = (U4_BUSY_Adr|IOR)?1'bz:~IO_B[3];
	
	assign SD = ((U1_RD_Adr & U2_RD_Adr & U3_RD_Adr & U4_RD_Adr) | IOR) ? 16'bz : ~{IN_SD[15:8],IO_A[7:0]}; 
	
	wire DX_WR_Adr,DY_WR_Adr,DZ_WR_Adr,SET_WR_Adr,X_GAIN_WR_Adr,X_OFFSET_WR_Adr,Y_GAIN_WR_Adr,Y_OFFSET_WR_Adr,AD669_Trigger;
	assign DX_WR_Adr = !(SA[11:0]==12'h210);  //320
	assign DY_WR_Adr = !(SA[11:0]==12'h212);
	assign DZ_WR_Adr = !(SA[11:0]==12'h214);
	assign SET_WR_Adr = !(SA[11:0]==12'h216);
		
	assign IOCS16 = (U1_RD_Adr & U2_RD_Adr & U3_RD_Adr & U4_RD_Adr & DX_WR_Adr & DY_WR_Adr & DZ_WR_Adr & SET_WR_Adr) ? 1'b1 : 1'b0; //16bit数据?地址选?

endmodule

