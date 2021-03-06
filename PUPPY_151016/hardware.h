/*
 * hardware.h hardware.c 包含各种硬件相关函数，包括 AD／DA 采集卡， DDS，数字 IO量的相关例程
 * Originated by 李渊 on 2007-08-23
 * Version 0.0.2
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.


  * Last Modified on 2008-02-19
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include<sys/io.h>
#include <string.h>
#include <pthread.h>

/*Optical couple 32 channels test*/
#define CPLD_SD_LOW	0x200
#define CPLD_SD_HIGH	0x201
#define CPLD_IN	0x202
#define CPLD_OUT	0x203

/*Optical couple AD669 test*/
#define	DA_X_SCAN_ADR	0x210
#define	DA_Y_SCAN_ADR	0x212
#define	DA_Z_ADR	0x214
#define	DA_SET_POINT_ADR 0x216
#define	X_GAIN_CS_ADR	0x218
#define	X_OFFSET_CS_ADR	0x219
#define	Y_GAIN_CS_ADR	0x21A
#define	Y_OFFSET_CS_ADR	0x21B

/*Optical couple AD7671 test*/
#define S_408_Adr 0x220
#define	U1234_WR_Adr 	0x221
#define	U1_RD_Adr 	0x222
#define	U2_RD_Adr 	0x224
#define	U3_RD_Adr 	0x226
#define	U4_RD_Adr 	0x228
#define U1_BUSY_Adr 0x22A
#define U2_BUSY_Adr 0x22B
#define U3_BUSY_Adr 0x22C
#define U4_BUSY_Adr 0x22D

/*Optical couple signal board test*/
#define S_SPM_MODE_ADR 0x240
#define HARD_P_ADR 0x241
#define HARD_I_ADR 0x242

#define	LASER_ADR				0x300
#define	Motor_ADR				0x301
#define	STEP_OFF_ADR		0x302
#define HV_ADR					0x303
#define DDS_RST_ADR			0x304
#define DDS_ADR					0x305
#define DDS_522_CS_ADR  0x306
#define DDS_522_DATA_ADR 0x307

#define	K_A_Adr 	0x310	//0x310
#define	AD_SAMPLE_Adr 0x311
#define	AD_BUSY_Adr	0x312
#define	AD_CAL_Adr 0x313
#define	AD7523_AMP_Adr 0x314
#define	Clk_Adr 0x315
#define	AD_RD_Adr 0x316

/*AD676新板双AD并行采集*/
#define	K_U2_Adr 	0x310	//0x310  与AD7671通用
#define	K_U3_Adr 	0x311	//0x311	 与AD7671通用
//#define	AD_SAMPLE_Adr 0x312
#define	AD_U2_RD_Adr 0x314
#define	AD_U3_RD_Adr 0x316

#define	DA_LOAD_ADR	0x328
#define	DA_RST_ADR		0x329


#define SPM_WORK_MODE 0x362
#define SPM_PID_MODE 0x363
#define HARD_PID_PR	0x364
#define HARD_PID_INR	0x365

#define BGD_MOTOR_X_EN		0x370
#define BGD_MOTOR_X_SIGN	0x371
#define BGD_MOTOR_X_MAG		0x372

#define BGD_MOTOR_Y_EN		0x373
#define BGD_MOTOR_Y_SIGN	0x374
#define BGD_MOTOR_Y_MAG		0x375

#define BGD_MOTOR_Z1_EN		0x366  //376 37F 地址不能使用
#define BGD_MOTOR_Z1_SIGN	0x377
#define BGD_MOTOR_Z1_MAG	0x378

#define BGD_MOTOR_Z2_EN		0x379
#define BGD_MOTOR_Z2_SIGN	0x37A
#define BGD_MOTOR_Z2_MAG	0x37B

#define BGD_MOTOR_Z3_EN		0x37C
#define BGD_MOTOR_Z3_SIGN	0x37D
#define BGD_MOTOR_Z3_MAG	0x37E
#define BGD_MOTOR_SPEED		0x367  //367

extern const unsigned char AD_LASER_SUM;
extern const unsigned char AD_OSAP;
extern const unsigned char AD_FICTION; //not true
extern const unsigned char AD_PHASE;
extern const unsigned char AD_ERROR;
extern const unsigned char AD_STM;
extern const unsigned char DA_X;
extern const unsigned char DA_Y;
extern const unsigned char DA_Z;

short read_ad();
unsigned short read_ad_times(int para);
unsigned short read_ad_times_byCHN(int para, int chn);
unsigned short *read_ad_times_4CHN(int para, int n);
unsigned short read_ad_times_byCHN0(int para);
short read_ad_by_channel(unsigned short para1,unsigned short para2);
void ad_channel_sel(unsigned short para);
void write_dac7744_by_channel(unsigned short para1,unsigned short para2);
void write_ad669_by_channel(unsigned short para1,unsigned short para2);

short read_AD( unsigned char channel );
short read_AD_TS( unsigned char channel );
short read_AD_N( unsigned char channel, int samplingNum );
short fast_AD();
short fast_AD_TS();
short fast_AD_N(int samplingNum);
void write_DA( unsigned char channel, unsigned short value );
void write_DA_TS( unsigned char channel, unsigned short value );

void read_ad676_chip1(short* val_1);
void read_ad676_chip2(short* val_2); 
void read_ad676_single_by_chn(short* val, char chn, char chip);
void read_ad_time_test();

void init_Hardware();

void IO_Out(unsigned char channel, unsigned char value);
void IO_Out8(unsigned char channel, unsigned char value);
void IO_Out16(unsigned short value);
void dds_Reset();
void dds_Out(double freq);
void dds_Stop();
void setWaveAmplitude(unsigned char amplitude);
void setPhase(unsigned short phase);
void setLedOn();
void setLedOff();
void setLaserOn();
void setLaserOff();
void setHighVoltageOn();
void setHighVoltageOff();
void motor_forward_one_step();
void motor_backward_one_step();
short motor_get_steps();
void motor_stop();
void setTubeSize(float paraX, float paraY, float paraZ);
void setIdentX(char para1,float para2);


inline static void udelay(int usecs)
{
	if(usecs < 0)	return;
	int i; for(i = 0; i < usecs; i++) outb(0, 0x80);
}

inline static void mdelay(int msecs)
{
	if(msecs < 0)	return;
	int i; for(i = 0; i < msecs; i++) udelay(1000);
}

inline static void sdelay(int ssecs)
{
	if(ssecs < 0)	return;
	int i; for(i = 0; i < ssecs; i++) mdelay(1000);
}

#define setBitToOne(Data, nBit)  ( Data |= (1 << nBit) )
#define setBitToZero(Data, nBit) ( Data &= ~(1 << nBit) )

#endif


