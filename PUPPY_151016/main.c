 /*
 * PC104 控制板上控制程序
 * main.c 主程序，主要完成通讯任务
 * Originated by 陈代谢 on 2008年12月25日
 * Version 0.0.1
 * Copyright (C) 2005, by Chen Daixie <chendaixie@mail.iee.ac.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.

 * modified on 2008年12月25日 星期日 21时55分15秒
 */
#include <sys/types.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>

#include "afm_comm.h"
#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

void catcher_SIGPIPE();
void init_para(void);
void init_Hardware();
void TestForHardware(void);

int connect_socket_fd,listen_socket_fd;
pthread_t laserTid;
/*Global Member*/
unsigned int g_point_time = 1000; //行扫描时，每个扫描点的停留时间 us
int g_point_time_left = 0;
int scanSamplingTimes = 1; //AD 采样次数
int g_step_resp_time = 100; //压电陶瓷响应延迟时间 us
char gb_lineFit = 0;
char gb_feedback_on = 0;
int gi_feedback_cycle_times = 1;
unsigned short PID_setPoint = 32768;
int gi_ad_sample_time = 10;
int gi_ad_sample_times = 30;

char gc_step_signal = 0;
unsigned short gus_step_height = 0;
int gi_step_time = 0;

int gi_da_x = 0;
int gi_da_y = 0;
int gi_da_z = 0;

char gc_tb_channel = 0; //Show_Y or Show_Lockin

float linerate = 1.00;

//测试用的变量
struct timespec ts_all, ts1_all, ts_da, ts1_da, ts_system, ts1_system;
volatile int flag_all;
volatile int flag_da;
volatile int flag_system;

int main(int argc, char* argv[])
{
	int len;
	struct sockaddr_in server_addr;
	struct command new_cmd;
	short s_tmp;
	int i = 0;

	/*将当前程序的优先级提高到实时进程优先级*/
//	struct sched_param param; 
//	param.sched_priority = 10;
//	sched_setscheduler(0, SCHED_FIFO, &param);	

	static struct sigaction sig_act;
	//注册SIGPIPE 信号处理函数，
	//当一个进程试图向一个已经断开连接的套接字 write 或 send 时，会收到 SIGPIPE 信号
	sig_act.sa_handler = catcher_SIGPIPE;
	sigfillset( &(sig_act.sa_mask) );
	sigaction(SIGPIPE, &sig_act, NULL);

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bzero( &(server_addr.sin_zero), 8 );

//	TestForHardware();
	init_Hardware(); 
	init_para();//declared below

	DEBUG0("20081212");
	setHighVoltageOn();
	DEBUG0("HV is on");

	
	//建立一个监听 socket，监听端口 7000，等待上位机连接（Internet I\O操作）
	if((listen_socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1)
	{
		perror("socket call failed");
		exit(1);
	}
	DEBUG0("socket create success!");
	if((bind(listen_socket_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in))) == -1)
	{
		perror("bind call failed");
		exit(2);
	}
	DEBUG0("socket bind success");
	if((listen(listen_socket_fd, 1))== -1)
	{
		perror("listen call failed");
		exit(3);
	}
	DEBUG0("socket listening success");
	
  //天大项目，打开laser线程，不断往上位机传数
	//pthread_create(&laserTid, NULL, laserThread, NULL);

	while(1)
	{
		DEBUG0("Waiting for a new connection.");
		// 接受一个连接请求，与上位机建立连接
		if((connect_socket_fd = accept(listen_socket_fd, NULL, NULL)) == -1)
    {
			perror("accept call failed");
			exit(4);
    }
		DEBUG0("A new connection set up sucessful.");
		init_para();
		while( (len = read(connect_socket_fd, (char *)&new_cmd, 12)) > 0 )
		{
			//分析命令字
			//激活工作线程
			DEBUG1("receive cmd 0x%x\n", new_cmd.cmd);
			dispatch_cmd(&new_cmd);
		}
		close(connect_socket_fd);
		DEBUG0("Connection closed");
		g_current_task = STOP;
		// 连接中断，强制结束所有工作线程，防止对扫描头造成损坏
		// 做其他善后工作，包括释放各种相应资源
		// 准备等待下一次新的连接
	}
	
}

void catcher_SIGPIPE()
{
	// 连接意外中断
	// 做各种善后清理工作
	// 此函数被系统调用的机会非常的小
	close(connect_socket_fd);
//	exit(0);
}

void TestForHardware(void)
{
	unsigned short us_tmp = 0, us_tmp1 = 0, us_tmp2 = 0;
	int uiTmp1 = 0, uiTmp2 = 0;
	int i = 0;
	float fTmp = 0;
	/*Test signal board 2013-11-25*/	
/*while(1)
{
	setWorkMode(0);
	sdelay(1);
	setWorkMode(1);
	sdelay(1);
	setWorkMode(2);
	sdelay(1);
	setPIDPara(0,255,0,0);
	sdelay(1);
	setPIDPara(255,0,0,0);
	sdelay(1);
}
*/
/*
while(1)
{	
outw(0x0,0x204);
sdelay(1);
DEBUG0("-10V\n");
outw(0xFFFF,0x204);
sdelay(1);
DEBUG0("10V\n\n");
}
*/

/*Hardware test in 2013*/
/*AD7671 board on 2013.11*/

outb(0x0,S_408_Adr);

while(1)
{	
//	outb(i%8,S_408_Adr);
	outb(0x0,S_408_Adr);
//	DEBUG1("Channel_%d\n",i%8);
//	mdelay(200);
	outb(0,U1234_WR_Adr); //启动AD转换
	while(!(inb(U4_BUSY_Adr) & 0x1));
	us_tmp = inw(U1_RD_Adr) & 0xffff;
	us_tmp = inw(U1_RD_Adr) & 0xffff;
	fTmp = (float)(us_tmp - 32767)/32767*10;
	DEBUG1("\nU1_RD_Adr = %.3f\n",fTmp);
	
//	outb(0,U1234_WR_Adr); //启动AD转换
//	while(!(inb(U4_BUSY_Adr) & 0x1));
	us_tmp = inw(U2_RD_Adr) & 0xffff;
	us_tmp = inw(U2_RD_Adr) & 0xffff;
	fTmp = (float)(us_tmp - 32767)/32767*10;
	DEBUG1("U2_RD_Adr = %.3f\n",fTmp);
	
//	outb(0,U1234_WR_Adr); //启动AD转换
//	while(!(inb(U3_BUSY_Adr) & 0x1)); //等待AD转换忙
//	udelay(10);
	uiTmp1 = inw(U3_RD_Adr) & 0xffff;
	uiTmp1 = inw(U3_RD_Adr) & 0xffff;
	fTmp = (float)(uiTmp1 - 32767)/32767*10;
	DEBUG1("U3_RD_Adr = %.3f\n",fTmp);

//	outb(0,U1234_WR_Adr); //启动AD转换
//	while(!(inb(U4_BUSY_Adr) & 0x1)); //等待AD转换忙
//	udelay(10);
	uiTmp2 = inw(U4_RD_Adr) & 0xffff;
	uiTmp2 = inw(U4_RD_Adr) & 0xffff;
	fTmp = (float)(uiTmp2 - 32767)/32767*10;
	DEBUG1("U4_RD_Adr = %.3f\n\n",fTmp);

	i++;
//	mdelay(1000);
	mdelay(200);
//	outb(0xF,S_408_Adr);
//}


/*AD669 board on 2013.11*/

//while(1)
//{
//	outb(0xFF,X_GAIN_CS_ADR);
//	outb(0x7F,X_OFFSET_CS_ADR);
//	outb(0xFF,Y_GAIN_CS_ADR);
//	outb(0x7F,Y_OFFSET_CS_ADR);
for(i = 0; i < 5; i++)
{	
	us_tmp = (float)i/5*65535;
//	us_tmp = 40000;
	write_ad669_by_channel( us_tmp, DA_X_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Y_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Z_ADR );
	write_ad669_by_channel( us_tmp, DA_SET_POINT_ADR );
	DEBUG1("DA=%.3f\n",(float)(us_tmp-32767)/32767*10);
	sdelay(1);
}
/*	
	us_tmp = 32767;
	write_ad669_by_channel( us_tmp, DA_X_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Y_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Z_ADR );
	write_ad669_by_channel( us_tmp, DA_SET_POINT_ADR );
	DEBUG1("DA=%.3f\n",(float)(us_tmp-32767)/32767*10);	
	mdelay(1000);
	
//for(i = 0; i < 100; i++)
//{
	us_tmp = 65535;
	write_ad669_by_channel( us_tmp, DA_X_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Y_SCAN_ADR );
	write_ad669_by_channel( us_tmp, DA_Z_ADR );
	write_ad669_by_channel( us_tmp, DA_SET_POINT_ADR );
	DEBUG1("DA=%.3f\n\n",(float)(us_tmp-32767)/32767*10);
	mdelay(1000);
//}
*/
}	

/*Optical couple board on 2013.11*/
while(1)
{
	outb(~0xFF,CPLD_OUT);
	outb(~0xAF,CPLD_SD_LOW);
	outb(~0xFA,CPLD_SD_HIGH);
	mdelay(100);

	outb(~0x0,CPLD_OUT);
	outb(~0x0,CPLD_SD_LOW);
	outb(~0x0,CPLD_SD_HIGH);
	mdelay(100);
	
	DEBUG1("CPLD_IN = %x\n",(~inb(CPLD_IN)) & 0xFF);
}
}

void init_para(void)
{
	//scanRangeX, scanRangeY, scanOffsetX, scanOffsetY, scanAngle
	setWorkMode(1); //Afm contact mode
	setPidMode(1); //Soft pid mode
	setFeedBackMode(1); //选择1号数字PID算法 for XL
	
	setScanRange(65528,	65528, 0,	0, 0);
	setScanPixel(512, 512);
	setLineRate(1);
	//setLineRate(30);//天大测试

	setPIDPara(25, 25, 0, 0);
	setPIDParaOther(20, 10, 10, 20000);
	setPIDChannel(10);
	setPIDSetPoint(0);
	
	outb(0xFF,X_GAIN_CS_ADR);
	outb(0x7F,X_OFFSET_CS_ADR);
	outb(0xFF,Y_GAIN_CS_ADR);
	outb(0x7F,Y_OFFSET_CS_ADR);
	
	write_ad669_by_channel( 32767, DA_X_SCAN_ADR ); 
	write_ad669_by_channel( 32767, DA_Y_SCAN_ADR ); 
	g_DA_z = 32767;
	write_ad669_by_channel( g_DA_z, DA_Z_ADR ); //初始Z向电压为0V
	
	outb(0x32,BGD_MOTOR_SPEED); //50
	
	g_current_task = STOP;
	
	outb(0x0,S_408_Adr);
	read_ad_time_test();
}

void init_Hardware()
{
	iopl(3);	/*使能直接操作端口，重要*/
	/*initial dac7744*/
	outb(0x0,DA_RST_ADR);	//reset dac7744 to midscale
	
	/*Calibrate AD676*/
	outb(0,AD_SAMPLE_Adr);
	outb(0,AD_CAL_Adr);
	outb(1,AD_CAL_Adr);
	outb(0,AD_CAL_Adr);
	
	/*Reset DDS*/
	dds_Reset();
}


