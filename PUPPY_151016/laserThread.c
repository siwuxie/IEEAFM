#include <unistd.h>
#include <sys/time.h>

#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

extern int g_step_resp_time;
extern int gi_da_z;
extern char gc_tb_channel;

/*
unsigned short X_data_trace[512 * 512];
unsigned short X_data_retrace[512 * 512];
unsigned short Y_data_trace[512 * 512];
*/

void readHugeData(void)
{
	/*
	int len;
	DEBUG0("IN readHugeData");
	
	len = recv(connect_socket_fd, (char *)X_data_trace, 512 * 512 * 2, MSG_WAITALL);
	len = recv(connect_socket_fd, (char *)X_data_retrace, 512 * 512 * 2, MSG_WAITALL);
	len = recv(connect_socket_fd, (char *)Y_data_trace, 512 * 512 * 2, MSG_WAITALL);

	DEBUG0("ALL DATA RECEIVED");
	*/
}

unsigned short sendBuf[DOTS_NUM_PLUS_CMD];

void setLaserOn()
{
	outb(1,LASER_ADR);
}
void setLaserOff()
{
	outb(0,LASER_ADR);
}

void* laserThread(void* para)
{
	int i, laserA, laserB, laserC, laserD;
	int testA;
	int currentTask;
	short s_error = 0, s_y = 0;
	int tmp = 0;
	int switchTimeDelay = 40;
	pthread_detach(pthread_self());//成为自由线程
	
	bzero(sendBuf, 8 * sizeof(unsigned short));
	
	sendBuf[0] = GET_LASER_POS;
	//sendBuf[0] = SEND_LASER_POSITION;
	sendBuf[1] = 4;
	DEBUG0("in laser thread");
	
	while(1)
	{
		//与天大通讯时将下面代码注释掉
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP) break;
		//与天大通讯时将上面代码注释掉
		
		laserA = 0;
		laserB = 0;
		laserC = 0;
		laserD = 0;

		sendBuf[0] = GET_LASER_POS;
		//sendBuf[0] = SEND_LASER_POSITION;
		sendBuf[1] = 4;
		sendBuf[6] = read_ad_times_byCHN(gi_ad_sample_times,0);		
		sendBuf[7] = read_ad_times_byCHN(gi_ad_sample_times,1);	
		sendBuf[8] = read_ad_times_byCHN(gi_ad_sample_times,2);			
		sendBuf[9] = read_ad_times_byCHN(gi_ad_sample_times,3);	
		send(connect_socket_fd, (char*)sendBuf, 20, 0);	
		mdelay(200);	
		//DEBUG1("send cmd 0x%x\n", sendBuf[0]);
		//DEBUG2("The step_off signal is %d,%d\n",sendBuf[6],sendBuf[7]);
	}
	
	sendBuf[1] = 0; //finish
	send(connect_socket_fd, (char*)sendBuf, 20, 0);
 
	DEBUG0("leave laser thread");
			
	return 0;
}

void* y_state_thread(void* para)
{
	int i = 0;
	int num = 512;
	int switchTimeDelay = 20; //ms
	short sTmp = 0;
	
	struct timeval tpstart,tpend;
	unsigned int  timeuse = 0;	
	char c_dir = 0;
	unsigned short us_tmp = 0;
	 				
	pthread_detach(pthread_self());//成为自由线程
	
	bzero(sendBuf, 8 * sizeof(unsigned short));
	
	sendBuf[0] = CMD_DRAW_Y_STATE;
	sendBuf[1] = 512;
	
	/* cancelled in 20120508 for AD_IN signal detection
	if(gc_tb_channel == 0)	ad_channel_sel(0x2); //y signal
	else	ad_channel_sel(0x7);
*/
	gettimeofday(&tpstart,NULL); //获取起始时间
	
	gi_da_z = g_DA_z;
	
	while(1)
	{		
		if(gb_feedback_on == 1)
		{
			sendBuf[6 + i % num] = PID_function03();	
		}		
		else
		{		
			sendBuf[6 + i % num] = read_ad_times_byCHN(gi_ad_sample_times,0); i++;
			sendBuf[6 + i % num] = read_ad_times_byCHN(gi_ad_sample_times,1); i++;
			sendBuf[6 + i % num] = read_ad_times_byCHN(gi_ad_sample_times,2); i++;
			sendBuf[6 + i % num] = read_ad_times_byCHN(gi_ad_sample_times,3); i++;
		}
			
		if(gc_step_signal == 1)
		{
			gettimeofday(&tpend,NULL); //获取终止时间
			timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + (tpend.tv_usec-tpstart.tv_usec); //us Level
			timeuse = timeuse / 1000; //ms level
	 				
	 		if(timeuse >= gi_step_time)
	 		{	
	 			gi_da_z = c_dir > 0 ? (g_DA_z + gus_step_height) : g_DA_z;
	 			write_ad669_by_channel( gi_da_z , DA_Z_ADR );
	 			udelay(g_step_resp_time); //阶跃响应延迟
	 			gettimeofday(&tpstart,NULL); //获取起始时间
	 			c_dir = !c_dir;
	 		}
		}
		
		if(i % num == 0)
		{
			send(connect_socket_fd, (char*)sendBuf, 1036, 0);
			mdelay(10);
			DEBUG1("Send line = %d\n",i);
		}
			
		if(g_current_task == STOP) break;
	}
	
	sendBuf[1] = 0; //finish
	send(connect_socket_fd, (char*)sendBuf, 1036, 0);
	
	bzero(sendBuf, 520 * sizeof(unsigned short));
	DEBUG0("leave y_state_thread");
	return 0;
}
