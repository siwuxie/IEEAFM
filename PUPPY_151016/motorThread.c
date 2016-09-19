#include <unistd.h>
#include "work_thread.h"
#include "hardware.h"
#include "afm_comm.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

//----�����������--
// ��Ҫ�ر���Ӧ��ÿ�ιر�ǰ�����ֵ�浽�ļ���
// ��ʱ���������û���õ�
int motorSteps; 

// �������ÿ������ʱ��400us - 1500us ֮��ʱ��תƽ��
const int stepMotorDelay = 2000;
const int delayInThread = 2000;
const int maxSteps = 40000; //���ν�����������

const char g_motorSteps[8] = {0x08, 0x09, 0x01, 0x05, 0x04, 0x06, 0x02, 0x0A};
static short g_steps = 0;

extern int g_step_resp_time;
extern unsigned short PID_setPoint;
extern char gc_tb_channel;

//�����õ�����
extern struct timespec ts_all, ts1_all, ts_da, ts1_da, ts_system, ts1_system;
extern int flag_all;
extern int flag_da;
extern int flag_system;

void motor_backward_one_step()
{
	int step;
	g_steps ++;
	step = g_steps + 30000;
	step %= 8;
	outb(g_motorSteps[step], Motor_ADR); 
}

void motor_forward_one_step()
{
	int step;
	g_steps --;
	step = g_steps + 30000;
	step %= 8;
	outb(g_motorSteps[step], Motor_ADR); 
}

void motor_stop()
{
	//outb(0x00, Motor_ADR); 
	
	/*add for BGD_AFM*/

			outb(0x0,BGD_MOTOR_X_EN);
			outb(0x0,BGD_MOTOR_Y_EN);
			outb(0x0,BGD_MOTOR_Z1_EN);
			outb(0x0,BGD_MOTOR_Z2_EN);
			outb(0x0,BGD_MOTOR_Z3_EN);
			
			DEBUG0("Stop!\n");
			/**/	
}

short motor_get_steps()
{
	return g_steps;
}

/**
*@brief  �����������
*@param  steps(int) ���˵Ĳ���
*@param  direction(int) ���� MOTOR_STEP_FORWARD / MOTOR_STEP_BACKWARD
*@return  void
*/
void motor_steps(int steps, int direction)
{
	int i;
	struct response resp;
	if(direction == MOTOR_STEP_FORWARD)
	{
		DEBUG0("Doing MOTOR_STEP_FORWARD");
		if(steps <= maxSteps)
		{
			for(i = 0; i < steps; i++)
			{
				motor_forward_one_step();
				udelay(stepMotorDelay);
			}
		}
		else
		{
			motor_forward_one_step();
			udelay(stepMotorDelay);
		}	
	}
	else
	{
		DEBUG0("Doing MOTOR_STEP_BACKWARD");
		if(steps <= maxSteps)
		{
			for(i = 0; i < steps; i++)
			{
				motor_backward_one_step();
				udelay(stepMotorDelay);
			}
		}
		else
		{
			motor_backward_one_step();
			udelay(stepMotorDelay);
		}
	}
	motor_stop();
	DEBUG0("Stopped auto!\n");
	resp.cmd = MOTOR_GET_STEPS;
	send(connect_socket_fd, (char*)&resp, 12, 0);
}

/**
*@brief  ����Զ������߳�
*@param  para(void*) 
*@return  void*
*/
void* motor_autoforward_thread(void* para)
{
	//int i;
	//int count = 0;
	int currentTask;
	struct response resp;
	//unsigned short forceBuf[506];
	//int bufNum = 500; //�����С
	//unsigned short errorBegin = 0, errorTmp = 0;
	unsigned short errorTmp = 0;
	int chn_a,chn_b,chn_c,chn_d,t_b;
	int A,B,C,D;
	
	//int iLeftData = 0;
		
	pthread_detach(pthread_self());//��Ϊ�����߳�
	
	//forceBuf[0] = MOTOR_AUTO_FORWARD_FINISH;
	//forceBuf[1] = 0;
	//forceBuf[2] = 0;
	//forceBuf[3] = 0;
	//forceBuf[4] = 0;
	//forceBuf[5] = bufNum;
	
	DEBUG0("motor auto forward thread!");
	
	/* cancelled in 20120508 for AD_IN signal detection
	if(gc_tb_channel == 0)	ad_channel_sel(0x2); //y signal
	else	ad_channel_sel(0x7);
*/


  //0xc���ĸ�ͨ����������
	ad_channel_sel(0xC); //for AD_IN detection
		
//	outb(CHN_Y,K_U2_Adr); //ad667
	mdelay(20);
	
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);

		if(currentTask == STOP) 
		{
			DEBUG0("motor auto forward stoped manually");
			goto GETOUT;
		}
			
		chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
	  chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
	  chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
	  chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);	
		//DEBUG1("chn_a is %d\n",chn_a);
		//DEBUG1("chn_b is %d\n",chn_b);
		//DEBUG1("chn_c is %d\n",chn_c);
		//DEBUG1("chn_d is %d\n",chn_d);

	  A = (chn_a - 32767) / 32767.0  * 10000; //����Ϊmv
	  B = (chn_b - 32767) / 32767.0  * 10000; 
	  C = (chn_c - 32767) / 32767.0  * 10000;
	  D = (chn_d - 32767) / 32767.0  * 10000;
	
	  errorTmp = (float)(A + B - C - D) / (A + B + C + D) * 1000;
		//errorTmp = read_ad_times(gi_ad_sample_times) + 32768; //select y	ͳһΪUSHORT
		
		if(gi_ad_sample_times <= 5) udelay(100);
		//read_ad676_chip1(&errorTmp);//ad676
		//mdelay(1);
		
		//forceBuf[6 + count % bufNum] = errorTmp;

		//if(count % bufNum == (bufNum - 1))
		//{
			//send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
			//DEBUG0("Send ok!\n");
		//}
		
		//count++;
	
//add by 20120508 		
	//	if((inb(STEP_OFF_ADR) & 0x1) == 0) 
	//DEBUG1("errortemp is %d\n",errorTmp);
	//DEBUG1("tempPoint is %d\n",PID_setPoint);
	if((errorTmp - PID_setPoint) >= 0)
		{
			//write_ad669_by_channel( 1, DA_Z_ADR );
			// ���͸���λ�����������
	    //resp.cmd = MOTOR_GET_STEPS;
		  resp.cmd = MOTOR_AUTO_FORWARD_FINISH;
      send(connect_socket_fd, (char*)&resp, sizeof(RESPONSE), 0);
	    goto GETOUT;

		}
	}

GETOUT:
	//motor_stop();	
	//mdelay(1);	
	
	//errorTmp = read_ad_times(gi_ad_sample_times) + 32768; //select y	ͳһΪUSHORT 
	//forceBuf[6 + count % bufNum] = errorTmp;
	
	//iLeftData = count % bufNum - 1;
	//iLeftData = iLeftData > 0 ? iLeftData : 0;
	
	//forceBuf[0] = MOTOR_AUTO_FORWARD_FINISH;
	//forceBuf[1] = 0;
	//forceBuf[2] = 1;
	//forceBuf[3] = 0;
	//forceBuf[4] = 1;
	//forceBuf[5] = iLeftData; //�������������
	//send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
	
	
  DEBUG1("send cmd 0x%x\n", resp.cmd);
	DEBUG0("Forward complete��\n");
	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG1("gb_feedback_on is %d\n",gb_feedback_on);

	//if(gb_feedback_on == 1)
	//{
		//g_current_task = CMD_FEEDBACK_SWITCH;
		//pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	//}
	
	DEBUG0("motor auto forward stoped auto");
	return (void*)0;
}







/**
*@brief  ����Զ����루�£�
*@param  para(void*) 
*@return  void*
*/
void* approachNewThread(void* para)
{
	DEBUG0("motor auto forward new style thread!");
	pthread_detach(pthread_self());//��Ϊ�����߳�
	
	struct response resp;
	int i = 0, count = 0;
	unsigned int  timeuse = 0;
	struct timeval tpstart,tpend;
	unsigned short forceBuf[506];
	int bufNum = 500; //�����С
	int times = 20; //����ƽ������
	short s_yTmp = 0;
	int iLeftData = 0;
	
	int iStepTime = ( (COMMAND *)para )->para1 * 1000; //us
	int iWaitTime = ( (COMMAND *)para )->para2; //ms
	int iExpandTime = ( (COMMAND *)para )->para3 * 1000;//us
	unsigned short usMinVol = ( (COMMAND *)para )->para4, usMaxVol = ( (COMMAND *)para )->para5;
	
	DEBUG1("iStepTime = %d us\n",iStepTime);
	DEBUG1("iWaitTime = %d ms\n",iWaitTime);
	DEBUG1("iExpandTime = %d ms\n",iExpandTime);
	DEBUG2("usMinVol = %d , usMaxVol = %d\n",usMinVol,usMaxVol);
	
	int steps = 0;
	steps = iExpandTime / g_step_resp_time;
	int stepValue = 0;
	stepValue = (usMaxVol - usMinVol) / steps;
	
	forceBuf[0] = CMD_FORCE_CURVE_START;
	forceBuf[1] = 0;
	forceBuf[2] = 0;
	forceBuf[3] = 0;
	forceBuf[4] = 0;
	forceBuf[5] = bufNum;
	
		/* cancelled in 20120508 for AD_IN signal detection
	if(gc_tb_channel == 0)	ad_channel_sel(0x2); //y signal
	else	ad_channel_sel(0x7);
*/

	ad_channel_sel(0xC); //for AD_IN detection
		
//	outb(CHN_Y,K_U2_Adr); //ad676
	mdelay(20);
	
	outb(0x0,BGD_MOTOR_Z1_SIGN);			
	outb(0x0,BGD_MOTOR_Z2_SIGN);			
	outb(0x0,BGD_MOTOR_Z3_SIGN);

	while(1)
	{
		timeuse = 0;
		if(g_current_task == STOP) 
		{
			DEBUG0("motor auto forward stoped manually");
			goto GETOUT;
		}
		
		write_ad669_by_channel( usMinVol, DA_Z_ADR ); 
		udelay(g_step_resp_time);
		
//add by 20120508 		
	//	if((inb(STEP_OFF_ADR) & 0x1) == 0)  goto GETOUT;
	if((read_ad_times(gi_ad_sample_times) + 32768 - PID_setPoint) <= 0) goto GETOUT;
			
		outb(0x1,BGD_MOTOR_Z1_EN);
		outb(0x1,BGD_MOTOR_Z2_EN);
		outb(0x1,BGD_MOTOR_Z3_EN);
		
		gettimeofday(&tpstart,NULL); //��ȡ��ʼʱ��
		while(timeuse < iStepTime) 
		{
			forceBuf[6 + count % bufNum] = read_ad_times(gi_ad_sample_times) + 32768; //select error	ͳһΪUSHORT
			mdelay(1);
			
			//read_ad676_single_by_chn(&s_yTmp,CHN_Y,1); //ad676
			//read_ad676_chip1(&s_yTmp);//ad676			
			//forceBuf[6 + count % bufNum] = s_yTmp + 32768;
			
			gettimeofday(&tpend,NULL); //��ȡ��ֹʱ��
	 		timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 						tpend.tv_usec-tpstart.tv_usec; //us Level
	 						
	 		if(count % bufNum == (bufNum - 1))		send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
	 		count++;
		}
		
		outb(0x0,BGD_MOTOR_Z1_EN);
		outb(0x0,BGD_MOTOR_Z2_EN);
		outb(0x0,BGD_MOTOR_Z3_EN);
		mdelay(iWaitTime);		
		
		for(i = 1; i <= steps; i++)
		{
			write_ad669_by_channel( usMinVol + i*stepValue , DA_Z_ADR ); //0~200V�մɹ� 200V~���
			udelay(g_step_resp_time);
		
			//if((inb(STEP_OFF_ADR) & 0x1) == 0) goto GETOUT;
				
			forceBuf[6 + count % bufNum] = read_ad_times(gi_ad_sample_times) + 32768; //select error	ͳһΪUSHORT
			if((forceBuf[6 + count % bufNum] - PID_setPoint) < 0) goto GETOUT;
			mdelay(1);
			
			//read_ad676_single_by_chn(&s_yTmp,CHN_Y,1); //ad676
			//read_ad676_chip1(&s_yTmp);//ad676			
			//forceBuf[6 + count % bufNum] = s_yTmp + 32768;
			
			if(count % bufNum == (bufNum - 1))		send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
	 		count++;
		}
	}

GETOUT:
	motor_stop();	
	write_ad669_by_channel( 0, DA_Z_ADR ); 
/*	
	resp.cmd = CMD_APPROACH_NEW_UP;
	send(connect_socket_fd, (char*)&resp, 12, 0);
*/	

	iLeftData = count % bufNum - 2;
	iLeftData = iLeftData > 0 ? iLeftData : 0;
		
	forceBuf[0] = CMD_FORCE_CURVE_START;
	forceBuf[1] = 0;
	forceBuf[2] = 1;
	forceBuf[3] = 0;
	forceBuf[4] = 1;
	forceBuf[5] = iLeftData; //�������������
	send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);

	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	
	DEBUG0("motor auto forward stoped auto");
	
	if(gb_feedback_on == 1)
	{
		g_current_task = CMD_FEEDBACK_SWITCH;
		pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	}
	
	return (void*)0;
}

/**
*@brief  ����Զ������߳�
*@param  para(void*) ���벽��
*@return  void*
*/
void* motor_autobackward_thread(void* para)
{
	int i;
	int count = 0;
	int currentTask;
	struct response resp;
	unsigned short forceBuf[506];
	int bufNum = 500; //�����С
	unsigned short errorBegin = 0, errorTmp = 0;
	int iLeftData = 0;
	
	pthread_detach(pthread_self());//��Ϊ�����߳�
	
	forceBuf[0] = CMD_FORCE_CURVE_START;
	forceBuf[1] = 0;
	forceBuf[2] = 0;
	forceBuf[3] = 0;
	forceBuf[4] = 0;
	forceBuf[5] = bufNum;
	
	DEBUG0("motor auto backward thread!");
	
	if(gc_tb_channel == 0)	ad_channel_sel(0x2); //y signal
	else	ad_channel_sel(0x7);
		
//	outb(CHN_Y,K_U2_Adr); //ad667
	mdelay(20);
	
	while(1)
	{
		if(g_current_task == STOP) 
		{
			DEBUG0("motor auto backward stoped manually");
			goto GETOUT;
		}
			
		errorTmp = read_ad_times(gi_ad_sample_times) + 32768; //select y	ͳһΪUSHORT
		
		if(gi_ad_sample_times <= 5) udelay(100);
		//read_ad676_chip1(&errorTmp);//ad676
		//mdelay(1);
		
		forceBuf[6 + count % bufNum] = errorTmp;

		if(count % bufNum == (bufNum - 1))
		{
			send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
			DEBUG0("f!\n");
		}
		
		count++;
	}

GETOUT:
	motor_stop();	
	mdelay(1);	
	
	errorTmp = read_ad_times(gi_ad_sample_times) + 32768; //select y	ͳһΪUSHORT 
	forceBuf[6 + count % bufNum] = errorTmp;
	
	iLeftData = count % bufNum - 1;
	iLeftData = iLeftData > 0 ? iLeftData : 0;
	
	forceBuf[0] = CMD_FORCE_CURVE_START;
	forceBuf[1] = 0;
	forceBuf[2] = 1;
	forceBuf[3] = 0;
	forceBuf[4] = 1;
	forceBuf[5] = iLeftData; //�������������
	send(connect_socket_fd, (char*)forceBuf, (12 + bufNum * 2), 0);
	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	
	if(gb_feedback_on == 1)
	{
		g_current_task = CMD_FEEDBACK_SWITCH;
		pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	}
	
	DEBUG0("motor auto backward stoped auto");
	DEBUG1("Left data is %d\n",iLeftData);
	return (void*)0;
}
/**
*@brief  ������ٽ����߳�
*@param  para(void*) û���õ�
*@return  void*
*/
void* motor_fast_forward_thread(void* para)
{
	int currentTask;
	pthread_detach(pthread_self());
	DEBUG0("motor fast forward..");
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		motor_forward_one_step();
		usleep(delayInThread);
	}
	
	motor_stop();
	pthread_mutex_unlock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("motor fast forward stoped");
	return (void*)0;
}
/**
*@brief  ������������߳�
*@param  para(void*) û���õ�
*@return  void*
*/
void* motor_fast_backward_thread(void* para)
{
	int currentTask;
	pthread_detach(pthread_self());
	DEBUG0("motor fast backward..");
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		motor_backward_one_step();
		usleep(delayInThread);
	}
	motor_stop();
	pthread_mutex_unlock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("motor fast backward stoped");
	return (void*)0;
}
//���¾��ǲ����õ�һЩ����
//�����Ըı�setpoint��ֵ
void* change_setpoint(void* para)
{
		DEBUG0("change setpoint begin");
		int before,after;
		//before = (500 + 32767) / 32767.0  * 100;
		//after = (500 - 32767) / 32767.0  * 100;
		before = 500;
		after = -500;
		while(1){
			mdelay(100);
		  setPIDSetPoint(before);
		  //DEBUG1("setpoint before is %d\n",PID_setPoint);
		  mdelay(100);
		  setPIDSetPoint(after);
		  //DEBUG1("setpoint after is %d\n",PID_setPoint);
		}
	
}

void* test_all(void* para)
{
	DEBUG0("teat all");
	
	flag_all = 1;
	while(flag_all){
		write_ad669_by_channel(32767,DA_X_SCAN_ADR );//����X��
		//if(nanosleep(&ts_all, &ts1_all) != 0)  DEBUG0("error");
		//write_ad669_by_channel(32767,DA_Y_SCAN_ADR );//����Y��
		//if(nanosleep(&ts_all, &ts1_all) != 0)  DEBUG0("error");
	}
}
void* test_ad(unsigned int times)
{
	DEBUG0("teat ad");
	gi_ad_sample_times = times * 1000;
	//read_ad_times_byCHN(gi_ad_sample_times,0);
  read_ad_times_byCHN0(gi_ad_sample_times);
}

void* test_da(void* para)
{
	DEBUG0("teat da");
	
	flag_da = 1;
	while(flag_da){
		//test_saw();
		test_square();
		
		//write_ad669_by_channel(32767, DA_X_SCAN_ADR);//��X��Ϊ�����в���
		//if(nanosleep(&ts_da, &ts1_da) != 0)  DEBUG0("error");
    //write_ad669_by_channel(32849, DA_X_SCAN_ADR);//0.5V,163Ϊ1V
    //if(nanosleep(&ts_da, &ts1_da) != 0)  DEBUG0("error");
	}
}

void* test_system(void* para)
{
	DEBUG0("teat system");
	
	flag_system = 1;
	while(flag_system){
		write_ad669_by_channel(32767,DA_Z_ADR );//��Z��Ϊ�����в���
		if(nanosleep(&ts_system, &ts1_system) != 0)  DEBUG0("error");
    write_ad669_by_channel(32849,DA_Z_ADR );//0.5V
    if(nanosleep(&ts_system, &ts1_system) != 0)  DEBUG0("error");
	}
}

void set_testDA_freq(unsigned int freq)
{
	ts_da.tv_sec = 0;
	ts_da.tv_nsec = 500000/freq; 
}

void set_testALL_freq(unsigned int freq)
{
	ts_all.tv_sec = 0;
	ts_all.tv_nsec = 500000/freq; 
}

void set_testSYSTEM_freq(unsigned int freq)
{
	ts_system.tv_sec = 0;
	ts_system.tv_nsec = 500000/freq; 
}

void* test_all_stop(void* para)
{
	flag_all = 0;
	DEBUG1("flag_all is %d\n", flag_all);
}

void* test_da_stop(void* para)
{
	flag_da = 0;
	DEBUG1("flag_da is %d\n", flag_da);
}

void* test_system_stop(void* para)
{
	flag_system = 0;
	DEBUG1("flag_system is %d\n", flag_system);
}

void test_saw(void)
{
	unsigned int i;
	//-50V~50V��ѹ���۲첨��
	for(i = 24576; i < 40958; i++)
	{
		write_ad669_by_channel(i, DA_X_SCAN_ADR);
		//DEBUG1("write i is %d\n", i);
	}
}

void test_square(void)
{
	write_ad669_by_channel(0, DA_X_SCAN_ADR);
	udelay(10);
	write_ad669_by_channel(65535, DA_X_SCAN_ADR);//10V
	udelay(10);
}


