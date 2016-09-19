#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <pthread.h>
#include <string.h>
#include "work_thread.h"
#include "hardware.h"

pthread_mutex_t g_current_task_mutex = PTHREAD_MUTEX_INITIALIZER;  //静态化初始互斥锁
int g_current_task;
char g_whole_scanning = 0;	//0~stop  1~run  2~pause
extern int g_step_resp_time;
extern char gb_lineFit;
extern int gi_feedback_cycle_times;
extern int gi_da_x, gi_da_y;
extern char gc_tb_channel;

pthread_mutex_t g_thread_mutex = PTHREAD_MUTEX_INITIALIZER;
// motor 相关线程的互斥
pthread_mutex_t g_motor_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_t scanTid, motorTid, laserTid, freqScanTid, servoTid;
pthread_t changeSetpoint, testDa, testAll, testSystem;//测试用到的线程；

void dispatch_cmd(struct command *pCmd)
{ 
	//分析命令字
	//激活工作线程
	int error;
	RESPONSE res;
	double dTemp;
	int nTemp;
	unsigned short uTemp;
	unsigned int unTemp1, unTemp2;
	unsigned char ucTmp1, ucTmp2, ucTmp3, ucTmp4;
	/* 将扫描线程的优先级提高为实时优先级 */
	pthread_attr_t scan_thread_attr;
	struct sched_param scan_thread_param;
	
	struct response resp;//测试响应时间用 
	
	pthread_attr_init(&scan_thread_attr);
	pthread_attr_setdetachstate(&scan_thread_attr, PTHREAD_CREATE_DETACHED);
	pthread_attr_setscope(&scan_thread_attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setinheritsched(&scan_thread_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_getschedparam(&scan_thread_attr, &scan_thread_param);
	scan_thread_param.sched_priority = 10;
	pthread_attr_setschedparam(&scan_thread_attr, &scan_thread_param);
	error = pthread_attr_setschedpolicy(&scan_thread_attr, SCHED_FIFO); //线程的调度策略，先入先出
	if(error)
	{
		fprintf(stderr, 
			"Failed to set the scheduling policy to SCHED_FIFO : %s\n", 
			strerror(error));
	}		
	switch(pCmd->cmd)
	{
		DEBUG1("dispatch cmd 0x%x\n", pCmd->cmd);
		DEBUG1("setpoint cmd 0x%x\n", SET_WORKING_POINT);
/*Laser*/
		case LASER_ON:
			setLaserOn();
			DEBUG0("LASER_ON");
			//outb(pCmd->para1 & 0xFF,LASER_ADR);
			break;
		case LASER_OFF:
			setLaserOff();
			DEBUG0("LASER_OFF");
			break;
			
		case GET_LASER_POS:
			DEBUG0("GET_LASER_POS");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = GET_LASER_POS;
			pthread_mutex_unlock(&g_current_task_mutex);
			DEBUG0("pthread_create(&laserTid, NULL, laser_thread, NULL);");
			
			mdelay(400);
			pthread_create(&laserTid, NULL, laserThread, NULL);

			break;
			
		case	CMD_SHOW_TB_CHANNEL:
			DEBUG0("CMD_SHOW_TB_CHANNEL");
			gc_tb_channel = pCmd->para1;
			break;	
			
		case CMD_SCAN_STOP:
			g_whole_scanning = 0;
			pthread_mutex_lock(&g_current_task_mutex);
			g_current_task = STOP;
			pthread_mutex_unlock(&g_current_task_mutex);
			DEBUG0("scan stop");
			break;
			
		case CMD_DRAW_Y_STATE:
			DEBUG0("CMD_DRAW_Y_STATE");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = CMD_DRAW_Y_STATE;
			pthread_mutex_unlock(&g_current_task_mutex);
			DEBUG0("pthread_create(&laserTid, NULL, y_state_thread, NULL);");
			
			mdelay(400);
			pthread_create(&laserTid, NULL, y_state_thread, NULL);

			break;
	
/*Step Motor*/		
		case MOTOR_STEP_FORWARD:
			DEBUG0("MOTOR_STEP_FORWARD");
			motor_steps(pCmd->para1, MOTOR_STEP_FORWARD);
			break;
		case MOTOR_STEP_BACKWARD:
			DEBUG0("MOTOR_STEP_BACKWARD");
			motor_steps(pCmd->para1, MOTOR_STEP_BACKWARD);
			break;
				
		case MOTOR_SPEED:
			outb(pCmd->para1,BGD_MOTOR_SPEED);
			DEBUG1("Set motor speed to %d\n",pCmd->para1);
			break;
		
		case MOTOR_AUTO_FORWARD:
			DEBUG0("MOTOR_AUTO_FORWARD");
			/*add for BGD_AFM*/		
			
			uTemp = pCmd->para2;
			if(uTemp & 0x1)
			{
				outb(0x1,BGD_MOTOR_X_EN);
				outb(0x0,BGD_MOTOR_X_SIGN);
				DEBUG0("Motor X run forward!\n");
			}
			if(uTemp & 0x2)
			{
				outb(0x1,BGD_MOTOR_Y_EN);
				outb(0x0,BGD_MOTOR_Y_SIGN);
				DEBUG0("Motor Y run forward!\n");
			}
			if(uTemp & 0x4)
			{
				outb(0x1,BGD_MOTOR_Z1_EN);
				outb(0x0,BGD_MOTOR_Z1_SIGN);
				DEBUG0("Motor Z1 run forward!\n");
			}
			if(uTemp & 0x8)
			{
				outb(0x1,BGD_MOTOR_Z2_EN);
				outb(0x0,BGD_MOTOR_Z2_SIGN);
				DEBUG0("Motor Z2 run forward!\n");
			}
			if(uTemp & 0x10)
			{
				outb(0x1,BGD_MOTOR_Z3_EN);
				outb(0x0,BGD_MOTOR_Z3_SIGN);
				DEBUG0("Motor Z3 run forward!\n");
			}
			/**/			
			if(pCmd->para1 == 0) 
			{
				DEBUG0("Stage_XY_FORWARD");
			}			
			else if(pCmd->para1 == 1)
			{
				DEBUG0("MOTOR_AUTO_FORWARD");
				pthread_mutex_lock(&g_current_task_mutex);

				g_current_task = MOTOR_AUTO_FORWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_autoforward_thread, NULL);");    
				
				mdelay(400); 
				pthread_create(&motorTid, NULL, motor_autoforward_thread, &(pCmd->para1));

			}
			else if(pCmd->para1 == 2)
			{
				DEBUG0("MOTOR_MANNUAL_FORWARD");
			}
			else if(pCmd->para1 == 3)
			{
				mdelay(pCmd->para3);
				motor_stop();
				DEBUG0("MOTOR_STEP_FORWARD");
			}
			
		break;
		case MOTOR_AUTO_FORWARD_NEW:
			
			//resp.cmd = MOTOR_AUTO_FORWARD_NEW;
      //send(connect_socket_fd, (char*)&resp, 12, 0);
      //DEBUG0("Response test\n");
			
			DEBUG0("JUDGE MOTOR STOP\n");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = MOTOR_AUTO_FORWARD;
			pthread_mutex_unlock(&g_current_task_mutex);
			DEBUG0("pthread_create(&motorTid, NULL, motor_autoforward_thread, NULL);");    
				
			mdelay(50); 
			pthread_create(&motorTid, NULL, motor_autoforward_thread, NULL);
			break;
		
		case CMD_APPROACH_NEW_UP:
			DEBUG0("JUDGE MOTOR STOP\n");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = CMD_APPROACH_NEW_UP;
			pthread_mutex_unlock(&g_current_task_mutex);  
			
			mdelay(400); 
			pthread_create(&motorTid, NULL, approachNewThread, pCmd);

			break;
		
		case MOTOR_AUTO_BACKWARD:
			DEBUG0("MOTOR_AUTO_BACKWARD\n");
			/*add for BGD_AFM*/
			uTemp = pCmd->para2;
			if(uTemp & 0x1)
			{
				outb(0x1,BGD_MOTOR_X_EN);
				outb(0x1,BGD_MOTOR_X_SIGN);
				DEBUG0("Motor X run backward!\n");
			}
			if(uTemp & 0x2)
			{
				outb(0x1,BGD_MOTOR_Y_EN);
				outb(0x1,BGD_MOTOR_Y_SIGN);
				DEBUG0("Motor Y run backward!\n");
			}
			if(uTemp & 0x4)
			{
				outb(0x1,BGD_MOTOR_Z1_EN);
				outb(0x1,BGD_MOTOR_Z1_SIGN);
				DEBUG0("Motor Z1 run backward!\n");
			}
			if(uTemp & 0x8)
			{
				outb(0x1,BGD_MOTOR_Z2_EN);
				outb(0x1,BGD_MOTOR_Z2_SIGN);
				DEBUG0("Motor Z2 run backward!\n");
			}
			if(uTemp & 0x10)
			{
				outb(0x1,BGD_MOTOR_Z3_EN);
				outb(0x1,BGD_MOTOR_Z3_SIGN);
				DEBUG0("Motor Z3 run backward!\n");
			}
			
			if(pCmd->para1 == 0) 
			{
				DEBUG0("Stage_XY_BACKWARD");
			}			
			else if(pCmd->para1 == 1)
			{
				DEBUG0("MOTOR_AUTO_BACKWARD");
				pthread_mutex_lock(&g_current_task_mutex);

				g_current_task = MOTOR_AUTO_BACKWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_autobackward_thread, NULL);");    
				
				mdelay(400); 
				pthread_create(&motorTid, NULL, motor_autobackward_thread, NULL);
			}
			else if(pCmd->para1 == 2)
			{
				DEBUG0("MOTOR_MANNUAL_BACKWARD");
			}
			else if(pCmd->para1 == 3)
			{
				mdelay(pCmd->para3);
				motor_stop();
				DEBUG0("MOTOR_STEP_BACKWARD");
			}
			/**/		
		break;
		
		case MOTOR_STOP:		
			DEBUG0("MOTOR_STOP\n");
			/*add for BGD_AFM*/
			outb(0x0,BGD_MOTOR_X_EN);
			outb(0x0,BGD_MOTOR_Y_EN);
			outb(0x0,BGD_MOTOR_Z1_EN);
			outb(0x0,BGD_MOTOR_Z2_EN);
			outb(0x0,BGD_MOTOR_Z3_EN);
			
			g_current_task = STOP;   //使得电机在运行的时候可以进行其他操作
			/**/	
			DEBUG0("MOTOR_STOP");
			short s_tmp;
			s_tmp = inb(STEP_OFF_ADR) & 0x1;
			DEBUG1("STEP_OFF state is %d\n",s_tmp);
			break;
			
		case MOTOR_GET_STEPS:
			DEBUG0("MOTOR_GET_STEPS");
			res.cmd = MOTOR_GET_STEPS;
			res.para1 = motor_get_steps();
			DEBUG1("Motor current step is %d", (int) res.para1);
			break;
		
		case MOTOR_FAST_FORWARD:
			DEBUG0("MOTOR_FAST_FORWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_FAST_FORWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);");
				pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);
			};
			break;
		case MOTOR_FAST_BACKWARD:
			DEBUG0("MOTOR_FAST_BACKWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_FAST_BACKWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);");
				pthread_create(&motorTid, NULL, motor_fast_backward_thread, NULL);
			};			
			break;
			
/*Set work mode*/
		case SET_WORK_MODE:
			DEBUG0("SET_WORK_MODE");
			setWorkMode(pCmd->para1);
			break;
			
		case SET_PID_MODE:
			DEBUG0("SET_PID_MODE");
			setPidMode(pCmd->para1);
			break;			
			
/*Parameter setting*/
		case SET_HV_ON:
			setHighVoltageOn();
			DEBUG0("HV is on");
			break;
		case SET_HV_OFF:
			setHighVoltageOff();
			DEBUG0("HV is off");
			break;
			
		case SET_SCAN_RANGE:
			DEBUG0("SET_SCAN_RANGE");
			setLinefitPara(pCmd->para1, pCmd->para1);
			pthread_mutex_lock(&g_current_task_mutex);
			setScanRange(pCmd->para1, pCmd->para1, pCmd->para3, pCmd->para4, pCmd->para5);
			pthread_mutex_unlock(&g_current_task_mutex);
			break;
		case SET_SCAN_PIXEL_OLD:
			DEBUG0("SET_SCAN_PIXEL");
			setScanPixel(pCmd->para1, pCmd->para2);
			setScanSamplingTimes(pCmd->para3);
			break;	
		case CMD_TUBE_SIZE:
			DEBUG0("CMD_TUBE_SIZE");
			setTubeSize(pCmd->para1, pCmd->para2,pCmd->para3);
			break;
			
		case SET_FEEDBACK_MODE:
			DEBUG0("SET_FEEDBACK_MODE");
			setFeedBackMode(pCmd->para1);
			break;
			
		case SET_PID_PARA:
			DEBUG0("SET_PID_PARA");
			pthread_mutex_lock(&g_PID_mutex);
			setPIDPara(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);
		//	DEBUG0("PID parameters have been changed");
			break;
		case SET_PID_PARA_SLIDER:
			DEBUG0("SET_PID_PARA_SLIDER");
			pthread_mutex_lock(&g_PID_mutex);
	//		setPIDParaSlider(pCmd->para1, pCmd->para2);
			setLineRate(pCmd->para3);
			pthread_mutex_unlock(&g_PID_mutex);
	//		DEBUG0("PID parameters have been changed");
			break;
			
		case SET_PID_OTHER:
			pthread_mutex_lock(&g_PID_mutex);
			setPIDParaOther(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);
			DEBUG0("PID parameters have been changed");
			break;
			
		case SET_WORKING_POINT:
			DEBUG0("receive setpoint");
			setPIDSetPoint(pCmd->para1);
			break;
			
		case SET_LINE_RATE:
			DEBUG0("SET_LINE_RATE");
			gi_ad_sample_times = 30 / (pCmd->para1);
			DEBUG1("gi_ad_sample_times = %d\n",gi_ad_sample_times);
			setLineRate(pCmd->para1);
			break;
			
		case CMD_DRIVE_WAVE_SEL:
			DEBUG0("CMD_DRIVE_WAVE_SEL");
			setDriveWave(pCmd->para1);
			break;
			
		case CMD_PIEZO_LINER_PARA:
			setHysteresisParaX(pCmd->para1,pCmd->para2,pCmd->para1,pCmd->para2);
			setHysteresisParaY(pCmd->para3,pCmd->para4,pCmd->para3,pCmd->para4);
			DEBUG2("X ParaA = %d, ParaB = %d\n",pCmd->para1,pCmd->para2);
			DEBUG2("Y ParaA = %d, ParaB = %d\n",pCmd->para3,pCmd->para4);
			break;
			
		case CMD_SET_Z_VOL:
			g_DA_z = pCmd->para1;
			write_ad669_by_channel( g_DA_z , DA_Z_ADR); //初始Z向电压为0V
			DEBUG1("Z is set to %d\n",g_DA_z);
			break;
			
		case CMD_SET_PIEZO_DELAY:
			DEBUG0("CMD_SET_PIEZO_DELAY");
			g_step_resp_time = pCmd->para1;
			break;
			
		case CMD_LINE_FIT:
			DEBUG0("CMD_LINE_FIT");
			gb_lineFit = pCmd->para1;
			break;
			
		case CMD_FEEDBACK_SWITCH:
			DEBUG0("CMD_FEEDBACK_SWITCH");
			gb_feedback_on = pCmd->para1;
			gi_feedback_cycle_times = pCmd->para2;		
			
			if(gb_feedback_on == 1 && (g_current_task == STOP || g_current_task == CMD_FEEDBACK_SWITCH))
			{
				pthread_mutex_lock(&g_current_task_mutex);
				g_current_task = CMD_FEEDBACK_SWITCH;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("Create feedback thread!\n");
				pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
			}
			break;
			
		case SET_TB_DRAW_SPEED:
			DEBUG0("SET_TB_DRAW_SPEED");
			break;
			
		case CMD_AD_SAMPLE_TIME:
			gi_ad_sample_times = pCmd->para1 / gi_ad_sample_time;
			DEBUG1("gi_ad_sample_times = %d\n",gi_ad_sample_times);
			break;
			
		case CMD_STEP_SIGNAL:
			DEBUG0("CMD_STEP_SIGNAL");
			gc_step_signal = pCmd->para1;
			gus_step_height = pCmd->para2; //mv
			gi_step_time = pCmd->para3; //ms
			break;
		
		case CMD_CMD_STEP_PARA:
			DEBUG0("CMD_CMD_STEP_PARA");
			gus_step_height = pCmd->para1;//mv
			gi_step_time = pCmd->para2;//ms
			break;
			
		case CMD_XY_CTR:
			DEBUG0("CMD_XY_CTR");
			gi_da_x = pCmd->para1;
			gi_da_y = pCmd->para2;
			
			write_ad669_by_channel(pCmd->para1,DA_X_SCAN_ADR);
			write_ad669_by_channel(pCmd->para2,DA_Y_SCAN_ADR);
			DEBUG2("X = %d, Y = %d\n",pCmd->para1,pCmd->para2);
			break;
			
/*Force curve*/
		case CMD_FORCE_CURVE_NEW_EXP:
			DEBUG0("CMD_FORCE_CURVE_NEW_EXP");
			if(pCmd->para1 == 1)	nTemp = pCmd->para2;
			else nTemp = pCmd->para2 - 65535;
				
			setForceCurveCompPara(nTemp,pCmd->para3);
			setForceCurveCycles(pCmd->para4);
			break;
			
		case CMD_FORCE_CURVE_NEW_START:
			DEBUG0("CMD_FORCE_CURVE_NEW_START");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = CMD_FORCE_CURVE_NEW_START;
			pthread_mutex_unlock(&g_current_task_mutex);
			
			mdelay(500);
			pthread_create(&scanTid, NULL, forceCurveThread, pCmd);

			break;
				
		case CMD_FORCE_CURVE_NEW_STOP:
			DEBUG0("CMD_FORCE_CURVE_NEW_STOP");
			g_current_task = STOP;
			break;
			
		case CMD_IDENT_X:
			DEBUG0("CMD_IDENT_X");
			setIdentX(pCmd->para1,(float)pCmd->para2 / 1000);
			break;		

/*Line scan*/
		case CMD_LINESCAN_START:
			DEBUG0("CMD_LINESCAN_START");
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = CMD_LINESCAN_START;
			pthread_mutex_unlock(&g_current_task_mutex);
			
			mdelay(500);
			pthread_create(&scanTid, NULL, lineScanThread, pCmd);

			break;
			
/*Whole scan*/
		case CMD_SCAN_WHOLE:
			g_whole_scanning = 1;
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = NOMAL_SCAN;
			pthread_mutex_unlock(&g_current_task_mutex);
			
			mdelay(50);
			//pthread_create(&scanTid, &scan_thread_attr, normalScanThread, pCmd);
			pthread_create(&scanTid, NULL, normalScanThread, pCmd);

			break;

		case CMD_SCAN_PAUSE:
			DEBUG0("CMD_SCAN_PAUSE\n");
			g_whole_scanning = 2;
			break;

/*Frequence scan mode*/
		case SET_FREQ_RANGE:
			memcpy(&unTemp1, &(pCmd->para1), sizeof(int));
			memcpy(&unTemp2, &(pCmd->para3), sizeof(int)); 
			setFreqRange(unTemp1, unTemp2);
			DEBUG1("The start freq is %d\n",unTemp1);
			DEBUG1("The end freq is %d\n",unTemp2);
			break;
			
		case SET_FREQ_PARA:
			DEBUG0("SET_FREQ_PARA\n");
			setFreqPara(pCmd);
			break;
			
		case SET_WORKING_FREQ:
			memcpy(&nTemp, &(pCmd->para1), sizeof(int));
			dTemp = (double)((unsigned int) nTemp) / 100.0;
			dds_Out(dTemp);
			DEBUG1("Set freq is %.3f\n",dTemp);
			break;
			
		case CMD_FREQ_STOP:
			DEBUG0("CMD_FREQ_STOP\n");
			dds_Reset();
			break;
			
		case CMD_FREQ_SCAN:
			DEBUG0("CMD_FREQ_SCAN\n");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				fprintf(stderr, "ERROR: another scan thread is working now\n");
			}
			else
			{
				g_current_task = CMD_FREQ_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("create freq scan thread");
				pthread_create(&scanTid, NULL, freqScanThread, pCmd);
			};
			break;

/*Expert mode*/
		case EXPERT_MODE_DA:
			DEBUG0("EXPERT_MODE_DA\n");
			switch(pCmd->para2)
			{
				case 0:
					uTemp = DA_X_SCAN_ADR;
					break;
				case 1:
					uTemp = DA_Y_SCAN_ADR;
					break;
				case 2:
					uTemp = DA_Z_ADR;
					break;
				case 3:
					uTemp = DA_SET_POINT_ADR;
					break;
			}
			//write_dac7744_by_channel(pCmd->para1, uTemp);
			write_ad669_by_channel(pCmd->para1, uTemp);
			DEBUG1("data is %d\n",pCmd->para1);
			DEBUG1("channel is %x\n",uTemp);
			DEBUG0("Export mode!\n");
			break;
			
		case EXPERT_MODE_GAIN_OFFSET:
			DEBUG0("EXPERT_MODE_GAIN_OFFSET\n");
			ucTmp1 = pCmd->para1 & 0xFF;
			ucTmp2 = pCmd->para2 & 0xFF;
			ucTmp3 = pCmd->para3 & 0xFF;
			ucTmp4 = pCmd->para4 & 0xFF;
			
			outb(ucTmp1,X_GAIN_CS_ADR);
			outb(ucTmp2,X_OFFSET_CS_ADR);
			outb(ucTmp3,Y_GAIN_CS_ADR);
			outb(ucTmp4,Y_OFFSET_CS_ADR);
			DEBUG1("X_GAIN %x\n",ucTmp1);
			DEBUG1("X_OFFSET is %x\n",ucTmp2);
			DEBUG1("Y_GAIN %x\n",ucTmp3);
			DEBUG1("Y_OFFSET is %x\n",ucTmp4);
			break;
			
		case EXPERT_MODE_AD:
			res.cmd = pCmd->cmd;
			res.para1 = read_ad_by_channel(pCmd->para1,AD_SAMPLE_TIMES);
			send(connect_socket_fd, (char*)&res, sizeof(RESPONSE), 0);
			DEBUG1("ad = %d\n", res.para1);
			break;
			//其它 测试setpoint
		case TEST_SETPOINT:
			
			DEBUG0("CHANGE SETPOINT\n");
			//setPIDSetPoint(100);
	
			pthread_mutex_lock(&g_current_task_mutex);

			g_current_task = TEST_SETPOINT;
			pthread_mutex_unlock(&g_current_task_mutex);
				
			mdelay(50); 
			pthread_create(&changeSetpoint, NULL, change_setpoint, NULL);
			
			break;
		case ONE_PID:
			setFeedBackMode(1);
			break;
		case FIVE_PID:
			setFeedBackMode(5);
			break;
		case SIX_PID:
			setFeedBackMode(6);
			break;
		case TEST_ALL:
			DEBUG1("pCmd->para1 = %d\n", pCmd->para1);
			
			set_testALL_freq(pCmd->para1);
			
      pthread_mutex_lock(&g_current_task_mutex);
			g_current_task = TEST_ALL;
			pthread_mutex_unlock(&g_current_task_mutex);
			mdelay(50); 
			pthread_create(&testAll, NULL, test_all, NULL);
			break;
		case TEST_AD:
			DEBUG1("pCmd->para1 = %d\n", pCmd->para1);
			test_ad(pCmd->para1);
			break;
		case TEST_DA:
			DEBUG1("pCmd->para1 = %d\n", pCmd->para1);
			
			//set_testDA_freq(pCmd->para1);
			
      pthread_mutex_lock(&g_current_task_mutex);
			g_current_task = TEST_DA;
			pthread_mutex_unlock(&g_current_task_mutex);
			mdelay(10); 
			pthread_create(&testDa, NULL, test_da, NULL);
			break;
		case TEST_SYSTEM:
			DEBUG1("pCmd->para1 = %d\n", pCmd->para1);
			
			//set_testSYSTEM_freq(pCmd->para1);
			
      pthread_mutex_lock(&g_current_task_mutex);
			g_current_task = TEST_SYSTEM;
			pthread_mutex_unlock(&g_current_task_mutex);
			mdelay(50); 
			pthread_create(&testSystem, NULL, test_system, NULL);
			break;
		case TEST_ALL_STOP:
			test_all_stop(pCmd);
			break;
		case TEST_DA_STOP:
			test_da_stop(pCmd);
			break;
		case TEST_SYSTEM_STOP:
			break;
			
			
	  /*以下均为天津大学项目添加*/
	  //表1-1
	  case HEAD_ENERGY:
	  	DEBUG0("SYSTEM READY");
	  	break;
	  case SHAKING_PIEZO_ALL:
	  	DEBUG0("SHAKING_PIEZO_ALL");
	  	setFreqPara(pCmd);
	  	break;
	  case SHAKING_PIEZO_AMP:
	   	DEBUG0("SHAKING_PIEZO_AMP");
	  	setFreqDriAmp(pCmd->para1);
	  	break;
	  case SHAKING_PIEZO_FRE:
	  	DEBUG0("SHAKING_PIEZO_FRE");
	  	setFreqRange(pCmd->para1, pCmd->para2);
	   	break;
	  case TAPPING_FEEDBACK:
	  	DEBUG0("TAPPING_FEEDBACK");
	  	setTappingSingle(pCmd->para1);
	  	break;
	  case PID_MODE:	
	    DEBUG0("PID_MODE");	 
	  	setTdPidMode(pCmd->para1);
	  	break;
	  case IMAGE_SCALE:
	   	DEBUG0("IMAGE_SCALE");	
	  	setImageScale(pCmd->para1);
	  	break;
	  case SLOW_AXIS:
	  	DEBUG0("SLOW_SAIS");	
	  	setSlowAxis(pCmd->para1);
	  	break;
		case PID_P:
			//DEBUG0("SET_PID_P");
			pthread_mutex_lock(&g_PID_mutex);
			setPIDPara(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);
			break;
		case PID_I:
			//DEBUG0("SET_PID_I");
			pthread_mutex_lock(&g_PID_mutex);
			setPIDPara(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);
			break;
		case FORWARD_GAIN:
			DEBUG0("SET_FORWORAD_GAIN");
			break;
		case OFFSET_X:
			DEBUG0("OFFSET_X");
			pthread_mutex_lock(&g_current_task_mutex);
			setScanRange(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4, pCmd->para5);
			pthread_mutex_unlock(&g_current_task_mutex);
			break;
		case OFFSET_Y:
			DEBUG0("OFFSET_Y");
			pthread_mutex_lock(&g_current_task_mutex);
			setScanRange(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4, pCmd->para5);
			pthread_mutex_unlock(&g_current_task_mutex);
			break;
		case SET_SCAN_PIXEL:
			DEBUG0("SET_SCAN_PIXEL");
			setScanPixelTd(pCmd->para1);
			break;	
		case SWITCH_ERROR:
			DEBUG0("SWITCH_ERROR");	
			setSwitchError(pCmd->para1);
			break;
		case SWITCH_FRICTION:
			DEBUG0("SWITCH_FRICTION");	
			setSwitchFriction(pCmd->para1);
			break;
		case SWITCH_DEFLECTION:
			DEBUG0("SWITCH_DEFLECTION");	
			setSwitchDeflection(pCmd->para1);
			break;
		case SWITCH_PHASE:
			DEBUG0("SWITCH_PHASE");	
			setSwitchPhase(pCmd->para1);
			break;
		case SWITCH_AMPLITUDE:
		  DEBUG0("SWITCH_AMPLITUDE");	
		  setSwitchAmplitude(pCmd->para1);
		 break;		
		case IMAGE_DIRECTION:
	  	DEBUG0("IMAGE_DIRECTION");	
	  	setImageDirection(pCmd->para1);
	  	break;
	  //表2-2
	  case SCAN_LOCATION:
	  	DEBUG0("SCAN_LOCATION");	
	  	break;
	  case MOTOR_Z1:
	  	DEBUG0("MOTOR_Z1");	
	  	break;
	  //case LIGHT_SENSOR:
	  	//DEBUG0("LIGHT_SENSOR");
	  	//break;
	  case AUTO_SCAN_RESULT:
	  	DEBUG0("AUTO_SCAN_RESULT");
	  	break;
	  case SINGLE_LINE_RESULT:
	  	DEBUG0("SINGLE_LINE_RESULT");
	  	break;
	};
}