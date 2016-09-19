/*
 * PC104 ���ư��Ͽ��Ƴ���
 * work_thread.h work_thread.c �����������߳�
 * Originated by Chendaixie on 2008��12��30��
 * Version 0.0.1
 * Copyright (C) 2008, by DaixieChen <chendaixie@mail.iee.ac.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
 */
/*
 * Last Modified on 2008.12.30 by DaixieChen
 */
#ifndef WORKTHREAD_H
#define WORKTHREAD_H

#include <pthread.h>
#include <sys/socket.h>
#include "afm_comm.h"
//#include "closeloop.h"

void readHugeData();

void dispatch_cmd(struct command *pCmd);
void setMode(struct command *pCmd);

void* feedbackServoThread(void* para);

/*in laserThread.c*/
void* laserThread(void* para);
void* y_state_thread(void* para);

void* get_error_thread(void* para);
void* pid_get_error_thread(void* para);

/*in motorThread.c*/
void motor_steps(int steps, int direction);
void* motor_autoforward_thread(void* para);
void* motor_autobackward_thread(void* para);
void* motor_fast_forward_thread(void* para);
void* motor_fast_backward_thread(void* para);
void* approachNewThread(void* para);
//������
void* change_setpoint(void* para);
void* test_all(void* para);//���������ٶ�
void* test_ad(unsigned int times);//����AD�ٶ�
void* test_da(void* para);//����DA�ٶ�
void* test_system(void* para);//����ϵͳ��Ӧʱ��
void set_testDA_freq(unsigned int freq);
void set_testALL_freq(unsigned int freq);
void set_testSYSTEM_freq(unsigned int freq);
void* test_all_stop(void* para);
void* test_da_stop(void* para);
void* test_system_stop(void* para);
void test_saw(void);//������ݲ�
void test_square(void);//��������

void setPIAuto(float para);//�Զ�����PI����
void setKp13Auto(float para);//�Զ�����KpRule����

/*in scanThread.c*/
void setLineRate(float para);
void setScanRange(int scanRangeX,	int scanRangeY, int scanOffsetX, int scanOffsetY, int scanAngle);
void setLinefitPara(int scanRangeX,	int scanRangeY);//����ɨ�跶Χ����linefit�Ĳ���
void setScanPixel(int XDots, int YDots);
void setScanPixelTd(unsigned short para);//���
void setScanSamplingTimes(int times);
void sendData(char para);
void* lineScanThread(void* para);
void* fastScanThread(void* para);
void* normalScanThread(void* para);

void* forceCurveThread(void* para);
void setForceCurveCycles(int para);

/* in FreqScanThread.h*/
void setFreqRange(unsigned int startFreq, unsigned int endFreq);
void setFreqPara(void* para);
void setFreqDriAmp(unsigned char para);
void* feedback_thread(void* para);// only for test purpose
void* freqScanThread(void* para);

/*Piezo line fit*/
static void updateHCCurve( int num );
static float InvSqrt(float x);
static float corr2 ( float y, float a );
static float corr3 ( float y, float a, float b );
void setHysteresisParaX(short a1, short b1, short a2, short b2);
void setHysteresisParaY(short a1, short b1, short a2, short b2);
void setDriveWave(char para);

//���������Ŀ����
void setTappingSingle(unsigned short para);
void setTdPidMode(unsigned short para);
void setImageScale(unsigned short para);
void setSlowAxis(unsigned short para);
void setSwitchError(unsigned short para);
void setSwitchFriction(unsigned short para);
void setSwitchDeflection(unsigned short para);
void setSwitchPhase(unsigned short para);
void setSwitchAmplitude(unsigned short para);
void setImageDirection(unsigned short para);
//���������Ŀ����

extern pthread_mutex_t g_current_task_mutex;
extern int g_current_task;
extern char g_whole_scanning;
extern int g_DA_z; //���� Z ��� DA ���ֵ
extern pthread_mutex_t g_PID_mutex;
extern int connect_socket_fd,listen_socket_fd; // ����λ��ͨѶ���� socket 
extern int scanSamplingTimes;

extern char gb_feedback_on;

extern int gi_ad_sample_time;
extern int gi_ad_sample_times;

extern char gc_step_signal;
extern unsigned short gus_step_height;
extern int gi_step_time;

extern pthread_t servoTid;

#endif
