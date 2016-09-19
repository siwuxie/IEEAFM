/*
 * closeloop.h closeloop.c 包含各种软闭环算法
 * Originated by chendaixie on 2008年12月22日
 * Version 0.0.1
 * Copyright (C) 2007, by Chen Daixie <chendaixie@mail.iee.ac.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
*/

/*
 *  现在只包括最简单的 PID 算法，其余算法待补充
*/

#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <pthread.h>

short PID_function01(void);
short PID_function02(void);
unsigned short PID_function03(void);
short PID_function04(void);
short PID_function05(void);
short PID_function06(void);
//模糊PI
float fuzzy_kp13(float e, float ec);
float fuzzy_ki13(float e, float ec);
float fuzzy_kp07(float e, float ec);
float fuzzy_ki07(float e, float ec);				
float uf(float x,float a,float b,float c);
float cuf(float x,float a,float b,float c);
float ufl(float x,float a,float b);
float cufl(float x,float a,float b);
float ufr(float x,float a,float b);
float cufr(float x,float a,float b);
float fand(float a,float b);
float forr(float a,float b);

short PID_function01_debug(short new_setPoint, short record_dots);

void setWorkMode(unsigned short para);
void setPidMode(unsigned short para);
void setPIDPara(unsigned short Kp, unsigned short Ki, unsigned short Kd, unsigned short loopCircles);
void setPIDParaSlider(unsigned short Kp, unsigned short Ki);
void setPIDParaOther(short eps, short times, short delay, short errorThresh);
void setFeedBackMode(short mode);
void setPIDChannel(unsigned char channel);
void setPIDSetPoint(unsigned short para);
short getError(int n);

// 下面是软闭环控制所需的各全局变量
extern pthread_mutex_t g_PID_mutex;
extern int g_DA_z; //保存 Z 向的 DA 输出值
extern short (*pid_func)();
extern unsigned char g_pid_mode;

#endif

