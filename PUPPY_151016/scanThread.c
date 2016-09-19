#include <math.h>

#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

#define XY_DELAY 20

extern unsigned int g_point_time; //��ɨ��ʱ��ÿ��ɨ����ͣ��ʱ��
extern int g_point_time_left;
extern char gb_lineFit;
extern float linerate;

//----ɨ�����ݴ洢��---
static unsigned short topoBuf[DOTS_NUM_PLUS_CMD];
static unsigned short errorBuf[DOTS_NUM_PLUS_CMD];
static unsigned short topoBufInv[DOTS_NUM_PLUS_CMD];
static unsigned short errorBufInv[DOTS_NUM_PLUS_CMD];

static unsigned short phaseBuf[DOTS_NUM_PLUS_CMD];
static unsigned short phaseBufInv[DOTS_NUM_PLUS_CMD];
static unsigned short fictionBuf[DOTS_NUM_PLUS_CMD];
static unsigned short fictionBufInv[DOTS_NUM_PLUS_CMD];
static unsigned short dampBuf[DOTS_NUM_PLUS_CMD];
static unsigned short dampBufInv[DOTS_NUM_PLUS_CMD];

//unsigned short topoBuf_retrace[518];
//unsigned short errorBuf_retrace[518];
//unsigned short phaseBuf_retrace[518];

//----ɨ�����---
static int scanLineTime; //ɨ��һ�������ʱ�䣨��������ɨʱ�䣩 ��λ��us
static int scanRangeX, scanRangeY, scanOffsetX, scanOffsetY;
static int scanAngle = 0;
static int scanPixels = 512;

//----ͨ��ѡ��---
static int hasInv = 0;
static int hasError = 0, hasPhase = 0, hasOther = 0, hasFiction = 0, hasDamp = 0;
static int hasPsdA = 0, hasPsdB = 0, hasPsdC = 0, hasPsdD = 0;

//----ѹ���մɷ����Խ���---
float hctc_xA = 0.0;//��ɨx����У��ϵ�� A, -1 +1
float hctc_xB = 0.0;//��ɨx����У��ϵ�� B, -1 +1
float hcrc_xA = 0.0;//��ɨx����У��ϵ�� A, -1 +1
float hcrc_xB = 0.0;//��ɨx����У��ϵ�� B, -1 +1
float hctc_yA = 0.0;//��ɨy����У��ϵ�� A, -1 +1
float hctc_yB = 0.0;//��ɨy����У��ϵ�� B, -1 +1
float hcrc_yA = 0.0;//��ɨy����У��ϵ�� A, -1 +1
float hcrc_yB = 0.0;//��ɨy����У��ϵ�� B, -1 +1
float hctc_x[DOTS_NUM]; //hysteresis correction trace curve, x dircection
float hcrc_x[DOTS_NUM]; //hysteresis correction retrace curve, x dircection
float hctc_y[DOTS_NUM]; //hysteresis correction trace curve, y dircection
float hcrc_y[DOTS_NUM]; //hysteresis correction retrace curve, y dircection

char cDriveWaveSel = 0; //default triangle wave

extern unsigned short PID_setPoint;
extern char gc_tb_channel;

//----ɨ�����ߴ�(um)----
float tubeSizeX = 40, tubeSizeY = 40, tubeSizeZ = 10;

//----ѹ��X����----
char bIdentXCheck = 0;
float fIdentX = 0;
extern int gi_da_x, gi_da_y;
extern char gc_tb_channel;

//----�����߻س��Ż�----
int iForceCurveInvComp = 0;
int iForceCurveInvDelay = 0; //ms
int iForceCurveCycles = 0;

void setHighVoltageOn()
{
	outb(1,HV_ADR);
}

void setHighVoltageOff()
{
	outb(0,HV_ADR);
}

/**
*@brief  ���� ɨ����Ƶ��ʵ�ʵ�ɨ��Ƶ�ʣ�����������ʱ�䣩
*@param  
*@return  void
*/
void setLineRate(float para) //1Hz~10Hz
{
	linerate = para;
	DEBUG1("linerate is %.3f\n", linerate);
	int uTime;
	uTime = 1e10 / para; //50000000
	scanLineTime = uTime;	
	g_point_time = scanLineTime / scanPixels / 2; //����ÿ��ɨ����ͣ��ʱ��
	DEBUG2("scanRate is %.3f, g_point_time is %d\n", para, g_point_time);
	
	g_point_time_left = g_point_time;
	if(hasDamp)
		g_point_time_left = g_point_time_left - 10000;
	if(hasFiction) 
		g_point_time_left = g_point_time_left - 10000;
}

/**
*@brief  ���� ɨ�跶Χ�Ȳ���
*@param  rangeX(int) X ����ɨ�跶Χ
*@param  rangeY(int) Y ����ɨ�跶Χ
*@param  offsetX(int) X ����ƫ��
*@param  offsetY(int) Y ����ƫ��
*@param  angle(int) ��ת�Ƕ�
*@return  void
*/
void setScanRange(int rangeX, int rangeY, int offsetX, int offsetY, int angle)
{
	scanRangeX = rangeX;
	scanRangeY = rangeY;
	scanOffsetX = offsetX;
	scanOffsetY = offsetY;
	scanAngle = angle;
	DEBUG0("scanRanges have been changed");
	DEBUG1("scanRangeX is %d\n", scanRangeX);
	DEBUG1("scanRangeY is %d\n", scanRangeY);
	DEBUG1("scanOffsetX is %d\n", scanOffsetX);
	DEBUG1("scanOffsetY is %d\n", scanOffsetY);
	DEBUG1("scanAngle is %d\n", scanAngle);
}

/**
*@brief  ���� linefit����
*@param  rangeX(int) X ����ɨ�跶Χ
*@param  rangeY(int) Y ����ɨ�跶Χ
*@return  void
*/
//����ʵ����linefit��ParaA����Ϊ4���ɣ�ParaB�뷶Χ�����Թ�ϵ, ����Ϊy = 0.2x + 4
void setLinefitPara(int scanRangeX,	int scanRangeY)
{
	short y; 
	if(scanRangeX != scanRangeY) 
	{
		DEBUG0("setLinefitPara fail, please set scanRangeX = scanRangeY");
	}
	else
	{
		y = 0.2 * scanRangeX + 4;
		hctc_xA = 4 / 100.0f;
		hcrc_xA = 4 / 100.0f;
		hctc_yA = 4 / 100.0f;
		hcrc_yA = 4 / 100.0f;
    hctc_xB = y / 100.0f;
    hcrc_xB = y / 100.0f;
    hctc_yB = y / 100.0f;
    hcrc_yB = y / 100.0f;
    
    DEBUG2("X ParaA = %d, ParaB = %d\n", hctc_xA, hctc_xB);
    DEBUG2("Y ParaA = %d, ParaB = %d\n", hctc_yA, hctc_yB);
    updateHCCurve(scanPixels);
	}
	
}

/*
/**
*@brief  �Զ����� PI��������ӦPID_function01()
*@param  para(float) ɨ��Ƶ��
*@return  void
*/
/*
void setPIAuto(float para)
{
	// 4Hz���£�����P = 0.1��I = 0.1
	if(para < 4)
	{
		PID_Kp = 0.1;
		PID_Ki = 0.1;
	}
	else 
	{
		float fv = 0.025; //����ʵ��õ���ϵ��
	  PID_Kp = para * fv;//P = ɨ��Ƶ��*������ϵ��
	  PID_Ki = 0.1;
	}
}
*/

/**
*@brief  �Զ����� PI��������ӦPID_function05(),13��ģ��PID
*@param  para(float) ɨ��Ƶ��
*@return  void
*/
void setKp13Auto(float para)
{
}

/**
*@brief  ���� ɨ�����
*@param  XDots(int) X ����ɨ�����
*@param  YDots(int) Y ����ɨ�����
*@return  void
*/
void setScanPixel(int XDots, int YDots)
{
	scanPixels = (XDots > YDots) ? XDots : YDots;
	DEBUG1("scanPixels is %d\n", scanPixels);
}
void setScanPixelTd(unsigned short para)
{
	switch (para)
	{
	case 0:
		scanPixels = 128;
    DEBUG1("scanPixels is %d\n", scanPixels);
		break;
	case 1:
		scanPixels = 256;
    DEBUG1("scanPixels is %d\n", scanPixels);
		break;
	case 2:
		scanPixels = 512;
    DEBUG1("scanPixels is %d\n", scanPixels);
		break;
	case 3:
		scanPixels = 1024;
    DEBUG1("scanPixels is %d\n", scanPixels);
		break;
	}
}
/**
*@brief  ���� AD �������������������������е� AD��
*@param  times(int) AD ��������
*@return  void
*/
void setScanSamplingTimes(int times)
{
	scanSamplingTimes = times;
	DEBUG1("scanSamplingTimes is %d\n", scanSamplingTimes);
}

/**
*@brief  ��鿪����ͨ��
*@param  para(COMMAND *) ͨ��ѡ��
*@return  void*
*/
void updateChannels(COMMAND * para)
{
	unsigned short channels; //ͨ��
	hasInv = para->para3;
	if(hasInv) DEBUG0("hasInv");
	channels = para->para1;
	
	hasError = 0;
	hasPhase = 0;
	hasFiction = 0;
	hasDamp = 0;
	
	if( channels & 0x2 ) {hasError = 1;DEBUG0("hasError");}	
	if( channels & 0x4 ) {hasPhase= 1;DEBUG0("hasPhase");}
	if( channels & 0x8 ) {hasFiction = 1;DEBUG0("hasFriction");}
	if( channels & 0x10 ) {hasDamp = 1;DEBUG0("hasDamp");}
}

/**
*@brief  ��ʼ��ɨ�����ݴ洢��
*@param  cmd(unsigned short) ɨ������
*@param  size(unsigned short) �������ݴ�С���ֽڣ�
*@return  void
*/
void initScanBuf(unsigned short cmd, unsigned short size)
{
	topoBuf[0] = cmd;
	topoBuf[1] = size;
	topoBuf[2] = 0; //ͨ����
	DEBUG1("topbuf command 0x%x\n", topoBuf[0]);
	
	topoBufInv[0] = cmd;
	topoBufInv[1] = size;
	topoBufInv[2] = 0; //ͨ����
	DEBUG1("topbufInv command 0x%x\n", topoBufInv[0]);
	
	errorBuf[0] = cmd;
	errorBuf[1] = size;
	errorBuf[2] = 1;//ͨ����	
	DEBUG1("error command 0x%x\n", errorBuf[0]);
	
	errorBufInv[0] = cmd;
	errorBufInv[1] = size;
	errorBufInv[2] = 1;//ͨ����
	DEBUG1("errorInv command 0x%x\n", errorBufInv[0]);
	
	phaseBuf[0] = cmd;
	phaseBuf[1] = size;
	phaseBuf[2] = 2;//ͨ����
	
	phaseBufInv[0] = cmd;
	phaseBufInv[1] = size;
	phaseBufInv[2] = 2;//ͨ����
	
	fictionBuf[0] = cmd;
	fictionBuf[1] = size;
	fictionBuf[2] = 3;//ͨ����
	
	fictionBufInv[0] = cmd;
	fictionBufInv[1] = size;
	fictionBufInv[2] = 3;//ͨ����
	
	dampBuf[0] = cmd;
	dampBuf[1] = size;
	dampBuf[2] = 4;//ͨ����
	
	dampBufInv[0] = cmd;
	dampBufInv[1] = size;
	dampBufInv[2] = 4;//ͨ����
}

/**
*@brief  �����к�
*@param  line(short) �кţ��� 1 ��ʼ ��������ʾ��ɨ��
*@return  void*
*/
void updateLineNum(int line)
{
		topoBuf[3] = line;	
		topoBufInv[3] = line;		
		
		errorBuf[3] = line;			
		errorBufInv[3] = line;
		
		phaseBuf[3] = line;
		phaseBufInv[3] = line;
		
		fictionBuf[3] = line;		
		fictionBufInv[3] = line;
		
		dampBuf[3] = line;
		dampBufInv[3] = line;
}

void sendData(char para)
{
	if(para == 1) //��ɨ����
	{
		send(connect_socket_fd, (char*)topoBuf, 2 * (scanPixels + 6), 0);//DEBUG0("send topoBuf");
		
		if(hasError) 
			{
				send(connect_socket_fd, (char*)errorBuf, 2 * (scanPixels + 6), 0);
		//		DEBUG0("send errorBuf");
			}
		if(hasPhase) 
			{
				send(connect_socket_fd, (char*)phaseBuf, 2 * (scanPixels + 6), 0);
		//		DEBUG0("send phaseBuf");
			}
		
		if(hasFiction)
			{
				send(connect_socket_fd, (char*)fictionBuf, 2 * (scanPixels + 6), 0);
			}
			
		if(hasDamp)
			{
				send(connect_socket_fd, (char*)dampBuf, 2 * (scanPixels + 6), 0);
				//DEBUG0("send dampBuf");
			}
	}
	else //��ɨ����
	{
		send(connect_socket_fd, (char*)topoBufInv, 2 * (scanPixels + 6), 0);
		//DEBUG0("send topoBufInv");
		
		if(hasError) 
			{
				send(connect_socket_fd, (char*)errorBufInv, 2 * (scanPixels + 6), 0);
			//	DEBUG0("send errorBufInv");
			}
		if(hasPhase) 
			{
				send(connect_socket_fd, (char*)phaseBufInv, 2 * (scanPixels + 6), 0);
				//DEBUG0("send phaseBufInv");
			}
		if(hasFiction)
			{
				send(connect_socket_fd, (char*)fictionBufInv, 2 * (scanPixels + 6), 0);
			}
		if(hasDamp)
			{
				send(connect_socket_fd, (char*)dampBufInv, 2 * (scanPixels + 6), 0);
				//DEBUG0("send dampBufInv");
			}
	}	
}

/**
*@brief  ����ɨ���̣߳�δ��ɣ�
*@param  para(void*) ͨ��ѡ��
*@return  void*
*/
void* fastScanThread(void* para)
{
	pthread_detach(pthread_self());//��Ϊ�����߳�

	//��鿪����ͨ��
	updateChannels(para);
	//ͨ��������
	
	return (void*)0;
}

/**
*@brief  �������߳�
*@param  para(void*) ͨ��ѡ��
*@return  void*
*/
void* forceCurveThread(void* para)
{
	DEBUG0("Force curve thread start.");
	pthread_detach(pthread_self());//��Ϊ�����߳�
	
	int i = 0, j = 0, k = 0;
	unsigned short usBegin = 0, usEnd = 0;
	int iStep = 0;
	unsigned short usErrorTmp = 0;
	int iZTmp = 0, iXTmp = 0;
	int imDelay = 0;
	int times = 0;
	short s_yTmp = 0;
	char bInvShow = 0;
	float fX0 = 0, fX = 0, fXtmp = 0;
	int iTmp = 0;
	unsigned int  timeuse = 0;
	struct timeval tpstart,tpend;
	
	memset(errorBuf, 0 , sizeof(COMMAND));
	errorBuf[0] = CMD_FORCE_CURVE_NEW_START;
	errorBuf[5] = 256;	
	
	usBegin = ( (COMMAND *)para )->para1;
	iStep = ( (COMMAND *)para )->para2;
	usEnd = ( (COMMAND *)para )->para3;	
	imDelay = ( (COMMAND *)para )->para4;
	bInvShow = ( (COMMAND *)para )->para5;
	
	iStep = iStep > 0 ? iStep : 1;
	
	if(usBegin > usEnd) iStep = -1 * iStep;
	
	times = (usEnd - usBegin) / iStep;
	
	DEBUG1("usBegin = %d\n",usBegin);
	DEBUG1("iStep = %d\n",iStep);
	DEBUG1("usEnd = %d\n",usEnd);
	DEBUG1("imDelay = %d\n",imDelay);
	DEBUG1("iForceCurveCycles = %d\n",iForceCurveCycles);
	
	iZTmp = usBegin;
	usErrorTmp = read_ad_times_byCHN(gi_ad_sample_times,2);
	
	DEBUG1("forceCurveThread setpoint is %d\n",PID_setPoint);
for(i = 0; i < iForceCurveCycles; i++)
{
	DEBUG1("ForceCurve Cycle %d.\n",i);
		while(usErrorTmp > PID_setPoint)
	{		
		iZTmp = iZTmp + iStep;
		write_ad669_by_channel( iZTmp, DA_Z_ADR ); //0~200V�մɹ� 200V~���
		mdelay(imDelay);
		
		if(iZTmp >= usEnd)	goto forceCurveExit;
		
		usErrorTmp = read_ad_times_byCHN(gi_ad_sample_times,2);
		
		errorBuf[2*j + 6] = iZTmp;
		errorBuf[2*j + 1 + 6] = usErrorTmp;		



/////////////////////////////////////////////////////////////////
	
		j++;
		if(j % 256 == 0)
		{
			send(connect_socket_fd, (char*)errorBuf, 1036, 0);
//			DEBUG2("iZTmp = %d, usErrorTmp = %d\n",iZTmp,usErrorTmp);
	//		DEBUG1("Current X drive is %d\n",iTmp);
			j = 0;
		}
		
		if(g_current_task == STOP)
		{
			goto forceCurveExit; 
		}	
		k++;
	}
	
	//mdelay(iForceCurveInvDelay); //�س�������ʱ�ȴ�
	gettimeofday(&tpstart,NULL); //��ȡ��ʼʱ��
	
	do
	{
		usErrorTmp = read_ad_times_byCHN(gi_ad_sample_times,2);
			
		errorBuf[2*j + 6] = iZTmp;
		errorBuf[2*j + 1 + 6] = usErrorTmp;
		
		j++;
			if(j % 256 == 0)
			{
				send(connect_socket_fd, (char*)errorBuf, 1036, 0);
	//			DEBUG2("iZTmp = %d, usErrorTmp = %d\n",iZTmp,usErrorTmp);
				j = 0;
			}
			
			if(g_current_task == STOP)
			{
				goto forceCurveExit; 
			}
			
		gettimeofday(&tpend,NULL); //��ȡ��ֹʱ��
		timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
		timeuse = timeuse / 1000;//ms
		mdelay(10);
	}while(timeuse <= iForceCurveInvDelay);
	
	
//	iZTmp = iZTmp + iForceCurveInvComp; //�س������Ż�����
	
		while(iZTmp >= usBegin)
		{		
			iZTmp = iZTmp - iStep;
			write_ad669_by_channel( iZTmp, DA_Z_ADR ); //0~200V�մɹ� 200V~���
			mdelay(imDelay);
			
			if(iZTmp <= 50)	goto forceCurveExit;
			
			if(bInvShow == 1)
			{
				usErrorTmp = read_ad_times_byCHN(gi_ad_sample_times,2);
				
				errorBuf[2*j + 6] = iZTmp;
				errorBuf[2*j + 1 + 6] = usErrorTmp;
			
				j++;
				if(j % 256 == 0)
				{
					send(connect_socket_fd, (char*)errorBuf, 1036, 0);
			//		DEBUG2("iZTmp = %d, usErrorTmp = %d\n",iZTmp,usErrorTmp);
					j = 0;
				}
			}
			if(g_current_task == STOP)
			{
				goto forceCurveExit; 
			}	
		}
}
forceCurveExit:	
	errorBuf[1] = 1;
	errorBuf[2] = 0;
	errorBuf[3] = 1;
	errorBuf[4] = 0;
	
	j = (j - 2) > 0 ? (j - 2) : 0;
	errorBuf[5] = j; //ʣ�µĵ�Ҳ��Ҫ�ϴ�
	
	send(connect_socket_fd, (char*)errorBuf, 1036, 0);
	DEBUG2("iZTmp = %d, usErrorTmp = %d\n",iZTmp,usErrorTmp);
	
	write_ad669_by_channel( usBegin, DA_Z_ADR );
	
	pthread_mutex_lock(&g_current_task_mutex);	
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("Force curve thread stoped");
	
	if(gb_feedback_on == 1)
	{
		g_current_task = CMD_FEEDBACK_SWITCH;
		pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	}
	
	return (void*) 0;
}

/**
*@brief  ��ɨ���߳�
*@param  para(void*) ͨ��ѡ��
*@return  void*
*/
void* lineScanThread(void* para)
{
	int count = 0;
	int i, line;
	int dots = 512;
	
	int currentTask;
	
	int rangeX, rangeY, offsetX, offsetY, samplingTimes;
	float angle, cosAngle, sinAngle;
	float stepX, stepY, tempX, tempY;
	float fTmp = 0;
	int posX, posY;
	short sTmp;
	unsigned short usTmp;
	int iFastAxis = 0, iSlowAxis = 0;

	DEBUG0("Line scan thread start.");
	pthread_detach(pthread_self());//��Ϊ�����߳�

	//��鿪����ͨ��
	updateChannels(para);

	line = ( (COMMAND *)para )->para2;//ɨ��ڼ���, �� 1 ��ʼ
	line --;//ʹ���ɴ��㿪ʼ������
	
	//���²���
	pthread_mutex_lock(&g_current_task_mutex);
	dots = scanPixels;// ɨ�����ÿ��ɨ�趼���䣬���ֻ����һ��
	rangeX = scanRangeX;
	rangeX > 65535 ? 65535 : rangeX;
	rangeY = scanRangeY;
	rangeY > 65535 ? 65535 : rangeY;
	offsetX = scanOffsetX;
	offsetY = scanOffsetY;
	samplingTimes = scanSamplingTimes;

	
	if(scanAngle == 90)
	{
		angle = 0;
		iFastAxis = DA_Y_SCAN_ADR;
		iSlowAxis = DA_X_SCAN_ADR;
	}
	else if(scanAngle == 270)
	{
		angle = 180;
		iFastAxis = DA_Y_SCAN_ADR;
		iSlowAxis = DA_X_SCAN_ADR;
	}
	else
	{
		angle = scanAngle;
		iFastAxis = DA_X_SCAN_ADR;
		iSlowAxis = DA_Y_SCAN_ADR;
	}
	
	angle = PI * angle / 180.0;
	currentTask = g_current_task;
	pthread_mutex_unlock(&g_current_task_mutex); 

	//initScanBuf(CMD_LINESCAN_START, dots); //��ʼ���ش�����
	initScanBuf(SINGLE_LINE_RESULT, dots); //��ʼ���ش�����
	
	if(currentTask == STOP)
	{
		goto lineScanExit; 
	}	
	//���²������	

	cosAngle = cos(angle);
	sinAngle = sin(angle);
	stepX = (float)rangeX / (dots - 1);
	stepY = (float)rangeY / (dots - 1);	
	
//��ʼ��ʽ��ɨ��
	while(1)
	{
//		if(gb_lineFit == 0)			mdelay(50);


    /*�����Ŀ���������򣬵����������������ͻȻ�ж�ʱ����ֹ�߳�*/
    //if((connect_socket_fd = accept(listen_socket_fd, NULL, NULL)) == -1)
    //{
		//	currentTask == STOP;
    //}
			
		for(i = 0; i < dots; i++)
		{
			if(gb_lineFit)			
				tempX = hctc_x[i] * rangeX / 2;
			else
				tempX = (stepX * i + offsetX - rangeX / 2);
			tempY = (stepY * line + offsetY - rangeY / 2);
			if(cDriveWaveSel == 0)
				posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			else
			{
				fTmp = (float)i / dots;
				posX = 32768 + cos(fTmp * PI + PI) * rangeX / 2;
			}	
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;
			
			write_ad669_by_channel(posX,iFastAxis);
			write_ad669_by_channel(posY,iSlowAxis);
//			DEBUG2("posX=%d,  posY=%d\n",posX,posY);
//			udelay(200);
			
			errorBuf[i + 6] = pid_func() + 32767; //ͳһΪUSHORT				
			topoBuf[i + 6] = g_DA_z;	//ͳһΪUSHORT
			
			if(hasPhase) phaseBuf[i + 6] = read_ad_times_byCHN(gi_ad_sample_times,1);
		}
		updateLineNum(line + 1);
		sendData(1);
//		DEBUG1("send one line data, line %d\n", line + 1);		
		
		//��ɨѭ��
		for(i = dots - 1; i >= 0; i--) //����ɨ�ٶ���ͬ
		{
			
			if(gb_lineFit)
				tempX = hctc_x[i] * rangeX / 2;
			else
				tempX = (stepX * i + offsetX - rangeX / 2);
			tempY = (stepY * line + offsetY - rangeY / 2);
			
			if(cDriveWaveSel == 0)
				posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			else
			{
				fTmp = (float)i / dots;
				posX = 32768 + cos(fTmp * PI + PI) * rangeX / 2;
			}	
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;

			write_ad669_by_channel(posX,iFastAxis);
	//		DEBUG2("posX=%d,  posY=%d\n",posX,posY);
		//	udelay(200);
			
			if(hasPhase) phaseBuf[i + 6] = read_ad_times_byCHN(gi_ad_sample_times,1);
			
			
			usTmp = pid_func() + 32767;
      if(hasInv) 
			{
					topoBufInv[i + 6] = g_DA_z;
					errorBufInv[i + 6] = usTmp; //ͳһΪUSHORT		
					if(hasPhase) phaseBufInv[i + 6] = read_ad_times_byCHN(gi_ad_sample_times,1);						
			}
		}
		DEBUG1("hasInv %d\n", hasInv);
		if(hasInv)
		//if(1)//��󣬷�ɨ������һֱ����
		{
			updateLineNum( -line - 1); //������ʾ��ɨ��
			sendData(0);
	//		DEBUG1("Send one line data, line %d\n", -line - 1);
		}

		//���²�����׼����һ��ɨ��
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		rangeX = scanRangeX;
		rangeY = scanRangeY;
		offsetX = scanOffsetX;
		offsetY = scanOffsetY;
		samplingTimes = scanSamplingTimes;
		
		if(scanAngle == 90)
		{
			angle = 0;
			iFastAxis = DA_Y_SCAN_ADR;
			iSlowAxis = DA_X_SCAN_ADR;
		}
		else if(scanAngle == 270)
		{
			angle = 180;
			iFastAxis = DA_Y_SCAN_ADR;
			iSlowAxis = DA_X_SCAN_ADR;
		}
		else
		{
			angle = scanAngle;
			iFastAxis = DA_X_SCAN_ADR;
			iSlowAxis = DA_Y_SCAN_ADR;
		}
		angle = PI * angle / 180.0;
		
		//���ݸ��²������㲽�࣬����ɨ����ֻ����һ��
		cosAngle = cos(angle);
		sinAngle = sin(angle);
		stepX = (float)rangeX / (dots - 1);
		stepY = (float)rangeY / (dots - 1);	
		
		pthread_mutex_unlock(&g_current_task_mutex); 	
			
		if(currentTask == STOP)
		{
			goto lineScanExit; 
		}	
	}
	
lineScanExit:	
	pthread_mutex_lock(&g_current_task_mutex);
	
	for(i = 0 ;i <= 512; i++)
	{
		write_ad669_by_channel(i * 32768 / 512,iFastAxis);
		udelay(100);
		if(g_pid_mode == 1)		pid_func(); //Soft pid
	}		
	
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("line scan stoped");
	
	if(gb_feedback_on == 1)
	{
		g_current_task = CMD_FEEDBACK_SWITCH;
		pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	}
	
	return (void*) 0;
}


void* normalScanThread(void* para)
{
	int count = 0;
	int i, row;
	int dots;
	int currentTask;
	int rangeX, rangeY, offsetX, offsetY, samplingTimes;
	float angle, cosAngle, sinAngle;
	float stepX, stepY, tempX, tempY;
	float fTmp = 0;
	int posX, posY;
	short sTmp;
	int iFastAxis = 0, iSlowAxis = 0;
	
	DEBUG0("Normal scan thread start.");
	pthread_detach(pthread_self());//��Ϊ�����߳�

	//��鿪����ͨ��
	updateChannels(para);

	//���²���
	pthread_mutex_lock(&g_current_task_mutex);
	
	dots = scanPixels;// ɨ�����ÿ��ɨ�趼���䣬���ֻ����һ��
	rangeX = scanRangeX;
	rangeY = scanRangeY;
	offsetX = scanOffsetX;
	offsetY = scanOffsetY;
	samplingTimes = scanSamplingTimes;
	
	DEBUG1("Angle is %d\n",scanAngle);
	
	if(scanAngle == 90)
	{
		angle = 0;
		iFastAxis = DA_Y_SCAN_ADR;
		iSlowAxis = DA_X_SCAN_ADR;
	}
	else if(scanAngle == 270)
	{
		angle = 180;
		iFastAxis = DA_Y_SCAN_ADR;
		iSlowAxis = DA_X_SCAN_ADR;
	}
	else
	{
		angle = scanAngle;
		iFastAxis = DA_X_SCAN_ADR;
		iSlowAxis = DA_Y_SCAN_ADR;
	}
	
	angle = PI * angle / 180.0;
	currentTask = g_current_task;
	pthread_mutex_unlock(&g_current_task_mutex); 

	initScanBuf(CMD_SCAN_WHOLE, dots);
//	ad_channel_sel(0x7);//��ʼ��ѡ��Errorͨ��

/* cancelled in 20120508 for AD_IN signal detection
	if(gc_tb_channel == 0)	ad_channel_sel(0x2); //y signal
	else	ad_channel_sel(0x7);
*/

//	ad_channel_sel(0xC); //for AD_IN detection
		
	if(currentTask == STOP)
	{
		goto normalScanExit; 
	}	
	//���²������	

	cosAngle = cos(angle);
	sinAngle = sin(angle);
	stepX = (float)rangeX / (dots - 1);
	stepY = (float)rangeY / (dots - 1);	
	
//��ʼ��ʽȫͼɨ��
	for (row = 0; row < dots; row++) // ��ǰ��
	{
		
		
		/*�����Ŀ���������򣬵����������������ͻȻ�ж�ʱ����ֹ�߳�*/
    //if((connect_socket_fd = accept(listen_socket_fd, NULL, NULL)) == -1)
    // {
		//currentTask == STOP;
    //}
			
		
		
		//��ɨѭ��
		for(i = 0; i < dots; i++)
		{			
			if(gb_lineFit)
			{
				tempX = hctc_x[i] * rangeX / 2;
				tempY = hctc_y[row] * rangeY / 2;
			}
			else
			{
				tempX = (stepX * i + offsetX - rangeX / 2);
				tempY = (stepY * row + offsetY - rangeY / 2);
			}
			
			if(cDriveWaveSel == 0)
				posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			else
			{
				fTmp = (float)i / dots;
				posX = 32768 + cos(fTmp * PI + PI) * rangeX / 2;
			}	
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;
			
			write_ad669_by_channel(posX,iFastAxis);
			write_ad669_by_channel(posY,iSlowAxis);
	//		udelay(50);
			
			if(g_pid_mode == 0) //Hard Pid
			{
	//			udelay(g_point_time - 50);
				if(count % 2 == 0)
						topoBuf[i + 6] = read_ad_by_channel(0x6,scanSamplingTimes) + 32768; //select height
				else	
						errorBuf[i + 6] = read_ad_by_channel(0x7,scanSamplingTimes) + 32768; //select error
						
				if(hasDamp)
					dampBuf[i + 6] = read_ad_by_channel(0xC,scanSamplingTimes) + 32768;
			}
			else  //Soft Pid
			{
				errorBuf[i + 6] = pid_func() + 32768; //����short��error
				topoBuf[i + 6] = g_DA_z; //�����short
				
				if(hasPhase) phaseBuf[i + 6] = read_ad_times_byCHN(gi_ad_sample_times,1);
/*				
				if(hasDamp)
					dampBuf[i + 6] = read_ad_by_channel(0xC,scanSamplingTimes) + 32768;	
					
				if(hasFiction)
					fictionBuf[i + 6] = read_ad_by_channel(0x1,scanSamplingTimes) + 32768;
				
				if(hasFiction || hasDamp)
				{
					if(gc_tb_channel == 0)
					{		
						ad_channel_sel(0x2); //y signal
					}
					else
					{
						ad_channel_sel(0x7);
					}
				}
*/
			}		
		}
		updateLineNum(row + 1);		//��1��ʼ
		
		if(g_pid_mode == 0)
		{
			if(count % 2 == 0)
			{
				send(connect_socket_fd, (char*)topoBuf, 1036, 0);
				DEBUG0("send topoBuf");
			}
			else
			{
				send(connect_socket_fd, (char*)errorBuf, 1036, 0);
				DEBUG0("send errorBuf");
			}
		}
		else
			sendData(1);
//		DEBUG1("send one line data, line %d\n\n", row+1);		
		
		//��ɨѭ��
		for(i = dots - 1; i >= 0; i--)
		{
			//tempY = (stepY * row + offsetY - rangeY / 2);
			
			if(gb_lineFit)
				tempX = hctc_x[i] * rangeX / 2;
			else
				tempX = (stepX * i + offsetX - rangeX / 2);
			
			if(cDriveWaveSel == 0)
				posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			else
			{
				fTmp = (float)i / dots;
				posX = 32768 + cos(fTmp * PI + PI) * rangeX / 2;
			}	
			//posY = 32768 - tempX * sinAngle + tempY * cosAngle;

			write_ad669_by_channel(posX,iFastAxis);
			//write_ad669_by_channel(posY,iSlowAxis);
//			udelay(50);
			
			if(g_pid_mode == 1)	sTmp = pid_func();
			
			if(hasInv) 
			//if(1)//���
			{
				if(g_pid_mode == 0)
				{
					topoBufInv[i + 6] = read_ad_by_channel(0x6,scanSamplingTimes); //select height
					if(hasError)	
						errorBufInv[i + 6] = read_ad_by_channel(0x7,scanSamplingTimes); //select error
					if(hasDamp)
						dampBufInv[i + 6] = read_ad_by_channel(0xC,scanSamplingTimes) + 32768;
				}
				else
				{
					topoBufInv[i + 6] = g_DA_z;					
						
					if(hasError) errorBufInv[i + 6] = sTmp+ 32768;
					
					//topoBuf[i + 6 + 512] = g_DA_z;//������
					//errorBuf[i + 6 + 512] = sTmp+ 32768;//������
						
					if(hasPhase) phaseBuf[i + 6] = read_ad_times_byCHN(gi_ad_sample_times,2);
						
					if(hasDamp)  dampBufInv[i + 6] = read_ad_by_channel(0xC,scanSamplingTimes) + 32768;	
					
					if(hasFiction)
						fictionBufInv[i + 6] = read_ad_by_channel(0x1,scanSamplingTimes) + 32768;
					
					if(hasFiction || hasDamp)
					{
						if(gc_tb_channel == 0)
						{	
							ad_channel_sel(0x2); //y signal
						}
						else
						{
							ad_channel_sel(0x7);
						}	
					}
				}				
			}
		}
		if(hasInv) 
		{
			updateLineNum( -row - 1);	
			sendData(0);
			DEBUG1("Send one line data, line %d\n", -row - 1);
		}			
		//sendData(1);������
		
		if(g_pid_mode == 0)
		{	
			if(hasError)
			{
				if(count % 2 == 0)
				{
					ad_channel_sel(0x7);
					row--;
				}
				else
					ad_channel_sel(0x6);
					
				count++;
			} 
		}

		//���²�����׼����һ��ɨ��
		/*
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		rangeX = scanRangeX;
		rangeY = scanRangeY;
		offsetX = scanOffsetX;
		offsetY = scanOffsetY;
		samplingTimes = scanSamplingTimes;
		angle = PI * scanAngle / 180.0;
		pthread_mutex_unlock(&g_current_task_mutex); 	
		*/
		
		//�ж��Ƿ�ֹͣɨ��
		if(currentTask == STOP)
		{
			goto normalScanExit; 
		}	
		
		if(g_whole_scanning == 0) //stop
			goto normalScanExit;
		else if(g_whole_scanning == 2) //pause
			while(g_whole_scanning == 2)	;
		
		//���ݸ��²������㲽�࣬����ɨ����ֻ����һ��
		/*
		cosAngle = cos(angle);
		sinAngle = sin(angle);
		stepX = (float)rangeX / (dots - 1);
		stepY = (float)rangeY / (dots - 1);	
		*/
	}

normalScanExit:	
	//��̽���λ������λ��	
	for(i =100 ;i >= 0; i--)
	{
		write_ad669_by_channel(32768 + i * posX / 100,iFastAxis);
		write_ad669_by_channel(32768 + i * posY / 100,iSlowAxis);
		udelay(50);
		if(g_pid_mode == 1) pid_func();
	}	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("Leave normal scan thread");	
	
	if(gb_feedback_on == 1)
	{
		g_current_task = CMD_FEEDBACK_SWITCH;
		pthread_create(&servoTid, NULL, feedbackServoThread, NULL);
	}
		
	return (void*) 0;
}

/**
    ���²�����ѹ��ɨ�������ͷ����Խ�������������
*/
static float InvSqrt(float x)
{  
    float xhalf = 0.5f * x;  
    long i = *(long*) &x;  
    i = 0x5f3759df- (i >> 1);  
    x = *(float*) &i;  
    x = x * (1.5f - xhalf * x * x);  
    return x;  
}   
/*
float sqrt(float a);
float sqrt(float a)
{
    float x1 = a / 2;
    float x2 = (x1 + a/ x1) / 2;
   
    while((x2 - x1) >= 1e-5 || (x1 - x2) >= 1e-5)
    {
          x1 = x2;
          x2 = (x1 + a/ x1) / 2;
    }
    return x1;
}
*/
/**
*@brief �ö��κ����ķ�������������У��
*@param y ��һ����λ��
*@param a ����ϵ��, ����Ϊ 0 �����г� 0 ����
*@return ��һ���ĵ�ѹ
*/
static float corr2 ( float y, float a ) //ֻ����ɨ��У��
{
    /*  ����ɨ�����ĳ������Կ��Ա�ʾΪ ���ζ���ʽ��ʽ
        ����Ϊ y = a x^2 - a + x
        ���ݳ���ϵ�� a �Ϳ��Լ��������������ѹ
    */
    float sq;
    if ( a == 0 ) return y;
    sq = sqrt ( 1 + 4 * a * ( a + y ) );
    if ( a > 0 ) //��Ӧ���Ƕ���ɨ���ߵĽ�����ȡ��ĸ�
        return 0.5 * ( -1 + sq ) / a;
    else //��Ӧ���ǶԷ�ɨ���ߵĽ�����ȡС�ĸ�
        return 0.5 * ( -1 - sq ) / a;
}


//�����κ����ķ�������������У��
static float corr3 ( float y, float a, float b )
{
    float x, f, ff;
    float i;
    if ( y > 1 ) y = 1;
    if ( y < -1 ) y = -1;

    //�����Է���Ϊ y = a x^3 + b x^2 - a x - b + x
    x = y; //set the initial value of iteration
    /* ����ţ�ٵ������󷽳̵ĸ� 
        ���ﾫ��Ҫ�󲻸ߣ��̶�������������
    */
    for ( i = 0; i < 5; i++ ) 
    {
        f = a * x * x * x + b * x * x - a * x - b + x - y;
        ff = 3 * a * x * x + 2 * b * x - a + 1;
        x = x - f / ff; // x = x - f / f'
    }
    return x;
}
/**
*@brief ����ɨ��ʱ����ĳ��ͽ���ϵ��
*@param a1 ��ɨʱ��У��ϵ�� A * 100, a1 ȡֵ [-100, 100]
*@param b1 ��ɨʱ��У��ϵ�� B * 100, b1 ȡֵ [-50, 50]
*@param a2 ��ɨʱ��У��ϵ�� A * 100, a2 ȡֵ [-100, 100]
*@param b2 ��ɨʱ��У��ϵ�� B * 100, b2 ȡֵ [-50, 50]
*@return void
*/
void setHysteresisParaX(short a1, short b1, short a2, short b2)
{
    if(a1 < -100) a1 = -100;
    if(a1 > 100) a1 = 100;
    if(b1 < -50) b1 = -50;
    if(b1 > 50) b1 = 50;
    if(a2 < -100) a2 = -100;
    if(a2 > 100) a2 = 100;
    if(b2 < -50) b2 = -50;
    if(b2 > 50) b2 = 50;  
    
    hctc_xA = a1 / 100.0f;
    hctc_xB = b1 / 100.0f;
    hcrc_xA = a2 / 100.0f;
    hcrc_xB = b2 / 100.0f;
    DEBUG2("X ParaA = %d, ParaB = %d\n", hctc_xA, hctc_xB);
    updateHCCurve(scanPixels);
}


/**
*@brief ����ɨ��ʱ����ĳ��ͽ���ϵ��
*@param a1 ��ɨʱ��У��ϵ�� A * 100, a1 ȡֵ [-100, 100]
*@param b1 ��ɨʱ��У��ϵ�� B * 100, b1 ȡֵ [-50, 50]
*@param a2 ��ɨʱ��У��ϵ�� A * 100, a2 ȡֵ [-100, 100]
*@param b2 ��ɨʱ��У��ϵ�� B * 100, b2 ȡֵ [-50, 50]
*/
void setHysteresisParaY(short a1, short b1, short a2, short b2)
{

    if(a1 < -100) a1 = -100;
    if(a1 > 100) a1 = 100;
    if(b1 < -50) b1 = -50;
    if(b1 > 50) b1 = 50;
    if(a2 < -100) a2 = -100;
    if(a2 > 100) a2 = 100;
    if(b2 < -50) b2 = -50;
    if(b2 > 50) b2 = 50;  
    
    hctc_yA = a1 / 100.0;
    hctc_yB = b1 / 100.0;
    hcrc_yA = a2 / 100.0;
    hcrc_yB = b2 / 100.0;
    DEBUG2("Y ParaA = %d, ParaB = %d\n", hctc_yA, hctc_yB);
    updateHCCurve(scanPixels);
}

/**
*@brief  ���·�����У�����ߣ��������κ�����������Ϊ��������
*@param  void
*@return  void
*/
void updateHCCurve ( int num )
{
    int i;
    double y;

    for ( i = 0; i < num; i++ )
    {
        y = 2.0 * i / num - 1;
        hctc_x[i] = corr3 ( y, hctc_xA, hctc_xB );
        hctc_y[i] = corr3 ( y, hctc_yA, hctc_yB );
       // DEBUG2("hcrc_x[%d] = %.2f\n",i,hcrc_x[i]);
       // DEBUG2("hcrc_y[%d] = %.2f\n",i,hcrc_y[i]);
    }
}

void setTubeSize(float paraX, float paraY, float paraZ)
{
	tubeSizeX = paraX;
	tubeSizeY = paraY;
	tubeSizeZ = paraZ;
}

void setIdentX(char para1,float para2)
{
	bIdentXCheck = para1;
	fIdentX = para2;
}

void setForceCurveCompPara(int para1, int para2)
{
	iForceCurveInvComp = para1;
	iForceCurveInvDelay = para2;
}

void setForceCurveCycles(int para)
{
	iForceCurveCycles = para;
}

void setDriveWave(char para)
{
	cDriveWaveSel = para;
}
