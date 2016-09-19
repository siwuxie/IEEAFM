#include "afm_comm.h"
#include "hardware.h"
#include <string.h>
#include "work_thread.h"

//----扫频参数------
static unsigned int freqScanStartFreq;
static unsigned int freqScanEndFreq;
unsigned char freqScanDriAmp; //驱动幅值
unsigned short freqScanRes;	//扫频点数
unsigned short freqScanDelay;	//扫频速度

static short freqScanAmpBuf[518];
static short freqScanPhaseBuf[518];

/**
*@brief  设置扫频频率范围
*@param  startFreq(unsigned int) 起始频率*100
*@param  endFreq(unsigned int) 截止频率*100
*@return  void
*/
void setFreqRange(unsigned int startFreq, unsigned int endFreq)
{
	freqScanStartFreq = startFreq;
	freqScanEndFreq = endFreq;
}

void setFreqPara(void* para)
{
	struct command *pCmd = (struct command *) para;
	freqScanDriAmp = pCmd->para1 & 0xFF;
	setFreqDriAmp(freqScanDriAmp);
	
	freqScanRes = pCmd->para2;
	freqScanDelay = pCmd->para3;
	DEBUG1("DriAmp is %d\n",freqScanDriAmp);
	DEBUG1("ScanRes is %d\n",freqScanRes);
	DEBUG1("ScanDelay is %d us\n",freqScanDelay);
}

void setFreqDriAmp(unsigned char para)
{
	int i = 0;
	char ctr;
	outb(0,DDS_522_CS_ADR); //使能MAX522
	ctr = 0x23;
	for(i = 7; i >= 0; i--) //发送控制字
	{
		outb(ctr >> i, DDS_522_DATA_ADR);
	}
	for(i = 7; i >= 0; i--) //发送数据
	{
		outb(para >> i, DDS_522_DATA_ADR);
	}
	outb(1,DDS_522_CS_ADR); //刷新MAX522
}

void dds_Reset()
{
	/*reset DDS*/
	outb(0x0,DDS_RST_ADR);
	outb(0x1,DDS_RST_ADR);
	outb(0x0,DDS_RST_ADR);
}

void dds_Out(double freq)
{
	const unsigned int DIV = 0xffffffff;
	const double Ratio = 125000000.0 / DIV; //125M clk
	unsigned int Data = (unsigned int) (freq / Ratio);
	unsigned char* pData = (unsigned char*)(&Data);
	
	//Write FRQ
	outb(0x00, DDS_ADR);
	outb(pData[3], DDS_ADR);
	outb(pData[2], DDS_ADR);
	outb(pData[1], DDS_ADR);
	outb(pData[0], DDS_ADR);
	// Update Frq
	inb(DDS_ADR);	 
}

/**
*@brief  扫频线程
*@param  para(struct command *) 通道选择等
*@return  void*
*/
void* freqScanThread(void* para)
{
	int i;
	int count = 0;
	int currentTask;
	pthread_detach(pthread_self());//成为自由线程
	
	double startFreq = (double) freqScanStartFreq;
	double endFreq = (double) freqScanEndFreq;
	double currentFreq;	
	
/************************************************************************/
	DEBUG0("STARTING FREQENCY SCANNING ..................................");
/************************************************************************/

	freqScanAmpBuf[0] = CMD_FREQ_SCAN;
	freqScanAmpBuf[1] = 0; //Amp
	freqScanPhaseBuf[0] = CMD_FREQ_SCAN;
	freqScanPhaseBuf[1] = 1; //Phase

//由于模拟开关的速度限制，频率和相位分两次发送
for(count = 0; count < 2; count++) 
{
	for(i = 0; i < freqScanRes; i++)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		currentFreq = startFreq + i * (endFreq - startFreq) / freqScanRes;
		dds_Out(currentFreq);
		udelay(freqScanDelay);
		if(count % 2 == 0)
			freqScanAmpBuf[6 + i] = read_ad_by_channel(0x3,scanSamplingTimes);
		else
			freqScanPhaseBuf[6 + i] = read_ad_by_channel(0x4,scanSamplingTimes);
	}
}

	send(connect_socket_fd, (char*)&freqScanAmpBuf, 1036, 0);
	DEBUG0("send(connect_socket_fd, (char*)&freqScanAmpBuf, 1036, 0);");

	send(connect_socket_fd, (char*)&freqScanPhaseBuf, 1036, 0);
	DEBUG0("send(connect_socket_fd, (char*)&freqScanPhaseBuf, 1036, 0);");

	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("End of freq scan thread ......");
	return (void*)0;
}

// 以下天大项目方法
void setTappingSingle(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("choose Amplitude");
		break;
	case 1:
		DEBUG0("choose Phase");
		break;
	case 2:
		DEBUG0("choose Frequency");
		break;
	}
}
void setTdPidMode(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("auto adjustment");
		break;
	case 1:
		DEBUG0("manual adjustment");
		break;
	}
}
void setImageScale(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("image scale 1:1");
		break;
	case 1:
		DEBUG0("image scale 1:4");
		break;
	case 2:
		DEBUG0("image scale 1:8");
		break;
	case 3:
		DEBUG0("image scale 1:16");
		break;
	}
}
void setSlowAxis(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("slow axis on");
		break;
	case 1:
		DEBUG0("slow axis off");
		break;
	}
}
void setSwitchError(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("switch error off");
		break;
	case 1:
		DEBUG0("switch error on");
		break;
	}
}
void setSwitchFriction(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("switch friction off");
		break;
	case 1:
		DEBUG0("switch friction on");
		break;
	}
}
void setSwitchDeflection(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("switch deflection off");
		break;
	case 1:
		DEBUG0("switch deflection on");
		break;
	}
}
void setSwitchPhase(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("switch phase off");
		break;
	case 1:
		DEBUG0("switch phase on");
		break;
	}
}
void setSwitchAmplitude(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("switch amplitude off");
		break;
	case 1:
		DEBUG0("swictch amplitude on");
		break;
	}
}
void setImageDirection(unsigned short para)
{
	switch (para)
	{
	case 0:
		DEBUG0("up to down");
		break;
	case 1:
		DEBUG0("down to up");
		break;
	}
}
// 以上天大项目方法

