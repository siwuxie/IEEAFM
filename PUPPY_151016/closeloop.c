#include "afm_comm.h"
#include "closeloop.h"
#include "hardware.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

extern unsigned int g_point_time; //闂佽法鍠愰弸濠氬箯闁垮顥囬梺璺ㄥ枑閺嬪骞忛柨瀣槯闂佽法鍠愰弸濠氬箯闁垮妲ㄩ梺璺ㄥ枑閺嬪骞忛柨瀣棁闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊宕戝鍫熸櫢闁哄倶鍊栫�氬綊寮崼鏇熸櫢闁哄倶鍊栫�氾拷
extern int g_step_resp_time;
extern int scanSamplingTimes;
extern float linerate;

/*Work mode para*/
static unsigned char g_work_mode = 0; //STM
const unsigned char AD_OSAP = 0x05;
const unsigned char AD_FICTION = 0x08; // not real
const unsigned char AD_PHASE = 0x0B;
const unsigned char AD_ERROR = 0x09;
const unsigned char AD_STM = 0x0C;

//---PID闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氾拷------
unsigned char g_pid_mode = 0; //Hard pid
pthread_mutex_t g_PID_mutex = PTHREAD_MUTEX_INITIALIZER;
float PID_Kp = 0.1;
float PID_Ki = 0.1;
float PID_Kd = 0;
int PID_loopCircles; //鐎甸偊浜弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷

int PID_eps;
int PID_errorThresh;
int PID_samplingTimes;//PID AD 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
int PID_delay;//闂佽法鍠栭妶鐑樻綇閻愵剙顏堕柡鍐ㄧ埣閺佹捇寮妶鍡楊伓

unsigned char PID_channel;
extern unsigned short PID_setPoint;
extern int g_current_task;

extern int g_point_time_left;
extern char gb_feedback_on;
extern int gi_da_z;

int g_DA_z = 32768; 	//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氾拷 Z 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹 DA 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁悶娑欘殣閹凤拷

short (*pid_func)();

#define PID_DELAY 10

struct timeval tpstart,tpend;

extern int gi_feedback_cycle_times;

extern int gi_ad_sample_times;

void setWorkMode(unsigned short para)
{
	switch (para)
	{
	//case 0:
		//outb(0x1,SPM_WORK_MODE);//-------------------------
		//g_work_mode = 0;
		//DEBUG0("set to STM mode");
		//break;
	//case 1:
	case 0://闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹
		outb(0x4,SPM_WORK_MODE);//-------------------------
		g_work_mode = 1;
		DEBUG0("set to AFM contact mode");
		break;
	//case 2:
	case 1://闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹
		outb(0x2,SPM_WORK_MODE);//-------------------------
		g_work_mode = 2;
		DEBUG0("set to AFM tapping mode");
		break;
	}
}

void setPidMode(unsigned short para)
{
	switch (para)
	{
	case 0:
		outb(0x1,SPM_PID_MODE);//-------------------------
		g_pid_mode = 0;
		DEBUG0("set to hard pid mode");
		break;
	case 1:
		outb(0x2,SPM_PID_MODE);//-------------------------
		g_pid_mode = 1;
		DEBUG0("set to soft pid mode");
		break;
	}
}

void setFeedBackMode(short mode)
{
	switch(mode)
	{
		case 1:
			pid_func = PID_function01;
			DEBUG0("Set PID Func to PID_function01");
			break;
		case 2:
			pid_func = PID_function02;
			DEBUG0("Set PID Func to PID_function02");
			break;
		case 3:
			pid_func = PID_function03;
			DEBUG0("Set PID Func to PID_function03");
			break;
		case 4:
			pid_func = PID_function04;
			DEBUG0("Set PID Func to PID_function04");
			break;	
		case 5:
			pid_func = PID_function05;
			DEBUG0("Set PID Func to PID_function05");
			break;		
		case 6:
			pid_func = PID_function05;
			DEBUG0("Set PID Func to PID_function06");
			break;			
	}
}

void setPIDPara(unsigned short Kp, unsigned short Ki, unsigned short Kd, unsigned short loopCircles)
{
	PID_Kp = (float)Kp/1000;
	PID_Ki = (float)Ki/1000;
	PID_Kd = (float)Kd/255;
	PID_loopCircles = loopCircles;
	
//	outb(Kp,HARD_PID_PR);
//	outb(Ki,HARD_PID_INR);
	
	DEBUG2("PID_Kp=%.3f, PID_Ki=%.3f\n", PID_Kp,PID_Ki);
	//DEBUG1("Set PID_Ki to %.3f\n", PID_Ki);
	//DEBUG1("Set PID_Kd to %.3f\n", PID_Kd);
	//DEBUG1("Set PID_loopCircles to %d\n", PID_loopCircles);
}

void setPIDParaSlider(unsigned short Kp, unsigned short Ki)
{
	PID_Kp = (float)Kp/255;
	PID_Ki = (float)Ki/255;
	
	outb(Kp,HARD_PID_PR);
	outb(Ki,HARD_PID_INR);
	
//	DEBUG1("Set PID_Kp to %.3f\n", PID_Kp);
//	DEBUG1("Set PID_Ki to %.3f\n", PID_Ki);
}

void setPIDParaOther(short times, short eps, short delay, short errorThresh)
{
	PID_eps = eps;
	PID_delay = delay;
	PID_errorThresh = errorThresh;
	if(times <= 0) PID_samplingTimes = 1;
	else PID_samplingTimes = times;
		
	DEBUG1("Set PID_eps to %d\n", PID_eps);
	DEBUG1("Set PID_delay to %d\n", PID_delay);
	DEBUG1("Set PID_errorThresh to %d\n", PID_errorThresh);
	DEBUG1("Set PID_samplingTimes to %d\n", PID_samplingTimes);
}

void setPIDChannel(unsigned char channel)
{
	PID_channel = channel;
	//DEBUG1("Set PID_channel to %d\n", PID_channel);
}

void setPIDSetPoint(unsigned short para) // -10V ~ 10V
{
	PID_setPoint = para; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎ID
//	write_ad669_by_channel(para, DA_SET_POINT_ADR); //婵☆垽绻濋弫鎾诲棘閵堝棗顏禤ID
	DEBUG1("Set PID_setPoint to %d\n", PID_setPoint);
}

// DA_Z 闂侇偅淇洪惌楣冩煥閻斿憡鐏柟鐤腹缁旀挳鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闂佹儳搴滈幏锟�
// g_DA_z = 0 (-10V) 闂佽法鍠愰弸濠氬箯妞嬪簺浠涢梺璺ㄥ枔绗戦柣锟狀棑濞呫垽骞忛悜鑺ユ櫢閻忕偛锕ら崺宀勬煥閻旇櫣鎽傞梻锟介敓锟�
// g_DA_z = 65535 (10V) 闂佽法鍠愰弸濠氬箯妞嬪簺浠涢梺璺ㄥ枔绗戦柣锟狀棑濞呫垽骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ櫐閹凤拷
/**
*@brief  闂佽法鍠愰弸濠氬箯閻戞枻鎷峰鍖㈤梺璺ㄥ枎椤洖鈻旈敓锟�
*@param  void
*@return  闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鎸婇弴銏℃櫢闁跨噦鎷� short
*/
short PID_function01(void)
{
	int nZ_Value;
	int tmp = 0;
	short sError,sError_test;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i = 1;
	unsigned int  timeuse = 0;
	int iTimeLeft = 0;
	int chn_a,chn_b,chn_c,chn_d,t_b,t_b_test;
	int A,B,C,D;
	unsigned short *chn;
	
	nZ_Value = g_DA_z;
	//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
	
	chn = read_ad_times_4CHN(gi_ad_sample_times,4);

	chn_a = chn[0];
	chn_b = chn[1];
	chn_c = chn[2];
	chn_d = chn[3];
//	chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
//	chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
//	chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
//  chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
	
	t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
	sError = t_b + PID_setPoint;//濞戞挻妞介弫鎾诲棘閵堝棗顏堕悹鍥︾窔閺佹捇寮妶鍡楊伓濞达絽绉归弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏秙etpoint濞戞挻妞介弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氳绋夐敓锟�+
	
	//DEBUG1("sError is %d\n",sError);
	//DEBUG1("g_point_time_left1 is %d\n",g_point_time_left);
	gettimeofday(&tpstart,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忓畡閭︽綏闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	do //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{

		n_p = PID_Kp * sError;
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎I
		
		//DEBUG2("PID_Kp=%.3f, PID_Ki=%.3f\n", PID_Kp,PID_Ki);
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		//DEBUG1("Nz_value is %d\n",nZ_Value);
		//DEBUG1("g_point_time_left is %d\n",g_point_time_left);
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
		//udelay(15); //闂佽法鍠愰弸濠氬箯閾氬倽绀�15us闂佽法鍠愰弸濠氬箯妞嬪酣鍏囩紓浣哄枛閺佹捇寮妶鍡楊伓婵﹣鍗抽弫鎾诲棘閵堝棗顏堕幖瀛樻⒒鐎硅櫕绋夐敓锟�10us
		//DEBUG1("g_step_resp_time is %d\n",g_step_resp_time);
		
		//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
//		chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
//		chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
//		chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
//		chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
	  
	  //DEBUG1("chn_a is %d\n",chn_a);
	  //DEBUG1("chn_b is %d\n",chn_b);
	  //DEBUG1("chn_c is %d\n",chn_c);
	  //DEBUG1("chn_d is %d\n",chn_d);
	  chn = read_ad_times_4CHN(gi_ad_sample_times,4);

	  chn_a = chn[0];
	  chn_b = chn[1];
	  chn_c = chn[2];
	  chn_d = chn[3];

		t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鎮抽锔芥櫢闁哄倶鍊栫�氬綊鏌ㄩ悢铏瑰摵缁炬澘顦扮�氬綊鏌ㄩ悢娲绘晭闁跨噦鎷�
		
		//DEBUG1("t_b is %d\n",t_b);
		//DEBUG1("setPoint is %d\n",PID_setPoint);
		
		sError = t_b + PID_setPoint;//t_b濞戞挻妞介弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�+setpoint闂佽法鍠愰弸濠氬箯閻ゎ垳妲堥梺璺ㄥ枑閺嬪骞忛摎鍌滅Т闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱畟etpoint闁稿﹤銈搁弫鎾诲棘閵堝棗顏跺☉鎾存そ閺佹捇寮妶鍡楊伓闁稿鎷�
		
		//DEBUG1("sError is %d\n",sError);
		//DEBUG1("sError_test is %d\n",sError_test);
		
		gettimeofday(&tpend,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忛柨瀣靛壘闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	 	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	 	//DEBUG1("timeuse is %d\n",timeuse);
	 	i++;
	}while(timeuse < g_point_time_left);
	
	g_DA_z = nZ_Value;
	return sError;
}

/**
*@brief  闁告瑯浜弫鎾诲矗椤愶絽鏁鹃柟椋庡厴閺佹捇寮妶鍡楊伓闂佽法鍠栭崣鐑樺濞嗘劕顏堕梺璺ㄥ枙椤㈡粍瀵煎▎鎰伓闂佽法鍠愰弸濠氬箯閿燂拷
*@param  void
*@return  error闂佽法鍠曢崜濂告偖鐎涙ê顏� short
*/
short PID_function02(void) //闂佽法鍠庨敓鐣屽枎閻ｆ儳顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
{   
	int nZ_Value;
	int tmp = 0;
	short sError;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i;
	
	nZ_Value = g_DA_z;
	sError = read_ad_by_channel( 0x7, 1 ); //select error
//	read_ad676_chip2(&sError); // ad676
	
	for(i= 0; i < gi_feedback_cycle_times; i++) //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
		n_p = PID_Kp * sError;
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i;
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = 65535 - nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
		
		sError = read_ad_times(1); //select error
//	read_ad676_chip2(&sError); // ad676
	}
	
	g_DA_z = nZ_Value;
	return sError;
}

unsigned short PID_function03(void) 
{   	
	int nZ_Value;
	int tmp = 0;
	int sError;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i;
	unsigned short us_Y = 0;
	unsigned int  timeuse = 0;
	int iTimeLeft = 0;
	
	//DEBUG0("Into PID thread!\n");
	nZ_Value = gi_da_z;
	//sError = 32768 - read_ad_times(gi_ad_sample_times) - PID_setPoint;
	sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
/*	
	for(i= 0; i < gi_feedback_cycle_times; i++) //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
		n_p = PID_Kp * sError;
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i;
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�

		us_Y = read_ad_times_byCHN(gi_ad_sample_times,2);
		sError = us_Y - PID_setPoint;;
	}
*/

	gettimeofday(&tpstart,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忓畡閭︽綏闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	do //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
		DEBUG0("pidfunction03\n");
		n_p = PID_Kp * sError;
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i;
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
				
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
		
		us_Y = read_ad_times_byCHN(gi_ad_sample_times,2);
		sError = us_Y - PID_setPoint;
		
		gettimeofday(&tpend,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忛柨瀣靛壘闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	 	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	}while(timeuse < g_point_time_left);
			
	gi_da_z = nZ_Value;
	g_DA_z = nZ_Value;
	return us_Y;
}
/*wangqian PID闂佽法鍠庨～鏇炩枖閿燂拷*/
//闂佽法鍠愰弸濠氬箯閾氬倻顏遍梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢宄靶涘ù鍏肩懄鐎氬綊鏌ㄩ悢娲绘晭闁规彃顑嗙�氬綊鏌ㄩ悢鍛婄伄闁归貌ID闂佽法鍠庨～鏇炩枖閿燂拷
short PID_function04(void) //闂佽法鍠庨敓鐣屽枎閻ｆ儳顕ラ鍫熸櫢闁哄倶鍊栫�氬綊寮崼鏇熸櫢闁哄倶鍊栫�氾拷
{
	int nZ_Value;
	int tmp = 0;
	short sError,sError_test;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i = 1;;
	unsigned int  timeuse = 0;
	int iTimeLeft = 0;
	int chn_a,chn_b,chn_c,chn_d,t_b,t_b_test;
	int A,B,C,D;
	
	int b = 500;
	int a = 1000; 
	short f;
	
	nZ_Value = g_DA_z;
	//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
	
	chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
	chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
	chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
	chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
	
	//A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	//B = (chn_b - 32767) / 32767.0  * 10000; 
	//C = (chn_c - 32767) / 32767.0  * 10000;
	//D = (chn_d - 32767) / 32767.0  * 10000;
	
	//t_b = (float)(A + B - C - D) / (A + B + C + D) * 1000;
	t_b = -(float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
	sError = t_b - PID_setPoint;
	
	//DEBUG1("sError is %d\n",sError);
	//DEBUG1("g_point_time_left1 is %d\n",g_point_time_left);
	gettimeofday(&tpstart,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忓畡閭︽綏闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	do //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
    // 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱畟Error闂佽法鍠愰弸濠氬箯瀹勫府鎷烽悡搴＄悼闂佽法鍠愰弸濠氬箯瀹勭増鍊遍梺璺ㄥ枑閺嬪骞忕粚鏀�
    if(abs(sError) <= b)
    {
    	n_i = n_i + PID_Ki * sError;
    	DEBUG0("< B");
    }else if(abs(sError)>b&&abs(sError)<=(a+b))
    {
    	f = (a-abs(sError)+b)/a;
    	n_i = n_i + PID_Ki * sError * f;
    	DEBUG0("B < ERROR < A");
    }else 
    {
    	n_i = n_i;
    	DEBUG0("> A");
    }

		n_p = PID_Kp * sError;
		
		nZ_Value = nZ_Value + n_p + n_i; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎I
		
		DEBUG2("PID_Kp=%.3f, PID_Ki=%.3f\n", PID_Kp,PID_Ki);
		
		if(nZ_Value < 1) nZ_Value = 1; //闂佽法鍠栭崳楣冨箮濡炲墽涓查柟椋庡厴閺佹挾鎮板Δ浣告暰闁归鍏橀弫鎾绘煀閻㈡鏆滈柟椋庡厴閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閿燂拷
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		DEBUG1("Nz_value is %d\n",nZ_Value);
		//DEBUG1("g_point_time_left is %d\n",g_point_time_left);
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
    //DEBUG1("g_step_resp_time is %d\n",g_step_resp_time);
		
		//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
		chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
		chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
		chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
		chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
		
		A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	  B = (chn_b - 32767) / 32767.0  * 10000; 
	  C = (chn_c - 32767) / 32767.0  * 10000;
	  D = (chn_d - 32767) / 32767.0  * 10000;
	  
	  //DEBUG1("chn_a is %d\n",chn_a);
	  //DEBUG1("chn_b is %d\n",chn_b);
	  //DEBUG1("chn_c is %d\n",chn_c);
	  //DEBUG1("chn_d is %d\n",chn_d);
	
	  //t_b_test = (float)(A + B - C - D) / (A + B + C + D) * 1000;
		t_b = -(float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
		
		//DEBUG1("t_b_test is %d\n",t_b_test);
		//DEBUG1("t_b is %d\n",t_b);
		//DEBUG1("setPoint is %d\n",PID_setPoint);
		
		sError = t_b + PID_setPoint;//t_b濞戞挻妞介弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�+setpoint闂佽法鍠愰弸濠氬箯閻ゎ垳妲堥梺璺ㄥ枑閺嬪骞忛摎鍌滅Т闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱畟etpoint闁稿﹤銈搁弫鎾诲棘閵堝棗顏跺☉鎾存そ閺佹捇寮妶鍡楊伓闁稿鎷�
		//sError_test = (sError - 32767) / 32767.0  * 10000; 
		
		DEBUG1("sError is %d\n",sError);
		//DEBUG1("sError_test is %d\n",sError_test);
		
		gettimeofday(&tpend,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忛柨瀣靛壘闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	 	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	 	DEBUG1("timeuse is %d\n",timeuse);
    i++;
	}while(timeuse < g_point_time_left);
	
	g_DA_z = nZ_Value;
	return sError;
}
//闂佽法鍠曟俊顓犳媼鐟欏嫬顏堕梺璺ㄥ枑閺嬪骞忛柨瀣嗕線鏌ㄩ悢鍛婄伄闁归貌I闂佽法鍠庨～鏇炩枖閺囥垺鏅搁柡鍌樺�栫�氾拷13闂佽法鍠愰弸濠氬箯閿燂拷
short PID_function05(void) //闂佽法鍠庨敓鐣屽枎閻ｆ儳顕ラ鍫熸櫢闁哄倶鍊栫�氬綊寮崼鏇熸櫢闁哄倶鍊栫�氾拷
{
	int nZ_Value;
	int tmp = 0;
	short sError,sError_k,ec,sError_test;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i = 1;;
	unsigned int  timeuse = 0;
	int iTimeLeft = 0;
	int chn_a,chn_b,chn_c,chn_d,t_b,t_b_test;
	int A,B,C,D;
	
	nZ_Value = g_DA_z;
	//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
	
	chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
	chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
	chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
	chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
	
	//A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	//B = (chn_b - 32767) / 32767.0  * 10000; 
	//C = (chn_c - 32767) / 32767.0  * 10000;
	//D = (chn_d - 32767) / 32767.0  * 10000;
	
	//t_b = (float)(A + B - C - D) / (A + B + C + D) * 1000;
	t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
	sError = t_b + PID_setPoint;
	
	//DEBUG1("sError is %d\n",sError);
	//DEBUG1("g_point_time_left1 is %d\n",g_point_time_left);
	gettimeofday(&tpstart,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忓畡閭︽綏闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	do //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
    PID_Kp = fuzzy_kp13((float)sError/100, (float)ec/2);
    PID_Ki = fuzzy_ki13((float)sError/100, (float)ec/2);
    
		n_p = PID_Kp * sError;//闂佽法鍠愰弸濠氬箯缁岋拷-1闂佽法鍠曟繛鍥╁枈婢跺顏秂rror
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎I
		
		//DEBUG2("PID_Kp=%.3f, PID_Ki=%.3f\n", PID_Kp,PID_Ki);
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		//DEBUG1("Nz_value is %d\n",nZ_Value);
		//DEBUG1("g_point_time_left is %d\n",g_point_time_left);
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
		//DEBUG1("g_step_resp_time is %d\n",g_step_resp_time);

		//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
		chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
		chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
		chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
		chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
		
		A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	  B = (chn_b - 32767) / 32767.0  * 10000; 
	  C = (chn_c - 32767) / 32767.0  * 10000;
	  D = (chn_d - 32767) / 32767.0  * 10000;
	  
	  //DEBUG1("chn_a is %d\n",chn_a);
	  //DEBUG1("chn_b is %d\n",chn_b);
	  //DEBUG1("chn_c is %d\n",chn_c);
	  //DEBUG1("chn_d is %d\n",chn_d);
	
	  //t_b_test = (float)(A + B - C - D) / (A + B + C + D) * 1000;
		t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
		
		//DEBUG1("t_b_test is %d\n",t_b_test);
		//DEBUG1("t_b is %d\n",t_b);
		//DEBUG1("setPoint is %d\n",PID_setPoint);
		
		sError_k = t_b + PID_setPoint;//t_b濞戞挻妞介弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�+setpoint闂佽法鍠愰弸濠氬箯閻ゎ垳妲堥梺璺ㄥ枑閺嬪骞忛摎鍌滅Т闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱畟etpoint闁稿﹤銈搁弫鎾诲棘閵堝棗顏跺☉鎾存そ閺佹捇寮妶鍡楊伓闁稿鎷�
		//DEBUG1("sError_k is %d\n",sError_k);
		//DEBUG1("sError1 is %d\n",sError);
		ec = sError_k - sError;
		sError = t_b + PID_setPoint; //闂佽法鍠愰弸濠氬箯缁屾棃鏌ㄩ悢璇测枏缁炬澘顦扮�氱rror
		//DEBUG1("sError is %d\n",sError);
		//sError_test = (sError - 32767) / 32767.0  * 10000; 
		
		//DEBUG1("ec is %d\n",ec);

		gettimeofday(&tpend,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忛柨瀣靛壘闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	 	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	 	//DEBUG1("timeuse is %d\n",timeuse);
	 	i++;
	}while(timeuse < g_point_time_left);
	
	g_DA_z = nZ_Value;
	return sError;
}

//闂佽法鍠曟俊顓犳媼鐟欏嫬顏堕梺璺ㄥ枑閺嬪骞忛柨瀣嗕線鏌ㄩ悢鍛婄伄闁归貌I闂佽法鍠庨～鏇炩枖閺囥垺鏅搁柡鍌樺�栫�氾拷7闂佽法鍠愰弸濠氬箯閿燂拷
short PID_function06(void) //闂佽法鍠庨敓鐣屽枎閻ｆ儳顕ラ鍫熸櫢闁哄倶鍊栫�氬綊寮崼鏇熸櫢闁哄倶鍊栫�氾拷
{
	int nZ_Value;
	int tmp = 0;
	short sError,sError_k,ec,sError_test;
	int n_p = 0, n_i = 0;
	unsigned short uTmp, uTmp_Z;
	int i = 1;;
	unsigned int  timeuse = 0;
	int iTimeLeft = 0;
	int chn_a,chn_b,chn_c,chn_d,t_b,t_b_test;
	int A,B,C,D;
	
	nZ_Value = g_DA_z;
	//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
	
	chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
	chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
	chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
	chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
	
	//A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	//B = (chn_b - 32767) / 32767.0  * 10000; 
	//C = (chn_c - 32767) / 32767.0  * 10000;
	//D = (chn_d - 32767) / 32767.0  * 10000;
	
	//t_b = (float)(A + B - C - D) / (A + B + C + D) * 1000;
	t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
	sError = t_b + PID_setPoint;
	
	//DEBUG1("sError is %d\n",sError);
	//DEBUG1("g_point_time_left1 is %d\n",g_point_time_left);
	gettimeofday(&tpstart,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忓畡閭︽綏闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	do //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱懓顕ラ鍫熸櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
	{
    PID_Kp = fuzzy_kp07((float)sError/200, (float)ec/4);
    PID_Ki = fuzzy_ki07((float)sError/200, (float)ec/4);
    
		n_p = PID_Kp * sError;//闂佽法鍠愰弸濠氬箯缁岋拷-1闂佽法鍠曟繛鍥╁枈婢跺顏秂rror
		
		n_i = n_i + PID_Ki * sError;	
		
		nZ_Value = nZ_Value + n_p + n_i; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎I
		
		//DEBUG2("PID_Kp=%.3f, PID_Ki=%.3f\n", PID_Kp,PID_Ki);
		
		if(nZ_Value < 1) nZ_Value = 1;
		else if(nZ_Value > 65530)	nZ_Value = 65530;
			
		uTmp_Z = nZ_Value;
	
		write_ad669_by_channel( uTmp_Z, DA_Z_ADR ); //0~200V闂佽法鍠撶瑧闁伙繝顥撳▍銏ゅ箯閿燂拷 200V~闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒娑橆儔閺嗭拷
		
		//DEBUG1("Nz_value is %d\n",nZ_Value);
		//DEBUG1("g_point_time_left is %d\n",g_point_time_left);
		
		udelay(g_step_resp_time); //闂佽法鍠愰弸濠氬箯閻ゎ垳鈹堥梺璺ㄥ枑閺嬪骞忓畡鎵畨闂佽法鍠愮敮瀵哥驳鐟欏嫬顏�
		//DEBUG1("g_step_resp_time is %d\n",g_step_resp_time);

		//sError = read_ad_times_byCHN(gi_ad_sample_times,2) - PID_setPoint;
		chn_a = read_ad_times_byCHN(gi_ad_sample_times,0);
		chn_b = read_ad_times_byCHN(gi_ad_sample_times,1);
		chn_c = read_ad_times_byCHN(gi_ad_sample_times,2);
		chn_d = read_ad_times_byCHN(gi_ad_sample_times,3);
		
		A = (chn_a - 32767) / 32767.0  * 10000; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氳绋夌花鍍�
	  B = (chn_b - 32767) / 32767.0  * 10000; 
	  C = (chn_c - 32767) / 32767.0  * 10000;
	  D = (chn_d - 32767) / 32767.0  * 10000;
	  
	  //DEBUG1("chn_a is %d\n",chn_a);
	  //DEBUG1("chn_b is %d\n",chn_b);
	  //DEBUG1("chn_c is %d\n",chn_c);
	  //DEBUG1("chn_d is %d\n",chn_d);
	
	  //t_b_test = (float)(A + B - C - D) / (A + B + C + D) * 1000;
		t_b = (float)(chn_a + chn_b - chn_c - chn_d) / (chn_a + chn_b + chn_c + chn_d) * 100000;
		
		//DEBUG1("t_b_test is %d\n",t_b_test);
		//DEBUG1("t_b is %d\n",t_b);
		//DEBUG1("setPoint is %d\n",PID_setPoint);
		
		sError_k = t_b + PID_setPoint;//t_b濞戞挻妞介弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�+setpoint闂佽法鍠愰弸濠氬箯閻ゎ垳妲堥梺璺ㄥ枑閺嬪骞忛摎鍌滅Т闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱畟etpoint闁稿﹤銈搁弫鎾诲棘閵堝棗顏跺☉鎾存そ閺佹捇寮妶鍡楊伓闁稿鎷�
		//DEBUG1("sError_k is %d\n",sError_k);
		//DEBUG1("sError1 is %d\n",sError);
		ec = sError_k - sError;
		sError = t_b + PID_setPoint; //闂佽法鍠愰弸濠氬箯缁屾棃鏌ㄩ悢璇测枏缁炬澘顦扮�氱rror
		DEBUG1("sError is %d\n",sError);
		//sError_test = (sError - 32767) / 32767.0  * 10000; 
		
		//DEBUG1("ec is %d\n",ec);

		gettimeofday(&tpend,NULL); //闂佽法鍠愰弸濠氬箯瀹勬澘绲块梺璺ㄥ枑閺嬪骞忛柨瀣靛壘闁哄啫鐖奸弫鎾诲棘閵堝棗顏�
	 	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	 	//DEBUG1("timeuse is %d\n",timeuse);
	 	i++;
	}while(timeuse < g_point_time_left);
	
	g_DA_z = nZ_Value;
	return sError;
}

/***********************************************
         闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲级鐢喚绉堕柟鐑芥敱鑶╅梺璺ㄥ枑閺嬪骞忕粙鐧怐闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓈p闂佽法鍠嶉懠搴ｆ兜闁垮顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
************************************************/ 
float   (float e, float ec)				//e,ec闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬湱绮堝ú顏呮櫢闁哄倶鍊栫�氬綊鎯堝▎鎯у⒕闁归鍏橀弫鎾诲棘閵堝棗顏舵繛瀵稿閸ㄤ粙骞忛悜鑺ユ櫢闁跨噦鎷�
{													  
	float Kp_calcu;
	unsigned char num,pe,pec; 
	float eRule[13]={-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0};   //闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒鍗炪偢閺佹捇寮妶鍡楊伓婵☆垽绻濋弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷
	float ecRule[13]={-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0}; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归膿C闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
	float eFuzzy[2]={0.0,0.0};				    //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枔闁卞洭鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁轰焦鐟ㄩ鎰板箯閿燂拷
	float ecFuzzy[2]={0.0,0.0};            //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閸ㄦ繈宕犻弽顓熸櫢闁哄倶鍊栫�氱C闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲极濞嗘帩鍟囬柟鍑ゆ嫹
	float kpRule[13]={0.0,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55,0.60};			//Kp闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲箳閵壯侊拷瀣箯閿燂拷
	
	/*********
	闂佽法鍠愰崺鍛村箟鐎ｎ偄顏舵繛澶堝姂閺佹挾鎲撮敐蹇曠獥
	闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾绘儍閸℃瑣锟藉骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍝勫祮缁炬澘顦扮�氬湱浜稿鍗睷ule闂佽法鍠愰弸濠氬箯瀹勬澘绲块柛濠傘偢閺佹捇寮妶鍡楊伓闁搞儲鎸抽弫鎾诲棘閵堝棗顏�
	闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柣锝呯焿缁辩櫤pRule[13]={0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.10,0.11,0.12}
	*********/
	
	float KpFuzzy[13]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};				//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归菒p闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲极濞嗘帩鍟囬柟鍑ゆ嫹
	int KpRule[13][13]=					  		//Kp闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枔鐎氭盯骞夌�ｎ偄顏�
	{
		12,12,12,11,10,10,10,9,8,7,6,6,6,
		12,12,12,11,10,9,9,9,8,7,6,5,5,
		12,12,12,11,10,9,8,8,8,7,6,5,4,
		11,11,11,11,10,9,8,7,7,6,5,5,4,
		10,10,10,10,10,9,8,7,6,5,4,4,4,
		10,10,10,9,9,8,7,6,5,4,3,3,3,
	  10,10,10,9,8,7,6,5,4,3,2,2,2,
    9,9,9,8,7,6,5,5,4,3,2,2,2,
    8,8,8,7,6,5,4,4,4,3,2,2,2,
    8,7,7,6,5,4,3,3,3,3,2,1,1,
    8,7,6,5,4,3,2,2,2,2,2,1,0,
    7,7,6,4,3,3,2,2,2,1,1,1,0,
    6,6,6,4,2,2,2,2,2,1,0,0,0,  
 	}; 
 	//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氾拷4Hz闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓎=0.1闂佽法鍠愰弸濠氬箯缁嬶拷=0.1闂佽法鍠愰弸濠氬箯閿燂拷4~30Hz闂佽法鍠愰弸濠氬箯缁嬶拷=0.025*linerate
  if (linerate < 31 && linerate > 4)
 	{
 		int i;
 		for(i = 0; i < 13; i++)
 		{
 		  kpRule[i] = kpRule[i]*(linerate/4.00);
 	  }
 	}
 	
   /*****闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒鍗炪偢閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�*****/
   if(e<eRule[0])							
	{
		eFuzzy[0] =1.0; 
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/2;
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/2;
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/2;
		pe = 2;
	}
    else if(eRule[3]<=e && e<eRule[4])
    {   eFuzzy[0] = (eRule[4]-e)/2;
        pe = 3;
    }
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/2;
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/2;
		pe = 5;
	}
	else if (eRule[6]<=e && e<eRule[7])
	{
		eFuzzy[0] = (eRule[7]-e)/2;
		pe = 6;
	}
	else if (eRule[7]<=e && e<eRule[8])
	{
		eFuzzy[0] = (eRule[8]-e)/2;
		pe = 7;
	}
	else if (eRule[8]<=e && e<eRule[9])
	{
		eFuzzy[0] = (eRule[9]-e)/2;
		pe = 8;
	}
	else if (eRule[9]<=e && e<eRule[10])
	{
		eFuzzy[0] = (eRule[10]-e)/2;
		pe = 9;
	}
	else if (eRule[10]<=e && e<eRule[11])
	{
		eFuzzy[0] = (eRule[11]-e)/2;
		pe = 10;
	}
	else if (eRule[11]<=e && e<eRule[12])
	{
		eFuzzy[0] = (eRule[12]-e)/2;
		pe = 11;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =11;
	}
    eFuzzy[1] =1.0 - eFuzzy[0];
	/*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归膿C闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷*****/
	if(ec<ecRule[0])							
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/2;
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/2;
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/2;
		pec = 2 ;
	}
    else if(ecRule[3]<=ec && ec<ecRule[4])
    {   ecFuzzy[0] = (ecRule[4]-ec)/2;
        pec=3;
    }
	else if(ecRule[4]<=ec && ec<ecRule[5])
    {   ecFuzzy[0] = (ecRule[5]-ec)/2;
        pec=4;
    }
	else if(ecRule[5]<=ec && ec<ecRule[6])
    {   ecFuzzy[0] = (ecRule[6]-ec)/2;
        pec=5;
    }
	else if(ecRule[6]<=ec && ec<ecRule[7])
    {   ecFuzzy[0] = (ecRule[7]-ec)/2;
	    pec=6;
    }
	else if(ecRule[7]<=ec && ec<ecRule[8])
    {   ecFuzzy[0] = (ecRule[8]-ec)/2;
	    pec=7;
    }
	else if(ecRule[8]<=ec && ec<ecRule[9])
    {   ecFuzzy[0] = (ecRule[9]-ec)/2;
	    pec=8;
    }
	else if(ecRule[9]<=ec && ec<ecRule[10])
    {   ecFuzzy[0] = (ecRule[10]-ec)/2;
	    pec=9;
    }
	else if(ecRule[10]<=ec && ec<ecRule[11])
    {   ecFuzzy[0] = (ecRule[11]-ec)/2;
	    pec=10;
    }
	else if(ecRule[11]<=ec && ec<ecRule[12])
    {   ecFuzzy[0] = (ecRule[12]-ec)/2;
	    pec=11;
    }
	else
	{
		ecFuzzy[0] =0.0;
		pec = 11;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0]; 

	/*********闂佽法鍠愰弸濠氬箯閻ゎ垼鍤勬俊顖ょ節閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悤鍌涘*********/
	num =KpRule[pe][pec];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KpRule[pe][pec+1];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[1];	
	num =KpRule[pe+1][pec];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KpRule[pe+1][pec+1];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/*********闂佽法鍠愰弸濠氬箯闁垮缍�妤犵偞濞婇弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氱懓螣閿熺姵鏅搁柡鍌樺�栫�氾拷*********/
  Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3]+KpFuzzy[4]*kpRule[4]+KpFuzzy[5]*kpRule[5]
  +KpFuzzy[6]*kpRule[6]+KpFuzzy[7]*kpRule[7]+KpFuzzy[8]*kpRule[8]+KpFuzzy[9]*kpRule[9]+KpFuzzy[10]*kpRule[10]+KpFuzzy[11]*kpRule[11]
  +KpFuzzy[12]*kpRule[12];
  return Kp_calcu;
}
/***********************************************
         闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲级鐢喚绉堕柟鐑芥敱鑶╅梺璺ㄥ枑閺嬪骞忕粙鐧怐闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓈i闂佽法鍠嶉懠搴ｆ兜闁垮顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
************************************************/ 
float fuzzy_ki13(float e, float ec)				        
{
	float Ki_calcu;
	unsigned char num,pe,pec;
	float eRule[13]={-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0};	
	float ecRule[13]={-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0};
	float eFuzzy[2]={0.0,0.0};					
	float ecFuzzy[2]={0.0,0.0};            
	float kiRule[13]={0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.11,0.12};		    
	float KiFuzzy[13]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};			
	int KiRule[13][13]=					       
	{
		0,0,0,1,2,2,2,3,4,5,6,6,6,
		0,0,0,1,2,3,3,3,4,5,6,6,6,
		0,0,0,1,2,3,4,4,4,5,6,6,6,
		0,1,1,2,3,3,4,5,5,6,7,7,7,
		0,1,2,3,4,4,4,5,6,7,8,8,8,
		1,1,2,3,4,5,5,6,7,8,9,9,9,
		2,2,2,3,4,5,6,7,8,9,10,10,10,
		2,3,3,4,5,6,7,7,8,9,10,11,11,
		2,3,4,5,6,7,8,8,8,9,10,11,12,
		4,4,5,6,7,7,8,9,9,10,11,11,12,
    6,6,6,7,8,8,8,9,10,11,12,12,12,
    6,6,6,7,8,9,9,9,10,11,12,12,12,
    6,6,6,7,8,9,10,10,10,11,12,12,12,
	};
   /*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾绘晸閿燂拷*****/
  if(e<eRule[0])							
	{
		eFuzzy[0] =1.0; 
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/2;
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/2;
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/2;
		pe = 2;
	}
    else if(eRule[3]<=e && e<eRule[4])
    {   eFuzzy[0] = (eRule[4]-e)/2;
        pe = 3;
    }
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/2;
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/2;
		pe = 5;
	}
	else if(eRule[6]<=e && e<eRule[7])
	{
		eFuzzy[0] = (eRule[7]-e)/2;
		pe = 6;
	}
	else if(eRule[7]<=e && e<eRule[8])
	{
		eFuzzy[0] = (eRule[8]-e)/2;
		pe = 7;
	}
	else if(eRule[8]<=e && e<eRule[9])
	{
		eFuzzy[0] = (eRule[9]-e)/2;
		pe = 8;
	}
	else if(eRule[9]<=e && e<eRule[10])
	{
		eFuzzy[0] = (eRule[10]-e)/2;
		pe = 9;
	}
	else if(eRule[10]<=e && e<eRule[11])
	{
		eFuzzy[0] = (eRule[11]-e)/2;
		pe = 10;
	}
	else if(eRule[11]<=e && e<eRule[12])
	{
		eFuzzy[0] = (eRule[12]-e)/2;
		pe = 11;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =11;
	}
    eFuzzy[1] =1.0 - eFuzzy[0];
	/*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�*****/
	if(ec<ecRule[0])							
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/2;
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/2;
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/2;
		pec = 2 ;
	}
    else if(ecRule[3]<=ec && ec<ecRule[4])
    {   ecFuzzy[0] = (ecRule[4]-ec)/2;
        pec=3;
    }
	else if(ecRule[4]<=ec && ec<ecRule[5])
    {   ecFuzzy[0] = (ecRule[5]-ec)/2;
        pec=4;
    }
	else if(ecRule[5]<=ec && ec<ecRule[6])
    {   ecFuzzy[0] = (ecRule[6]-ec)/2;
        pec=5;
    }
	else if(ecRule[6]<=ec && ec<ecRule[7])
    {   ecFuzzy[0] = (ecRule[7]-ec)/2;
    	pec=6;
    }
	else if(ecRule[7]<=ec && ec<ecRule[8])
    {   ecFuzzy[0] = (ecRule[8]-ec)/2;
	    pec=7;
    }
	else if(ecRule[8]<=ec && ec<ecRule[9])
    {   ecFuzzy[0] = (ecRule[9]-ec)/2;
	    pec=8;
    }
	else if(ecRule[9]<=ec && ec<ecRule[10])
    {   ecFuzzy[0] = (ecRule[10]-ec)/2;
	    pec=9;
    }
	else if(ecRule[10]<=ec && ec<ecRule[11])
    {   ecFuzzy[0] = (ecRule[11]-ec)/2;
    	pec=10;
    }
	else if(ecRule[11]<=ec && ec<ecRule[12])
    {   ecFuzzy[0] = (ecRule[12]-ec)/2;
	    pec=11;
    }
	else
	{
		ecFuzzy[0] =0.0;
		pec = 11;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0]; 
	/***********闂佽法鍠愰弸濠氬箯閻ゎ垼鍤勬俊顖ょ節閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悤鍌涘***************/
	num =KiRule[pe][pec];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KiRule[pe][pec+1];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[1];	
	num =KiRule[pe+1][pec];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KiRule[pe+1][pec+1];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/********闂佽法鍠愰弸濠氬箯闁垮缍�妤犵偞濞婇弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氱懓螣閿熺姵鏅搁柡鍌樺�栫�氾拷********/
	Ki_calcu=KiFuzzy[0]*kiRule[0]+KiFuzzy[1]*kiRule[1]+KiFuzzy[2]*kiRule[2]+KiFuzzy[3]*kiRule[3]+KiFuzzy[4]*kiRule[4]+KiFuzzy[5]*kiRule[5]
	+KiFuzzy[6]*kiRule[6]+KiFuzzy[7]*kiRule[7]+KiFuzzy[8]*kiRule[8]+KiFuzzy[9]*kiRule[9]+KiFuzzy[10]*kiRule[10]+KiFuzzy[11]*kiRule[11]
	+KiFuzzy[12]*kiRule[12]; 
  return Ki_calcu;
}

/***********************************************
         闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲级鐢喚绉堕柟鐑芥敱鑶╅梺璺ㄥ枑閺嬪骞忕粙鐧怐闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓈p闂佽法鍠嶉懠搴ｆ兜闁垮顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
************************************************/ 
float fuzzy_kp07(float e, float ec)				//e,ec闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬湱绮堝ú顏呮櫢闁哄倶鍊栫�氬綊鎯堝▎鎯у⒕闁归鍏橀弫鎾诲棘閵堝棗顏舵繛瀵稿閸ㄤ粙骞忛悜鑺ユ櫢闁跨噦鎷�
{													  
	float Kp_calcu;
	unsigned char num,pe,pec; 
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};   //闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒鍗炪偢閺佹捇寮妶鍡楊伓婵☆垽绻濋弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归膿C闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
	float eFuzzy[2]={0.0,0.0};				    //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枔闁卞洭鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁轰焦鐟ㄩ鎰板箯閿燂拷
	float ecFuzzy[2]={0.0,0.0};            //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閸ㄦ繈宕犻弽顓熸櫢闁哄倶鍊栫�氱C闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲极濞嗘帩鍟囬柟鍑ゆ嫹
	float kpRule[4]={0.0,0.2,0.4,0.6};			//Kp闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲箳閵壯侊拷瀣箯閿燂拷
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};				//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归菒p闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲极濞嗘帩鍟囬柟鍑ゆ嫹
	int KpRule[7][7]=					  		//Kp闂佽法鍠愰弸濠氬箯闁垮瑔渚�鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枔鐎氭盯骞夌�ｎ偄顏�
	{
		6,6,5,5,4,3,3,
		6,6,5,4,4,3,2,
		5,5,5,4,3,2,2,
		5,5,4,3,2,1,1,
		4,4,3,2,2,1,1,
		4,3,2,1,1,1,0,
		3,3,2,2,2,1,1
	}; 
   /*****闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒鍗炪偢閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�*****/
   if(e<eRule[0])							
	{
		eFuzzy[0] =1.0; 
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]);
		pe = 2;
	}
    else if(eRule[3]<=e && e<eRule[4])
    {   eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]);
        pe = 3;
    }
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]);
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =6;
	}
    eFuzzy[1] =1.0 - eFuzzy[0];
	/*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归膿C闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷*****/
	if(ec<ecRule[0])							
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]);
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]);
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]);
		pec = 2 ;
	}
    else if(ecRule[3]<=ec && ec<ecRule[4])
    {   ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
        pec=3;
    }
	else if(ecRule[4]<=ec && ec<ecRule[5])
    {   ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
        pec=4;
    }
	else if(ecRule[5]<=ec && ec<ecRule[6])
    {   ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
        pec=5;
    }
	else
	{
		ecFuzzy[0] =0.0;
		pec = 6;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0]; 
	/*********闂佽法鍠愰弸濠氬箯閻ゎ垼鍤勬俊顖ょ節閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悤鍌涘*********/
	num =KpRule[pe][pec];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KpRule[pe][pec+1];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[1];	
	num =KpRule[pe+1][pec];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KpRule[pe+1][pec+1];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/*********闂佽法鍠愰弸濠氬箯闁垮缍�妤犵偞濞婇弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氱懓螣閿熺姵鏅搁柡鍌樺�栫�氾拷*********/
  Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
  return Kp_calcu;
}
/***********************************************
         闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲级鐢喚绉堕柟鐑芥敱鑶╅梺璺ㄥ枑閺嬪骞忕粙鐧怐闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氱瓈i闂佽法鍠嶉懠搴ｆ兜闁垮顏堕梺璺ㄥ枑閺嬪骞忛敓锟�
************************************************/ 
float fuzzy_ki07(float e, float ec)				        
{
	float Ki_calcu;
	unsigned char num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};	
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};
	float eFuzzy[2]={0.0,0.0};					
	float ecFuzzy[2]={0.0,0.0};            
	float kiRule[4]={0.00,0.04,0.08,0.12};		    
	float KiFuzzy[4]={0.0,0.0,0.0,0.0};			
	int KiRule[7][7]=					       
	{
		0,0,1,1,2,3,3,
		0,0,1,2,2,3,3,
		0,1,2,2,3,4,4,
		1,1,2,3,4,5,5,
		1,2,3,4,4,5,6,
		3,3,4,4,5,6,6,
		3,3,4,5,5,6,6
	};
   /*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾绘晸閿燂拷*****/
  if(e<eRule[0])							
	{
		eFuzzy[0] =1.0; 
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]);
		pe = 2;
	}
    else if(eRule[3]<=e && e<eRule[4])
    {   eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]);
        pe = 3;
    }
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]);
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =5;
	}
    eFuzzy[1] =1.0 - eFuzzy[0];
	/*****闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柟瀛樺笒鐎垫煡鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�*****/
	if(ec<ecRule[0])							
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]);
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]);
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]);
		pec = 2 ;
	}
    else if(ecRule[3]<=ec && ec<ecRule[4])
    {   ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
        pec=3;
    }
	else if(ecRule[4]<=ec && ec<ecRule[5])
    {   ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
        pec=4;
    }
	else if(ecRule[5]<=ec && ec<ecRule[6])
    {   ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
        pec=5;
    }
	else
	{
		ecFuzzy[0] =0.0;
		pec = 6;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0]; 
	/***********闂佽法鍠愰弸濠氬箯閻ゎ垼鍤勬俊顖ょ節閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悤鍌涘***************/
	num =KiRule[pe][pec];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KiRule[pe][pec+1];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[1];	
	num =KiRule[pe+1][pec];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KiRule[pe+1][pec+1];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/********闂佽法鍠愰弸濠氬箯闁垮缍�妤犵偞濞婇弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氱懓螣閿熺姵鏅搁柡鍌樺�栫�氾拷********/
	Ki_calcu=KiFuzzy[0]*kiRule[0]+KiFuzzy[1]*kiRule[1]+KiFuzzy[2]*kiRule[2]+KiFuzzy[3]*kiRule[3]; 
  return Ki_calcu;
}

/**************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾搭殰閻氬绉堕柟椋庡厴閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁弶鐐插皡缂嶅洭骞忛柨瀣嗕線鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氾拷***************/
/*
float uf(float x,float a,float b,float c)
{
if(x<=a)
return 0;
else if((a<x)&&(x<=b))
return (x-a)/(b-a);
else if((b<x)&&(x<=c))
return (c-x)/(c-b);
else if(x>c)
return 0;
}*/
/****************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢璇测枏闁告垯鍊栫�氱懓螣閿熺姵鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�**********************/
/*
float cuf(float x,float a,float b,float c)
{ 
float y,z;
z=(b-a)*x+a;
y=c-(c-b)*x;
return (y+z)/2;
}*/
/*****************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氾拷(闂佽法鍠愰弸濠氬箯閿燂拷)闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏� 婵☆垽绻濋弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�*******************/
/*
float ufl(float x,float a,float b)
{
if(x<=a)  
return 1;
else if((a<x)&&(x<=b))
return (b-x)/(b-a);
else if(x>b)
return 0;
}*/
/*******************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁弶鐐茬仢閸ゆ牠骞忛柨瀣嗕線鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�***********************/
/*
float cufl(float x,float a,float b)
{
return b-(b-a)*x;
}*/


/*****************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氾拷(闂佽法鍠愰弸濠氬箯閿燂拷)闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺�栫�氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏� 婵☆垽绻濋弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛敓锟�*******************/
/*
float ufr(float x,float a,float b)
{
if(x<=a)
return 0;
if((a<x)&&(x<b))
return (x-a)/(b-a);
if(x>=b)
return 1;
}*/
/*******************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁弶鐐茬仢閸ゆ牠骞忛柨瀣嗕線鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�***********************/
/*
float cufr(float x,float a,float b)
{
return (b-a)*x +a;
}*/
/*******************闂佽法鍠曢〃銈嗙閵堝洢锟藉骞忛敓锟�***********************/
/*
float fand(float a,float b)
{
return (a<b)?a:b;
}*/
/*******************闂佽法鍠曢〃銈夌嵁閸撲降锟藉骞忛敓锟�***********************/
/*
float forr(float a,float b)
{
return (a<b)?b:a;
}*/

void* feedbackServoThread(void* para)
{
	pthread_detach(pthread_self());//闂佽法鍠愰弸濠氬箯閾氬倽绀嬮梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫�氬綊鏌ㄩ悢渚痪缂佹稖顫夌�氾拷
	
	DEBUG0("Enter the feedbackServoThread!\n");
	
	while(1)
	{
		PID_function03();
		
		if(g_current_task != CMD_FEEDBACK_SWITCH)	break;
		if(gb_feedback_on == 0) break;
	}
	
	DEBUG0("Leave the feedbackServoThread!\n");
}


