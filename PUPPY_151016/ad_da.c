#include "hardware.h"
#include "afm_comm.h"
#include <sys/time.h>
#include <stdio.h>

int Ad676SampleTime = 40; //us

extern int gi_ad_sample_time;
extern int gi_ad_sample_times;

struct timeval tpstart,tpend;

/*直接读取AD*/
short read_ad() 
{	
	outb(0xFF,AD7523_AMP_Adr); //满增益
	outb(1,AD_SAMPLE_Adr); 
	outb(0,AD_SAMPLE_Adr); //启动AD转换
	while(inb(AD_BUSY_Adr) & 0x1); //等待AD复位忙
	udelay(10);
	return inw(AD_RD_Adr) & 0xffff; 
}

unsigned short read_ad_times(int para) //只读出了1通道的信息
{	
	int i = 0;
	unsigned long sum = 0;
	unsigned short avarage = 0;
	
	if(para == 0) para = 1;
	for(i = 0; i < para; i++)
	{
		outb(0,U1234_WR_Adr); //启动AD转换
		while(!(inb(U4_BUSY_Adr) & 0x1));
		avarage = inw(U1_RD_Adr) & 0xffff;
		avarage = inw(U1_RD_Adr) & 0xffff;
		
		sum += avarage;
	}
	avarage = sum / para;
	return avarage; 
}

unsigned short *read_ad_times_4CHN(int para, int n)
{
  int i;
  unsigned short AD_sum[10] = {0,0,0,0,0,0,0,0,0,0};
  static unsigned long AD_avarage[10] = {0,0,0,0,0,0,0,0,0,0};

  //unsigned int timeuse = 0;

  if(para == 0) para = 1;

  //DEBUG1("ad times is %d\n",para);
  //gettimeofday(&tpstart,NULL);
  for(i = 0; i < para; i++)
    {
      out(0,U1234_WR_Adr); //启动AD转换
      while(!(inb(U4_BUSY_Adr)) & 0x1);//判断AD是否BUSY

      AD_avarage[0] = inw(U1_RD_Adr) & 0xffff; //对每一路采样信号进行读取
      AD_avarage[0] = inw(U1_RD_Adr) & 0xffff;
      //DEBUG1("case0 is %d\n",avarage[0]);
      AD_sum[0] += AD_avarage[0];

      AD_avarage[1] = inw(U2_RD_Adr) & 0xffff;
      //AD_avarage[1] = inw(U2_RD_Adr) & 0xffff;
      //DEBUG1("case1 is %d\n",avarage[1]);
      AD_sum[1] += AD_avarage[1];

      AD_avarage[2] = inw(U3_RD_Adr) & 0xffff;
      //AD_avarage[2] = inw(U3_RD_Adr) & 0xffff;
      //DEBUG1("case2 is %d\n",avarage[2]);
      AD_sum[2] += AD_avarage[2];

      AD_avarage[3] = inw(U4_RD_Adr) & 0xffff;
     // AD_avarage[3] = inw(U4_RD_Adr) & 0xffff;
      //DEBUG1("case3 is %d\n",avarage[3]);
      AD_sum[3] += AD_avarage[3];
    }
  //gettimeofday(&tpend,NULL);
  //timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+
  //            tpend.tv_usec-tpstart.tv_usec; //us
  //DEBUG1("ad timeuse is %d\n",timeuse);//打印AD采样时间

  for(i = 0; i < n;i++)
    AD_avarage[i] += AD_sum[i] / para;
  return AD_avarage;

}
unsigned short read_ad_times_byCHN(int para)
{
	int i = 0;
	unsigned long sum = 0;
	unsigned short avarage = 0;

	//unsigned int  timeuse = 0;

	if(para == 0) para = 1;

	//DEBUG1("ad times is %d\n",para);
	//gettimeofday(&tpstart,NULL); //获取起始时间

	for(i = 0; i < para; i++)
	{
		outb(0,U1234_WR_Adr); //启动AD转换
		while(!(inb(U4_BUSY_Adr) & 0x1));
		switch(chn)
		{
			case 0:
				avarage = inw(U1_RD_Adr) & 0xffff;
				avarage = inw(U1_RD_Adr) & 0xffff;
				//DEBUG1("case0 %d\n", avarage);
				break;
			case 1:
				avarage = inw(U2_RD_Adr) & 0xffff;
				avarage = inw(U2_RD_Adr) & 0xffff;
				//DEBUG1("case1 %d\n", avarage);
				break;
			case 2:
				avarage = inw(U3_RD_Adr) & 0xffff;
				avarage = inw(U3_RD_Adr) & 0xffff;
				//DEBUG1("case2 %d\n", avarage);
				break;
			case 3:
				avarage = inw(U4_RD_Adr) & 0xffff;
				avarage = inw(U4_RD_Adr) & 0xffff;
				//DEBUG1("case3 %d\n", avarage);
				break;
		}

		sum += avarage;
	}
	//gettimeofday(&tpend,NULL); //获取终止时间
	//timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	//DEBUG1("ad timeuse is %d\n",timeuse);
	avarage = sum / para;
	return avarage;
}

/*设置通道后读取AD*/
/*para1=channel para2=times */
short read_ad_by_channel(unsigned short para1,unsigned short para2) 
{	
	int i = 0;
	int sum = 0;
	short avarage = 0;
	outb(para1,K_A_Adr); //设置通道
	outb(0xFF,AD7523_AMP_Adr); //满增益	
	
	//模拟开关切换太快会带来系统噪声 一般需延迟1ms左右
	mdelay(5);
	
	for(i = 0; i < para2; i++)
	{
		outb(1,AD_SAMPLE_Adr); 
		outb(0,AD_SAMPLE_Adr); //启动AD转换
		while(inb(AD_BUSY_Adr) & 0x1); //等待AD复位忙
		udelay(10);
		sum += inw(AD_RD_Adr) & 0xffff;
	}
	avarage = sum / para2;
	return avarage; 
}

/*设置AD通道*/
//模拟开关切换太快会带来系统噪声 一般需延迟1ms左右
void ad_channel_sel(unsigned short para) 
{
	outb(para,K_A_Adr);
	mdelay(1);
}

/*设置带通道参数的DA*/
void write_dac7744_by_channel(unsigned short para1,unsigned short para2) //para1 = data , para2 = channel
{
	outw(para1,para2);
	outb(0x0,DA_LOAD_ADR); //load data to reg
	outb(0x1,DA_LOAD_ADR); //hold data of reg
	outb(0x0,DA_LOAD_ADR);
}

void write_ad669_by_channel(unsigned short para1,unsigned short para2) //para1 = data , para2 = channel
{
	outw(para1, para2);
}

void read_ad676_chip1(short* val_1) 
{	
	float fTmp = 0;
	outb(1,AD_SAMPLE_Adr); 
	outb(0,AD_SAMPLE_Adr); //启动AD转换
	udelay(Ad676SampleTime);
	*val_1 = inw(AD_U2_RD_Adr) & 0xffff; 	
}

void read_ad676_chip2(short* val_2) 
{	
	float fTmp = 0;
	outb(1,AD_SAMPLE_Adr); 
	outb(0,AD_SAMPLE_Adr); //启动AD转换
	udelay(Ad676SampleTime);
	*val_2 = inw(AD_U3_RD_Adr) & 0xffff; 	
}

void read_ad676_single_by_chn(short* val, char chn, char chip) 
{	
	float fTmp = 0;
	
	switch (chip)
	{
		case 1:
			outb(chn,K_U2_Adr);
			mdelay(20);
			outb(1,AD_SAMPLE_Adr); 
			outb(0,AD_SAMPLE_Adr); //启动AD转换
			udelay(Ad676SampleTime);
			*val = inw(AD_U2_RD_Adr) & 0xffff; 	
			break;
		case 2:
			outb(chn,K_U3_Adr);
			mdelay(20);
			outb(1,AD_SAMPLE_Adr); 
			outb(0,AD_SAMPLE_Adr); //启动AD转换
			udelay(Ad676SampleTime);
			*val = inw(AD_U3_RD_Adr) & 0xffff; 	
			break;
		default:
			break;
	} 
}

void read_ad_time_test()
{
	struct timeval tpstart,tpend;
	int i = 0;
	unsigned int  timeuse = 0;
	float fTime = 0;
	unsigned short us_tmp = 0;
	gettimeofday(&tpstart,NULL); //获取起始时间

	for(i = 0; i < 1000; i++)
	{
	outb(0,U1234_WR_Adr); //启动AD转换
	while(!(inb(U4_BUSY_Adr) & 0x1));
	us_tmp = inw(U1_RD_Adr) & 0xffff;
	us_tmp = inw(U1_RD_Adr) & 0xffff;
	}
	
	gettimeofday(&tpend,NULL); //获取终止时间
	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	DEBUG1("AD sample 1000 times takes %d ms\n",timeuse / 1000);	
}

unsigned short read_ad_times_byCHN0(int para) 
{	
	int i = 0;
	unsigned long sum = 0;
	unsigned short avarage = 0;
	
	unsigned int  timeuse = 0;
	
	if(para == 0) para = 1;
		
	//DEBUG1("ad times is %d\n",para);
	gettimeofday(&tpstart,NULL); //获取起始时间

	for(i = 0; i < para; i++)
	{
		outb(0,U1234_WR_Adr); //启动AD转换
		while(!(inb(U4_BUSY_Adr) & 0x1));
	
		avarage = inw(U1_RD_Adr) & 0xffff;
		avarage = inw(U1_RD_Adr) & 0xffff;
		
		sum += avarage;
	}
	gettimeofday(&tpend,NULL); //获取终止时间
	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ 
	 					tpend.tv_usec-tpstart.tv_usec; //us Level
	DEBUG1("ad timeuse is %d\n",timeuse);
	avarage = sum / para;
	return avarage; 
}

