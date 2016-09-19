#ifndef AFM_COMM
#define AFM_COMM

#include <stdio.h>

#define VERSION "0.0.1"
#define AUTHOR "Chen Daixie"
#define LAST_REVISED "2008.12.12"

#define true 0
#define false 1
#define PI 3.14159265358979
#define SERVER_ADDR "172.16.14.119"
#define SERVER_PORT 7000

#define MAX_LOOP 30
#define ERROR_EPS 20
#define DA_DELAY 100  //us
#define AD_SAMPLE_TIMES 1 //times used for test
#define STEP_RESPONSE_DELAY 350 //us

#define DOTS_NUM 1024
#define DOTS_NUM_PLUS_CMD 2054 //���
//#define DOTS_NUM_PLUS_CMD 1030

typedef struct command
{
	unsigned short cmd;//������
	unsigned short para1;//���� 1
	unsigned short para2;//���� 2
	unsigned short para3;//���� 3
	unsigned short para4;//���� 4
	unsigned short para5;//���� 5
} COMMAND;//�ܹ�12���ֽ�

typedef struct response
{
	unsigned short cmd;//������
	short para1;//���� 1
	short para2;//���� 2
	short para3;//���� 3
	short para4;//���� 4
	short para5;//���� 5
} RESPONSE;//�ܹ�12���ֽ�

typedef struct response_u
{
	unsigned short cmd;//������
	unsigned short para1;//���� 1
	unsigned short para2;//���� 2
	unsigned short para3;//���� 3
	unsigned short para4;//���� 4
	unsigned short para5;//���� 5
} RESPONSE_U;//�ܹ�12���ֽ�

#define MOTOR_STEP_FORWARD_PULSE	0x19
#define MOTOR_STEP_BACKWARD_PULSE 	0x1A

#define MOTOR_STEP_FORWARD	0x1F
#define MOTOR_STEP_BACKWARD 	0x1E
#define MOTOR_AUTO_FORWARD	0x1D
#define MOTOR_AUTO_BACKWARD	0x17
#define MOTOR_GET_STEPS	0x1C
#define MOTOR_FAST_FORWARD	0x2C
#define MOTOR_FAST_BACKWARD 	0x2E
#define MOTOR_STOP		0x16
#define MOTOR_SPEED		0x18

#define LASER_ON		0x40
#define LASER_OFF		0x41
#define GET_LASER_POS		0x42
//#define SET_WORK_MODE		0x43
#define SET_VOLTAGE		0x44
#define SET_PID_MODE		0x80

#define SET_HV_ON		0x49
#define SET_HV_OFF		0x4A
/*------------------------------------------------------*/
#define CMD_FREQ_SCAN		0x45
#define SET_WORKING_FREQ	0x46
#define SET_FREQ_AMPLITUDE	0x47
#define CMD_FREQ_STOP		0x48
#define SET_FREQ_RANGE		0x4B
#define SET_FREQ_PARA		0x4C
/*------------------------------------------------------*/

#define CMD_LINESCAN_START	0x20
//#define CMD_FAST_SCAN		0x22
#define CMD_SCAN_STOP		0x21
//#define CMD_SCAN_PAUSE		0x2F
#define SET_FEEDBACK_MODE	0x29
#define SET_PID_PARA		0x57
#define SET_PID_OTHER		0x28
#define CMD_FORCE_CURVE_START		0x2A
#define CMD_FORCE_CURVE_STOP		0x2B
#define SET_PID_PARA_SLIDER 0x2D
#define SET_SCAN_PIXEL_OLD 0x55

/*------------------------------------------------------*/
//#define LINE_SCAN_ONCE 		0x01
//#define LINE_SCAN_NOMAL 	0x02
//#define FAST_SCAN 		0x03
#define NOMAL_SCAN 		0x56
#define STOP 			0x00
//#define LASER_POS 		0x06

/*------------------------------------------------------*/
#define EXPERT_MODE_LED_ON	0xE0
#define EXPERT_MODE_LED_OFF	0xE1
#define EXPERT_MODE_AD		0xf0
#define EXPERT_MODE_DA		0xf1
#define EXPERT_MODE_DDS_FREQ	0xf2
#define EXPERT_MODE_DDS_AMPL	0xf3
#define EXPERT_MODE_DDS_STOP	0xf4
#define EXPERT_MODE_PID		0xf5
#define EXPERT_MODE_LASER_POS	0xf6
#define EXPERT_MODE_IO_OUT1	0xf7
#define EXPERT_MODE_IO_OUT8	0xf8
#define EXPERT_MODE_IO_OUT16	0xf9
#define EXPERT_MODE_DDS_PHASE	0xfa
#define EXPERT_MODE_GAIN_OFFSET 0xfb
/*------------------------------------------------------*/
//#define MODE_AFM_CONTACT	0x01
//#define MODE_AFM_TAPPING	0x02

#define CMD_PIEZO_LINER_PARA 0x30
#define CMD_FORCE_CURVE_NEW_START		0x31
#define CMD_FORCE_CURVE_NEW_STOP		0x32
#define CMD_APPROACH_NEW_UP		0x33
#define CMD_APPROACH_NEW_DOWN	0x34

#define CMD_SET_Z_VOL	0x35
#define CMD_SET_PIEZO_DELAY	0x36
#define CMD_LINE_FIT	0x37
#define CMD_DRAW_Y_STATE 0x38
#define CMD_FEEDBACK_SWITCH	0x39
#define SET_TB_DRAW_SPEED 0x3A
#define CMD_AD_SAMPLE_TIME 0x3B
#define CMD_STEP_SIGNAL 0x3C
#define CMD_CMD_STEP_PARA 0x3D
#define CMD_XY_CTR 0x3E
#define CMD_TUBE_SIZE 0x3F
#define CMD_IDENT_X 0x50
#define CMD_SHOW_TB_CHANNEL	0x51
#define CMD_FORCE_CURVE_NEW_EXP 0x52
#define CMD_DRIVE_WAVE_SEL 0x53

/*���¾�Ϊ�����Ŀ��Э��*/
#define HEAD_ENERGY 0x8801 //��ͷʹ��
#define SET_WORK_MODE	0x8802 //ɨ�淽ʽѡ��
#define SHAKING_PIEZO_ALL 0x8803 //Shaking piezo����
#define SHAKING_PIEZO_AMP 0x8804 //shaking piezo������ֵ
#define SHAKING_PIEZO_FRE 0x8805 //shaking piezo����Ƶ��
#define TAPPING_FEEDBACK 0x8806 //Tapping�����ź�ѡ��
#define SET_WORKING_POINT	0x8807 //Setpoint
#define PID_MODE 0x8808 //���������ʽѡ��
#define PID_P 0x8809 //�ֶ���������
#define PID_I 0x880A //�ֶ���������
#define FORWARD_GAIN 0x880B //�ֶ�ǰ������
#define SET_SCAN_RANGE 0x880C //ɨ�淶Χ
#define OFFSET_X 0x880D //X����ƫ��
#define OFFSET_Y 0x880E //Y����ƫ��
#define SET_LINE_RATE	0x880F //ɨ����Ƶ
#define IMAGE_SCALE	0x8810 //ͼ�����
#define SET_SCAN_PIXEL	0x8811 //ͼ��ֱ���
#define SLOW_AXIS	0x8812 //����ɨ�迪��
#define SWITCH_ERROR 0x8813 //Errorͨ������
#define SWITCH_FRICTION 0x8814 //Frictionͨ������
#define SWITCH_DEFLECTION 0x8815 //Deflectionͨ������
#define SWITCH_PHASE 0x8822 //Phaseͨ������
#define SWITCH_AMPLITUDE 0x8823 //Amplitudeͨ������
#define CMD_SCAN_WHOLE	0x8824 //ɨ�迪ʼ/��ͣ
#define CMD_SCAN_PAUSE	0x882F //ɨ�迪ʼ/��ͣ
#define MOTOR_AUTO_FORWARD_NEW 0x8825 //����
#define MOTOR_AUTO_BACKWARD_NEW 0x8826 //����
#define IMAGE_DIRECTION 0x8827 //ͼ�����ɷ���

#define SCAN_LOCATION 0x8881 //��ɨ������ǰλ��
#define MOTOR_Z1 0x8882 //Z1���������
#define SEND_LASER_POSITION 0x8883 //���̽����
#define AUTO_SCAN_RESULT 0x8884 //�Զ�ɨƵ���
#define SINGLE_LINE_RESULT 0x8885 //��������ɨ�����ݼ�����Z������ѹƽ��ֵ
#define MOTOR_AUTO_FORWARD_FINISH 0x8886 //�������

// ������������
#define TEST_SETPOINT 0x54
#define ONE_PID 0x85
#define FIVE_PID 0x86
#define SIX_PID 0x87
#define TEST_ALL 0x88
#define TEST_AD 0x89
#define TEST_DA 0x90
#define TEST_ALL_STOP 0x91
#define TEST_DA_STOP 0x92
#define TEST_SYSTEM 0x93
#define TEST_SYSTEM_STOP 0x94

/*AD676 2 chips*/
#define CHN_ERROR  0x7
#define CHN_Z 		 0x6
#define CHN_Y 		 0x2
/*
#define AD_LASER_A 0x0
#define AD_LASER_B 0x1
#define AD_LASER_C 0x2
#define AD_LASER_D 0x3
*/


#define AD_LASER_A 0x8
#define AD_LASER_B 0x9
#define AD_LASER_C 0xA
#define AD_LASER_D 0xB

#ifdef _DEBUG_
#define DEBUG0( msg ) puts(msg);
#define DEBUG1(msg, val) printf(msg, val);
#define DEBUG2(msg, val1, val2) printf(msg, val1, val2);
#else
#define DEBUG0( msg )
#define DEBUG1(msg, val)
#define DEBUG2(msg, val1, val2)
#endif

#endif
