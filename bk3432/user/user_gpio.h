
#ifndef _USER_GPIO_H_
#define _USER_GPIO_H_

#include "user_config.h"

//#######################################################################################//
//#################################### 引脚定义 ##########################################//
//#######################################################################################//
//##################################### LNH_01 ##################################################//
#if DU_PD01B
	#define MOTOR_PIN_P		0x11				//喂食电机正转
	#define MOTOR_PIN_N		0x04				//喂食电机反转
	#define MOTOR_DET		0x34				//喂食电机到位
	#define GPIO_315MHz		0x12				//315Mhz通信数据线

//	#define RST_KEY			0x33				//复位按键
	#define SET_KEY			0x01				//电源按键
	#define RECORD_KEY		0x00				//录音按键
	#define FEED_KEY		0x14				//喂食按键

	#define LED_RED			0x06				//红灯（错误指示灯）
	#define LED_GREEN		0x07				//绿灯（网络指示灯）

	#define SOUND_PLAY		0x13				//录音芯片播放脚
	#define SOUND_REC		0x33				//录音芯片录音脚

	#define DATA_315MHZ		0x12				//315Mhz通信数据线
	#define PWR_HOLD		0x05				//电源保持脚
	#define MOTOR_ADC		0x35				//

	#define CHARGE_DET		0x32				//外电检测
	#define BAT_DET			0x31				//电池电量检测

	#define I2C_SDA			0x03				
	#define I2C_SCL			0x02
#endif
//##################################### DU_PD01B ##################################################//
#if DU5C_B01
	#define MOTOR_PIN_P		0x11				//喂食电机正转
	#define MOTOR_PIN_N		0x10				//喂食电机反转
	#define MOTOR_DET		0x34				//喂食电机到位

	#define SET_KEY			0x01				//电源按键
	#define RECORD_KEY		0x00				//录音按键
	#define FEED_KEY		0x14				//喂食按键

	#define LED_RED			0x06				//红灯（错误指示灯）
	#define LED_GREEN		0x07				//绿灯（网络指示灯）

	#define SOUND_PLAY		0x33				//录音芯片播放脚
	#define SOUND_REC		0x13				//录音芯片录音脚

	#define BUZZER_EN		0x12				//蜂鸣器
	#define MOTOR_ADC		0x35				//

	#define CHARGE_DET		0x32				//外电检测
	#define BAT_DET			0x31				//电池电量检测

#endif
//##################################### PT01K_BK ##################################################//


//######################### 计时时间 ####################################//
#define LED_PERIOD 				40			//400ms	警报灯（红灯）闪烁时间
#define ERROR_PERIOD 			100			//1s	电机错误时间检测
#define FEED_AGAIN_PERIOD 		30000		//300s  喂食错误之后5分钟再试一次

#ifndef MOTO_TIMEOUT
#define MOTO_TIMEOUT			900			//13秒  	喂食电机检测到位超时时间
#endif

enum LED_TYPE_E
{
	LED_NO,
	LED_ON,
	LED_OFF,
};

#ifdef NORMAL_DOOR_DET				//普通到位
#define DOOR_OPEN_DET_OFF  				(gpio_read(IR_DET_D2) == 0)			//
#define DOOR_OPEN_DET_ON   				(gpio_read(IR_DET_D2))
#define DOOR_CLOSE_DET_OFF  			(gpio_read(IR_DET_D) == 0)
#define DOOR_CLOSE_DET_ON   			(gpio_read(IR_DET_D))
#else								//光栅到位
#define DOOR_OPEN_DET_OFF  				(gpio_read(IR_DET_D2))
#define DOOR_OPEN_DET_ON   				(gpio_read(IR_DET_D2) == 0)
#define DOOR_CLOSE_DET_OFF  			(gpio_read(IR_DET_D))
#define DOOR_CLOSE_DET_ON   			(gpio_read(IR_DET_D) == 0)
#endif

#if defined NEW_RECORD_IC && defined NEW_RECORD_IC_LOW		//新录音ID ，低电平有效触发
#define RECORD_ON  					(0)			//
#define RECORD_OFF  				(1)			//
#define PLAYER_ON  					(0)			//
#define PLAYER_OFF  				(1)			//
#else								//旧录音IC，高电平有效触发
#define RECORD_ON  					(1)			//
#define RECORD_OFF  				(0)			//
#define PLAYER_ON  					(1)			//
#define PLAYER_OFF  				(0)			//
#endif

enum FOOD_STATE_E{
	LED_ENOUGH_FOOD 	= 0,		//粮桶粮食充足
    LED_NO_FOOD			= 1,		//粮桶无粮
    LED_NOT_ENOUGH_FOOD	= 2,		//粮桶粮食不多
};
//######################### 电机功能定义 ####################################//


//######################### 按键触发类型 ####################################//
enum KEY_TYPE_E{
	KEY_NO			= 0,		//无按键
    KEY_SET_S,					//按键短按
    KEY_UP_S,		//
    KEY_DOWN_S,		//
    KEY_RECORD_S,		//
    KEY_LOCK_S,		//
    KEY_FEED_S,		//
	
    KEY_SET_L,					//按键长按触发
    KEY_UP_L,		//
    KEY_DOWM_L,		//
    KEY_RECORD_L,		//
    KEY_LOCK_L,		//
    KEY_FEED_L,		//
	
    KEY_SET_L_UP,				//按键长按松开
    KEY_UP_L_UP,		//
    KEY_DOWN_L_UP,		//
    KEY_RECORD_L_UP,		//
    KEY_LOCK_L_UP,		//
    KEY_FEED_L_UP,		//
	
};



#ifndef KEY_SHORT_TIME
#define KEY_SHORT_TIME			10			//按键短按时间	1s
#endif
#ifndef KEY_LONG_TIME
#define KEY_LONG_TIME			50			//按键长按时间	5s
#endif
#ifndef KEY_LONG_SET_TIME
#define KEY_LONG_SET_TIME			KEY_LONG_TIME			//按键长按时间	5s
#endif
#ifndef KEY_LONG_FEED_TIME
#define KEY_LONG_FEED_TIME			KEY_LONG_TIME			//按键长按时间	5s
#endif
#ifndef KEY_LONG_LOCK_TIME
#define KEY_LONG_LOCK_TIME			KEY_LONG_TIME			//按键长按时间	5s
#endif
#ifndef LOCK_TIMEOUT_TIME
#define LOCK_TIMEOUT_TIME		20			//无操作20s后锁屏
#endif
#ifndef KEY_LONG_TIME_SET
#define KEY_LONG_TIME_SET		30			//按键长按时间	3s
#endif
#ifndef MOTOR_ADC_CHAN
#define MOTOR_ADC_CHAN		5				//ADC检测反转卡脚 ADC通道 
#endif
#ifndef REVERSE_TIME
#define REVERSE_TIME		150				//反转时长  1.5s 
#endif
#ifndef REVERSE_STOP_TIME
#define REVERSE_STOP_TIME		300			//反转停止时长  3s 
#endif
#ifndef REVERSE_COUNT
#define REVERSE_COUNT			1			//反转次数	1次 
#endif




#endif /* _USER_GPIO_H_ */
