/**
 *******************************************************************************
 *
 * @file user_config.h
 *
 * @brief Application configuration definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *******************************************************************************
 */
 
#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "utc_clock.h"
#include "stddef.h"
#include "string.h"
#include "tuya_ble_unix_time.h"

//#######################################################################################//
//#################################### 型号定义 ##########################################//
//#######################################################################################//
#define DU_PD01B		1					//小安零食机
//#define DU5C_B01		0					//小安圆盘BK3432

#define TEST_UID	0						//用测试UID授权标志  用测试授权：16

//#define TUYA_APP_LOG_ENABLE 1
//#define TUYA_BLE_LOG_ENABLE 1


//#######################################################################################//
//#################################### 功能定义 ##########################################//
//#######################################################################################//
//#######################################################################################//
#if DU_PD01B
#define USER_VERSION 	"DU_PD01B.0.1"
#define USER_DATA 		"20210825"

//	#define RTC_TIME			1		//外部时钟计时
	#define MOTOR_REVERSE		1		//电机反转功能
//	#define BATTERY_FUNC		1		//ADC检测电池电量功能
	#define BATTERY_CHAN		1		//ADC检测电池电量的通道口
	
	#define NEW_RECORD_IC		1		//新录音IC
//	#define NEW_RECORD_IC_LOW	1		//新录音IC低电平触发(加3极管是高电平触发)
	#define FUNC_315MHZ			1		//315Mhz无线通信功能
	#define POWER_HOLD_FUNC		1		//控制电源
	#define NO_FEED_PLAN		1		//没有喂食计划功能

	#define UART_2_INIT			1		//串口2初始化
	#define UART_2_PRINTF		0		//串口2打印
	#define UART_1_INIT			0		//串口1初始化
	#define UART_1_PRINTF		0		//串口1打印

	#define MOTOR_DELAY 		1		//喂食一份后延时停止电机时间
	#define BATTERY_MIN 		273		//电量最低值
	#define BATTERY_MAX 		396		//电量最高值（用于计算电量百分比）	
	#define MOTO_TIMEOUT		400		//4秒  	喂食电机检测到位超时时间


#endif
//#######################################################################################//
#if DU5C_B01
#define USER_VERSION 	"DU5C_B01.0.1"
#define USER_DATA 		"20210825"

	#define RTC_TIME			1		//外部时钟计时
	#define MOTOR_REVERSE		1		//电机反转功能
	#define MOTOR_REVERSE_STOP	1		//电机反转后停止功能
	#define MOTOR_REVERSE_ADC	1		//ADC检测电机反转功能
	#define BATTERY_FUNC		1		//ADC检测电池电量功能
	#define BATTERY_CHAN		1		//ADC检测电池电量的通道口
	
	#define NEW_RECORD_IC		1		//新录音IC
	#define NEW_RECORD_IC_LOW	1		//新录音IC低电平触发(加3极管是高电平触发)

	#define UART_2_INIT			1		//串口2初始化
	#define UART_2_PRINTF		1		//串口2打印
	#define UART_1_INIT			0		//串口1初始化
	#define UART_1_PRINTF		0		//串口1打印

	#define MOTOR_DELAY 		1		//喂食一份后延时停止电机时间
	#define BATTERY_MIN 		273		//电量最低值
	#define BATTERY_MAX 		396		//电量最高值（用于计算电量百分比）	
	#define MOTO_TIMEOUT		1200	//12秒  	喂食电机检测到位超时时间

#define KEY_LONG_FEED_TIME		80		//喂食按键长按时间	8s
#define REVERSE_TIME			300		//反转时长  3s 
#define REVERSE_COUNT			5		//反转次数	5次

#endif


//#######################################################################################//
//#######################################################################################//
//#######################################################################################//

//#define IR_CHECK_FOOD			1		//出食对射管检测落粮
//#define IR_FEED_TUBE_LED		1		//出食对射管有发射管
//#define IR_FOOD_TUBE			1		//粮仓余粮检测对射管
//#define RTC_TIME				1		//时钟计时
//#define MOTOR_DET_ADDR		1		//电机控制到位停止位置（选中就是松开停止）
//#define DOOR_CONTROL			1		//门仓控制
//#define LINUX_RECORD			1		//linux录音功能
//#define LOCAL_RECORD			1		//本地录音功能
//#define ONE_KEY_RECORD		1		//单按键录音功能
//#define NO_RECORD_FUNC		1		//没有录音功能
//#define LOCK_KEY_E			1		//喂食按键作为锁键功能
//#define MOTOR_REVERSE			1		//电机反转功能
//#define REC_KEY_3SEC			1		//长按3s才开始录音
//#define KEY_BUZZER_FUNC		1		//按键蜂鸣器响功能
//#define POWER_BEEP			1		//有源蜂鸣器
//#define LOCK_KEY_KEY_E		1		//锁键按键作为锁键功能
//#define RELOCK_KEY_KEY_E		1		//解锁键按键作为锁键功能
//#define 315Mhz_FUNC			1		//315Mhz无线通信功能
//#define POWER_HOLD_FUNC		1		//控制电源
//#define NO_FEED_PLAN			1		//没有喂食计划功能

//  //###################### 特殊控制 ######################//
//#define LED_ON_HIGHT			1		//灯高低电平变化控制（选中高电平亮灯）
//#define KEY_DOWM_HIGHT		1		//按键高低电平控制（选中高电平触发按键(按下是高)）
//#define FEED_ALWAYS_ONE		1		//喂食固定一份
//#define IR_FEED_TUBE			1		//出食对射管
//#define BK3431Q_40PIN			1		//BK3431Q_40PIN芯片
//#define UART_PRINTF_EN		0		//UART使能控制宏
//#define BEEP_CHANNAL			2		//蜂鸣器PWM通道

//#define UART1_INIT_E		1		//串口1初始化
//#define UART2_INIT_E		1		//串口2初始化
//#define UART1_PRINTF_E	1		//串口1为打印串口
//#define UART2_PRINTF_E	1		//串口1为打印串口
//#define UART2_WRITE_INFO	1		//串口2写授权信息

#define FEED_MAX_NUM 			10 			//定时喂食最大条数
#define FLASH_KEEP_VAL 			12 			//flash存储标识号
#define FEED_RECORD_NUM 		50 			//最大喂食记录保存条数
#define BATTERY_SAMPLE_TIME 	20 			//电池电量读数，取中间平均值为电池电量


//#define BLE_SAVE_ADDR	0x80
//#define BLE_PLAN_ADDR	0x8000
#define BLE_SAVE_ADDR	0x26000
#define BLE_PLAN_ADDR	0x26400

#define FLASH_SIZE_ONE	512

//#######################################################################################//
//#################################### 状态定义  ##########################################//
//#######################################################################################//

//#################################### 灯状态  ##########################################//
#ifdef LED_ON_HIGHT
	#define SET_LED_ON(led_pin) 		gpio_set(led_pin, 1)
	#define SET_LED_OFF(led_pin) 		gpio_set(led_pin, 0)
#else
	#define SET_LED_ON(led_pin) 		gpio_set(led_pin, 0)
	#define SET_LED_OFF(led_pin) 		gpio_set(led_pin, 1)
#endif

enum DOOR_CONTROL_E
{
	DOOR_PULL		= 0x1,			//推门
	DOOR_PUSH   	= 0x2,			//拉门
	DOOR_STOP	   	= 0x3			//停止
};

enum MOTOR_CONTROL_E
{
	MOTOR_PULL		= 0x4,			//电机正转
	MOTOR_PUSH   	= 0x5,			//电机反转
	MOTOR_STOP	   	= 0x6			//电机停止
};

enum FLASH_STYE
{
	FLASH_PLAN 	= 0x01,
	FLASH_LOG 	= 0x02,
};

//#################################### 喂食错误类型 ##########################################//
//涂鸦公版
#define ERROR_EMPTY				0x01		//无粮
#define ERROR_IR				0x02		//红外错误(堵粮)
#define ERROR_MOTOR_TIMEOUT		0x04		//电机超时
#define ERROR_NOT_ENOUGH		0x08		//余粮不足
#define ERROR_KEY_COUNT			0x10		//按键次数达到最大值

//#################################### 喂食类型 ##########################################//
#define FEED_NEW 		0x1
#define FEED_AUTO 		0x1
#define FEED_OLD 		0x2
#define FEED_MANUAL 	0x3

//#################################### 喂食类型
enum FEED_TYPE {
	NONE_FEED = 0,
	AUTO_FEED,
	APP_FEED,
	KEY_FEED,
};

//#################################### 喂食状态
enum FEED_STATUS_TYPE {
	FEED_STEP_NONE = 0,				//无喂食
	FEED_STEP_SOUND,				//播音
	FEED_STEP_READY,				//喂食
	FEED_STEP_LEAVE_DET,			//喂食一份，到位
	FEED_STEP_WAIT_DET,				//喂食完，算份数，检测设备异常
	FEED_STEP_OVER,					//喂食结束，统计喂食记录
	FEED_STEP_ERROR_IR,				//红外对管堵塞异常检测
	FEED_STEP_IR_COUNT_DOWN,		//红外对管堵塞异常计时5分钟取消剩余份数
	FEED_STEP_REVERSE,				//电机反转
	FEED_STEP_REVERSE_STOP,			//电机反转停止
};

enum BK_315_BYTE {
 NONE = 0,
 HEAD_1,
 HEAD_2,
 BIT_H,
 BIT_L,
};
//#################################### 需要flash存储的变量  ##########################################//

typedef struct {
	uint8_t week;
	uint8_t hour;
	uint8_t min;
	uint8_t weight;
	uint8_t flag;
} FEED_INFO_t;

typedef struct {
	uint32_t mark;						//标志位
//	uint8_t sound;						//声音
	uint8_t lock;						//喂食锁
	uint8_t RTC_flag;					//RTC是否存时间标志
	uint16_t RTC_zone_f;				//时区正负
	uint16_t RTC_zone;					//时区
	uint8_t reset_day;					//重启标志
	uint16_t record_time;				//录音时长
	#ifndef NO_FEED_PLAN
	FEED_INFO_t plans[FEED_MAX_NUM];	//喂食计划,周+时+分+份数+开关
	#endif
	#ifdef FUNC_315MHZ
	uint8_t key[8][24];					//315无线通信数据
	#endif
} FEED_PLAN_t;

typedef struct {
	uint32_t mark;						//标志位
	uint32_t logs[FEED_RECORD_NUM][3];	//喂食记录，时间戳+喂食份数+是否上传标志
} FEED_LOG_INFO_t;

extern FEED_PLAN_t ty_plan;
extern FEED_LOG_INFO_t ty_logs;

void printf_flash_info(void);
void flash_data_init(uint8_t type);
void led_control(uint8_t link_led, uint8_t red_led, uint8_t led_level);
void beep_test(void);

extern uint8_t keep_dowm_flag;			//按键长按标志
extern uint8_t key_flag;				//按键触发类型

extern FEED_INFO_t feed_now;
extern int time_zone_sec;
extern uint8_t feed_flag;
extern uint8_t feeding_flag;			//正在喂食标志
extern uint8_t feed_key;
extern uint8_t calender_flag;
extern uint8_t time_sync;
extern uint8_t lock_led;
extern uint8_t lock_time;				//锁键按键次数
extern uint8_t led_test;
extern uint8_t low_power;
extern uint8_t dc_flag;
extern uint8_t connect_flag;
extern uint8_t sleep_tick;
extern uint8_t led_tick;
extern uint8_t restore_flag;
extern uint8_t app_weight;
extern uint8_t err_type;				//喂食异常

extern uint8_t feed_key_flag;			//喂食按键按下标志
extern uint8_t record_key_flag;			//录音按键按下标志
extern uint8_t ret_key_flag;			//复位按键按下标志
extern uint8_t lock_key_flag;			//锁键按键按下标志
extern uint8_t relock_key_flag;			//解锁键按键按下标志
extern uint8_t debug_flag;
extern uint8_t check_voltage_flag;		//读取电量标志
extern uint8_t adc_reverse_flag;		//ADC反转标志
extern uint8_t read_adc_flag;			//ADC检测反转标志 0:读数开始检测 1：已有读数值
extern volatile uint8_t byte_flag;
extern uint8_t pair_flag;
extern uint8_t save_flag;


void feed_check(void);
void feed_run_callback(void);
void log_upload_callback(void);
void plan_upload_callback(void);
uint32_t get_timestamp_zero(void);										//获取0时区时间戳
void send_feed_error(uint32_t t_zero_t, uint8_t err_type_t);		//发送喂食异常
void send_feed_record(uint32_t t_zero_t, uint8_t feed_weight);		//发送喂食结果份数
void restart_time_set(int seed);
void PCF8563_init(void);
void save_flash(enum FLASH_STYE flash_type);

void BK_315MHz_timer_set(void);
void byte_debug_show(void);

typedef struct
{
    uint8_t dp_id;
    uint8_t dp_type;
    uint8_t dp_data_len;
    uint8_t dp_data[256];
} demo_dp_t;



 /******************************************************************************
  *############################################################################*
  * 							SYSTEM MACRO CTRL                              *
  *############################################################################*
  *****************************************************************************/

//如果需要使用GPIO进行调试，需要打开这个宏
#define GPIO_DBG_MSG					0
//UART使能控制宏
#define UART_PRINTF_EN					1
//蓝牙硬件调试控制
#define DEBUG_HW						0




 
/*******************************************************************************
 *#############################################################################*
 *								APPLICATION MACRO CTRL                         *
 *#############################################################################*
 *******************************************************************************/
 
//连接参数更新控制
#define UPDATE_CONNENCT_PARAM  			1

//最小连接间隔
#define BLE_UAPDATA_MIN_INTVALUE		20
//最大连接间隔 
#define BLE_UAPDATA_MAX_INTVALUE		40
//连接Latency
#define BLE_UAPDATA_LATENCY				0
//连接超时
#define BLE_UAPDATA_TIMEOUT				600


//设备名称
#define APP_DFLT_DEVICE_NAME           ("TY_wen")

//#define USER_MAC_DEFAULT		"DC234D6767EC"
//gzjk71f9dfdc4e7b	
//UVu8IDId5RHtdX2KNzGwCkUfzgV6xSal	
//DC234D6767EC

 //广播包配置
#define APP_TUYA_ADV_DATA       "\x02\x01\x06\x03\x02\x01\xA2\x14\x16\x01\xA2\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
#define APP_TUYA_ADV_DATA_LEN    (28)

//扫描响应包数据
#define APP_SCNRSP_DATA        "\x03\x09\x54\x59\x19\xFF\xD0\x07\x00\x03\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" //
#define APP_SCNRSP_DATA_LEN     (30)


//广播参数配置
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MIN         (320)
/// Advertising maximum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MAX         (320)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)




/*******************************************************************************
 *#############################################################################*
 *								DRIVER MACRO CTRL                              *
 *#############################################################################*
 ******************************************************************************/
#ifdef RTC_TIME
#include "tuya_ble_unix_time.h"
#include "PCF8563.h" 

#define PCF8563_Read		0x0A3
#define PCF8563_Write		0x0A2

#define RTC_YEAR_ADDR		0x08
#define RTC_MONTH_ADDR		0x07
#define RTC_WEEK_ADDR		0x06
#define RTC_DAY_ADDR		0x05
#define RTC_HOUR_ADDR		0x04
#define RTC_MINUTE_ADDR		0x03
#define RTC_SECOND_ADDR		0x02

#define RTC_CONTROL2_ADDR				0x01
#define RTC_CONTROL1_ADDR				0x00

#define RTC_MIN_ALARM_ADDR				0x09
#define RTC_HOUR_ALARM_ADDR				0x0a
#define RTC_DAY_ALARM_ADDR				0x0b
#define RTC_WEEK_ALARM_ADDR				0x0c
#define RTC_CLKOUT_COTL_ADDR			0x0d
#define RTC_TIMER_COTL_ADDR				0x0e
#define RTC_TIMER_COUNT_DOWN_ADDR		0x0f

#define SCLK 	0x02//I2C_SCL
#define SDAT 	0x03//I2C_SDA

#define SCL0_HIGH()		  gpio_set(SCLK,1)
#define SCL0_LOW()		  gpio_set(SCLK,0)
#define SDA0_HIGH()		  gpio_set(SDAT,1)
#define SDA0_LOW()		  gpio_set(SDAT,0)
#define SDA0_SetInput()	  gpio_config(SDAT,INPUT,PULL_HIGH)
#define SDA0_SetOutput()  gpio_config(SDAT,OUTPUT,PULL_HIGH)
#define SDA0_READ()		  gpio_get_input(SDAT)

extern int time_zone_RTC_rec;

void i2cs_init(void);
void i2cs_start(void);
void i2cs_stop(void);
uint8_t i2cs_tx_byte(uint8_t dat);
uint8_t i2cs_rx_byte(char ack);
void i2cs_tx_data(uint8_t devAddr7,uint8_t addr,uint8_t*buf,uint8_t size);
void i2cs_rx_data(uint8_t devAddr7,uint8_t addr,uint8_t*buf,uint8_t size);
void IIC_Ack(unsigned char a);
unsigned char BCD2HEX(unsigned char bcd_data); //BCD转为HEX子程序 
unsigned char HEX2BCD(unsigned char hex_data); //HEX转为BCD子程序 

uint32_t get_RTC_time(tuya_ble_time_struct_data_t *currTime);
void set_RTC_time(tuya_ble_time_struct_data_t *currTime );
void RTC_H8563_init(void);

#endif


//DRIVER CONFIG
#define UART_DRIVER						0
#define UART2_DRIVER					1
#define GPIO_DRIVER						1
#define AUDIO_DRIVER					0
#define RTC_DRIVER						1
#define ADC_DRIVER						1
#define I2C_DRIVER						0
#define PWM_DRIVER						1



#if RTC_DRIVER
#include "rtc.h"
#endif

#if ADC_DRIVER
#include "adc.h"
#endif

#if PWM_DRIVER
#include "pwm.h"
#endif

#include "string.h"
#include "stdio.h"

#include "gpio.h"
#include "rf.h"
#include "uart2.h"
#include "uart.h"
#include "flash.h"
#include "motor.h"
#include "nvds.h"

#include "lcd.h"
#include "function.h"
#include "user_gpio.h"
#include "tuya_ble_type.h"
#include "tuya_ble_api.h"
#include "app_task.h"
#include "ke_timer.h"

#include "PCF8563.h"





#endif /* _USER_CONFIG_H_ */
