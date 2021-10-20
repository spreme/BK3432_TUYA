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
//#################################### �ͺŶ��� ##########################################//
//#######################################################################################//
#define DU_PD01B		1					//С����ʳ��
//#define DU5C_B01		0					//С��Բ��BK3432

#define TEST_UID	0						//�ò���UID��Ȩ��־  �ò�����Ȩ��16

//#define TUYA_APP_LOG_ENABLE 1
//#define TUYA_BLE_LOG_ENABLE 1


//#######################################################################################//
//#################################### ���ܶ��� ##########################################//
//#######################################################################################//
//#######################################################################################//
#if DU_PD01B
#define USER_VERSION 	"DU_PD01B.0.1"
#define USER_DATA 		"20210825"

//	#define RTC_TIME			1		//�ⲿʱ�Ӽ�ʱ
	#define MOTOR_REVERSE		1		//�����ת����
//	#define BATTERY_FUNC		1		//ADC����ص�������
	#define BATTERY_CHAN		1		//ADC����ص�����ͨ����
	
	#define NEW_RECORD_IC		1		//��¼��IC
//	#define NEW_RECORD_IC_LOW	1		//��¼��IC�͵�ƽ����(��3�����Ǹߵ�ƽ����)
	#define FUNC_315MHZ			1		//315Mhz����ͨ�Ź���
	#define POWER_HOLD_FUNC		1		//���Ƶ�Դ
	#define NO_FEED_PLAN		1		//û��ιʳ�ƻ�����

	#define UART_2_INIT			1		//����2��ʼ��
	#define UART_2_PRINTF		0		//����2��ӡ
	#define UART_1_INIT			0		//����1��ʼ��
	#define UART_1_PRINTF		0		//����1��ӡ

	#define MOTOR_DELAY 		1		//ιʳһ�ݺ���ʱֹͣ���ʱ��
	#define BATTERY_MIN 		273		//�������ֵ
	#define BATTERY_MAX 		396		//�������ֵ�����ڼ�������ٷֱȣ�	
	#define MOTO_TIMEOUT		400		//4��  	ιʳ�����⵽λ��ʱʱ��


#endif
//#######################################################################################//
#if DU5C_B01
#define USER_VERSION 	"DU5C_B01.0.1"
#define USER_DATA 		"20210825"

	#define RTC_TIME			1		//�ⲿʱ�Ӽ�ʱ
	#define MOTOR_REVERSE		1		//�����ת����
	#define MOTOR_REVERSE_STOP	1		//�����ת��ֹͣ����
	#define MOTOR_REVERSE_ADC	1		//ADC�������ת����
	#define BATTERY_FUNC		1		//ADC����ص�������
	#define BATTERY_CHAN		1		//ADC����ص�����ͨ����
	
	#define NEW_RECORD_IC		1		//��¼��IC
	#define NEW_RECORD_IC_LOW	1		//��¼��IC�͵�ƽ����(��3�����Ǹߵ�ƽ����)

	#define UART_2_INIT			1		//����2��ʼ��
	#define UART_2_PRINTF		1		//����2��ӡ
	#define UART_1_INIT			0		//����1��ʼ��
	#define UART_1_PRINTF		0		//����1��ӡ

	#define MOTOR_DELAY 		1		//ιʳһ�ݺ���ʱֹͣ���ʱ��
	#define BATTERY_MIN 		273		//�������ֵ
	#define BATTERY_MAX 		396		//�������ֵ�����ڼ�������ٷֱȣ�	
	#define MOTO_TIMEOUT		1200	//12��  	ιʳ�����⵽λ��ʱʱ��

#define KEY_LONG_FEED_TIME		80		//ιʳ��������ʱ��	8s
#define REVERSE_TIME			300		//��תʱ��  3s 
#define REVERSE_COUNT			5		//��ת����	5��

#endif


//#######################################################################################//
//#######################################################################################//
//#######################################################################################//

//#define IR_CHECK_FOOD			1		//��ʳ����ܼ������
//#define IR_FEED_TUBE_LED		1		//��ʳ������з����
//#define IR_FOOD_TUBE			1		//���������������
//#define RTC_TIME				1		//ʱ�Ӽ�ʱ
//#define MOTOR_DET_ADDR		1		//������Ƶ�λֹͣλ�ã�ѡ�о����ɿ�ֹͣ��
//#define DOOR_CONTROL			1		//�Ųֿ���
//#define LINUX_RECORD			1		//linux¼������
//#define LOCAL_RECORD			1		//����¼������
//#define ONE_KEY_RECORD		1		//������¼������
//#define NO_RECORD_FUNC		1		//û��¼������
//#define LOCK_KEY_E			1		//ιʳ������Ϊ��������
//#define MOTOR_REVERSE			1		//�����ת����
//#define REC_KEY_3SEC			1		//����3s�ſ�ʼ¼��
//#define KEY_BUZZER_FUNC		1		//�����������칦��
//#define POWER_BEEP			1		//��Դ������
//#define LOCK_KEY_KEY_E		1		//����������Ϊ��������
//#define RELOCK_KEY_KEY_E		1		//������������Ϊ��������
//#define 315Mhz_FUNC			1		//315Mhz����ͨ�Ź���
//#define POWER_HOLD_FUNC		1		//���Ƶ�Դ
//#define NO_FEED_PLAN			1		//û��ιʳ�ƻ�����

//  //###################### ������� ######################//
//#define LED_ON_HIGHT			1		//�Ƹߵ͵�ƽ�仯���ƣ�ѡ�иߵ�ƽ���ƣ�
//#define KEY_DOWM_HIGHT		1		//�����ߵ͵�ƽ���ƣ�ѡ�иߵ�ƽ��������(�����Ǹ�)��
//#define FEED_ALWAYS_ONE		1		//ιʳ�̶�һ��
//#define IR_FEED_TUBE			1		//��ʳ�����
//#define BK3431Q_40PIN			1		//BK3431Q_40PINоƬ
//#define UART_PRINTF_EN		0		//UARTʹ�ܿ��ƺ�
//#define BEEP_CHANNAL			2		//������PWMͨ��

//#define UART1_INIT_E		1		//����1��ʼ��
//#define UART2_INIT_E		1		//����2��ʼ��
//#define UART1_PRINTF_E	1		//����1Ϊ��ӡ����
//#define UART2_PRINTF_E	1		//����1Ϊ��ӡ����
//#define UART2_WRITE_INFO	1		//����2д��Ȩ��Ϣ

#define FEED_MAX_NUM 			10 			//��ʱιʳ�������
#define FLASH_KEEP_VAL 			12 			//flash�洢��ʶ��
#define FEED_RECORD_NUM 		50 			//���ιʳ��¼��������
#define BATTERY_SAMPLE_TIME 	20 			//��ص���������ȡ�м�ƽ��ֵΪ��ص���


//#define BLE_SAVE_ADDR	0x80
//#define BLE_PLAN_ADDR	0x8000
#define BLE_SAVE_ADDR	0x26000
#define BLE_PLAN_ADDR	0x26400

#define FLASH_SIZE_ONE	512

//#######################################################################################//
//#################################### ״̬����  ##########################################//
//#######################################################################################//

//#################################### ��״̬  ##########################################//
#ifdef LED_ON_HIGHT
	#define SET_LED_ON(led_pin) 		gpio_set(led_pin, 1)
	#define SET_LED_OFF(led_pin) 		gpio_set(led_pin, 0)
#else
	#define SET_LED_ON(led_pin) 		gpio_set(led_pin, 0)
	#define SET_LED_OFF(led_pin) 		gpio_set(led_pin, 1)
#endif

enum DOOR_CONTROL_E
{
	DOOR_PULL		= 0x1,			//����
	DOOR_PUSH   	= 0x2,			//����
	DOOR_STOP	   	= 0x3			//ֹͣ
};

enum MOTOR_CONTROL_E
{
	MOTOR_PULL		= 0x4,			//�����ת
	MOTOR_PUSH   	= 0x5,			//�����ת
	MOTOR_STOP	   	= 0x6			//���ֹͣ
};

enum FLASH_STYE
{
	FLASH_PLAN 	= 0x01,
	FLASH_LOG 	= 0x02,
};

//#################################### ιʳ�������� ##########################################//
//Ϳѻ����
#define ERROR_EMPTY				0x01		//����
#define ERROR_IR				0x02		//�������(����)
#define ERROR_MOTOR_TIMEOUT		0x04		//�����ʱ
#define ERROR_NOT_ENOUGH		0x08		//��������
#define ERROR_KEY_COUNT			0x10		//���������ﵽ���ֵ

//#################################### ιʳ���� ##########################################//
#define FEED_NEW 		0x1
#define FEED_AUTO 		0x1
#define FEED_OLD 		0x2
#define FEED_MANUAL 	0x3

//#################################### ιʳ����
enum FEED_TYPE {
	NONE_FEED = 0,
	AUTO_FEED,
	APP_FEED,
	KEY_FEED,
};

//#################################### ιʳ״̬
enum FEED_STATUS_TYPE {
	FEED_STEP_NONE = 0,				//��ιʳ
	FEED_STEP_SOUND,				//����
	FEED_STEP_READY,				//ιʳ
	FEED_STEP_LEAVE_DET,			//ιʳһ�ݣ���λ
	FEED_STEP_WAIT_DET,				//ιʳ�꣬�����������豸�쳣
	FEED_STEP_OVER,					//ιʳ������ͳ��ιʳ��¼
	FEED_STEP_ERROR_IR,				//����Թܶ����쳣���
	FEED_STEP_IR_COUNT_DOWN,		//����Թܶ����쳣��ʱ5����ȡ��ʣ�����
	FEED_STEP_REVERSE,				//�����ת
	FEED_STEP_REVERSE_STOP,			//�����תֹͣ
};

enum BK_315_BYTE {
 NONE = 0,
 HEAD_1,
 HEAD_2,
 BIT_H,
 BIT_L,
};
//#################################### ��Ҫflash�洢�ı���  ##########################################//

typedef struct {
	uint8_t week;
	uint8_t hour;
	uint8_t min;
	uint8_t weight;
	uint8_t flag;
} FEED_INFO_t;

typedef struct {
	uint32_t mark;						//��־λ
//	uint8_t sound;						//����
	uint8_t lock;						//ιʳ��
	uint8_t RTC_flag;					//RTC�Ƿ��ʱ���־
	uint16_t RTC_zone_f;				//ʱ������
	uint16_t RTC_zone;					//ʱ��
	uint8_t reset_day;					//������־
	uint16_t record_time;				//¼��ʱ��
	#ifndef NO_FEED_PLAN
	FEED_INFO_t plans[FEED_MAX_NUM];	//ιʳ�ƻ�,��+ʱ+��+����+����
	#endif
	#ifdef FUNC_315MHZ
	uint8_t key[8][24];					//315����ͨ������
	#endif
} FEED_PLAN_t;

typedef struct {
	uint32_t mark;						//��־λ
	uint32_t logs[FEED_RECORD_NUM][3];	//ιʳ��¼��ʱ���+ιʳ����+�Ƿ��ϴ���־
} FEED_LOG_INFO_t;

extern FEED_PLAN_t ty_plan;
extern FEED_LOG_INFO_t ty_logs;

void printf_flash_info(void);
void flash_data_init(uint8_t type);
void led_control(uint8_t link_led, uint8_t red_led, uint8_t led_level);
void beep_test(void);

extern uint8_t keep_dowm_flag;			//����������־
extern uint8_t key_flag;				//������������

extern FEED_INFO_t feed_now;
extern int time_zone_sec;
extern uint8_t feed_flag;
extern uint8_t feeding_flag;			//����ιʳ��־
extern uint8_t feed_key;
extern uint8_t calender_flag;
extern uint8_t time_sync;
extern uint8_t lock_led;
extern uint8_t lock_time;				//������������
extern uint8_t led_test;
extern uint8_t low_power;
extern uint8_t dc_flag;
extern uint8_t connect_flag;
extern uint8_t sleep_tick;
extern uint8_t led_tick;
extern uint8_t restore_flag;
extern uint8_t app_weight;
extern uint8_t err_type;				//ιʳ�쳣

extern uint8_t feed_key_flag;			//ιʳ�������±�־
extern uint8_t record_key_flag;			//¼���������±�־
extern uint8_t ret_key_flag;			//��λ�������±�־
extern uint8_t lock_key_flag;			//�����������±�־
extern uint8_t relock_key_flag;			//�������������±�־
extern uint8_t debug_flag;
extern uint8_t check_voltage_flag;		//��ȡ������־
extern uint8_t adc_reverse_flag;		//ADC��ת��־
extern uint8_t read_adc_flag;			//ADC��ⷴת��־ 0:������ʼ��� 1�����ж���ֵ
extern volatile uint8_t byte_flag;
extern uint8_t pair_flag;
extern uint8_t save_flag;


void feed_check(void);
void feed_run_callback(void);
void log_upload_callback(void);
void plan_upload_callback(void);
uint32_t get_timestamp_zero(void);										//��ȡ0ʱ��ʱ���
void send_feed_error(uint32_t t_zero_t, uint8_t err_type_t);		//����ιʳ�쳣
void send_feed_record(uint32_t t_zero_t, uint8_t feed_weight);		//����ιʳ�������
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

//�����Ҫʹ��GPIO���е��ԣ���Ҫ�������
#define GPIO_DBG_MSG					0
//UARTʹ�ܿ��ƺ�
#define UART_PRINTF_EN					1
//����Ӳ�����Կ���
#define DEBUG_HW						0




 
/*******************************************************************************
 *#############################################################################*
 *								APPLICATION MACRO CTRL                         *
 *#############################################################################*
 *******************************************************************************/
 
//���Ӳ������¿���
#define UPDATE_CONNENCT_PARAM  			1

//��С���Ӽ��
#define BLE_UAPDATA_MIN_INTVALUE		20
//������Ӽ�� 
#define BLE_UAPDATA_MAX_INTVALUE		40
//����Latency
#define BLE_UAPDATA_LATENCY				0
//���ӳ�ʱ
#define BLE_UAPDATA_TIMEOUT				600


//�豸����
#define APP_DFLT_DEVICE_NAME           ("TY_wen")

//#define USER_MAC_DEFAULT		"DC234D6767EC"
//gzjk71f9dfdc4e7b	
//UVu8IDId5RHtdX2KNzGwCkUfzgV6xSal	
//DC234D6767EC

 //�㲥������
#define APP_TUYA_ADV_DATA       "\x02\x01\x06\x03\x02\x01\xA2\x14\x16\x01\xA2\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
#define APP_TUYA_ADV_DATA_LEN    (28)

//ɨ����Ӧ������
#define APP_SCNRSP_DATA        "\x03\x09\x54\x59\x19\xFF\xD0\x07\x00\x03\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" //
#define APP_SCNRSP_DATA_LEN     (30)


//�㲥��������
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
unsigned char BCD2HEX(unsigned char bcd_data); //BCDתΪHEX�ӳ��� 
unsigned char HEX2BCD(unsigned char hex_data); //HEXתΪBCD�ӳ��� 

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
