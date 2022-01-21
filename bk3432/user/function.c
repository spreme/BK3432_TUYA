#include "function.h"
#include "wdt.h"
#include "rwip.h"

uint8_t lock_flag = 1;				//设备锁标志
uint8_t key_lock = 0;				//锁按键标志
uint8_t key_scan_flag = 0;			//正在检查按键标志
uint8_t key_flag = 0;				//按键触发类型

//uint32_t rtc_timestamp = 1;			//时间戳时间

uint8_t beep_flag = 0;				//蜂鸣器响标志
uint8_t keep_dowm_flag = 0;			//按键长按标志

uint8_t feed_error = 0;				//喂食异常标志
uint8_t calender_flag = 0;			//检测标志
uint8_t connect_flag = UNBONDING_UNCONN;		//连接状态
uint8_t feed_key = 0;				//按键喂食类型，1按键喂食，0无喂食，10无授权不喂食
uint8_t restore_flag = 0;			//复位标志
uint8_t dc_flag = 1;				//电池供电标志(0:电池供电)
uint8_t low_power = 0;				//电池低电标志
uint8_t led_test = 0;				//产测模式led灯模式
uint8_t lock_led = 0;				//锁键标志
uint8_t lock_time = 0;				//锁键按键次数
uint8_t check_voltage_flag = 0;		//读取电量标志
uint8_t adc_reverse_flag = 0;		//ADC反转标志
//uint8_t lock = 0;					//喂食锁

FEED_PLAN_t ty_plan;
FEED_LOG_INFO_t ty_logs;

int time_zone_sec = 0;	//时区按秒计
uint8_t time_sync = 0;
uint8_t app_weight = 0;

#if BATTERY_FUNC || MOTOR_REVERSE_ADC
uint32_t suble_util_shell_sort(int* buf, int size)
{
    int i;
    int j;
    int temp;  
    int gap;  //步长
    for (gap = size / 2; gap >= 1; gap /= 2) {  // 步长初始化为数组长度的一半，每次遍历后步长减半
        for (i = 0 + gap; i < size; i += gap) {   //对步长为gap的元素进行直插排序，当gap为1时，就是直插排序
            temp = buf[i];  //备份a[i]的值
            j = i - gap;  //j初始化为i的前一个元素（与i相差gap长度）
            while (j >= 0 && buf[j] > temp) {
                buf[j + gap] = buf[j];  //将在a[i]前且比temp的值大的元素向后移动一位
                j -= gap;
            }
            buf[j + gap] = temp; 
        }
    }
    return 0;
}

#endif
#if BATTERY_FUNC
uint16_t voltage[BATTERY_SAMPLE_TIME] = {0};
uint32_t adc_value_sum = 0;
//uint32_t adc_value = 0;
//显示电量
void disp_voltage(void)
{
	static uint8_t i = 0;

	dc_flag = gpio_get_input(CHARGE_DET);
	
	if(i < BATTERY_SAMPLE_TIME)
	{
		voltage[i] = adc_get_value(BATTERY_CHAN);
	}
	else
	{
        //排序
		suble_util_shell_sort((void*)voltage, BATTERY_SAMPLE_TIME);
		//求和
        for(uint8_t idx = 5; idx <= 14; idx++) {
            adc_value_sum += voltage[idx];
        }
		
        //求值
        int adc_value = adc_value_sum / 10;
        //计算百分比
        int percent = (adc_value - BATTERY_MIN) * 100 / (BATTERY_MAX - BATTERY_MIN);
        //限制百分比的合理范围
        if(percent > 100) {
            percent = 100;
        }
        if(percent <= 1) {
            percent = 1;
        }
		
        //上报
        UART_PRINTF("battery_percent_report: %d%% \n", percent);
		check_voltage_flag = 0;

		if(percent <= 20)
		{
			low_power = 1;
		}
		else
		{
			low_power = 0;
		}

		connect_flag = tuya_ble_connect_status_get();
		if(connect_flag == BONDING_CONN)
		{
			demo_dp_t dp_test;
			memset(&dp_test, 0, sizeof(demo_dp_t));
			
			dp_test.dp_id = 0xb;
			dp_test.dp_type = 0x2;
			dp_test.dp_data_len = 0x4;
			dp_test.dp_data[3] = (uint8_t)percent;
			
			tuya_ble_dp_data_report((void*)&dp_test, dp_test.dp_data_len+3);	//上传电量
			
			Delay_ms(100);
			
			memset(&dp_test, 0, sizeof(demo_dp_t));
			
			dp_test.dp_id = 0xc;
			dp_test.dp_type = 0x1;
			dp_test.dp_data_len = 0x1;
			dp_test.dp_data[0] = dc_flag;
			
			tuya_ble_dp_data_report((void*)&dp_test, dp_test.dp_data_len+3);	//上传外电检测
		}
		i = 0;
		adc_value_sum = 0;	
	}
	i++;
	
}

#endif
#if MOTOR_REVERSE_ADC
uint16_t adc_voltage[20] = {0};
uint32_t adc_reverse_sum = 0;
uint8_t read_adc_flag = 0;		//ADC检测反转标志 0:读数开始检测 1：已有读数值

void adc_reverse_init(void)
{
	adc_init(MOTOR_ADC_CHAN,1);
}
//
void adc_reverse(void)
{
	static uint8_t i = 0;
	static uint8_t reverse_coun = 0;
	static uint8_t reverse_adc_cont = 0;
	static uint16_t reverse_adc = 0;

	if(read_adc_flag == 1)
	{
		reverse_coun = 0;
		reverse_adc_cont = 0;
		reverse_adc = 0;
		read_adc_flag = 0;
	}
	
	if(i < 20)
	{
		adc_voltage[i] = adc_get_value(MOTOR_ADC_CHAN);
	}
	else
	{
        //排序
		suble_util_shell_sort((void*)adc_voltage, 20);
		//求和
        for(uint8_t idx = 5; idx <= 14; idx++) {
            adc_reverse_sum += adc_voltage[idx];
        }
		
        //求值
        int adc_value = adc_reverse_sum / 10;
		
		if(reverse_adc == 0)
		{
			if(adc_value > 0)
			{
				reverse_adc_cont++;
				reverse_adc = reverse_adc + adc_value;
				if(reverse_adc_cont >= 6)
				{
					reverse_adc = reverse_adc / 6;
				}
			}
		}
		else
		{
			if(adc_value > reverse_adc)
			{
				reverse_coun++;
				if(reverse_coun >= 5)
				{
					adc_reverse_flag = 1;
				}
			}
			else
				reverse_coun = 0;
		}
        UART_PRINTF("adc_reverse: %d \n", adc_value);

		i = 0;
		check_voltage_flag = 0;
		adc_reverse_sum = 0;	
	}
	i++;
	
}

#endif
void play_record_control(void)
{
	gpio_set(SOUND_PLAY, PLAYER_ON);
	Delay_ms(100);
	gpio_set(SOUND_PLAY, PLAYER_OFF);
}

void record_reset_control(void)
{
	gpio_set(SOUND_REC, RECORD_ON);
	#ifdef NEW_RECORD_IC
	Delay_ms(1300);
	#else
	Delay_ms(600);	
	#endif
	gpio_set(SOUND_REC, RECORD_OFF);
}

void led_control(uint8_t link_led, uint8_t red_led, uint8_t led_level)
{
	#ifndef NO_LED_E
	static uint8_t old_led_level = 0;
	
	if(led_level >= old_led_level || led_level == 0)
	{
//		UART_PRINTF("########## led_level:%d \r\n",led_level);
		old_led_level = led_level;
		
		switch(link_led)
		{
			case LED_ON:
				SET_LED_ON(LED_GREEN);

				break;
			
			case LED_OFF:
				SET_LED_OFF(LED_GREEN);
				break;
		}
		
		switch(red_led)
		{
			case LED_ON:
				SET_LED_ON(LED_RED);
				break;
			
			case LED_OFF:
				SET_LED_OFF(LED_RED);
				break;
		}	
	}
	#endif
}

#ifdef BACKLIGHT_CONTROL
PWM_DRV_DESC timer_desc_3;
void update_backlight(uint8_t light)
{
	timer_desc_3.duty_cycle = light;
	pwm_set_duty(&timer_desc_3);
}
void backlight_init(void)
{	
	timer_desc_3.channel = 4;
	timer_desc_3.mode = 1<<0 | 0<<1 | 0<<2 | 0<<4;
	timer_desc_3.end_value = 6;
	timer_desc_3.duty_cycle = 0;
	pwm_init(&timer_desc_3);
}
void set_backlight(uint8_t *pwm, uint8_t up)
{
	if(up) 
	{
		if(*pwm <= 6)
			*pwm += 1;
	} 
	else 
	{
		if (*pwm > 1)
			*pwm -= 1;
	}
		
	update_backlight(*pwm);
}
#endif

void printf_flash_info(void)
{
	uint8_t a;
	
	#ifndef NO_FEED_PLAN
	for(a = 0; a < 4;a++)
	{
		UART_PRINTF("%02X-%02X-%02X-%02X-%02X\n",ty_plan.plans[a].week,ty_plan.plans[a].hour,
			ty_plan.plans[a].min, ty_plan.plans[a].weight, ty_plan.plans[a].flag);	
	}
	#endif
	UART_PRINTF("ty_plan.lock:%d\n",ty_plan.lock);	
	UART_PRINTF("RTC_flag:%d\n",ty_plan.RTC_flag);	
	UART_PRINTF("RTC_zone_f:%d\n",ty_plan.RTC_zone_f);	
	UART_PRINTF("RTC_zone:%d\n",ty_plan.RTC_zone);	
	UART_PRINTF("reset_day:%d\n",ty_plan.reset_day);	
	UART_PRINTF("record_time:%d\n",ty_plan.record_time);	
}

void save_flash(enum FLASH_STYE flash_type)
{
	UART_PRINTF("################  save flash flash_type:%d\n",flash_type);	
	if(flash_type == FLASH_PLAN)
	{
		tuya_ble_nv_erase(BLE_PLAN_ADDR, 512);
		tuya_ble_nv_write(BLE_PLAN_ADDR, (uint8_t *)&ty_plan, sizeof(FEED_PLAN_t));
//		flash_erase_sector(FLASH_SPACE_TYPE_MAIN, BLE_PLAN_ADDR / 4);
//		flash_write(FLASH_SPACE_TYPE_MAIN, BLE_PLAN_ADDR / 4, sizeof(FEED_PLAN_t), (uint8_t *)&ty_plan);
	}
	else if(flash_type == FLASH_LOG)
	{
		tuya_ble_nv_erase(BLE_SAVE_ADDR, 512);
		tuya_ble_nv_write(BLE_SAVE_ADDR, (uint8_t *)&ty_logs, sizeof(FEED_LOG_INFO_t));
//		flash_erase_sector(FLASH_SPACE_TYPE_MAIN, BLE_SAVE_ADDR / 4);
//		flash_write(FLASH_SPACE_TYPE_MAIN, BLE_SAVE_ADDR / 4, sizeof(FEED_LOG_INFO_t), (uint8_t *)&ty_logs);	
	}
}

void PCF8563_init(void)
{
	#ifdef RTC_TIME
	tuya_ble_time_struct_data_t currTime;
	//软件I2C初始化
	i2cs_init();
	
//	if(PCF8563_Check() == 0)
	{
		u8 dat;
		tuya_ble_time_struct_data_t t_struct;
		uint32_t time_stamp_t = 0;
		
		UART_PRINTF("PCF8563_Check success @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		PCF8563_Start();
		
		tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
		if(ty_plan.RTC_flag == 0)
		{
			PCF8563_Set_Times(PCF_Format_BIN, PCF_Century_20xx, 21,1,1,5,00,00);
		}
		else
		{
			PCF8563_GetDate(PCF_Format_BIN, &dat, &t_struct);
			PCF8563_GetTime(PCF_Format_BIN, &t_struct);
			time_stamp_t = tuya_ble_mytime_2_utc_sec(&t_struct, 0);
			
			if(ty_plan.RTC_zone_f == 1)
			{
				time_zone_RTC_rec = ty_plan.RTC_zone  * 3600;
			}
			else
			{
				time_zone_RTC_rec = -ty_plan.RTC_zone * 3600;
			}
		
			UART_PRINTF("ty_plan.RTC_zone @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@1:%d",ty_plan.RTC_zone);
			UART_PRINTF("ty_plan.RTC_zone_f @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@1:%d",ty_plan.RTC_zone_f);
			UART_PRINTF("time_zone_RTC_rec @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@1:%d",time_zone_RTC_rec);

			time_stamp_t = time_stamp_t - time_zone_RTC_rec;
			utc_set_clock(time_stamp_t);
			UART_PRINTF("utc_set_clock @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@1:%ld",time_stamp_t);

			utc_update();
			
			UART_PRINTF("Get-time to rtc: %04d-%02d-%02d %02d:%02d:%02d weekday:%02d",
					t_struct.nYear, t_struct.nMonth, t_struct.nDay,
					t_struct.nHour, t_struct.nMin, t_struct.nSec, t_struct.DayIndex);

			time_sync = 1;
		}
		time_zone_sec = time_zone_RTC_rec;
		restart_time_set(time_stamp_t);
	}
	#endif	
}

