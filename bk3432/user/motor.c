#include "gpio.h"
#include "rf.h"        // RF initialization
#include "user_config.h"
#include "tuya_ble_unix_time.h"
#include "wdt.h"
#include <stdio.h>
#include "utc_clock.h"
#include <stdlib.h>


uint8_t feed_flag = 0;
FEED_INFO_t feed_now;

static uint8_t min_before = 99;
static uint8_t feed_count = 0;
static uint32_t key_count = 0;
static uint32_t motor_timeout = 0;
uint8_t err_type = 0;

uint8_t feeding_flag = 0;			//正在喂食标志
uint8_t restart_time[2] = {0};
uint8_t feed_status = 0;

void restart_check(tuya_ble_time_struct_data_t *time);

#ifndef NO_FEED_PLAN				//没有喂食计划功能
static int week_check(uint8_t wd_now, uint8_t wd_tar)
{
//	UART_PRINTF("week_check, wd_now = %x, wd_tar = %x", wd_now, wd_tar);
	if(wd_tar == 0)											//仅执行一次
		return 1;
	else if((wd_now == 1) && (((wd_tar >> 6) & 1) == 1))	//周一
		return 1;
	else if((wd_now == 2) && (((wd_tar >> 5) & 1) == 1))	//周二
		return 1;
	else if((wd_now == 3) && (((wd_tar >> 4) & 1) == 1))	//周三
		return 1;
	else if((wd_now == 4) && (((wd_tar >> 3) & 1) == 1))	//周四
		return 1;
	else if((wd_now == 5) && (((wd_tar >> 2) & 1) == 1))	//周五
		return 1;
	else if((wd_now == 6) && (((wd_tar >> 1) & 1) == 1))	//周六
		return 1;
	else if((wd_now == 0) && (((wd_tar >> 0) & 1) == 1))	//周日
		return 1;
	else
		return 0;
}

int feed_scan(tuya_ble_time_struct_data_t *time, FEED_INFO_t *feed_t)
{
	uint8_t i = 0;//, t = 0;
	uint32_t time_now = 0;
	uint32_t time_feed = 0;
	FEED_INFO_t feed;
	uint8_t ret = 0;
	
	UART_PRINTF("Feed Scan now");
	
	time_now = time->nHour * 60 + time->nMin;
	
	tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
	for(i=0; i<10; i++)
	{
		memset(&feed, 0, sizeof(FEED_INFO_t));
		memcpy(&feed, &ty_plan.plans[i], sizeof(FEED_INFO_t));
		
//		UART_PRINTF("%02x%02x%02x%02x%02x", feed.week, feed.hour, feed.min, feed.weight, feed.flag);
		
		if(feed.flag == 0)
			continue;
		
		if(feed.weight == 0)
			continue;
		
		ret = week_check(time->DayIndex, feed.week);
        if(ret == 0)
            continue;
		
		time_feed = feed.hour * 60 + feed.min;
		
		if(time_feed == time_now)
		{
			memcpy(feed_t, &feed, 5);
			if(feed.week == 0)	//仅执行1次
			{
				feed.flag = 0;
				memcpy(&ty_plan.plans[i], &feed, sizeof(FEED_INFO_t));

				save_flash(FLASH_PLAN);
			}
			
			return AUTO_FEED;
		}
	}
	
	if(ty_plan.reset_day != time->nDay)
	{
		if((restart_time[0] == time->nHour) && (restart_time[1] == time->nMin))
		{
			restart_check(time);
		}
	}
	
	
	return NONE_FEED;
}
#endif

void save_log(uint32_t time, uint32_t weight)
{
	int i;
	uint32_t res[3] = {0};

	res[0] = time;
	res[1] = weight;
	res[2] = 0;
	
	tuya_ble_nv_read(BLE_SAVE_ADDR, (uint8_t *) &ty_logs, sizeof(FEED_LOG_INFO_t));

	for(i=48; i>=0; i--)
	{
		memcpy(ty_logs.logs[i+1], ty_logs.logs[i], sizeof(res));
	}

	memcpy(ty_logs.logs[0], &res, sizeof(res));
	
	UART_PRINTF("Save LOG\r\n");
	save_flash(FLASH_LOG);
}

void motor_contorl(enum MOTOR_CONTROL_E motor_c)
{
	switch(motor_c)
	{
		case MOTOR_PULL:
		{
			gpio_set(MOTOR_PIN_P, 1);
			#ifdef MOTOR_REVERSE
			gpio_set(MOTOR_PIN_N, 0);
			#endif
		}
		break;
		
		case MOTOR_PUSH:
		{
			gpio_set(MOTOR_PIN_P, 0);
			#ifdef MOTOR_REVERSE
			gpio_set(MOTOR_PIN_N, 1);
			#endif
		}
		break;
		
		case MOTOR_STOP:
		{
			gpio_set(MOTOR_PIN_P, 0);
			#ifdef MOTOR_REVERSE
			gpio_set(MOTOR_PIN_N, 0);
			#endif
		}
		break;
		
		default:
			break;			
	}
}

void feed_run_callback(void)	//按步骤，非阻塞式喂食
{
#ifdef IR_FEED_TUBE
	uint8_t i = 0;
	static uint16_t IR_time = 0;
#endif
	static uint8_t reverse_flag = 0;
	
	//播放音频
	if(feed_status == FEED_STEP_SOUND)
	{
		tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
		if(ty_plan.record_time != 0)	//有录音
		{				
			gpio_set(SOUND_PLAY, PLAYER_ON);
			Delay_ms(100);
			gpio_set(SOUND_PLAY, PLAYER_OFF);
			
			feed_status = FEED_STEP_READY;
			feed_count = feed_now.weight;
		
			UART_PRINTF("play sound !!\r\n");
		}
		else	//无录音
		{
			feed_status = FEED_STEP_READY;
			feed_count = feed_now.weight;
		}
		
		
		UART_PRINTF("feed_status == FEED_STEP_SOUND !!\r\n");
		UART_PRINTF("record_time:%d\r\n", ty_plan.record_time);
		ke_timer_set(FEED_RUN_TASK, TASK_APP, ty_plan.record_time * 10);
		return;
	}
		
	//喂食份数
	if(feed_status == FEED_STEP_READY)
	{
		if(feed_count > 0)
		{
			key_count = 0;
			motor_timeout = MOTO_TIMEOUT;
			motor_contorl(MOTOR_PULL);
			feed_status = FEED_STEP_LEAVE_DET;
			UART_PRINTF("feed_count:%d\r\n", feed_count);
//			UART_PRINTF("feed_status == FEED_STEP_READY to  FEED_STEP_LEAVE_DET!!");
		}
		else
		{
			UART_PRINTF("FEED OVER\r\n");
			motor_contorl(MOTOR_STOP);
			feed_status = FEED_STEP_OVER;
			err_type = 0;
		}
		
		#ifdef MOTOR_REVERSE_ADC
		read_adc_flag = 1;
		#endif
		adc_reverse_flag = 0;
		ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
		return;
	}
	
	//喂食到位，喂食完一份
	if(feed_status == FEED_STEP_LEAVE_DET)
	{
		if(gpio_get_input(MOTOR_DET))
        {
            key_count++;
            if(key_count >= 5)
			{
				key_count = 0;
				motor_timeout = MOTO_TIMEOUT;
				feed_status = FEED_STEP_WAIT_DET;
				UART_PRINTF("feed_status == LEAVE_DET ERROR to WAIT_DET!!\r\n");
			}
        }

		motor_timeout--;
		if(motor_timeout == 0 || adc_reverse_flag)
		{
			#ifdef MOTOR_REVERSE
			if(reverse_flag < REVERSE_COUNT)
			{
				reverse_flag++;
				feed_status = FEED_STEP_REVERSE;
				
				ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
				
				UART_PRINTF("feed timeout reverse first !!\r\n");
				return;
			}
			#endif

			UART_PRINTF("MOTOR ERROR !!\r\n");
			feed_error = 1;
			err_type = ERROR_MOTOR_TIMEOUT;
			motor_contorl(MOTOR_STOP);
			feed_status = FEED_STEP_OVER;
		}
		
		ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
		return;
	}
	
	//喂食份数减少计算，超时设备异常检测
	if(feed_status == FEED_STEP_WAIT_DET)
	{
		if(gpio_get_input(MOTOR_DET) == 0)
		{
			#ifdef IR_FEED_TUBE
			//落粮对管堵塞检测
			if(gpio_get_input(IR_DET))
			{
				UART_PRINTF("MOTOR ERROR IR_error check!!\r\n");
				feed_status = FEED_STEP_ERROR_IR;
				
				motor_contorl(MOTOR_STOP);
				ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
				return;
			}
			#endif
		
			key_count++;
            if(key_count >= 5)
			{
				key_count = 0;
				feed_error = 0;
				feed_count--;
				reverse_flag = 0;
				feed_status = FEED_STEP_READY;
				
				ke_timer_set(FEED_RUN_TASK, TASK_APP, MOTOR_DELAY);
				
				UART_PRINTF("feed one success !!\r\n");
				return;
			}
		}

		motor_timeout--;
		if(motor_timeout == 0 || adc_reverse_flag)
		{
			#ifdef MOTOR_REVERSE
			if(reverse_flag < REVERSE_COUNT)
			{
				reverse_flag++;
				feed_status = FEED_STEP_REVERSE;
				
				ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
				
				UART_PRINTF("feed timeout reverse first !!\r\n");
				return;
			}
			#endif
			UART_PRINTF("MOTOR ERROR !!\r\n");
			feed_error = 1;
			err_type = ERROR_MOTOR_TIMEOUT;
			motor_contorl(MOTOR_STOP);
			feed_status = FEED_STEP_OVER;
		}
		
		ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
		return;
	}
	
#ifdef MOTOR_REVERSE
	if(feed_status == FEED_STEP_REVERSE_STOP)
	{
		motor_contorl(MOTOR_STOP);
		feed_status = FEED_STEP_READY;

		#ifdef MOTOR_REVERSE_ADC
		read_adc_flag = 1;
		#endif
		ke_timer_set(FEED_RUN_TASK, TASK_APP, REVERSE_STOP_TIME);
		return;
	}
	//电机反转
	if(feed_status == FEED_STEP_REVERSE)
	{
		motor_contorl(MOTOR_PUSH);
		#ifdef MOTOR_REVERSE_STOP
		feed_status = FEED_STEP_REVERSE_STOP;
		#else
		feed_status = FEED_STEP_READY;
		#endif

		ke_timer_set(FEED_RUN_TASK, TASK_APP, REVERSE_TIME);
		return;
	}
#endif

	
#ifdef IR_FEED_TUBE		
	if(feed_status == FEED_STEP_ERROR_IR)			//检测喂食堵粮延时5s确认堵粮
	{
		if(gpio_get_input(IR_DET))
		{
			IR_time++;
			if(IR_time > 500)
			{
				feed_count--;
				feed_status = FEED_STEP_OVER;
				err_type = ERROR_IR;
				feed_error = 2;
				UART_PRINTF("MOTOR ERROR IR_error !!\r\n");
			}
		}
		else
		{
			IR_time = 0;
			feed_status = FEED_STEP_WAIT_DET;
			return;
		}
	}	

	if(feed_status == FEED_STEP_IR_COUNT_DOWN)			//堵粮计时5分钟取消剩余份数
	{
		#ifdef IR_FEED_TUBE_LED
		gpio_set(IR_LED, 1);
		#endif /* end of IR_FEED_TUBE_LED */
		if(gpio_get_input(IR_DET) == 0)
		{
			IR_time++;
			if(IR_time > 30)						//防抖300ms
			{
				feed_status = FEED_STEP_SOUND;		//重新喂食
				IR_time = 0;
			}
		}
		else
		{
			IR_time = 0;
		}
	}
#endif
	
	if(feed_status == FEED_STEP_OVER)
	{
		if(time_sync == 1)
		{
			uint32_t t_zero = 0;
			tuya_ble_time_struct_data_t t_struct;
			
			memset(&t_struct, 0, sizeof(tuya_ble_time_struct_data_t));
//			t_zero = suble_get_timestamp();
			utc_update();
			t_zero = utc_get_clock();
			tuya_ble_utc_sec_2_mytime(t_zero, &t_struct, 0);
			
			adc_reverse_flag = 0;
			#ifdef MOTOR_REVERSE_ADC
			read_adc_flag = 1;
			#endif
			
			connect_flag = tuya_ble_connect_status_get();
		
			UART_PRINTF("@@@@@@@@@@@@@@@@@@@@@@@@@@@@ t_zero:%ld\r\n",t_zero);
			UART_PRINTF("feed_status == FEED_STEP_OVER !!\r\n");
			if(connect_flag == BONDING_CONN)			//连接状态
			{
				demo_dp_t dp_test;
				memset(&dp_test, 0, sizeof(demo_dp_t));
				
				send_feed_record(t_zero, feed_now.weight - feed_count);
				
				send_feed_error(t_zero, err_type);
				
				#ifndef NO_FEED_PLAN
				if(feed_now.week == 0)
				{
					FEED_INFO_t feed_p;
					int i = 0;
					memset(&dp_test, 0, sizeof(demo_dp_t));
					
					dp_test.dp_id = 0x1;
					dp_test.dp_type = 0x0;
					dp_test.dp_data_len = 0;
					
					for(i=0; i<10; i++)
					{
						memset(&feed_p, 0, sizeof(FEED_INFO_t));
						memcpy(&feed_p, &ty_plan.plans[i], sizeof(FEED_INFO_t));
						if(feed_p.weight > 0)
						{
							memcpy(dp_test.dp_data+dp_test.dp_data_len, &feed_p, sizeof(FEED_INFO_t));
							dp_test.dp_data_len += sizeof(FEED_INFO_t);
						}
					}
					
					if(dp_test.dp_data_len == 0)
						dp_test.dp_data_len = 1;
					
					tuya_ble_dp_data_report((void*)&dp_test, dp_test.dp_data_len+3);
				}
				#endif
			}
			else
			{
				save_log(t_zero, feed_now.weight - feed_count);
			}
			
			if(err_type == ERROR_IR)
			{
				feed_status = FEED_STEP_IR_COUNT_DOWN;
				feed_now.weight = feed_count;			//剩余未喂食份数
				
				ke_timer_set(FEED_RUN_TASK, TASK_APP, 1);
				return;
			}
		}
		
		ke_timer_clear(FEED_RUN_TASK, TASK_APP);
		feed_status = FEED_STEP_NONE;
		feeding_flag = 0;								//结束喂食
		reverse_flag = 0;

		#ifdef IR_FEED_TUBE_LED
		gpio_set(IR_LED, 0);
		#endif
	}
	
	return;
}

void feed_check(void)
{
	uint8_t res = 0;
	#ifdef RTC_TIME
	static uint16_t get_time_cont = 0;
	#endif
//	static uint8_t nSec_before = 99;

	if(feed_key == 1)
	{
		memset(&feed_now, 0, sizeof(FEED_INFO_t));
		feed_now.weight = 1;
		feed_now.flag = KEY_FEED;
		feed_flag = KEY_FEED;
		feed_key = 0;
	}
	
	#ifndef NO_FEED_PLAN
	if((calender_flag == 1) && (time_sync == 1))
	{
		calender_flag = 0;
		
		uint32_t t_now = 0;
		tuya_ble_time_struct_data_t t_struct;
		memset(&t_struct, 0, sizeof(tuya_ble_time_struct_data_t));
		
//		t_now = suble_get_timestamp() + time_zone_sec;
		utc_update();
		t_now = utc_get_clock() + time_zone_sec;
		
		tuya_ble_utc_sec_2_mytime(t_now, &t_struct, 0);
		
		UART_PRINTF("%04d-%02d-%02d %02d:%02d:%02d weekday:%02d \r\n",
						t_struct.nYear, t_struct.nMonth, t_struct.nDay,
						t_struct.nHour, t_struct.nMin, t_struct.nSec, t_struct.DayIndex);

		if(t_struct.nMin != min_before)
		{

			#ifdef RTC_TIME
			uint8_t dat = 0;
			
			get_time_cont++;
			if(get_time_cont >= 360)		
			{
				uint32_t time_stamp_t = 0;
				
//				time_stamp_t = get_RTC_time(&t_struct);
//				PCF8563_Start();
//				suble_flash_read(FLASH_FEED_INFO_ADDR, (u8 *)&ty_plan, sizeof(FEED_PLAN_t));

				PCF8563_GetDate(PCF_Format_BIN, &dat, &t_struct);
				PCF8563_GetTime(PCF_Format_BIN, &t_struct);
				time_stamp_t = tuya_ble_mytime_2_utc_sec(&t_struct, 0);
				
				UART_PRINTF("get-time to rtc: %04d-%02d-%02d %02d:%02d:%02d weekday:%02d \r\n",
					t_struct.nYear, t_struct.nMonth, t_struct.nDay,
					t_struct.nHour, t_struct.nMin, t_struct.nSec, t_struct.DayIndex);
	
				if(ty_plan.RTC_zone_f == 1)
				{
					time_zone_RTC_rec = ty_plan.RTC_zone  * 3600;
				}
				else
				{
					time_zone_RTC_rec = -ty_plan.RTC_zone * 3600;
				}
				
				time_stamp_t = time_stamp_t - time_zone_RTC_rec;
//				suble_update_timestamp(time_stamp_t);
				utc_set_clock(time_stamp_t);
				UART_PRINTF("utc_set_clock @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@3:%ld \r\n",time_stamp_t);
				utc_update();
				
				get_time_cont = 0;
			}
			#endif
			
			
			if(min_before != t_struct.nMin)
			{
				res = feed_scan(&t_struct, &feed_now);

				UART_PRINTF("feed scan res: %d \r\n", res);

				if(res == AUTO_FEED)
				{
#if defined DDV41L				
					feed_now.weight = 1;
#endif			
					feed_flag = AUTO_FEED;
				}
				min_before = t_struct.nMin;
			}
		}
		
	}
	#endif
		
	if(feed_flag != 0)
	{
		UART_PRINTF("get feed flag: %d \r\n", feed_flag);	
		
		if(feed_flag == APP_FEED)
		{
			memset(&feed_now, 0, sizeof(FEED_INFO_t));

#if defined FEED_ALWAYS_ONE
			feed_now.weight = 1;
#else
			feed_now.weight = app_weight;
#endif
			
			feed_now.flag = APP_FEED;
		}
		
		feed_flag = 0;

#ifdef IR_FEED_TUBE
		#ifdef IR_FEED_TUBE_LED
		gpio_set(IR_LED, 1);
		#endif /* end of IR_FEED_TUBE_LED */
		
		Delay_ms(10);
		if(gpio_get_input(IR_DET) == 0)
		{
			feed_status = FEED_STEP_SOUND;	//锁定喂食状态
		}
		else
		{
			feed_status = FEED_STEP_OVER;		//结束上报喂食结果，堵粮
			err_type = ERROR_IR;
			feed_error = 2;
			feed_count = feed_now.weight;
		}
#else
		feed_status = FEED_STEP_SOUND;	//锁定喂食状态
#endif	

		
		UART_PRINTF("feed weight: %d \r\n", feed_now.weight);
		feeding_flag = 1;								//正在喂食
		feed_run_callback();
	}
}

//uint32_t get_timestamp_zero(void)
//{
//	uint32_t t_zero = 0;
////	tuya_ble_time_struct_data_t t_struct;
//			
////	memset(&t_struct, 0, sizeof(tuya_ble_time_struct_data_t));
////	t_zero = suble_get_timestamp();
//	utc_update();
//	t_zero = utc_get_clock();
//	
//	
////	tuya_ble_utc_sec_2_mytime(t_zero, &t_struct, 0);
//	
//	return t_zero;
//}

void send_feed_record(uint32_t t_zero_t, uint8_t feed_weight)
{
	tuya_ble_status_t ret = TUYA_BLE_ERR_UNKNOWN;
	demo_dp_t dp_test;			
	
	memset(&dp_test, 0, sizeof(demo_dp_t));	
	dp_test.dp_id = 0xf;
	dp_test.dp_type = 0x2;
	dp_test.dp_data_len = 0x4;
	dp_test.dp_data[3] = feed_weight;

	ret = tuya_ble_dp_data_with_time_report(t_zero_t, (void*)&dp_test, dp_test.dp_data_len+3);
	if(ret != TUYA_BLE_SUCCESS)
	{
		ret = tuya_ble_dp_data_with_time_report(t_zero_t, (void*)&dp_test, dp_test.dp_data_len+3);
		if(ret != TUYA_BLE_SUCCESS)
		{
			save_log(t_zero_t, feed_weight);
			UART_PRINTF("send feed_weight err:%ld-%ld",t_zero_t, feed_weight);
		}
	}
}

void send_feed_error(uint32_t t_zero_t, uint8_t err_type_t)
{
	tuya_ble_status_t ret = TUYA_BLE_ERR_UNKNOWN;
	demo_dp_t dp_test;
	
	memset(&dp_test, 0, sizeof(demo_dp_t));	
	dp_test.dp_id = 0xe;
	dp_test.dp_type = 0x2;
	dp_test.dp_data_len = 0x4;
	dp_test.dp_data[3] = err_type_t;
	
	ret = tuya_ble_dp_data_with_time_report(t_zero_t, (void*) &dp_test, dp_test.dp_data_len+3);
	if(ret != TUYA_BLE_SUCCESS)
	{
		ret = tuya_ble_dp_data_with_time_report(t_zero_t, (void*) &dp_test, dp_test.dp_data_len+3);
		if(ret != TUYA_BLE_SUCCESS)
		{
			UART_PRINTF("send feed_error err");
		}
	}
}



void restart_time_set(int seed)
{
    srand(seed);
    restart_time[0] = rand() % 2 + 2;
    restart_time[1] = rand() % 40 + 5;
	
    UART_PRINTF("restart time set: %02d:%02d\r\n", restart_time[0], restart_time[1]);
}

void restart_check(tuya_ble_time_struct_data_t *time)
{
	uint8_t i = 0;
	FEED_INFO_t feed;
//	int ret = 0, t = 0;
	uint32_t time_now = 0;
	uint32_t time_feed = 0;
	
	time_now = time->nHour * 60 + time->nMin;

	#ifndef NO_FEED_PLAN
	tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
    for(i=0; i<FEED_MAX_NUM; i++)
	{
		memset(&feed, 0, sizeof(FEED_INFO_t));
		memcpy(&feed, &ty_plan.plans[i], sizeof(FEED_INFO_t));
		
		if(feed.flag == 0)
			continue;
		
		if(feed.weight == 0)
			continue;
		
        time_feed = feed.hour * 60 + feed.min;
		
		if(((time_feed-3) <= time_now) && (time_now <= (time_feed+3)))
		{
            restart_time[1] += 1;

            if(restart_time[1] >= 60)
            {
                restart_time[1] = 5;
                if(restart_time[0] < 3)
                    restart_time[0]++;
            }

            UART_PRINTF("time clash, change: %02d:%02d\r\n", restart_time[0], restart_time[1]);
            return;
        }
    }
	#endif

    UART_PRINTF("check over, system will restart\r\n");

    ty_plan.reset_day = time->nDay;
	save_flash(FLASH_PLAN);	
    wdt_enable(10);
}
