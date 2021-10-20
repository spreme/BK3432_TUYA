/**
 ****************************************************************************************
 *
 * @file app_tuya.c
 *
 * @brief tuya Application
 *
 * @auth  yonghui.gao
 *
 * @date  2020.01.09
 *
 * 
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app_tuya_service.h"              // tuya Application Module Definitions
#include "app_tuya.h" 
#include "app.h"                    // Application Definitions
#include "app_task.h"             // application task definitions
#include "tuya_service_task.h"           // health thermometer functions
#include "co_bt.h"
#include "prf_types.h"             // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "tuya_service.h"
#include "ke_timer.h"
#include "uart.h"
#include "tuya_ble_log.h"
#include "tuya_ble_type.h"
#include "tuya_ble_api.h"
#include "tuya_ble_main.h"
#include "app_tuya_ota.h"
#include "uart2.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static uint8_t log_tick = 0;


tuya_ble_device_param_t device_param = {0};

//DC234D2A5728	gzjk3798d2b9b201	HVLI88EQA9moP0p7MUOBAOrnS56YLb2e

static const char auth_key_test[] = "HVLI88EQA9moP0p7MUOBAOrnS56YLb2e";
static const char device_id_test[] = "gzjk3798d2b9b201";
static const uint8_t mac_test[6] = {0xDC,0x23,0x4D,0x2A,0x57,0x28};


static uint8_t dp_data_array[200];
static uint16_t dp_data_len = 0;


#define APP_CUSTOM_EVENT_1  1
#define APP_CUSTOM_EVENT_2  2
#define APP_CUSTOM_EVENT_3  3
#define APP_CUSTOM_EVENT_4  4
#define APP_CUSTOM_EVENT_5  5


typedef struct {
    uint8_t data[50];
} custom_data_type_t;

void custom_data_process(int32_t evt_id,void *data)
{
    custom_data_type_t *event_1_data;
    TUYA_APP_LOG_DEBUG("custom event id = %d",evt_id);
    switch (evt_id)
    {
        case APP_CUSTOM_EVENT_1:
            event_1_data = (custom_data_type_t *)data;
            TUYA_APP_LOG_HEXDUMP_DEBUG("received APP_CUSTOM_EVENT_1 data:",event_1_data->data,50);
            break;
        case APP_CUSTOM_EVENT_2:
            break;
        case APP_CUSTOM_EVENT_3:
            break;
        case APP_CUSTOM_EVENT_4:
            break;
        case APP_CUSTOM_EVENT_5:
            break;
        default:
            break;

    }
}

custom_data_type_t custom_data;

void custom_evt_1_send_test(uint8_t data)
{    
    tuya_ble_custom_evt_t event;
    
    for(uint8_t i=0; i<50; i++)
    {
        custom_data.data[i] = data;
    }
    event.evt_id = APP_CUSTOM_EVENT_1;
    event.custom_event_handler = (void *)custom_data_process;
    event.data = &custom_data;
    tuya_ble_custom_event_send(event);
}


void log_upload_callback(void)
{
	int ret = -1;
	
	connect_flag = tuya_ble_connect_status_get();
	
	if(connect_flag == BONDING_CONN)
	{
		tuya_ble_nv_read(BLE_SAVE_ADDR, (uint8_t *) &ty_logs, sizeof(FEED_LOG_INFO_t));

		if(ty_logs.logs[log_tick][0] != 0)
		{
			if(ty_logs.logs[log_tick][2] == 0)	//未上传过的记录，为0
			{
				demo_dp_t dp_test;
				memset(&dp_test, 0, sizeof(demo_dp_t));
				
				dp_test.dp_id = 0xf;
				dp_test.dp_type = 0x2;
				dp_test.dp_data_len = 0x4;
				dp_test.dp_data[3] = ty_logs.logs[log_tick][1];
				
				ret = tuya_ble_dp_data_with_time_report(ty_logs.logs[log_tick][0], (void*)&dp_test, dp_test.dp_data_len+3);
				
				if(ret == TUYA_BLE_SUCCESS)
				{
//					UART_PRINTF("send feed_log success: %ld-%ld-%ld", ty_logs.logs[log_tick][0], ty_logs.logs[log_tick][1], ty_logs.logs[log_tick][2]);

					ty_logs.logs[log_tick][2] = 1;	//每当成功上传，置1
				}
			}
			
			log_tick++;
			if(log_tick >= FEED_RECORD_NUM)
			{
				save_flash(FLASH_LOG);
//				ke_timer_clear(LOG_UPLOAD_TASK, TASK_APP);
			}
			else
			{
				ke_timer_set(LOG_UPLOAD_TASK, TASK_APP, 5);			
			}
		}
		else
		{
			save_flash(FLASH_LOG);
//			ke_timer_clear(LOG_UPLOAD_TASK, TASK_APP);
		}
	}
	else
	{
		save_flash(FLASH_LOG);
//		ke_timer_clear(LOG_UPLOAD_TASK, TASK_APP);
	}
}

void plan_upload_callback(void)
{
	#ifndef NO_FEED_PLAN
	connect_flag = tuya_ble_connect_status_get();
	
	if(connect_flag == BONDING_CONN)
	{
		FEED_INFO_t feed_p;
		int i = 0;
		demo_dp_t dp_test;
		memset(&dp_test, 0, sizeof(demo_dp_t));
		
		dp_test.dp_id = 0x1;
		dp_test.dp_type = 0x0;
		dp_test.dp_data_len = 0;
		
		tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
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
		
//		ke_timer_clear(UPLOAD_PLAN_TASK, TASK_APP);
//		ke_timer_clear(LOG_UPLOAD_TASK, TASK_APP);
		ke_timer_set(LOG_UPLOAD_TASK, TASK_APP, 5);

	}
//	else
//		ke_timer_clear(UPLOAD_PLAN_TASK, TASK_APP);
	#endif
}

static void ble_protocol(uint8_t *ble, uint32_t len)
{
	demo_dp_t dp_re;
	memset(&dp_re, 0, sizeof(demo_dp_t));
	
	memcpy(&dp_re, ble, len);
	
	UART_PRINTF("ble cmd: %x", dp_re.dp_id);
	
	switch(dp_re.dp_id)
	{
		#ifndef NO_FEED_PLAN
		case 0x1:	//设置喂食计划
			tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
			memset(&ty_plan.plans, 0, sizeof(ty_plan.plans));
			memcpy(&ty_plan.plans, dp_re.dp_data, dp_re.dp_data_len);
			UART_PRINTF("Change Feedplans");
			save_flash(FLASH_PLAN);
		break;
		#endif
		
		case 0x3:	//APP手动喂食
			app_weight = dp_re.dp_data[3];
			feed_flag = APP_FEED;
			break;
		
		default:
			break;
	}
}


//static uint16_t sn = 0;
//static uint32_t time_stamp = 1587795793;
static void tuya_cb_handler(tuya_ble_cb_evt_param_t* event)
{
//    int16_t result = 0;
    switch (event->evt)
    {
    case TUYA_BLE_CB_EVT_CONNECTE_STATUS:
        TUYA_APP_LOG_INFO("received tuya ble conncet status update event,current connect status = %d",event->connect_status);
        break;
    case TUYA_BLE_CB_EVT_DP_WRITE:
        dp_data_len = event->dp_write_data.data_len;
        memset(dp_data_array,0,sizeof(dp_data_array));
        memcpy(dp_data_array,event->dp_write_data.p_data,dp_data_len);        
        TUYA_APP_LOG_HEXDUMP_DEBUG("received dp write data :",dp_data_array,dp_data_len);
        ble_protocol(event->dp_write_data.p_data, event->dp_write_data.data_len);
	
        tuya_ble_dp_data_report(dp_data_array,dp_data_len);
        break;
    case TUYA_BLE_CB_EVT_DP_DATA_REPORT_RESPONSE:
        TUYA_APP_LOG_INFO("received dp data report response result code =%d",event->dp_response_data.status);

        break;
    case TUYA_BLE_CB_EVT_DP_DATA_WTTH_TIME_REPORT_RESPONSE:
        TUYA_APP_LOG_INFO("received dp data report response result code =%d",event->dp_with_time_response_data.status);
    
        break;
    case TUYA_BLE_CB_EVT_DP_DATA_WITH_FLAG_REPORT_RESPONSE:

        break;
    case TUYA_BLE_CB_EVT_DP_DATA_WITH_FLAG_AND_TIME_REPORT_RESPONSE:
       
        break;
    case TUYA_BLE_CB_EVT_UNBOUND:
        
//        TUYA_APP_LOG_INFO("received unbound req");
		restore_flag = 1;
        break;
    case TUYA_BLE_CB_EVT_ANOMALY_UNBOUND:
        
        TUYA_APP_LOG_INFO("received anomaly unbound req");

        break;
    case TUYA_BLE_CB_EVT_DEVICE_RESET:
        
        TUYA_APP_LOG_INFO("received device reset req");
		restore_flag = 1;
        break;
    case TUYA_BLE_CB_EVT_DP_QUERY:
        TUYA_APP_LOG_INFO("received TUYA_BLE_CB_EVT_DP_QUERY event");
        if(dp_data_len>0)
        {
            tuya_ble_dp_data_report(dp_data_array,dp_data_len);
        }
        break;
    case TUYA_BLE_CB_EVT_TIME_STAMP:
//        TUYA_APP_LOG_INFO("received unix timestamp : %s ,time_zone : %d",event->timestamp_data.timestamp_string,event->timestamp_data.time_zone);
			if(strlen((const char *)event->timestamp_data.timestamp_string) >= 10)
			{
				int time_stamp_t = 0;
				int i = sscanf((const char *)event->timestamp_data.timestamp_string, "%10d", &time_stamp_t);
				TUYA_APP_LOG_DEBUG("GET INT SEC: %d", time_stamp_t);
//				suble_update_timestamp(time_stamp_t);
				utc_set_clock(time_stamp_t);
				UART_PRINTF("utc_set_clock @@@@@@@@@@@@@@@@@2:%ld \r\n",time_stamp_t);
				utc_update();
				
				time_zone_sec = event->timestamp_data.time_zone / 100 * 3600;
				time_sync = 1;
				
				#ifdef RTC_TIME
				tuya_ble_time_struct_data_t t_struct;
				
				time_zone_RTC_rec = time_zone_sec;
				time_stamp_t = time_stamp_t + time_zone_RTC_rec;
				tuya_ble_utc_sec_2_mytime(time_stamp_t, &t_struct, 0);
				
				PCF8563_SetDate(PCF_Format_BIN, PCF_Century_20xx, &t_struct);
				PCF8563_SetTime(PCF_Format_BIN, &t_struct);
				
				UART_PRINTF("set-time to rtc: %04X-%02X-%02X %02X:%02X:%02X weekday:%02X \r\n",
					t_struct.nYear, t_struct.nMonth, t_struct.nDay,
					t_struct.nHour, t_struct.nMin, t_struct.nSec, t_struct.DayIndex);

//				set_RTC_time(&t_struct);
				tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
				if(event->timestamp_data.time_zone >= 0)
				{
					ty_plan.RTC_zone_f = 1;
					ty_plan.RTC_zone = event->timestamp_data.time_zone / 100;
				}
				else
				{
					ty_plan.RTC_zone_f = 0;
					ty_plan.RTC_zone = -event->timestamp_data.time_zone / 100;
				}
				UART_PRINTF("ty_plan.RTC_zone @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2:%d \r\n",ty_plan.RTC_zone);

				ty_plan.RTC_flag = 1;
				save_flash(FLASH_PLAN);
				#endif
				
			}
			log_tick = 0;
//			ke_timer_clear(UPLOAD_PLAN_TASK, TASK_APP);
			ke_timer_set(UPLOAD_PLAN_TASK, TASK_APP, 5);

		break;
    case TUYA_BLE_CB_EVT_TIME_NORMAL:

        break;
    default:
        TUYA_APP_LOG_WARNING("app_tuya_cb_queue msg: unknown event type 0x%04x \r\n",event->evt);
        break;
    }
}

static void tuya_ble_app_init(void)
{
    device_param.device_id_len = TEST_UID;    //If use the license stored by the SDK,initialized to 0, Otherwise 16.
    
    if(device_param.device_id_len==16)
    {
        memcpy(device_param.auth_key,(void *)auth_key_test,AUTH_KEY_LEN);
        memcpy(device_param.device_id,(void *)device_id_test,DEVICE_ID_LEN);
        memcpy(device_param.mac_addr.addr,mac_test,6);
        device_param.mac_addr.addr_type = TUYA_BLE_ADDRESS_TYPE_RANDOM;
    }
    device_param.product_id_len = 8;
    if(device_param.product_id_len>0)
    {        
        device_param.p_type = TUYA_BLE_PRODUCT_ID_TYPE_PID;
        memcpy(device_param.product_id,APP_PRODUCT_ID,8);
    }
    
    device_param.firmware_version = TY_APP_VER_NUM;
    device_param.hardware_version = TY_HARD_VER_NUM;

    tuya_ble_sdk_init(&device_param);
    tuya_ble_callback_queue_register(tuya_cb_handler);

    tuya_ota_init();
	TUYA_APP_LOG_INFO("current version : %s",TY_APP_VER_STR);
}

void app_tuya_init(void)
{
    app_tuya_service_init();
    tuya_ble_app_init();
	
}








