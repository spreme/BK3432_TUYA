/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ******** ********************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "rwip_config.h" // RW SW configuration

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "boot.h"      // boot definition
#include "rwip.h"      // RW SW initialization
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#include "timer.h"     // TIMER initialization
#include "icu.h"
#include "flash.h"
#include "uart.h"      	// UART initialization
#include "uart2.h" 
#include "flash.h"     // Flash initialization
//#include "led.h"       // Led initialization
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "rf.h"        // RF initialization
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT

#if (BLE_APP_PRESENT)
#include "app.h"       // application functions
#endif // BLE_APP_PRESENT

#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif
#include "reg_assert_mgr.h"
#include "BK3432_reg.h"
#include "RomCallFlash.h"
#include "gpio.h"
#include "pwm.h"
#include "app_task.h"
#include "ir.h"
//#include "oads.h"
#include "wdt.h"
#include "rtc.h"
#include "user_config.h"
#include "ke_timer.h"

/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */




/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

// Creation of uart external interface api
struct rwip_eif_api uart_api;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

static void Stack_Integrity_Check(void);

extern void code_sanity_check(void);

#if (UART_DRIVER)
void uart_rx_handler(uint8_t *buf, uint8_t len);
#endif
#if (UART2_DRIVER)
void uart2_rx_handler(uint8_t *buf, uint8_t len);
#endif
#if ((UART_PRINTF_EN) &&(UART_DRIVER || UART2_DRIVER))
void assert_err(const char *condition, const char * file, int line)
{
	UART_PRINTF("%s,condition %s,file %s,line = %d\r\n",__func__,condition,file,line);

}

void assert_param(int param0, int param1, const char * file, int line)
{
	UART_PRINTF("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);

}

void assert_warn(int param0, int param1, const char * file, int line)
{
	UART_PRINTF("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);

}

void dump_data(uint8_t* data, uint16_t length)
{
	UART_PRINTF("%s,data = %d,length = %d,file = %s,line = %d\r\n",__func__,data,length);

}
#else
void assert_err(const char *condition, const char * file, int line)
{

}

void assert_param(int param0, int param1, const char * file, int line)
{

}

void assert_warn(int param0, int param1, const char * file, int line)
{

}

void dump_data(uint8_t* data, uint16_t length)
{

}
#endif //UART_PRINTF_EN


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void platform_reset(uint32_t error)
{
	//void (*pReset)(void);

	UART_PRINTF("error = %x\r\n", error);

	// Disable interrupts
	GLOBAL_INT_STOP();

#if UART_PRINTF_EN
	// Wait UART transfer finished
	#if UART_1_INIT
	uart_finish_transfers();
	#endif
	#if UART_2_INIT
	uart2_finish_transfers();
	#endif
#endif //UART_PRINTF_EN


	if(error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
	{
		// Not yet supported
	}
	else
	{
		//Restart FW
		//pReset = (void * )(0x0);
		//pReset();
		wdt_enable(10);
		while(1);
	}
}

#include "tuya_ble_type.h"
extern tuya_ble_parameters_settings_t tuya_ble_current_para;
uint8_t tmp_bt_mac_str[6] = {0xDC, 0x23, 0x4D, 0x67, 0x67, 0xEC};

void bdaddr_env_init(void)
{
	struct bd_addr co_bdaddr;
    memcpy(co_bdaddr.addr,tuya_ble_current_para.auth_settings.mac,6);
	if(co_bdaddr.addr[0]!=0 ||co_bdaddr.addr[1]!=0||
	        co_bdaddr.addr[2]!=0||co_bdaddr.addr[3]!=0||
	        co_bdaddr.addr[4]!=0||co_bdaddr.addr[5]!=0 )
	{
		memcpy(&co_default_bdaddr,&co_bdaddr,6);
	}
    else
    {
//        memset(&co_default_bdaddr,0xFE,6);
        memcpy(&co_default_bdaddr,tmp_bt_mac_str,6);
    }
}


void ble_clk_enable(void)
{
	REG_AHB0_ICU_BLECLKCON =  0;
}


#if 0
void user_timer_cb(unsigned char ucChannel)
{
    gpio_triger(0x11);
}


void user_timer_init(void)
{
	icu_set_sleep_mode(0);
	rwip_prevent_sleep_set(BK_DRIVER_TIMER_ACTIVE);
	PWM_DRV_DESC timer_desc;

	timer_desc.channel = 1;
    timer_desc.mode    = 1<<0 | 1<<1 | 1<<2 | 1<<4;   
    timer_desc.end_value  = 65534;                      
    timer_desc.duty_cycle = 0;                        
    timer_desc.p_Int_Handler = user_timer_cb;  	

	REG_AHB0_ICU_PWMCLKCON |= (1<<1);
    REG_AHB0_ICU_PWMCLKCON &= ~(7<<12);
    REG_AHB0_ICU_PWMCLKCON |= (8<<12);
	pwm_init(&timer_desc);

}
#endif

#ifdef KEY_BUZZER_FUNC
PWM_DRV_DESC timer_desc_2;

void beep_test(void)
{
	timer_desc_2.duty_cycle = 3;
	pwm_set_duty(&timer_desc_2);
	Delay_ms(100);
	timer_desc_2.duty_cycle = 0;
	pwm_set_duty(&timer_desc_2);
}

void beep_init(void)
{	
	timer_desc_2.channel = 2;
	timer_desc_2.mode = 1<<0 | 0<<1 | 0<<2 | 0<<4;
	timer_desc_2.end_value = 6;
	timer_desc_2.duty_cycle = 0;
	pwm_init(&timer_desc_2);
}
#endif

FEED_PLAN_t info_default = 
{
	FLASH_KEEP_VAL,					//��־λ
	0,								//ιʳ��
	0,								//RTC�Ƿ��ʱ���־
	0,								//ʱ������
	8,								//ʱ��
	0,								//������־
	0,								//¼��ʱ��
//	{
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//		{0,0,0,0,0},
//	},
};

void flash_data_init(uint8_t type)
{
	//��λ
	UART_PRINTF("flash_data_init:%d \n",type);
	if(type > 0)
	{
		#if defined NO_RECORD_FUNC || defined KEY_BUZZER_FUNC
		beep_test();
		Delay_ms(200);
		beep_test();
		Delay_ms(100);
		beep_test();
		#else
		gpio_set(SOUND_REC, RECORD_ON);
		#ifdef NEW_RECORD_IC
		Delay_ms(1300);
		#else
		Delay_ms(300);	
		#endif
		gpio_set(SOUND_REC, RECORD_OFF);
		#endif

		memset(&ty_plan, 0, sizeof(FEED_PLAN_t));
//		ty_plan.mark = FLASH_KEEP_VAL;
		memcpy(&ty_plan, &info_default, sizeof(FEED_PLAN_t));
//		UART_PRINTF("reset ty_plan 1\n");
		save_flash(FLASH_PLAN);
		
		memset(&ty_logs, 0, sizeof(FEED_LOG_INFO_t));
		ty_logs.mark = FLASH_KEEP_VAL;
//		UART_PRINTF("reset ty_logs 1\n");
		save_flash(FLASH_LOG);
		
		Delay_ms(500);
		platform_reset(0);
//		wdt_enable(10);

	}
	else
	{
		memset(&ty_plan, 0, sizeof(FEED_PLAN_t));
		memset(&ty_logs, 0, sizeof(FEED_LOG_INFO_t));
		
		tuya_ble_nv_read(BLE_SAVE_ADDR, (uint8_t *) &ty_logs, sizeof(FEED_LOG_INFO_t));
		tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
//		flash_read(FLASH_SPACE_TYPE_NVR, BLE_SAVE_ADDR, sizeof(FEED_LOG_INFO_t), (uint8_t *) &ty_logs);
//		flash_read(FLASH_SPACE_TYPE_NVR, BLE_PLAN_ADDR, sizeof(FEED_PLAN_t), (uint8_t *) &ty_plan);
		printf_flash_info();
		if(ty_plan.mark != FLASH_KEEP_VAL)
		{
//			UART_PRINTF("reset flash  ty_plan\r\n");
			memset(&ty_plan, 0, sizeof(FEED_PLAN_t));
			ty_plan.mark = FLASH_KEEP_VAL;
			save_flash(FLASH_PLAN);
		}
		
		if(ty_logs.mark != FLASH_KEEP_VAL)
		{
//			UART_PRINTF("reset flash  ty_logs \r\n");

			memset(&ty_logs, 0, sizeof(FEED_LOG_INFO_t));
			ty_logs.mark = FLASH_KEEP_VAL;
			save_flash(FLASH_LOG);
		}
	}
}

/**
 *******************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 *******************************************************************************
 */

extern struct rom_env_tag rom_env;

void rwip_eif_api_init(void);
void rw_main(void)
{
	uint8_t utc_flag = 0;

	/*
	 ***************************************************************************
	 * Platform initialization
	 ***************************************************************************
	 */
#if SYSTEM_SLEEP
	uint8_t sleep_type = 0;
#endif
	icu_init();

	// Initialize random process
	srand(1);

	//get System sleep flag
	system_sleep_init();

	// Initialize the exchange memory interface
	emi_init();

	// Initialize timer module
	timer_init();

	rwip_eif_api_init();

	// Initialize the Interrupt Controller
	intc_init();
	// Initialize UART component
//#if (UART_DRIVER)
////	uart_init(115200);
//    uart2_init(115200);	
//	//uart_cb_register(uart_rx_handler);
//#endif

#if (UART2_DRIVER) && UART_2_INIT
	uart2_init(115200);
	uart2_cb_register(uart2_rx_handler);
#endif
#if PLF_NVDS
	// Initialize NVDS module
	struct nvds_env_tag env;
	env.flash_read = &flash_read;
	env.flash_write = &flash_write;
	env.flash_erase = &flash_erase;
	nvds_init(env);
#endif

	flash_init();
	rom_env_init(&rom_env);

	/*
	  ***************************************************************************
	  * RW SW stack initialization
	  ***************************************************************************
	  */
	//enable ble clock
	ble_clk_enable();

	// Initialize RW SW stack
	rwip_init(0);

	bdaddr_env_init();

	//gpio_init();

	REG_AHB0_ICU_INT_ENABLE |= (0x01 << 15); //BLE INT
	REG_AHB0_ICU_IRQ_ENABLE = 0x03;

	// finally start interrupt handling
	GLOBAL_INT_START();
	
	wdt_enable(0xffff);
	UART_PRINTF("start 2\r\n");
	UART_PRINTF("version:%s \r\n",USER_VERSION);
	UART_PRINTF("data:%s \r\n",USER_DATA);

	gpio_init();
	SET_LED_ON(LED_GREEN);
	SET_LED_ON(LED_RED);
	adc_init(BATTERY_CHAN,1);
	#if MOTOR_REVERSE_ADC
	adc_reverse_init();
	#endif
//	beep_init();

#if (UART_DRIVER) && UART_1_INIT
	uart_init(115200);
	uart_cb_register(uart_rx_handler);
#endif
	utc_update();
	utc_set_clock(1);
	
	flash_data_init(0);
	/*
	 ***************************************************************************
	 * Main loop
	 ***************************************************************************
	 */
	while(1)
	{
		//schedule all pending events
		rwip_schedule();
		wdt_enable(0xffff);
		
//		UART_PRINTF("while \r\n");
		if(utc_flag == 0)
		{
			utc_flag = 1;
			#ifdef FUNC_315MHZ
			BK_315MHz_timer_set();
			#endif
			ke_timer_set(UTC_TASK, TASK_APP, 100);
			ke_timer_set(KEY_SCAN_TASK, TASK_APP, 10);	
		}
		
		#if BATTERY_FUNC
		if(check_voltage_flag && feed_status == FEED_STEP_NONE)
		{
			disp_voltage();					//�������			
		}
		#endif	
		
		#ifdef FUNC_315MHZ
		if(byte_flag == 1)
		{
			byte_flag = 0;
			byte_debug_show();
		}
		#endif
		
		if(feed_status == FEED_STEP_NONE)
			feed_check();
		#ifdef MOTOR_REVERSE_ADC
		else if(feed_status > FEED_STEP_SOUND)
		{
			adc_reverse();
			Delay_ms(10);
		}
		#endif
		
		if(restore_flag == 1)
		{
			tuya_ble_device_factory_reset();
			restore_flag = 2;
		}
		else if(restore_flag == 3)
		{
			flash_data_init(1);
			restore_flag = 4;
		}


//		if(debug_flag == 1) 
//		{
//			debug_flag = 0;
//			UART_PRINTF("####### BK_315_callback 10000 times #######\r\n");
//		}
		
		// Checks for sleep have to be done with interrupt disabled
//		GLOBAL_INT_DISABLE();

		//oad_updating_user_section_pro();

		if(wdt_disable_flag==1)
		{
			wdt_disable();
		}
#if SYSTEM_SLEEP

		// Check if the processor clock can be gated
		sleep_type = rwip_sleep();
		if((sleep_type & RW_MCU_DEEP_SLEEP) == RW_MCU_DEEP_SLEEP)
		{
			// 1:idel  0:reduce voltage
			if(icu_get_sleep_mode())
			{
				cpu_idle_sleep();
			}
			else
			{
				cpu_reduce_voltage_sleep();
			}
		}
		else if((sleep_type & RW_MCU_IDLE_SLEEP) == RW_MCU_IDLE_SLEEP)
		{
			cpu_idle_sleep();
		}
#endif
//		Stack_Integrity_Check();
//		GLOBAL_INT_RESTORE();
	}
}


#if (UART_DRIVER)
static void uart_rx_handler(uint8_t *buf, uint8_t len)
{
	
	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("0x%x ", buf[i]);
	}
	UART_PRINTF("\r\n");
	
}
#endif
#if (UART2_DRIVER)
static void uart2_rx_handler(uint8_t *buf, uint8_t len)
{
	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("0x%x ", buf[i]);
	}
	UART_PRINTF("\r\n");
}
#endif

void rwip_eif_api_init(void)
{
	#ifdef UART_1_INIT
	uart_api.read = &uart_read;
	uart_api.write = &uart_write;
	uart_api.flow_on = &uart_flow_on;
	uart_api.flow_off = &uart_flow_off;
	#endif
	
	#ifdef UART_2_INIT
	uart_api.read = &uart2_read;
	uart_api.write = &uart2_write;
	uart_api.flow_on = &uart2_flow_on;
	uart_api.flow_off = &uart2_flow_off;	
	#endif
}

const struct rwip_eif_api* rwip_eif_get(uint8_t type)
{
	const struct rwip_eif_api* ret = NULL;
	switch(type)
	{
	case RWIP_EIF_AHI:
	{
		ret = &uart_api;
	}
	break;
#if (BLE_EMB_PRESENT) || (BT_EMB_PRESENT)
	case RWIP_EIF_HCIC:
	{
		ret = &uart_api;
	}
	break;
#elif !(BLE_EMB_PRESENT) || !(BT_EMB_PRESENT)
	case RWIP_EIF_HCIH:
	{
		ret = &uart_api;
	}
	break;
#endif
	default:
	{
		ASSERT_INFO(0, type, 0);
	}
	break;
	}
	return ret;
}

//static void Stack_Integrity_Check(void)
//{
//	if ((REG_PL_RD(STACK_BASE_UNUSED)!= BOOT_PATTERN_UNUSED))
//	{
//		while(1)
//		{
//			UART_PUTCHAR("Stack_Integrity_Check STACK_BASE_UNUSED fail!\r\n");
//		}
//	}

//	if ((REG_PL_RD(STACK_BASE_SVC)!= BOOT_PATTERN_SVC))
//	{
//		while(1)
//		{
//			UART_PUTCHAR("Stack_Integrity_Check STACK_BASE_SVC fail!\r\n");
//		}
//	}

//	if ((REG_PL_RD(STACK_BASE_FIQ)!= BOOT_PATTERN_FIQ))
//	{
//		while(1)
//		{
//			UART_PUTCHAR("Stack_Integrity_Check STACK_BASE_FIQ fail!\r\n");
//		}
//	}

//	if ((REG_PL_RD(STACK_BASE_IRQ)!= BOOT_PATTERN_IRQ))
//	{
//		while(1)
//		{
//			UART_PUTCHAR("Stack_Integrity_Check STACK_BASE_IRQ fail!\r\n");
//		}
//	}

//}


void rom_env_init(struct rom_env_tag *api)
{
	memset(&rom_env,0,sizeof(struct rom_env_tag));
	rom_env.prf_get_id_from_task = prf_get_id_from_task;
	rom_env.prf_get_task_from_id = prf_get_task_from_id;
	rom_env.prf_init = prf_init;
	rom_env.prf_create = prf_create;
	rom_env.prf_cleanup = prf_cleanup;
	rom_env.prf_add_profile = prf_add_profile;
	rom_env.rwble_hl_reset = rwble_hl_reset;
	rom_env.rwip_reset = rwip_reset;
#if SYSTEM_SLEEP
	rom_env.rwip_prevent_sleep_set = rwip_prevent_sleep_set;
	rom_env.rwip_prevent_sleep_clear = rwip_prevent_sleep_clear;
	rom_env.rwip_sleep_lpcycles_2_us = rwip_sleep_lpcycles_2_us;
	rom_env.rwip_us_2_lpcycles = rwip_us_2_lpcycles;
	rom_env.rwip_wakeup_delay_set = rwip_wakeup_delay_set;
#endif
	rom_env.platform_reset = platform_reset;
	rom_env.assert_err = assert_err;
	rom_env.assert_param = assert_param;
	#if UART_1_INIT
	rom_env.Read_Uart_Buf = Read_Uart_Buf;
	rom_env.uart_clear_rxfifo = uart_clear_rxfifo;
	#endif
	#if UART_2_INIT
	rom_env.Read_Uart2_Buf = Read_Uart2_Buf;
	rom_env.uart2_clear_rxfifo = uart2_clear_rxfifo;
	#endif
}

/// @} DRIVERS
