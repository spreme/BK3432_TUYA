#include "user_gpio.h"
#include "rwip.h"
#include "user_config.h"
#include "icu.h"

#ifdef FUNC_315MHZ
uint8_t save_flag = 0;

static volatile uint8_t BK_315_status = NONE;
static volatile uint64_t BK_315_tick = 0;
static volatile uint64_t BK_315_tick_before = 0;
static volatile uint8_t BK_315_gpio_before = 0;
static volatile uint8_t bit_tick = 0;
 volatile uint8_t byte_flag = 0;
static volatile uint8_t bit_buf[24] = {0};
static volatile uint8_t bit_show[24] = {0};

void BK_315_callback(unsigned char ucChannel)	//为了确保速度，不要在函数里加打印
{	
	
	volatile uint8_t BK_315_gpio = gpio_get_input(GPIO_315MHz);
	volatile uint64_t high_tick = 0;
	volatile uint64_t low_tick = 0;
	
	BK_315_tick++;
	
	if(BK_315_gpio != BK_315_gpio_before)	//当检测到与上次电平不同
	{
		BK_315_gpio_before = BK_315_gpio;
		
		if(BK_315_gpio == 1)
			low_tick = BK_315_tick - BK_315_tick_before;
		else
			high_tick = BK_315_tick - BK_315_tick_before;
		
		BK_315_tick_before = BK_315_tick;
		
		switch(BK_315_status)
		{
			case NONE:
				if((high_tick >= 2) && (high_tick <= 6))	//每字节开头1为高电平400us
				{
					BK_315_status = HEAD_1;
					memset((void *)bit_buf, 0, sizeof(bit_buf));
					bit_tick = 0;
				}
				break;
			
			case HEAD_1:
				{
					if((low_tick >= 86) && (low_tick <= 162))	//开头12.4ms为低电平
					{
						BK_315_status = HEAD_2;
					}
					else
						BK_315_status = NONE;
				}
				break;
				
			case HEAD_2:
				{
					if((high_tick >= 8) && (high_tick <= 16))	//每bit拉高1.2ms为高电平开头
					{
						BK_315_status = BIT_H;
					}
					else if((high_tick >= 2) && (high_tick <= 6))	//每bit拉高400us为低电平开头
					{
						BK_315_status = BIT_L;
					}
					else
						BK_315_status = NONE;
				}
				break;
				
			case BIT_H:
				{
					if((low_tick >= 2) && (low_tick <= 6))	//每bit拉低400us为结束1bit
					{
						bit_buf[bit_tick] = 1;
						bit_tick++;
						if(bit_tick >= 24)
						{
							memcpy((void *)bit_show, (const void *)bit_buf, 24);
							byte_flag = 1;
							BK_315_status = NONE;
						}
						else
							BK_315_status = HEAD_2;
					}
					else
						BK_315_status = NONE;
				}
				break;
				
			case BIT_L:
				{
					if((low_tick >= 8) && (low_tick <= 16))	//每bit拉低1.2ms为结束1bit
					{
						bit_buf[bit_tick] = 0;
						bit_tick++;
						if(bit_tick >= 24)
						{
							memcpy((void *)bit_show, (const void *)bit_buf, 24);
							byte_flag = 1;
							BK_315_status = NONE;
						}
						else
							BK_315_status = HEAD_2;
					}
					else
						BK_315_status = NONE;
				}
				break;
			
			default:
				{
					BK_315_status = NONE;
				}
				break;
		}
	}
}

void pair_check(void)
{
	int i, j, ret = 0;
	
	if(pair_flag == 1)
	{
		for(i=0; i<8; i++)
		{
			if((ty_plan.key[i][20] == 0) && (ty_plan.key[i][21] == 1)
				&& (ty_plan.key[i][22] == 1) && (ty_plan.key[i][23] == 1))	//如果存在有效数据位，先判断是否相同
			{
//				UART_PRINTF("ty_plan.key[%d]: ", i);
//				for(j=0; j<24; j++)
//				{
//					UART_PRINTF("%d", ty_plan.key[i][j]);
//				}
//				UART_PRINTF("\r\n");
				
				ret = 0;
				for(j=0; j<24; j++)
				{
					if(bit_show[j] != ty_plan.key[i][j])	//当有1个码不同时跳过
					{
						UART_PRINTF("-----different KEY\r\n");
						ret = 1;
						break;
					}
				}
				if(ret == 0)	//所有码均相同，无需保存
				{
					UART_PRINTF("-----same KEY\r\n");
					save_flag = 1;
					return;
				}
			}
			else	//空数据位，直接保存
			{
				UART_PRINTF("-----BIT save\r\n");
				memcpy(ty_plan.key[i], (const void *)bit_show, 24);
				save_flag = 1;
				save_flash(FLASH_PLAN);
				return;
			}
		}
	}
	else
	{
		if((ty_plan.key[0][20] == 0) && (ty_plan.key[0][21] == 1)
			&& (ty_plan.key[0][22] == 1) && (ty_plan.key[0][23] == 1))	//曾经配对过
		{
			for(i=0; i<8; i++)
			{
				if((ty_plan.key[i][20] != 0) || (ty_plan.key[i][21] != 1)
					|| (ty_plan.key[i][22] != 1) || (ty_plan.key[i][23] != 1))	//当已配对数目为空
					return;
				else
				{
					ret = 0;
					for(j=0; j<24; j++)
					{
						if(bit_show[j] != ty_plan.key[i][j])	//当有1个码不同时跳过
						{
							ret = 1;
							break;
						}
					}
					if(ret == 0)	//所有码均相同，喂食
					{
						if(feed_status == FEED_STEP_NONE && feed_key != 10)
							feed_key = 1;
						
						return;
					}
				}
			}
		}
		else	//不存在配对
		{
			if(feed_status == FEED_STEP_NONE && feed_key != 10)
				feed_key = 1;
						
			return;
		}
	}
}

void byte_debug_show(void)
{
	int i;
	
	UART_PRINTF("BIT SHOW DEBUG: ");
	for(i=0; i<24; i++)
	{
		UART_PRINTF("%d", bit_show[i]);
	}
	UART_PRINTF("\r\n");
	
	if((bit_show[20] == 0) && (bit_show[21] == 1)
		&& (bit_show[22] == 1) && (bit_show[23] == 1))
		pair_check();
}

void BK_315MHz_timer_set(void)
{
	PWM_DRV_DESC timer_desc;
	
	timer_desc.channel = 0;
	timer_desc.mode = 1<<0 | 1<<1 | 1<<2 | 1<<4;	//开定时器，开中断，定时器模式，16M时钟源
	timer_desc.pre_divid = 0;
	timer_desc.end_value = 1600;	//必须是800倍数，目前是100us回调
	timer_desc.duty_cycle = 0;
	timer_desc.p_Int_Handler = BK_315_callback;
	
	REG_AHB0_ICU_PWMCLKCON |= (1<<1);
	REG_AHB0_ICU_PWMCLKCON &= ~(7<<12);
	REG_AHB0_ICU_PWMCLKCON |= (8<<12);
	
	pwm_init(&timer_desc);
}


#endif
