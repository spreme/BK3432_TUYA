#ifndef __ROM_CALL_FLASH_H_
#define __ROM_CALL_FLASH_H_

#include "ke_msg.h"
#include "co_bt.h"            // common bt definitions
#include "ke_task.h"
#include "gapm_task.h"

#include "rwble_hl.h"
#include "prf.h"
#include "lld_sleep.h"
#include "rwip.h"

#include <string.h>


struct rom_env_tag
{
	ke_task_id_t (*prf_get_id_from_task)(ke_msg_id_t task);
	ke_task_id_t (*prf_get_task_from_id)(ke_msg_id_t id);
	
	void (*prf_init)(bool reset);
	void (*prf_create)(uint8_t conidx);
	void (*prf_cleanup)(uint8_t conidx, uint8_t reason);
	
	uint8_t (*prf_add_profile)(struct gapm_profile_task_add_cmd * params, ke_task_id_t* prf_task);
	void (*rwble_hl_reset)(void);
	
	void (*rwip_reset)(void);
	
	void (*rwip_prevent_sleep_set)(uint16_t prv_slp_bit);
	void (*rwip_prevent_sleep_clear)(uint16_t prv_slp_bit);
	
	uint32_t (*rwip_sleep_lpcycles_2_us)(uint32_t lpcycles);
	uint32_t (*rwip_us_2_lpcycles)(uint32_t us);
	
	
	void (*rwip_wakeup_delay_set)(uint16_t wakeup_delay);
	
	void (*platform_reset)(uint32_t error);
	
  void(*assert_err) (const char *condition, const char * file, int line);
	
	void(*assert_param)(int param0, int param1, const char * file, int line);

	void (*assert_warn)(int param0, int param1, const char * file, int line);
		
	#if UART_1_INIT
	uint8_t (*Read_Uart_Buf)(void);
	void (*uart_clear_rxfifo)(void);
	#endif
	#if UART_2_INIT
	uint8_t (*Read_Uart2_Buf)(void);
	void (*uart2_clear_rxfifo)(void);
	#endif	
};




void rom_env_init(struct rom_env_tag *api);

//extern struct flash_for_rom_func_tag RcallF_func_list;
extern struct rom_env_tag rom_env;


void lld_bdaddr_init(const struct  bd_addr *addr);















#endif // __ROM_CALL_FLASH_H_

