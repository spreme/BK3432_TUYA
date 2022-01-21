/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions

#include "ke_timer.h"             // Kernel timer
#include "app_tuya_service.h"              // fff0 Module Definition
#include "tuya_service_task.h"
//#include "app_fff0.h"              // fff0 Module Definition
//#include "fff0s_task.h"
//#include "app_dis.h"              // Device Information Module Definition
//#include "diss_task.h"
//#include "app_batt.h"             // Battery Module Definition
//#include "bass_task.h"
//#include "app_oads.h"             
//#include "oads_task.h"              
#include "gpio.h"
#include "uart.h"
#include "BK3432_reg.h"
#include "icu.h"
#include "reg_ble_em_cs.h"
#include "lld.h"
#include "wdt.h"
#include "tuya_ble_type.h"
#include "tuya_ble_main.h"
#include "tuya_ble_api.h"
#include "app_tuya_ota.h"
#include "tuya_ble_app_production_test.h"
#include "tuya_ble_log.h"


enum KEY_STATE_E
{
	KEY_SET_E = 0x1,
	KEY_UP_E = 0x2,
	KEY_DOWM_E = 0x4,
	KEY_LOCK_E = 0x8,
	KEY_FEED_E = 0x10,	
	KEY_RECORD_E = 0x20,	
};
uint32_t set_key_tick = 0;				//设置按键计时
uint32_t lock_key_tick = 0;				//锁键按键计时
uint32_t feed_key_tick = 0;				//喂食按键计时
uint32_t record_key_tick = 0;			//录音按键计时
uint32_t rec_key_tick = 0;				//录音操作计时标志
static uint8_t pair_tick = 0;
uint8_t pair_flag = 0;
uint8_t pair_timeout = 0;
uint8_t lock_time_out = 0;				//30s自动锁键计时

#if 1
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {
			
        struct ke_msg_handler handler = (*(handler_list->msg_table + counter - 1));
			
        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    // Application has not been initialized
    ASSERT_ERR(ke_state_get(dest_id) == APPM_INIT);

    // Reset the stack
    struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                              TASK_GAPM, TASK_APP,
                                              gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("param->operation = 0x%x, param->status = 0x%x \r\n", param->operation, param->status);
    switch(param->operation)
    {
        // Reset completed
        case (GAPM_RESET):
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
                // Set Device configuration
                struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
	                                                                   TASK_GAPM, TASK_APP,
                                                                   gapm_set_dev_config_cmd);
                // Set the operation
                cmd->operation = GAPM_SET_DEV_CONFIG;
                // Set the device role - Peripheral
                cmd->role      = GAP_ROLE_PERIPHERAL;
                // Set Data length parameters
                cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
                cmd->sugg_max_tx_time   = BLE_MIN_TIME;
								
		 		cmd->max_mtu = 131;//BLE_MIN_OCTETS;
                //Do not support secure connections
                cmd->pairing_mode = GAPM_PAIRING_LEGACY;
				cmd->att_cfg = GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN|GAPM_MASK_ATT_SVC_CHG_EN|GAPM_MASK_ATT_APPEARENCE_PERM|GAPM_MASK_ATT_NAME_PERM;
                
 				//cmd->addr_type   = GAPM_CFG_ADDR_HOST_PRIVACY; //2017-10-24 by alen
                // load IRK
                memcpy(cmd->irk.key, app_env.loc_irk, KEY_LEN);

		        app_env.next_svc = 0;

                // Send message
                ke_msg_send(cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;
        case (GAPM_PROFILE_TASK_ADD):
        {
            // Add the next requested service
            if (!appm_add_svc())
            {
                // Go to the ready state
                ke_state_set(TASK_APP, APPM_READY);
							
				ke_timer_set(LED_TIMER_TASK, TASK_APP, 300);
				appm_start_advertising();
				PCF8563_init();
            }
        }
        break;
        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

            // Go to the create db state
            ke_state_set(TASK_APP, APPM_CREATE_DB);

            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            appm_add_svc();
        }
        break;	

        case (GAPM_ADV_NON_CONN):
        case (GAPM_ADV_UNDIRECT):
        case (GAPM_ADV_DIRECT):
		case (GAPM_UPDATE_ADVERTISE_DATA):
        case (GAPM_ADV_DIRECT_LDC):
		{
			if (param->status == GAP_ERR_TIMEOUT)
			{
                ke_state_set(TASK_APP, APPM_READY);
				
				//device not bonded, start general adv
				appm_start_advertising();
            }
		}
        break;

        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("param->req=%d\r\n", param->req);
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm);
            cfm->req = param->req;
            
            // No appearance
            cfm->info.appearance = 0;

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                    								src_id, dest_id,
                                                    gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_params.con_intv_min = 8;
            // Slave preferred Connection interval Max
            cfm->info.slv_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_params.slave_latency  = 0;
            // Slave preferred Link supervision timeout
            cfm->info.slv_params.conn_timeout  = 600;  // 6s (600*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

		case GAPC_PER_PRIVATE_FLAG:
	{
		// Allocate message
		struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
		                                    src_id, dest_id,
		                                    gapc_get_dev_info_cfm);
		cfm->req = param->req;
		cfm->info.private_flag = 0;
		// Send message
		ke_msg_send(cfm);
	}
	break;

        default: /* Do Nothing */
			break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Set Device configuration
	struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                 gapc_set_dev_info_cfm);
	// Reject to change parameters
	cfm->status = GAP_ERR_REJECTED;
	cfm->req = param->req;
	// Send message
	ke_msg_send(cfm);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{	
	UART_PRINTF("%s\r\n", __func__);
	
    app_env.conidx = KE_IDX_GET(src_id);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

        cfm->auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        // Send the message
        ke_msg_send(cfm);

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/
         
        // Enable Battery Service
        //app_batt_enable_prf(app_env.conhdl);
        //app_tuya_enable_prf(app_env.conhdl);
		
        // We are now in connected State
        ke_state_set(dest_id, APPM_CONNECTED);
        
        tuya_ble_connected_handler();
		
		#if UPDATE_CONNENCT_PARAM
		ke_timer_set(APP_PARAM_UPDATE_REQ_IND,TASK_APP,100); 
		#endif	
	        
    }
    else
    {
        // No connection has been establish, restart advertising
		appm_start_advertising();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("gapc_cmp_evt_handler operation = %x\r\n",param->operation);
	switch(param->operation)
	{
    	case (GAPC_UPDATE_PARAMS):  //0x09
    	{
			if (param->status != GAP_ERR_NO_ERROR)
        	{
            	UART_PRINTF("gapc update params fail !\r\n");
			}
			else
			{
				UART_PRINTF("gapc update params ok !\r\n");
			}
			
    	} break;

		case (GAPC_SECURITY_REQ): //0x0c
		{
			if (param->status != GAP_ERR_NO_ERROR)
	        {
	            UART_PRINTF("gapc security req fail !\r\n");
	        }
	        else
	        {
	            UART_PRINTF("gapc security req ok !\r\n");
	        }
		}break;
		case (GAPC_BOND): // 0xa
    	{
	        if (param->status != GAP_ERR_NO_ERROR)
	        {
	            UART_PRINTF("gapc bond fail !\r\n");
	        }
	        else
	        {
	            UART_PRINTF("gapc bond ok !\r\n");
	        }
    	}break;
		
		case (GAPC_ENCRYPT): // 0xb
		{
			if (param->status != GAP_ERR_NO_ERROR)
			{
				UART_PRINTF("gapc encrypt start fail !\r\n");
			}
			else
			{
				UART_PRINTF("gapc encrypt start ok !\r\n");
			}
		}
		break;
		

    	default:
    	  break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	UART_PRINTF("disconnect link reason = 0x%x\r\n",param->reason);
	
    // Go to the ready state
    ke_state_set(TASK_APP, APPM_READY);

	wdt_disable_flag = 1;

	// Restart Advertising
	appm_start_advertising();

    tuya_ble_disconnected_handler();
	
	tuya_ota_init_disconnect();
	
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles profile add indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Current State
    uint8_t state = ke_state_get(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
            default: 
			break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}


/*******************************************************************************
 * Function: app_period_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *		  param -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance (TASK_GAP).
 *		  ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_tuya_connect_monitor_timer_handler(ke_msg_id_t const msgid,
                                          void *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
	//gpio_triger(0x31);
	//ke_timer_set(APP_PERIOD_TIMER, TASK_APP, 1);
    tuya_ble_vtimer_connect_monitor_callback();

    return KE_MSG_CONSUMED;
}

static int app_tuya_prod_monitor_timer_handler(ke_msg_id_t const msgid,
                                          void *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
	tuya_ble_vtimer_prod_monitor_callback();

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol          = KE_MSG_CONSUMED;


    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

        case (TASK_ID_TUYAS):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_tuya_table_handler, msgid, param, src_id);
        } break;
        
//        case (TASK_ID_FFF0S):
//        {
//            // Call the Health Thermometer Module
//          //  msg_pol = appm_get_handler(&app_fff0_table_handler, msgid, param, src_id);
//        } break;
				
        case (TASK_ID_DISS):
        {
            // Call the Device Information Module
         //   msg_pol = appm_get_handler(&app_dis_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_BASS):
        {
            // Call the Battery Module
         //   msg_pol = appm_get_handler(&app_batt_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_OADS):
        {
            // Call the Health Thermometer Module
         //   msg_pol = appm_get_handler(&app_oads_table_handler, msgid, param, src_id);
        } break;

        default:
        {
        } break;
    }

    return (msg_pol);
}


/*******************************************************************************
 * Function: gapc_update_conn_param_req_ind_handler
 * Description: Update request command processing from slaver connection parameters
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_update_conn_param_req_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_update_req_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

	UART_PRINTF("slave send param_update_req\r\n");
	struct gapc_conn_param  up_param;
	
	up_param.intv_min   = BLE_UAPDATA_MIN_INTVALUE;
	up_param.intv_max   = BLE_UAPDATA_MAX_INTVALUE; 
	up_param.latency    = BLE_UAPDATA_LATENCY;  
	up_param.time_out   = BLE_UAPDATA_TIMEOUT; 
	
	appm_update_param(&up_param);
	
	return KE_MSG_CONSUMED;
}

 
/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_le_pkt_size_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
   	UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("1max_rx_octets = %d\r\n",param->max_rx_octets);
	UART_PRINTF("1max_rx_time = %d\r\n",param->max_rx_time);
	UART_PRINTF("1max_tx_octets = %d\r\n",param->max_tx_octets);
	UART_PRINTF("1max_tx_time = %d\r\n",param->max_tx_time);
	
	return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_updated_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("con_interval = %d\r\n",param->con_interval);
	UART_PRINTF("con_latency = %d\r\n",param->con_latency);
	UART_PRINTF("sup_to = %d\r\n",param->sup_to);
	
	return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_mtu_changed_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n",__func__);
	UART_PRINTF("ind->mtu = %d,seq = %d\r\n",ind->mtu,ind->seq_num);
	
 	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   GAPC_PARAM_UPDATE_REQ_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                struct gapc_param_update_req_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);
	// Prepare the GAPC_PARAM_UPDATE_CFM message
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                                             src_id, dest_id,
                                             gapc_param_update_cfm);
	 
	cfm->ce_len_max = 0xffff;
	cfm->ce_len_min = 0xffff;
	cfm->accept = true; 

	// Send message
    ke_msg_send(cfm);
	 
	return (KE_MSG_CONSUMED);
}


static int app_tuya_event_handler(ke_msg_id_t const msgid,
                                struct app_tuya_ble_evt_param_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	//UART_PRINTF("%s \r\n", __func__);
    TUYA_BLE_LOG_DEBUG("TUYA_BLE_EVENT - EVENT ID = 0x%04x \r\n", param->evt.hdr.event);
	
    tuya_ble_event_process((tuya_ble_evt_param_t *)&param->evt);
	 
	return (KE_MSG_CONSUMED);                                    
                                         
}
#endif
/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */
#if DU_PD01B
uint8_t stop_sys_flag = 0;

uint8_t get_key_state()
{
	uint8_t key_flag_e = 0;
	
	if(gpio_get_input(SET_KEY))
	{
		key_flag_e = key_flag_e | KEY_SET_E;
	}
	if(gpio_get_input(RECORD_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_RECORD_E;
	}
	#ifdef LOCK_KEY_KEY_E
	if(gpio_get_input(LOCK_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_LOCK_E;
	}
	#endif
	if(gpio_get_input(FEED_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_FEED_E;
	}
	
	return key_flag_e;
}
void key_function(uint8_t keep_flag)
{
	static uint8_t first_key_flag = 0;
//	if(key_scan_flag == 0)
	{
		if(gpio_get_input(SET_KEY) == 0)
		{
			if(set_key_tick <= KEY_SHORT_TIME && set_key_tick != 0)				//按键时间小于1s
			{
				lock_time_out = 0;
//				lock_time++;
//				if(lock_time >= 5)
//				{
//					tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
//					if(ty_plan.lock == 0)
//						ty_plan.lock = 1;
//					else if(ty_plan.lock == 1)
//						ty_plan.lock = 0;
//					
//					save_flash(FLASH_PLAN);
//				
//					lock_led = 1;
//					lock_time = 0;
//					UART_PRINTF("lock/unlock !!!!!!!!!!!! \r\n");
//				}
//				UART_PRINTF("lock_time:%d \r\n",lock_time);
			}
//			else if(set_key_tick > KEY_LONG_TIME_SET)
//			{
//				key_flag = KEY_SET_L_UP;
//				UART_PRINTF("key_flag = KEY_SET_L_UP\r\n");
//			}
			set_key_tick = 0;
		}
		else if(keep_flag == KEY_SET_E)
		{
			lock_time_out = 0;
			set_key_tick++;
			if(set_key_tick == KEY_LONG_TIME)
			{
				UART_PRINTF("stop system !!!!!!! \r\n");
				ke_timer_clear(UTC_TASK,TASK_APP);
				stop_sys_flag = 1;
			}
		}
			
		#ifdef LOCK_KEY_KEY_E
		if(gpio_get_input(LOCK_KEY))
		{
			if(lock_key_tick <= KEY_SHORT_TIME && lock_key_tick != 0)				//按键时间小于1s
			{
			}
			else if(lock_key_tick > KEY_LONG_TIME_SET)
			{
			}
			lock_key_tick = 0;
		}
		else if(keep_flag == KEY_LOCK_E )
		{
			lock_key_tick++;
			if(lock_key_tick == KEY_LONG_TIME_SET)
			{
			}
		}
		#endif
		if(gpio_get_input(FEED_KEY))
		{
			if(feed_key_tick <= KEY_SHORT_TIME && feed_key_tick != 0)				//按键时间小于1s
			{
				tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
				if(ty_plan.lock == 0 && first_key_flag != 0 && feed_key != 10)
				{
					feed_key = 1;
					UART_PRINTF("feed key\r\n");
				}
			}
			first_key_flag = 1;
//			else if(feed_key_tick > KEY_LONG_TIME)
//			{
//				key_flag = KEY_FEED_L_UP;
//				UART_PRINTF("key_flag = KEY_FEED_L_UP\r\n");
//			}
			feed_key_tick = 0;
		}
		else 
		{
			lock_time_out = 0;
			if(gpio_get_input(RECORD_KEY) == 0)
				feed_key_tick = 0;
			else
			{
				feed_key_tick++;
				if(feed_key_tick == KEY_LONG_TIME)
				{
					restore_flag = 1;
					UART_PRINTF("restore system !!!!!!!!!!\r\n");
				}
			}
		}
			
		if(gpio_get_input(SET_KEY) == 0)
		{
			pair_tick = 0;
		}
		else
		{
			lock_time_out = 0;
			if(gpio_get_input(RECORD_KEY) == 0)
			{
				pair_tick++;
				
				if(pair_tick >= 5)
				{
					pair_tick = 0;
					if(pair_flag == 0)
					{
						pair_timeout = 0;
						pair_flag = 1;
						ke_timer_clear(PAIR_TIMEOUT_TASK, TASK_APP);
						ke_timer_set(PAIR_TIMEOUT_TASK, TASK_APP, 100);
					}
				}
			}
			else
			{
				pair_tick = 0;
			}
		}
		
		if(gpio_get_input(RECORD_KEY))
		{
			lock_time = 0;
		}
		else
		{
			lock_time_out = 0;
			if(gpio_get_input(FEED_KEY) == 0)
			{
				lock_time++;
				
				if(lock_time >= 5)
				{
					tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
					if(ty_plan.lock == 0)
						ty_plan.lock = 1;
					else if(ty_plan.lock == 1)
						ty_plan.lock = 0;
					
					save_flash(FLASH_PLAN);

//					if(lock)
//					{
//						lock = 0;
//						lock_led = 1;
//					}
					lock_led = 1;
					lock_time = 0;
					UART_PRINTF("unlock !!!!!!!!!!!! \r\n");
				}
			}
			else
			{
				lock_time = 0;
			}
		}		
	}	

}
#endif
#if DU5C_B01
uint8_t get_key_state()
{
	uint8_t key_flag_e = 0;
	
	if(gpio_get_input(SET_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_SET_E;
	}
	if(gpio_get_input(RECORD_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_RECORD_E;
	}
	#ifdef LOCK_KEY_KEY_E
	if(gpio_get_input(LOCK_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_LOCK_E;
	}
	#endif
	if(gpio_get_input(FEED_KEY) <= 0)
	{
		key_flag_e = key_flag_e | KEY_FEED_E;
	}
	
	return key_flag_e;
}
void key_function(uint8_t keep_flag)
{
//	if(key_scan_flag == 0)
	{
//		if(keep_flag > 0)
//		{
//			UART_PRINTF("keep_flag:%02X\r\n",keep_flag);
//			UART_PRINTF("set_key_tick:%d\r\n",set_key_tick);
//			UART_PRINTF("feed_key_tick:%d\r\n",feed_key_tick);		
//		}
		if(gpio_get_input(SET_KEY) > 0)
		{
			if(set_key_tick <= KEY_SHORT_TIME && set_key_tick != 0)				//按键时间小于1s
			{
				
			}

			set_key_tick = 0;
		}
		else if(keep_flag == KEY_SET_E)
		{
			set_key_tick++;
			if(set_key_tick == KEY_LONG_SET_TIME)
			{
				restore_flag = 1;
				UART_PRINTF("restory system !!!!!!! \r\n");				
			}
		}
			
		#ifdef LOCK_KEY_KEY_E
		if(gpio_get_input(LOCK_KEY))
		{
			if(lock_key_tick <= KEY_SHORT_TIME && lock_key_tick != 0)				//按键时间小于1s
			{
			}
			lock_key_tick = 0;
		}
		else if(keep_flag == KEY_LOCK_E )
		{
			lock_key_tick++;
			if(lock_key_tick == KEY_LONG_LOCK_TIME)
			{
			}
		}
		#endif
		if(gpio_get_input(FEED_KEY) > 0)
		{
			if(feed_key_tick <= KEY_SHORT_TIME && feed_key_tick != 0)				//按键时间小于1s
			{
				UART_PRINTF("feed key  11111  \r\n");
				tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
				UART_PRINTF("lock:%d  \r\n",ty_plan.lock);
				if(ty_plan.lock == 0 && feed_key != 10)
				{
					feed_key = 1;
					UART_PRINTF("feed key\r\n");
				}
			}
//			else if(feed_key_tick > KEY_LONG_TIME)
//			{
//				key_flag = KEY_FEED_L_UP;
//				UART_PRINTF("key_flag = KEY_FEED_L_UP\r\n");
//			}
			feed_key_tick = 0;
		}
		else if(keep_flag == KEY_FEED_E)
		{
			feed_key_tick++;
			if(feed_key_tick == KEY_LONG_FEED_TIME)
			{
				tuya_ble_nv_read(BLE_PLAN_ADDR, (uint8_t *) &ty_plan, sizeof(FEED_PLAN_t));
				if(ty_plan.lock != 0)
					ty_plan.lock = 0;
				else 
					ty_plan.lock = 1;
				
				save_flash(FLASH_PLAN);
			
				lock_led = 1;
				//feed_key_tick = 0;
				UART_PRINTF("lock/unlock !!!!!!!!!!!! \r\n");
			}
		}
	}	

}
#endif

void rec_key_callback(void)
{
	if(ty_plan.lock == 0)
	{
		if(gpio_get_input(RECORD_KEY))
		{
			if(rec_key_tick <= 10)
			{
				UART_PRINTF("play sound !!\r\n");

				play_record_control();
			}
			else
			{
				gpio_set(SOUND_REC, RECORD_OFF);
				UART_PRINTF("rec over !!\r\n");
				ty_plan.record_time = rec_key_tick - 10;
				save_flash(FLASH_PLAN);
			}
			rec_key_tick = 0;
		}
		else
		{
			lock_time_out = 0;
			if(gpio_get_input(SET_KEY) || gpio_get_input(FEED_KEY) == 0)
				rec_key_tick = 0;
			else
			{
				rec_key_tick++;
				
				if(rec_key_tick > 10)
				{
					gpio_set(SOUND_REC, RECORD_ON);
				}
				if(rec_key_tick >= 110)
				{
					rec_key_tick = 0;
					gpio_set(SOUND_REC, RECORD_OFF);
					UART_PRINTF("rec over !!\r\n");
					ty_plan.record_time = 100;
					save_flash(FLASH_PLAN);
				}
				else
				{
	//				UART_PRINTF("@@@@@@@@@@@@@@@@@@@@@@@ REC_KEY_TASK exit\r\n");

					ke_timer_set(REC_KEY_TASK, TASK_APP, 10);
				}
			}
		}
	}
}



void key_scan_callback(void)
{
	keep_dowm_flag = get_key_state();
	if(KEY_RECORD_E == get_key_state())
	{
		ke_timer_set(REC_KEY_TASK, TASK_APP, 10);
	}
//	if(keep_dowm_flag)
	{
		ke_timer_set(KEY_SCAN_TASK, TASK_APP, 10);	
	}
//	if(keep_dowm_flag > 0)
//		UART_PRINTF("keep_dowm_flag:%d \r\n",keep_dowm_flag);
	
	key_function(keep_dowm_flag);

}

uint8_t led_tick = 0;
uint8_t sleep_tick = 0;

void utc_callback(void)
{
	static uint8_t restore_time = 0;
	static uint8_t rtc_tick = 0;

	connect_flag = tuya_ble_connect_status_get();

	#if DU_PD01B
//	if(lock_time > 0)
//	{
//		lock_time_out++;
//		if(lock_time_out > 2)
//		{
//			lock_time = 0;
//			lock_time_out = 0;
//		}
//	}
//	else
//	{
//		lock_time_out = 0;
//	}
//	if(lock == 0)
//	{
//		lock_time_out++;
//		if(lock_time_out >= 10)
//		{
//			lock_time_out = 0;
//			lock = 1;
//			lock_led = 1;
//		}
//	}
	#endif
	
	
	if(restore_flag == 2)
	{
		restore_time++;
		if(restore_time >=2)
		{
			restore_flag = 3;
		}
	}
	rtc_tick++;
	if(rtc_tick >= 5)
	{
		rtc_tick = 0;
		check_voltage_flag = 1;
		calender_flag = 1;
	}
	
#ifdef IR_FEED_TUBE
	if(feeding_flag == 0)
	{
		#ifdef IR_FEED_TUBE_LED
		gpio_set(IR_LED, 1);
		#endif /* end of IR_FEED_TUBE_LED */
		Delay_ms(10);
		if(gpio_get_input(IR_DET))			//对管堵塞
		{
			if(feed_error != 1)				//不是设备异常，转为堵粮异常
			{
				uint32_t t_zero = 0;
				
//				t_zero = suble_get_timestamp();
				utc_update();
				t_zero = utc_get_clock();
				
				feed_error = 2;
				err_type |= ERROR_IR;
				send_feed_error(t_zero, err_type);
			}
		}
		else
		{
			if(feed_error == 2)				//堵粮异常切换为不堵
			{
				uint32_t t_zero = 0;
				
//				t_zero = suble_get_timestamp();
				utc_update();
				t_zero = utc_get_clock();
				
				feed_error = 0;
				
				err_type &= ~ERROR_IR;
				send_feed_error(t_zero, err_type);
			}
		}
		#ifdef IR_FEED_TUBE_LED
		gpio_set(IR_LED, 0);
		#endif /* end of IR_FEED_TUBE_LED*/
	}
#endif

		
//	UART_PRINTF("@@@@@@@@@@@@@@@@@@@@@@@ UTC_TASK exit\r\n");
//	rwip_schedule();
	if(KEY_RECORD_E == get_key_state())
	{
		ke_timer_set(REC_KEY_TASK, TASK_APP, 10);
	}
	else if(get_key_state() > 0)
	{
		ke_timer_set(KEY_SCAN_TASK, TASK_APP, 10);
	}
	ke_timer_set(UTC_TASK, TASK_APP, 100);
}

static void led_timer_callback(void)
{
	if(stop_sys_flag > 0)
	{
		if(stop_sys_flag == 1)
		{
			SET_LED_OFF(LED_GREEN);
			SET_LED_OFF(LED_RED);
			Delay_ms(100);
			SET_LED_ON(LED_GREEN);
			SET_LED_ON(LED_RED);
			Delay_ms(100);
			SET_LED_OFF(LED_GREEN);
			SET_LED_OFF(LED_RED);
			Delay_ms(100);
			SET_LED_ON(LED_GREEN);
			SET_LED_ON(LED_RED);
			Delay_ms(100);
			
			stop_sys_flag = 2;
		}
		SET_LED_OFF(LED_GREEN);
		SET_LED_OFF(LED_RED);
		
		gpio_set(PWR_HOLD, 0);		
	}
	else if(lock_led == 1)
	{
		UART_PRINTF("lock_led \r\n");
		lock_led = 0;
		SET_LED_OFF(LED_GREEN);
		
		SET_LED_ON(LED_RED);
		Delay_ms(100);
		SET_LED_OFF(LED_RED);
		Delay_ms(100);
		SET_LED_ON(LED_RED);
		Delay_ms(100);
		SET_LED_OFF(LED_RED);
		
		ke_timer_set(LED_TIMER_TASK, TASK_APP, 100);
	}
	else if(save_flag == 1)
	{
		save_flag = 0;
		
		SET_LED_ON(LED_RED);
		SET_LED_ON(LED_GREEN);
		Delay_ms(100);
		SET_LED_OFF(LED_RED);
		SET_LED_OFF(LED_GREEN);
		Delay_ms(100);
		SET_LED_ON(LED_RED);
		SET_LED_ON(LED_GREEN);
		Delay_ms(100);
		SET_LED_OFF(LED_RED);
		SET_LED_OFF(LED_GREEN);
		
		ke_timer_set(LED_TIMER_TASK, TASK_APP, 30);
	}
	else if(pair_flag == 1)
	{
		if(sleep_tick%2 == 0)
		{
			SET_LED_ON(LED_RED);
			SET_LED_ON(LED_GREEN);
		}
		else
		{
			SET_LED_OFF(LED_RED);
			SET_LED_OFF(LED_GREEN);
		}
		
		sleep_tick++;
		ke_timer_set(LED_TIMER_TASK, TASK_APP, 30);
	}
	else if((feed_error == 1) || (led_test == 1) || (feed_error == 2) ||
		((low_power == 1) && (dc_flag == 0)))
	{
//		UART_PRINTF("feed_error:%d led_test:%d low_power:%d dc_flag:%d \r\n",feed_error,led_test, low_power,dc_flag);
		
		SET_LED_OFF(LED_GREEN);
		
		if(connect_flag != BONDING_CONN)
		{
			if(sleep_tick < 60)
			{
				if(sleep_tick%2 == 0)
					SET_LED_ON(LED_RED);
				else
					SET_LED_OFF(LED_RED);
				
				sleep_tick++;
			}
			else
			{
				if(led_tick < 9)
				{
					SET_LED_OFF(LED_RED);
					led_tick++;
				}
				else
				{
					SET_LED_ON(LED_RED);
					Delay_ms(100);
					SET_LED_OFF(LED_RED);
					led_tick = 0;
				}
			}
		}
		else
		{
			sleep_tick = 0;
			SET_LED_ON(LED_RED);
		}
		ke_timer_set(LED_TIMER_TASK, TASK_APP, 100);
	}
	else if(feed_key == 10)		//MAC是默认的，无授权，按键不喂食,红灯常亮
	{
		SET_LED_ON(LED_RED);
		SET_LED_OFF(LED_GREEN);
	}
	else
	{
		SET_LED_OFF(LED_RED);
		
		if(connect_flag != BONDING_CONN)
		{
			if(sleep_tick < 60)
			{
				if(sleep_tick%2 == 0)
					SET_LED_ON(LED_GREEN);
				else
					SET_LED_OFF(LED_GREEN);
				
				sleep_tick++;
			}
			else
			{
				if(led_tick < 9)
				{
					SET_LED_OFF(LED_GREEN);
					led_tick++;
				}
				else
				{
					SET_LED_ON(LED_GREEN);
					Delay_ms(100);
					SET_LED_OFF(LED_GREEN);
					led_tick = 0;
				}
			}
		}
		else
		{
			sleep_tick = 0;
			SET_LED_ON(LED_GREEN);
		}
		ke_timer_set(LED_TIMER_TASK, TASK_APP, 100);
	}
}

static int led_timer_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	led_timer_callback();
	return (KE_MSG_CONSUMED);
}

static void pair_timeout_callback(void)
{
	pair_timeout++;
	if(pair_timeout >= 10)
	{
		pair_timeout = 0;
		pair_flag = 0;
	}
	else
		ke_timer_set(PAIR_TIMEOUT_TASK, TASK_APP, 100);
}

static int pair_timeout_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	pair_timeout_callback();
	return (KE_MSG_CONSUMED);
}

static int app_utc_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	utc_callback();	
	return (KE_MSG_CONSUMED);
}

static int app_get_rec_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	rec_key_callback();	
	return (KE_MSG_CONSUMED);
}

static int key_scan_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	key_scan_callback();	
	return (KE_MSG_CONSUMED);
}
static int feed_run_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	feed_run_callback();	
	return (KE_MSG_CONSUMED);
}
static int plan_upload_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	plan_upload_callback();	
	return (KE_MSG_CONSUMED);
}
static int log_upload_handler(ke_msg_id_t const msgid,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	log_upload_callback();	
	return (KE_MSG_CONSUMED);
}

/* Default State handlers definition. */
const struct ke_msg_handler appm_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    	(ke_msg_func_t)appm_msg_handler},
    {GAPM_DEVICE_READY_IND,     	(ke_msg_func_t)gapm_device_ready_ind_handler},
    {GAPM_CMP_EVT,             		(ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,   	(ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT,             		(ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND,       	(ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPM_PROFILE_ADDED_IND,    	(ke_msg_func_t)gapm_profile_added_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,			(ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_PARAM_UPDATED_IND,		(ke_msg_func_t)gapc_param_updated_ind_handler},
    {GATTC_MTU_CHANGED_IND,			(ke_msg_func_t)gattc_mtu_changed_ind_handler},	
    {GAPC_PARAM_UPDATE_REQ_IND, 	(ke_msg_func_t)gapc_param_update_req_ind_handler},
    {APP_PARAM_UPDATE_REQ_IND, 		(ke_msg_func_t)gapc_update_conn_param_req_ind_handler},
    {APP_TUYA_CONNECT_MONITOR_TIMER, (ke_msg_func_t)app_tuya_connect_monitor_timer_handler},
	{APP_TUYA_PROD_MONITOR_TIMER,   (ke_msg_func_t)app_tuya_prod_monitor_timer_handler},
    {APP_TUYA_BLE_EVT,				(ke_msg_func_t)app_tuya_event_handler},
	{REC_KEY_TASK,		    		(ke_msg_func_t)app_get_rec_handler},
	{KEY_SCAN_TASK,		    		(ke_msg_func_t)key_scan_handler},
	{FEED_RUN_TASK,		    		(ke_msg_func_t)feed_run_handler},
	{UTC_TASK,		    			(ke_msg_func_t)app_utc_handler},
	{UPLOAD_PLAN_TASK,		    	(ke_msg_func_t)plan_upload_handler},
	{LOG_UPLOAD_TASK,		    	(ke_msg_func_t)log_upload_handler},
	{LED_TIMER_TASK,				(ke_msg_func_t)led_timer_handler},
	{PAIR_TIMEOUT_TASK,				(ke_msg_func_t)pair_timeout_handler},

};

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_default_state);

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
