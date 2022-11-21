/**
 ****************************************************************************************
 *
 * @file user_peripheral.c
 *
 * @brief Peripheral project source code.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include "gap.h"
#include "app_easy_timer.h"
#include "user_peripheral.h"
#include "user_custs1_impl.h"
#include "user_custs1_def.h"
#include "co_bt.h"
#include "uart.h"
#include "spi_flash.h"
#include "sw_aes.h"
#include "aes.h"

#include "gpio.h"
#include "app_api.h"
#include "user_periph_setup.h"

#include "user_version.h"

extern void handle_fn(void);
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


void user_app_on_db_init_complete(void)
{
    /* Set application version number in DISS */
    char sw_version[] = SDK_VERSION;
    struct diss_set_value_req *req = KE_MSG_ALLOC_DYN(DISS_SET_VALUE_REQ,
                                                      prf_get_task_from_id(TASK_ID_DISS),
                                                      TASK_APP,
                                                      diss_set_value_req,
                                                      sizeof(sw_version));
    req->value = DIS_SW_REV_STR_CHAR;
    req->length = sizeof(sw_version);
    memcpy(req->data, sw_version, sizeof(sw_version));
    ke_msg_send(req);

    default_app_on_db_init_complete();
}

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

bool my_button __SECTION_ZERO("retention_mem_area0"); // @RETENTION MEMORY


uint8_t app_connection_idx                      __SECTION_ZERO("retention_mem_area0");
timer_hnd app_adv_data_update_timer_used        __SECTION_ZERO("retention_mem_area0");
timer_hnd app_param_update_request_timer_used   __SECTION_ZERO("retention_mem_area0");

// Retained variables
struct mnf_specific_data_ad_structure mnf_data  __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
// Index of manufacturer data in advertising data or scan response data (when MSB is 1)
uint8_t mnf_data_index                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data_len                     __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data_len                __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data[ADV_DATA_LEN]           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data[SCAN_RSP_DATA_LEN] __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
	uint8_t data1[22]={0x11,0x39,0x55,0x45,0xA0,0x0F,0xD0,0xC0,0xE0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
	int8_t ret;
		int8_t array[22];
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/


/**
 ****************************************************************************************
 * @brief Advertisement data update timer callback function.
 ****************************************************************************************
*/
static void adv_data_update_timer_cb()
{
       // Update advertising data on the fly
    app_easy_gap_update_adv_data(stored_adv_data, stored_adv_data_len, stored_scan_rsp_data, stored_scan_rsp_data_len);

    // Restart timer for the next advertising update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 ****************************************************************************************
*/
static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}
///

///
void user_app_init(void)
{
	extern void lock();
	lock();
	handle_button_enable();
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;


    // Initialize Advertising and Scan Response Data
    memcpy(stored_adv_data, USER_ADVERTISE_DATA, USER_ADVERTISE_DATA_LEN);
    stored_adv_data_len = USER_ADVERTISE_DATA_LEN;
    memcpy(stored_scan_rsp_data, USER_ADVERTISE_SCAN_RESPONSE_DATA, USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN);
    stored_scan_rsp_data_len = USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN;
//Interupt
//	GPIO_EnableIRQ(GPIO_HANDLE_PORT, GPIO_HANDLE_PIN, GPIO0_IRQn, true, true, 150);
//    GPIO_RegisterCallback(GPIO0_IRQn, handle_fn);
//
    // Start AES encryption/decryption process
   
	//user_flash_write();
	//	user_flash_read();
	//retention mem
	my_button = false;
		extern bool my_m1_state;
		extern bool my_m2_state;
		extern bool my_en_state;
	my_en_state = false;
	my_m1_state = false;
	my_m2_state = false;
    default_app_on_init();
}

void user_app_adv_start(void)
{
    // Schedule the next advertising data update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);

//    struct gapm_start_advertise_cmd* cmd;
 //   cmd = app_easy_gap_undirected_advertise_get_active();


    app_easy_gap_undirected_advertise_start();
}

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = connection_idx;

        // Stop the advertising data update timer
        app_easy_timer_cancel(app_adv_data_update_timer_used);

        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }
    }
    else
    {
        // No connection has been established, restart advertising
        user_app_adv_start();
    }

    default_app_on_connection(connection_idx, param);
}

void user_app_adv_undirect_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status == GAP_ERR_CANCELED)
    {
        user_app_adv_start();
    }
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    // Cancel the parameter update request timer
    if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_param_update_request_timer_used);
        app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    }

    // Restart Advertising
    user_app_adv_start();
		
		
		// Issue a platform reset when it is requested by the suotar procedure
if (suota_state.reboot_requested)
{
    // Reboot request will be served
    suota_state.reboot_requested = 0;

    // Platform reset
    platform_reset(RESET_AFTER_SUOTA_UPDATE);
}
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {


                case SVC1_IDX_LED_STATE_VAL:
                    user_svc1_led_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;
								case SVC1_IDX_ADC_VAL_1_NTF_CFG:
                    user_svc1_adc_val_1_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;


                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_NTF_CFM:
        {
            struct custs1_val_ntf_cfm const *msg_param = (struct custs1_val_ntf_cfm const *)(param);

            switch (msg_param->handle)
            {
									case SVC1_IDX_ADC_VAL_1_VAL:
											break;
                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_IND_CFM:
        {
            struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);

            switch (msg_param->handle)
            {
           
                default:
                    break;
             }
        } break;

        case CUSTS1_ATT_INFO_REQ:
        {
            struct custs1_att_info_req const *msg_param = (struct custs1_att_info_req const *)param;

            switch (msg_param->att_idx)
            {


                default:
                    break;
             }
        } break;

        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preferred ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        case CUSTS1_VALUE_REQ_IND:
        {
            struct custs1_value_req_ind const *msg_param = (struct custs1_value_req_ind const *) param;

            switch (msg_param->att_idx)
            {


                default:
                {
                    // Send Error message
                    struct custs1_value_req_rsp *rsp = KE_MSG_ALLOC(CUSTS1_VALUE_REQ_RSP,
                                                                    src_id,
                                                                    dest_id,
                                                                    custs1_value_req_rsp);

                    // Provide the connection index.
                    rsp->conidx  = app_env[msg_param->conidx].conidx;
                    // Provide the attribute index.
                    rsp->att_idx = msg_param->att_idx;
                    // Force current length to zero.
                    rsp->length = 0;
                    // Set Error status
                    rsp->status  = ATT_ERR_APP_ERROR;
                    // Send message
                    ke_msg_send(rsp);
                } break;
             }
        } break;

        default:
            break;
    }
}
char buffer2[30];
	char buffer3[30];
void user_flash_read()
{
	
		GPIO_Disable_HW_Reset();
//		uint32_t bytes_read;
//		
//		array[sizeof(data1)] = spi_flash_read_data(&data1[0], 0x07000, 22, &bytes_read);
//		sprintf(buffer3,"\n\rRead Status: %d", ret);
//		uart_send(UART2,buffer3, 18,UART_OP_BLOCKING);
//		sprintf(buffer3,"\n\rBytes Read: %d", bytes_read);
//		uart_send(UART2,buffer3, 17,UART_OP_BLOCKING);	
////		for(z=0;z<22;z++)
////	{
////		printf_byte(UART1, data1[z]);
//	}
	
	
			//Reading from SPI Flash
    uint32_t bytes_read;
    uint8_t data_read[8] = { 0 };

    ret = spi_flash_read_data(&data_read[0], 0x7001, sizeof(data_read), &bytes_read);
    while (bytes_read) {
        bytes_read--;
    }
///read end ///
	
		GPIO_Enable_HW_Reset();
		
}
void user_flash_write()
{
		GPIO_Disable_HW_Reset();
	
//		uint32_t bytes_written;
//		spi_flash_block_erase(0x07000, 0x20);
//	//	ret = spi_flash_write_data(&data1[0], 0x07000, 22, &bytes_written);
//		ret = spi_flash_write_data(&data1[0], 0x07000, 22, &bytes_written);
//		sprintf(buffer2,"\n\rWrite Status: %d", ret);
//		uart_send(UART2,buffer2,19 ,UART_OP_BLOCKING);
//		sprintf(buffer2,"\n\rBytes Written: %d", bytes_written);
//		uart_send(UART2,buffer2, 20,UART_OP_BLOCKING);

    //ret = spi_flash_block_erase(0x7000, SPI_FLASH_OP_SE);
    //arch_printf("\n\rErase Status: %d", ret);
		//Writing to SPI Flash
    uint32_t bytes_written;
    uint8_t data[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    ret = spi_flash_write_data(&data[0], 0x7000, sizeof(data), &bytes_written);
		GPIO_Enable_HW_Reset();
}

void on_suotar_status_change(const uint8_t suotar_event)
{
#if (!SUOTAR_SPI_DISABLE)
    uint8_t dev_id;

    // Release Flash from power down
    spi_flash_release_from_power_down();

    // Try to auto-detect the device
    spi_flash_auto_detect(&dev_id);

    if (suotar_event == SUOTAR_END)
    {
        // Power down SPI Flash
        spi_flash_power_down();
    }
#endif
}

/// @} APP
