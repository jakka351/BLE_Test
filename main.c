#region Copyright (c) 2024, Jack Leighton
// /////     __________________________________________________________________________________________________________________
// /////
// /////                  __                   __              __________                                      __   
// /////                _/  |_  ____   _______/  |_  __________\______   \_______   ____   ______ ____   _____/  |_ 
// /////                \   __\/ __ \ /  ___/\   __\/ __ \_  __ \     ___/\_  __ \_/ __ \ /  ___// __ \ /    \   __\
// /////                 |  | \  ___/ \___ \  |  | \  ___/|  | \/    |     |  | \/\  ___/ \___ \\  ___/|   |  \  |  
// /////                 |__|  \___  >____  > |__|  \___  >__|  |____|     |__|    \___  >____  >\___  >___|  /__|  
// /////                           \/     \/            \/                             \/     \/     \/     \/      
// /////                                                          .__       .__  .__          __                    
// /////                               ____________   ____   ____ |__|____  |  | |__| _______/  |_                  
// /////                              /  ___/\____ \_/ __ \_/ ___\|  \__  \ |  | |  |/  ___/\   __\                 
// /////                              \___ \ |  |_> >  ___/\  \___|  |/ __ \|  |_|  |\___ \  |  |                   
// /////                             /____  >|   __/ \___  >\___  >__(____  /____/__/____  > |__|                   
// /////                                  \/ |__|        \/     \/        \/             \/                         
// /////                                  __                         __  .__                                        
// /////                   _____   __ ___/  |_  ____   _____   _____/  |_|__|__  __ ____                            
// /////                   \__  \ |  |  \   __\/  _ \ /     \ /  _ \   __\  \  \/ // __ \                           
// /////                    / __ \|  |  /|  | (  <_> )  Y Y  (  <_> )  | |  |\   /\  ___/                           
// /////                   (____  /____/ |__|  \____/|__|_|  /\____/|__| |__| \_/  \___  >                          
// /////                        \/                         \/                          \/                           
// /////                                                  .__          __  .__                                      
// /////                                       __________ |  |  __ ___/  |_|__| ____   ____   ______                
// /////                                      /  ___/  _ \|  | |  |  \   __\  |/  _ \ /    \ /  ___/                
// /////                                      \___ (  <_> )  |_|  |  /|  | |  (  <_> )   |  \\___ \                 
// /////                                     /____  >____/|____/____/ |__| |__|\____/|___|  /____  >                
// /////                                          \/                                      \/     \/                 
// /////                                   Tester Present Specialist Automotive Solutions
// /////     __________________________________________________________________________________________________________________
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////      |       https://github.com/jakka351/| https://testerPresent.com.au | https://facebook.com/testerPresent        |
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////      | Copyright (c) 2022/2023/2024 Benjamin Jack Leighton                                                          |          
// /////      | All rights reserved.                                                                                         |
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////        Redistribution and use in source and binary forms, with or without modification, are permitted provided that
// /////        the following conditions are met:
// /////        1.    With the express written consent of the copyright holder.
// /////        2.    Redistributions of source code must retain the above copyright notice, this
// /////              list of conditions and the following disclaimer.
// /////        3.    Redistributions in binary form must reproduce the above copyright notice, this
// /////              list of conditions and the following disclaimer in the documentation and/or other
// /////              materials provided with the distribution.
// /////        4.    Neither the name of the organization nor the names of its contributors may be used to
// /////              endorse or promote products derived from this software without specific prior written permission.
// /////      _________________________________________________________________________________________________________________
// /////      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// /////      INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// /////      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// /////      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// /////      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// /////      WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// /////      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// /////      _________________________________________________________________________________________________________________
// /////
// /////       This software can only be distributed with my written permission. It is for my own educational purposes and  
// /////       is potentially dangerous to ECU health and safety. Gracias a Gato Blancoford desde las alturas del mar de chelle.                                                        
// /////      _________________________________________________________________________________________________________________
// /////
// /////
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endregion License
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/* ChatGPT */
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */
#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */
#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

NRF_BLE_SCAN_DEF(m_scan); // Scanning module instance
BLE_DB_DISCOVERY_DEF(m_db_disc); // Database discovery module instance
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm, // USB CDC ACM instance
 cdc_acm_user_ev_handler,
 NRF_DRV_USBD_EPIN2,
 NRF_DRV_USBD_EPOUT2,
 NRF_DRV_USBD_EPIN3,
 APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */                                      /**< Database discovery module instance. */                                             /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


static void ble_stack_init(void)
{
   ret_code_t err_code;
   // Initialize SoftDevice handler module.
   err_code = nrf_sdh_enable_request();
   APP_ERROR_CHECK(err_code);
   // Configure the BLE stack using the default settings.
   uint32_t ram_start = 0;
   err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
   APP_ERROR_CHECK(err_code);
   // Enable BLE stack.
   err_code = nrf_sdh_ble_enable(&ram_start);
   APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
// BLE STACK INIT
static void ble_stack_init(void)
{
   ret_code_t err_code;
   // Initialize SoftDevice handler module.
   err_code = nrf_sdh_enable_request();
   APP_ERROR_CHECK(err_code);
   // Configure the BLE stack using the default settings.
   uint32_t ram_start = 0;
   err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
   APP_ERROR_CHECK(err_code);
   // Enable BLE stack.
   err_code = nrf_sdh_ble_enable(&ram_start);
   APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}
static void scan_init(void)
{
   ret_code_t err_code;
   nrf_ble_scan_init_t init_scan = {0};
   // Set the scanning parameters
   init_scan.connect_if_match = false;
   init_scan.p_scan_param = NULL;
   err_code = nrf_ble_scan_init(&m_scan, &init_scan, NULL);
   APP_ERROR_CHECK(err_code);
}
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
   switch(p_scan_evt->scan_evt_id)
   {
     case NRF_BLE_SCAN_EVT_FILTER_MATCH:
     case NRF_BLE_SCAN_EVT_FILTER_NO_MATCH:
     {
        // Get the advertising report
        const ble_gap_evt_adv_report_t * p_adv_report = p_scan_evt->params.filter_match.p_adv_report;
        // Extract the MAC address
        char addr_str[BLE_GAP_ADDR_LEN * 2 + 1];
        for (int i = 0; i < BLE_GAP_ADDR_LEN; i++)
        {
          sprintf(&addr_str[i * 2], "%02X", p_adv_report->peer_addr.addr[BLE_GAP_ADDR_LEN - i - 1]);
        }
        // Send the MAC address over USB CDC ACM
        app_usbd_cdc_acm_write(&m_app_cdc_acm, addr_str, strlen(addr_str));
        app_usbd_cdc_acm_write(&m_app_cdc_acm, "\r\n", 2);
     } break;
    default:
      break;
    }
 }

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event)
{
   switch (event)
   {
     case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        // Set up the first transfer
        app_usbd_cdc_acm_read(&m_app_cdc_acm, NULL, 0);
        break;
     case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        break;
     case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        break;
     case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        break;
     default:
        break;
     }
}
static void usbd_init(void)
{
   ret_code_t err_code;
   // Initialize the USB stack
   app_usbd_config_t usbd_config = {
   .ev_state_proc = app_usbd_event_execute
   };
   err_code = app_usbd_init(&usbd_config);
   APP_ERROR_CHECK(err_code);
   // Add the CDC ACM class
   app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
   err_code = app_usbd_class_append(class_cdc_acm);
   APP_ERROR_CHECK(err_code);
}


int main(void)
{
   ret_code_t err_code;
   // Initialize the clock driver
   err_code = nrf_drv_clock_init();
   APP_ERROR_CHECK(err_code);
   // Initialize power management
   err_code = nrf_pwr_mgmt_init();
   APP_ERROR_CHECK(err_code);
   // Initialize the USB driver
   usbd_init();
   // Start the USB stack
   err_code = app_usbd_power_events_enable();
   APP_ERROR_CHECK(err_code);
   // Initialize the BLE stack
   ble_stack_init();
   // Initialize scanning
   scan_init();
   // Start scanning
   scan_start();
   while (true)
   {
      while (app_usbd_event_queue_process())
      {
        // Nothing to do
      }
      // Sleep until an interrupt occurs
      nrf_pwr_mgmt_run();
   }
}
