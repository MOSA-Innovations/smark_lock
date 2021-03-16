/**
 * Copyright (c) 2021, GoVelo
 *
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY GOVELO "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL GOVELO OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**  @brief     Smart Lock.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage the smart lock.
 */

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"

#include "smart_lock.h"
#include "smart_lock_core.h"
#include "smart_lock_gatt_service.h"
#include "power_management_control.h"
#include "timers_control.h"
#include "ble_control.h"
#include "nfc_control.h"
#include "button_control.h"
#include "led_control.h"
#include "log_control.h"
#include "peer_control.h"
#include "lock_mech_control.h"
#include "persistent_storage.h"
#include "security.h"
#include "audio_control.h"

/**@brief Function for resetting the smart lock.
 */
void smart_lock_reset(void)
{
    lock_mech_status_t lock_mech_status;

    lock_mech_status = lock_mech_init();
    led_init();
    audio_init();
    buttons_init();
    security_init();

    // Start execution.
    NRF_LOG_INFO("GoVelo Smart Lock Reset and Running");
}

void smart_lock_send_data(uint8_t data)
{
    uint8_t notify[2] = {0, '\n'};
    uint16_t len = 1;

    notify[0] = data;

    led_pulse();

    smart_lock_gatt_service_data_send(&m_smart_lock_gatt_service, notify, &len, m_conn_handle);
}

void smart_lock_parse_data(uint8_t *buf, int len)
{
    uint8_t cmd = buf[0];
    int sl_len = buf[1];

    switch (cmd)
    {
        case SL_UNLOCK_CMD:
        {
//            if (sl_info.update == false)
//            {
//                    NRF_LOG_INFO("SmartLocker is not updated yet");
//                    smart_lock_send_data(SL_DEV_NEED_UPDATE);
//                    goto out;
//            }

            security_check_status_t security_check_status = security_check(buf, len);

            switch(security_check_status)
            {
                case SECURITY_CHECK_PASSED:
                {
                    // TODO: Update LED state
                    audio_beep_unlock_success_event();
                    lock_mech_disengage();
                    smart_lock_send_data(SL_UNLOCK_SUCCESS);
                    break;
                }
                case SECURITY_CHECK_FAILED_OUT_OF_CODES:
                {
                    // TODO: Update LED state
                    audio_beep_unlock_fail_event();
                    smart_lock_send_data(SL_CODE_RUN_OUT);
                    break;
                }
                case SECURITY_CHECK_FAILED_INVALID_CODE:
                {
                    // TODO: Update LED state
                    audio_beep_unlock_fail_event();
                    smart_lock_send_data(SL_UNLOCK_FAIL);
                    break;
                }
                case SECURITY_CHECK_FAILED_CODE_OUT_OF_DATE:
                {
                    // TODO: Update LED state
                    audio_beep_unlock_fail_event();
                    smart_lock_send_data(SL_CODE_OUT_OF_DATE);
                    break;
                }
                case SECURITY_CHECK_FAILED_INVALID_DEV_ID:
                {
                    // TODO: Update LED state
                    audio_beep_unlock_fail_event();
                    smart_lock_send_data(SL_DEV_ID_FAIL);
                    break;
                }
            }
            break;
        }

        case SL_UPDATE_CODE_CMD:
        {
            security_update_codes(buf, sl_len);
            smart_lock_send_data(SL_UPDATE_SUCCESS);
            NRF_LOG_INFO("Update access codes successfully!");
            break;
        }
        case SL_LOCK_CMD:
        {
            NRF_LOG_INFO("Received Lock Command");

            lock_mech_status_t lock_mech_status = lock_mech_get_status();

            if(lock_mech_status != SMART_LOCK_UNLOCKED_CLOSED)
            {
                // TODO: Update LED state
                audio_beep_lock_fail_event();
                smart_lock_send_data(SL_LOCK_FAIL);
            }

            lock_mech_status = lock_mech_engage(true);

            if (lock_mech_status == SMART_LOCK_LOCKED)
            {
                // TODO: Update LED state
                audio_beep_lock_success_event();
                smart_lock_send_data(SL_LOCK_SUCCESS);
            }
            else
            {
                // TODO: Update LED state
                audio_beep_lock_fail_event();
                smart_lock_send_data(SL_LOCK_FAIL);
            }
            break;
        }
        case SL_APP_READY_CMD:
            break;
        case SL_RESET_CMD:
        {
            smart_lock_reset();
            smart_lock_send_data(SL_RESET_SUCCESS);
            break;
        }
        default:
        {
            NRF_LOG_INFO("Unknown CMD !");
        }
            break;
    }
}


/**@brief Function for handling the data from the GoVelo Smart Lock GATT Service.
 *
 * @details This function will process the data received from the GoVelo Smart Lock GATT Service.
 *
 * @param[in] p_evt       GoVelo Smart Lock GATT Service event.
 */
/**@snippet [Handling the data received over BLE] */
void smart_lock_rx_data_handler(smart_lock_gatt_service_evt_t * p_evt)
{
    uint8_t code[GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN] = {0};

    if (p_evt->type == GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA)
    {
        uint32_t err_code;

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            code[i] = p_evt->params.rx_data.p_data[i];
            if (code[i] >= 0x30 && code[i] <= 0x39)
              code[i] = code[i] - 0x30;
            else
              code[i] = code[i] - 0x57;
        }

        smart_lock_parse_data(code, p_evt->params.rx_data.length);
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
void smart_lock_services_init(void)
{
    uint32_t                          err_code;
    smart_lock_gatt_service_init_t    context_init;

    // Initialize context.
    memset(&context_init, 0, sizeof(smart_lock_gatt_service_init_t));

    context_init.data_handler = smart_lock_rx_data_handler;

    err_code = smart_lock_gatt_service_init(&m_smart_lock_gatt_service, &context_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the smart lock core.
 */
void smart_lock_init(void)
{
    lock_mech_status_t lock_mech_status;

    // Initialise
    log_init();
    lock_mech_status = lock_mech_init();
    led_init();
    audio_init();
    buttons_init();
    timer_init();
    power_management_init();

    ble_stack_init();
    ble_set_device_name(DEVICE_NAME);
    gap_params_init();
    gatt_init();
    qwr_init();
    smart_lock_services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init(true);
    NRF_LOG_INFO("Bonds erased!");
    nfc_pairing_init();
    peer_manager_delete_peers();
    persistent_storage_init();
    security_init();

    // Start execution.
    NRF_LOG_INFO("GoVelo Smart Lock Initialised and Running");
}

/**@brief Main loop of the smart lock.
 */
void smart_lock_run(void)
{
    // Enter main loop.
    while (true)
    {
        power_management_idle_state_handle();
        lock_mech_get_status();
    }
}

//EOF