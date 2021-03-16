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

/**  @brief     GoVelo Smart Lock GATT Service.
 *   @details   This module contains most of the functions used
 *              by the application to manage the custom GATT service
 *              and custom comms protocol between device and phone app.
 */

#include <stdlib.h>
#include <string.h>
#include "sdk_common.h"
#include "app_error.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "smart_lock_gatt_service.h"

#define NRF_LOG_MODULE_NAME SMART_LOCK_GATT_SERVICE
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

uint16_t m_smart_lock_gatt_service_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;   /**< Maximum length of data (in bytes) that can be transmitted to the peer by the GoVelo Smart Lock GATT Service module. */
SMART_LOCK_GATT_SERVIVE_DEF(m_smart_lock_gatt_service, NRF_SDH_BLE_TOTAL_LINK_COUNT);    /**< GoVelo Smart Lock GATT Service instance. */


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_context GoVelo Smart Lock GATT Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(smart_lock_gatt_service_t* p_context, ble_evt_t const * p_ble_evt)
{
    ret_code_t                                  err_code;
    smart_lock_gatt_service_evt_t               evt;
    ble_gatts_value_t                           gatts_val;
    uint8_t                                     cccd_value[2];
    smart_lock_gatt_service_client_context_t*   p_client = NULL;

    err_code = blcm_link_ctx_get(p_context->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_context->rx_handles.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS)     &&
        (p_context->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }

        memset(&evt, 0, sizeof(smart_lock_gatt_service_t));
        evt.type        = GOVELO_SMART_LOCK_GATT_SERVICE_EVT_COMM_STARTED;
        evt.p_context   = p_context;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_context->data_handler(&evt);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_context GoVelo Smart Lock GATT Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(smart_lock_gatt_service_t* p_context, ble_evt_t const * p_ble_evt)
{
    ret_code_t                                  err_code;
    smart_lock_gatt_service_evt_t               evt;
    smart_lock_gatt_service_client_context_t*   p_client;
    ble_gatts_evt_write_t const*                p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_context->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }
	
    memset(&evt, 0, sizeof(smart_lock_gatt_service_t));

    evt.p_context   = p_context;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;
 
    if ((p_evt_write->handle == p_context->tx_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = GOVELO_SMART_LOCK_GATT_SERVICE_EVT_COMM_STARTED;
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = GOVELO_SMART_LOCK_GATT_SERVICE_EVT_COMM_STOPPED;
            }

            if (p_context->data_handler != NULL)
            {
                p_context->data_handler(&evt);
            }

        }
    }
    else if ((p_evt_write->handle == p_context->rx_handles.value_handle) &&
             (p_context->data_handler != NULL))
    {
	evt.type                  = GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;
        p_context->data_handler(&evt);
    }
    else
    {
       // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_context GoVelo Smart Lock GATT Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(smart_lock_gatt_service_t* p_context, ble_evt_t const * p_ble_evt)
{
    ret_code_t                                err_code;
    smart_lock_gatt_service_evt_t             evt;
    smart_lock_gatt_service_client_context_t* p_client;
    
    err_code = blcm_link_ctx_get(p_context->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    //if (p_client->is_notification_enabled)
    //{
          memset(&evt, 0, sizeof(smart_lock_gatt_service_t));
          evt.type        = GOVELO_SMART_LOCK_GATT_SERVICE_EVT_TX_RDY;
          evt.p_context   = p_context;
          evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
          evt.p_link_ctx  = p_client;
          p_context->data_handler(&evt);
    //}
}





void smart_lock_gatt_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect((smart_lock_gatt_service_t*)p_context, p_ble_evt);
            break;
			
	case BLE_GATTS_EVT_WRITE:
            on_write((smart_lock_gatt_service_t*)p_context, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete((smart_lock_gatt_service_t*)p_context, p_ble_evt);
            break;

        default:
            break;
    }
}


uint32_t smart_lock_gatt_service_init(smart_lock_gatt_service_t * p_context, smart_lock_gatt_service_init_t const * p_context_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         smart_lock_base_uuid = GOVELO_SMART_LOCK_GATT_SERVICE_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_context);
    VERIFY_PARAM_NOT_NULL(p_context_init);

    p_context->data_handler = p_context_init->data_handler;

    err_code = sd_ble_uuid_vs_add(&smart_lock_base_uuid, &p_context->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_context->uuid_type;
    ble_uuid.uuid = GOVELO_SMART_LOCK_GATT_SERVICE_UUID;

    // Add a custom base UUID.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_context->service_handle);
    VERIFY_SUCCESS(err_code);

		// Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = GOVELO_SMART_LOCK_GATT_SERVICE_UUID_RX_CHARACTERISTIC;
    add_char_params.uuid_type                = p_context->uuid_type;
    add_char_params.max_len                  = GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint16_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_context->service_handle, &add_char_params, &p_context->rx_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the TX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = GOVELO_SMART_LOCK_GATT_SERVICE_UUID_TX_CHARACTERISTIC;
    add_char_params.uuid_type         = p_context->uuid_type;
    add_char_params.max_len           = GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_context->service_handle, &add_char_params, &p_context->tx_handles);
    return err_code;
}


uint32_t smart_lock_gatt_service_data_send(smart_lock_gatt_service_t* p_context, uint8_t* p_data, uint16_t* p_length, uint16_t conn_handle)
{
    ret_code_t                                err_code;
    ble_gatts_hvx_params_t                    hvx_params;
    smart_lock_gatt_service_client_context_t* p_client;

    VERIFY_PARAM_NOT_NULL(p_context);
	
    err_code = blcm_link_ctx_get(p_context->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_context->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}
// EOF