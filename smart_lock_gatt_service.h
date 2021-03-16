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

#ifndef SMART_LOCK_GATT_SERVICE_H__
#define SMART_LOCK_GATT_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MANUFACTURER_NAME                                   "GoVelo"        /**< Manufacturer. Will be passed to Device Information Service. */
#define GOVELO_SMART_LOCK_GATT_SERVICE_OBSERVER_PRIORITY    2               /**< Priority with which BLE events are dispatched to the UART Service. */

/**@brief   Macro for defining a GoVelo Smart Lock GATT Service instance.
 *
 * @param     _name           Name of the instance.
 * @param[in] _max_clients    Maximum number of clients connected at a time.
 * @hideinitializer
 */
#define SMART_LOCK_GATT_SERVIVE_DEF(_name, _max_clients)        \
  BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
      (_max_clients),                                           \
      sizeof(smart_lock_gatt_service_client_context_t));        \
  smart_lock_gatt_service_t _name =                             \
  {                                                             \
      .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
  };                                                            \
  NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
       GOVELO_SMART_LOCK_GATT_SERVICE_OBSERVER_PRIORITY,        \
       smart_lock_gatt_service_on_ble_evt,                      \
       &_name)


// 128bis UUID
#define GOVELO_SMART_LOCK_GATT_SERVICE_BASE_UUID              {{0x40, 0xE3, 0x4A, 0x1D, 0xC2, 0x5F, 0xB0, 0x9C, 0xB7, 0x47, 0xE6, 0x43, 0x00, 0x00, 0x53, 0x86}}
#define GOVELO_SMART_LOCK_GATT_SERVICE_UUID_TYPE              BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the GoVelo Smart Lock GATT Service (vendor specific). */

// 16bit UUID of characteristic and service
#define GOVELO_SMART_LOCK_GATT_SERVICE_UUID                   0x000A
#define GOVELO_SMART_LOCK_GATT_SERVICE_UUID_TX_CHARACTERISTIC 0x000B           
#define GOVELO_SMART_LOCK_GATT_SERVICE_UUID_RX_CHARACTERISTIC 0x000C

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the GoVelo Smart Lock GATT Service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define GOVELO_SMART_LOCK_GATT_SERVICE_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

/**@brief   GoVelo Smart Lock GATT Service event types. */
typedef enum
{
    GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA,       /**< Data received. */
    GOVELO_SMART_LOCK_GATT_SERVICE_EVT_TX_RDY,        /**< Service is ready to accept new data to be transmitted. */
    GOVELO_SMART_LOCK_GATT_SERVICE_EVT_COMM_STARTED,  /**< Notification has been enabled. */
    GOVELO_SMART_LOCK_GATT_SERVICE_EVT_COMM_STOPPED,  /**< Notification has been disabled. */
} smart_lock_gatt_service_evt_type_t;	

typedef struct smart_lock_gatt_service_s smart_lock_gatt_service_t;

/**@brief   GoVelo Smart Lock GATT Service @ref GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} smart_lock_gatt_service_evt_rx_data_t;



/**< Indicates if the peer has enabled notification of the RX characteristic.*/
typedef struct
{
    bool is_notification_enabled; 
} smart_lock_gatt_service_client_context_t;


/**@brief   GoVelo Smart Lock GATT Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    smart_lock_gatt_service_evt_type_t          type;         /**< Event type. */
    smart_lock_gatt_service_t*                  p_context;    /**< A pointer to the instance. */
    uint16_t                                    conn_handle;  /**< Connection handle. */
    smart_lock_gatt_service_client_context_t*   p_link_ctx;   /**< A pointer to the link context. */
    union
    {
        smart_lock_gatt_service_evt_rx_data_t   rx_data;      /**< @ref GOVELO_SMART_LOCK_GATT_SERVICE_EVT_RX_DATA event data. */
    } params;
} smart_lock_gatt_service_evt_t;

/**@brief GoVelo Smart Lock GATT Service event handler type. */
typedef void (* smart_lock_gatt_service_data_handler_t) (smart_lock_gatt_service_evt_t * p_evt);


/**@brief   GoVelo Smart Lock GATT Service Structure.
 *
 * @details This structure contains status information related to the service.
 */
struct smart_lock_gatt_service_s
{
    uint8_t                         uuid_type;          /**< UUID type for GoVelo Smart Lock GATT Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of GoVelo Smart Lock GATT Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    smart_lock_gatt_service_data_handler_t data_handler;/**< Event handler to be called for handling received data. */
};

/**@brief   GoVelo Smart Lock GATT Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref smart_lock_gatt_service_init
 *          function.
 */
typedef struct
{
    smart_lock_gatt_service_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} smart_lock_gatt_service_init_t;


/**@brief   Function for initializing the GoVelo Smart Lock GATT Service.
 *
 * @param[out] p_context    GoVelo Smart Lock GATT Service structure. This structure must be supplied
 *                          by the application. It is initialized by this function and will
 *                          later be used to identify this particular service instance.
 * @param[in] p_context_init   Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_context or p_inst_init is NULL.
 */
uint32_t smart_lock_gatt_service_init(smart_lock_gatt_service_t * p_context, smart_lock_gatt_service_init_t const * p_context_init);




/**@brief   Function for handling the GoVelo Smart Lock GATT Service's BLE events.
 *
 * @details The application should call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the GoVelo Smart Lock GATT Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     GoVelo Smart Lock GATT Service structure.
 */
void smart_lock_gatt_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_context   Pointer to the GoVelo Smart Lock GATT Service structure.
 * @param[in]     p_data      Data to be sent.
 * @param[in,out] p_length    Pointer length of the data. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t smart_lock_gatt_service_data_send(smart_lock_gatt_service_t* p_context, uint8_t* p_data, uint16_t* p_length, uint16_t conn_handle);


extern uint16_t m_smart_lock_gatt_service_max_data_len;
extern smart_lock_gatt_service_t m_smart_lock_gatt_service;

#ifdef __cplusplus
}
#endif

#endif //SMART_LOCK_GATT_SERVICE