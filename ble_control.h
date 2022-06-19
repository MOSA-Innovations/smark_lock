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

/**  @brief     BLE Connection Control.
 *   @details   This module contains most of the functions used
 *              by the application to manage BLE stack events
 *              and BLE connections.
 */

#ifndef BLE_CONTROL_H__
#define BLE_CONTROL_H__

#include <stdint.h>

#include "ble_advertising.h"

/**@brief Function for initializing the advertising functionality. */
static void advertising_init(void);
void new_advertising_init(void);

/**@brief   Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void);


/**@brief Function for initializing the Connection Parameters module. */
void conn_params_init(void);


/**@brief   Function for initializing GAP.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void);


/**@brief   Function for initializing the GATT module.
 *
 * @details This function initializes the module that handles Data Length update and GATT ATT_MTU
 *          exchanges procedures.
 */
void gatt_init(void);


/**@brief Function for initializing the Queued Write module.
 */
void qwr_init(void);


/**@brief Function for terminating the BLE connection. */
void ble_disconnect(void);


/**@brief Function for setting the BLE device name.
 *
 * @param[in] device_name  Pointer to a new device name string.
 */
void ble_set_device_name(char const * device_name);


/**@brief Function for getting the advertising module instance.
 *
 * @return Pointer to the advertising module instance.
 */
ble_advertising_t * ble_adv_instance_ptr_get(void);

extern uint16_t  m_conn_handle;

/**@brief Function for initializing BLE pairing module without NFC bonding.
 */
void ble_pairing_init(void);


void advertising_start(void);
#endif //BLE_CONTROL_H__
