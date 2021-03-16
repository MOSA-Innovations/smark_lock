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

/**  @brief     Peer Manager.
 *   @details   This module contains functions used by the application
 *              to set up the Peer Manager and handle its events.
 */

#include "peer_control.h"
#include "ble_conn_state.h"
#include "ble_m.h"
#include "fds.h"
#include "sdk_errors.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nfc_ble_pair_lib.h"

#define NRF_LOG_MODULE_NAME PEER_CONTROL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_PARAMS_REQ:
            // Send event to the NFC BLE pairing library as it may dynamically alternate
            // security parameters to achieve highest possible security level.
            err_code = nfc_ble_pair_on_pm_params_req(p_evt);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

void peer_manager_init(bool erase_bonds)
{
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

void peer_manager_delete_peers(void)
{
    pm_peers_delete();
}


