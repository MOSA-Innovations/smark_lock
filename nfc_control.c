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

/**  @brief     NFC Connection Control.
 *   @details   This module contains most of the functions used
 *              by the application to manage NFC stack events
 *              and NFC connections.
 */

 #include "nfc_control.h"
 #include "nfc_ble_pair_lib.h"

 #include "ble_control.h"

#ifndef NFC_PAIRING_MODE
    #define NFC_PAIRING_MODE NFC_PAIRING_MODE_JUST_WORKS
#endif


/**@brief Function for initializing NFC BLE pairing module.
 */
void nfc_pairing_init()
{
    ble_advertising_t * const p_advertising = ble_adv_instance_ptr_get();

    ret_code_t err_code = nfc_ble_pair_init(p_advertising, (nfc_pairing_mode_t)NFC_PAIRING_MODE);
    APP_ERROR_CHECK(err_code);
}

 //EOF