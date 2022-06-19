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

#include <stdint.h>
#include "nfc_t2t_lib.h"
#include "nfc_ndef_msg.h"
#include "nfc_text_rec.h"
#include "boards.h"
#include "app_error.h"
#include "hardfault.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




#define MAX_REC_COUNT      2     /**< Maximum records count. */


/* Buffer used to hold an NFC NDEF message. */
uint8_t m_ndef_msg_buf[256];
static bool is_nfc_t2t_event_field_on = false;

bool is_nfc_event_field_on(void)
{
    if (is_nfc_t2t_event_field_on)
    {
        is_nfc_t2t_event_field_on= false;
        return true;
    }
    else
    {
        return false;
    }

}

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            //advertising_start();
            is_nfc_t2t_event_field_on = true;
            NRF_LOG_INFO("new nfc event");

            //bsp_board_led_on(BSP_BOARD_LED_0);
            
            break;
        case NFC_T2T_EVENT_FIELD_OFF:
            //bsp_board_led_off(BSP_BOARD_LED_0);
            break;
        default:
            break;
    }
}


/**
 * @brief Function for encoding the NDEF text message.
 */
static ret_code_t welcome_msg_encode(uint8_t * p_buffer, uint32_t * p_len)
{
    /** @snippet [NFC text usage_2] */
    ret_code_t err_code;

    /*
    uint32_t macaddress[2];

    //The address is DA:C2:E2:C3:81:BF

    macaddress[0] = NRF_FICR->DEVICEADDR[0];
    macaddress[1] = (NRF_FICR->DEVICEADDR[1]| 0xc000) & 0x0000FFFF;
    NRF_LOG_INFO(" Device Addr : 0x%4X%08X\r", macaddress[1], macaddress[0]);
     */
    /* Text message with the device name. */
    /** @snippet [NFC text usage_1] */
    static const uint8_t device_name_payload[] =
    {
        'S', 'm', 'a', 'r', 't', 'l', 'o', 'c', 'k'
    };
    static const uint8_t device_name_code[] = {'n', 'm'};
    /** @snippet [NFC text usage_1] */


    uint8_t mac_address_payload[6];

    mac_address_payload[5] = (NRF_FICR->DEVICEADDR[0]);
    mac_address_payload[4] = (NRF_FICR->DEVICEADDR[0]) >> 8;
    mac_address_payload[3] = (NRF_FICR->DEVICEADDR[0]) >> 16;
    mac_address_payload[2] = (NRF_FICR->DEVICEADDR[0]) >> 24;
    mac_address_payload[1] = (NRF_FICR->DEVICEADDR[1]| 0xc000);
    mac_address_payload[0] = (NRF_FICR->DEVICEADDR[1]| 0xc000) >> 8;

    NRF_LOG_INFO("MAC_ADDR: %x:%x:%x:%x:%x:%x\r", mac_address_payload[5], mac_address_payload[4], mac_address_payload[3], mac_address_payload[2], mac_address_payload[1], mac_address_payload[0]);


    static const uint8_t mac_address_code[] = {'a', 'd'};


    /* Create NFC NDEF text record description in English */
    NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_device_name_text_rec,
                                  UTF_8,
                                  device_name_code,
                                  sizeof(device_name_code),
                                  device_name_payload,
                                  sizeof(device_name_payload));
    /** @snippet [NFC text usage_2] */

    /* Create NFC NDEF text record description in Norwegian */
    NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_mac_address_text_rec,
                                  UTF_8,
                                  mac_address_code,
                                  sizeof(mac_address_code),
                                  mac_address_payload,
                                  sizeof(mac_address_payload));



    /* Create NFC NDEF message description, capacity - MAX_REC_COUNT records */
    /** @snippet [NFC text usage_3] */
    NFC_NDEF_MSG_DEF(nfc_text_msg, MAX_REC_COUNT);
    /** @snippet [NFC text usage_3] */

    /* Add text records to NDEF text message */
    /** @snippet [NFC text usage_4] */
    err_code = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_text_msg),
                                       &NFC_NDEF_TEXT_RECORD_DESC(nfc_device_name_text_rec));
    VERIFY_SUCCESS(err_code);
    /** @snippet [NFC text usage_4] */
    err_code = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_text_msg),
                                       &NFC_NDEF_TEXT_RECORD_DESC(nfc_mac_address_text_rec));
    VERIFY_SUCCESS(err_code);


    /** @snippet [NFC text usage_5] */
    err_code = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_text_msg),
                                   p_buffer,
                                   p_len);
    return err_code;
    /** @snippet [NFC text usage_5] */
}

int nfc_init(void)
{
    uint32_t  len = sizeof(m_ndef_msg_buf);
    uint32_t  err_code;

    /* Set up NFC */
    err_code = nfc_t2t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    /* Encode welcome message */
    err_code = welcome_msg_encode(m_ndef_msg_buf, &len);
    APP_ERROR_CHECK(err_code);

    /* Set created message as the NFC payload */
    err_code = nfc_t2t_payload_set(m_ndef_msg_buf, len);
    APP_ERROR_CHECK(err_code);

    /* Start sensing NFC field */
    err_code = nfc_t2t_emulation_start();
    APP_ERROR_CHECK(err_code);

    return err_code;

}

 //EOF