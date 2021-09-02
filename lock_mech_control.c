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

/**  @brief     Lock Mech Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage Lock Mech events.
 */

#include "lock_mech_control.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

#include "smart_lock.h"
#include "led_control.h"

#if defined(NRF52840_XXAA)
#define CHAIN_DETECT_PIN    NRF_GPIO_PIN_MAP(1, 3)
#define LOCK_MOTOR_DIR_PIN  NRF_GPIO_PIN_MAP(1, 1)
#define LOCK_MOTOR_STEP_PIN NRF_GPIO_PIN_MAP(1, 2)
#elif defined(NRF52832_XXAA)
//#define CHAIN_DETECT_PIN    NRF_GPIO_PIN_MAP(0, 2)
//#define LOCK_MOTOR_DIR_PIN  NRF_GPIO_PIN_MAP(0, 3)
//#define LOCK_MOTOR_STEP_PIN NRF_GPIO_PIN_MAP(0, 4)
#define CHAIN_DETECT_PIN    NRF_GPIO_PIN_MAP(0, 2)
#define LOCK_MOTOR_DIR_PIN  NRF_GPIO_PIN_MAP(0, 3)
#define LOCK_MOTOR_STEP_PIN NRF_GPIO_PIN_MAP(0, 4)
#else
#define CHAIN_DETECT_PIN    NRF_GPIO_PIN_MAP(0, 0)
#define LOCK_MOTOR_DIR_PIN  NRF_GPIO_PIN_MAP(0, 0)
#define LOCK_MOTOR_STEP_PIN NRF_GPIO_PIN_MAP(0, 0)
#endif

#define NUM_PULSES          (500)
#define PULSE_WIDTH_MS      (1)

typedef struct
{
    bool new_chain_attach_event;
    bool is_chain_attached;
    bool is_chain_locked;
    lock_mech_status_t lock_mech_status;
} lock_mech_state_t;

lock_mech_state_t lock_mech_state;

static bool is_chain_attached(void)
{
    return ~nrf_drv_gpiote_in_is_set(CHAIN_DETECT_PIN);
}

static bool is_chain_locked(void)
{
    return nrf_gpio_pin_out_read(LOCK_MOTOR_DIR_PIN);
}

static void lock_mech_update_status(void)
{
    if (lock_mech_state.new_chain_attach_event) 
    {
        lock_mech_state.new_chain_attach_event = false;
        nrf_delay_ms(1000);
        if(is_chain_attached() && lock_mech_state.lock_mech_status >= SMART_LOCK_UNLOCKED_CLOSED)
        {
            lock_mech_engage(false);
            smart_lock_send_data(SL_LOCK_SUCCESS);
        }
    }

    lock_mech_state.is_chain_attached = is_chain_attached();
    lock_mech_state.is_chain_locked = is_chain_locked();

    if (lock_mech_state.is_chain_attached == true && 
        lock_mech_state.is_chain_locked == true)
    {
        lock_mech_state.lock_mech_status = SMART_LOCK_LOCKED;
    }
    else if (lock_mech_state.is_chain_attached == true &&
             lock_mech_state.is_chain_locked == false)
    {
        lock_mech_state.lock_mech_status = SMART_LOCK_UNLOCKED_CLOSED;
    }
    else if (lock_mech_state.is_chain_attached == false &&
             lock_mech_state.is_chain_locked == false)
    {
        lock_mech_state.lock_mech_status = SMART_LOCK_UNLOCKED_OPEN;
    }
    else 
    {
        lock_mech_state.lock_mech_status = SMART_LOCK_MECH_ERROR;
    }
}

lock_mech_status_t lock_mech_get_status(void)
{
    lock_mech_update_status();

    /*switch(lock_mech_state.lock_mech_status)
    {
        case SMART_LOCK_LOCKED:
            NRF_LOG_INFO("Locking Mechanism Status: SMART_LOCK_LOCKED");
            break;
        case SMART_LOCK_UNLOCKED_CLOSED:
            NRF_LOG_INFO("Locking Mechanism Status: SMART_LOCK_UNLOCKED_CLOSED");
            break;
        case SMART_LOCK_UNLOCKED_OPEN:
            NRF_LOG_INFO("Locking Mechanism Status: SMART_LOCK_UNLOCKED_OPEN");
            break;
        case SMART_LOCK_MECH_ERROR:
            NRF_LOG_INFO("Locking Mechanism Status: SMART_LOCK_MECH_ERROR");
            break;
    }*/
    
    return lock_mech_state.lock_mech_status;
}

lock_mech_status_t lock_mech_engage(bool check_status)
{
    lock_mech_status_t lock_mech_status = SMART_LOCK_MECH_ERROR;
    NRF_LOG_INFO("Engage Locking Mechanism");
    nrf_drv_gpiote_out_set(LOCK_MOTOR_DIR_PIN);
    nrf_delay_ms(1000);
    for(int k = 0; k < NUM_PULSES; k++)
    {
        nrf_drv_gpiote_out_set(LOCK_MOTOR_STEP_PIN);
        nrf_delay_ms(PULSE_WIDTH_MS);
        nrf_drv_gpiote_out_clear(LOCK_MOTOR_STEP_PIN);
        nrf_delay_ms(PULSE_WIDTH_MS);
    }

    if (check_status)
    {
        lock_mech_status = lock_mech_get_status();
    }
    else
    {
        lock_mech_status = SMART_LOCK_LOCKED;
    }

    led_lock_event();
    return lock_mech_status;
}

lock_mech_status_t lock_mech_disengage(void)
{
    NRF_LOG_INFO("Disengage Locking Mechanism");
    nrf_drv_gpiote_out_clear(LOCK_MOTOR_DIR_PIN);
    nrf_delay_ms(1000);
    for(int k = 0; k < NUM_PULSES; k++)
    {
        nrf_drv_gpiote_out_set(LOCK_MOTOR_STEP_PIN);
        nrf_delay_ms(PULSE_WIDTH_MS);
        nrf_drv_gpiote_out_clear(LOCK_MOTOR_STEP_PIN);
        nrf_delay_ms(PULSE_WIDTH_MS);
    }
    led_unlock_event();
    return lock_mech_get_status();
}

void chain_detect_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == GPIOTE_CONFIG_POLARITY_HiToLo) {
        lock_mech_state.new_chain_attach_event = true;
    }
}

lock_mech_status_t lock_mech_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t lock_motor_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code = nrf_drv_gpiote_out_init(LOCK_MOTOR_DIR_PIN, &lock_motor_dir_pin_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t lock_motor_step_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(LOCK_MOTOR_STEP_PIN, &lock_motor_step_pin_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t chain_detect_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    chain_detect_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(CHAIN_DETECT_PIN, &chain_detect_config, chain_detect_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(CHAIN_DETECT_PIN, true);

    lock_mech_state.new_chain_attach_event = false;
    lock_mech_state.is_chain_attached = true;
    lock_mech_state.is_chain_locked = true;
    lock_mech_state.lock_mech_status = SMART_LOCK_LOCKED;

    return lock_mech_engage(true);
}
//EOF