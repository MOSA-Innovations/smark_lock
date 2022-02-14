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
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"

#include "hardware.h"
#include "led_control.h"
#include "smart_lock.h"

#define NUM_PULSES_ADMIN (900)
#define NUM_PULSES (800)
#define PULSE_WIDTH_MS (2000)

#define FULL_STEP (0)
#define HALF_STEP (1)
#define QUAD_STEP (2)
#define OCTA_STEP (3)
#define SIXT_STEP (4)
#define CW (0)
#define CCW (1)

typedef struct
{
  bool new_chain_attach_event;
  bool is_chain_attached;
  bool is_chain_locked;
  bool new_admin_chain_attach_event;
  bool is_admin_chain_attached;
  bool is_admin_chain_locked;
  lock_mech_status_t lock_mech_status;
} lock_mech_state_t;

lock_mech_state_t lock_mech_state;

static bool is_chain_attached(void) {
  return nrf_gpio_pin_read(CHAIN_DETECT_PIN_A);
}

static bool is_chain_locked(void) {
  return !nrf_gpio_pin_out_read(STP1_DIR);
}

static bool is_admin_chain_attached(void) {
  return nrf_gpio_pin_read(CHAIN_DETECT_PIN_B);
}

static bool is_admin_chain_locked(void) {
  return !nrf_gpio_pin_out_read(STP2_DIR);
}

/**@brief Function for powering down both motors.
 *
 * @details Turns off both motors. Call after motor use to save power.
 */
void motor_powerdown() {
  nrf_gpio_pin_write(STP1_STANDBY, 0);
  nrf_gpio_pin_write(STP2_STANDBY, 0);
  nrf_gpio_pin_write(STP1_STCK, 0);
  nrf_gpio_pin_write(STP2_STCK, 0);
}

/**@brief Function for controling motor A.
 *
 * @details  Controls motor A only.
 */
void motorA_loop(uint32_t pulse_width, uint16_t steps, bool dir, uint8_t mode) {
  // Wake up the motor drivers
  nrf_gpio_pin_write(STP1_STANDBY, 1);
  // Set the mode and the direction
  nrf_gpio_pin_write(STP1_MODE1, mode & 1);
  nrf_gpio_pin_write(STP1_MODE2, mode & 2);
  nrf_gpio_pin_write(STP1_MODE3, mode & 4);
  nrf_gpio_pin_write(STP1_DIR, dir);
  nrf_delay_ms(1000);
  for (int i = 0; i < steps; ++i) {
    nrf_gpio_pin_write(STP1_STCK, 1);
    nrf_delay_us(PULSE_WIDTH_MS);
    nrf_gpio_pin_write(STP1_STCK, 0);
    nrf_delay_us(PULSE_WIDTH_MS);
  }
  motor_powerdown();
}

/**@brief Function for controling both motors simultaneously.
 *
 * @details  Controls both motors at the same time.
 */
void motor_loop(uint32_t pulse_width, uint16_t steps, bool dir, uint8_t mode) {
  // Wake up the motor drivers
  nrf_gpio_pin_write(STP1_STANDBY, 1);
  nrf_gpio_pin_write(STP2_STANDBY, 1);
  // Set the mode and the direction
  nrf_gpio_pin_write(STP1_MODE1, mode & 1);
  nrf_gpio_pin_write(STP2_MODE1, mode & 1);
  nrf_gpio_pin_write(STP1_MODE2, mode & 2);
  nrf_gpio_pin_write(STP2_MODE2, mode & 2);
  nrf_gpio_pin_write(STP1_MODE3, mode & 4);
  nrf_gpio_pin_write(STP2_MODE3, mode & 4);
  nrf_gpio_pin_write(STP1_DIR, dir);
  nrf_gpio_pin_write(STP2_DIR, dir);
  nrf_delay_ms(1000);
  for (int i = 0; i < steps; ++i) {
    nrf_gpio_pin_write(STP1_STCK, 1);
    nrf_gpio_pin_write(STP2_STCK, 1);
    nrf_delay_us(PULSE_WIDTH_MS);
    nrf_gpio_pin_write(STP1_STCK, 0);
    nrf_gpio_pin_write(STP2_STCK, 0);
    nrf_delay_us(PULSE_WIDTH_MS);
  }
  motor_powerdown();
}

/**@brief Function for updating the lock mech status.
 *
 * @details  Checks and updates the lock mech status.
 */
static void lock_mech_update_status(void) {
  //handle new_chain_attach_event 
  if (lock_mech_state.new_chain_attach_event && is_admin_chain_locked() && ! lock_mech_state.lock_mech_status !=SMART_LOCK_UNLOCKED_OPEN_ADMIN) {
    lock_mech_state.new_chain_attach_event = false;
    nrf_delay_ms(10);
    NRF_LOG_INFO("new_chain_attach_event and is_admin_chain_attached");

    if (is_chain_attached() && lock_mech_state.lock_mech_status >= SMART_LOCK_UNLOCKED_CLOSED ) {
      lock_mech_engage(true);
      NRF_LOG_INFO("is_chain_attached() && lock_mech_state.lock_mech_status >= SMART_LOCK_UNLOCKED_CLOSED");
      smart_lock_send_data(SL_LOCK_SUCCESS);
    }
  }
  
  //handle new_admin_chain_attach_event 
  if (lock_mech_state.new_admin_chain_attach_event) {
    nrf_delay_ms(10);

    if (is_chain_attached() && is_admin_chain_attached() && lock_mech_state.lock_mech_status >= SMART_LOCK_UNLOCKED_CLOSED) {
      lock_mech_state.new_admin_chain_attach_event = false;
      lock_mech_state.new_chain_attach_event = false;
      lock_mech_engage_admin(true);
      NRF_LOG_INFO("is_chain_attached() & is_admin_chain_attached() && lock_mech_state.lock_mech_status >= SMART_LOCK_UNLOCKED_CLOSED");
    }
  }
  //NRF_LOG_INFO("is_chain_attached: %d is_chain_locked: %d is_admin_chain_attached: %d  is_admin_chain_locked: %d",is_chain_attached(), is_chain_locked(),  is_admin_chain_attached(), is_admin_chain_locked() );
  lock_mech_state.is_chain_attached = is_chain_attached();
  lock_mech_state.is_chain_locked = is_chain_locked();
  lock_mech_state.is_admin_chain_attached = is_admin_chain_attached();
  lock_mech_state.is_admin_chain_locked = is_admin_chain_locked();

  if (lock_mech_state.is_chain_attached == true &&
      lock_mech_state.is_chain_locked == true &&
      lock_mech_state.is_admin_chain_attached == true &&
      lock_mech_state.is_admin_chain_locked == true) {
    lock_mech_state.lock_mech_status = SMART_LOCK_LOCKED;
  } else if (lock_mech_state.is_chain_attached == true &&
             lock_mech_state.is_chain_locked == false &&
             lock_mech_state.is_admin_chain_attached == true &&
             lock_mech_state.is_admin_chain_locked == true) {
    lock_mech_state.lock_mech_status = SMART_LOCK_UNLOCKED_CLOSED;
  } else if (lock_mech_state.is_chain_attached == false &&
             lock_mech_state.is_chain_locked == false) {
    lock_mech_state.lock_mech_status = SMART_LOCK_UNLOCKED_OPEN;
  } else {
    lock_mech_state.lock_mech_status = SMART_LOCK_MECH_ERROR;
  }
}

/**@brief Function for getting the lock mech status.
 *
 * @details  checks the lock mech status and returns it.
 */
lock_mech_status_t lock_mech_get_status(void) {
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

/**@brief Function for engaging the lock.
 *
 * @details moves the motor to lock the chain and returns the lock mech status.
 */
lock_mech_status_t lock_mech_engage(bool check_status) {
  lock_mech_status_t lock_mech_status = SMART_LOCK_MECH_ERROR;
  NRF_LOG_INFO("Engage Locking Mechanism");
  motorA_loop(PULSE_WIDTH_MS, NUM_PULSES, CW, HALF_STEP);

  if (check_status) {
    lock_mech_status = lock_mech_get_status();
  } else {
    lock_mech_status = SMART_LOCK_LOCKED;
  }
  led_audio_lock_event();
  NRF_LOG_INFO("lock_mech_status: %d", lock_mech_get_status());
  return lock_mech_status;
}

/**@brief Function for engaging the lock.
 *
 * @details moves the motor to lock the chain and returns the lock mech status.
 */
lock_mech_status_t lock_mech_engage_admin(bool check_status) {
  lock_mech_status_t lock_mech_status = SMART_LOCK_MECH_ERROR;
  NRF_LOG_INFO("Engage Locking Mechanism for both chains");
  motor_loop(PULSE_WIDTH_MS, NUM_PULSES, CW, HALF_STEP);

  if (check_status) {
    lock_mech_status = lock_mech_get_status();
  } else {
    lock_mech_status = SMART_LOCK_LOCKED;
  }
  led_audio_lock_event();
  NRF_LOG_INFO("lock_mech_status: %d", lock_mech_get_status());
  return lock_mech_status;
}

void led_audio_lock_event() {
  for (int i = 0; i < 2; i++) {
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
    nrf_drv_WS2812_show();
    audio_beep_lock_success_event();
    nrf_delay_ms(100);
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
  }
  nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
  nrf_drv_WS2812_show();
  nrf_delay_ms(1000);
  nrf_drv_WS2812_set_pixels_rgb(0, 0, 0);
  nrf_drv_WS2812_show();
  nrf_delay_ms(10);
}

/**@brief Function for disengaging the lock.
 *
 * @details moves the motor to unlock the chain and returns the lock mech status.
 */
lock_mech_status_t lock_mech_disengage(void) {
  NRF_LOG_INFO("Disengage Locking Mechanism");
  motorA_loop(PULSE_WIDTH_MS, NUM_PULSES, CCW, HALF_STEP);
  NRF_LOG_INFO("lock_mech_status: %d", lock_mech_get_status());

  return lock_mech_get_status();
}

/**@brief Function for disengaging the lock.
 *
 * @details moves the motor to unlock the chain and returns the lock mech status.
 */
lock_mech_status_t lock_mech_disengage_admin(void) {
  NRF_LOG_INFO("Disengage Locking Mechanism for both motors");
  motor_loop(PULSE_WIDTH_MS, NUM_PULSES_ADMIN, CCW, HALF_STEP);
  NRF_LOG_INFO("lock_mech_status: %d", lock_mech_get_status());

  return lock_mech_get_status();
}

/**@brief Function for engaging the chain on boot.
 *
 * @details moves the motor to unlock the chain and returns the lock mech status.
 */
lock_mech_status_t loch_mech_on_boot(void) {
  // Disengage both motors for the demo. 
  lock_mech_disengage_admin();
  // Wait until both chains are attached. 
  while (!is_chain_attached() && !is_admin_chain_attached()) {
  }
  lock_mech_state.new_admin_chain_attach_event = false;
  lock_mech_state.new_chain_attach_event = false;

  lock_mech_state.lock_mech_status = SMART_LOCK_LOCKED;
  //nrf_gpio_pin_write(STP1_DIR, 0);
  //nrf_gpio_pin_write(STP1_DIR, 0);

  return lock_mech_engage_admin(true);
}

/**@brief Callback Handler for the two chain detect interupts. 
 *
 * @details creates a new chain attached event if a chain is attached.
 */
void chain_detect_pins_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  NRF_LOG_INFO("Interupted on %d, with action %d, with pin state %d", pin, action, nrf_gpio_pin_read(pin));
  if (pin == CHAIN_DETECT_PIN_A && nrf_gpio_pin_read(pin) == 1) {
    lock_mech_state.new_chain_attach_event = true;
    NRF_LOG_INFO("CHAIN 1 is attached. lock_mech_state.lock_mech_status: %d", lock_mech_state.lock_mech_status);
  }
  if (pin == CHAIN_DETECT_PIN_B && nrf_gpio_pin_read(pin) == 1) {
    lock_mech_state.new_admin_chain_attach_event = true;
    NRF_LOG_INFO("CHAIN 2 is attached. lock_mech_state.lock_mech_status: %d", lock_mech_state.lock_mech_status);
  }
}

/**@brief Function for initializing motors.
 *
 * @details Initializes the motors and the Chain detect interrupts.
 */
lock_mech_status_t lock_mech_init(void) {
  //Stepper1
  nrf_gpio_cfg_output(STP1_STCK);
  nrf_gpio_cfg_output(STP1_MODE1);
  nrf_gpio_cfg_output(STP1_MODE2);
  nrf_gpio_cfg_output(STP1_MODE3);
  nrf_gpio_cfg_output(STP1_DIR);
  nrf_gpio_cfg_input(STP1_STATUS, NRF_GPIO_PIN_PULLDOWN);
  //Stepper2
  nrf_gpio_cfg_output(STP2_STCK);
  nrf_gpio_cfg_output(STP2_MODE1);
  nrf_gpio_cfg_output(STP2_MODE2);
  nrf_gpio_cfg_output(STP2_MODE3);
  nrf_gpio_cfg_output(STP2_DIR);
  nrf_gpio_cfg_input(STP2_STATUS, NRF_GPIO_PIN_PULLDOWN);
  //Stand by the motors
  nrf_gpio_cfg_output(STP1_STANDBY);
  nrf_gpio_pin_write(STP1_STANDBY, 0);
  nrf_gpio_cfg_output(STP2_STANDBY);
  nrf_gpio_pin_write(STP2_STANDBY, 0);
  nrf_gpio_pin_write(STP1_STCK, 0);
  nrf_gpio_pin_write(STP2_STCK, 0);

  nrf_gpio_cfg_output(POWER_EN_3V3);
  nrf_gpio_cfg_output(POWER_EN_12V);

  nrf_gpio_cfg_input(CHAIN_DETECT_PIN_A, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(CHAIN_DETECT_PIN_B, NRF_GPIO_PIN_PULLDOWN);
  // Set power enable pins to be high all the time for now.
  nrf_gpio_pin_set(POWER_EN_3V3);
  nrf_gpio_pin_set(POWER_EN_12V);

  // Enable the two chain detect interupts
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  in_config.sense = GPIO_PIN_CNF_SENSE_Low;

  err_code = nrf_drv_gpiote_in_init(CHAIN_DETECT_PIN_A, &in_config, chain_detect_pins_handler);
  err_code = nrf_drv_gpiote_in_init(CHAIN_DETECT_PIN_B, &in_config, chain_detect_pins_handler);

  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(CHAIN_DETECT_PIN_A, true);
  nrf_drv_gpiote_in_event_enable(CHAIN_DETECT_PIN_B, true);

  lock_mech_state.new_chain_attach_event = false;
  lock_mech_state.is_chain_attached = true;
  lock_mech_state.is_chain_locked = true;
  lock_mech_state.lock_mech_status = SMART_LOCK_LOCKED;

  //NRF_LOG_INFO("CHAIN 2 is attached. lock_mech_state.lock_mech_status: %d", lock_mech_state.lock_mech_status);
  

  nrf_delay_ms(10);


  return lock_mech_get_status();
}
//EOF