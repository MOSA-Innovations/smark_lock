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

/**  @brief     Hardware test2
 *   @details   This module was created based on the twi scan example
 *              and was developed according to Govelo Bike-Hardware Verification  
 *              code first_Second stage_v2.pdf.
 */

#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_pwm.h"
#include <stdio.h>
#include <string.h>

#include "nrf_drv_saadc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_WS2812.h"

#define FULL_STEP 0
#define HALF_STEP 1
#define QUAD_STEP 2
#define OCTA_STEP 3
#define SIXT_STEP 4
#define CW 0
#define CCW 1

// Pin Definitions
#define STP1_STATUS 31
#define STP1_MODE1 25
#define STP1_MODE2 28
#define STP1_MODE3 29
#define STP1_STCK 24
#define STP1_DIR 23

#define STP_STANDBY 5

#define STP2_STATUS 11
#define STP2_MODE1 16
#define STP2_MODE2 15
#define STP2_MODE3 12
#define STP2_STCK 8
#define STP2_DIR 22

#define NUM_PULSES 200
#define PULSE_WIDTH_MS 2000


void motor_init(void) {
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
  nrf_gpio_cfg_output(STP_STANDBY);
  nrf_gpio_pin_write(STP_STANDBY, 0);
  nrf_gpio_pin_write(STP1_STCK, 0);
  nrf_gpio_pin_write(STP2_STCK, 0);
}

void motor_loop(uint32_t pulse_width, uint16_t steps, bool dir, uint8_t mode) {
  // Wake up the motor drivers
  nrf_gpio_pin_write(STP_STANDBY, 1);
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
}

void motor_powerdown() {
  nrf_gpio_pin_write(STP_STANDBY, 0);
  nrf_gpio_pin_write(STP1_STCK, 0);
  nrf_gpio_pin_write(STP2_STCK, 0);
}

void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
}

int main(void) {

  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  nrf_drv_WS2812_init(17);

  NRF_LOG_INFO("Testing custom PCB stage 2");
  NRF_LOG_FLUSH();

  NRF_LOG_INFO("WS2812 Display blue");
  NRF_LOG_FLUSH();
  nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
  nrf_drv_WS2812_show();
  nrf_delay_ms(1000);
  NRF_LOG_INFO("WS2812 Display green");
  NRF_LOG_FLUSH();
  nrf_drv_WS2812_set_pixels_rgb(0, 100, 0);
  nrf_drv_WS2812_show();
  nrf_delay_ms(1000);
  NRF_LOG_INFO("WS2812 Display red");
  NRF_LOG_FLUSH();
  nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
  nrf_drv_WS2812_show();
  nrf_delay_ms(1000);

  motor_init();
  NRF_LOG_INFO("Set the STSPIN820 Step ClockWise");
  NRF_LOG_FLUSH();

  NRF_LOG_FLUSH();
  motor_loop(PULSE_WIDTH_MS, NUM_PULSES, CW, HALF_STEP);

  nrf_delay_ms(3000);

  NRF_LOG_INFO("Moving counterclockwise.");
  NRF_LOG_FLUSH();
  motor_loop(PULSE_WIDTH_MS, NUM_PULSES, CCW, HALF_STEP);

  NRF_LOG_INFO("Release stepper motor.");
  NRF_LOG_FLUSH();
  motor_powerdown();

  nrf_delay_ms(5000);

  uint32_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  nrf_saadc_channel_config_t channel_config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
  nrf_saadc_channel_config_t channel_config2 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

  while (1) {
    NRF_LOG_INFO("Set the ADC for AIN0 m AIN1 and AIN2 ");
    NRF_LOG_FLUSH();
    //Reading saadc channel 0

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
    APP_ERROR_CHECK(err_code);

    static nrf_saadc_value_t sample;
    nrf_drv_saadc_sample_convert(0, &sample);

    NRF_LOG_INFO("AIN0: %d", sample);
    NRF_LOG_FLUSH();
    err_code = nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();

    nrf_delay_ms(1000);
    //Reading saadc channel 1

    //channel_config =NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);

    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_sample_convert(1, &sample);

    NRF_LOG_INFO("AIN1: %d", sample);
    NRF_LOG_FLUSH();
    nrf_drv_saadc_channel_uninit(1);
    nrf_drv_saadc_uninit();
    nrf_delay_ms(1000);
    //Reading saadc channel 2

    //channel_config =NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_config2);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_sample_convert(2, &sample);

    NRF_LOG_INFO("AIN2: %d", sample);
    NRF_LOG_FLUSH();
    nrf_drv_saadc_channel_uninit(2);
    nrf_drv_saadc_uninit();

    nrf_delay_ms(1000);
  }
}

/** @} */