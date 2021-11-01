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

/**  @brief     Hardware test3
 *   @details   This module was created based on the twi scan example
 *              and was developed according to Govelo Bike-Hardware Verification  
 *              code first_Second_Third stage.pdf.
 */

#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include <stdio.h>
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_saadc.h"

#include "app_mpu.h"
#include "nrf_drv_WS2812.h"

//Stepper motor definitions
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

#define NUM_PULSES 1500
#define PULSE_WIDTH_MS 2000

#define POWER_3V3 20
#define POWER_12V 19
#define PIN_IN 30
#define BUZZER 18
#define MAGNETIC1 13
#define MAGNETIC2 14

void gpio_init(void) {
  nrf_gpio_cfg_output(POWER_3V3);
  nrf_gpio_cfg_output(POWER_12V);
  nrf_gpio_cfg_output(BUZZER);
  nrf_gpio_cfg_input(MAGNETIC1, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(MAGNETIC2, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_pin_clear(POWER_3V3);
  nrf_gpio_pin_clear(POWER_12V);
  nrf_gpio_pin_clear(BUZZER);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  NRF_LOG_INFO("Had an interupt on IO30");
  nrf_drv_gpiote_in_event_disable(PIN_IN);
  NRF_LOG_INFO("Disabled the interupt events on IO30");
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_mpu_interrupt_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;
  in_config.sense = GPIO_PIN_CNF_SENSE_Low;

  err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

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

void motor1_loop(uint32_t pulse_width, uint16_t steps, bool dir, uint8_t mode) {
  // Wake up the motor drivers
  nrf_gpio_pin_write(STP_STANDBY, 1);
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
}

void motor2_loop(uint32_t pulse_width, uint16_t steps, bool dir, uint8_t mode) {
  // Wake up the motor drivers
  nrf_gpio_pin_write(STP_STANDBY, 1);
  // Set the mode and the direction
  nrf_gpio_pin_write(STP2_MODE1, mode & 1);
  nrf_gpio_pin_write(STP2_MODE2, mode & 2);
  nrf_gpio_pin_write(STP2_MODE3, mode & 4);
  nrf_gpio_pin_write(STP2_DIR, dir);
  nrf_delay_ms(1000);
  for (int i = 0; i < steps; ++i) {
    nrf_gpio_pin_write(STP2_STCK, 1);
    nrf_delay_us(PULSE_WIDTH_MS);
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

void mpu_init(void) {
  ret_code_t ret_code;
  // Initiate MPU driver
  ret_code = app_mpu_init();
  APP_ERROR_CHECK(ret_code); // Check for errors in return value

  // Setup and configure the MPU with intial values
  app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
  p_mpu_config.smplrt_div = 19;                         // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
  p_mpu_config.accel_config.afs_sel = AFS_2G;           // Set accelerometer full scale range to 2G
  ret_code = app_mpu_config(&p_mpu_config);             // Configure the MPU with above values
  APP_ERROR_CHECK(ret_code);                            // Check for errors in return value
}

#define PWR_CTRL_1 19
#define PWR_CTRL_2 20

int main(void) {

  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  NRF_LOG_INFO("Testing custom PCB stage 3");
  NRF_LOG_FLUSH();

  gpio_init();
  

  gpio_mpu_interrupt_init();

  mpu_init();
  uint8_t chip_id;
  app_mpu_who_am_i(&chip_id);
  NRF_LOG_INFO("MPU6050 CHIP ID is 0x%x", chip_id);
  NRF_LOG_FLUSH();
  NRF_LOG_INFO("Enabled MPU6050 interrupt mode");
  NRF_LOG_FLUSH();
  app_mpu_int_enable_t mpu_int_config;
  mpu_int_config.data_rdy_en = 1;
  app_mpu_int_enable(&mpu_int_config);
  uint8_t int_source;
  nrf_delay_ms(1000);
  app_mpu_read_int_source(&int_source);
  NRF_LOG_INFO("Had an interupt Interupt source:  0x%x", int_source);
  NRF_LOG_FLUSH();

  // Start execution.
  nrf_gpio_pin_set(POWER_12V);

  nrf_gpio_pin_set(POWER_3V3);
  nrf_delay_ms(10);

  nrf_drv_WS2812_init(17);
  NRF_LOG_INFO("Testing WS2812 LED");
  NRF_LOG_FLUSH();

  for (int i = 0; i < 3; i++) {
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
    nrf_drv_WS2812_show();
    nrf_delay_ms(1000);
    nrf_drv_WS2812_set_pixels_rgb(0, 100, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(1000);
    nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(1000);
  }

  nrf_gpio_pin_clear(POWER_3V3);
  nrf_gpio_pin_set(POWER_12V);
  nrf_delay_ms(10);

  NRF_LOG_INFO("Testing the BUZZER");
  NRF_LOG_FLUSH();
  for (int i = 0; i < 2; i++) {
    nrf_gpio_pin_set(BUZZER);
    nrf_delay_ms(50);
    nrf_gpio_pin_clear(BUZZER);
    nrf_delay_ms(1000);
  }
  NRF_LOG_INFO("Testing Magnetic and MOTO Magnetic_1 for MOTO1, Magnetic_2 for MOTO2");
  NRF_LOG_FLUSH();
  nrf_delay_ms(1000);
  NRF_LOG_INFO("Keep magnetic_1 induction close to iron");
  NRF_LOG_FLUSH();
  while (!nrf_gpio_pin_read(MAGNETIC1)) {
    nrf_delay_ms(100);
  }
  motor_init();
  NRF_LOG_INFO("Magnetic_1 is %d moving MOTO 1", nrf_gpio_pin_read(MAGNETIC1));
  NRF_LOG_FLUSH();
  motor1_loop(PULSE_WIDTH_MS, NUM_PULSES, CW, HALF_STEP);

  while (!nrf_gpio_pin_read(MAGNETIC2)) {
    nrf_delay_ms(100);
  }
  nrf_delay_ms(1000);
  NRF_LOG_INFO("Magnetic_2 is %d moving MOTO 2", nrf_gpio_pin_read(MAGNETIC1));
  NRF_LOG_FLUSH();
  motor2_loop(PULSE_WIDTH_MS, NUM_PULSES, CW, HALF_STEP);

  NRF_LOG_INFO("Release stepper motor and turn off 12V");
  NRF_LOG_FLUSH();
  motor_powerdown();
  //nrf_gpio_pin_clear(POWER_12V);

  nrf_delay_ms(3000);

  uint32_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  nrf_saadc_channel_config_t channel_config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
  nrf_saadc_channel_config_t channel_config2 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

  NRF_LOG_INFO("Set the ADC for AIN0, AIN1 and AIN2");
  NRF_LOG_FLUSH();

  static nrf_saadc_value_t sample1;
  static nrf_saadc_value_t sample2;
  static nrf_saadc_value_t sample3;

  while (1) {

    //Reading saadc channel 0
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_sample_convert(0, &sample1);

    err_code = nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();

    nrf_delay_ms(1000);
    //Reading saadc channel 1

    //channel_config =NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);

    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_sample_convert(1, &sample2);

    nrf_drv_saadc_channel_uninit(1);
    nrf_drv_saadc_uninit();
    nrf_delay_ms(1000);
    //Reading saadc channel 2

    //channel_config =NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_config2);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_sample_convert(2, &sample3);

    NRF_LOG_INFO("AIN0: %d, AIN1: %d, AIN2: %d", sample1, sample2, sample3);
    NRF_LOG_FLUSH();
    nrf_drv_saadc_channel_uninit(2);
    nrf_drv_saadc_uninit();

    nrf_delay_ms(1000);
  }
}

/** @} */