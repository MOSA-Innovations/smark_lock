/*
 * The library is not extensively tested and only
 * meant as a simple explanation and for inspiration.
 * NO WARRANTY of ANY KIND is provided.
 */

#include "mpu_control.h"
#include "nrf_delay.h"
#include "nrf_drv_mpu.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_peripherals.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "audio_control.h"
#include "led_control.h"

#define MPU60x0
#define MPU_USES_TWI

uint32_t app_mpu_config(app_mpu_config_t *config) {
  uint8_t *data;
  data = (uint8_t *)config;
  return nrf_drv_mpu_write_registers(MPU_REG_SMPLRT_DIV, data, 4);
}

uint32_t app_mpu_int_cfg_pin(app_mpu_int_pin_cfg_t *cfg) {
  uint8_t *data;
  data = (uint8_t *)cfg;
  return nrf_drv_mpu_write_single_register(MPU_REG_INT_PIN_CFG, *data);
}

uint32_t app_mpu_int_enable(app_mpu_int_enable_t *cfg) {
  uint8_t *data;
  data = (uint8_t *)cfg;
  return nrf_drv_mpu_write_single_register(MPU_REG_INT_ENABLE, *data);
}

uint8_t app_mpu_who_am_i(uint8_t *device_id) {
  return nrf_drv_mpu_read_registers(MPU_REG_WHO_AM_I, device_id, 1);
}

uint32_t app_mpu_init(void) {
  uint32_t err_code;

  // Initate TWI or SPI driver dependent on what is defined from the project
  err_code = nrf_drv_mpu_init();
  if (err_code != NRF_SUCCESS)
    return err_code;

  uint8_t reset_value = 7; // Resets gyro, accelerometer and temperature sensor signal paths.
  err_code = nrf_drv_mpu_write_single_register(MPU_REG_SIGNAL_PATH_RESET, reset_value);
  if (err_code != NRF_SUCCESS)
    return err_code;

  // Chose  PLL with X axis gyroscope reference as clock source
  err_code = nrf_drv_mpu_write_single_register(MPU_REG_PWR_MGMT_1, 1);
  if (err_code != NRF_SUCCESS)
    return err_code;

  return NRF_SUCCESS;
}

uint32_t app_mpu_read_accel(accel_values_t *accel_values) {
  uint32_t err_code;
  uint8_t raw_values[6];
  err_code = nrf_drv_mpu_read_registers(MPU_REG_ACCEL_XOUT_H, raw_values, 6);
  if (err_code != NRF_SUCCESS)
    return err_code;

  // Reorganize read sensor values and put them into value struct
  uint8_t *data;
  data = (uint8_t *)accel_values;
  for (uint8_t i = 0; i < 6; i++) {
    *data = raw_values[5 - i];
    data++;
  }
  return NRF_SUCCESS;
}

uint32_t app_mpu_read_gyro(gyro_values_t *gyro_values) {
  uint32_t err_code;
  uint8_t raw_values[6];
  err_code = nrf_drv_mpu_read_registers(MPU_REG_GYRO_XOUT_H, raw_values, 6);
  if (err_code != NRF_SUCCESS)
    return err_code;

  // Reorganize read sensor values and put them into value struct
  uint8_t *data;
  data = (uint8_t *)gyro_values;
  for (uint8_t i = 0; i < 6; i++) {
    *data = raw_values[5 - i];
    data++;
  }
  return NRF_SUCCESS;
}

uint32_t app_mpu_read_temp(temp_value_t *temperature) {
  uint32_t err_code;
  uint8_t raw_values[2];
  err_code = nrf_drv_mpu_read_registers(MPU_REG_TEMP_OUT_H, raw_values, 2);
  if (err_code != NRF_SUCCESS)
    return err_code;

  *temperature = (temp_value_t)(raw_values[0] << 8) + raw_values[1];

  return NRF_SUCCESS;
}

uint32_t app_mpu_read_int_source(uint8_t *int_source) {
  return nrf_drv_mpu_read_registers(MPU_REG_INT_STATUS, int_source, 1);
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

void led_audio_alarm_event_short() {
  for (int i = 0; i < 2; i++) {
    nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
    nrf_drv_WS2812_show();
    audio_beep_lock_success_event();
    nrf_delay_ms(100);
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
  }
}

void led_audio_alarm_event_long() {
  for (int i = 0; i < 2; i++) {
    nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
    nrf_drv_WS2812_show();
    // audio_beep_lock_success_event();
    audio_beep_unlock_fail_event();
    nrf_delay_ms(100);
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
  }
}

void mpu_check(void) {
  float Pi = 3.14159;
  int roll_threshold = 4;
  static int heading_old = 0;
  static uint16_t counter = 0;
  static int8_t alarm_flag = 0;
  accel_values_t accel_values;
  ret_code_t err_code;

  err_code = app_mpu_read_accel(&accel_values);

  APP_ERROR_CHECK(err_code);

  // For a simple anti tamper detection we calculate roll.
  float roll = 180 * atan(accel_values.x / sqrt(accel_values.y * accel_values.y + accel_values.z * accel_values.z)) / Pi;

  // NRF_LOG_INFO("accel_values.y: %06d  accel_values.x: %06d accel_values.z:%d roll:%04d  counter:%04d flag:%01d",accel_values.y, accel_values.x,accel_values.z, roll, counter, alarm_flag);

  if (roll >= roll_threshold && alarm_flag == 0) {
    NRF_LOG_INFO("Alarm event with diff value: %04d", roll);
    led_audio_alarm_event_short();
    alarm_flag = 1;
    counter = 0;
  } else if (alarm_flag == 1 && roll >= roll_threshold && counter > 20 && counter < 50) {
    NRF_LOG_INFO("Second Alarm event in a row with diff value: %04d and counter: %04d ", roll, counter);
    led_audio_alarm_event_long();
    alarm_flag == 0;
  } else if (alarm_flag == 1 && counter == 50) {
    alarm_flag = 0;
    counter = 0;
  }
  if (alarm_flag == 1 && counter < 50) {
    counter++;
  }

  nrf_delay_ms(50);
}

// EOF