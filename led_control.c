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

/**  @brief     LED Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage LED events.
 */

#include "led_control.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "nrf_drv_WS2812.h"
#include "hardware.h"


void led_init(void)
{
  nrf_gpio_cfg_output(POWER_EN_3V3);
  nrf_gpio_pin_set(POWER_EN_3V3);
  nrf_delay_ms(10);
  nrf_drv_WS2812_init(LED_DATA_PIN);
  nrf_delay_ms(10);
}

void led_deinit(void)
{
  nrf_drv_WS2812_clear();
  nrf_drv_WS2812_show();
  nrf_gpio_pin_clear(POWER_EN_3V3);
  nrf_delay_ms(10);
}


void led_pulse(void)
{
    for (int i = 0; i < 2; i++)
    {
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
    nrf_drv_WS2812_set_pixels_rgb(0, 100, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
    nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(100);
    }
}


void led_unlock_event(void)
{
    nrf_drv_WS2812_set_pixels_rgb(0, 100, 0);
    nrf_drv_WS2812_show();
}

void led_pairing_event(void)
{
    nrf_drv_WS2812_set_pixels_rgb(0, 0, 100);
    nrf_drv_WS2812_show();
    nrf_delay_ms(10);
}

void led_admin_event(void)
{
    nrf_drv_WS2812_set_pixels_rgb(100, 50, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(50);
}

void led_alarm_event(void)
{
    nrf_drv_WS2812_set_pixels_rgb(100, 0, 0);
    nrf_drv_WS2812_show();
    nrf_delay_ms(50);
}



//EOF