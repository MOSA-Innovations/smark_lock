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

/**  @brief     nrf_drv_WS2812
 *   @details   This module is a low level driver to control the WS2812 LEDs using  
 *              the PWM peripheral.   
 */

#ifndef NRF_DRV_WS2812_H__
#define NRF_DRV_WS2812_H__

#include <stdint.h>

#define NR_OF_PIXELS 12

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} nrf_drv_WS2812_pixel_t;

void nrf_drv_WS2812_init(uint8_t pin);
void nrf_drv_WS2812_set_pixel_rgb(uint8_t pixel_nr, uint8_t red, uint8_t green, uint8_t blue);
void nrf_drv_WS2812_set_pixel(uint8_t pixel_nr, nrf_drv_WS2812_pixel_t *color);
void nrf_drv_WS2812_set_pixels(uint8_t pixel_nr, nrf_drv_WS2812_pixel_t *color);
void nrf_drv_WS2812_show(void);
void nrf_drv_WS2812_clear(void);

#endif //NRF_DRV_WS2812
