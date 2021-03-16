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

#define LED_PIN     LED_1

void led_init(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_set(LED_PIN);
}

void led_pulse(void)
{
    for (int i = 0; i < 2; i++)
    {
        nrf_gpio_pin_clear(LED_PIN);
        nrf_delay_ms(100);
        nrf_gpio_pin_set(LED_PIN);
        nrf_delay_ms(100);
        nrf_gpio_pin_clear(LED_PIN);
    }
}
//EOF