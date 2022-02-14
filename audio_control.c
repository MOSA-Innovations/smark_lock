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

/**  @brief     Audio Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage Audio events.
 */

#include "audio_control.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "hardware.h"


#define LOCK_SUCCESS_BEEP_LENGTH_MS     (30)
#define LOCK_FAIL_BEEP_LENGTH_MS        (3000)
#define UNLOCK_SUCCESS_BEEP_LENGTH_MS   (30)
#define UNLOCK_FAIL_BEEP_LENGTH_MS      (1000)

 // nrf_gpio_cfg_output(BUZZER);
 // nrf_gpio_pin_clear(BUZZER);

typedef struct 
{
    bool beep_on;
} smart_lock_audio_t;

static smart_lock_audio_t smart_lock_audio;

void audio_beeper_control(bool enable)
{
    if (smart_lock_audio.beep_on == true && enable == true)
        return;

    if (smart_lock_audio.beep_on == false && enable == false)
        return;

    if (enable)
        smart_lock_audio.beep_on = true;
    else
        smart_lock_audio.beep_on = false;

    nrf_gpio_pin_toggle(BUZZER_PIN);
}

void audio_init(void)
{
    nrf_gpio_cfg_output(BUZZER_PIN);
    nrf_gpio_pin_clear(BUZZER_PIN);

    smart_lock_audio.beep_on = false;

    audio_beeper_control(false);
}

void audio_beep_lock_success_event(void)
{
    audio_beeper_control(true);
    nrf_delay_ms(LOCK_SUCCESS_BEEP_LENGTH_MS);
    audio_beeper_control(false);
}

void audio_beep_unlock_success_event(void)
{
    audio_beeper_control(true);
    nrf_delay_ms(UNLOCK_SUCCESS_BEEP_LENGTH_MS);
    audio_beeper_control(false);
}

void audio_beep_lock_fail_event(void)
{
    audio_beeper_control(true);
    nrf_delay_ms(LOCK_FAIL_BEEP_LENGTH_MS);
    audio_beeper_control(false);
}

void audio_beep_unlock_fail_event(void)
{
    audio_beeper_control(true);
    nrf_delay_ms(UNLOCK_FAIL_BEEP_LENGTH_MS);
    audio_beeper_control(false);
}



//EOF