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

#ifndef LED_CONTROL_H__
#define LED_CONTROL_H__


/**@brief Function for initializing device leds.
 */
void led_init(void);

/**@brief Function for deinitializing device leds.
 */
void led_deinit(void);

/**@brief Function to pulse LEDs.
 */
void led_pulse(void);

/**@brief LED event on unlock
 */
void led_unlock_event(void);

/**@brief LED event on admin mode
 */
void led_admin_event(void);

/**@brief LED event on pairing
 */
void led_pairing_event(void);

#endif // LED_CONTROL_H__