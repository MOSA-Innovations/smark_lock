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

/**  @brief     GPIO Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage GPIO events.
 */

#ifndef GPIO_CONTROL_H__
#define GPIO_CONTROL_H__

#include "bsp.h"

/**@brief Function for initializing device gpios.
 */
void gpio_init(void);

/**@brief Function for toggling a GPIO.
 */
void gpio_pin_toggle(uint32_t pin_number);

#endif // GPIO_CONTROL_H__