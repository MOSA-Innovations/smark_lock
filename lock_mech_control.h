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

/**  @brief     Lock Mech Control Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage Lock Mech events.
 */

#ifndef LOCK_MECH_CONTROL_H__
#define LOCK_MECH_CONTROL_H__

#include <stdbool.h>

typedef enum{
    SMART_LOCK_LOCKED,
    SMART_LOCK_UNLOCKED_CLOSED,
    SMART_LOCK_UNLOCKED_OPEN,
    SMART_LOCK_UNLOCKED_OPEN_ADMIN,
    SMART_LOCK_MECH_ERROR
} lock_mech_status_t;

/**@brief Function for initializing the locking mechanism.
 */
lock_mech_status_t lock_mech_init(void);

/**@brief Function to engage locking mechanism.
 */
lock_mech_status_t lock_mech_engage(bool check_status);

/**@brief Function to engage locking mechanism.
 */
lock_mech_status_t lock_mech_engage_admin(bool check_status);

/**@brief Function to disengage locking mechanism for both motors.
 */
lock_mech_status_t lock_mech_disengage_admin(void);

/**@brief Function to disengage locking mechanism.
 */
lock_mech_status_t lock_mech_disengage(void);

/**@brief Function check locking mechanism status.
 */
lock_mech_status_t lock_mech_get_status(void);

lock_mech_status_t loch_mech_on_boot(void);

void led_audio_lock_event(void);

#endif // LOCK_MECH_CONTROL_H__