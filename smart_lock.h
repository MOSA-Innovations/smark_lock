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

/**  @brief     Smart Lock.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage the smart lock.
 */

#ifndef SMART_LOCK_H__
#define SMART_LOCK_H__

//SmartLock protocol
#define SL_UNLOCK_CMD                   0x0
#define SL_LOCK_CMD                     0x1
#define SL_UPDATE_CODE_CMD              0x2
#define SL_APP_READY_CMD                0x3
#define SL_RESET_CMD                    0x4
#define SL_ADMIN_UNLOCK_CMD             0x5
#define SL_LOCK_MECH_STATE_CMD          0x6

#define SL_LOCK_SUCCESS                 0x10
#define SL_UNLOCK_SUCCESS               0x11
#define SL_UPDATE_SUCCESS               0x12
#define SL_RESET_SUCCESS                0x13
#define SL_DEV_NEED_UPDATE              0x14
#define SL_CODE_RUN_OUT                 0x15
#define SL_UNLOCK_FAIL                  0x16
#define SL_LOCK_FAIL                    0x17
#define SL_CODE_OUT_OF_DATE             0x18
#define SL_CODE_INVALID                 0x19
#define SL_DEV_ID_FAIL                  0x1A
#define SL_LOCK                         0xD0
#define SL_UNLOCK                       0xD1
#define SL_UNUSABLE                     0xD2

/**@brief Function for initializing the smart lock.
 */
void smart_lock_init(void);

/**@brief Main loop of the smart lock.
 */
void smart_lock_run(void);

/**@brief Function for sending data over the GATT service
 */
void smart_lock_send_data(uint8_t data);

void smart_lock_pulse_led(void);

void smart_lock_parse_data(uint8_t *buf, int len);

void smart_lock_detect(void);

#endif // SMART_LOCK_CORE_H__

