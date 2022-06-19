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
 *              by the application to set up and manage audio events.
 */

#ifndef AUDIO_CONTROL_H__
#define AUDIO_CONTROL_H__

/**@brief Function for initializing audio events.
 */
void audio_init(void);


/**@brief Function to trigger a beep on a successfull lock event.
 */
void audio_beep_lock_success_event(void);

/**@brief Function to trigger a beep on a successfull unlock event.
 */
void audio_beep_unlock_success_event(void);

/**@brief Function to trigger a beep on a failed lock event.
 */
void audio_beep_lock_fail_event(void);

/**@brief Function to trigger a beep on a failed unlock event.
 */
void audio_beep_unlock_fail_event(void);



#endif // AUDIO_CONTROL_H__