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

/**  @brief     Security Control.
 *   @details   This module contains most of the functions used
 *              by the application to set up and manage security events.
 */

#ifndef SECURITY_H__
#define SECURITY_H__

#include <stdint.h>
#include <stdbool.h>

#define SL_DEV_ID_LEN                   16
#define SL_CODE_NUM                     10
#define SL_CODE_TIME_LEN                10 //yymmddhhmm

typedef enum{
    SECURITY_CHECK_PASSED,
    SECURITY_CHECK_FAILED_OUT_OF_CODES,
    SECURITY_CHECK_FAILED_INVALID_CODE,
    SECURITY_CHECK_FAILED_CODE_OUT_OF_DATE,
    SECURITY_CHECK_FAILED_INVALID_DEV_ID
} security_check_status_t;

/**@brief Function to update new security codes.
 */
void security_update_codes(uint8_t *buf, int code_len);

/**@brief Function to perform a security check.
 */
security_check_status_t security_check(uint8_t *buf, int len);

/**@brief Function for initializing security.
 */
void security_init(void);

#endif // SECURITY_H__