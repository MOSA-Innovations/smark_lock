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

#include "security.h"
#include "nrf.h"
#include "nrf_log.h"

typedef struct 
{
    int years;
    int months;
    int days;
    int hours;
    int minutes;
} security_code_date_t;

typedef struct
{
	int code_valid;
	uint8_t access_code[SL_CODE_NUM];
	uint8_t code_date[SL_CODE_TIME_LEN];
	uint8_t dev_id[SL_DEV_ID_LEN];
	bool update;
	bool dev_lock;
} security_state_t;

security_state_t security_state;
security_code_date_t security_code_date;
uint8_t tmp_dev_id[SL_DEV_ID_LEN] = {0};

void security_clear_codes(void)
{
    security_state.code_valid = 0;
    //security_state.dev_lock = false;

    for (int i = 0; i < SL_CODE_NUM; i++)
    {
        security_state.access_code[i] = 0xFF;
    }
}

void security_update_codes(uint8_t *buf, int code_len)
{
    security_clear_codes();

    memcpy(security_state.access_code, buf + 2, code_len);

    memcpy(security_state.dev_id, buf + 5, SL_DEV_ID_LEN);

    memcpy(security_state.code_date, buf + 21, SL_CODE_TIME_LEN);

    security_code_date.years = security_state.code_date[0] * 10 + security_state.code_date[1];
    security_code_date.months = security_state.code_date[2] * 10 + security_state.code_date[3];
    security_code_date.days = security_state.code_date[4] * 10 + security_state.code_date[5];
    security_code_date.hours = security_state.code_date[6] * 10 + security_state.code_date[7];
    security_code_date.minutes = security_state.code_date[8] * 10 + security_state.code_date[9];

    security_state.code_valid = code_len;
    security_state.update = true;
}

int security_compare_date(uint8_t *buf)
{
    int years = 0, months = 0, days = 0, hours = 0, minutes = 0;
    uint8_t tmp[SL_CODE_TIME_LEN] = {0};

    memcpy(tmp, buf + 19, SL_CODE_TIME_LEN);

    years = tmp[0] * 10 + tmp[1];
    months = tmp[2] * 10 + tmp[3];
    days = tmp[4] * 10 + tmp[5];
    hours = tmp[6] * 10 + tmp[7];
    minutes = tmp[8] * 10 + tmp[9];

    NRF_LOG_INFO("code_year = %d, code_months = %d, code_days = %d, code_hours = %d, code_minutes = %d",
                                                    security_code_date.years, security_code_date.months, security_code_date.days, security_code_date.hours, security_code_date.minutes);
    NRF_LOG_INFO("tmp_year = %d, tmp_months = %d, tmp_days = %d, tmp_hours = %d, tmp_minutes = %d",
                                                    years, months, days, hours, minutes);
    /*To DO add a time limit.
    if (years > security_code_date.years || months > security_code_date.months)
    {
        return -1;
    }

    if (days > security_code_date.days)
    {
        return -1;
    }

    if (hours > security_code_date.hours)
    {
        return -1;
    }

    if (minutes > security_code_date.minutes + 1)
    {
        return -1;
    }
    */

    return 0;
}

int security_compare_dev_id(uint8_t *buf)
{
    int i;

    if (security_state.dev_lock)
    {
        for (i = 0; i < SL_DEV_ID_LEN; i++)
        {
            NRF_LOG_INFO("[%d]: tmp dev id = %x, buf = %x", i, tmp_dev_id[i], buf[3 + i]);
            if (tmp_dev_id[i] != buf[3 + i])
                    break;
        }

        if (i != SL_DEV_ID_LEN)
            return -1;
    }
    else
    {
        for (i = 0; i < SL_DEV_ID_LEN; i++)
        {
            tmp_dev_id[i] = buf[3 + i];
            NRF_LOG_INFO("store tmp_dev_id = %x", tmp_dev_id[i]);
        }
    }
    return 0;
}

security_check_status_t security_check(uint8_t *buf, int len)
{
    uint8_t user_code = buf[2];
    NRF_LOG_INFO("User access code = %x", user_code);
    NRF_LOG_INFO("valid code left = %d", security_state.code_valid);

    if (security_state.code_valid <= 1)
    {
        return SECURITY_CHECK_FAILED_OUT_OF_CODES;
    }

    if (security_compare_dev_id(buf) != 0)
    {
        return SECURITY_CHECK_FAILED_INVALID_DEV_ID;
    }

    if (security_compare_date(buf) != 0)
    {
        return SECURITY_CHECK_FAILED_CODE_OUT_OF_DATE;
    }

    for (int i = 0; i < SL_CODE_NUM; i++)
    {
        if (user_code == security_state.access_code[i])
        {
            security_state.code_valid--;
            security_state.access_code[i] = 0xFF;
            security_state.dev_lock = true;
            
            return SECURITY_CHECK_PASSED;
        }
    }

    return SECURITY_CHECK_FAILED_INVALID_CODE;
}

void security_init(void)
{
    memset(&security_state, 0x0, sizeof(security_state));
    memset(&security_code_date, 0x0, sizeof(security_code_date));

    security_state.code_valid = 0;
    security_state.update = false;
    security_state.dev_lock = false;

    for (int i = 0; i < SL_CODE_NUM; i++)
    {
        security_state.access_code[i] = 0xFF;
    }
}
//EOF