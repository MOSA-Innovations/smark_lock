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

/**  @brief     Hardware test1
 *   @details   This module was created based on the twi scan example
 *              and wsa developed according to Govelo Bike-Hardware Verification code first stage.docx.
 */


#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define PWR_CTRL_1         19
#define PWR_CTRL_2         20

void gpio_init(void)
{
    nrf_gpio_cfg_output(PWR_CTRL_1);
    nrf_gpio_cfg_output(PWR_CTRL_2);
    nrf_gpio_pin_clear(PWR_CTRL_1);
    nrf_gpio_pin_clear(PWR_CTRL_2);
}

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    gpio_init();
    
    ret_code_t err_code;
    
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Testing custom PCB.");
    NRF_LOG_FLUSH();
    twi_init();

    while (true)
    {
   
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    NRF_LOG_INFO("Scanning for TWI devices.");

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }
    
    NRF_LOG_INFO("Set the POWER Control PIN(IO19,IO20) High and Low.");
    NRF_LOG_FLUSH();
    nrf_gpio_pin_set(PWR_CTRL_1);
    nrf_gpio_pin_clear(PWR_CTRL_2);
    nrf_delay_ms(5000);
    NRF_LOG_INFO("Set the POWER Control PIN(IO19,IO20) Low and High.");
    NRF_LOG_FLUSH();
    nrf_gpio_pin_clear(PWR_CTRL_1);
    nrf_gpio_pin_set(PWR_CTRL_2);
    nrf_delay_ms(5000);
    NRF_LOG_INFO("Set the POWER Control PIN(IO19,IO20) High and High.\n");
    NRF_LOG_FLUSH();
    nrf_gpio_pin_set(PWR_CTRL_1);
    nrf_gpio_pin_set(PWR_CTRL_2);
    nrf_delay_ms(5000);
    }
}

/** @} */
