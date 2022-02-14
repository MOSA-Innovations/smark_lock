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

/**  @brief     hardware.
 *   @details   This module contains the hardware GPIOs and configurations.
 */

// Motor1 Pin Definitions
#define STP1_STATUS 11
#define STP1_MODE1 16
#define STP1_MODE2 15
#define STP1_MODE3 12
#define STP1_STCK 8
#define STP1_DIR 22

// Motor2 Pin Definitions
#define STP2_STATUS 31
#define STP2_MODE1 25
#define STP2_MODE2 28
#define STP2_MODE3 29
#define STP2_STCK 24
#define STP2_DIR 23

// Enable motor drivers
#define STP1_STANDBY 4
#define STP2_STANDBY 5

// Power enable pins for leds and 12V devices. 
#define POWER_EN_3V3 20
#define POWER_EN_12V 19

// Chain detect interrup pins
#define CHAIN_DETECT_PIN_A 13
#define CHAIN_DETECT_PIN_B 14

// WS2812 LED data pin
#define LED_DATA_PIN 17

// Buzzer pin
#define BUZZER_PIN 18

// MPU i2c definitions
#define MPU_TWI_SCL_PIN 27
#define MPU_TWI_SDA_PIN 26

// MPU interrupt pin
#define MPU_INT 30
//EOF