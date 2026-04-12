#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// Hardware pin mapping (ESP32 side)
#define DAC1_CHANNEL DAC_CHAN_0
#define DAC2_CHANNEL DAC_CHAN_1
#define FLOW_PIN GPIO_NUM_33
#define RPM_PIN GPIO_NUM_32

// External system control outputs (available GPIOs; change as needed)
// System 1: Brine agitator
#define BRINE_AGITATOR_ENABLE_PIN GPIO_NUM_13
// System 2: Salt thrower (turn on when commanded salt percent > 0)
#define SALT_THROWER_ENABLE_PIN GPIO_NUM_12
// Sabertooth power relay control
#define SABERTOOTH_RELAY_PIN GPIO_NUM_14
// Vibration motor switch output
#define VIBRATION_MOTOR_ENABLE_PIN GPIO_NUM_27

// UART link to STM32 (command + feedback channel)
#define STM32_UART_NUM UART_NUM_1
#define STM32_UART_TX_PIN GPIO_NUM_17
#define STM32_UART_RX_PIN GPIO_NUM_16
#define STM32_UART_BAUD 230400

// Encoder pulses-per-revolution for RPM conversion
#define PPR 600.0f

// Output operating ranges used by apply_percentage(%)
#define RPM_MIN 10.0f
#define RPM_MAX 90.0f
#define FLOW_MIN 700.0f
#define FLOW_MAX 1300.0f

// Analog output electrical limits and DAC scale
#define OUTPUT_VOLTAGE_MIN 1.65f
#define OUTPUT_VOLTAGE_MAX 3.3f
#define DAC_OUTPUT_MAX_CODE 255.0f

// Calibration formulas:
// V1 = (RPM + RPM_TO_V1_OFFSET) / RPM_TO_V1_DIVISOR
// V2 = (FLOW + FLOW_TO_V2_OFFSET) / FLOW_TO_V2_DIVISOR
#define RPM_TO_V1_OFFSET 99.51f
#define RPM_TO_V1_DIVISOR 58.60f
#define FLOW_TO_V2_OFFSET 1012.37f
#define FLOW_TO_V2_DIVISOR 717.01f

// Feedback conversion factors (sensor frequency -> display units)
#define FLOW_HZ_TO_LPM_DIVISOR 86.0f
#define LPM_TO_MLMIN_FACTOR (1000.0f / 60.0f)
#define SALT_FROM_BRINE_DIVISOR 9U

// Task and measurement timing
#define BOARD_STARTUP_DELAY_MS 200
#define DAC_CHANNEL_INIT_DELAY_MS 50
#define FLOW_MEASUREMENT_WINDOW_MS 1000
#define RPM_TASK_PERIOD_MS 1000
#define MAIN_LOOP_PERIOD_MS 500
#define STM32_RX_TIMEOUT_MS 20
#define STM32_RX_IDLE_FLUSH_MS 120

// ESP32 runtime memory headroom
#define STM32_UART_RX_BUF_SIZE 2048
#define STM32_UART_EVENT_QUEUE_LEN 16
#define RPM_TASK_STACK_SIZE 3072
#define FLOW_TASK_STACK_SIZE 3072
#define STM32_RX_TASK_STACK_SIZE 6144
#define MAIN_LOOP_TASK_STACK_SIZE 6144

#define STARTUP_CHECK_DURATION_MS 180000
#define STARTUP_UNLOCK_TIMEOUT_MS 15000

// UART line/message buffer sizes
#define STM32_RX_LINE_BUFFER_LEN 256
#define STM32_TX_FLOW_MSG_BUFFER_LEN 256

#endif
