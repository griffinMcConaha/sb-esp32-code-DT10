#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "dispersion_controller.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Shared runtime state guarded by spinlocks for ISR/task safety.
static portMUX_TYPE pulse_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t rpm_pulse_count = 0;
static volatile float rpm_value = 0.0f;

static volatile uint32_t flow_pulse_count = 0;
static volatile float flow_freq_hz = 0.0f;

static volatile float v1 = 0.0f;
static volatile float v2 = 0.0f;
static volatile uint8_t dac1_val = 0;
static volatile uint8_t dac2_val = 0;

static dac_oneshot_handle_t dac1_handle = NULL;
static dac_oneshot_handle_t dac2_handle = NULL;

typedef struct {
	float rpm;
	float flow_hz;
	uint32_t timestamp_ms;
} FeedbackData_t;

static volatile FeedbackData_t feedback = {0.0f, 0.0f, 0};
static portMUX_TYPE feedback_mux = portMUX_INITIALIZER_UNLOCKED;

// Clamp helper used by output calibration functions.
static float clamp_voltage(float voltage)
{
	if (voltage < OUTPUT_VOLTAGE_MIN) {
		return OUTPUT_VOLTAGE_MIN;
	}
	if (voltage > OUTPUT_VOLTAGE_MAX) {
		return OUTPUT_VOLTAGE_MAX;
	}
	return voltage;
}

static float clamp_percent(float percent)
{
	if (percent < 0.0f) {
		return 0.0f;
	}
	if (percent > 100.0f) {
		return 100.0f;
	}
	return percent;
}

static float rpm_to_v1(float rpm)
{
	return clamp_voltage((rpm + RPM_TO_V1_OFFSET) / RPM_TO_V1_DIVISOR);
}

static float flow_to_v2(float flow)
{
	return clamp_voltage((flow + FLOW_TO_V2_OFFSET) / FLOW_TO_V2_DIVISOR);
}

static uint8_t volts_to_dac(float voltage)
{
	if (voltage < 0.0f) {
		voltage = 0.0f;
	}
	if (voltage > OUTPUT_VOLTAGE_MAX) {
		voltage = OUTPUT_VOLTAGE_MAX;
	}
	return (uint8_t)((voltage / OUTPUT_VOLTAGE_MAX) * DAC_OUTPUT_MAX_CODE);
}

// Applies independent SALT/BRINE percentages by mapping each one to target
// RPM/flow, then converting to analog DAC setpoints through calibration formulas.
static void apply_percentages(float salt_percent, float brine_percent)
{
	float bounded_salt = clamp_percent(salt_percent);
	float bounded_brine = clamp_percent(brine_percent);
	float rpm = RPM_MIN + (bounded_salt / 100.0f) * (RPM_MAX - RPM_MIN);
	float flow = FLOW_MIN + (bounded_brine / 100.0f) * (FLOW_MAX - FLOW_MIN);

	float next_v1 = rpm_to_v1(rpm);
	float next_v2 = flow_to_v2(flow);
	uint8_t next_dac1 = volts_to_dac(next_v1);
	uint8_t next_dac2 = volts_to_dac(next_v2);

	taskENTER_CRITICAL(&pulse_mux);
	v1 = next_v1;
	v2 = next_v2;
	dac1_val = next_dac1;
	dac2_val = next_dac2;
	taskEXIT_CRITICAL(&pulse_mux);

	printf("Updated SALT %.1f%%, BRINE %.1f%% -> V1 = %.2f V, V2 = %.2f V\n",
		   bounded_salt, bounded_brine, next_v1, next_v2);
}

static void apply_percentage(float percent)
{
	apply_percentages(percent, percent);
}

static void send_stm32_line(const char *line)
{
	if (!line) {
		return;
	}
	(void)uart_write_bytes(STM32_UART_NUM, line, strlen(line));
}

// Accepts commands:
// 1) PCT:<percent>
// 2) SALT:<percent>,BRINE:<percent>
static void process_stm32_command(const char *line)
{
	if (!line || line[0] == '\0') {
		return;
	}

	float percent = 0.0f;
	float salt = 0.0f;
	float brine = 0.0f;

	if (sscanf(line, "PCT:%f", &percent) == 1) {
		apply_percentage(percent);
		send_stm32_line("STATUS:OK\r\n");
		return;
	}

	if (sscanf(line, "SALT:%f,BRINE:%f", &salt, &brine) == 2) {
		apply_percentages(salt, brine);
		send_stm32_line("STATUS:OK\r\n");
		return;
	}

	printf("Invalid STM32 command: %s\n", line);
	send_stm32_line("STATUS:ERROR,BAD_CMD\r\n");
}

// Measures flow frequency by pulse delta in a fixed 1-second window.
static void measure_flow_frequency(void)
{
	uint32_t count_start;
	uint32_t count_end;

	portENTER_CRITICAL(&pulse_mux);
	count_start = flow_pulse_count;
	portEXIT_CRITICAL(&pulse_mux);

	vTaskDelay(pdMS_TO_TICKS(FLOW_MEASUREMENT_WINDOW_MS));

	portENTER_CRITICAL(&pulse_mux);
	count_end = flow_pulse_count;
	portEXIT_CRITICAL(&pulse_mux);

	uint32_t pulse_delta = count_end - count_start;
	float frequency_hz = (float)pulse_delta;

	if (frequency_hz < 1.0f) {
		frequency_hz = 0.0f;
	}

	portENTER_CRITICAL(&pulse_mux);
	flow_freq_hz = frequency_hz;
	portEXIT_CRITICAL(&pulse_mux);
}

static void IRAM_ATTR on_rpm_pulse(void *arg)
{
	(void)arg;
	portENTER_CRITICAL_ISR(&pulse_mux);
	rpm_pulse_count++;
	portEXIT_CRITICAL_ISR(&pulse_mux);
}

static void IRAM_ATTR on_flow_pulse(void *arg)
{
	(void)arg;
	portENTER_CRITICAL_ISR(&pulse_mux);
	flow_pulse_count++;
	portEXIT_CRITICAL_ISR(&pulse_mux);
}

// Periodic RPM estimator task.
static void rpm_task(void *arg)
{
	(void)arg;

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(RPM_TASK_PERIOD_MS));

		uint32_t count_snapshot;
		portENTER_CRITICAL(&pulse_mux);
		count_snapshot = rpm_pulse_count;
		rpm_pulse_count = 0;
		rpm_value = (count_snapshot / PPR) * 60.0f;
		portEXIT_CRITICAL(&pulse_mux);

		portENTER_CRITICAL(&feedback_mux);
		feedback.rpm = rpm_value;
		feedback.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
		portEXIT_CRITICAL(&feedback_mux);
	}
}

// Periodic flow estimator task.
static void flow_task(void *arg)
{
	(void)arg;

	while (1) {
		measure_flow_frequency();

		portENTER_CRITICAL(&feedback_mux);
		feedback.flow_hz = flow_freq_hz;
		feedback.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
		portEXIT_CRITICAL(&feedback_mux);
	}
}

// Line-oriented UART parser for incoming STM32 commands.
static void stm32_rx_task(void *arg)
{
	(void)arg;

	char line[STM32_RX_LINE_BUFFER_LEN];
	size_t index = 0;
	uint8_t byte = 0;

	while (1) {
		int got = uart_read_bytes(STM32_UART_NUM, &byte, 1, pdMS_TO_TICKS(STM32_RX_TIMEOUT_MS));
		if (got <= 0) {
			continue;
		}

		if (byte == '\r' || byte == '\n') {
			if (index > 0) {
				line[index] = '\0';
				process_stm32_command(line);
				index = 0;
			}
			continue;
		}

		if (index < (sizeof(line) - 1)) {
			line[index++] = (char)byte;
		} else {
			index = 0;
			send_stm32_line("STATUS:ERROR,OVERFLOW\r\n");
		}
	}
}

// Main control/telemetry loop:
// - pushes DAC outputs
// - reads latest feedback snapshot
// - sends FLOW:SALT,BRINE,RPM telemetry to STM32
static void main_loop_task(void *arg)
{
	(void)arg;

	while (1) {
		uint8_t out_dac1;
		uint8_t out_dac2;
		float out_v1;
		float out_v2;

		portENTER_CRITICAL(&pulse_mux);
		out_dac1 = dac1_val;
		out_dac2 = dac2_val;
		out_v1 = v1;
		out_v2 = v2;
		portEXIT_CRITICAL(&pulse_mux);

		if (dac1_handle != NULL) {
			(void)dac_oneshot_output_voltage(dac1_handle, out_dac1);
		}
		if (dac2_handle != NULL) {
			(void)dac_oneshot_output_voltage(dac2_handle, out_dac2);
		}

		float fb_rpm;
		float fb_flow_hz;
		portENTER_CRITICAL(&feedback_mux);
		fb_rpm = feedback.rpm;
		fb_flow_hz = feedback.flow_hz;
		portEXIT_CRITICAL(&feedback_mux);

		float flow_lpm = (fb_flow_hz > 0.0f) ? (fb_flow_hz / FLOW_HZ_TO_LPM_DIVISOR) : 0.0f;
		float flow_mlmin = flow_lpm * LPM_TO_MLMIN_FACTOR;
		uint16_t brine_mlmin = (flow_mlmin > 0.0f) ? (uint16_t)flow_mlmin : 0;
		uint16_t salt_mlmin = brine_mlmin / SALT_FROM_BRINE_DIVISOR;

		printf("DAC25 = %.2f V, DAC26 = %.2f V | Freq = %.1f Hz | Flow = %.2f L/min | RPM = %.1f\n",
			   out_v1, out_v2, fb_flow_hz, flow_lpm, fb_rpm);

		char flow_msg[STM32_TX_FLOW_MSG_BUFFER_LEN];
		snprintf(flow_msg, sizeof(flow_msg), "FLOW:SALT:%u,BRINE:%u,RPM:%.1f\r\n", salt_mlmin, brine_mlmin, fb_rpm);
		send_stm32_line(flow_msg);

		vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
	}
}

void dispersion_controller_start(void)
{
	// Bring up analog outputs first, then UART and GPIO/interrupt inputs.
	vTaskDelay(pdMS_TO_TICKS(BOARD_STARTUP_DELAY_MS));

	dac_oneshot_config_t dac1_cfg = {
		.chan_id = DAC1_CHANNEL,
	};
	dac_oneshot_config_t dac2_cfg = {
		.chan_id = DAC2_CHANNEL,
	};
	ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac1_cfg, &dac1_handle));
	vTaskDelay(pdMS_TO_TICKS(DAC_CHANNEL_INIT_DELAY_MS));
	ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac2_cfg, &dac2_handle));
	vTaskDelay(pdMS_TO_TICKS(DAC_CHANNEL_INIT_DELAY_MS));

	uart_config_t uart_cfg = {
		.baud_rate = STM32_UART_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(STM32_UART_NUM, 256, 0, 0, NULL, 0);
	uart_param_config(STM32_UART_NUM, &uart_cfg);
	uart_set_pin(STM32_UART_NUM, STM32_UART_TX_PIN, STM32_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	gpio_config_t flow_cfg = {
		.pin_bit_mask = (1ULL << FLOW_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&flow_cfg);

	gpio_config_t rpm_cfg = {
		.pin_bit_mask = (1ULL << RPM_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&rpm_cfg);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(RPM_PIN, on_rpm_pulse, NULL);
	gpio_isr_handler_add(FLOW_PIN, on_flow_pulse, NULL);

	apply_percentage(0.0f);
	send_stm32_line("STATUS:OK\r\n");

	// Start runtime workers.
	xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 11, NULL);
	xTaskCreate(flow_task, "flow_task", 2048, NULL, 11, NULL);
	xTaskCreate(stm32_rx_task, "stm32_rx_task", 4096, NULL, 8, NULL);
	xTaskCreate(main_loop_task, "main_loop_task", 4096, NULL, 9, NULL);

	printf("[CTRL] Dispersion controller started\n");

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
