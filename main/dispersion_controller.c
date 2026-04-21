#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "dispersion_controller.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// This module is the ESP32-side spreader controller. It translates high-level
// SALT/BRINE commands from the STM32 into DAC outputs and GPIO enables, while
// also measuring flow/RPM feedback and reporting that telemetry back upstream.

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
static QueueHandle_t uart_event_queue = NULL;

typedef struct {
	float rpm;
	float flow_hz;
	uint32_t timestamp_ms;
} FeedbackData_t;

static volatile FeedbackData_t feedback = {0.0f, 0.0f, 0};
static portMUX_TYPE feedback_mux = portMUX_INITIALIZER_UNLOCKED;
static bool salt_thrower_active = false;
static volatile float current_salt_percent = 0.0f;
static volatile float current_brine_percent = 0.0f;
static volatile bool startup_check_completed = false;
static volatile bool startup_check_running = false;
static volatile TickType_t startup_check_started_tick = 0;
static volatile TickType_t startup_gate_opened_tick = 0;

static void send_stm32_line(const char *line);

// The external drivers and actuators are deliberately split across dedicated
// enable pins so the controller can independently gate agitator, thrower,
// relay, and vibration hardware during startup/test sequences.
static void configure_external_system_outputs(void)
{
	gpio_config_t output_cfg = {
		.pin_bit_mask = (1ULL << BRINE_AGITATOR_ENABLE_PIN) |
				(1ULL << SALT_THROWER_ENABLE_PIN) |
				(1ULL << SABERTOOTH_RELAY_PIN) |
				(1ULL << VIBRATION_MOTOR_ENABLE_PIN),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&output_cfg);
}

static void set_brine_agitator_active(bool active)
{
	gpio_set_level(BRINE_AGITATOR_ENABLE_PIN, active ? 1 : 0); // logic active-high
}

static void set_salt_thrower_active(bool active)
{
	gpio_set_level(SALT_THROWER_ENABLE_PIN, active ? 1 : 0); // logic active-high
}

static void set_sabertooth_relay_active(bool active)
{
	gpio_set_level(SABERTOOTH_RELAY_PIN, active ? 1 : 0);
}

static void set_vibration_motor_active(bool active)
{
	gpio_set_level(VIBRATION_MOTOR_ENABLE_PIN, active ? 1 : 0); // logic active-high
}

static void update_salt_thrower_from_salt_percent(float salt_percent)
{
	bool should_enable = salt_percent > 0.0f;
	if (should_enable != salt_thrower_active) {
		salt_thrower_active = should_enable;
		set_salt_thrower_active(salt_thrower_active);
		set_vibration_motor_active(salt_thrower_active);
		printf("[CTRL] Salt thrower %s (salt=%.1f%%)\n", salt_thrower_active ? "ON" : "OFF", salt_percent);
	}
}

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
static bool apply_percentages(float salt_percent, float brine_percent)
{
	if (!startup_check_completed) {
		// Refuse real dispensing commands until the startup gate is satisfied so
		// the mechanical system has a chance to prime safely first.
		printf("[CTRL] Ignored SALT/BRINE command until STARTUP_CHECK completes\n");
		send_stm32_line("STATUS:ERROR,STARTUP_REQUIRED\r\n");
		return false;
	}

	float bounded_salt = clamp_percent(salt_percent);
	float bounded_brine = clamp_percent(brine_percent);
	float rpm = (bounded_salt == 0.0f)
			? 0.0f
			: (RPM_MIN + (bounded_salt / 100.0f) * (RPM_MAX - RPM_MIN));
	float flow = (bounded_brine == 0.0f)
			 ? 0.0f
			 : (FLOW_MIN + (bounded_brine / 100.0f) * (FLOW_MAX - FLOW_MIN));

	float next_v1 = rpm_to_v1(rpm);
	float next_v2 = flow_to_v2(flow);
	uint8_t next_dac1 = volts_to_dac(next_v1);
	uint8_t next_dac2 = volts_to_dac(next_v2);

	taskENTER_CRITICAL(&pulse_mux);
	v1 = next_v1;
	v2 = next_v2;
	dac1_val = next_dac1;
	dac2_val = next_dac2;
	current_salt_percent = bounded_salt;
	current_brine_percent = bounded_brine;
	taskEXIT_CRITICAL(&pulse_mux);

	update_salt_thrower_from_salt_percent(bounded_salt);

	printf("Updated SALT %.1f%%, BRINE %.1f%% -> V1 = %.2f V, V2 = %.2f V\n",
		   bounded_salt, bounded_brine, next_v1, next_v2);
	return true;
}

static bool apply_percentage(float percent)
{
	return apply_percentages(percent, percent);
}

static void send_stm32_line(const char *line)
{
	if (!line) {
		return;
	}
	(void)uart_write_bytes(STM32_UART_NUM, line, strlen(line));
}

static void start_startup_check(void)
{
	if (startup_check_running) {
		printf("[CTRL] STARTUP_CHECK already running\n");
		send_stm32_line("STATUS:OK,STARTUP_CHECK_RUNNING\r\n");
		return;
	}

	startup_check_running = true;
	startup_check_completed = false;
	startup_check_started_tick = xTaskGetTickCount();
	set_brine_agitator_active(true);

	printf("[CTRL] STARTUP_CHECK accepted; agitating brine for %u ms\n", STARTUP_CHECK_DURATION_MS);
	send_stm32_line("STATUS:OK,STARTUP_CHECK_STARTED\r\n");
}

static void bypass_startup_check(void)
{
	startup_check_running = false;
	startup_check_completed = true;
	set_brine_agitator_active(false);
	printf("[CTRL] STARTUP_CHECK bypassed by STM command\n");
	send_stm32_line("STATUS:OK,STARTUP_CHECK_BYPASSED\r\n");
}

// Accepts commands:[]
// 1) PCT:<percent>
// 2) SALT:<percent>,BRINE:<percent>
// 3) TEST SALT <percent>
// 4) TEST BRINE <percent>
// 5) STARTUP_CHECK
// 6) STARTUP_BYPASS
// 7) s / S (shortcut for STARTUP_BYPASS)
// 8) AGITATOR ON / AGITATOR OFF
// 9) THROWER ON  / THROWER OFF
// 10) VIBRATION ON / VIBRATION OFF
static void process_stm32_command(const char *line)
{
	if (!line || line[0] == '\0') {
		return;
	}

	float percent = 0.0f;
	float salt = 0.0f;
	float brine = 0.0f;

	// Keep command parsing explicit and line-oriented because this path is used
	// both by real upstream traffic and by manual bench-test commands.
	if (strcmp(line, "STARTUP_CHECK") == 0) {
		start_startup_check();
		return;
	}

	if (strcmp(line, "STARTUP_BYPASS") == 0) {
		bypass_startup_check();
		return;
	}

	if ((strcmp(line, "s") == 0) || (strcmp(line, "S") == 0)) {
		bypass_startup_check();
		return;
	}

	if (sscanf(line, "PCT:%f", &percent) == 1) {
		if (apply_percentage(percent)) {
			send_stm32_line("STATUS:OK\r\n");
		}
		return;
	}

	if (sscanf(line, "SALT:%f,BRINE:%f", &salt, &brine) == 2) {
		if (apply_percentages(salt, brine)) {
			send_stm32_line("STATUS:OK\r\n");
		}
		return;
	}

	if (sscanf(line, "TEST SALT %f", &salt) == 1) {
		if (apply_percentages(salt, 0.0f)) {
			send_stm32_line("STATUS:OK\r\n");
		}
		return;
	}

	if (sscanf(line, "TEST BRINE %f", &brine) == 1) {
		if (apply_percentages(0.0f, brine)) {
			send_stm32_line("STATUS:OK\r\n");
		}
		return;
	}

	if (strcmp(line, "AGITATOR ON") == 0) {
		set_brine_agitator_active(true);
		printf("[CTRL] Brine agitator forced ON via test command\n");
		send_stm32_line("STATUS:OK,AGITATOR:ON\r\n");
		return;
	}

	if (strcmp(line, "AGITATOR OFF") == 0) {
		set_brine_agitator_active(false);
		printf("[CTRL] Brine agitator forced OFF via test command\n");
		send_stm32_line("STATUS:OK,AGITATOR:OFF\r\n");
		return;
	}

	if (strcmp(line, "THROWER ON") == 0) {
		salt_thrower_active = true;
		set_salt_thrower_active(true);
		printf("[CTRL] Salt thrower forced ON via test command\n");
		send_stm32_line("STATUS:OK,THROWER:ON\r\n");
		return;
	}

	if (strcmp(line, "THROWER OFF") == 0) {
		salt_thrower_active = false;
		set_salt_thrower_active(false);
		printf("[CTRL] Salt thrower forced OFF via test command\n");
		send_stm32_line("STATUS:OK,THROWER:OFF\r\n");
		return;
	}

	if (strcmp(line, "RELAY ON") == 0) {
		set_sabertooth_relay_active(true);
		printf("[CTRL] Sabertooth relay forced ON via test command\n");
		send_stm32_line("STATUS:OK,RELAY:ON\r\n");
		return;
	}

	if (strcmp(line, "RELAY OFF") == 0) {
		set_sabertooth_relay_active(false);
		printf("[CTRL] Sabertooth relay forced OFF via test command\n");
		send_stm32_line("STATUS:OK,RELAY:OFF\r\n");
		return;
	}

	if (strcmp(line, "VIBRATION ON") == 0) {
		set_vibration_motor_active(true);
		printf("[CTRL] Vibration motor forced ON via test command\n");
		send_stm32_line("STATUS:OK,VIBRATION:ON\r\n");
		return;
	}

	if (strcmp(line, "VIBRATION OFF") == 0) {
		set_vibration_motor_active(false);
		printf("[CTRL] Vibration motor forced OFF via test command\n");
		send_stm32_line("STATUS:OK,VIBRATION:OFF\r\n");
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

// Line-oriented UART parser for incoming STM32 commands using event queue.
static void stm32_rx_task(void *arg)
{
	(void)arg;

	char line[STM32_RX_LINE_BUFFER_LEN];
	size_t index = 0;
	uart_event_t event = {0};
	uint8_t *dtmp = (uint8_t *)malloc(1024);

	while (1) {
		if (xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY)) {
			switch (event.type) {
			case UART_DATA:
				// Reassemble complete newline-delimited commands before handing
				// them to the command parser so partial UART reads stay harmless.
				// Read available bytes
				int len = uart_read_bytes(STM32_UART_NUM, dtmp, event.size, 0);
				for (int i = 0; i < len; i++) {
					uint8_t byte = dtmp[i];

					if (byte == '\r' || byte == '\n') {
						if (index > 0) {
							line[index] = '\0';
							process_stm32_command(line);
							index = 0;
						}
					} else if (index < (sizeof(line) - 1)) {
						line[index++] = (char)byte;
					} else {
						index = 0;
						send_stm32_line("STATUS:ERROR,OVERFLOW\r\n");
					}
				}
				break;
			default:
				break;
			}
		}
	}
	free(dtmp);
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
		float cmd_salt_percent;
		float cmd_brine_percent;

		portENTER_CRITICAL(&pulse_mux);
		out_dac1 = dac1_val;
		out_dac2 = dac2_val;
		out_v1 = v1;
		out_v2 = v2;
		cmd_salt_percent = current_salt_percent;
		cmd_brine_percent = current_brine_percent;
		portEXIT_CRITICAL(&pulse_mux);

		if (dac1_handle != NULL) {
			(void)dac_oneshot_output_voltage(dac1_handle, out_dac1);
		}
		if (dac2_handle != NULL) {
			(void)dac_oneshot_output_voltage(dac2_handle, out_dac2);
		}

		if (startup_check_running) {
			TickType_t now_ticks = xTaskGetTickCount();
			uint32_t elapsed_ms = (uint32_t)((now_ticks - startup_check_started_tick) * portTICK_PERIOD_MS);
			if (elapsed_ms >= STARTUP_CHECK_DURATION_MS) {
				startup_check_running = false;
				startup_check_completed = true;
				set_brine_agitator_active(false);
				printf("[CTRL] STARTUP_CHECK complete after %lu ms\n", (unsigned long)elapsed_ms);
				send_stm32_line("STATUS:OK,STARTUP_CHECK_COMPLETE\r\n");
			}
		} else if (!startup_check_completed) {
			// The timeout path is a safety valve for development and recovery: if
			// the startup procedure is never explicitly completed, command gating
			// eventually opens so the system does not stay permanently locked out.
			TickType_t now_ticks = xTaskGetTickCount();
			uint32_t gate_elapsed_ms = (uint32_t)((now_ticks - startup_gate_opened_tick) * portTICK_PERIOD_MS);
			if (gate_elapsed_ms >= STARTUP_UNLOCK_TIMEOUT_MS) {
				startup_check_completed = true;
				set_brine_agitator_active(false);
				printf("[CTRL] Startup gate timed out after %lu ms; enabling commands\n", (unsigned long)gate_elapsed_ms);
				send_stm32_line("STATUS:OK,STARTUP_TIMEOUT_UNLOCK\r\n");
			}
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
		// Report both measured output and commanded percentages so the STM32 can
		// compare intent versus observed spreader behavior in one frame.
		snprintf(flow_msg, sizeof(flow_msg),
			 "FLOW:SALT:%u,BRINE:%u,RPM:%.1f,LPM:%.2f,HZ:%.1f,CMD_SALT:%.1f,CMD_BRINE:%.1f\r\n",
			 salt_mlmin, brine_mlmin, fb_rpm, flow_lpm, fb_flow_hz, cmd_salt_percent, cmd_brine_percent);
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
	uart_driver_install(STM32_UART_NUM, STM32_UART_RX_BUF_SIZE, 0, STM32_UART_EVENT_QUEUE_LEN, &uart_event_queue, 0);
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

	configure_external_system_outputs();
	set_sabertooth_relay_active(true);
	set_brine_agitator_active(false);
	set_salt_thrower_active(false);
	set_vibration_motor_active(false);
	printf("[CTRL] Brine agitator OFF at boot; waiting for explicit STM command\n");
 
	startup_gate_opened_tick = xTaskGetTickCount();

	startup_check_completed = true;
	(void)apply_percentage(0.0f);
	startup_check_completed = false;
	send_stm32_line("STATUS:OK\r\n");

	// Start runtime workers.
	xTaskCreate(rpm_task, "rpm_task", RPM_TASK_STACK_SIZE, NULL, 11, NULL);
	xTaskCreate(flow_task, "flow_task", FLOW_TASK_STACK_SIZE, NULL, 11, NULL);
	xTaskCreate(stm32_rx_task, "stm32_rx_task", STM32_RX_TASK_STACK_SIZE, NULL, 8, NULL);
	xTaskCreate(main_loop_task, "main_loop_task", MAIN_LOOP_TASK_STACK_SIZE, NULL, 9, NULL);

	printf("[CTRL] Dispersion controller started\n");

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
