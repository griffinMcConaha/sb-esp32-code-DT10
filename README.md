# SB ESP32 Dispersion Controller

This project runs on ESP32 and provides dispersion control + telemetry exchange with an STM32 over UART.

## Overview

- Receives control commands from STM32 on UART1 (ESP32 GPIO16/GPIO17).
- Converts standardized percentages into calibrated DAC outputs:
  - `SALT%` -> RPM target -> DAC V1 (GPIO25 / DAC CH0)
  - `BRINE%` -> flow target -> DAC V2 (GPIO26 / DAC CH1)
- Measures pulse inputs:
  - RPM pulses on GPIO32
  - Flow pulses on GPIO33
- Sends periodic telemetry back to STM32.

## File Structure

- `main/main.c` - minimal ESP-IDF entrypoint (`app_main`).
- `main/dispersion_controller.c` - runtime logic (UART parsing, tasks, DAC updates, telemetry).
- `main/dispersion_controller.h` - controller public interface.
- `main/board_config.h` - board/config constants (pins, timing, calibration, protocol sizes).

## UART Protocol (ESP32 <-> STM32)

### Incoming commands (STM32 -> ESP32)

- `PCT:<percent>`
  - Applies same percent to both SALT and BRINE paths.
- `SALT:<percent>,BRINE:<percent>`
  - Applies independent standardized percentages.

Valid percentage range is `0..100` (values are clamped).

### Outgoing status/telemetry (ESP32 -> STM32)

- `STATUS:OK`
- `STATUS:ERROR,BAD_CMD`
- `STATUS:ERROR,OVERFLOW`
- `FLOW:SALT:<ml_min>,BRINE:<ml_min>,RPM:<rpm>` (sent periodically)

Line termination is `\r\n`.

## Wiring

Use UART cross-connect plus shared ground:

- ESP32 `GPIO17 (TX)` -> STM32 `UART4_RX` (typically `PC11`)
- ESP32 `GPIO16 (RX)` <- STM32 `UART4_TX` (typically `PC10`)
- ESP32 `GND` <-> STM32 `GND`

Signal levels must be 3.3V logic.

## Build / Flash / Monitor (ESP-IDF)

From this workspace in VS Code with ESP-IDF extension:

1. Build
2. Flash
3. Monitor

If using CLI flow, run the equivalent `idf.py build`, `idf.py -p <COMx> flash`, `idf.py -p <COMx> monitor`.

## Configuration and Tuning

Tune these in `main/board_config.h`:

- Pin mapping (`FLOW_PIN`, `RPM_PIN`, UART pins)
- Timing (`RPM_TASK_PERIOD_MS`, `FLOW_MEASUREMENT_WINDOW_MS`, `MAIN_LOOP_PERIOD_MS`)
- Calibration constants (`RPM_TO_V1_*`, `FLOW_TO_V2_*`)
- Range constraints (`RPM_MIN/MAX`, `FLOW_MIN/MAX`, voltage limits)

## Notes

- `app_main` is required by ESP-IDF and intentionally minimal.
- Runtime logic is centralized in `dispersion_controller.c` for easier maintenance.
- If monitor output shows all zeros, verify sensor wiring and pulse generation on GPIO32/GPIO33.
