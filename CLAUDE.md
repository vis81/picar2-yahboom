# Yahboom ROS STM32F103 Firmware

Zephyr RTOS firmware for the Yahboom ROS control board (PICAR-2 robot project).

## Skill

A build/flash skill is available at `.claude/skills/yahboom-build/SKILL.md`.
Read it before helping with any build, flash, or environment setup tasks in this project.

## Application Structure

The real application is in `yahboom/src/` (not `app/src/`, which is a Zephyr sample template).

Key source files:
- `main.c` — initializes all subsystems; registers `sys` shell commands (bootloader/reboot/halt/uptime/version)
- `comms.c/h` — Pi↔STM32 protocol host (USART1, 460800 baud); connection watchdog; per-stream rate control
- `protocol.c/h` — framing: CRC-8 Dallas/Maxim (0x31), frame `[0xAA][TYPE][LEN][PAYLOAD][CRC8]`
- `motor.c/h` — DC motor control + encoder position (quad decoder ticks)
- `servo.c/h` — software-PWM steering servo (0–100 range)
- `imu.c/h` — MPU-9250: accel, gyro, mag, temp
- `battery.c/h` — battery voltage ADC + charge-percent estimate
- `rc.c/h` — FlySky SBUS RC receiver (USART2, 100000 baud); active when Pi disconnected
- `buzzer.c/h` — piezo buzzer
- `selftest.c` — board self-test

## Custom Drivers

- `drivers/motor/hbridge.c` — H-bridge motor driver
- `drivers/pwm/pwm_stm32_sw.c` — software PWM (servos; zero-latency timer ISR)
- `drivers/pwm/pwm_servo.c` — servo PWM abstraction
- `drivers/input/input_stm32_qdec.c` — STM32 hardware quadrature decoder (wheel encoders)

## Protocol (USART1, 460800 baud)

Frame: `[0xAA][TYPE:u8][LEN:u8][PAYLOAD:LEN bytes][CRC8]`

STM32→Pi: `0x01` JOINT_STATE, `0x02` IMU, `0x03` BATTERY, `0x04` STATS  
Pi→STM32: `0x80` CMD_VEL, `0x81` REQ, `0x82` SET_RATE, `0x83` GET_STATS

All streams default to 0 Hz (silent). CMD_VEL is the keepalive; 500 ms silence → RC fallback.
See `docs/plans/protocol.md` for full frame definitions.

## Serial Ports

| Port | UART | Baud | Purpose |
|---|---|---|---|
| `/dev/ttyYahboom0` | USART1 | 921600 (ROM bootloader) / 460800 (app) | stm32flash entry; Pi binary protocol when running |
| `/dev/ttyYahboom1` | USART3 | 921600 | Zephyr shell console (DMA async RX+TX) |

USART2 (internal): 100000 baud, FlySky SBUS RC receiver (inverted signal).

## Tools

- `tools/picar2_gui.py` — Python/tkinter GUI: manual drive + telemetry (`/dev/ttyYahboom0`, 460800)
- `tests/integration/test_protocol.py` — protocol integration tests (requires board connected)
- `tests/integration/test_shell_echo.py` — shell echo integrity test (USART3)
- `yahboom/scripts/pid_test.py` — motor PID test helper

## Persistent Storage

NVS flash + Zephyr Settings API is enabled. Currently used by `imu.c` to persist magnetometer
calibration offsets under the key `imu/offsets`. Shell commands `imu cal` / `imu cal reset`
save/clear via `settings_save_one` / `settings_delete`.

## Zephyr Patches

Two patches in `zephyr_patches/` are applied to `zephyr_os/zephyr` manually — `activate.sh`
and `west update` do **not** apply them automatically. After any `west update`, re-apply:

```sh
git -C zephyr_os/zephyr apply ../../zephyr_patches/uart_stm32_async_rxne_irq_guard.patch
git -C zephyr_os/zephyr apply ../../zephyr_patches/rtio_workq_thread_name.patch
```

- `uart_stm32_async_rxne_irq_guard.patch` — guards the RXNE ISR path behind `!data->user_cb`
  so it doesn't fire (and clear the flag) when the port is in async/DMA mode.
- `rtio_workq_thread_name.patch` — names the RTIO workqueue threads for easier debug.

`picar2comms.patch` at the repo root is a historical artifact showing how `protocol.c`/`comms.c`
were added; it is already applied and can be ignored.

## Hardware Notes

- MCU: STM32F103RCT6, 256 KB flash, 48 KB RAM
- BOOT0 is a physical button (not a jumper). Use `sys bootloader` shell command to enter ROM bootloader from software.
- USB-serial adapters: CH340 chips. udev symlinks set by `etc/99-usb-serial.rules`.
- MPU-9250 connected via bit-bang I2C3 (PB13=SCL, PB15=SDA) — not hardware I2C peripheral.
- USART3 uses DMA async (DMA1 CH3=RX, CH2=TX) to avoid per-byte RXNE ISR overrun at 921600 baud.
- USART1 app baud is 460800 (not 921600) — STM32F103 USART has no RX FIFO; motor ISRs at 921600 caused ~30% CRC errors.
- C library: `boards/yahboom_ros_stm32f103.conf` forces `CONFIG_NEWLIB_LIBC=y` (overrides `CONFIG_PICOLIBC=y` in `prj.conf`) due to picolibc 1.8.x / SDK 0.17.x lock-type incompatibility with Zephyr v3.7.0.
