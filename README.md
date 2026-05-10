# Yahboom ROS Control Board Firmware

Zephyr-based firmware for the **Yahboom ROS STM32F103RCT6** robot control board (PICAR-2).

## Hardware

| | |
|---|---|
| MCU | STM32F103RCT6 |
| Flash | 256 KB |
| RAM | 48 KB |
| Peripherals | GPIO, I2C, SPI, PWM, ADC, UART |

## Repository layout

```
yahboom/          — main application (motors, servos, RC input, battery, buzzer)
drivers/
  input/          — SBUS RC receiver, STM32 quadrature decoder
  motor/          — H-bridge motor driver
  pwm/            — software PWM (STM32 timer + GPIO), PWM servo
boards/           — yahboom_ros_stm32f103 board definition
dts/              — devicetree bindings
zephyr/           — Zephyr module descriptor (module.yml)
west.yml          — west manifest (Zephyr version pinned per branch)
sdk-version.txt   — Zephyr SDK version used on this branch
activate.sh       — one-shot workspace setup script
Makefile          — build / flash shortcuts
```

## SDK compatibility

Each Zephyr version requires a specific SDK version — see the
[Zephyr SDK compatibility matrix](https://docs.google.com/spreadsheets/d/1wzGJLRuR6urTgnDFUqKk7pEB8O6vWu6Sxziw_KROxMA/edit?gid=789715970#gid=789715970).
The SDK version for this branch is in `sdk-version.txt`.

## Branches

Each branch pins a specific Zephyr version via `west.yml`:

| Branch | Zephyr |
|--------|--------|
| master | v3.5.0 |
| v3.7   | v3.7.x |

## Getting started

### First-time setup

```sh
git clone <repo> && cd yahboom
source activate.sh
```

`activate.sh` handles everything: creates a Python venv, installs west, writes
`.west/config`, runs `west update` to fetch Zephyr and modules, installs Zephyr
Python requirements, and downloads the SDK into `zephyr_os/sdk/`.

### Daily use

```sh
source activate.sh   # re-activates venv and sets env vars
make                 # build
make flash           # flash via ST-Link
```

### Switching Zephyr versions

```sh
git checkout <branch>
source activate.sh && west update
```

### Full clean (start over)

```sh
make distclean       # removes build/, zephyr_os/, .west/
source activate.sh   # re-fetches everything
```

## Build targets

| Target | Description |
|--------|-------------|
| `make` | Build the main application |
| `make mcuboot` | Build MCUboot bootloader |
| `make app-signed` | Build application with MCUboot signing |
| `make flash` | Flash via west (ST-Link) |
| `make flash-mcuboot` | Flash MCUboot bootloader |
| `make flash-app-signed` | Flash signed app at 0x08010000 |
| `make flash-app-signed-mcumgr` | Upload signed app via mcumgr (serial DFU) |
| `make clean` | Remove build directory |
| `make distclean` | Remove build + zephyr_os/ + .west/ |
