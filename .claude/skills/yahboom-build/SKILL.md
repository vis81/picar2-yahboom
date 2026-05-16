---
name: yahboom-build
description: >
  Build, flash, and troubleshoot the Yahboom ROS STM32F103 Zephyr firmware.
  Use this skill whenever the user is working in the yahboom workspace or mentions
  building, flashing, or debugging the Yahboom robot firmware — even if they just say
  "make isn't working", "flash failed", "serial port not found", "west not found",
  "stm32flash error", or "I forgot to source something". Also use it for branch
  switching, Zephyr SDK/environment setup, and serial console questions in this project.
---

# PICAR-2 Build & Flash Workflow

## Workspace layout

```
~/PICAR2/picar2_ws/yahboom/    ← all commands run from here
  activate.sh                  ← sets up (or re-activates) the whole environment
  Makefile                     ← wraps all west commands
  yahboom/                     ← main application source
  drivers/                     ← custom Zephyr drivers
  build/                       ← build output (gitignored)
  zephyr_os/                   ← fetched by west (gitignored)
    zephyr/                    ← Zephyr OS itself
    modules/hal/               ← HAL for the target platform
    sdk/                       ← Zephyr SDK (toolchain)
  sdk-version.txt              ← pinned Zephyr SDK version
  west.yml                     ← pinned Zephyr version (v3.7.0 on v3.7 branch)
```

Board: `yahboom_ros_stm32f103` (STM32F103RCT6, 256 KB flash, 48 KB RAM)

---

## Step 1: Environment — the most common source of pain

Every shell session needs the environment activated before any `make` or `west` command:

```sh
cd ~/PICAR2/picar2_ws/yahboom
source activate.sh
```

**First run** — downloads the Zephyr SDK, runs `west update`, installs Python deps (takes a few minutes).  
**Subsequent runs** — fast: re-activates the venv and sets `ZEPHYR_BASE` + `ZEPHYR_SDK_INSTALL_DIR`.

### Diagnosing "forgot to source" errors

| Error message | Cause | Fix |
|---|---|---|
| `west: command not found` | venv not active | `source activate.sh` |
| `ZEPHYR_BASE is not set` | env vars missing | `source activate.sh` |
| `No module named west` | venv not active | `source activate.sh` |
| `CMake Error: ZEPHYR_BASE` | env vars missing | `source activate.sh` |
| toolchain / `arm-zephyr-eabi` not found | SDK not on PATH | `source activate.sh` |

**After switching git branches**, always re-fetch Zephyr modules:
```sh
git checkout <branch>
source activate.sh && west update
```

---

## Step 2: Build

```sh
make          # build the main application (incremental)
```

To force a clean rebuild:
```sh
make clean && make
```

Full nuclear reset (re-fetches Zephyr + SDK):
```sh
make distclean
source activate.sh
```

Build output lives in `build/zephyr/` — `zephyr.bin` and `zephyr.hex` are the flash images.

---

## Serial ports

The board exposes two independent serial ports via USB-serial adapters:

| Port | MCU UART | Baud | Purpose |
|---|---|---|---|
| `/dev/ttyYahboom0` | USART1 | 921600 (ROM bootloader flash) / 460800 (app) | stm32flash entry; custom Pi↔STM32 binary protocol when app is running |
| `/dev/ttyYahboom1` | USART3 | 921600 | Zephyr shell console |

They are independent — you can keep the shell console open while flashing.

**Install the udev rule** (one-time, if the symlinks don't appear):
```sh
sudo cp ~/PICAR2/picar2_ws/yahboom/etc/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Unplug and replug both USB-serial adapters
```

If either symlink is missing after that, check `ls /dev/ttyUSB*` and compare
`udevadm info` output against the VID/PID in the rules file.

---

## Step 3: Flash

Flashing uses `stm32flash` talking to the STM32 ROM bootloader over USART1 (`/dev/ttyYahboom0`).

### Entering the ROM bootloader

**Option A — software (preferred, no hardware changes needed):**

Send `sys bootloader\n` to `/dev/ttyYahboom1` at 921600 baud, then immediately run `make flash`.
Claude can do this autonomously:

```sh
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyYahboom1', 921600, timeout=1)
s.write(b'sys bootloader\r\n')
time.sleep(0.3)
s.close()
"
make flash
```

**Option B — hardware BOOT0 button** (for a bricked/unresponsive board):
1. Press and hold the BOOT0 button
2. Press and hold reset
3. Release reset — the MCU now boots into the ROM bootloader
4. Release the BOOT0 button
5. Run `make flash` — the program starts automatically after flashing

### Flashing

```sh
make flash    # runs: west flash --runner stm32flash --device /dev/ttyYahboom0 --baud-rate 921600
```

### Common flash failures

| Symptom | Likely cause | Fix |
|---|---|---|
| `/dev/ttyYahboom0: No such file or directory` | USB-serial adapter not plugged in, or udev rule missing | Plug in the CH340 adapter; install udev rule (see below) |
| `Permission denied: /dev/ttyYahboom0` | User not in `dialout` group | `sudo usermod -aG dialout $USER` then log out/in |
| `stm32flash: failed to init device` | Board not in bootloader mode | Enter bootloader first (`sys bootloader` or BOOT0 pin) |
| `Failed to read device signature` | Wrong baud / bad connection | Check cable; ensure BOOT0 is actually high |
| `verify FAILED` | Flash write error | Retry `make flash`; check cable quality |

If flashing fails or stm32flash can't initialize, check whether `/dev/ttyYahboom0` is held open by another process — that will block stm32flash even if the board is in bootloader mode:

```sh
lsof /dev/ttyYahboom0
# or
fuser /dev/ttyYahboom0
```

Kill or close whatever is holding it, then retry `make flash`.

---

## Step 4: Serial console

The Zephyr shell runs on `/dev/ttyYahboom1` (USART3, 921600 8N1). This is independent
from the flash port, so you can leave it open while running `make flash`.

```sh
picocom -b 921600 /dev/ttyYahboom1
# or
screen /dev/ttyYahboom1 921600
```

Useful shell commands once connected:

| Command | What it does |
|---|---|
| `sys version` | Print Zephyr + app version |
| `sys uptime` | Time since boot |
| `sys bootloader` | Jump to ROM bootloader (for flashing) |
| `sys reboot` | Soft reboot |
| `sys halt` | Enter STM32 STANDBY (~2 µA); needs power-cycle to wake |

---

## Quick reference: all make targets

| Target | What it does |
|---|---|
| `make` | Build main app |
| `make flash` | Flash via stm32flash (ROM bootloader) |
| `make clean` | Remove `build/` |
| `make distclean` | Remove `build/` + `zephyr_os/` + `.west/` |
| `make tests` | Run all unit tests (native_sim + STM32 build-only) |
| `make mcuboot` | Build MCUboot bootloader |
| `make app-signed` | Build app with MCUboot signing |
| `make flash-mcuboot` | Flash MCUboot bootloader |
| `make flash-app-signed` | Flash signed app at 0x08010000 |
| `make flash-app-signed-mcumgr` | DFU upload via mcumgr (serial) |

---

## Branch / Zephyr version

The active branch is `master`, pinned to Zephyr v3.7.0. Other branches are stale.

If you ever need to switch branches, run `source activate.sh && west update` afterwards —
mismatched Zephyr versions cause confusing build errors that look unrelated to the switch.
