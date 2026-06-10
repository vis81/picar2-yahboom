.DEFAULT_GOAL := all

BOARD       = yahboom_ros_stm32f103
CONN_STRING = dev=/dev/ttyYahboom0,baud=115200
UROS_DEV    = /dev/ttyYahboom0
UROS_BAUD   = 921600
APP_DIR     = yahboom
MCUMGR      = ~/go/bin/mcumgr

.PHONY: help all $(APP_DIR) tests test-pwm-servo test-stm32-sw test-sbus test-shell-echo \
        test-timesync build-timesync \
        clean distclean \
        mcuboot app-signed \
        flash flash-app-signed flash-mcuboot flash-app-signed-mcumgr \
        check-env

TIMESYNC_SRC = tests/integration/timesync_test.cpp
TIMESYNC_BIN = tests/integration/timesync_test
TIMESYNC_PORT ?= /dev/ttyYahboom0

# ── help ─────────────────────────────────────────────────────────────────────

help:
	@echo ""
	@echo "First-time setup:"
	@echo "  source activate.sh              — create venv, init west, fetch deps, download SDK"
	@echo ""
	@echo "Build:"
	@echo "  make                            — build the main application"
	@echo "  make test-pwm-servo             — build and run PWM servo unit tests (native_sim)"
	@echo "  make test-sbus                  — build and run SBUS receiver unit tests (native_sim)"
	@echo "  make test-stm32-sw              — build STM32 SW-PWM unit tests (build only)"
	@echo "  make tests                      — run all: test-pwm-servo + test-sbus + test-stm32-sw"
	@echo "  make test-shell-echo            — shell UART echo integrity test (hardware, /dev/ttyYahboom1)"
	@echo "  make test-timesync              — C++ timesync accuracy benchmark (hardware, TIMESYNC_PORT=/dev/ttyYahboom0)"
	@echo "  make build-timesync             — build the timesync benchmark binary only"
	@echo "  make mcuboot                    — build MCUboot bootloader"
	@echo "  make app-signed                 — build application with MCUboot signing"
	@echo ""
	@echo "Flash:"
	@echo "  make flash                      — flash via west (ST-Link)"
	@echo "  make flash-mcuboot              — flash MCUboot bootloader"
	@echo "  make flash-app-signed           — flash signed app at 0x08010000"
	@echo "  make flash-app-signed-mcumgr    — upload signed app via mcumgr (serial DFU)"
	@echo ""
	@echo "Clean:"
	@echo "  make clean                      — remove build directory"
	@echo "  make distclean                  — remove build + zephyr_os/ + .west/ + .venv/"
	@echo ""
	@echo "Switching Zephyr versions:"
	@echo "  git checkout <branch>"
	@echo "  source activate.sh && west update"
	@echo ""

# ── environment check ─────────────────────────────────────────────────────────

check-env:
	@if ! command -v west > /dev/null 2>&1; then \
		echo ""; \
		echo "ERROR: 'west' not found.  Run:  source activate.sh"; \
		echo ""; \
		exit 1; \
	fi
	@if [ -z "$$ZEPHYR_BASE" ]; then \
		echo ""; \
		echo "ERROR: ZEPHYR_BASE not set.  Run:  source activate.sh"; \
		echo ""; \
		exit 1; \
	fi

# ── build targets ─────────────────────────────────────────────────────────────

all: $(APP_DIR)

test-pwm-servo: check-env
	west build -p auto -s tests/drivers/pwm/pwm_servo -b native_sim -d build/tests/pwm_servo
	./build/tests/pwm_servo/zephyr/zephyr.exe

test-sbus: check-env
	west build -p auto -s tests/drivers/input/sbus_receiver -b native_sim -d build/tests/sbus_receiver
	./build/tests/sbus_receiver/zephyr/zephyr.exe

test-stm32-sw: check-env
	west build -p auto -s tests/drivers/pwm/pwm_stm32_sw -b $(BOARD) -d build/tests/pwm_stm32_sw

tests: test-pwm-servo test-sbus test-stm32-sw

test-shell-echo:
	python3 tests/integration/test_shell_echo.py

build-timesync:
	g++ -O2 -std=c++17 -Wall -Wno-unused-result -o $(TIMESYNC_BIN) $(TIMESYNC_SRC)

test-timesync: build-timesync
	$(TIMESYNC_BIN) $(TIMESYNC_PORT)

clean:
	rm -rf build
	rm -f $(TIMESYNC_BIN)

distclean:
	rm -rf build zephyr_os .west .venv

$(APP_DIR): check-env
	west build -p auto -s $(APP_DIR) -b $(BOARD)

mcuboot: check-env
	west build -p \
		-s zephyr_os/bootloader/mcuboot/boot/zephyr \
		-d build/mcuboot \
		-b $(BOARD) \
		-- \
		-DCONFIG_BOOT_SIGNATURE_TYPE_NONE=y \
		-DCONFIG_MCUBOOT_SERIAL=y \
		-DCONFIG_SERIAL=n \
		-DCONFIG_BOOT_SERIAL_WAIT_FOR_DFU=y \
		-DCONFIG_BOOT_SERIAL_WAIT_FOR_DFU_TIMEOUT=1000 \
		-DCONFIG_SINGLE_APPLICATION_SLOT=y

app-signed: check-env
	west build -p \
		-s $(APP_DIR) \
		-d build/app-signed \
		-b $(BOARD) \
		-- \
		-DCONFIG_BOOTLOADER_MCUBOOT=y \
		-DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"zephyr_os/bootloader/mcuboot/root-rsa-2048.pem\"

flash: check-env
	west flash

flash-app-signed: check-env
	west flash -d build/app-signed/ --start-addr=0x08010000

flash-mcuboot: check-env
	west flash -d build/mcuboot/

flash-app-signed-mcumgr:
	echo "Press reset"
	$(MCUMGR) --conntype serial --connstring $(CONN_STRING) image upload build/app-signed/zephyr/zephyr.signed.bin
	$(MCUMGR) --conntype serial --connstring $(CONN_STRING) image list
	$(MCUMGR) --conntype serial --connstring $(CONN_STRING) reset
