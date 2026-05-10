BOARD       = yahboom_ros_stm32f103
CONN_STRING = dev=/dev/ttyUSB0,baud=115200
APP_DIR     = yahboom
MCUMGR      = ~/go/bin/mcumgr

.PHONY: help all $(APP_DIR) clean distclean \
        mcuboot app-signed \
        flash flash-app-signed flash-mcuboot flash-app-signed-mcumgr \
        check-env

# ── help ─────────────────────────────────────────────────────────────────────

help:
	@echo ""
	@echo "First-time setup:"
	@echo "  source activate.sh              — create venv, init west, download SDK"
	@echo "  west update                     — fetch/update Zephyr and modules"
	@echo ""
	@echo "Build:"
	@echo "  make                            — build the main application"
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
	@echo "  make distclean                  — remove build + zephyr_os/ + .west/"
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

clean:
	rm -rf build

distclean:
	rm -rf build zephyr_os .west

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
