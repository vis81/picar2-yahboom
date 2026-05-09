# ── workspace manager ────────────────────────────────────────────────────────
ZEPHYR_VER   ?= v3.7.0
SDK_VER      ?= 0.17.0
WS_NAME      ?= $(ZEPHYR_VER)
TOOLCHAIN    ?= arm-zephyr-eabi
SDK_PLATFORM ?= linux-x86_64
MODULES      ?= cmsis hal_stm32 mcuboot

export TOOLCHAIN SDK_PLATFORM MODULES

# ── app build ────────────────────────────────────────────────────────────────
BOARD       = yahboom_ros_stm32f103
CONN_STRING = dev=/dev/ttyUSB0,baud=115200
APP_DIR     = yahboom
MCUMGR      = ~/go/bin/mcumgr

.PHONY: help all yahboom clean \
        sample_nvs mcuboot app-signed \
        flash flash-app-signed flash-mcuboot flash-app-signed-mcumgr \
        test_control check-env \
        new sdk sdk-list list clean-ws

# ── help ─────────────────────────────────────────────────────────────────────

help:
	@echo ""
	@echo "Workspace management:"
	@echo "  make new         [ZEPHYR_VER=v3.7.0] [SDK_VER=0.17.0] [WS_NAME=v3.7.0]"
	@echo "                   [MODULES='cmsis hal_stm32 mcuboot']"
	@echo "                   Create a new workspace in ws/<ver>/"
	@echo "  make sdk         [SDK_VER=0.17.0]   Download SDK into sdks/"
	@echo "  make sdk-list                        List installed SDKs"
	@echo "  make list                            List workspaces"
	@echo "  make clean-ws    [WS_NAME=v3.7.0]   Delete a workspace"
	@echo ""
	@echo "App build (requires activated workspace):"
	@echo "  make yahboom     Build the main application"
	@echo "  make mcuboot     Build MCUboot bootloader"
	@echo "  make app-signed  Build application with MCUboot signing"
	@echo "  make flash       Flash via west"
	@echo "  make clean       Remove build directory"
	@echo ""
	@echo "Activate a workspace first:"
	@echo "  source ws/$(WS_NAME)/activate.sh"
	@echo "  source ./zephyr-env.sh"
	@echo ""
	@echo "Examples:"
	@echo "  make new ZEPHYR_VER=v3.5.0 SDK_VER=0.16.9"
	@echo "  make new ZEPHYR_VER=v3.7.0 SDK_VER=0.17.0"
	@echo "  make sdk SDK_VER=0.17.0"
	@echo ""

# ── workspace manager targets ────────────────────────────────────────────────

new:
	./setup-workspace.sh new $(ZEPHYR_VER) $(SDK_VER) $(WS_NAME)

sdk:
	./setup-workspace.sh sdk download $(SDK_VER)

sdk-list:
	./setup-workspace.sh sdk list

list:
	./setup-workspace.sh list

clean-ws:
	@WS_PATH="$(CURDIR)/ws/$(WS_NAME)"; \
	if [ ! -d "$$WS_PATH" ]; then \
	    echo "Workspace not found: $$WS_PATH"; exit 1; \
	fi; \
	read -r -p "Delete $$WS_PATH? [y/N] " ans; \
	[ "$$ans" = "y" ] || [ "$$ans" = "Y" ] || { echo "Aborted."; exit 0; }; \
	rm -rf "$$WS_PATH" && echo "Deleted $$WS_PATH"

# ── app build targets ─────────────────────────────────────────────────────────

check-env:
	@if ! command -v west > /dev/null 2>&1; then \
		echo ""; \
		echo "ERROR: 'west' not found. Activate a workspace first:"; \
		echo ""; \
		echo "  source ./ws/$(WS_NAME)/activate.sh"; \
		echo "  source ./zephyr-env.sh"; \
		echo ""; \
		exit 1; \
	fi
	@if [ -z "$$ZEPHYR_BASE" ]; then \
		echo ""; \
		echo "ERROR: ZEPHYR_BASE is not set. Activate a workspace first:"; \
		echo ""; \
		echo "  source ./ws/$(WS_NAME)/activate.sh"; \
		echo "  source ./zephyr-env.sh"; \
		echo ""; \
		exit 1; \
	fi
	@if [ -z "$$ZEPHYR_EXTRA_MODULES" ]; then \
		echo ""; \
		echo "ERROR: ZEPHYR_EXTRA_MODULES is not set. Register the app module:"; \
		echo ""; \
		echo "  source ./zephyr-env.sh"; \
		echo ""; \
		exit 1; \
	fi

all: yahboom

clean:
	rm -rf build

yahboom: check-env
	west build -s $(APP_DIR) -b $(BOARD)

sample_nvs: check-env
	west build -b $(BOARD) ../zephyr/samples/subsys/nvs/

mcuboot: check-env
	west build -p \
		-s ../bootloader/mcuboot/boot/zephyr \
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
		-DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"bootloader/mcuboot/root-rsa-2048.pem\"

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

test_control: yahboom/test/test_control.cpp
	g++ -Wall -g -pthread $<  -lgtest_main  -lgtest -lpthread -o $@
