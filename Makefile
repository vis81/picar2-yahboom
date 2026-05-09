BOARD       = yahboom_ros_stm32f103
CONN_STRING = dev=/dev/ttyUSB0,baud=115200
APP_DIR     = yahboom

MCUMGR = ~/go/bin/mcumgr

# ZEPHYR_SDK_INSTALL_DIR and ZEPHYR_BASE are set by `source activate.sh`
# in the workspace root before running make.

.PHONY: all yahboom clean \
        sample_nvs mcuboot app-signed \
        flash flash-app-signed flash-mcuboot flash-app-signed-mcumgr \
        test_control check-env

check-env:
	@if ! command -v west > /dev/null 2>&1; then \
		echo ""; \
		echo "ERROR: 'west' not found. Activate a workspace first:"; \
		echo ""; \
		echo "  source ../ws-v3.5.0/activate.sh"; \
		echo "  source ./zephyr-env.sh"; \
		echo ""; \
		exit 1; \
	fi
	@if [ -z "$$ZEPHYR_BASE" ]; then \
		echo ""; \
		echo "ERROR: ZEPHYR_BASE is not set. Activate a workspace first:"; \
		echo ""; \
		echo "  source ../ws-v3.5.0/activate.sh"; \
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
