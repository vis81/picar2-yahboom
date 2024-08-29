BOARD=yahboom_ros_stm32f103
CONN_STRING=dev=/dev/ttyUSB0,baud=115200
#CONN_STRING=dev=/dev/ttyUSB0,baud=460800
#APP_DIR=samples/subsys/shell/devmem_load/
APP_DIR=yahboom

mcuboot:
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

#		-DCONFIG_BOOT_SERIAL_DETECT_DELAY=200 

app-signed:
	west build -p \
		-s $(APP_DIR) \
		-d build/app-signed \
		-b $(BOARD) \
		-- \
		-DCONFIG_BOOTLOADER_MCUBOOT=y \
		-DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"bootloader/mcuboot/root-rsa-2048.pem\"

flash-app-signed:
	west flash -d build/app-signed/ --start-addr=0x08010000

flash-mcuboot:
	west flash -d build/mcuboot/

flash-app-signed-mcumgr:
	echo "Press reset"
	~/go/bin/mcumgr --conntype serial --connstring $(CONN_STRING) image upload build/app-signed/zephyr/zephyr.signed.bin
	~/go/bin/mcumgr --conntype serial --connstring $(CONN_STRING) image list
	~/go/bin/mcumgr --conntype serial --connstring $(CONN_STRING) reset

