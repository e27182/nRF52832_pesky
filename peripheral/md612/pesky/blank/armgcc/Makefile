PROJECT_NAME     := md612_pesky
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := md612_gcc_nrf52.ld
# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52.c \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/../../common/timestamping.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/uart/retarget.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/uart/retarget.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/libraries/twi/app_twi.c \
  $(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/drivers_nrf/rtc/nrf_drv_rtc.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/nRF5/log_nRF5.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/eMPL/inv_mpu.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/eMPL/inv_mpu_dmp_motion_driver.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/mpl.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/data_builder.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/storage_manager.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/start_manager.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/results_holder.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/message_layer.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/ml_math_func.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite/hal_outputs.c \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/eMPL-hal/eMPL_outputs.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/config/md612_pesky \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  ../config \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/examples/bsp \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../../common \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/include \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/nRF5 \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/driver/eMPL \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/eMPL-hal \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mpl \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mllite \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/drivers_nrf/rtc \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \
  $(PROJ_DIR)/../../external/motion_driver_6.12/core/mpl/liblibmplmpu.a \

# C flags common to all targets
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF52
CFLAGS += -DNRF52832
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DMPU9250
CFLAGS += -DEMPL
CFLAGS += -DUSE_DMP
CFLAGS += -DDEBUG
CFLAGS += -DNRF_LOG_BACKEND_SERIAL_USES_UART
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -O3 -g3 #-Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# generate dependency output file
CFLAGS += -MP -MD

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DMPU9250
ASMFLAGS += -DEMPL
ASMFLAGS += -DUSE_DMP
ASMFLAGS += -DDEBUG
ASMFLAGS += -DNRF_LOG_BACKEND_SERIAL_USES_UART

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys







.PHONY: $(TARGETS) default all clean help flash 
# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common
$(foreach target, $(TARGETS), $(call define_target, $(target)))
-include $(foreach target, $(TARGETS), $($(target)_dependencies))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52