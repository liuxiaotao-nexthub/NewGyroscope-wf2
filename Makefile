# STM32F103C8 Makefile for VS Code
# 基于VisualGDB配置

# 工具链配置 - 需要根据您的实际安装路径修改
TOOLCHAIN_ROOT = C:/SysGCC/arm-eabi
CC = $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-objcopy
SIZE = $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-size

# BSP路径
BSP_ROOT = C:/Users/liuxi/AppData/Local/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.stm32

# 项目名称
PROJECT = NewGyroscope

# 构建目录
BUILD_DIR = VisualGDB/Debug

# MCU配置
MCU = -mthumb -mcpu=cortex-m3

# 编译选项
CFLAGS = $(MCU) \
	-ggdb -O0 \
	-DARM_MATH_CM3 \
	-Dflash_layout \
	-DSTM32F103C8 \
	-DSTM32F103xB \
	-DUSE_HAL_DRIVER \
	-DUSE_FREERTOS \
	-DDEBUG=1 \
	-ffunction-sections \
	-fdata-sections

# 包含路径
INCLUDES = \
	-I. \
	-Iinclude \
	-I$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Inc \
	-I$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Inc/Legacy \
	-I$(BSP_ROOT)/STM32F1xxxx/CMSIS_HAL/Core/Include \
	-I$(BSP_ROOT)/STM32F1xxxx/CMSIS_HAL/Device/ST/STM32F1xx/Include \
	-I$(BSP_ROOT)/STM32F1xxxx/CMSIS_HAL/Include \
	-I$(BSP_ROOT)/STM32F1xxxx/CMSIS_HAL/RTOS2/Include \
	-I$(BSP_ROOT)/FreeRTOS/Source/CMSIS_RTOS \
	-I$(BSP_ROOT)/FreeRTOS/Source/include \
	-I$(BSP_ROOT)/FreeRTOS/Source/portable/GCC/ARM_CM3

# 链接选项
LDFLAGS = $(MCU) \
	-Wl,-gc-sections \
	-T$(BSP_ROOT)/STM32F1xxxx/LinkerScripts/STM32F103C8_flash.lds \
	--specs=nano.specs \
	--specs=nosys.specs \
	-Wl,--no-warn-rwx-segments \
	-Wl,--start-group \
	-lm

# 用户源文件
USER_SOURCES = \
	src/can.c \
	src/can_task.c \
	src/fly_task.c \
	src/flybox.c \
	src/motor.c \
	src/NewGyroscope.c \
	src/system_stm32f1xx.c

# BSP源文件
BSP_SOURCES = \
	$(BSP_ROOT)/STM32F1xxxx/StartupFiles/startup_stm32f103xb.S \
	$(BSP_ROOT)/FreeRTOS/Source/croutine.c \
	$(BSP_ROOT)/FreeRTOS/Source/event_groups.c \
	$(BSP_ROOT)/FreeRTOS/Source/list.c \
	$(BSP_ROOT)/FreeRTOS/Source/queue.c \
	$(BSP_ROOT)/FreeRTOS/Source/stream_buffer.c \
	$(BSP_ROOT)/FreeRTOS/Source/tasks.c \
	$(BSP_ROOT)/FreeRTOS/Source/timers.c \
	$(BSP_ROOT)/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
	$(BSP_ROOT)/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
	$(BSP_ROOT)/FreeRTOS/Source/portable/MemMang/heap_4.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
	$(BSP_ROOT)/STM32F1xxxx/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c

# 生成目标文件列表
USER_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(USER_SOURCES:.c=.o)))
USER_OBJECTS := $(USER_OBJECTS:.S=.o)

BSP_OBJECTS = $(BSP_SOURCES:.c=.o)
BSP_OBJECTS := $(BSP_OBJECTS:.S=.o)
BSP_OBJECTS := $(subst $(BSP_ROOT),$(BUILD_DIR)/__BSP_ROOT__,$(BSP_OBJECTS))

ALL_OBJECTS = $(USER_OBJECTS) $(BSP_OBJECTS)

# 默认目标
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin $(BUILD_DIR)/$(PROJECT).hex
	@echo "编译完成"
	$(SIZE) $(BUILD_DIR)/$(PROJECT).elf

# 编译用户源文件
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	@echo "编译: $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -MD -MP -MF $(BUILD_DIR)/$*.dep

# 编译BSP源文件
$(BUILD_DIR)/__BSP_ROOT__/%.o: $(BSP_ROOT)/%.c
	@mkdir -p $(dir $@)
	@echo "编译BSP: $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -MD -MP -MF $(@:.o=.dep)

$(BUILD_DIR)/__BSP_ROOT__/%.o: $(BSP_ROOT)/%.S
	@mkdir -p $(dir $@)
	@echo "编译启动文件: $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# 链接
$(BUILD_DIR)/$(PROJECT).elf: $(ALL_OBJECTS)
	@echo "链接: $@"
	$(CC) $(ALL_OBJECTS) $(LDFLAGS) -Wl,--end-group -o $@ -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map

# 生成bin文件
$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

# 生成hex文件
$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 清理
clean:
	rm -rf $(BUILD_DIR)/*.o $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.bin $(BUILD_DIR)/*.hex $(BUILD_DIR)/*.map $(BUILD_DIR)/*.dep
	rm -rf $(BUILD_DIR)/__BSP_ROOT__

# 烧录（需要配置OpenOCD或ST-Link工具）
flash: $(BUILD_DIR)/$(PROJECT).bin
	@echo "请使用VisualGDB或ST-Link Utility烧录 $(BUILD_DIR)/$(PROJECT).bin"

.PHONY: all clean flash
