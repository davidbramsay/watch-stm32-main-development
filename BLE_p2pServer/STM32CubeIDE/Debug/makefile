################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/subdir.mk
-include Middlewares/STM32_WPAN/subdir.mk
-include Drivers/STM32WBxx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/P-NUCLEO-WB55.Nucleo/subdir.mk
-include Application/User/Startup/subdir.mk
-include Application/User/STM32_WPAN/Target/subdir.mk
-include Application/User/STM32_WPAN/App/subdir.mk
-include Application/User/Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := BLE_p2pServer
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BLE_p2pServer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BLE_p2pServer.list \

OBJCOPY_HEX += \
BLE_p2pServer.hex \


# All Target
all: main-build

# Main-build Target
main-build: BLE_p2pServer.elf secondary-outputs

# Tool invocations
BLE_p2pServer.elf: $(OBJS) $(USER_OBJS) Z:\Documents\WB_freertos_ble_integration_test\BLE_p2pServer\STM32CubeIDE\STM32WB55RGVX_FLASH.ld
	arm-none-eabi-gcc -o "BLE_p2pServer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"Z:\Documents\WB_freertos_ble_integration_test\BLE_p2pServer\STM32CubeIDE\STM32WB55RGVX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BLE_p2pServer.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BLE_p2pServer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BLE_p2pServer.list"
	@echo 'Finished building: $@'
	@echo ' '

BLE_p2pServer.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "BLE_p2pServer.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
