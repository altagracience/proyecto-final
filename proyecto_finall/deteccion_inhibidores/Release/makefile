################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
spi_test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
spi_test.list \

OBJCOPY_BIN += \
spi_test.bin \


# All Target
all: spi_test.elf secondary-outputs

# Tool invocations
spi_test.elf: $(OBJS) $(USER_OBJS) C:\Users\teto_\Documents\FACULTAD_2020\Proyecto\ final\prueba_SPI\spi_test\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "spi_test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\teto_\Documents\FACULTAD_2020\Proyecto final\prueba_SPI\spi_test\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="spi_test.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

spi_test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "spi_test.list"
	@echo 'Finished building: $@'
	@echo ' '

spi_test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "spi_test.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
