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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
prueba_interruptiontime.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
prueba_interruptiontime.list \

OBJCOPY_BIN += \
prueba_interruptiontime.bin \


# All Target
all: prueba_interruptiontime.elf secondary-outputs

# Tool invocations
prueba_interruptiontime.elf: $(OBJS) $(USER_OBJS) /home/julian/proyecto-final/proyecto_finall/deteccion_inhibidores/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "prueba_interruptiontime.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/julian/proyecto-final/proyecto_finall/deteccion_inhibidores/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="prueba_interruptiontime.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

prueba_interruptiontime.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "prueba_interruptiontime.list"
	@echo 'Finished building: $@'
	@echo ' '

prueba_interruptiontime.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "prueba_interruptiontime.bin"
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
