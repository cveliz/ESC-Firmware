################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: F030_BLDCControl.elf

dependents:
	-cd F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/Release && $(MAKE) all

# Tool invocations
F030_BLDCControl.elf: $(OBJS) $(USER_OBJS) F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/Release/libstm32f030brkout_32qfp_hal_lib.a F:\WorkSpace\SW4STM32\F030_BLDCControl/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -L"F:\WorkSpace\SW4STM32\stm32f030brkout_32qfp_hal_lib\Release" -T"F:/WorkSpace/SW4STM32/F030_BLDCControl/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "F030_BLDCControl.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "F030_BLDCControl.elf" "F030_BLDCControl.bin"
	arm-none-eabi-size -B "F030_BLDCControl.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/Release/libstm32f030brkout_32qfp_hal_lib.a:

-include ../makefile.targets
