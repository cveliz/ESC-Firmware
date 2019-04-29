################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/CMSIS/core" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/CMSIS/device" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/HAL_Driver/Inc/Legacy" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/HAL_Driver/Inc" -I"F:/WorkSpace/SW4STM32/F030_BLDCControl/inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


