################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/bemf_detection.c \
../src/main.c \
../src/motor_control.c \
../src/stm32f0xx_it.c \
../src/syscalls.c \
../src/system_stm32f0xx.c \
../src/uart_utils.c 

OBJS += \
./src/bemf_detection.o \
./src/main.o \
./src/motor_control.o \
./src/stm32f0xx_it.o \
./src/syscalls.o \
./src/system_stm32f0xx.o \
./src/uart_utils.o 

C_DEPS += \
./src/bemf_detection.d \
./src/main.d \
./src/motor_control.d \
./src/stm32f0xx_it.d \
./src/syscalls.d \
./src/system_stm32f0xx.d \
./src/uart_utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F0 -DSTM32F030K6Tx -DDEBUG -DSTM32F030x6 -DUSE_HAL_DRIVER -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/CMSIS/core" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/CMSIS/device" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/HAL_Driver/Inc/Legacy" -I"F:/WorkSpace/SW4STM32/stm32f030brkout_32qfp_hal_lib/HAL_Driver/Inc" -I"F:/WorkSpace/SW4STM32/F030_BLDCControl/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


