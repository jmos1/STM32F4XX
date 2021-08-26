################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/uart_demo.c 

OBJS += \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/uart_demo.o 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/uart_demo.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/jmorar567/Desktop/EmbeddedCode/NucleoF446/UART/drivers/Src" -I"C:/Users/jmorar567/Desktop/EmbeddedCode/NucleoF446/UART/drivers/Inc" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

