################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/portable/GCC/ARM_CM3/port.c 

OBJS += \
./Source/portable/GCC/ARM_CM3/port.o 

C_DEPS += \
./Source/portable/GCC/ARM_CM3/port.d 


# Each subdirectory must supply rules for building sources it contributes
Source/portable/GCC/ARM_CM3/%.o Source/portable/GCC/ARM_CM3/%.su Source/portable/GCC/ARM_CM3/%.cyclo: ../Source/portable/GCC/ARM_CM3/%.c Source/portable/GCC/ARM_CM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Core/Src" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/include" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/portable/GCC/ARM_CM3" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Source-2f-portable-2f-GCC-2f-ARM_CM3

clean-Source-2f-portable-2f-GCC-2f-ARM_CM3:
	-$(RM) ./Source/portable/GCC/ARM_CM3/port.cyclo ./Source/portable/GCC/ARM_CM3/port.d ./Source/portable/GCC/ARM_CM3/port.o ./Source/portable/GCC/ARM_CM3/port.su

.PHONY: clean-Source-2f-portable-2f-GCC-2f-ARM_CM3

