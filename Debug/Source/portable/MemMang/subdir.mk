################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/portable/MemMang/heap_2.c 

OBJS += \
./Source/portable/MemMang/heap_2.o 

C_DEPS += \
./Source/portable/MemMang/heap_2.d 


# Each subdirectory must supply rules for building sources it contributes
Source/portable/MemMang/%.o Source/portable/MemMang/%.su Source/portable/MemMang/%.cyclo: ../Source/portable/MemMang/%.c Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Core/Src" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/include" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/portable/GCC/ARM_CM3" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source" -I"C:/Users/quocv/STM32CubeIDE/workspace_1.14.0/motor MERC2024/Source/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Source-2f-portable-2f-MemMang

clean-Source-2f-portable-2f-MemMang:
	-$(RM) ./Source/portable/MemMang/heap_2.cyclo ./Source/portable/MemMang/heap_2.d ./Source/portable/MemMang/heap_2.o ./Source/portable/MemMang/heap_2.su

.PHONY: clean-Source-2f-portable-2f-MemMang

