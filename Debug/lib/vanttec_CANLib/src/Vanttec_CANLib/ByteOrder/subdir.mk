################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.cpp 

OBJS += \
./lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.o 

CPP_DEPS += \
./lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.d 


# Each subdirectory must supply rules for building sources it contributes
lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/%.o lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/%.su: ../lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/%.cpp lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../lib/vanttec_CANLib/src/Vanttec_CANLib -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-vanttec_CANLib-2f-src-2f-Vanttec_CANLib-2f-ByteOrder

clean-lib-2f-vanttec_CANLib-2f-src-2f-Vanttec_CANLib-2f-ByteOrder:
	-$(RM) ./lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.d ./lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.o ./lib/vanttec_CANLib/src/Vanttec_CANLib/ByteOrder/inet.su

.PHONY: clean-lib-2f-vanttec_CANLib-2f-src-2f-Vanttec_CANLib-2f-ByteOrder

