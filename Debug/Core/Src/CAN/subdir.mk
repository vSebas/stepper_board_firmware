################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CAN/can_bus.c \
../Core/Src/CAN/can_bus_parser.c \
../Core/Src/CAN/can_msg_queue.c \
../Core/Src/CAN/can_tasks.c 

C_DEPS += \
./Core/Src/CAN/can_bus.d \
./Core/Src/CAN/can_bus_parser.d \
./Core/Src/CAN/can_msg_queue.d \
./Core/Src/CAN/can_tasks.d 

OBJS += \
./Core/Src/CAN/can_bus.o \
./Core/Src/CAN/can_bus_parser.o \
./Core/Src/CAN/can_msg_queue.o \
./Core/Src/CAN/can_tasks.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/CAN/%.o Core/Src/CAN/%.su: ../Core/Src/CAN/%.c Core/Src/CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../lib/vanttec_CANLib/src/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-CAN

clean-Core-2f-Src-2f-CAN:
	-$(RM) ./Core/Src/CAN/can_bus.d ./Core/Src/CAN/can_bus.o ./Core/Src/CAN/can_bus.su ./Core/Src/CAN/can_bus_parser.d ./Core/Src/CAN/can_bus_parser.o ./Core/Src/CAN/can_bus_parser.su ./Core/Src/CAN/can_msg_queue.d ./Core/Src/CAN/can_msg_queue.o ./Core/Src/CAN/can_msg_queue.su ./Core/Src/CAN/can_tasks.d ./Core/Src/CAN/can_tasks.o ./Core/Src/CAN/can_tasks.su

.PHONY: clean-Core-2f-Src-2f-CAN

