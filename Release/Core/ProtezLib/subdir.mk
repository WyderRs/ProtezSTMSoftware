################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/ProtezLib/ProtezHandControl.cpp \
../Core/ProtezLib/ProtezHandUsbProtocol.cpp 

OBJS += \
./Core/ProtezLib/ProtezHandControl.o \
./Core/ProtezLib/ProtezHandUsbProtocol.o 

CPP_DEPS += \
./Core/ProtezLib/ProtezHandControl.d \
./Core/ProtezLib/ProtezHandUsbProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ProtezLib/%.o Core/ProtezLib/%.su Core/ProtezLib/%.cyclo: ../Core/ProtezLib/%.cpp Core/ProtezLib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-ProtezLib

clean-Core-2f-ProtezLib:
	-$(RM) ./Core/ProtezLib/ProtezHandControl.cyclo ./Core/ProtezLib/ProtezHandControl.d ./Core/ProtezLib/ProtezHandControl.o ./Core/ProtezLib/ProtezHandControl.su ./Core/ProtezLib/ProtezHandUsbProtocol.cyclo ./Core/ProtezLib/ProtezHandUsbProtocol.d ./Core/ProtezLib/ProtezHandUsbProtocol.o ./Core/ProtezLib/ProtezHandUsbProtocol.su

.PHONY: clean-Core-2f-ProtezLib

