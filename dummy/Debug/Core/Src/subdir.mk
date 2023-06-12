################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Endeffector.c \
../Core/Src/JoyStick.c \
../Core/Src/KalmanFilterV2.c \
../Core/Src/ModBusRTU.c \
../Core/Src/PIDController.c \
../Core/Src/QuinticTrajectory.c \
../Core/Src/ReadEncoderV2.c \
../Core/Src/Storage.c \
../Core/Src/Trajectory.c \
../Core/Src/TrayLocalization.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Endeffector.o \
./Core/Src/JoyStick.o \
./Core/Src/KalmanFilterV2.o \
./Core/Src/ModBusRTU.o \
./Core/Src/PIDController.o \
./Core/Src/QuinticTrajectory.o \
./Core/Src/ReadEncoderV2.o \
./Core/Src/Storage.o \
./Core/Src/Trajectory.o \
./Core/Src/TrayLocalization.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Endeffector.d \
./Core/Src/JoyStick.d \
./Core/Src/KalmanFilterV2.d \
./Core/Src/ModBusRTU.d \
./Core/Src/PIDController.d \
./Core/Src/QuinticTrajectory.d \
./Core/Src/ReadEncoderV2.d \
./Core/Src/Storage.d \
./Core/Src/Trajectory.d \
./Core/Src/TrayLocalization.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tanawatp/Desktop/65_02/FRA262_ProjectModule/Github/ProjectModuleV2/dummy/DSP/ComputeLibrary/Include" -I"C:/Users/tanawatp/Desktop/65_02/FRA262_ProjectModule/Github/ProjectModuleV2/dummy/DSP/Include" -I"C:/Users/tanawatp/Desktop/65_02/FRA262_ProjectModule/Github/ProjectModuleV2/dummy/DSP/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Endeffector.cyclo ./Core/Src/Endeffector.d ./Core/Src/Endeffector.o ./Core/Src/Endeffector.su ./Core/Src/JoyStick.cyclo ./Core/Src/JoyStick.d ./Core/Src/JoyStick.o ./Core/Src/JoyStick.su ./Core/Src/KalmanFilterV2.cyclo ./Core/Src/KalmanFilterV2.d ./Core/Src/KalmanFilterV2.o ./Core/Src/KalmanFilterV2.su ./Core/Src/ModBusRTU.cyclo ./Core/Src/ModBusRTU.d ./Core/Src/ModBusRTU.o ./Core/Src/ModBusRTU.su ./Core/Src/PIDController.cyclo ./Core/Src/PIDController.d ./Core/Src/PIDController.o ./Core/Src/PIDController.su ./Core/Src/QuinticTrajectory.cyclo ./Core/Src/QuinticTrajectory.d ./Core/Src/QuinticTrajectory.o ./Core/Src/QuinticTrajectory.su ./Core/Src/ReadEncoderV2.cyclo ./Core/Src/ReadEncoderV2.d ./Core/Src/ReadEncoderV2.o ./Core/Src/ReadEncoderV2.su ./Core/Src/Storage.cyclo ./Core/Src/Storage.d ./Core/Src/Storage.o ./Core/Src/Storage.su ./Core/Src/Trajectory.cyclo ./Core/Src/Trajectory.d ./Core/Src/Trajectory.o ./Core/Src/Trajectory.su ./Core/Src/TrayLocalization.cyclo ./Core/Src/TrayLocalization.d ./Core/Src/TrayLocalization.o ./Core/Src/TrayLocalization.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

