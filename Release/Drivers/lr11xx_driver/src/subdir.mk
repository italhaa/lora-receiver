################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lr11xx_driver/src/lr11xx_bootloader.c \
../Drivers/lr11xx_driver/src/lr11xx_crypto_engine.c \
../Drivers/lr11xx_driver/src/lr11xx_driver_version.c \
../Drivers/lr11xx_driver/src/lr11xx_gnss.c \
../Drivers/lr11xx_driver/src/lr11xx_lr_fhss.c \
../Drivers/lr11xx_driver/src/lr11xx_radio.c \
../Drivers/lr11xx_driver/src/lr11xx_radio_timings.c \
../Drivers/lr11xx_driver/src/lr11xx_regmem.c \
../Drivers/lr11xx_driver/src/lr11xx_rttof.c \
../Drivers/lr11xx_driver/src/lr11xx_system.c \
../Drivers/lr11xx_driver/src/lr11xx_wifi.c 

OBJS += \
./Drivers/lr11xx_driver/src/lr11xx_bootloader.o \
./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.o \
./Drivers/lr11xx_driver/src/lr11xx_driver_version.o \
./Drivers/lr11xx_driver/src/lr11xx_gnss.o \
./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.o \
./Drivers/lr11xx_driver/src/lr11xx_radio.o \
./Drivers/lr11xx_driver/src/lr11xx_radio_timings.o \
./Drivers/lr11xx_driver/src/lr11xx_regmem.o \
./Drivers/lr11xx_driver/src/lr11xx_rttof.o \
./Drivers/lr11xx_driver/src/lr11xx_system.o \
./Drivers/lr11xx_driver/src/lr11xx_wifi.o 

C_DEPS += \
./Drivers/lr11xx_driver/src/lr11xx_bootloader.d \
./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.d \
./Drivers/lr11xx_driver/src/lr11xx_driver_version.d \
./Drivers/lr11xx_driver/src/lr11xx_gnss.d \
./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.d \
./Drivers/lr11xx_driver/src/lr11xx_radio.d \
./Drivers/lr11xx_driver/src/lr11xx_radio_timings.d \
./Drivers/lr11xx_driver/src/lr11xx_regmem.d \
./Drivers/lr11xx_driver/src/lr11xx_rttof.d \
./Drivers/lr11xx_driver/src/lr11xx_system.d \
./Drivers/lr11xx_driver/src/lr11xx_wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lr11xx_driver/src/%.o Drivers/lr11xx_driver/src/%.su Drivers/lr11xx_driver/src/%.cyclo: ../Drivers/lr11xx_driver/src/%.c Drivers/lr11xx_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lr11xx_driver-2f-src

clean-Drivers-2f-lr11xx_driver-2f-src:
	-$(RM) ./Drivers/lr11xx_driver/src/lr11xx_bootloader.cyclo ./Drivers/lr11xx_driver/src/lr11xx_bootloader.d ./Drivers/lr11xx_driver/src/lr11xx_bootloader.o ./Drivers/lr11xx_driver/src/lr11xx_bootloader.su ./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.cyclo ./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.d ./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.o ./Drivers/lr11xx_driver/src/lr11xx_crypto_engine.su ./Drivers/lr11xx_driver/src/lr11xx_driver_version.cyclo ./Drivers/lr11xx_driver/src/lr11xx_driver_version.d ./Drivers/lr11xx_driver/src/lr11xx_driver_version.o ./Drivers/lr11xx_driver/src/lr11xx_driver_version.su ./Drivers/lr11xx_driver/src/lr11xx_gnss.cyclo ./Drivers/lr11xx_driver/src/lr11xx_gnss.d ./Drivers/lr11xx_driver/src/lr11xx_gnss.o ./Drivers/lr11xx_driver/src/lr11xx_gnss.su ./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.cyclo ./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.d ./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.o ./Drivers/lr11xx_driver/src/lr11xx_lr_fhss.su ./Drivers/lr11xx_driver/src/lr11xx_radio.cyclo ./Drivers/lr11xx_driver/src/lr11xx_radio.d ./Drivers/lr11xx_driver/src/lr11xx_radio.o ./Drivers/lr11xx_driver/src/lr11xx_radio.su ./Drivers/lr11xx_driver/src/lr11xx_radio_timings.cyclo ./Drivers/lr11xx_driver/src/lr11xx_radio_timings.d ./Drivers/lr11xx_driver/src/lr11xx_radio_timings.o ./Drivers/lr11xx_driver/src/lr11xx_radio_timings.su ./Drivers/lr11xx_driver/src/lr11xx_regmem.cyclo ./Drivers/lr11xx_driver/src/lr11xx_regmem.d ./Drivers/lr11xx_driver/src/lr11xx_regmem.o ./Drivers/lr11xx_driver/src/lr11xx_regmem.su ./Drivers/lr11xx_driver/src/lr11xx_rttof.cyclo ./Drivers/lr11xx_driver/src/lr11xx_rttof.d ./Drivers/lr11xx_driver/src/lr11xx_rttof.o ./Drivers/lr11xx_driver/src/lr11xx_rttof.su ./Drivers/lr11xx_driver/src/lr11xx_system.cyclo ./Drivers/lr11xx_driver/src/lr11xx_system.d ./Drivers/lr11xx_driver/src/lr11xx_system.o ./Drivers/lr11xx_driver/src/lr11xx_system.su ./Drivers/lr11xx_driver/src/lr11xx_wifi.cyclo ./Drivers/lr11xx_driver/src/lr11xx_wifi.d ./Drivers/lr11xx_driver/src/lr11xx_wifi.o ./Drivers/lr11xx_driver/src/lr11xx_wifi.su

.PHONY: clean-Drivers-2f-lr11xx_driver-2f-src

