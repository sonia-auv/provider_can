################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/provider_can/devices/bottom_light.cc \
../src/provider_can/devices/can_device.cc 

OBJS += \
./src/provider_can/devices/bottom_light.o \
./src/provider_can/devices/can_device.o 

CC_DEPS += \
./src/provider_can/devices/bottom_light.d \
./src/provider_can/devices/can_device.d 


# Each subdirectory must supply rules for building sources it contributes
src/provider_can/devices/%.o: ../src/provider_can/devices/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


