################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/provider_can/can_node.cc \
../src/provider_can/main.cc 

OBJS += \
./src/provider_can/can_node.o \
./src/provider_can/main.o 

CC_DEPS += \
./src/provider_can/can_node.d \
./src/provider_can/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/provider_can/%.o: ../src/provider_can/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


