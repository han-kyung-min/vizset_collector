################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../example/raytrace_run.cpp 

CPP_DEPS += \
./example/raytrace_run.d 

OBJS += \
./example/raytrace_run.o 


# Each subdirectory must supply rules for building sources it contributes
example/%.o: ../example/%.cpp example/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/opencv4 -I/home/hankm/binary_ws/raytrace/include -include/home/hankm/binary_ws/raytrace/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-example

clean-example:
	-$(RM) ./example/raytrace_run.d ./example/raytrace_run.o

.PHONY: clean-example

