################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/costmap_2d.cpp \
../src/costmap_math.cpp \
../src/image_loader.cpp \
../src/raytrace.cpp 

CPP_DEPS += \
./src/costmap_2d.d \
./src/costmap_math.d \
./src/image_loader.d \
./src/raytrace.d 

OBJS += \
./src/costmap_2d.o \
./src/costmap_math.o \
./src/image_loader.o \
./src/raytrace.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/opencv4 -I/home/hankm/binary_ws/raytrace/include -include/home/hankm/binary_ws/raytrace/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/costmap_2d.d ./src/costmap_2d.o ./src/costmap_math.d ./src/costmap_math.o ./src/image_loader.d ./src/image_loader.o ./src/raytrace.d ./src/raytrace.o

.PHONY: clean-src

