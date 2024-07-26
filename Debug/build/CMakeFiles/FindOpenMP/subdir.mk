################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.cpp \
../build/CMakeFiles/FindOpenMP/OpenMPTryFlag.cpp 

C_SRCS += \
../build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.c \
../build/CMakeFiles/FindOpenMP/OpenMPTryFlag.c 

CPP_DEPS += \
./build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.d \
./build/CMakeFiles/FindOpenMP/OpenMPTryFlag.d 

C_DEPS += \
./build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.d \
./build/CMakeFiles/FindOpenMP/OpenMPTryFlag.d 

OBJS += \
./build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.o \
./build/CMakeFiles/FindOpenMP/OpenMPTryFlag.o 


# Each subdirectory must supply rules for building sources it contributes
build/CMakeFiles/FindOpenMP/%.o: ../build/CMakeFiles/FindOpenMP/%.c build/CMakeFiles/FindOpenMP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/hankm/binary_ws/raytrace/include -include/home/hankm/binary_ws/raytrace/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

build/CMakeFiles/FindOpenMP/%.o: ../build/CMakeFiles/FindOpenMP/%.cpp build/CMakeFiles/FindOpenMP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/opencv4 -I/home/hankm/binary_ws/raytrace/include -include/home/hankm/binary_ws/raytrace/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-build-2f-CMakeFiles-2f-FindOpenMP

clean-build-2f-CMakeFiles-2f-FindOpenMP:
	-$(RM) ./build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.d ./build/CMakeFiles/FindOpenMP/OpenMPCheckVersion.o ./build/CMakeFiles/FindOpenMP/OpenMPTryFlag.d ./build/CMakeFiles/FindOpenMP/OpenMPTryFlag.o

.PHONY: clean-build-2f-CMakeFiles-2f-FindOpenMP

