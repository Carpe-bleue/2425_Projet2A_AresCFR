################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.c \
../lvgl/src/extra/libs/fsdrv/lv_fs_posix.c \
../lvgl/src/extra/libs/fsdrv/lv_fs_stdio.c \
../lvgl/src/extra/libs/fsdrv/lv_fs_win32.c 

OBJS += \
./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.o \
./lvgl/src/extra/libs/fsdrv/lv_fs_posix.o \
./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.o \
./lvgl/src/extra/libs/fsdrv/lv_fs_win32.o 

C_DEPS += \
./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.d \
./lvgl/src/extra/libs/fsdrv/lv_fs_posix.d \
./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.d \
./lvgl/src/extra/libs/fsdrv/lv_fs_win32.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/src/extra/libs/fsdrv/%.o lvgl/src/extra/libs/fsdrv/%.su lvgl/src/extra/libs/fsdrv/%.cyclo: ../lvgl/src/extra/libs/fsdrv/%.c lvgl/src/extra/libs/fsdrv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-src-2f-extra-2f-libs-2f-fsdrv

clean-lvgl-2f-src-2f-extra-2f-libs-2f-fsdrv:
	-$(RM) ./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.cyclo ./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.d ./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.o ./lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.su ./lvgl/src/extra/libs/fsdrv/lv_fs_posix.cyclo ./lvgl/src/extra/libs/fsdrv/lv_fs_posix.d ./lvgl/src/extra/libs/fsdrv/lv_fs_posix.o ./lvgl/src/extra/libs/fsdrv/lv_fs_posix.su ./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.cyclo ./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.d ./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.o ./lvgl/src/extra/libs/fsdrv/lv_fs_stdio.su ./lvgl/src/extra/libs/fsdrv/lv_fs_win32.cyclo ./lvgl/src/extra/libs/fsdrv/lv_fs_win32.d ./lvgl/src/extra/libs/fsdrv/lv_fs_win32.o ./lvgl/src/extra/libs/fsdrv/lv_fs_win32.su

.PHONY: clean-lvgl-2f-src-2f-extra-2f-libs-2f-fsdrv

