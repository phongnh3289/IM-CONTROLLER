################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd 

LIB_SRCS += \
D:/INSTALL/controlSUITE/libs/math/IQmath/v15c/lib/IQmath_fpu32.lib \
D:/INSTALL/controlSUITE/libs/math/FPUfastRTS/V100/lib/rts2800_fpu32_fast_supplement.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_ADC_cal.asm \
D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm \
../DSP2833x_usDelay.asm 

CMD_UPPER_SRCS += \
../F28335_RAM_HVACI_Sensorless.CMD 

C_SRCS += \
D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c \
../HVACI_Sensorless-DevInit_F2833x.c \
../HVACI_Sensorless.c 

C_DEPS += \
./DSP2833x_GlobalVariableDefs.d \
./HVACI_Sensorless-DevInit_F2833x.d \
./HVACI_Sensorless.d 

OBJS += \
./DLOG4CHC.obj \
./DSP2833x_ADC_cal.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_usDelay.obj \
./HVACI_Sensorless-DevInit_F2833x.obj \
./HVACI_Sensorless.obj 

ASM_DEPS += \
./DLOG4CHC.d \
./DSP2833x_ADC_cal.d \
./DSP2833x_CodeStartBranch.d \
./DSP2833x_usDelay.d 

OBJS__QUOTED += \
"DLOG4CHC.obj" \
"DSP2833x_ADC_cal.obj" \
"DSP2833x_CodeStartBranch.obj" \
"DSP2833x_GlobalVariableDefs.obj" \
"DSP2833x_usDelay.obj" \
"HVACI_Sensorless-DevInit_F2833x.obj" \
"HVACI_Sensorless.obj" 

C_DEPS__QUOTED += \
"DSP2833x_GlobalVariableDefs.d" \
"HVACI_Sensorless-DevInit_F2833x.d" \
"HVACI_Sensorless.d" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_ADC_cal.asm" \
"D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm" \
"../DSP2833x_usDelay.asm" 

C_SRCS__QUOTED += \
"D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c" \
"../HVACI_Sensorless-DevInit_F2833x.c" \
"../HVACI_Sensorless.c" 


