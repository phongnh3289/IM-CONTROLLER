################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_6.4.6

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"D:/INSTALL/controlSUITE/libs/math/FPUfastRTS/V100/lib/rts2800_fpu32_fast_supplement.lib" \
-lrts2800_fpu32.lib \
"D:/INSTALL/controlSUITE/libs/math/IQmath/v15c/lib/IQmath_fpu32.lib" \
"./DLOG4CHC.obj" \
"./DSP2833x_ADC_cal.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_usDelay.obj" \
"./HVACI_Sensorless-DevInit_F2833x.obj" \
"./HVACI_Sensorless.obj" \
"D:/INSTALL/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd" \
"../F28335_RAM_HVACI_Sensorless.CMD" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
HVACI_Sensorless_2833x.out \

EXE_OUTPUTS__QUOTED += \
"HVACI_Sensorless_2833x.out" \


# All Target
all: HVACI_Sensorless_2833x.out

# Tool invocations
HVACI_Sensorless_2833x.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_6.4.6/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 -z -m"HVACI_Sensorless_2833x.map" --stack_size=0x380 --warn_sections -i"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_6.4.6/lib" -i"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_6.4.6/include" -i"D:/OneDrive - Industrial University of HoChiMinh City/3_FIRMWARE/2018-2019/GIT/HVACI" --reread_libs --xml_link_info="HVACI_Sensorless_2833x_linkInfo.xml" --rom_model -o "HVACI_Sensorless_2833x.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "DLOG4CHC.obj" "DSP2833x_ADC_cal.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_usDelay.obj" "HVACI_Sensorless-DevInit_F2833x.obj" "HVACI_Sensorless.obj" 
	-$(RM) "DSP2833x_GlobalVariableDefs.d" "HVACI_Sensorless-DevInit_F2833x.d" "HVACI_Sensorless.d" 
	-$(RM) "DLOG4CHC.d" "DSP2833x_ADC_cal.d" "DSP2833x_CodeStartBranch.d" "DSP2833x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

