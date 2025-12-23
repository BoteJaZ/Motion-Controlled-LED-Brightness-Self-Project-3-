################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
device/%.obj: ../device/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"F:/CCS_20.3.1.00005_win/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="F:/CCS_20.3.1.00005_win/Motion-Controlled LED Brightness (Self-Project 3)/motion-controlled_LED_Dimming_bitfield_cpu1" --include_path="C:/TI/C2000Ware_6_00_01_00/device_support/f2837xd/common/include/" --include_path="C:/TI/C2000Ware_6_00_01_00/device_support/f2837xd/headers/include/" --include_path="F:/CCS_20.3.1.00005_win/Motion-Controlled LED Brightness (Self-Project 3)/motion-controlled_LED_Dimming_bitfield_cpu1/device" --include_path="C:/TI/C2000Ware_6_00_01_00/driverlib/f2837xd/driverlib/" --include_path="F:/CCS_20.3.1.00005_win/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/include" --define=DEBUG --define=_DUAL_HEADERS --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


