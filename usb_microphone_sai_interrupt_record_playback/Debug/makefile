################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include xip/subdir.mk
-include utilities/subdir.mk
-include usb/phy/subdir.mk
-include usb/host/class/subdir.mk
-include usb/host/subdir.mk
-include usb/device/source/ehci/subdir.mk
-include usb/device/source/subdir.mk
-include usb/device/class/video/subdir.mk
-include usb/device/class/printer/subdir.mk
-include usb/device/class/phdc/subdir.mk
-include usb/device/class/msc/subdir.mk
-include usb/device/class/hid/subdir.mk
-include usb/device/class/dfu/subdir.mk
-include usb/device/class/cdc_rndis/subdir.mk
-include usb/device/class/cdc/subdir.mk
-include usb/device/class/ccid/subdir.mk
-include usb/device/class/audio/subdir.mk
-include usb/device/class/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/osa/subdir.mk
-include component/lists/subdir.mk
-include component/i2c/subdir.mk
-include codec/port/wm8960/subdir.mk
-include codec/subdir.mk
-include board/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := usb_microphone_sai_interrupt_record_playback
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: usb_microphone_sai_interrupt_record_playback.axf

# Tool invocations
usb_microphone_sai_interrupt_record_playback.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="usb_microphone_sai_interrupt_record_playback.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -T usb_microphone_sai_interrupt_record_playback_Debug.ld -o "usb_microphone_sai_interrupt_record_playback.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) usb_microphone_sai_interrupt_record_playback.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "usb_microphone_sai_interrupt_record_playback.axf"; # arm-none-eabi-objcopy -v -O binary "usb_microphone_sai_interrupt_record_playback.axf" "usb_microphone_sai_interrupt_record_playback.bin" ; # checksum -p MIMXRT1011xxxxx -d "usb_microphone_sai_interrupt_record_playback.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
