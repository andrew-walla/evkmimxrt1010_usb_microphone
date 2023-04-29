################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../usb/device/class/dfu/usb_device_dfu.c 

C_DEPS += \
./usb/device/class/dfu/usb_device_dfu.d 

OBJS += \
./usb/device/class/dfu/usb_device_dfu.o 


# Each subdirectory must supply rules for building sources it contributes
usb/device/class/dfu/%.o: ../usb/device/class/dfu/%.c usb/device/class/dfu/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -DSDK_DEBUGCONSOLE=1 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DSDK_I2C_BASED_COMPONENT_USED=1 -DBOARD_USE_CODEC=1 -DCODEC_WM8960_ENABLE -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_BAREMETAL -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\drivers" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port\wm8960" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\i2c" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\utilities" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\uart" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\lists" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\xip" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\CMSIS" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\osa" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\audio" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\output\source\device\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\ccid" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\cdc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\cdc_rndis" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\source" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\dfu" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\source\ehci" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\hid" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\msc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\phdc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\printer" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\output\source\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\video" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\host\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\host" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\phy" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\source\generated" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\drivers" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port\wm8960" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\i2c" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\utilities" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\uart" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\lists" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\xip" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\CMSIS" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\osa" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\source" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\board" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\evkmimxrt1010\driver_examples\sai\interrupt_record_playback" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-usb-2f-device-2f-class-2f-dfu

clean-usb-2f-device-2f-class-2f-dfu:
	-$(RM) ./usb/device/class/dfu/usb_device_dfu.d ./usb/device/class/dfu/usb_device_dfu.o

.PHONY: clean-usb-2f-device-2f-class-2f-dfu

