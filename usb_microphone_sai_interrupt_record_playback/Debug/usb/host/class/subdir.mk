################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../usb/host/class/usb_host_audio.c \
../usb/host/class/usb_host_cdc.c \
../usb/host/class/usb_host_cdc_rndis.c \
../usb/host/class/usb_host_hid.c \
../usb/host/class/usb_host_hub.c \
../usb/host/class/usb_host_hub_app.c \
../usb/host/class/usb_host_msd.c \
../usb/host/class/usb_host_msd_ufi.c \
../usb/host/class/usb_host_phdc.c \
../usb/host/class/usb_host_printer.c \
../usb/host/class/usb_host_video.c 

C_DEPS += \
./usb/host/class/usb_host_audio.d \
./usb/host/class/usb_host_cdc.d \
./usb/host/class/usb_host_cdc_rndis.d \
./usb/host/class/usb_host_hid.d \
./usb/host/class/usb_host_hub.d \
./usb/host/class/usb_host_hub_app.d \
./usb/host/class/usb_host_msd.d \
./usb/host/class/usb_host_msd_ufi.d \
./usb/host/class/usb_host_phdc.d \
./usb/host/class/usb_host_printer.d \
./usb/host/class/usb_host_video.d 

OBJS += \
./usb/host/class/usb_host_audio.o \
./usb/host/class/usb_host_cdc.o \
./usb/host/class/usb_host_cdc_rndis.o \
./usb/host/class/usb_host_hid.o \
./usb/host/class/usb_host_hub.o \
./usb/host/class/usb_host_hub_app.o \
./usb/host/class/usb_host_msd.o \
./usb/host/class/usb_host_msd_ufi.o \
./usb/host/class/usb_host_phdc.o \
./usb/host/class/usb_host_printer.o \
./usb/host/class/usb_host_video.o 


# Each subdirectory must supply rules for building sources it contributes
usb/host/class/%.o: ../usb/host/class/%.c usb/host/class/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -DSDK_DEBUGCONSOLE=1 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DSDK_I2C_BASED_COMPONENT_USED=1 -DBOARD_USE_CODEC=1 -DCODEC_WM8960_ENABLE -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_BAREMETAL -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\drivers" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port\wm8960" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\i2c" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\utilities" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\uart" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\lists" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\xip" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\CMSIS" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\osa" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\audio" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\output\source\device\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\ccid" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\cdc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\cdc_rndis" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\source" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\dfu" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\source\ehci" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\hid" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\msc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\phdc" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\printer" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\output\source\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\device\class\video" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\host\class" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\host" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\phy" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\source\generated" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\drivers" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port\wm8960" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\codec\port" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\i2c" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\utilities" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\device" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\uart" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\lists" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\xip" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\CMSIS" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\usb\include" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\component\osa" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\source" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\board" -I"C:\Users\Beardy McBeards\Documents\MCUXpressoIDE_11.7.0_9198\workspace\usb_microphone_sai_interrupt_record_playback\evkmimxrt1010\driver_examples\sai\interrupt_record_playback" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-usb-2f-host-2f-class

clean-usb-2f-host-2f-class:
	-$(RM) ./usb/host/class/usb_host_audio.d ./usb/host/class/usb_host_audio.o ./usb/host/class/usb_host_cdc.d ./usb/host/class/usb_host_cdc.o ./usb/host/class/usb_host_cdc_rndis.d ./usb/host/class/usb_host_cdc_rndis.o ./usb/host/class/usb_host_hid.d ./usb/host/class/usb_host_hid.o ./usb/host/class/usb_host_hub.d ./usb/host/class/usb_host_hub.o ./usb/host/class/usb_host_hub_app.d ./usb/host/class/usb_host_hub_app.o ./usb/host/class/usb_host_msd.d ./usb/host/class/usb_host_msd.o ./usb/host/class/usb_host_msd_ufi.d ./usb/host/class/usb_host_msd_ufi.o ./usb/host/class/usb_host_phdc.d ./usb/host/class/usb_host_phdc.o ./usb/host/class/usb_host_printer.d ./usb/host/class/usb_host_printer.o ./usb/host/class/usb_host_video.d ./usb/host/class/usb_host_video.o

.PHONY: clean-usb-2f-host-2f-class

