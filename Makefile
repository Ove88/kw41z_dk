# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := BUILD
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# Boiler-plate
###############################################################################
# Project settings

PROJECT := kw41z_dk


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += main.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/ASP.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/MPM.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/PhyISR.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/PhyPacketProcessor.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/PhyPlmeData.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/PhyStateMachine.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/source/PhyTime.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/source/FunctionLib.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/source/Messaging.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/source/GenericList.o
OBJECTS += ../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/source/MemManager.o


 SYS_OBJECTS += TARGET_KW41Z/TOOLCHAIN_GCC_ARM/startup_MKW41Z4.o

INCLUDE_PATHS += -I../
INCLUDE_PATHS += -I../.
INCLUDE_PATHS += -I../TARGET_KW41Z
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/TARGET_FRDM
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/device
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/drivers
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/rf_phy/rf_phy
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/framework_5.3.2
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/wireless/framework_5.3.2/XCVR
INCLUDE_PATHS += -I../TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/api
INCLUDE_PATHS += -I../TARGET_KW41Z/TOOLCHAIN_GCC_ARM
INCLUDE_PATHS += -I../drivers
INCLUDE_PATHS += -I../hal
INCLUDE_PATHS += -I../platform
INCLUDE_PATHS += -I../rtos
INCLUDE_PATHS += -I../rtx
INCLUDE_PATHS += -I../rtx/TARGET_CORTEX_M
INCLUDE_PATHS += -I../netsocket
INCLUDE_PATHS += -I../events
INCLUDE_PATHS += -I../events/equeue
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/nanostack-interface
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/mbed-mesh-api/mbed-mesh-api
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/mbed-mesh-api/mbed-mesh-api/source/include
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/nanostack
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/nanostack/platform
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/targets
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/coap-service/coap-service
INCLUDE_PATHS += -I../nanostack/FEATURE_NANOSTACK/coap-service/source/include


LIBRARY_PATHS := -L../TARGET_KW41Z/TOOLCHAIN_GCC_ARM  -L../nanostack/FEATURE_NANOSTACK_FULL/TOOLCHAIN_ARM/TARGET_LIKE_CORTEX_M0
LIBRARIES := -l:libmbed-os.a  -l:libnanostack_armcc_Cortex-M0_nanostack_full.ar
LINKER_SCRIPT ?= ../TARGET_KW41Z/TOOLCHAIN_GCC_ARM/MKW41Z512xxx4.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = 'arm-none-eabi-gcc' '-x' 'assembler-with-cpp' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
CC      = 'arm-none-eabi-gcc' '-std=gnu99' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
CPP     = 'arm-none-eabi-g++' '-std=gnu++98' '-fno-rtti' '-Wvla' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
LD      = 'arm-none-eabi-gcc' '-Wl,--gc-sections' '-mcpu=cortex-m0plus' '-mthumb'
ELF2BIN = 'arm-none-eabi-objcopy'


C_FLAGS += -std=gnu99
C_FLAGS += -DTARGET_KSDK2_MCUS
C_FLAGS += -D__MBED__=1
C_FLAGS += -DDEVICE_I2CSLAVE=1
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DTARGET_Freescale
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DDEVICE_RTC=1
C_FLAGS += -DTOOLCHAIN_object
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DFSL_RTOS_MBED
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -DTARGET_DEBUG
C_FLAGS += -DTARGET_M0P
C_FLAGS += -DDEVICE_ANALOGOUT=1
C_FLAGS += -DTARGET_UVISOR_UNSUPPORTED
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DCPU_MKW41Z512VHT4
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -D__CORTEX_M0PLUS
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1490875208.59
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DTARGET_FF_ARDUINO
C_FLAGS += -DARM_MATH_CM0PLUS
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -DDEVICE_SLEEP=1
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DTARGET_MCUXpresso_MCUS
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -DDEVICE_ERROR_RED=1
C_FLAGS += -DDEVICE_SPISLAVE=1
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -DTARGET_KW41Z
C_FLAGS += -DTARGET_FRDM
C_FLAGS += -DTARGET_LIKE_CORTEX_M0
C_FLAGS += -include
C_FLAGS += mbed_config.h

CXX_FLAGS += -std=gnu++98
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -DTARGET_KSDK2_MCUS
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DDEVICE_I2CSLAVE=1
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DTARGET_Freescale
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DDEVICE_RTC=1
CXX_FLAGS += -DTOOLCHAIN_object
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DFSL_RTOS_MBED
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -DTARGET_DEBUG
CXX_FLAGS += -DTARGET_M0P
CXX_FLAGS += -DDEVICE_ANALOGOUT=1
CXX_FLAGS += -DTARGET_UVISOR_UNSUPPORTED
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DCPU_MKW41Z512VHT4
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -D__CORTEX_M0PLUS
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1490875208.59
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DTARGET_FF_ARDUINO
CXX_FLAGS += -DARM_MATH_CM0PLUS
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -DDEVICE_SLEEP=1
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DTARGET_MCUXpresso_MCUS
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -DDEVICE_ERROR_RED=1
CXX_FLAGS += -DDEVICE_SPISLAVE=1
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -DTARGET_KW41Z
CXX_FLAGS += -DTARGET_FRDM
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M0
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h

ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DCPU_MKW41Z512VHT4
ASM_FLAGS += -D__CORTEX_M0PLUS
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -DFSL_RTOS_MBED
ASM_FLAGS += -DARM_MATH_CM0PLUS


LD_FLAGS :=-Wl,--gc-sections -mcpu=cortex-m0plus -mthumb 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.asm.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(LINKER_SCRIPT)
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter %.ld, $^) $(LIBRARY_PATHS) --output $@ $(filter %.o, $^) $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
