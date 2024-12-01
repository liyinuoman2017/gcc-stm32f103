# toolchain
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp
# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJECT_NAME=stm32f10x_project

# define include dir
INCLUDE_DIRS =

# define stm32f10x lib dir
STM32F10x_LIB_DIR      = ./stm32f10x_lib

# define user dir
USER_DIR     = ./user

# link file
LINK_SCRIPT  = ./stm32_flash.ld

# user specific
SRC       =
ASM_SRC   =
SRC      += $(USER_DIR)/main.c
# user include
INCLUDE_DIRS  = $(USER_DIR)

# source director
STM32F1_STD_LIB     = $(STM32F10x_LIB_DIR)/STM32F10x_StdPeriph_Driver
STM32F1_CORE_DIR    = $(STM32F10x_LIB_DIR)/CMSIS/CM3/CoreSupport
STM32F1_DEVICE_DIR  = $(STM32F10x_LIB_DIR)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
STM32F1_SRC_DIR     = $(STM32F1_STD_LIB)/src
STM32F1_INC_DIR     = $(STM32F1_STD_LIB)/inc

# startup
ASM_SRC  += $(STM32F1_DEVICE_DIR)/startup/gcc_ride7/startup_stm32f10x_md.s

# CMSIS
SRC  += $(STM32F1_DEVICE_DIR)/system_stm32f10x.c
SRC  += $(STM32F1_CORE_DIR)/core_cm3.c

# use libraries, please add or remove when you use or remove it.
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_rcc.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_gpio.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_exti.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_usart.c
SRC  += $(STM32F1_SRC_DIR)/misc.c

# include directories
INCLUDE_DIRS += $(STM32F1_CORE_DIR)
INCLUDE_DIRS += $(STM32F1_DEVICE_DIR)
INCLUDE_DIRS += $(STM32F1_INC_DIR)
INCLUDE_DIRS += $(STM32F1_STD_LIB)
INC_DIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))
OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)

# Define optimisation level here
MC_FLAGS = -mcpu=cortex-m3
AS_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst)
CP_FLAGS = $(MC_FLAGS) -Os -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) 
LD_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb -nostartfiles -Xlinker --gc-sections -T$(LINK_SCRIPT) -Wl,-Map=$(PROJECT_NAME).map,--cref,--no-warn-mismatch

# makefile rules
all: $(OBJECTS) $(PROJECT_NAME).elf  $(PROJECT_NAME).hex $(PROJECT_NAME).bin
	$(TOOLCHAIN)size $(PROJECT_NAME).elf

%.o: %.c
	$(CC) -c $(CP_FLAGS) -I . $(INC_DIR) $< -o $@

%.o: %.s
	$(AS) -c $(AS_FLAGS) $< -o $@

%.elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@

%.hex: %.elf
	$(CP) -O ihex $< $@

%.bin: %.elf
	$(CP) -O binary -S  $< $@

clean:
	del /Q  $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

