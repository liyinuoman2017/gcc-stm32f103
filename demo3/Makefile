# toolchain
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp

# all the files will be generated with this name
PROJECT_NAME=stm32f10x_project

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

# startup
ASM_SRC  += $(USER_DIR)/startup_stm32f10x_md.s

# include directories
INCLUDE_DIRS += $(STM32F1_CORE_DIR)
#INCLUDE_DIRS += $(STM32F1_DEVICE_DIR)
INCLUDE_DIRS += $(STM32F1_INC_DIR)
INCLUDE_DIRS += $(STM32F1_STD_LIB)

INC_DIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)
# Define optimisation level here
OPT = -Os

MC_FLAGS = -mcpu=cortex-m3

AS_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst)
CP_FLAGS = $(MC_FLAGS) $(OPT) -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) 
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

