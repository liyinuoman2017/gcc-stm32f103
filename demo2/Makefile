# toolchain
CC           = arm-none-eabi-gcc
CP           = arm-none-eabi-objcopy
AS           = arm-none-eabi-gcc -x assembler-with-cpp

# all the files will be generated with this name 
PROJECT_NAME=stm32f10x_project

# user specific
SRC      += ./user/main.c
# startup
ASM_SRC  += ./user/startup_stm32f10x_md.s

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)
# Define optimisation level here
MC_FLAGS = -mcpu=cortex-m3
AS_FLAGS = $(MC_FLAGS) -g  -mthumb  
CP_FLAGS = $(MC_FLAGS)  -g  -mthumb  -Wall -fverbose-asm 
LD_FLAGS = $(MC_FLAGS) -g  -mthumb  -Xlinker --gc-sections -T stm32_flash.ld 

# makefile rules
all: $(OBJECTS) $(PROJECT_NAME).elf  $(PROJECT_NAME).hex $(PROJECT_NAME).bin
	 arm-none-eabi-size $(PROJECT_NAME).elf

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

