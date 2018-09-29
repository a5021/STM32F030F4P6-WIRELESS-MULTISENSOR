TARGET = STM32F030-WIRELESS-MULTISENSOR

DEBUG = 0
OPT = -Ofast -flto

BUILD_DIR = _build

C_SOURCES =  \
../../src/adc.c \
../../src/bmp180.c \
../../src/control.c \
../../main.c \
../../drivers/src/system_stm32f0xx.c 

ASM_SOURCES = startup_stm32f031x6.s

PREFIX = arm-none-eabi-

ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
C_DEFS =  \
-DSTM32F031x6 \
-DSWD_DISABLED \
-DNDEBUG

C_INCLUDES =  \
-I../../drivers/inc \
-I../../inc/CMSIS  \
-I../../inc \
-Iinc

ASFLAGS = -mcpu=cortex-m0 -mthumb $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS  = -mcpu=cortex-m0 -mthumb $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fverbose-asm -std=c11

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

STLINK = ST-LINK_CLI.exe
STLINK_FLAGS = -c UR -V -P $(BUILD_DIR)/$(TARGET).hex -Hardrst -Run

LDSCRIPT = STM32F031F6Px_FLASH.ld

LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = -mcpu=cortex-m0 -mthumb -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

# Display compiler version information.
gccversion : 
	@$(CC) --version

# Program the device.  
program: $(BUILD_DIR)/$(TARGET).hex
	$(STLINK) $(STLINK_FLAGS) 

clean:
	-rm -fR $(BUILD_DIR)
