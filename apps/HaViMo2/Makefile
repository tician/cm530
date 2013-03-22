# setup
# modified by zerom for WinARM 8/2010
# modified by NaN for gcc-arm-none-eabi

#~~~~~~~~~~~~~~~~~~~~ Output File Name ~~~~~~~~~~~~~~~~~~~~
MAIN_OUT = HaViMo2_APP_CM530

EASYFUNCTIONSDIR = ../../easy-functions
LIBSTM32DIR = $(EASYFUNCTIONSDIR)/stm32f10x_lib

#~~~~~~~~~~~~~~~~~~~~ Source Files ~~~~~~~~~~~~~~~~~~~~
MAIN_OBJS = \
 ./main.o \
 $(EASYFUNCTIONSDIR)/src/stm32f10x_it.o \
 $(EASYFUNCTIONSDIR)/src/cm530.o 

#~~~~~~~~~~~~~~~~~~~~ Include Directories ~~~~~~~~~~~~~~~~~~~~
INCLUDE_DIRS = -I. -I$(LIBSTM32DIR)/inc -I$(EASYFUNCTIONSDIR)/inc -I$(EASYFUNCTIONSDIR)

#~~~~~~~~~~~~~~~~~~~~ Library Directories ~~~~~~~~~~~~~~~~~~~~
LIBRARY_DIRS = -L$(LIBSTM32DIR)



#~~~~~~~~~~~~~~~~~~~~ Compiler Options ~~~~~~~~~~~~~~~~~~~~
COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall -pedantic -g -Os -fno-common 

#~~~~~~~~~~~~~~~~~~~~ Toolchain Prefix ~~~~~~~~~~~~~~~~~~~~
TCHAIN_PREFIX=arm-none-eabi-

CC = $(TCHAIN_PREFIX)gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) 

CXX = $(TCHAIN_PREFIX)g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) 

AS = $(TCHAIN_PREFIX)gcc
ASFLAGS = $(COMPILE_OPTS) -c 

LD = $(TCHAIN_PREFIX)gcc
LDFLAGS = -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T $(EASYFUNCTIONSDIR)/stm32.ld 

OBJCP = $(TCHAIN_PREFIX)objcopy
OBJCPFLAGS_HEX = -O ihex
OBJCPFLAGS_BIN = -O binary

OBJDUMP = $(TCHAIN_PREFIX)objdump
OBJDUMPFLAGS = -h -S -C -D

SIZE = $(TCHAIN_PREFIX)size

AR = $(TCHAIN_PREFIX)ar
ARFLAGS = cr

MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_HEX = $(MAIN_OUT).hex
MAIN_OUT_BIN = $(MAIN_OUT).bin
MAIN_OUT_LSS = $(MAIN_OUT).lss


#~~~~~~~~~~~~~~~~~~~~ all ~~~~~~~~~~~~~~~~~~~~
all: begin gccversion sizebefore build sizeafter end

#~~~~~~~~~~~~~~~~~~~~ build ~~~~~~~~~~~~~~~~~~~~
build: \
	$(MAIN_OUT_ELF) \
	$(MAIN_OUT_HEX) \
	$(MAIN_OUT_BIN) \
	$(MAIN_OUT_LSS)


$(MAIN_OUT_ELF): $(MAIN_OBJS) $(LIBSTM32DIR)/libstm32.a
	$(LD) $(MAIN_OBJS) $(LIBSTM32DIR)/libstm32.a $(LDFLAGS) --output $@

$(MAIN_OUT_HEX): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS_HEX) $< $@

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS_BIN) $< $@

$(MAIN_OUT_LSS): $(MAIN_OUT_ELF)
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@

#~~~~~~~~~~~~~~~~~~~~ libstm32.a ~~~~~~~~~~~~~~~~~~~~
LIBSTM32_OUT = $(LIBSTM32DIR)/libstm32.a

LIBSTM32_OBJS = \
 $(LIBSTM32DIR)/src/stm32f10x_adc.o \
 $(LIBSTM32DIR)/src/stm32f10x_bkp.o \
 $(LIBSTM32DIR)/src/stm32f10x_exti.o \
 $(LIBSTM32DIR)/src/stm32f10x_flash.o \
 $(LIBSTM32DIR)/src/stm32f10x_gpio.o \
 $(LIBSTM32DIR)/src/stm32f10x_lib.o \
 $(LIBSTM32DIR)/src/stm32f10x_nvic.o \
 $(LIBSTM32DIR)/src/stm32f10x_pwr.o \
 $(LIBSTM32DIR)/src/stm32f10x_rcc.o \
 $(LIBSTM32DIR)/src/stm32f10x_systick.o \
 $(LIBSTM32DIR)/src/stm32f10x_tim.o \
 $(LIBSTM32DIR)/src/stm32f10x_tim1.o \
 $(LIBSTM32DIR)/src/stm32f10x_usart.o \
 $(LIBSTM32DIR)/src/cortexm3_macro.o \
 $(LIBSTM32DIR)/src/stm32f10x_can.o \
 $(LIBSTM32DIR)/src/stm32f10x_dma.o \
 $(LIBSTM32DIR)/src/stm32f10x_i2c.o \
 $(LIBSTM32DIR)/src/stm32f10x_iwdg.o \
 $(LIBSTM32DIR)/src/stm32f10x_rtc.o \
 $(LIBSTM32DIR)/src/stm32f10x_spi.o \
 $(LIBSTM32DIR)/src/stm32f10x_wwdg.o \
 $(LIBSTM32DIR)/src/stm32f10x_vector.o

 
$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBSTM32_OBJS): $(EASYFUNCTIONSDIR)/stm32f10x_conf.h



MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:

#~~~~~~~~~~~~~~~~~~~~ Eye candy ~~~~~~~~~~~~~~~~~~~~
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo

sizebefore:
	@if test -f $(MAIN_OUT_ELF); then echo; \
	echo $(MSG_SIZE_BEFORE);                \
	$(SIZE) $(MAIN_OUT_ELF);                \
	$(SIZE) --target=ihex $(MAIN_OUT_HEX);  \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(MAIN_OUT_ELF); then echo; \
	echo $(MSG_SIZE_AFTER);                 \
	$(SIZE) $(MAIN_OUT_ELF);                \
	$(SIZE) --target=ihex $(MAIN_OUT_HEX);  \
	2>/dev/null; echo; fi

gccversion : 
	@$(CC) --version

#~~~~~~~~~~~~~~~~~~~~ clean ~~~~~~~~~~~~~~~~~~~~
clean: begin clean_list end

clean_list:
	-rm $(MAIN_OBJS)
	-rm $(LIBSTM32_OBJS)
	-rm $(LIBSTM32_OUT)
	-rm $(MAIN_OUT_ELF)
	-rm $(MAIN_OUT_HEX)
	-rm $(MAIN_OUT_BIN)
	-rm $(MAIN_OUT_LSS)
	-rm $(MAIN_OUT_ELF).map

#~~~~~~~~~~~~~~~~~~~~ backup ~~~~~~~~~~~~~~~~~~~~
backup: clean
	tar cJvf ../$(MAIN_OUT)_`date +"%Y-%m-%d_%H%M"`.tar.xz *
