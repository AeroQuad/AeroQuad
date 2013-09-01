# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
# Carlos Lamas
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging, 
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

BUILDTYPE ?= STM32
#BUILDTYPE ?= XMEGA

BOARD ?= aeroquad32
#BOARD ?= aeroquad32f1
#BOARD ?= aeroquad32mini
#BOARD	= discovery_f4
#BOARD ?= freeflight

V=1

ADDITIONALDEFINES = 
#ADDITIONALDEFINES = -DSENSORBOARD_ALA42

ifeq ($(BUILDTYPE), XMEGA)
	# MCU name
	BUILDPREFIX=avr
	MCU = atxmega128a1
	AVRDUDE_MCU = atxmega128a1
	CPUCLASS=XMEGA
	F_CPU = 32000000
	DEFINECPU = -mmcu=$(MCU)
	EXTRAINCDIRS = ../arduinoXMega ../arduinoXMega/Libraries
	RUNTIMELIB = ../arduinoXMega/libAVRArduino.a
endif

ifeq ($(BUILDTYPE), STM32)
	BUILDPREFIX= arm-none-eabi
	CPUCLASS=STM32
	F_CPU = 72000000
	MEMORY_TARGET ?= flash
	MCU_OPTIONS = -mcpu=cortex-m3 -mthumb 
	#-fshort-double
	
	# Guess the MCU based on the BOARD (can be overridden )
	ifeq ($(BOARD), maple)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103RB
	   PRODUCT_ID := 0003
	endif
	ifeq ($(BOARD), aeroquad32f1)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103VE
	   DENSITY := STM32_HIGH_DENSITY
	   PRODUCT_ID := 0003
	   MCU_OPTIONS += -DBOARD_aeroquad32
	endif
	ifeq ($(BOARD), aeroquad32)
	   MCU_FAMILY := STM32F2
	   MCU := STM32F406VG
	   DENSITY := STM32_HIGH_DENSITY
	   PRODUCT_ID := 0003
	   MCU_OPTIONS = -mcpu=cortex-m4 -mthumb  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
	   F_CPU = 168000000
	endif
	ifeq ($(BOARD), aeroquad32mini)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103CB
	   DENSITY := STM32_MEDIUM_DENSITY
	   PRODUCT_ID := 0003
	endif
	ifeq ($(BOARD), freeflight)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103CB
	   DENSITY := STM32_MEDIUM_DENSITY
	   PRODUCT_ID := 0003
	endif
	ifeq ($(BOARD), maple_native)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103ZE
	   DENSITY := STM32_HIGH_DENSITY
	   PRODUCT_ID := 0003
	endif
	ifeq ($(BOARD), maple_mini)
	   MCU_FAMILY := STM32F1
	   MCU := STM32F103CB
	   DENSITY := STM32_MEDIUM_DENSITY
	   PRODUCT_ID := 0003
	endif
	ifeq ($(BOARD), discovery_f4)
	   MCU_FAMILY := STM32F2
	   MCU := STM32F406VG
	   DENSITY := STM32_HIGH_DENSITY
	   PRODUCT_ID := 0003
	   MCU_OPTIONS = -mcpu=cortex-m4 -mthumb  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
	   F_CPU = 168000000
	endif

	LIB_MAPLE_HOME = ../Libmaple/libmaple
	SRCROOT := $(LIB_MAPLE_HOME)
	SUPPORT_PATH := $(SRCROOT)/support
	LDDIR    := $(SUPPORT_PATH)/ld
##### Uncomment the -nostartfiles if compiling on OS X #####
	LDFLAGSSTM32 = -T$(LDDIR)/$(LDSCRIPT) -L$(LDDIR)  -Xlinker --gc-sections -lc -nostartfiles

# Set up build rules and some useful templates
include $(SUPPORT_PATH)/make/build-rules.mk
include $(SUPPORT_PATH)/make/build-templates.mk

# Some target specific things
ifeq ($(MEMORY_TARGET), ram)
   LDSCRIPT := $(BOARD)/ram.ld
   VECT_BASE_ADDR := VECT_TAB_RAM
endif
ifeq ($(MEMORY_TARGET), flash)
   LDSCRIPT := $(BOARD)/flash.ld
   VECT_BASE_ADDR := VECT_TAB_FLASH
endif
ifeq ($(MEMORY_TARGET), jtag)
   LDSCRIPT := $(BOARD)/jtag.ld
   VECT_BASE_ADDR := VECT_TAB_BASE
endif

DEFINECPU = $(MCU_OPTIONS) -DBOARD_$(BOARD) -DMCU_$(MCU) -D$(MCU_FAMILY) -D$(DENSITY) -fno-exceptions 
EXTRACPPFLAGS = -fno-rtti
RUNTIMELIB = $(LIB_MAPLE_HOME)/build/libmaple.a
MAPLEINCDIRS = $(LIB_MAPLE_HOME) $(LIB_MAPLE_HOME)/libmaple $(LIB_MAPLE_HOME)/wirish $(LIB_MAPLE_HOME)/wirish/comm $(LIB_MAPLE_HOME)/wirish/boards $(LIB_MAPLE_HOME)/libraries/Wire
EXTRAINCDIRS = $(MCDIR) $(SRCDIRAQ32) $(MAPLEINCDIRS)
endif

EXTRAINCDIRS += $(LIBDIR)/AQ_Accelerometer $(LIBDIR)/AQ_BarometricSensor $(LIBDIR)/AQ_BatteryMonitor \
 $(LIBDIR)/AQ_CameraStabilizer $(LIBDIR)/AQ_Compass $(LIBDIR)/AQ_Defines \
 $(LIBDIR)/AQ_FlightControlProcessor $(LIBDIR)/ $(LIBDIR)/AQ_Gps $(LIBDIR)/AQ_Gyroscope \
 $(LIBDIR)/AQ_I2C $(LIBDIR)/AQ_Kinematics $(LIBDIR)/AQ_Math $(LIBDIR)/AQ_Motors \
 $(LIBDIR)/AQ_OSD  $(LIBDIR)/AQ_Platform_APM $(LIBDIR)/AQ_Platform_CHR6DM \
 $(LIBDIR)/AQ_Platform_MPU6000 $(LIBDIR)/AQ_Platform_Wii $(LIBDIR)/AQ_RangeFinder \
 $(LIBDIR)/AQ_Receiver $(LIBDIR)/AQ_SPI $(LIBDIR)/AQ_RSSI $(LIBDIR)/AQ_SoftModem \
 $(LIBDIR)/AQ_RSCode $(LIBDIR)/AQ_MAVLink $(LIBMAPLEDIR)/FatFS


# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the 
#     processor frequency. You can then use this symbol in your source code to 
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#     Typical values are:
#         F_CPU =  1000000
#         F_CPU =  1843200
#         F_CPU =  2000000
#         F_CPU =  3686400
#         F_CPU =  4000000
#         F_CPU =  7372800
#         F_CPU =  8000000
#         F_CPU = 11059200
#         F_CPU = 14745600
#         F_CPU = 16000000
#         F_CPU = 18432000
#         F_CPU = 20000000


# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIRBASE = obj$(CPUCLASS)
# added a dummy directory as BASEDIR is ..
OBJDIR = $(OBJDIRBASE)/dummy

# Target file name (without extension).
TARGETSOURCE = $(SRCDIRAQ32)/AeroQuadMain
TARGET = $(OBJDIR)/$(TARGETSOURCE)

DEPENDENCIES = $(SRCDIRAQ32)/AeroQuad_STM32.h

# List C source files here. (C dependencies are automatically generated.)
SRC =  	

BASEDIR = ..
SRCDIR     = $(BASEDIR)/AeroQuad
LIBDIR     = $(BASEDIR)/Libraries
SRCDIRAQ32 = $(BASEDIR)/AeroQuad32
MCDIR      = $(SRCDIRAQ32)/MapleCompatibility
LIBMAPLEDIR = $(BASEDIR)/Libmaple/libmaple/libraries

# List C++ source files here. (C dependencies are automatically generated.)
CPPSRC = $(TARGETSOURCE).cpp
#CPPSRC += #$(SRCDIR)/Median.cpp $(SRCDIR)/Smooth.cpp $(SRCDIR)/MS561101BA.cpp
#CPPSRC += $(SRCDIR)/MS561101BA.cpp
CPPSRC += $(MCDIR)/EEPROM.cpp 
CPPSRC += $(LIBDIR)/AQ_I2C/Device_I2C.cpp
#CPPSRC += $(LIBDIR)/AQ_Gps/TinyGPS.cpp
CPPSRC += $(LIBDIR)/AQ_Math/AQMath.cpp
SRC += $(MCDIR)/flash_stm32.c

# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC = 


# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
#EXTRAINCDIRS = arduinoXMega arduinoXMega/Libraries


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL
CDEFS += $(ADDITIONALDEFINES)

# Place -D or -U options here for ASM sources
ADEFS = -DF_CPU=$(F_CPU)


# Place -D or -U options here for C++ sources
CPPDEFS = -DF_CPU=$(F_CPU)UL
CPPDEFS += $(ADDITIONALDEFINES)
#CPPDEFS += -D__STDC_LIMIT_MACROS
#CPPDEFS += -D__STDC_CONSTANT_MACROS



#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fsingle-precision-constant
#CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
#CFLAGS += -mshort-calls
#CFLAGS += -fno-unit-at-a-time
#CFLAGS += -Wundef
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wsign-compare

#CFLAGS += -mcall-prologues

CFLAGS += -Wa,-adhlns=$(<:%.c=$(OBJDIR)/%.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)


#---------------- Compiler Options C++ ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CPPFLAGS = -g$(DEBUG)
CPPFLAGS += $(CPPDEFS)
CPPFLAGS += -O$(OPT)
CPPFLAGS += -ffunction-sections
CPPFLAGS += -fdata-sections
CPPFLAGS += -funsigned-char
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -fsingle-precision-constant
#CPPFLAGS += -fpack-struct
CPPFLAGS += -fshort-enums
CPPFLAGS += -fno-exceptions
CPPFLAGS += -Wall
CPPFLAGS += -Wundef

# -gdwarf-2 -MD -MP -MT Print.o -MF dep/Print.o.d  -c  ../arduinoXMega/Print.cpp

#CPPFLAGS += -mshort-calls
#CPPFLAGS += -fno-unit-at-a-time
#CPPFLAGS += -Wstrict-prototypes
#CPPFLAGS += -Wunreachable-code
#CPPFLAGS += -Wsign-compare

#CPPFLAGS += -mcall-prologues

CPPFLAGS += -Wa,-adhlns=$(<:%.cpp=$(OBJDIR)/%.lst)
CPPFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
#CPPFLAGS += $(CSTANDARD)


#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adhlns:   create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex 
#       dump that will be displayed for a given single line of source input.
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:%.S=$(OBJDIR)/%.lst),-gstabs,--listing-cont-lines=100


#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
#PRINTF_LIB = 
#PRINTF_LIB = $(PRINTF_LIB_MIN)
PRINTF_LIB = $(PRINTF_LIB_FLOAT)


# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = 
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)


MATH_LIB = -lc -lm -lc


# List any extra directories to look for libraries here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRALIBDIRS = ../arduinoXMega





#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,--gc-section,-Map=$(TARGET).map,--cref,--relax
LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
#LDFLAGS += -T linker_script.x

ifeq ($(BUILDTYPE), STM32)
	LDFLAGS = $(LDFLAGSSTM32)
endif



#---------------- Programming Options (avrdude) ----------------

# Programming hardware
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = usbasp

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = usb

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex:i
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

AVRDUDE_WRITE_FUSES = -U lfuse:w:0xf7:m -U hfuse:w:0xd7:m -U efuse:w:0x07:m 
AVRDUDE_READ_FUSES = -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i 

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(AVRDUDE_MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)



#---------------- Debugging Options ----------------

# For simulavr only - target MCU frequency.
DEBUG_MFREQ = $(F_CPU)

# Set the DEBUG_UI to either gdb or insight.
# DEBUG_UI = gdb
DEBUG_UI = insight

# Set the debugging back-end to either avarice, simulavr.
DEBUG_BACKEND = avarice
#DEBUG_BACKEND = simulavr

# GDB Init Filename.
GDBINIT_FILE = __avr_gdbinit

# When using avarice settings for the JTAG
JTAG_DEV = /dev/com1

# Debugging port used to communicate between GDB / avarice / simulavr.
DEBUG_PORT = 4242

# Debugging host used to communicate between GDB / avarice / simulavr, normally
#     just set to localhost unless doing some sort of crazy debugging when 
#     avarice is running on a different computer.
DEBUG_HOST = localhost



#============================================================================

# Define programs and commands.
SHELL = sh
CC = $(BUILDPREFIX)-gcc
OBJCOPY = $(BUILDPREFIX)-objcopy
OBJDUMP = $(BUILDPREFIX)-objdump
SIZE = $(BUILDPREFIX)-size
AR = $(BUILDPREFIX)-ar rcs
NM = $(BUILDPREFIX)-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp
WINSHELL = cmd


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
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




# Define all object files.
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o) 

# Define all listing files.
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst) 


# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = $(DEFINECPU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_CPPFLAGS = $(DEFINECPU) -I. -x c++ $(CPPFLAGS) $(EXTRACPPFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = $(DEFINECPU) -I. -x assembler-with-cpp $(ASFLAGS)





# Default target.
#all: begin gccversion sizebefore build sizeafter end
all: sizebefore build sizeafter

# Change the build target to build a HEX file or a library.
build: elf bin hex eep lss sym
#build: lib

AeroQuadMain.cpp:	$(DEPENDENCIES)
elf: $(TARGET).elf
bin: $(TARGET).bin
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym
LIBNAME=lib$(TARGET).a
lib: $(LIBNAME)

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) --mcu=$(MCU) --format=avr $(TARGET).elf
ELFSIZE = $(SIZE) $(TARGET).elf

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); 2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); 2>/dev/null; echo; fi



# Display compiler version information.
gccversion : 
	@$(CC) --version



# Program the device.  
program: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) 

# Set device fuses  
write-fuses: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES) 

# Read device fuses  
read-fuses: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_READ_FUSES) 

# Un brick the device with a 1 MHz crystal connected to pin7 (XTAL1  
unbrick: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES) -B 100


# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set 
#     a breakpoint at main().
gdb-config: 
	@$(REMOVE) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif
	@echo break main >> $(GDBINIT_FILE)

debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
	@$(WINSHELL) /c pause

else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)




# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT = $(OBJCOPY) --debugging
COFFCONVERT += --change-section-address .data-0x800000
COFFCONVERT += --change-section-address .bss-0x800000
COFFCONVERT += --change-section-address .noinit-0x800000
COFFCONVERT += --change-section-address .eeprom-0x810000



coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	@$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	@$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof



# Create final output files (.hex, .eep) from ELF output file.
%.bin: %.elf
	@echo
	@echo $(MSG_FLASH) binary $@
	$(OBJCOPY) -v -Obinary $< $@

%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	@-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) $< $@ || exit 0

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	@$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	@$(NM) -n $< > $@



# Create library from object files.
.SECONDARY : $(TARGET).a
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	@echo
	@echo $(MSG_CREATING_LIBRARY) $@
	$(AR) $@ $(OBJ)


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ) $(RUNTIMELIB)
	@echo
	@echo $(MSG_LINKING) $@
	@$(CC) $(ALL_CFLAGS) $^ $(RUNTIMELIB) $(LIB_MAPLE_HOME)/build/libmaple/syscalls.o --output $@ $(LDFLAGS)
#	@$(CC) $(ALL_CFLAGS) $^ AeroQuad32/libm.a $(RUNTIMELIB) $(LIB_MAPLE_HOME)/build/libmaple/syscalls.o --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
#@$(CC) -c $(ALL_CFLAGS) $< -o $@
$(OBJDIR)/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	@$(CC) -c $(ALL_CFLAGS) $(abspath $<) -o $@ $(VS_TRIM_ERRORS)


# Compile: create object files from C++ source files.
#@$(CC) -c $(ALL_CPPFLAGS) $< -o $@
#$(CC) -c $(ALL_CPPFLAGS) $(abspath $<) -o $@ $(VS_TRIM_ERRORS)
$(OBJDIR)/%.o : %.cpp
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	@$(CC) -c $(ALL_CPPFLAGS) $(abspath $<) -o $@ $(VS_TRIM_ERRORS)


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C++ source files.
%.s : %.cpp
	$(CC) -S $(ALL_CPPFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E $(DEFINECPU) -I. $(CFLAGS) $< -o $@ 


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVEDIR) .dep
	-$(REMOVEDIR) $(OBJDIRBASE)


# Create object files directory
ifeq ($(BUILDTYPE), XMEGA)
$(shell mkdir -p $(OBJDIR)/arduinoXMega/Libraries 2>/dev/null)
endif
ifeq ($(BUILDTYPE), STM32)
$(shell mkdir -p $(OBJDIR)/$(SRCDIRAQ32) $(OBJDIR)/$(MCDIR) 2>/dev/null)
endif
#$(shell mkdir -p $(OBJDIR) $(OBJDIR)/$(SRCDIR) $(OBJDIR)/$(SRCDIRAQ32) $(OBJDIR)/$(MCDIR) $(OBJDIR)/arduinoXMega $(OBJDIR)/arduinoXMega/Libraries $(OBJDIR)/$(LIBDIR)  $(OBJDIR)/$(LIBDIR)/AQ_Gps $(OBJDIR)/$(LIBDIR)/AQ_I2C $(OBJDIR)/$(LIBDIR)/AQ_Math 2>/dev/null)
$(shell mkdir -p $(OBJDIR) $(OBJDIR)/$(SRCDIR) $(OBJDIR)/$(LIBDIR)  $(OBJDIR)/$(LIBDIR)/AQ_Gps $(OBJDIR)/$(LIBDIR)/AQ_I2C $(OBJDIR)/$(LIBDIR)/AQ_Math 2>/dev/null)

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion build elf hex eep lss sym coff extcoff clean clean_list program debug gdb-config


