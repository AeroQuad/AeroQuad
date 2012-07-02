# Top level Makefile for libmaple

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld

# The main library
lib_LIBRARIES = \
	libmaple.a \
	libmapleusb.a

#	libwirish.a

# noinst_PROGRAMS = \
# 	main

# main_SOURCES = \
# 	startup2.c \
# 	main.cpp

main_LDFLAGS = \
	--gc-sections \
	-Map=main.map

main_LDADD = libmaple.a

# Main library
libmaple_a_SOURCES = \
	libmaple/adc.c \
	libmaple/bkp.c \
	libmaple/dac.c \
	libmaple/dma.c \
	libmaple/exti.c \
	libmaple/flash.c \
	libmaple/fsmc.c \
	libmaple/gpio.c \
	libmaple/iwdg.c \
	libmaple/nvic.c \
	libmaple/pwr.c \
	libmaple/i2c.c \
	libmaple/rcc.c \
	libmaple/spi.c \
	libmaple/syscalls.c \
	libmaple/systick.c \
	libmaple/timer.c \
	libmaple/usart.c \
	libmaple/util.c

nobase_include_HEADERS = \
	libmaple/adc.h \
	libmaple/bitband.h \
	libmaple/bkp.h \
	libmaple/dac.h \
	libmaple/delay.h \
	libmaple/dma.h \
	libmaple/exti.h \
	libmaple/flash.h \
	libmaple/fsmc.h \
	libmaple/gpio.h \
	libmaple/i2c.h \
	libmaple/iwdg.h \
	libmaple/libmaple.h \
	libmaple/libmaple_types.h \
	libmaple/nvic.h \
	libmaple/pwr.h \
	libmaple/rcc.h \
	libmaple/ring_buffer.h \
	libmaple/scb.h \
	libmaple/spi.h \
	libmaple/stm32.h \
	libmaple/systick.h \
	libmaple/timer.h \
	libmaple/usart.h \
	libmaple/util.h \
	libmaple/usb/descriptors.h \
	libmaple/usb/usb.h \
	libmaple/usb/usb_callbacks.h \
	libmaple/usb/usb_config.h \
	libmaple/usb/usb_hardware.h \
	libmaple/usb/usb_lib/usb_core.h \
	libmaple/usb/usb_lib/usb_def.h \
	libmaple/usb/usb_lib/usb_init.h \
	libmaple/usb/usb_lib/usb_int.h \
	libmaple/usb/usb_lib/usb_lib.h \
	libmaple/usb/usb_lib/usb_mem.h \
	libmaple/usb/usb_lib/usb_regs.h \
	libmaple/usb/usb_lib/usb_type.h

libmapleusb_a_SOURCES = \
	libmaple/usb/descriptors.c \
	libmaple/usb/usb.c \
	libmaple/usb/usb_callbacks.c \
	libmaple/usb/usb_hardware.c \
	libmaple/usb/usb_lib/usb_core.c \
	libmaple/usb/usb_lib/usb_init.c \
	libmaple/usb/usb_lib/usb_int.c \
	libmaple/usb/usb_lib/usb_mem.c \
	libmaple/usb/usb_lib/usb_regs.c

libwirish_a_SOURCES = \
	wirish/wirish_math.cpp \
	wirish/Print.cpp \
	wirish/boards.cpp \
	wirish/boards/maple.cpp \
	wirish/boards/maple_mini.cpp \
	wirish/boards/maple_native.cpp \
	wirish/boards/maple_RET6.cpp \
	wirish/comm/HardwareSerial.cpp \
	wirish/comm/HardwareSPI.cpp \
	wirish/HardwareTimer.cpp \
	wirish/usb_serial.cpp \
	wirish/cxxabi-compat.cpp \
	wirish/wirish_shift.cpp \
	wirish/wirish_analog.cpp \
	wirish/wirish_time.cpp \
	wirish/pwm.cpp \
	wirish/ext_interrupts.cpp \
	wirish/wirish_digital.cpp

MCU := STM32F103RB
BOARD ?= maple
DENSITY = STM32_MEDIUM_DENSITY

FLAGS = \
	-Os -ggdb -nostdlib -Wall \
	-ffunction-sections -fdata-sections -Wl,--gc-sections \
	-mcpu=cortex-m3 -mthumb -fshort-enums -mfloat-abi=soft \
	-DBOARD_$(BOARD) -DMCU_$(MCU) -D$(DENSITY) -DVECT_TAB_BASE \
	-I$(srcdir)/libmaple \
	-I$(srcdir)/libmaple/usb \
	-I$(srcdir)/libmaple/usb/usb_lib

AM_CFLAGS = $(FLAGS) -std=gnu99

AM_CXXFLAGS = $(FLAGS) \
	-I$(srcdir)/wirish -I$(srcdir)/wirish/comm -I$(srcdir)/wirish/boards \
	-fno-rtti -fno-exceptions
