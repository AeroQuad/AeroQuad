# setup environment

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

CC = arm-none-eabi-gcc
CFLAGS =

AS = $(CC) -x assembler-with-cpp -c $(TARGET_ARCH)
ASFLAGS =

AR = arm-none-eabi-ar
ARFLAGS = cr

LIB_OBJS = stm32_vector_table.o stm32_isrs.o start.o start_c.o

help:
	@echo "Targets:"
	@echo "\t medium-density: Target medium density chips (e.g. Maple)"
	@echo "\t high-density: Target high density chips (e.g. Maple-native)"

.PHONY: help medium high

medium-density: $(LIB_OBJS)
	$(AR) $(ARFLAGS) libcs3_stm32_med_density.a $(LIB_OBJS)
	rm -f $(LIB_OBJS)

high-density: CFLAGS := -DSTM32_HIGH_DENSITY
high-density: ASFLAGS := -DSTM32_HIGH_DENSITY
high-density: $(LIB_OBJS)
	$(AR) $(ARFLAGS) libcs3_stm32_high_density.a $(LIB_OBJS)
	rm -f $(LIB_OBJS)

# clean
.PHONY: clean
clean:
	-rm -f $(LIB_OBJS) *.a
