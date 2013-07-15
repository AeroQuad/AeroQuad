# Useful tools
CC       := arm-none-eabi-gcc
CXX      := arm-none-eabi-g++
LD       := arm-none-eabi-ld -v
AR       := arm-none-eabi-ar
AS       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
DISAS    := arm-none-eabi-objdump
OBJDUMP  := arm-none-eabi-objdump
SIZE     := arm-none-eabi-size
DFU      := dfu-util
OPENOCD_WRAPPER  := support/scripts/openocd-wrapper.sh

# Suppress annoying output unless V is set
ifndef V
   SILENT_CC       = @echo '  [CC]       ' $(@:$(BUILD_PATH)/%.o=%.c);
   SILENT_AS       = @echo '  [AS]       ' $(@:$(BUILD_PATH)/%.o=%.S);
   SILENT_CXX      = @echo '  [CXX]      ' $(@:$(BUILD_PATH)/%.o=%.cpp);
   SILENT_LD       = @echo '  [LD]       ' $(@F);
   SILENT_AR       = @echo '  [AR]       '
   SILENT_OBJCOPY  = @echo '  [OBJCOPY]  ' $(@F);
   SILENT_DISAS    = @echo '  [DISAS]    ' $(@:$(BUILD_PATH)/%.bin=%).disas;
   SILENT_OBJDUMP  = @echo '  [OBJDUMP]  ' $(OBJDUMP);
endif

BUILDDIRS :=
TGT_BIN   :=

CFLAGS   = $(GLOBAL_CFLAGS) $(TGT_CFLAGS)
CXXFLAGS = $(GLOBAL_CXXFLAGS) $(TGT_CXXFLAGS)
ASFLAGS  = $(GLOBAL_ASFLAGS) $(TGT_ASFLAGS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

ifdef IDE_VS
VS_TRIM_ERRORS = 2>&1 | sed -e 's/\(\w\+\):\([0-9]\+\):/\1(\2):/'
else
VS_TRIM_ERRORS =
endif

# General directory independent build rules, generate dependency information
	#$(SILENT_CC) $(CC) $(CFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<
$(BUILD_PATH)/%.o: %.c
	$(SILENT_CC) $(CC) $(CFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $(abspath $<) $(VS_TRIM_ERRORS)

#$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<
$(BUILD_PATH)/%.o: %.cpp
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $(abspath $<) $(VS_TRIM_ERRORS)

#$(SILENT_AS) $(AS) $(ASFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<
$(BUILD_PATH)/%.o: %.S
	$(SILENT_AS) $(AS) $(ASFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $(abspath $<) $(VS_TRIM_ERRORS)
