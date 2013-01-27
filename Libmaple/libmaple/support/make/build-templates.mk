define LIBMAPLE_MODULE_template
dir := $(1)
LIBMAPLE_INCLUDES += -I$$(dir)
include $$(dir)/rules.mk
endef

