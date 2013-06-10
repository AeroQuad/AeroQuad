target remote localhost:3333
symbol-file build/maple.elf
source test.gdb
delete breakpoints
##break main.cpp:setup()
##monitor reset halt
#display/i $pc
# display/x *0xe000ed2c
# display/x *0xE000ED28
