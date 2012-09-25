set filename=%1%
if "%filename%" == "" set filename=build\maple_boot.bin
:loop
	..\..\tools\dfu-util.exe --reset --device 0483:df11 --alt 0 --dfuse-address 0x08010000 --download %filename% || goto :loop
pause
