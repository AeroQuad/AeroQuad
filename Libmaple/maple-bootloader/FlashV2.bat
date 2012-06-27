set flasher="%ProgramFiles%\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
set binary=build\maple_boot.bin

%flasher% -c SWD -P %binary% 0x08000000  -Rst
pause