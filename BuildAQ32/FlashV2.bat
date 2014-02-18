set flasher="%ProgramFiles%\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
set filename=%1%
if "%filename%" == "" set filename=objSTM32\AeroQuad32\AeroQuadMain.bin

%flasher% -c SWD -P %filename% 0x08010000 -V -Rst
pause