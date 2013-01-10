cls
@echo off
set /p buildLibmaple= Build Libmaple? [y/n]
set /p buildClean= Build code? [y/n]:
if %buildLibmaple% == y (
	cd ..\Libmaple\libmaple
	make clean
	make library -j4
	cd ..
	cd ..
	cd BuildAQ32
)
if %buildClean% == y ( 
	echo Cleaning... 
	make clean 
	echo Building... 
	make -j4
)
..\tools\dfu-util.exe -l
echo .............
set /p isConnected= Is device connected in DFU mode and ready for flashing (is it listed above)? [y/n]:
if %isConnected% == y (
	..\tools\dfu-util.exe --reset --device 0483:df11 --alt 0 --dfuse-address 0x08010000 --download objSTM32\AeroQuad32\AeroQuadMain.bin
)
pause