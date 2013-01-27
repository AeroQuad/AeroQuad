#include <../AeroQuad/UserConfiguration.h>
#include <SerialMapping.h>
#include <WProgram.h>

__attribute__(( constructor )) void premain() {
    init();
}

extern "C"{
        void _init(){};
}

// Uncomment this if compiling on OS X
/*extern "C"{
	void _init(){}; // dummy _init function for support of GNU toolchain from https://launchpad.net/gcc-arm-embedded
}*/

int main(void)
{
	//init();
  	setup();

	for (;;)
		loop();

	return 0;
}


#include "../AeroQuad/AeroQuad.ino"

