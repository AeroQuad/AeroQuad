#include <../AeroQuad/UserConfiguration.h>
#include <SerialMapping.h>
#include <WProgram.h>

__attribute__(( constructor )) void premain() {
    init();
}

int main(void)
{
	//init();
  	setup();

	for (;;)
		loop();

	return 0;
}


#include "../AeroQuad/AeroQuad.ino"

