#include <SerialMapping.h>
#include <WProgram.h>

__attribute__(( constructor )) void premain() {
    init();
}

extern "C"{
	void _initxx(){};
}

extern "C" { void systick_attach_callback(void (*callback)(void));}
extern "C" { void logWorker();}


int main(void)
{
  	setup();
  	systick_attach_callback(loop);

	for (;;) {
		//loop();
		logWorker();
	}

	return 0;
}


#include "../AeroQuad/AeroQuad.ino"
//#include "../AeroQuad/AeroQuad.cpp"
