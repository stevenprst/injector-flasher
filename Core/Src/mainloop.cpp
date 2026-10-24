#include "mainloop.hpp"

// The hooks are declared extern "C" in main.h, so these definitions keep C linkage.
void mainLoopInit(void)
{
}

void mainLoopRun(void)
{
}

// The USB receive callback of the CDC interface calls this for every packet. The link comes in a later commit.
void injectorUsbReceive(const uint8_t *data, uint32_t length)
{
	(void)data;
	(void)length;
}
