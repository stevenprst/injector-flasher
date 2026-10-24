#ifndef MAINLOOP_HPP
#define MAINLOOP_HPP

#include "main.h"

// The hooks that main.c and usbd_cdc_if.c call are declared in main.h, inside USER CODE EFP:
//   void mainLoopInit(void);
//   void mainLoopRun(void);
//   void injectorUsbReceive(const uint8_t *data, uint32_t length);

#endif // MAINLOOP_HPP
