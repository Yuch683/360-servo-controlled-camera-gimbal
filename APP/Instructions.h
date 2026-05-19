#ifndef __INSTRUCTIONS_H
#define __INSTRUCTIONS_H

#include <stdint.h>

void Instructions_Init(void);
void Instructions_RxByteHandler(uint8_t rx_byte);
void Instructions_Process(void);

#endif
