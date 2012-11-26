#ifndef __TRACE_H__
#define __TRACE_H__

#include <stdint.h>

void TRACE_Init();
void TRACE(char *format, ...);
void TRACE_SendPayload(uint8_t *payload, uint16_t size);

#endif