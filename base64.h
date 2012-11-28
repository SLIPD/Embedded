#ifndef __BASE64_H__
#define __BASE64_H__

#include <stdint.h>

void decodeData(uint8_t* data, char* message, char* debug);
void encodeDataString(uint8_t *string, uint8_t *returnData);

#endif