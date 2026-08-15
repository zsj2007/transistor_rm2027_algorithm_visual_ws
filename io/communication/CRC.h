// CRC.h
#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <cstddef>

// CRC8查表法表格声明
extern const uint8_t CRC8_TAB[256];

// CRC8校验函数声明
uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

#endif // CRC_H
