// CRC.h
#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <cstddef>

// CRC8查表法表格声明
extern const uint8_t CRC8_TAB[256];

// CRC8校验函数声明（标准化签名：const uint8_t*, size_t）
uint8_t CRC8_Check_Sum(const uint8_t *pchMessage, size_t dwLength);

// CRC32校验函数声明（STM32 HAL兼容）
uint32_t CRC32_Calculate(const uint8_t* data, size_t length);

#endif // CRC_H
