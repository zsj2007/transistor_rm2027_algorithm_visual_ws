// CRC.h
#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <cstddef>

// CRC8查表法表格声明
extern const uint8_t CRC8_TAB[256];

// CRC8校验函数声明（const 指针 + size_t 长度，兼容旧串口协议与 TorqueController）
uint8_t CRC8_Check_Sum(const uint8_t *pchMessage, size_t dwLength);

// CRC32校验函数声明（TorqueController IMU 协议使用，STM32 HAL 兼容，多项式 0x04C11DB7）
uint32_t CRC32_Calculate(const uint8_t* data, size_t length);

#endif // CRC_H
