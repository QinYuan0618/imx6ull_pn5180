/*
 * pn5180.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Administrator
 */

#ifndef INC_PN5180_H_
#define INC_PN5180_H_

#include "main.h"

#define USE_SPI_HAL_LIB    0

#define pn5180_spi_handle (&hspi3)

int PN5180_Init(void);
void PN5180_Reset(void);
void PN5180_FieleOff(void);
void PN5180_FieleOn(void);
uint8_t PN5180_GetFieleState(void);
uint32_t PN5180_GetIRQStatus(void);
uint32_t PN5180_GetRxStatus(void);
uint32_t PN5180_GetRfStatus(void);
uint32_t PN5180_GetTransceiveState(void);
void PN5180_LoadRFConfiguration(int protocol);
int PN5180_ClearIRQStatus(uint32_t Mask);
void PN5180_WriteBytes(const void* Buffer, uint32_t len);
void PN5180_ReadBytes(void* Buffer, uint32_t len);
void PN5180_ReadE2Prom(uint8_t Address, void* Buffer, uint8_t Length);
void PN5180_WriteE2Prom(uint8_t Address, const void* Buffer, uint8_t Length);
uint32_t PN5180_ReadRegister(uint8_t Reg);
void PN5180_WriteRegister(uint8_t Reg, uint32_t Value);
void PN5180_WriteRegisterOrMask(uint8_t Reg, uint32_t Mask);
void PN5180_WriteRegisterAndMask(uint8_t Reg, uint32_t Mask);

#endif /* INC_PN5180_H_ */
