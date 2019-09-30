/*
 * i2c_interface.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __I2C_INTERFACE_H__
#define __I2C_INTERFACE_H__

extern void I2C_StartTransmission(I2C_TypeDef* I2Cx, uint8_t transmissionDirection,  uint8_t slaveAddress);
extern void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);
extern uint8_t I2C_ReadData(I2C_TypeDef* I2Cx);

#endif /* __I2C_INTERFACE_H__ */
