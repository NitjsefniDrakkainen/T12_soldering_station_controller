/*
 * i2c_interface.c
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#include "stm32f10x.h"
#include "i2c_interface.h"

void I2C_StartTransmission(I2C_TypeDef* I2Cx, uint8_t transmissionDirection,  uint8_t slaveAddress)
{

    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2Cx, ENABLE);

    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, slaveAddress/*<<1*/, transmissionDirection);
    if(transmissionDirection== I2C_Direction_Transmitter)
    {
    	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    if(transmissionDirection== I2C_Direction_Receiver)
    {
    	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_ReadData(I2C_TypeDef* I2Cx)
{
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    uint8_t data;
    data = I2C_ReceiveData(I2Cx);
    return data;
}
