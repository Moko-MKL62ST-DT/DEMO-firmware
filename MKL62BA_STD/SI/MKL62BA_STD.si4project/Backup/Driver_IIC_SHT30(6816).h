////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Copyright 2016 ���ڽݿ�Ƽ����޹�˾
// All rights reserved.
//
// Filename     ��Driver_GPRS.h
// Version      ��1.0
// Author       ��zhangjun
// Accomplished date �� 2016.7.1
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#ifndef Driver_IIC_SHT30_H
#define Driver_IIC_SHT30_H
//*--------------------------------------------------------------------------------------
//*���Ŷ���
//*--------------------------------------------------------------------------------------
#endif 
#define  LIB_DRIVER_IIC_SHT30
#include "hw.h"

#ifdef LIB_DRIVER_IIC_SHT30

//IIC��ʼ��
void I2C_SHT30_Init(void);

//I2C_SHT30�����ź�
unsigned char I2C_SHT30_Start(void);

//I2C_SHT30ֹͣ�ź�
void I2C_SHT30_Stop(void);

//IIC����ACKӦ���ź�
void I2C_SHT30_Ack(void);

//IIC������ACKӦ���ź�
void I2C_SHT30_NAck(void);

//IIC�ȴ�IIC��ACKӦ���ź�
unsigned char I2C_SHT30_WaitAck(void); 	 

//IIC����һ���ֽ�
void I2C_SHT30_SendByte(unsigned char SendByte) ;

//IIC��ȡһ���ֽ�
unsigned char I2C_SHT30_ReadByte(unsigned char ack);

//SHT30 IICдһ���ֽ�
unsigned char SHT30_WriteByte(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     

//SHT30 IIC��һ���ֽ�
unsigned char SHT30_ReadByte(unsigned char SlaveAddress,unsigned char REG_Address);

void ReadTemHumTaskInit(void);

void Read_tempHum_valu();
int Get_sht30HumValu(void);
int Get_sht30TemValu(void);
#else

//IIC��ʼ��
extern void I2C_SHT30_Init(void);

//I2C_SHT30�����ź�
extern unsigned char I2C_SHT30_Start(void);

//I2C_SHT30ֹͣ�ź�
extern void I2C_SHT30_Stop(void);

//IIC����ACKӦ���ź�
extern void I2C_SHT30_Ack(void);

//IIC������ACKӦ���ź�
extern void I2C_SHT30_NAck(void);

//IIC�ȴ�IIC��ACKӦ���ź�
extern unsigned char I2C_SHT30_WaitAck(void); 	 

//IIC����һ���ֽ�
extern void I2C_SHT30_SendByte(unsigned char SendByte) ;

//IIC��ȡһ���ֽ�
extern unsigned char I2C_SHT30_ReadByte(unsigned char ack);

//SHT30 IICдһ���ֽ�
extern unsigned char SHT30_WriteByte(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);	 

//SHT30 IIC��һ���ֽ�
extern unsigned char SHT30_ReadByte(unsigned char SlaveAddress,unsigned char REG_Address);

#endif
