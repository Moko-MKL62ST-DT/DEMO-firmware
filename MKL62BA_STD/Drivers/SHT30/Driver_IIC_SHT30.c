////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Copyright 2016 ���ڽݿ�Ƽ����޹�˾
// All rights reserved.
//
// Filename     �fDriver_SD.c
// Version      ��1.0
// Author       ��zhangjun
// Accomplished date �� 2016.7.1
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#ifndef LIB_DRIVER_IIC_SHT30
#define LIB_DRIVER_IIC_SHT30
#include "main.h"
#include "gpio.h"
#endif

#define PORT_IIC_SHT30_SDA    GPIOB
#define PORT_IIC_SHT30_SCL    GPIOB

#define PIN_IIC_SHT30_SDA     GPIO_PIN_11
#define PIN_IIC_SHT30_SCL     GPIO_PIN_10

#define SDA_SHT30_H    HAL_GPIO_WritePin(PORT_IIC_SHT30_SDA, PIN_IIC_SHT30_SDA, GPIO_PIN_SET)       
#define SDA_SHT30_L    HAL_GPIO_WritePin(PORT_IIC_SHT30_SDA, PIN_IIC_SHT30_SDA, GPIO_PIN_RESET)       

#define SCL_SHT30_H    HAL_GPIO_WritePin(PORT_IIC_SHT30_SCL, PIN_IIC_SHT30_SCL, GPIO_PIN_SET)       
#define SCL_SHT30_L    HAL_GPIO_WritePin(PORT_IIC_SHT30_SCL, PIN_IIC_SHT30_SCL, GPIO_PIN_RESET) 

#define SDA_SHT30_read    HAL_GPIO_ReadPin(PORT_IIC_SHT30_SDA,PIN_IIC_SHT30_SDA)

#define IIC_DELAY	2


 
//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	  I2C_SHT30_Init 
// Function capacity��  I2C_SHT30��ʼ��
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void I2C_SHT30_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct; 

    GPIO_InitStruct.Pin = PIN_IIC_SHT30_SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PORT_IIC_SHT30_SDA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PIN_IIC_SHT30_SCL;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PORT_IIC_SHT30_SCL, &GPIO_InitStruct);
    
//    SCL_SHT30_H;
//    SDA_SHT30_H;
	delay_us(IIC_DELAY/2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void SDA_SHT30_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PIN_IIC_SHT30_SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PORT_IIC_SHT30_SDA, &GPIO_InitStruct);        
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void SDA_SHT30_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PIN_IIC_SHT30_SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull =GPIO_PULLUP;         //����   
    HAL_GPIO_Init(PORT_IIC_SHT30_SDA, &GPIO_InitStruct);
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	  IIC_Start 
// Function capacity��  IIC��ʼ�ź�
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned char I2C_SHT30_Start(void)
{
 	SDA_SHT30_OUT();      //sda�����
	SDA_SHT30_H;	  	  
	SCL_SHT30_H;
	delay_us(IIC_DELAY);
 	SDA_SHT30_L;           
	delay_us(IIC_DELAY);
	SCL_SHT30_L;          //ǯסI2C_SHT30���ߣ�׼�����ͻ�������� 	
	return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	  IIC_Stop 
// Function capacity��  IICֹͣ�ź�
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void I2C_SHT30_Stop(void)
{
 	SDA_SHT30_OUT();      //sda�����

	SCL_SHT30_L;
	SDA_SHT30_L;  
 	delay_us(IIC_DELAY);
	
	SCL_SHT30_H;
	SDA_SHT30_H;          
 	delay_us(IIC_DELAY);	         							   	
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  ����ACKӦ��
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void I2C_SHT30_Ack(void)
{
	SCL_SHT30_L;
 	SDA_SHT30_OUT();      //sda�����
	SDA_SHT30_L;	
	delay_us(IIC_DELAY);
	SCL_SHT30_H;
 	delay_us(IIC_DELAY);
	SCL_SHT30_L;
} 

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  ������ACKӦ��
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� void
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void I2C_SHT30_NAck(void)
{
	SCL_SHT30_L;
 	SDA_SHT30_OUT();      //sda�����
	SDA_SHT30_H;
 	delay_us(IIC_DELAY);
	SCL_SHT30_H;
 	delay_us(IIC_DELAY);
	SCL_SHT30_L;
} 

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  �ȴ�ACKӦ��
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� 1��ACK,=0��ACK
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned char I2C_SHT30_WaitAck(void) 	 
{
    unsigned char ucErrTime = 0;

 	SDA_SHT30_IN();      //SDA����Ϊ����

	SDA_SHT30_H;
	delay_us(1);	
	SCL_SHT30_H;	
 	delay_us(IIC_DELAY);
	while(SDA_SHT30_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_SHT30_Stop();
			return 1;
		}
	}
	SCL_SHT30_L;
 	delay_us(IIC_DELAY);
	return 0;
}
 
//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  IIC����һ���ֽ�
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� 
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
void I2C_SHT30_SendByte(unsigned char SendByte) 
{
    unsigned char i;
 	SDA_SHT30_OUT();      //sda�����
	
	SCL_SHT30_L;          //����ʱ�ӿ�ʼ���ݴ���	
	delay_us(IIC_DELAY); 
    for(i=0;i<8;i++)
    {  		
        if(SendByte&0x80)
            SDA_SHT30_H;  
        else 
            SDA_SHT30_L;   
        SendByte<<=1;
		
        delay_us(IIC_DELAY/2);				
        SCL_SHT30_H;
		delay_us(IIC_DELAY);		
		SCL_SHT30_L;          //����ʱ�ӿ�ʼ���ݴ���	
		delay_us(IIC_DELAY/2); 		
    }	
}  

//////////////////////////////////////////////////////////////////////////////////////////////
// Function name��	   
// Function capacity��  IIC����һ���ֽ�
// Parameter declare��
// parameter 1�� void
//	......
// Return value�� 
// Designed idea�� 
// Author��	fengchao
// Creation date�� 2016.10.27
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned char I2C_SHT30_ReadByte(unsigned char ack)
{
    unsigned char i;
    unsigned char ReceiveByte=0; 
	
 	SDA_SHT30_IN();//SDA����Ϊ����
	delay_us(IIC_DELAY);
    for(i=0;i<8;i++ )
	{
        SCL_SHT30_L; 
        delay_us(IIC_DELAY);
				SCL_SHT30_H;
        ReceiveByte<<=1;
        if(SDA_SHT30_read)ReceiveByte++;   
        delay_us(IIC_DELAY);
    }

    if (!ack)
        I2C_SHT30_NAck();//����nACK
    else
        I2C_SHT30_Ack(); //����ACK	
    return ReceiveByte;
}
