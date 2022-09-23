#include "bh1750.h"
#include "delay.h"

#define slave_addr 0x23

void    BH1750_IIC_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

		BH1750_SCL_0;
		BH1750_SDA_0;

	 GPIO_InitStruct.GPIO_Pin = BH1750_SCL_PIN|BH1750_SDA_PIN;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP ;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStruct);

    BH1750_SCL_1;
    BH1750_SDA_1;
}
void    BH1750_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

	 GPIO_InitStruct.GPIO_Pin = BH1750_SDA_PIN;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP ;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(BH1750_SDA_PORT, &GPIO_InitStruct);
}
void    BH1750_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

     GPIO_InitStruct.GPIO_Pin = BH1750_SDA_PIN;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU ;
     GPIO_Init(BH1750_SDA_PORT, &GPIO_InitStruct);
}
void    BH1750_IIC_START(void)
{
    BH1750_SDA_OUT();
    BH1750_SCL_1;
    BH1750_SDA_1;
    delay_us(5);
    BH1750_SDA_0;
    delay_us(6);
    BH1750_SCL_0;
}
void    BH1750_IIC_STOP(void)
{
    BH1750_SDA_OUT();
    BH1750_SCL_0;
    BH1750_SDA_0;
    BH1750_SCL_1;
    delay_us(6);
    BH1750_SDA_1;
    delay_us(6);
}
uint8_t BH1750_IIC_WaitACK(void)
{
    uint8_t temptime=0;

    BH1750_SDA_1;
    delay_us(1);
    BH1750_SDA_IN();
    BH1750_SCL_1;
    delay_us(1);
    while(BH1750_SDA_READ)
    {
        temptime++;
        if (temptime>250)
        {
            BH1750_IIC_STOP();
            return 1;
        }
    }
    BH1750_SCL_0;
    return 0;
}
void    BH1750_IIC_ACK(void)
{
    BH1750_SCL_0;
    BH1750_SDA_OUT();
    BH1750_SDA_0;
    delay_us(2);
    BH1750_SCL_1;
    delay_us(5);
    BH1750_SCL_0;
}
void    BH1750_IIC_NACK(void)
{
    BH1750_SCL_0;
    BH1750_SDA_OUT();
    BH1750_SDA_1;
    delay_us(2);
    BH1750_SCL_1;
    delay_us(5);
    BH1750_SCL_0;
}
void    BH1750_IIC_SendByte(uint8_t data)
{
    uint8_t i;

    BH1750_SDA_OUT();
    BH1750_SCL_0;
    for(i=0;i<8;i++)
    {
        if(data&0x80)
            BH1750_SDA_1;
        else
            BH1750_SDA_0;
        
        delay_us(2);
        BH1750_SCL_1;
				delay_us(2);
				data<<=1;
        BH1750_SCL_0;
        delay_us(2);
    }
}
uint8_t BH1750_IIC_ReadByte(uint8_t ack)
{
    uint8_t i,receive=0;

    BH1750_SDA_IN();
    for(i=0;i<8;i++)
    {
        BH1750_SCL_0;
        delay_us(2);
        BH1750_SCL_1;
        receive<<=1;
        if(BH1750_SDA_READ) receive++;
        delay_us(1);
    }
    if(!ack)
        BH1750_IIC_NACK();
    else
        BH1750_IIC_ACK();
    return receive;
}

float BH1750_Read_data(void)
{
	
	u8 hbyte=0, lbyte=0;
	
	BH1750_IIC_SendByte(0x01);
	
	BH1750_IIC_START();
	BH1750_IIC_SendByte((slave_addr<<1)|0x00);
	
	while(BH1750_IIC_WaitACK());
	
	BH1750_IIC_SendByte(0x23);//One time L-resolution mode
	
	while(BH1750_IIC_WaitACK());
	
	BH1750_IIC_STOP();
	
	delay_ms(30);
	
	BH1750_IIC_START();
	BH1750_IIC_SendByte((slave_addr<<1)|0x01);
	
	while(BH1750_IIC_WaitACK());
	
	hbyte = BH1750_IIC_ReadByte(1);
	
	lbyte = BH1750_IIC_ReadByte(0);
	
	BH1750_IIC_STOP();
	
	return ((hbyte<<8)+lbyte)/1.2;
	
}


