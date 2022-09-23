#include "spi.h"
#include "delay.h"

#define Dummy_Byte    0xFF
/*
	函数名：void SPI_IoInit(void)
	输入： 无
	输出： 无
	功能： 初始化SPI接口管脚
*/
void SPI_IoInit(void)
{                         
	GPIO_InitTypeDef GPIO_InitStructure;
	// 打开端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	// 初始化MOSI和NSS管脚，推挽输出
    GPIO_InitStructure.GPIO_Pin = mosi_pin|nss_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(mosi_port, &GPIO_InitStructure);
	// 初始化SCK管脚，推挽输出
    GPIO_InitStructure.GPIO_Pin = sck_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(sck_port, &GPIO_InitStructure);
	// 初始化MISO管脚，上拉输入
    GPIO_InitStructure.GPIO_Pin = miso_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
    GPIO_Init(miso_port, &GPIO_InitStructure);
	// 初始化管脚输出电平，SCK和NSS为高电平
    SPI_NSS_1;
    SPI_SCK_1;
}
/*
	函数名：u8 SPI_ReadWriteByte(u8 txData)
	输入： txData：写入数据
	输出： rxData：读取数据
	功能： 和从机交换一字节数据
*/
u8 SPI_ReadWriteByte(u8 txData)
 {
     u8 i;
     u8 rxData = 0;
 
     for(i = 0; i < 8; i++)
     {
         SPI_SCK_0;
         delay_us(1);

         if(txData & 0x80){        
             SPI_MOSI_1;
         }else{
             SPI_MOSI_0;
         }
         txData <<= 1;
         delay_us(1);
 
         SPI_SCK_1;
         delay_us(1);

         rxData <<= 1;
         if(SPI_READ_MISO){
             rxData |= 0x01;
         }
         delay_us(1);
     }
    SPI_SCK_0;
 
	return rxData;
}
/*
	函数名：u8 SPI_ReadByte(void)
	输入： 无
	输出： 一个字节数据，读取数据
	功能： 从从机读取一字节数据
*/
u8 SPI_ReadByte(void)
{
	return SPI_ReadWriteByte(Dummy_Byte);
}
/*
	函数名：void SPI_WriteByte(u8 txData)
	输入： 一个字节数据，写入数据
	输出： 无
	功能： 向从机写入一字节数据
*/
void SPI_WriteByte(u8 txData)
{
	(void)SPI_ReadWriteByte(txData);
}


