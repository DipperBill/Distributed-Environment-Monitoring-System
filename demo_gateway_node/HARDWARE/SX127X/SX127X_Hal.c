/**			MCU为STM32F103Z
  *        PC7  ---> DIO0
  *        TXE  //	状态指示
  *        RXE	//	状态指示	
  *        PC8  ---> RST
  *        PC6 ---> NSS
  *        PB15 ---> M0SI
  *        PB14 ---> MISO
  *        PB13 ---> SCK
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "SX127X_Hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"

//#include "stm32l4xx_hal.h"
//SPI_HandleTypeDef SPI3_InitStruct;

//-----------------------------GPIO-----------------------------//
//该部分函数为系统用到的GPIO的初始化函数，用户根据自己的平台相应修改
//--------------------------------------------------------------//

/**
  * @简介：该函数为DIO0输入初始化及中断、优先级配置；
  * @参数：无	DIO0--PC7
  * @返回值：无
  */
void SX127X_DIO0_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStructure;
//	__GPIOB_CLK_ENABLE();
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PB.5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;				//下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5

	
	

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);	
	
	//printf ("DIO0初始化成功\r\n");
//	GPIO_InitStruct.Pin = GPIO_Pin_7;
//	GPIO_InitStruct.Mode = GPIO_Mode_IPU;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
}
/**
  * @简介：该函数为DIO0输入中断开启使能；
  * @参数：无
  * @返回值：无
  */
void SX127X_DIO0_INTENABLE()
{
//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);	
}
/**
  * @简介：该函数为DIO0输入中断关闭使能；
  * @参数：无
  * @返回值：无
  */
void SX127X_DIO0_INTDISABLE()
{
//	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);		
}
/**
  * @简介：该函数为DIO0输入状态获取；
  * @参数：无
  * @返回值：DIO0状态"1"or"0"
  */
BitAction SX127X_DIO0_GetState()
{
//	GPIO_PinState State;
	BitAction State;
    State = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
    return State;
}
/**
  * @简介：该函数为DIO0清除中断标志位；
  * @参数：BitAction State
  * @返回值：DIO0状态"1"or"0"
  */
void EXTI9_5_IRQHandler(void)
{

	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)==1)	  //按键KEY2
	{
		EXTI_ClearITPendingBit(EXTI_Line7);  //清除LINE2上的中断标志位 
	}
	 


}
/**
  * @简介：该函数为DIO1输入初始化及中断、优先级配置；
  * @参数：无
  * @返回值：无
  */
//void SX127X_DIO1_INPUT()
//{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
//    //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    //HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
//}
/**
  * @简介：该函数为DIO2输入初始化及中断、优先级配置；
  * @参数：无
  * @返回值：无
  */
//void SX127X_DIO2_INPUT()
//{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_2;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    //HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
//}
/**
  * @简介：该函数为高频开关TXE输出控制；
  * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
  * @返回值：无
  */
void SX127X_TXE_OUTPUT(BitAction PinState)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PE.5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
	
    GPIO_WriteBit(GPIOE, GPIO_Pin_5, PinState);
}
/**
  * @简介：该函数为高频开关RXE输出控制；
  * @参数：PinState为"1"表示输                                                                                                                                                                                     出高电平，"0"输出低电平；
  * @返回值：无
  */
void SX127X_RXE_OUTPUT(BitAction PinState)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
		
	
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, PinState);
}
/**
  * @简介：该函数为SPI的片选引脚NSS输出控制；
  * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
  * @返回值：无
  */
void SX127X_NSS_OUTPUT(BitAction PinState)
{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, PinState);
	GPIO_WriteBit(nss_port, nss_pin, PinState);
}
/**
  * @简介：该函数为SX127X复位引脚NRST输出控制；
  * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
  * @返回值：无
  */
void SX127X_RESET_OUTPUT(BitAction PinState)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    __HAL_RCC_GPIOA_CLK_ENABLE();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PB,PE端口时钟
	GPIO_InitStructure.GPIO_Pin = reset_pin;				 //LED0-->PB.5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(reset_port, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
	
	GPIO_WriteBit(reset_port, reset_pin, PinState);				 //PC8输出

	//printf ("复位 高电平成功\r\n");
//	GPIO_InitStruct.Pin = GPIO_Pin_8;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, PinState);
}

//-----------------------------SPI-----------------------------//
//该部分函数为MCU对SX127X模块SPI通信部分，包含SPI口及配置初始化
//--------------------------------------------------------------//

/**
  * @简介：该函数用于MCU对SPI对应IO口初始化；
  * @参数：无
  * @返回值：无
  */
void SX127X_SPIGPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTA时钟使能 
	
	GPIO_InitStructure.GPIO_Pin = mosi_pin| nss_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB
	
	 GPIO_InitStructure.GPIO_Pin = miso_pin;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	 GPIO_Init(miso_port, &GPIO_InitStructure);
//	

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );//PORTB时钟使能 

	GPIO_InitStructure.GPIO_Pin = sck_pin;  // PB12 推挽 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
	printf ("SPI2 IO初始化成功\r\n");
//    GPIO_InitTypeDef GPIO_InitStruct;
//    /* Configure the SX126X_NSS pin */
//    GPIO_InitStruct.Pin = GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /* SPI SCK GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = GPIO_PIN_10;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//    /* SPI MISO GPIO pin configuration  */
//    GPIO_InitStruct.Pin = GPIO_PIN_11;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    /* SPI MoSi GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = GPIO_PIN_12;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
/**
  * @简介：该函数用于MCU对SPI配置初始化；
  * @参数：无
  * @返回值：无
  */
void SX127X_SPI_Init()
{
//    __HAL_RCC_GPIOA_CLK_ENABLE();//PORTA时钟使能
//    __HAL_RCC_GPIOC_CLK_ENABLE();//PORTC时钟使能
//    __HAL_RCC_SPI3_CLK_ENABLE();//SPI2时钟使能
	
	SPI_InitTypeDef  SPI_InitStructure;

    SX127X_SPIGPIO_Init();
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能 	

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
	printf ("SPI2 初始化成功\r\n");

//    SPI3_InitStruct.Instance = SPI3; //使用SPI3
//    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPI模式：主机模式
//    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//两线全双工
//    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//数据宽度：8位
//    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //串行同步字时钟控制为低速时钟
//    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0模式
//    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSSD由软件管理
//    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//波特率分频，8分频
//    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//数据从MSB开始
//    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
//    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRC校验不使能
//    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRC值计算的多项式
//    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

//    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)

//    {
//        while(1);
//    }
//    __HAL_SPI_ENABLE(&SPI3_InitStruct);
}

//-----------------------SX127X Read and Write-------------------//
//该部分函数为MCU对SX127X模块寄存器进行读写
//--------------------------------------------------------------//

/**
  * @简介：SX127X  向寄存器地址连续发送数据
  * @参数：uint8_t addr,寄存器地址 uint8_t *buffer,发送数组指针 uint8_t size指针长度
  * @返回值：无
  */
unsigned char SX127X_ReadWriteByte(unsigned char data)
{
//    unsigned char RxDat;
//    HAL_SPI_TransmitReceive(&SPI3_InitStruct, &data, &RxDat, 1, 1000);
//    return RxDat;
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, data); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据	
}
/**
  * @简介：SX127X  向寄存器地址连续发送数据
  * @参数：uint8_t addr,寄存器地址 uint8_t *buffer,发送数组指针 uint8_t size指针长度
  * @返回值：无
  */
void SX127X_WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    //SX127X_NSS_OUTPUT(Bit_RESET);
		SPI_NSS_0;
	
    //SX127X_ReadWriteByte(addr | 0x80);
		SPI_WriteByte(addr|0x80);
	
    for( i = 0; i < size; i++ )
    {
        //SX127X_ReadWriteByte(buffer[i]);
			SPI_WriteByte(buffer[i]);
    }
    //SX127X_NSS_OUTPUT(Bit_SET);
		SPI_NSS_1;
}
/**
  * @简介：SX127X  向寄存器地址连续读数据
  * @参数：uint8_t addr,寄存器地址 uint8_t *buffer,发送数组指针 uint8_t size指针长度
  * @返回值：数据返回到*buffer中
  */
void SX127X_ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    //SX127X_NSS_OUTPUT(Bit_RESET);
		SPI_NSS_0;
	
    //SX127X_ReadWriteByte(addr & 0x7F);
	SPI_WriteByte(addr&0x7F);
	
    for( i = 0; i < size; i++ )
    {
        //buffer[i] = SX127X_ReadWriteByte(0x00);
			buffer[i] = SPI_ReadByte();
    }

    SPI_NSS_1;
		//SX127X_NSS_OUTPUT(Bit_SET);
}



