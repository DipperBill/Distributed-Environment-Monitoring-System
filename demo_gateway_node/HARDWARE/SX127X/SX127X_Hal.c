/**			MCUΪSTM32F103Z
  *        PC7  ---> DIO0
  *        TXE  //	״ָ̬ʾ
  *        RXE	//	״ָ̬ʾ	
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
//�ò��ֺ���Ϊϵͳ�õ���GPIO�ĳ�ʼ���������û������Լ���ƽ̨��Ӧ�޸�
//--------------------------------------------------------------//

/**
  * @��飺�ú���ΪDIO0�����ʼ�����жϡ����ȼ����ã�
  * @��������	DIO0--PC7
  * @����ֵ����
  */
void SX127X_DIO0_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStructure;
//	__GPIOB_CLK_ENABLE();
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PC�˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PB.5 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;				//��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5

	
	

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);	
	
	//printf ("DIO0��ʼ���ɹ�\r\n");
//	GPIO_InitStruct.Pin = GPIO_Pin_7;
//	GPIO_InitStruct.Mode = GPIO_Mode_IPU;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
}
/**
  * @��飺�ú���ΪDIO0�����жϿ���ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO0_INTENABLE()
{
//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);	
}
/**
  * @��飺�ú���ΪDIO0�����жϹر�ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO0_INTDISABLE()
{
//	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);		
}
/**
  * @��飺�ú���ΪDIO0����״̬��ȡ��
  * @��������
  * @����ֵ��DIO0״̬"1"or"0"
  */
BitAction SX127X_DIO0_GetState()
{
//	GPIO_PinState State;
	BitAction State;
    State = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
    return State;
}
/**
  * @��飺�ú���ΪDIO0����жϱ�־λ��
  * @������BitAction State
  * @����ֵ��DIO0״̬"1"or"0"
  */
void EXTI9_5_IRQHandler(void)
{

	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)==1)	  //����KEY2
	{
		EXTI_ClearITPendingBit(EXTI_Line7);  //���LINE2�ϵ��жϱ�־λ 
	}
	 


}
/**
  * @��飺�ú���ΪDIO1�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
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
  * @��飺�ú���ΪDIO2�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
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
  * @��飺�ú���Ϊ��Ƶ����TXE������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
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
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PE.5 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
	
    GPIO_WriteBit(GPIOE, GPIO_Pin_5, PinState);
}
/**
  * @��飺�ú���Ϊ��Ƶ����RXE������ƣ�
  * @������PinStateΪ"1"��ʾ��                                                                                                                                                                                     ���ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
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
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
		
	
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, PinState);
}
/**
  * @��飺�ú���ΪSPI��Ƭѡ����NSS������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_NSS_OUTPUT(BitAction PinState)
{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, PinState);
	GPIO_WriteBit(nss_port, nss_pin, PinState);
}
/**
  * @��飺�ú���ΪSX127X��λ����NRST������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_RESET_OUTPUT(BitAction PinState)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    __HAL_RCC_GPIOA_CLK_ENABLE();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = reset_pin;				 //LED0-->PB.5 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(reset_port, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
	
	GPIO_WriteBit(reset_port, reset_pin, PinState);				 //PC8���

	//printf ("��λ �ߵ�ƽ�ɹ�\r\n");
//	GPIO_InitStruct.Pin = GPIO_Pin_8;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, PinState);
}

//-----------------------------SPI-----------------------------//
//�ò��ֺ���ΪMCU��SX127Xģ��SPIͨ�Ų��֣�����SPI�ڼ����ó�ʼ��
//--------------------------------------------------------------//

/**
  * @��飺�ú�������MCU��SPI��ӦIO�ڳ�ʼ����
  * @��������
  * @����ֵ����
  */
void SX127X_SPIGPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTAʱ��ʹ�� 
	
	GPIO_InitStructure.GPIO_Pin = mosi_pin| nss_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOB
	
	 GPIO_InitStructure.GPIO_Pin = miso_pin;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	 GPIO_Init(miso_port, &GPIO_InitStructure);
//	

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );//PORTBʱ��ʹ�� 

	GPIO_InitStructure.GPIO_Pin = sck_pin;  // PB12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
	printf ("SPI2 IO��ʼ���ɹ�\r\n");
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
  * @��飺�ú�������MCU��SPI���ó�ʼ����
  * @��������
  * @����ֵ����
  */
void SX127X_SPI_Init()
{
//    __HAL_RCC_GPIOA_CLK_ENABLE();//PORTAʱ��ʹ��
//    __HAL_RCC_GPIOC_CLK_ENABLE();//PORTCʱ��ʹ��
//    __HAL_RCC_SPI3_CLK_ENABLE();//SPI2ʱ��ʹ��
	
	SPI_InitTypeDef  SPI_InitStructure;

    SX127X_SPIGPIO_Init();
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2ʱ��ʹ�� 	

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	printf ("SPI2 ��ʼ���ɹ�\r\n");

//    SPI3_InitStruct.Instance = SPI3; //ʹ��SPI3
//    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPIģʽ������ģʽ
//    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//����ȫ˫��
//    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//���ݿ�ȣ�8λ
//    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //����ͬ����ʱ�ӿ���Ϊ����ʱ��
//    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0ģʽ
//    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSSD���������
//    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//�����ʷ�Ƶ��8��Ƶ
//    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//���ݴ�MSB��ʼ
//    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
//    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRCУ�鲻ʹ��
//    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRCֵ����Ķ���ʽ
//    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

//    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)

//    {
//        while(1);
//    }
//    __HAL_SPI_ENABLE(&SPI3_InitStruct);
}

//-----------------------SX127X Read and Write-------------------//
//�ò��ֺ���ΪMCU��SX127Xģ��Ĵ������ж�д
//--------------------------------------------------------------//

/**
  * @��飺SX127X  ��Ĵ�����ַ������������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ����
  */
unsigned char SX127X_ReadWriteByte(unsigned char data)
{
//    unsigned char RxDat;
//    HAL_SPI_TransmitReceive(&SPI3_InitStruct, &data, &RxDat, 1, 1000);
//    return RxDat;
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, data); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����	
}
/**
  * @��飺SX127X  ��Ĵ�����ַ������������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ����
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
  * @��飺SX127X  ��Ĵ�����ַ����������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ�����ݷ��ص�*buffer��
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



