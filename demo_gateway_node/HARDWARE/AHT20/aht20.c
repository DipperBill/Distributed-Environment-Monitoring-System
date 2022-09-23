
#include "aht20.h"



void Delay_N10us(uint32_t t)//��ʱ����
{
  uint32_t k;

   while(t--)
  {
    for (k = 0; k < 2; k++);//110
  }
}

void SensorDelay_us(uint32_t t)//��ʱ����
{
		
	for(t = t-2; t>0; t--)
	{
		Delay_N10us(1);
	}
}

void DELAY_us(u16 t)
{
	u16 i;
	for(i=0;i<t;i++)
		Delay_N10us(1);
}

void Delay_1ms(uint32_t t)		//��ʱ����
{
   while(t--)
  {
    SensorDelay_us(1000);//////��ʱ1ms
  }
}


void    AHT20_IIC_GPIO_INIT(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

		AHT20_SCL_0;
		AHT20_SDA_0;

	 GPIO_InitStruct.GPIO_Pin = AHT20_SCL_PIN|AHT20_SDA_PIN;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP ;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStruct);

    AHT20_SCL_1;
    AHT20_SDA_1;
}
void    AHT20_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

	 GPIO_InitStruct.GPIO_Pin = AHT20_SDA_PIN;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP ;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(AHT20_SDA_PORT, &GPIO_InitStruct);
}
void    AHT20_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

     GPIO_InitStruct.GPIO_Pin = AHT20_SDA_PIN;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU ;
     GPIO_Init(AHT20_SDA_PORT, &GPIO_InitStruct);
}
void    AHT20_IIC_START(void)
{
    AHT20_SDA_OUT();
    AHT20_SCL_1;
    AHT20_SDA_1;
    DELAY_us(5);
    AHT20_SDA_0;
    DELAY_us(6);
    AHT20_SCL_0;
}
void    AHT20_IIC_STOP(void)
{
    AHT20_SDA_OUT();
    AHT20_SCL_0;
    AHT20_SDA_0;
    AHT20_SCL_1;
    DELAY_us(6);
    AHT20_SDA_1;
    DELAY_us(6);
}

u8 AHT20_IIC_WaitACK(void)
{
    uint8_t temptime=0;

    AHT20_SDA_1;
    DELAY_us(1);
    AHT20_SDA_IN();
    AHT20_SCL_1;
    DELAY_us(1);
    while(AHT20_SDA_READ)
    {
        temptime++;
        if (temptime>250)
        {
            AHT20_IIC_STOP();
            return 1;
        }
    }
    AHT20_SCL_0;
    return 0;
}
void    AHT20_IIC_ACK(void)
{
    AHT20_SCL_0;
    AHT20_SDA_OUT();
    AHT20_SDA_0;
    DELAY_us(2);
    AHT20_SCL_1;
    DELAY_us(5);
    AHT20_SCL_0;
}
void    AHT20_IIC_NACK(void)
{
    AHT20_SCL_0;
    AHT20_SDA_OUT();
    AHT20_SDA_1;
    DELAY_us(2);
    AHT20_SCL_1;
    DELAY_us(5);
    AHT20_SCL_0;
}

void    AHT20_IIC_SendByte(u8 data)
{
    uint8_t i;

    AHT20_SDA_OUT();
    AHT20_SCL_0;
    for(i=0;i<8;i++)
    {
        if(data&0x80)
            AHT20_SDA_1;
        else
            AHT20_SDA_0;
        
        DELAY_us(2);
        AHT20_SCL_1;
				DELAY_us(2);
				data<<=1;
        AHT20_SCL_0;
        DELAY_us(2);
    }
}
u8 AHT20_IIC_ReadByte(u8 ack)
{
    uint8_t i,receive=0;

    AHT20_SDA_IN();
    for(i=0;i<8;i++)
    {
        AHT20_SCL_0;
        DELAY_us(2);
        AHT20_SCL_1;
        receive<<=1;
        if(AHT20_SDA_READ) receive++;
        DELAY_us(1);
    }
    if(!ack)
        AHT20_IIC_NACK();
    else
        AHT20_IIC_ACK();
    return receive;
}

u8 AHT20_Read_Status(void)//��ȡAHT20��״̬�Ĵ���
{
	uint8_t Byte_first;	
	
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x71);
	while(AHT20_IIC_WaitACK());
	Byte_first = AHT20_IIC_ReadByte(0);
	AHT20_IIC_STOP();
	return Byte_first;
}
void AHT20_SendAC(void) //��AHT20����AC����
{

	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x70);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0xac);//0xAC�ɼ�����
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x33);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_STOP();

}



void AHT20_Read_CTdata(uint32_t *ct) //û��CRCУ�飬ֱ�Ӷ�ȡAHT20���¶Ⱥ�ʪ������
{
	u8  Byte_1th=0;
	u8  Byte_2th=0;
	u8  Byte_3th=0;
	u8  Byte_4th=0;
	u8  Byte_5th=0;
	u8  Byte_6th=0;
	u32 RetuData = 0;
	u16 cnt = 0;
	AHT20_SendAC();//��AHT10����AC����
	Delay_1ms(80);//��ʱ80ms����	
  cnt = 0;
	while(((AHT20_Read_Status()&0x80)==0x80))//ֱ��״̬bit[7]Ϊ0����ʾΪ����״̬����Ϊ1����ʾæ״̬
	{
		SensorDelay_us(1508);
		if(cnt++>=100)
		{
		 break;
		 }
	}
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x71);
	while(AHT20_IIC_WaitACK());
	Byte_1th = AHT20_IIC_ReadByte(1);//״̬�֣���ѯ��״̬Ϊ0x98,��ʾΪæ״̬��bit[7]Ϊ1��״̬Ϊ0x1C������0x0C������0x08��ʾΪ����״̬��bit[7]Ϊ0
	Byte_2th = AHT20_IIC_ReadByte(1);//ʪ��
	Byte_3th = AHT20_IIC_ReadByte(1);//ʪ��
	Byte_4th = AHT20_IIC_ReadByte(1);//ʪ��/�¶�
	Byte_5th = AHT20_IIC_ReadByte(1);//�¶�
	Byte_6th = AHT20_IIC_ReadByte(0);//�¶�
	AHT20_IIC_STOP();
/*
	RetuData = (RetuData|Byte_2th)<<8;
	RetuData = (RetuData|Byte_3th)<<8;
	RetuData = (RetuData|Byte_4th);
	RetuData =RetuData >>4;
	ct[0] = RetuData;//ʪ��
	RetuData = 0;
	RetuData = (RetuData|Byte_4th)<<8;
	RetuData = (RetuData|Byte_5th)<<8;
	RetuData = (RetuData|Byte_6th);
	RetuData = RetuData&0xfffff;
	ct[1] =RetuData; //�¶�
*/
	RetuData = (RetuData|Byte_1th)<<8;
	RetuData = (RetuData|Byte_2th)<<8;
	RetuData = (RetuData|Byte_3th);
	ct[0] = RetuData;
	RetuData = 0;
	RetuData = (RetuData|Byte_4th)<<8;
	RetuData = (RetuData|Byte_5th)<<8;
	RetuData = (RetuData|Byte_6th);
	ct[1] = RetuData;
}






void AHT20_IIC_INIT(void)   //��ʼ��AHT20
{	
	AHT20_IIC_GPIO_INIT();
	
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x70);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0xa8);//0xA8����NOR����ģʽ
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_STOP();

	Delay_1ms(10);//��ʱ10ms����

	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x70);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0xbe);//0xBE��ʼ�����AHT20�ĳ�ʼ��������0xBE,   AHT10�ĳ�ʼ��������0xE1
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x08);//��ؼĴ���bit[3]��1��ΪУ׼���
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_STOP();
	Delay_1ms(10);//��ʱ10ms����
}
void JH_Reset_REG(uint8_t addr)
{
	
	uint8_t Byte_first,Byte_second,Byte_third;
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x70);//ԭ����0x70
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(addr);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0x00);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_STOP();

	Delay_1ms(5);//��ʱ5ms����
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x71);//
	while(AHT20_IIC_WaitACK());
	Byte_first = AHT20_IIC_ReadByte(1);
	Byte_second = AHT20_IIC_ReadByte(1);
	Byte_third = AHT20_IIC_ReadByte(0);
	AHT20_IIC_STOP();
	
  Delay_1ms(10);//��ʱ10ms����
	AHT20_IIC_START();
	AHT20_IIC_SendByte(0x70);///
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(0xB0|addr);////�Ĵ�������
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(Byte_second);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_SendByte(Byte_third);
	while(AHT20_IIC_WaitACK());
	AHT20_IIC_STOP();
	
	Byte_second=0x00;
	Byte_third =0x00;
}

void AHT20_Start_Init(void)
{
	JH_Reset_REG(0x1b);
	JH_Reset_REG(0x1c);
	JH_Reset_REG(0x1e);
}








void AHT20_INIT(void)
{
	AHT20_IIC_INIT();
	Delay_1ms(500);
	
	if((AHT20_Read_Status()&0x18)!=0x18)
	{
	AHT20_Start_Init(); //���³�ʼ���Ĵ���
	Delay_1ms(10);
	}	
}

