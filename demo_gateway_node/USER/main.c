/**********************************************************
    课程：     《智能电子设备开发》
    时间：     2021年11月1日
    实验简介：
            本实验为LoRa通信实验，有一个网关和四个节点组成。
            节点通过IIC协议读取AHT20温湿度传感器、BH1750光
        照度传感器数值，通过ADC读取光敏电阻分压，通过SPI接口
        与LoRa模块通信进行数据的接收和发送。
            网关通过SPI接口与LoRa模块通信进行数据的接收与发
        送，通过串口1和串口2分别与PC和ESP32模块通信。
    MCU：STM32103C6T6
***********************************************************/

#include "delay.h"
#include "sys.h"
 
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "SX127X_Driver.h"
#include "main.h"

#if NODE
#include "bh1750.h"
#include "AHT20-21_DEMO_V1_3.h" 
#include "adc.h"

#define NODE_NUMBER 5


#else
#include "usart.h"
#include "tcp.h"
#include "esp8266.h"
#include "mqtt.h"
#endif

// LoRa通信
#define TX_ING 0
#define RX_ING 1
u8 state;
u8 SF;
u32 Fre[5] = {471300000, 494600000, 510000000, 868000000, 915000000};

// 接收和发送数据缓冲区
u8 RXbuffer[20];
u8 TXbuffer[20];

u8 current_node = 0;    //当前节点0,1,2,3
u8 temp;    //存储中断标志

#if NODE
// 节点测量值存储变量
float gzd;
u16 adc_value;
u32 CT_data[2];
#else
// ESP32发送变量
u8 res;
char str[100]={0};

// 网关存储变量
u8 shidu;
u8 wendu;
u16 adc;
float guang;
// 网关定时发送测量指令
u16 tx_t1=0;
u8 tx_t2=0;
#endif

// 修改发送缓冲数组
void change_TX(float gzd, u32* CTdata, u16 adc);

#if NODE        //节点
int main(void)
 {	 
    //延时函数初始化
    delay_init();
    //中断优先级分组，分2组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre[0];        //中心频率470MHz
    G_LoRaConfig.BandWidth = BW125KHZ;      //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = SF09;    //SF = 9
    G_LoRaConfig.CodingRate = CR_4_6;       //CR = 4/6
    G_LoRaConfig.PowerCfig = 15;            //19±1dBm
    G_LoRaConfig.MaxPowerOn = true;         //最大功率开启
    G_LoRaConfig.CRCON = true;              //CRC校验开启
    G_LoRaConfig.ExplicitHeaderOn = true;   //Header开启
    G_LoRaConfig.PayloadLength = 100;       //数据包长度

    if(SX127X_Lora_init() != NORMAL){       //无线模块初始化
        while(1)
        {
         		//printf ("初始化失败\r\n");	
        }
    }

    //处理SF显示
    switch(G_LoRaConfig.SpreadingFactor) {
    case SF06:
        SF = 6;
        break;
    case SF07:
        SF = 7;
        break;
    case SF08:
        SF = 8;
        break;
    case SF09:
        SF = 9;
        break;
    case SF10:
        SF = 10;
        break;
    case SF11:
        SF = 11;
        break;
    case SF12:
        SF = 12;
        break;
    }

	// 传感器、ADC初始化
    BH1750_IIC_INIT();
    AHT20_Init();
    ADCx_Init();
	if((AHT20_Read_Status()&0x18)!=0x18){
        AHT20_Start_Init();
        Delay_1ms(10);
	}

    /*
        节点默认为接收状态，接收到测量信号（为节点编号），若与自身编号一致，转发送状态
        发送状态进行数据测量以及发送，发送完转接收状态
    */
    state = RX_ING;		
    SX127X_StartRx();
    while (1){
        switch(state){
            case TX_ING:    //发送状态
                gzd = BH1750_Read_data();               //读取光照度信息
                AHT20_Read_CTdata(CT_data);             //读取温湿度信息
                adc_value = Get_ADC_Value(1,5);         //读取光敏电阻ADC

                change_TX(gzd, CT_data, adc_value);     //修改发送缓冲数组，准备发送

                SX127X_TxPacket(TXbuffer);              //发送数据包
                Delay_1ms(1000);

                SX127X_StartRx();                       //转为接收状态
                state = RX_ING;
            break;

            case RX_ING:    //接收状态
                
                SX127X_Read(REG_LR_IRQFLAGS, &temp);    //读取中断标志
                if(temp == 0x40){                       //进入中断（接收到信号）
                    SX127X_ReadFifo(RXbuffer, 1);       //读取接收信号

                    if(RXbuffer[0] == NODE_NUMBER){     //对比自身节点编号，若一致则转发送状态
                        state = TX_ING;
                    }else{                              //不一致继续接收
                        SX127X_StartRx();
                        Delay_1ms(1000);
                    }
                }else{                                  //未进入中断，继续接收
                    Delay_1ms(500);
                }
            break;
        }
    }
}
#else
int main(void)
{	 
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart1_Init(115200);
	ESP8266_Init(115200);

    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre[0];        //中心频率470MHz
    G_LoRaConfig.BandWidth = BW125KHZ;      //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = SF09;    //SF = 9
    G_LoRaConfig.CodingRate = CR_4_6;       //CR = 4/6
    G_LoRaConfig.PowerCfig = 15;            //19±1dBm
    G_LoRaConfig.MaxPowerOn = true;         //最大功率开启
    G_LoRaConfig.CRCON = true;              //CRC校验开启
    G_LoRaConfig.ExplicitHeaderOn = true;   //Header开启
    G_LoRaConfig.PayloadLength = 100;       //数据包长度

    if(SX127X_Lora_init() != NORMAL){       //无线模块初始化
        while(1){
            printf ("failed\r\n");	
        }
    }

    //处理SF显示
    switch(G_LoRaConfig.SpreadingFactor) {
    case SF06:
    SF = 6;
    break;
    case SF07:
    SF = 7;
    break;
    case SF08:
    SF = 8;
    break;
    case SF09:
    SF = 9;
    break;
    case SF10:
    SF = 10;
    break;
    case SF11:
    SF = 11;
    break;
    case SF12:
    SF = 12;
    break;
    }

    /*
        网关默认发送状态，发送当前查询节点编号，发送结束转为接收状态
        接收状态等待节点回应，若有回应则处理数据并由WiFI发送给PC（字符串模式）
        若无回应，等待一段时间转发送，再次查询
        若查询2次未回应，查询下一个节点。
    */
    state = TX_ING;
    tx_t1 = 0;tx_t2 = 0;

    //ESP32初始化
	ESP8266_STA_TCPClient_Test();
		
    while (1)
    {
        switch(state){
            case RX_ING:        //接收模式
                
                SX127X_Read(REG_LR_IRQFLAGS, &temp);    //读取中断标志
                if(temp == 0x40){                       //进入中断
                    SX127X_ReadFifo(RXbuffer, 16);      //读取接收缓冲区数据
					
                    //数据转换
                    shidu = 0;
                    wendu = 0;
                    shidu = ((((((RXbuffer[0]<<8)|RXbuffer[1])<<8)|RXbuffer[2])<<8)|RXbuffer[3])*1000/1024/1024;
                    wendu = ((((((RXbuffer[4]<<8)|RXbuffer[5])<<8)|RXbuffer[6])<<8)|RXbuffer[7])*2000/1024/1024-500;
                    guang = RXbuffer[8]*1000+RXbuffer[9]*100+RXbuffer[11]*10+RXbuffer[12]+RXbuffer[13]/10.0+RXbuffer[14]/100.0;
                    adc = RXbuffer[14]*256 + RXbuffer[15];

                    //转为字符串存储
                    sprintf(str,"node:%d, temperature:%.1lf, humidity:%.1lf%%, illuminance:%.2lflx, ADC:%d\n",current_node+2,shidu/10.0,wendu/10.0,guang,adc);
                    //ESP32发送数据
                    ESP8266_SendString(ENABLE, str, 0, Single_ID_0);
                    delay_ms(100);
                    //查询TCP是否关闭，如果关了就重连
                    if(TcpClosedFlag){
                        ESP8266_ExitUnvarnishSend();
                        do{
                            res = ESP8266_Get_LinkStatus();
                        }
                        while(!res);
                        if(res == 4){
                            while (!ESP8266_JoinAP(User_ESP8266_SSID, User_ESP8266_PWD ) );
                            while (!ESP8266_Link_Server(enumTCP, User_ESP8266_TCPServer_IP, User_ESP8266_TCPServer_PORT, Single_ID_0 ) );        
                        } 
                        while(!ESP8266_UnvarnishSend());                    
                    }
                    //串口输出
                    
                    printf("node:%d\r\n", current_node+1);
                    printf("shidu:%.1lf\r\n", shidu/10.0);
                    printf("wendu:%.1lf\r\n", wendu/10.0);
                    printf("guang:%.2lf\r\n", guang);
                    printf("ADC:%d\r\n", adc);
                    
                    //查询到最后一个节点等待一段时间
                    if(current_node == 3){
                        delay_ms(1000);
                        delay_ms(1000);
                        // delay_ms(1000);
                        // delay_ms(1000);
                        // delay_ms(1000);
                    }
                    //当前节点加一  0 -> 1 -> 2 -> 3 -> 0
                    current_node = ((current_node+1) & 0x03);
                    //转为发送模式
                    state = TX_ING;
                }
                //等待一段时间，转发送
                tx_t1++;
                if(tx_t1 == 50000){
                    tx_t1 = 0;
                    tx_t2++;
                    state = TX_ING;
                }
                //重发两次不回应后查询下一个节点
                if(tx_t2 == 2){
                    tx_t2 = 0;
                    current_node = ((current_node+1) & 0x03);
                    state = TX_ING;
                }
            break;

            case TX_ING:        //发送模式
                TXbuffer[0] = current_node+2;   //发送当前节点编号
                SX127X_TxPacket(TXbuffer);      //发送缓冲区数据
                delay_ms(500);
                //转为接收模式
                SX127X_StartRx();
                state = RX_ING;
            break;
        }
    }
}
#endif

void change_TX(float gzd, u32* CTdata, u16 adc)
{
    u16 gzd_int;

    // 光照度存储
    gzd_int = (u16)gzd;
    gzd = (gzd-gzd_int)*100.0;
    TXbuffer[8] = (gzd_int/1000)%10;
    TXbuffer[9] = (gzd_int/100)%10;
    TXbuffer[10]= (gzd_int/10)%10;
    TXbuffer[11]= (gzd_int)%10;
    gzd_int = (u16)gzd;
    TXbuffer[12]= (gzd_int/10)%10;
    TXbuffer[13]= (gzd_int)%10;

    // 温湿度存储
    TXbuffer[0] = (u8)((CTdata[0]&0xff000000)>>24);
    TXbuffer[1] = (u8)((CTdata[0]&0x00ff0000)>>16);
    TXbuffer[2] = (u8)((CTdata[0]&0x0000ff00)>>8);
    TXbuffer[3] = (u8) (CTdata[0]&0x000000ff);
    TXbuffer[4] = (u8)((CTdata[1]&0xff000000)>>24);
    TXbuffer[5] = (u8)((CTdata[1]&0x00ff0000)>>16);
    TXbuffer[6] = (u8)((CTdata[1]&0x0000ff00)>>8);
    TXbuffer[7] = (u8) (CTdata[1]&0x000000ff);

    // ADC存储
    TXbuffer[14] = (u8)((adc&0xff00)>>8);
    TXbuffer[15] = (u8)(adc&0x00ff);
}


