#ifndef __TCP_H
#define __TCP_H 			   
#include "stm32f10x.h"


/*
*���²�����Ҫ�û������޸Ĳ��ܲ����ù�
*/

#define User_ESP8266_SSID     "OnePlus8T"          //wifi��
#define User_ESP8266_PWD      "e9a537b59ef0"      //wifi����

#define User_ESP8266_TCPServer_IP     "192.168.175.46"     //������IP
#define User_ESP8266_TCPServer_PORT   "8888"      //�������˿ں�


extern volatile uint8_t TcpClosedFlag;  //����״̬��־

void ESP8266_STA_TCPClient_Test(void);

#endif
