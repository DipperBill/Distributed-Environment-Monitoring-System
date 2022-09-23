#ifndef __TCP_H
#define __TCP_H 			   
#include "stm32f10x.h"


/*
*以下参数需要用户自行修改才能测试用过
*/

#define User_ESP8266_SSID     "OnePlus8T"          //wifi名
#define User_ESP8266_PWD      "e9a537b59ef0"      //wifi密码

#define User_ESP8266_TCPServer_IP     "192.168.175.46"     //服务器IP
#define User_ESP8266_TCPServer_PORT   "8888"      //服务器端口号


extern volatile uint8_t TcpClosedFlag;  //连接状态标志

void ESP8266_STA_TCPClient_Test(void);

#endif
