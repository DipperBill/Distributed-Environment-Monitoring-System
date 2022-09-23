#ifndef __MQTT_H
#define __MQTT_H 			   
#include "stm32f10x.h"


/*
*���²�����Ҫ�û������޸Ĳ��ܲ����ù�
*/


#define User_ESP8266_SSID     "OnePlus8T"    //wifi��
#define User_ESP8266_PWD      "e9a537b59ef0"      //wifi����

#define User_ESP8266_client_id    "aithinker"   //MQTTclientID ���ڱ�־client���  �256�ֽ�
#define User_ESP8266_username     "admin"						//���ڵ�¼ MQTT ������ �� username, � 64 �ֽ�	
#define User_ESP8266_password			"public"          //���ڵ�¼ MQTT ������ �� password, � 64 �ֽ�
#define User_ESP8266_MQTTServer_IP     "192.168.159.46"     //MQTT���ط�����IP
#define User_ESP8266_MQTTServer_PORT   1883     //�������˿ں�
#define User_ESP8266_MQTTServer_Topic  "topic"    //����MQTT����

void ESP8266_STA_MQTTClient_Test(void);

#endif
