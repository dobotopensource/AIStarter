/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:             WhiteBalance
** Latest modified Date:  2018-12-5
** Latest Version:        V1.0.0
** Descriptions:          Ai-Starter Demo
*********************************************************************************************************/
#include <AIStarter.h>

/************* 初始化 **************/
void setup() {
    // put your setup code here, to run once:
    AIStarter_SmartBotInit();
    pinMode(BEEP,OUTPUT);
}

/************* 程序主循环 **************/
void loop() {
    // put your main code here, to run repeatedly:
    AIStarter_SmartBotSetColorWB(COLORSENOR1);
    AIStarter_SmartBotSetColorWB(COLORSENOR2);
    digitalWrite(BEEP,HIGH);
    delay(500);
    digitalWrite(BEEP,LOW);
    while(1){
    }
}
