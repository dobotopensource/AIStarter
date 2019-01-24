/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:             LinePatrol
** Latest modified Date:  2018-12-5
** Latest Version:        V1.0.0
** Descriptions:          Ai-Starter Demo
*********************************************************************************************************/
#include <AIStarter.h>

#define  IR_NUM    6              //设置红外对管数量
#define  DBG_EN    0
#define  DBG_COLOR 0

/************* 初始化 **************/
void setup() 
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    pinMode(36, INPUT);
    AIStarter_SmartBotInit();
    Serial.println("SmartBotInit is completed");
    AIStarter_SmartBotSetLED(LED1,BLINK);
}

/************* 获取巡线传感器状态 **************/
void getCurrentIRState(int *irstate)
{
    *irstate = 0;
    for (int i = 0; i < IR_NUM; i++) {
        *irstate |= AIStarter_SmartBotGetIRModuleValue(i) << i;
    }
}

/************* 获取当前车体姿态 **************/
float getCurrentPos(const int irstate)
{
    const float coeff = 0.7;
    const int irPos[] = {-30, -18, -6, 6, 18, 30};//mm
    static float lastPos;
    float curPos;
    float readPos;
    int total = 0;
    int irOffCnt = 0;
    //calculate the car position offset
    for (int i = 0; i < IR_NUM; i++) {
        if (irstate & (1 << i)) {
            total += irPos[i];
            irOffCnt++;
        }
    }
    if (irOffCnt) {
        readPos = total / irOffCnt;
    }
    else {
        readPos = lastPos;
    }
    //calculate the current position
    curPos = (1 - coeff) * lastPos + coeff * readPos;
    lastPos = curPos;
    return curPos;
}

/************* 设置小车速度 **************/
void setCarSpeed(const float curPos)
{
    const int baseSpeed = 100; //rpm
    //baseSpeed 70  kp 1  ki 0.06
    
    //for speed 50
    /*const float kp = 1;
    const float ki = 0.06;
    const float kd = 0.0;
    const float errorsumLimit = 50;*/

    //for speed 100
    const float kp = 1.2;
    const float ki = 0.1;
    const float kd = 0.0;
    const float errorsumLimit = 100;

    float error = curPos;
    static float lastError;
    static float errorsum;
    float errorChange;
    int speedLeftWheel;
    int speedRightWheel;
    int speedOffset;

    //pid
    errorsum += error;
    if (errorsum > errorsumLimit) {
        errorsum = errorsumLimit;
    }
    else if (errorsum < -errorsumLimit){
        errorsum = -errorsumLimit;
    }
    errorChange = error - lastError;
    speedOffset = kp * error + ki * errorsum + kd * errorChange;
    lastError = error;

    //calculate the wheel speed
    speedLeftWheel  = baseSpeed + speedOffset;
    speedRightWheel = baseSpeed - speedOffset;

    //set tht wheel speed
#if DBG_EN
    Serial.print("speedleftwheel = ");
    Serial.print(speedLeftWheel);
    Serial.print("    speedrightwheel = ");
    Serial.println(speedRightWheel);
#endif
    AIStarter_SmartBotSetMotor(MOTORL, speedLeftWheel);
    AIStarter_SmartBotSetMotor(MOTORR, speedRightWheel);
    Serial.println("go ahead");
}

/************* 程序主循环 **************/
void loop()
{
    if(!AIStarter_SmartBotGetKeyValue(37)) {
        delay(5);
        if(!AIStarter_SmartBotGetKeyValue(37)) {
            digitalWrite(BEEP,HIGH);
            delay(10);
        }   
    } else {
        if(AIStarter_SmartBotGetKeyValue(37)) {
            digitalWrite(BEEP,LOW);
        }
    }
    int irstate;
    float curPos;
    getCurrentIRState(&irstate);
    curPos = getCurrentPos(irstate);
    setCarSpeed(curPos);
    delay(20);
}
