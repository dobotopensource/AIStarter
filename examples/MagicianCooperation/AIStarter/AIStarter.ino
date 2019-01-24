/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:             ColorRecogition
** Latest modified Date:  2018-12-5
** Latest Version:        V1.0.0
** Descriptions:          Ai-Starter Demo
*********************************************************************************************************/
#include "AISTARTER.h"
#define  Threshold 15             //设置颜色阈值
#define  IR_NUM    6              //设置红外对管数量
#define  DBG_EN    0              
#define  DBG_COLOR 0
#define  DBG_WB 0
#define  DBG_Loop 1
int lineState;                    //设置巡线、出入黑线状态
bool colorRec = false;            //颜色识别状态位
int beepFlag = LOW;               //蜂鸣器状态

/************* lineState设置巡线、出入黑线状态 **************/
enum{
    LINEPATROL,                    //巡线
    ENTERBLACK,                    //进黑线
    EXITBLACK                      //出黑线
};

/************* 颜色传感器检测到的颜色状态，红、绿和其他 **************/
enum{
    OTHER,                         //其他
    RED,                           //红色
    GREEN                          //绿色
};
/************* 初始化 **************/
void setup() 
{
    Serial.begin(115200);          //设置串口波特率
    pinMode(13, OUTPUT);
    pinMode(36, INPUT);
    AIStarter_SmartBotInit();       //小车初始化
    AIStarter_SmartBotSetLED(LED1,BLINK);       //LED1闪烁
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
//    const float kp = 1;
//    const float ki = 0.06;
//    const float kd = 0.0;
//    const float errorsumLimit = 50;

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

/************* 颜色传感器检测到除红绿线外发生的事件 **************/
void otherLineEvent()
{
    delay(3000);  
}

/************* 颜色传感器检测到红线发生的事件 **************/
void rLineEvent()
{
    for(uint8_t i = 0; i<6; i++) {
        digitalWrite(BEEP,beepFlag = !beepFlag);
        delay(500);
    }
}

/************* 颜色传感器检测到绿线发生的事件 **************/
void gLineEvent()
{
    digitalWrite(BEEP,beepFlag = !beepFlag);
    delay(100);
    digitalWrite(BEEP,beepFlag = !beepFlag);
    for(uint8_t i=0; i<60; i++) {
        delay(1000);  
    }
}

/************* 程序主循环 **************/
void loop() 
{
#if DBG_WB
    AIStarter_SmartBotSetColorWB(COLORSENOR1);
    AIStarter_SmartBotSetColorWB(COLORSENOR2);
#endif
#if DBG_Loop
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
    int colorState;
    getCurrentIRState(&irstate);
    Serial.print("irstate = ");
    Serial.println(irstate);
    if(irstate == 63) {
        if(lineState == LINEPATROL) {
            lineState = ENTERBLACK;
        } else if(lineState == EXITBLACK) {
            AIStarter_SmartBotSetMotor(MOTORL, 50);
            AIStarter_SmartBotSetMotor(MOTORR, 50);   
        }
    } else {
        lineState = LINEPATROL;
        curPos = getCurrentPos(irstate);
        setCarSpeed(curPos);
    }
    switch (lineState) {
        case LINEPATROL:
            colorRec = false;
        break;
        case ENTERBLACK:
            colorRec = true;
        break;
        case EXITBLACK:
            colorRec = false;
        break;
        default:
        break;
    }
    while(colorRec){
        AIStarter_SmartBotSetMotor(MOTORL, 0);
        AIStarter_SmartBotSetMotor(MOTORR, 0);
        Serial.println("colorRec Stop");
        delay(1000);
        if(AIStarter_SmartBotGetColorSenor(COLORSENOR1,RCOLOR) - AIStarter_SmartBotGetColorSenor(COLORSENOR1,GCOLOR) > Threshold ) {
            colorState = RED;
            Serial.println("ENTER RLINE");
        } else if(AIStarter_SmartBotGetColorSenor(COLORSENOR1,GCOLOR) - AIStarter_SmartBotGetColorSenor(COLORSENOR1,BCOLOR) > Threshold) {
            colorState = GREEN;
            Serial.println("ENTER GLINE");
        } else {
            colorState = OTHER;
            Serial.println("ENTER OTHERLINE");
        }
        switch(colorState) {
            case OTHER:
                otherLineEvent();
                colorRec = false;
                lineState = EXITBLACK;
            break;
            case RED:
                rLineEvent();
                colorRec = false;
                lineState = EXITBLACK;
            break;
            case GREEN:
                gLineEvent();
                colorRec = false;
                lineState = EXITBLACK;
            break;
            default:
            break;  
        }
    }
    delay(20);
    
#endif

#if DBG_COLOR
    Serial.print("Senor1:");
    Serial.print("R:");
    Serial.print(AIStarter_SmartBotGetColorSenor(COLORSENOR1,RCOLOR));
    Serial.print("    G:");
    Serial.print(AIStarter_SmartBotGetColorSenor(COLORSENOR1,GCOLOR));
    Serial.print("    B:");
    Serial.println(AIStarter_SmartBotGetColorSenor(COLORSENOR1,BCOLOR));
    Serial.print("Senor2:");
    Serial.print("R:");
    Serial.print(AIStarter_SmartBotGetColorSenor(COLORSENOR2,RCOLOR));
    Serial.print("    G:");
    Serial.print(AIStarter_SmartBotGetColorSenor(COLORSENOR2,GCOLOR));
    Serial.print("    B:");
    Serial.println(AIStarter_SmartBotGetColorSenor(COLORSENOR2,BCOLOR));
#endif
    
}
