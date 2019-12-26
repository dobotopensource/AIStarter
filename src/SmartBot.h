/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           SmartBot.cpp
** Latest modified Date:2017-11.2
** Latest Version:      V0.9.0
** Descriptions:        SmartBot Api

** Modified by:         Jason
** Created date:        2018-11-29
** Version:             V1.0.0
** Descriptions:        Mixly API
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#ifndef __SMARTBOT_H
#define __SMARTBOT_H

#include "Arduino.h"
#include "Beep.h"
#include "HC-SR04.h"
#include "IRModule.h"
#include "MMC5883L.h"
#include "Motor.h"
#include "TCS3200.h"
#include "TimerOne.h"
#include <EEPROM.h>
#include <Servo.h>
#include "E2PROM.h"
/*key*/
#define START_PIN PINSW3
#define PINSW2 36
#define PINSW1 35
/*LED*/
#define LED1 13
#define LED2 12
//low power alarm
#define LOWP_LED 38
/*Light*/
#define LIGHT A8
//servo pin
#define PINSERVO1 7
#define PINSERVO2 44
/*Battery*/
#define BATTERYPIN A12
#define LOWPOWER (3 / 5) * 1024

extern int gHWVersion;
extern bool promgramRun;

typedef struct
{
    int bias;
    int lastBias;
    int speed;
    int lastSpeed;
    int target;
    int pwm;
    int ivalue;
    int ilimit;
} PIParams;

enum
{
    SW1,
    SW2,
    SW3
};

/*Movement*/
enum
{
    FRONT,
    BACK,
    RIGHT,
    LEFT
};

enum
{
    MOTORR,
    MOTORL
};

enum
{
    SERVO1,
    SERVO2
};

enum
{
    POWER,
    SPEED
};

enum
{
    ON,
    OFF,
    BLINK
};

/*******************SmartBotInit*****************/
/*********************************************************************************************************
** Function name:       SmartBotInit
** Descriptions:        Init SmartBot Scource
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotInit();

/*********************************************************************************************************
** Function name:       SmartBotSetMovment
** Descriptions:        Set Movement Direction and Speed
** Input parameters:    int dir(FRONT,BACK,RIGHT,LEFT),int speed(value<=255)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetMovment(int dir, int speed);

/*********************************************************************************************************
** Function name:       SmartBotSetMovmentTime
** Descriptions:        Set Motor R&L
** Input parameters:    int dir(FRONT, BACK, RIGHT, LEFT), int speed(value <= 255), float time(s)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetMovmentTime(int dir, int speed, float time);

/*********************************************************************************************************
** Function name:       SmartBotSetMotor
** Descriptions:        Set Motor R&L
** Input parameters:    int port(MOTORR,MOTORL),int speed(<200rpm)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetMotor(int port, int speed);

/*********************************************************************************************************
** Function name:       SmartBotSetMotorPI
** Descriptions:        Set Motor PI
** Input parameters:    float KP,float KI
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetMotorPI(float KP, float KI);

/*********************************************************************************************************
** Function name:       SmartBotGetMotorPose
** Descriptions:        Set Motor R&L
** Input parameters:    int port(the wheel number)
** Output parameters:   none
** Returned value:      (float)coder value
*********************************************************************************************************/
extern float AIStarter_SmartBotGetMotorPose(int port);

/*********************************************************************************************************
** Function name:       SmartBotSetSonar
** Descriptions:        Set Sonar Init
** Input parameters:    int port(SONAR1,SONAR2,SONAR3)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetSonar(int port);

/*********************************************************************************************************
** Function name:       SmartBotGetSonar
** Descriptions:        Get Snoar Distance
** Input parameters:    int port(SONAR1,SONAR2,SONAR3)
** Output parameters:   none
** Returned value:      (float)Distance, cm
*********************************************************************************************************/
extern float AIStarter_SmartBotGetSonar(int port);

/*********************************************************************************************************
** Function name:       SmartBotGetSonar
** Descriptions:        Get Snoar Barrier
** Input parameters:    int port(SONAR1,SONAR2,SONAR3)
** Output parameters:   none
** Returned value:      (float)Distance
*********************************************************************************************************/
extern bool AIStarter_SmartBotGetBarrier(int port);

/*********************************************************************************************************
** Function name:       SmartBotGetIRModulePort
** Descriptions:        Get IRModule valuwe
** Input parameters:    int port(IR1,IR2,IR3,IR4,IR5,IR6)
** Output parameters:   none
** Returned value:      valure(bit0-6 to IR1-6)
*********************************************************************************************************/
extern int AIStarter_SmartBotGetIRModuleValue(int port);
/*********************************************************************************************************
** Function name:       SmartBotGetCompass
** Descriptions:        Get Compass Angle
** Input parameters:    none
** Output parameters:   none
** Returned value:      Angle
*********************************************************************************************************/
extern float AIStarter_SmartBotGetCompass();
/*********************************************************************************************************
** Function name:       SmartBotSetCompassCalibration
** Descriptions:        Set Calibration
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern void AIStarter_SmartBotSetCompassCalibration();
/*********************************************************************************************************
** Function name:       SmartBotSetColorWB
** Descriptions:        Set ColorSenor White Balance
** Input parameters:    int port(COLORSENOR1,COLORSENOR2)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetColorWB(int port);
/*********************************************************************************************************
** Function name:       SmartBotSetColorSenor
** Descriptions:        Set ColorSenor Init or Deinit
** Input parameters:    int port(COLORSENOR1,COLORSENOR2),bool ison(true,false)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetColorSenor(int port, bool ison);
/*********************************************************************************************************
** Function name:       SmartBotGetColorSenor
** Descriptions:        Get ColorSenor Value
** Input parameters:    int port(COLORSENOR1,COLORSENOR2),int color(RCOLOR,GCOLOR,BCOLOR)
** Output parameters:   none
** Returned value:      Color Value
*********************************************************************************************************/
extern int AIStarter_SmartBotGetColorSenor(int port, int color);

/*********************************************************************************************************
** Function name:       SmartBotDetColorSenor
** Descriptions:        Get ColorSenor Value
** Input parameters:    int port(COLORSENOR1,COLORSENOR2),int color(RCOLOR,GCOLOR,BCOLOR)
** Output parameters:   bool (true or flase)
** Returned value:      Color Value
*********************************************************************************************************/
extern bool AIStarter_SmartBotDetColorSenor(int port, int color);

/*********************************************************************************************************
** Function name:       SmartBotSetKeyInit
** Descriptions:        Set Key Init
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetKeyInit();

/*********************************************************************************************************
** Function name:       SmartBotGetKeyValue
** Descriptions:        Get Key Value
** Input parameters:    int key(PINSW1,PINSW2,PINSW3)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotGetKeyValue(int key);

/*********************************************************************************************************
** Function name:       SmartBotGetLightAnalog
** Descriptions:        Get Photoresistance Value
** Input parameters:    none
** Output parameters:   none
** Returned value:      Photoresistance Value
*********************************************************************************************************/
extern int AIStarter_SmartBotGetLightAnalog();

/*********************************************************************************************************
** Function name:       SmartBotSetLED
** Descriptions:        SetLED
** Input parameters:    port(LED1/LED2),state(ON/OFF/BLINK)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetLED(int port, int state);

/*********************************************************************************************************
** Function name:       SmartBotSetSonarThreshold
** Descriptions:        SetSonarThreshold
** Input parameters:    int dis(cm)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotSetSonarThreshold(int dis);

/*********************************************************************************************************
** Function name:       SmartBotServoAttach
** Descriptions:        Attach the servo pin
** Input parameters:    servo(SERVO1/SERVO2)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotServoAttach(int servo);

/*********************************************************************************************************
** Function name:       SmartBotServoWrite
** Descriptions:        Set the servo angle
** Input parameters:    servo(SERVO1/SERVO2), value(0-180)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotServoWrite(int servo, int value);

/*********************************************************************************************************
** Function name:       SmartBotServoDetach
** Descriptions:        Detach the servo pin
** Input parameters:    servo(SERVO1/SERVO2)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotServoDetach(int servo);

/*********************************************************************************************************
** Function name:       SmartBotTimerTaskAttach
** Descriptions:        Attach the timertask
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotTimerTaskAttach(void);

/*********************************************************************************************************
** Function name:       SmartBotTimerTaskDetach
** Descriptions:        Detach the timertask
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotTimerTaskDetach(void);

/*********************************************************************************************************
** Function name:       SmartBotXbeeRead
** Descriptions:        Xbee read data
** Input parameters:    none
** Output parameters:   str(read string form the uart)
** Returned value:      none
*********************************************************************************************************/
extern String& AIStarter_SmartBotXbeeRead(void);

/*********************************************************************************************************
** Function name:       SmartBotXbeeWrite
** Descriptions:        Xbee write string
** Input parameters:    str(string to be written)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotXbeeWrite(const String &str);

/*********************************************************************************************************
** Function name:       SmartBotXbeeCompare
** Descriptions:        Xbee string compare
** Input parameters:    str1, str2(string to be compared)
** Output parameters:   none
** Returned value:      int(0 is equal, others is not equal)
*********************************************************************************************************/
extern int AIStarter_SmartBotXbeeCompare(const String &str1, const String &str2);

/*********************************************************************************************************
** Function name:       SmartBotXbeeCompare
** Descriptions:        XbeeReadBuffClear 
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern int AIStarter_SmartBotXbeeClear(void);

#endif
