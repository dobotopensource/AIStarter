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
#include "SmartBot.h"

/*********************************************************************************************************
** Descriptions:        Put the global variable here
*********************************************************************************************************/
/*LED */
int gLED1State, gLED2State;
bool LED1state, LED2state;
int gBlinkFre; //Per 50ms
bool promgramRun = false;
/*mMotro*/
bool gSpeedMode;
int gMotorRSpeed, gMotorLSpeed;

PIParams gPIR, gPIL;

/*PI params*/

float kp = 1.3, ki = 0.13, kd = 1.0;
const float kp_max = 2.5;
const float kp_min = 0.5;
const float ki_max = 0.5;
const float ki_min = 0.05;
/**/
int gPRMR;
int gPRML;

static Servo servo1;
static Servo servo2;

const int color_thr = 30;
static String xbeeReadBuffer;
const int xbeeReadBufferLen = 128;

/*********************************************************************************************************
** Function name:       MotorPI
** Descriptions:        PI Control
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
static int MotorPI(PIParams *pi)
{
    pi->bias = (pi->target - pi->speed);
    pi->ivalue += pi->bias;

    if (pi->ivalue > pi->ilimit)
    {
        pi->ivalue = pi->ilimit;
    }
    else if (pi->ivalue < -pi->ilimit)
    {
        pi->ivalue = -pi->ilimit;
    }

    pi->pwm = kp * pi->bias + ki * pi->ivalue + kd * (pi->bias - pi->lastBias);
    pi->lastBias = pi->bias;

    /*Serial.print("pi->bias ");
    Serial.print(pi->bias);
    Serial.print(", pi->ivalue ");
    Serial.print(pi->ivalue);
    Serial.print(", pi->target ");
    Serial.print(pi->target);
    Serial.print(", pi->speed ");
    Serial.print(pi->speed);
    Serial.print(", pi->pwm ");
    Serial.println(pi->pwm);*/

    return pi->pwm;
}
/*********************************************************************************************************
** Function name:       MotorPI
** Descriptions:        PI Control
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
static int LEDControl(void)
{
    switch (gLED1State)
    {
    case ON:
        LED1state = LOW;
        break;
    case OFF:
        LED1state = HIGH;
        break;
    case BLINK:
        LED1state = !LED1state;
        break;
    default:
        break;
    }

    switch (gLED2State)
    {
    case ON:
        LED2state = LOW;
        break;
    case OFF:
        LED2state = HIGH;
        break;
    case BLINK:
        LED2state = !LED2state;
        break;
    default:
        break;
    }
    digitalWrite(LED1, LED1state);
    digitalWrite(LED2, LED2state);
    return 0;
}

/*********************************************************************************************************
** Function name:       InitPiSt
** Descriptions:
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
static void InitPiSt(void)
{
    gPIR.pwm = 0;
    gPIL.pwm = 0;
    gPIR.speed = 0;
    gPIL.speed = 0;
    gPIR.lastSpeed = 0;
    gPIL.lastSpeed = 0;
    gPIR.target = 0;
    gPIL.target = 0;
    gPIR.bias = 0;
    gPIL.bias = 0;
    gPIR.lastBias = 0;
    gPIL.lastBias = 0;
    gPIR.ilimit = 2000;
    gPIL.ilimit = 2000;
    gPIR.ivalue = 0;
    gPIL.ivalue = 0;
}

/*********************************************************************************************************
** Function name:       InitVariable
** Descriptions:
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
static int InitVariable(void)
{
    /*LED*/
    gLED1State = OFF;
    gLED2State = OFF;
    gBlinkFre = 500;

    /*Motor*/
    gSpeedMode = false;
    InitPiSt();
    return 0;
}

#define RUN_BY_LIMIT_BLOCK(ms, BLOCK)           \
    {                                           \
        static uint32_t last_exec_ms = 0;       \
        uint32_t cur_ms_tick = millis();        \
        if (cur_ms_tick >= (last_exec_ms + ms)) \
        {                                       \
            BLOCK;                              \
            last_exec_ms = cur_ms_tick;         \
        }                                       \
    }

#define GEAR_RATIO 20                         //1:20
#define MAX_SPEED 200                         //rpm
#define WHEEL_DIA 65                          //mm
#define MOTOR_PPR 13                          //PPR
#define TOTAL_PPR (GEAR_RATIO * MOTOR_PPR)    //PPR
#define PPMS_TO_PPM (60.0 * 1000 / TOTAL_PPR) //pulse per ms, convert to rpm
#define COEFF_SPEED_FILTER 0.7

static void Motor_ctrl(void)
{
    static int state = 0;
    static bool start = false;
    static unsigned long lastMillis;
    unsigned long curMillis;
    unsigned long deltaMillis;

    switch (state)
    {
    case 0: //init the last value for motor speed calculation
        lastMillis = millis();
        gLastCounterR = gCounterR;
        gLastCounterL = gCounterL;
        state++;
        break;
    case 1:
        start = true;
        break;
    }

    if (start)
    {
        if (gSpeedMode == true)
        {
            MotorR(MotorPI(&gPIR));
            MotorL(MotorPI(&gPIL));
        }
        curMillis = millis();
        deltaMillis = curMillis - lastMillis;
        lastMillis = curMillis;

        int speedReadR;
        int speedReadL;

        speedReadR = PPMS_TO_PPM * (gCounterR - gLastCounterR) / deltaMillis; //rpm
        speedReadL = PPMS_TO_PPM * (gCounterL - gLastCounterL) / deltaMillis; //rpm

        gPIR.speed = (1 - COEFF_SPEED_FILTER) * gPIR.lastSpeed + COEFF_SPEED_FILTER * speedReadR;
        gPIL.speed = (1 - COEFF_SPEED_FILTER) * gPIL.lastSpeed + COEFF_SPEED_FILTER * speedReadL;

        gPIR.lastSpeed = gPIR.speed;
        gPIL.lastSpeed = gPIL.speed;

        gLastCounterR = gCounterR;
        gLastCounterL = gCounterL;
    }
}

static void lowpower_monitor(void)
{
    static bool lowpower;
    /*Battery*/
    uint16_t power = analogRead(A12);
    if (power > ((3.8 / 5.2) * 1024))
    {
        lowpower = false;
    }
    else if (analogRead(A12) < ((3.6 / 5.2) * 1024))
    {
        lowpower = true;
    }
    if (lowpower)
    {
        digitalWrite(LOWP_LED, LOW);
    }
    else
    {
        digitalWrite(LOWP_LED, HIGH);
    }
}

static void led_ctrl(void)
{
    LEDControl();
}

static void ColorDetect(void)
{
    ColorDetectCallBack1();
    ColorDetectCallBack2();
}

static void StartKeyCheck(void)
{
    const int key_filter_cnt = 3;
    static int state = 0;
    static bool toggle = false;
    static int cnt = 0;
    int readKeyState;
    readKeyState = digitalRead(START_PIN);

    switch (state)
    {
    case 0:
        if (!readKeyState)
        {
            if (++cnt > key_filter_cnt)
            {
                cnt = 0;
                state++; //pressed
                // Serial.println("pressed");
            }
        }
        else
        {
            cnt = 0;
        }
        break;
    case 1:
        if (readKeyState)
        {
            if (++cnt > key_filter_cnt)
            {
                cnt = 0;
                state = 0;
                toggle = true; //released
                // Serial.println("released");
            }
        }
        else
        {
            cnt = 0;
        }
        break;
    }

    if (toggle)
    {
        toggle = false;
        if (promgramRun)
        {
            promgramRun = false;
            // Serial.println("promgramRun false");
        }
        else
        {
            promgramRun = true;
            // Serial.println("promgramRun true");
        }
    }
}

/*********************************************************************************************************
** Function name:       SmartBotSetTimeOneMs
** Descriptions:        Time task for ColorSensor & Motor pose
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
static void TimeTask()
{
    if (promgramRun)
    {
        RUN_BY_LIMIT_BLOCK(20, Motor_ctrl());
        RUN_BY_LIMIT_BLOCK(gBlinkFre, led_ctrl());
        RUN_BY_LIMIT_BLOCK(2, ColorDetect());
    }
    else
    {
        MotorR(0);
        MotorL(0);
        InitPiSt();
    }

    RUN_BY_LIMIT_BLOCK(50, lowpower_monitor());
    RUN_BY_LIMIT_BLOCK(20, StartKeyCheck());
}
/*********************************************************************************************************
** Function name:       SmartBotSetTimeOneMs
** Descriptions:        Set Time Intrrupt
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

static int SmartBotSetTimeOneMs() //ms
{
    Timer1.initialize(5000);          // 初始化,  interval 以 us 為單位
    Timer1.attachInterrupt(TimeTask); // attach the service routine here
}

////////////////////////User Interface//////////////////////////////////
int AIStarter_SmartBotInit()
{
    Serial.begin(115200);
    Serial.println("======Enter application======");

    Serial2.begin(57600); //Xbee

    pinMode(LED1, OUTPUT);  //LED1
    pinMode(LED2, OUTPUT);  //LED2
    pinMode(PINSW3, INPUT); //PINSW3
    pinMode(PINSW2, INPUT); //PINSW2
    pinMode(PINSW1, INPUT); //PINSW1

    BeepInit();

    MotorInit();
    EncoderInit();

    AIStarter_SmartBotSetKeyInit();
    static bool releasedFlag = true;
    while (!promgramRun)
    {
        if (!digitalRead(START_PIN))
        {
            delay(20);
            if (!digitalRead(START_PIN))
            {
                releasedFlag = false;
                digitalWrite(BEEP, HIGH);
            }
        }
        if (!releasedFlag)
        {
            if (digitalRead(START_PIN))
            {
                promgramRun = true;
                digitalWrite(BEEP, LOW);
            }
        }
    }
    Serial.println("ProgramStar:");
    /*InitVariable*/
    InitVariable();

    SonarInit(SONAR1);
    SonarInit(SONAR2);
    SonarInit(SONAR3);

    IRInit();

    CompassSenorInit();

    ColorInit(COLORSENOR1);
    ColorInit(COLORSENOR2);

    pinMode(BATTERYPIN, INPUT); //VOTAGLE
    pinMode(LOWP_LED, OUTPUT);  //LOWPOWER LED

    EepromRead(VERSIONADD, &gHWVersion, sizeof(gHWVersion));
    AIStarter_SmartBotSetLED(LED1, ON);
    AIStarter_SmartBotSetLED(LED2, ON);
    SmartBotSetTimeOneMs();
}

int AIStarter_SmartBotSetMovment(int dir, int speed)
{
    speed = -speed;
    ///////////////////the leftmotor line & right motor line are going across with each other
    ////the MotorR contrl the left motor & the MotorR contrl the right motor
    if (!promgramRun)
        return -1;
    gSpeedMode = false;
    switch (dir)
    {
    case FRONT:
        MotorR(speed);
        MotorL(speed);
        break;
    case BACK:
        MotorR(-speed);
        MotorL(-speed);
        break;
    case RIGHT:
        MotorR(0);
        MotorL(speed);
        break;
    case LEFT:
        MotorR(speed);
        MotorL(0);

        break;
    default:
        break;
    }

    return 0;
}

int AIStarter_SmartBotSetMovmentTime(int dir, int speed, float time)
{

    AIStarter_SmartBotSetMovment(dir, speed);
    delay(time * 1000);

    MotorR(0);
    MotorL(0);
    return 0;
}

int AIStarter_SmartBotSetMotor(int port, int speed)
{
    speed = -speed;
    gSpeedMode = true;
    switch (port)
    {
    case MOTORR:
        gPIR.target = speed;
        break;
    case MOTORL:
        gPIL.target = speed;
        break;
    default:
        break;
    }
    return 0;
}

int AIStarter_SmartBotSetMotorPI(float KP, float KI)
{
    if (KP < kp_max && KP > kp_min)
    {
        kp = KP;
        Serial.println("kp set success");
    }
    else
    {
        Serial.println("kp value is not valid");
    }
    if (KI < ki_max && KI > ki_min)
    {
        ki = KI;
        Serial.println("ki set success");
    }
    else
    {
        Serial.println("ki value is not valid");
    }
    return 0;
}

float AIStarter_SmartBotGetMotorPose(int port)
{
    switch (port)
    {
    case MOTORR:
        return gCounterR;
        break;
    case MOTORL:
        return gCounterL;
        break;
    default:
        break;
    }
    return 0;
}

int AIStarter_SmartBotSetSonar(int port)
{
    return SonarInit(port);
}

float AIStarter_SmartBotGetSonar(int port)
{
    return SonarDetect(port);
}

bool AIStarter_SmartBotGetBarrier(int port)
{
    return SonarDetect(port) < 10;
}

int AIStarter_SmartBotGetIRModuleValue(int port)
{
    return IRDetectPort(port);
}

float AIStarter_SmartBotGetCompass()
{
    return CompassSenorDetect();
}

void AIStarter_SmartBotSetCompassCalibration()
{
    CompassSenorCalibration();
}

int AIStarter_SmartBotSetColorWB(int port)
{
    ColorWB(port);
}

int AIStarter_SmartBotSetColorSenor(int port, bool ison)
{
    if (ison)
    {
        ColorInit(port);
    }
    else
    {
        ColorDeinit(port);
    }
}

int AIStarter_SmartBotGetColorSenor(int port, int color)
{
    switch (port)
    {
    case COLORSENOR1:
        switch (color)
        {
        case RCOLOR:
            return gColorCounter1[RCOLOR];
            break;
        case GCOLOR:
            return gColorCounter1[GCOLOR];
            break;
        case BCOLOR:
            return gColorCounter1[BCOLOR];
            break;
        default:
            break;
        }
        break;
    case COLORSENOR2:
        switch (color)
        {
        case RCOLOR:
            return gColorCounter2[RCOLOR];
            break;
        case GCOLOR:
            return gColorCounter2[GCOLOR];
            break;
        case BCOLOR:
            return gColorCounter2[BCOLOR];
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

bool AIStarter_SmartBotDetColorSenor(int port, int color)
{
    switch (port)
    {
    case COLORSENOR1:
        switch (color)
        {
        case RCOLOR:
            if (gColorCounter1[RCOLOR] > gColorCounter1[GCOLOR] &&
                gColorCounter1[RCOLOR] - gColorCounter1[GCOLOR] > color_thr &&
                gColorCounter1[RCOLOR] > gColorCounter1[BCOLOR] &&
                gColorCounter1[RCOLOR] - gColorCounter1[BCOLOR] > color_thr)
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        case GCOLOR:
            if (gColorCounter1[GCOLOR] > gColorCounter1[RCOLOR] &&
                gColorCounter1[GCOLOR] - gColorCounter1[RCOLOR] > color_thr &&
                gColorCounter1[GCOLOR] > gColorCounter1[BCOLOR] &&
                gColorCounter1[GCOLOR] - gColorCounter1[BCOLOR] > color_thr)
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        case BCOLOR:
            if (gColorCounter1[BCOLOR] > gColorCounter1[RCOLOR] &&
                gColorCounter1[BCOLOR] - gColorCounter1[RCOLOR] > color_thr &&
                gColorCounter1[BCOLOR] > gColorCounter1[GCOLOR] &&
                gColorCounter1[BCOLOR] - gColorCounter1[GCOLOR] > color_thr)
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        default:
            break;
        }
        break;
    case COLORSENOR2:
        switch (color)
        {
        case RCOLOR:
            if (gColorCounter2[RCOLOR] > gColorCounter2[GCOLOR] && gColorCounter2[RCOLOR] > gColorCounter2[BCOLOR])
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        case GCOLOR:
            if (gColorCounter2[GCOLOR] > gColorCounter2[RCOLOR] && gColorCounter2[GCOLOR] > gColorCounter2[BCOLOR])
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        case BCOLOR:
            if (gColorCounter2[BCOLOR] > gColorCounter2[RCOLOR] && gColorCounter2[BCOLOR] > gColorCounter2[GCOLOR])
            {
                return 1;
            }
            else
            {
                return 0;
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

int AIStarter_SmartBotSetKeyInit()
{
    pinMode(PINSW1, INPUT);
    pinMode(PINSW2, INPUT);
    pinMode(PINSW3, INPUT);
}

int AIStarter_SmartBotGetKeyValue(int key)
{
    return digitalRead(key);
}

int AIStarter_SmartBotGetLightAnalog()
{
    return analogRead(A8);
}

int AIStarter_SmartBotSetLED(int port, int state)
{
    switch (port)
    {
    case LED1:
        gLED1State = state;
        break;
    case LED2:
        gLED2State = state;
        break;
    default:
        break;
    }
    return 0;
}

int AIStarter_SmartBotSetSonarThreshold(int dis)
{
    gSonarTime = (int)(dis * 58);
    return 0;
}

int AIStarter_SmartBotServoAttach(int servo)
{
    Timer1.detachInterrupt();
    if (SERVO1 == servo)
    {
        servo1.attach(PINSERVO1);
    }
    else if (SERVO2 == servo)
    {
        Timer1.detachInterrupt();
        servo2.attach(PINSERVO2);
    }
    return 0;
}

int AIStarter_SmartBotServoWrite(int servo, int value)
{
    if (SERVO1 == servo)
    {
        servo1.write(value);
    }
    else if (SERVO2 == servo)
    {
        Timer1.detachInterrupt();
        servo2.write(value);
    }
    return 0;
}

int AIStarter_SmartBotServoDetach(int servo)
{
    Timer1.attachInterrupt(TimeTask);
    if (SERVO1 == servo)
    {
        servo1.detach();
    }
    else if (SERVO2 == servo)
    {
        servo2.detach();
    }
    return 0;
}

int AIStarter_SmartBotTimerTaskAttach(void)
{
    Timer1.attachInterrupt(TimeTask);
    return 0;
}

int AIStarter_SmartBotTimerTaskDetach(void)
{
    Timer1.detachInterrupt();
    return 0;
}

String& AIStarter_SmartBotXbeeRead(void)
{
    while(Serial2.available() && xbeeReadBuffer.length() < xbeeReadBufferLen-1){
        xbeeReadBuffer += ((char)(Serial2.read()));
    }
    return xbeeReadBuffer;
}

int AIStarter_SmartBotXbeeWrite(const String &str)
{
    Serial2.print(str);
    return 0;
}

int AIStarter_SmartBotXbeeCompare(const String &str1, const String &str2)
{
    if (str1 == str2)
    {
        return 0;
    }
    return -1;
}

int AIStarter_SmartBotXbeeClear(void)
{
    xbeeReadBuffer.remove(0, xbeeReadBuffer.length());
    return 0;
}
