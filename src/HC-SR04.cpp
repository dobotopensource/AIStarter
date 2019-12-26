#include "HC-SR04.h"
#include "Arduino.h"
#define SONARDEBUG false

int gSonarTime = 3000;
int SonarInit(int port)
{
    switch (port)
    {
    case SONAR1:
        pinMode(TRIG1, OUTPUT);
        pinMode(ECHO1, INPUT);
        break;
    case SONAR2:
        pinMode(TRIG2, OUTPUT);
        pinMode(ECHO2, INPUT);
        break;
    case SONAR3:
        pinMode(TRIG3, OUTPUT);
        pinMode(ECHO3, INPUT);
        break;
    default:
        break;
    }
    return 0;
}

float SonarDetect(int port)
{
    int distance;
    switch (port)
    {
    case SONAR1:
        digitalWrite(TRIG1, LOW); // 使发出发出超声波信号接口低电平2μs
        delayMicroseconds(2);
        digitalWrite(TRIG1, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
        delayMicroseconds(10);
        digitalWrite(TRIG1, LOW);                    // 保持发出超声波信号接口低电平
        distance = pulseIn(ECHO1, HIGH, gSonarTime); // 读出脉冲时间
        if (distance == 0)
        {
            distance = gSonarTime;
        }
        break;
    case SONAR2:
        digitalWrite(TRIG2, LOW); // 使发出发出超声波信号接口低电平2μs
        delayMicroseconds(2);
        digitalWrite(TRIG2, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
        delayMicroseconds(10);
        digitalWrite(TRIG2, LOW);                    // 保持发出超声波信号接口低电平
        distance = pulseIn(ECHO2, HIGH, gSonarTime); // 读出脉冲时间
        if (distance == 0)
        {
            distance = gSonarTime;
        }
        break;
    case SONAR3:
        digitalWrite(TRIG3, LOW); // 使发出发出超声波信号接口低电平2μs
        delayMicroseconds(2);
        digitalWrite(TRIG3, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
        delayMicroseconds(10);
        digitalWrite(TRIG3, LOW);                    // 保持发出超声波信号接口低电平
        distance = pulseIn(ECHO3, HIGH, gSonarTime); // 读出脉冲时间
        if (distance == 0)
        {
            distance = gSonarTime;
        }
        break;
    default:
        break;
    }
    //
    float dis = (float)distance / 58; // 将脉冲时间转化为距离（单位：厘米）
//distance= distance/58; // 将脉冲时间转化为距离（单位：厘米）
#if SONARDEBUG
    Serial.print("Sonar");
    Serial.print(port);
    Serial.print(":");
    Serial.println(dis);

#endif
    return dis;
}
