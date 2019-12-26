#include "IRModule.h"
#include "Arduino.h"

int IRInit()
{
    pinMode(IRPIN1, INPUT);
    pinMode(IRPIN2, INPUT);
    pinMode(IRPIN3, INPUT);
    pinMode(IRPIN4, INPUT);
    pinMode(IRPIN5, INPUT);
    pinMode(IRPIN6, INPUT);

    return 0;
}

int IRDetectPort(int port)
{
    int result = 0;

    switch (port)
    {
    case IR1:
        result = digitalRead(IRPIN1);
        break;
    case IR2:
        result = digitalRead(IRPIN2);
        break;
    case IR3:
        result = digitalRead(IRPIN3);
        break;
    case IR4:
        result = digitalRead(IRPIN4);
        break;
    case IR5:
        result = digitalRead(IRPIN5);
        break;
    case IR6:
        result = digitalRead(IRPIN6);
        break;
    }
    return result;
}

int IRDetect()
{
    int val = 0;

    val |= digitalRead(IRPIN1) << 0;
    val |= digitalRead(IRPIN2) << 1;
    val |= digitalRead(IRPIN3) << 2;
    val |= digitalRead(IRPIN4) << 3;
    val |= digitalRead(IRPIN5) << 4;
    val |= digitalRead(IRPIN6) << 5;

    return val;
}
