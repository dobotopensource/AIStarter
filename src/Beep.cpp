#include "Arduino.h"
#include "Beep.h"

int BeepInit(void)
{
    pinMode(BEEP, OUTPUT);

    return 0;
}

int BeepFre(int fre)
{
    TCCR1B = 0;                      // 只是个复位的习惯可以不要
    TCCR1A = _BV(COM1A0);            // 要是困扰的话你也可以写成 TCCR1A = 0b01000000;
    TCCR1B = _BV(WGM12) | _BV(CS10); //  这里等值于 TCCR1B = 0b00001001;
                                     // 最有意思的了，16位的玩意来确定要一个怎样的频率
    OCR1A = 16000000 / 2 / fre - 1;  // 计算为：F_CPU/2/Need_HZ-1 来得到你想要的，遗憾的是四舍五入来截取频率

    return 0;
}

int BeepVoice(int ton, int rhythm)
{
    (void)ton;
    (void)rhythm;

    return 0;
}
