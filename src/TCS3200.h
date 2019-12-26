#ifndef __TSC3200_H
#define __TSC3200_H

#include "Arduino.h"
#include "E2PROM.h"
#define WBBASIC 200

#define Color1_S2 65 //S2和S3的组合决定让红、绿、蓝，哪种光线通过滤波器
#define Color1_S3 64
#define Color1_OUT 18 //TCS3200颜色传感器输出信号连接到Arduino中断引脚，并引发脉冲信号中断
#define ITRNUM1 5
#define Color1_LED 63 //PK1-A9//控制TCS3200颜色传感器是否点亮LED灯

#define Color2_S2 30 //S2和S3的组合决定让红、绿、蓝，哪种光线通过滤波器
#define Color2_S3 31
#define Color2_OUT 19 //TCS3200颜色传感器输出信号连接到Arduino中断引脚，并引发脉冲信号中断
#define ITRNUM2 4
#define Color2_LED 32 //控制TCS3200颜色传感器是否点亮LED灯

#define TCSTIME 100
/*250+4*6 = 274*/
//typedef enum{
//    ColorWBBase = 250,
//    ColorWBR1 = ColorWBBase + 0,
//    ColorWBG1 = ColorWBBase + 4,
//    ColorWBB1 = ColorWBBase + 8,
//    ColorWBR2 = ColorWBBase + 12,
//    ColorWBG2 = ColorWBBase + 16,
//    ColorWBB2 = ColorWBBase + 20
//  }colorAddress;

enum
{
    COLORSENOR1,
    COLORSENOR2
};

enum
{
    RCOLOR,
    GCOLOR,
    BCOLOR,
    IDLECOLOR
};

extern uint32_t gCount1;
extern uint32_t gCount2;

extern uint32_t gColorCounter1[3], gColorCounter2[3];
extern uint32_t colorCounter1[3], colorCounter2[3];
extern int ColorInit(int port);
extern void ColorPortInit(int port);
extern int ColorDeinit(int port);
extern int ColorWB(int port);

extern void ColorCount1();
extern void ColorCount2();
extern void ColorDetectCallBack2();
extern void ColorDetectCallBack1();

#endif