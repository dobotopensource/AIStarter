#include "TCS3200.h"
#include "Arduino.h"
#include "TimerOne.h"
#include "stdio.h"
#include "SoftI2CMaster.h"

#define TCSDEBUG false
#define COMPENSATE (0.8)
uint8_t temp[128];

float gColorSF1[3], gColorSF2[3]; // Color coefficient

uint32_t gCount1 = 0; // 脉冲计数
uint32_t gCount2 = 0;

/*保存数据脉冲，以及色值*/
uint32_t gColorCounter1[3], gColorCounter2[3];
uint32_t colorCounter1[3], colorCounter2[3];

int colorStatue1 = RCOLOR; // 滤波器模式选择顺序标志
int colorStatue2 = RCOLOR;

void ColorDetectCallBack1()
{
    static float vocoe[3] = {0.7, 0.8, 0.8};
    if (!digitalRead(41))
    {
        vocoe[0] = 0.75;
        vocoe[1] = 0.8;
        vocoe[2] = 0.8;
    }
    else
    {
        vocoe[0] = 1;
        vocoe[1] = 1;
        vocoe[2] = 1;
    }
    switch (colorStatue1)
    {
    case RCOLOR:
        gCount1 = 0;
        digitalWrite(Color1_S2, LOW);
        digitalWrite(Color1_S3, LOW); //选择让红色光线通过滤波器的模式
        colorStatue1 = GCOLOR;
        break;
    case GCOLOR:
        colorCounter1[0] = gCount1;
        gCount1 = 0;
        digitalWrite(Color1_S2, HIGH);
        digitalWrite(Color1_S3, HIGH); //选择让绿色光线通过滤波器的模式
        colorStatue1 = BCOLOR;
        break;
    case BCOLOR:
        colorCounter1[1] = gCount1;
        gCount1 = 0;
        digitalWrite(Color1_S2, LOW);
        digitalWrite(Color1_S3, HIGH); //选择让蓝色光线通过滤波器的模式
        colorStatue1 = IDLECOLOR;
        break;
    case IDLECOLOR:
        colorCounter1[2] = gCount1;
        gCount1 = 0;
        digitalWrite(Color1_S2, LOW);
        digitalWrite(Color1_S3, LOW); //选择让红色光线通过滤波器的模式
        colorStatue1 = GCOLOR;
        memcpy(gColorCounter1, colorCounter1, sizeof(uint32_t) * 3);
        for (int i = 0; i < 3; i++)
        {
            gColorCounter1[i] = colorCounter1[i] * gColorSF1[i] / vocoe[i];
            if (gColorCounter1[i] > 255)
            {
                gColorCounter1[i] = 255;
            }
        }
        break;
    }
}
void ColorDetectCallBack2()
{
    static float vocoe[3] = {0.7, 0.8, 0.8};

    if (!digitalRead(41))
    {
        vocoe[0] = 0.75;
        vocoe[1] = 0.8;
        vocoe[2] = 0.8;
    }
    else
    {
        vocoe[0] = 1;
        vocoe[1] = 1;
        vocoe[2] = 1;
    }
    switch (colorStatue2)
    {
    case RCOLOR:
        gCount2 = 0;
        digitalWrite(Color2_S2, LOW);
        digitalWrite(Color2_S3, LOW); //选择让红色光线通过滤波器的模式
        colorStatue2 = GCOLOR;
        break;
    case GCOLOR:
        colorCounter2[0] = gCount2;
        gCount2 = 0;
        digitalWrite(Color2_S2, HIGH);
        digitalWrite(Color2_S3, HIGH); //选择让绿色光线通过滤波器的模式
        colorStatue2 = BCOLOR;
        break;
    case BCOLOR:
        colorCounter2[1] = gCount2;
        gCount2 = 0;
        digitalWrite(Color2_S2, LOW);
        digitalWrite(Color2_S3, HIGH); //选择让蓝色光线通过滤波器的模式
        colorStatue2 = IDLECOLOR;
        break;
    case IDLECOLOR:
        colorCounter2[2] = gCount2;
        gCount2 = 0;
        digitalWrite(Color2_S2, LOW);
        digitalWrite(Color2_S3, LOW); //选择让红色光线通过滤波器的模式
        colorStatue2 = GCOLOR;

        memcpy(gColorCounter2, colorCounter2, sizeof(uint32_t) * 3);
        for (int i = 0; i < 3; i++)
        {
            gColorCounter2[i] = colorCounter2[i] * gColorSF2[i] / vocoe[i];
            if (gColorCounter2[i] > 255)
            {
                gColorCounter2[i] = 255;
            }
        }
        break;
    }
}

//中断函数，计算TCS3200输出信号的脉冲数
void ColorCount1()
{
    gCount1++;
}

void ColorCount2()
{
    gCount2++;
}

int ColorWB(int port)
{
    delay(500);
    if (port == COLORSENOR1)
    {
        gColorSF1[0] = 255.0 / colorCounter1[0]; //红色光比例因子
        gColorSF1[1] = 255.0 / colorCounter1[1]; //绿色光比例因子
        gColorSF1[2] = 255.0 / colorCounter1[2]; //蓝色光比例因子

        EepromWrite(ColorWBR1, &gColorSF1[0], sizeof(gColorSF1[0]));
        EepromWrite(ColorWBG1, &gColorSF1[1], sizeof(gColorSF1[1]));
        EepromWrite(ColorWBB1, &gColorSF1[2], sizeof(gColorSF1[2]));
        Serial.print("ColorWB1,R:");
        Serial.print(gColorSF1[0], 5);
        Serial.print("    G:");
        Serial.print(gColorSF1[1], 5);
        Serial.print("    B:");
        Serial.println(gColorSF1[2], 5);
    }
    else if (port == COLORSENOR2)
    {
        gColorSF2[0] = 255.0 / colorCounter2[0]; //红色光比例因子
        gColorSF2[1] = 255.0 / colorCounter2[1]; //绿色光比例因子
        gColorSF2[2] = 255.0 / colorCounter2[2]; //蓝色光比例因子
        EepromWrite(ColorWBR2, &gColorSF2[0], sizeof(gColorSF2[0]));
        EepromWrite(ColorWBG2, &gColorSF2[1], sizeof(gColorSF2[1]));
        EepromWrite(ColorWBB2, &gColorSF2[2], sizeof(gColorSF2[2]));
        Serial.print("ColorWB2,R:");
        Serial.print(gColorSF2[0], 5);
        Serial.print("    G:");
        Serial.print(gColorSF2[1], 5);
        Serial.print("    B:");
        Serial.println(gColorSF2[2], 5);
    }
    return 0;
}

int ColorInit(int port)
{
    if (port == COLORSENOR1)
    {
        Serial.print("ColorInit1");
        pinMode(Color1_S2, OUTPUT);
        pinMode(Color1_S3, OUTPUT);
        pinMode(Color1_OUT, INPUT);
        pinMode(Color1_LED, OUTPUT);
        float TempSF[3];
        EepromRead(ColorWBR1, &TempSF[0], sizeof(TempSF[0]));
        EepromRead(ColorWBG1, &TempSF[1], sizeof(TempSF[1]));
        EepromRead(ColorWBB1, &TempSF[2], sizeof(TempSF[2]));
        if (TempSF[0] == 0xff || TempSF[0] == 0.00)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                gColorSF1[i] = 3;
            }
        }
        else
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                gColorSF1[i] = TempSF[i];
            }
        }
        Serial.print("    R:");
        Serial.print(gColorSF1[0], 5);
        Serial.print("    G:");
        Serial.print(gColorSF1[1], 5);
        Serial.print("    B:");
        Serial.println(gColorSF1[2], 5);
        attachInterrupt(ITRNUM1, ColorCount1, RISING);
        digitalWrite(Color1_LED, HIGH); //点亮LED灯
    }
    else if (port == COLORSENOR2)
    {
        Serial.print("ColorInit2");
        pinMode(Color2_S2, OUTPUT);
        pinMode(Color2_S3, OUTPUT);
        pinMode(Color2_OUT, INPUT);
        pinMode(Color2_LED, OUTPUT);
        float TempSF[3];
        EepromRead(ColorWBR2, &TempSF[0], sizeof(TempSF[0]));
        EepromRead(ColorWBG2, &TempSF[1], sizeof(TempSF[1]));
        EepromRead(ColorWBB2, &TempSF[2], sizeof(TempSF[2]));
        if (TempSF[0] == 0xFF || TempSF[0] == 0x00)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                gColorSF2[i] = 3;
            }
        }
        else
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                gColorSF2[i] = TempSF[i];
            }
        }
        Serial.print("    R:");
        Serial.print(gColorSF2[0], 5);
        Serial.print("    G:");
        Serial.print(gColorSF2[1], 5);
        Serial.print("    B:");
        Serial.println(gColorSF2[2], 5);
        attachInterrupt(ITRNUM2, ColorCount2, RISING);
        digitalWrite(Color2_LED, HIGH); //点亮LED灯
    }
    return 0;
}

int ColorDeinit(int port)
{
    if (port == COLORSENOR1)
    {
        detachInterrupt(ITRNUM1);
        pinMode(Color1_LED, INPUT);
        digitalWrite(Color1_LED, LOW); //关闭LED灯
        gCount1 = 0;
    }
    else if (port == COLORSENOR2)
    {
        detachInterrupt(ITRNUM2);
        pinMode(Color2_LED, INPUT);
        digitalWrite(Color2_LED, LOW); //关闭LED灯
        gCount2 = 0;
    }
    return 0;
}
