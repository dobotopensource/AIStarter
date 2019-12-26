#ifndef __E2PROM_H
#define __E2PROM_H

#include <EEPROM.h>
#include "Arduino.h"

typedef enum
{
    VERSIONADD = 212,  //education or contest
    ColorWBBase = 250, //color sensor
    ColorWBR1 = ColorWBBase + 0,
    ColorWBG1 = ColorWBBase + 4,
    ColorWBB1 = ColorWBBase + 8,
    ColorWBR2 = ColorWBBase + 12,
    ColorWBG2 = ColorWBBase + 16,
    ColorWBB2 = ColorWBBase + 20,
    CompassBase = 290,
    CompassOffsetX = CompassBase + 0,       //int is 2byte(arduino platform)
    CompassOffsetY = CompassBase + 2,       //int is 2byte(arduino platform)
    CompassOffsetZ = CompassBase + 4,       //int is 2byte(arduino platform)
    CompassCoefficientX = CompassBase + 6,  //float is 4byte(arduino platform)
    CompassCoefficientY = CompassBase + 10, //float is 4byte(arduino platform)
    CompassCoefficientZ = CompassBase + 14, //float is 4byte(arduino platform)
} compassAddress;

/*eeprom*/
extern void Eeprom(void);
extern void EepromWrite(int add, float *var, int len);
extern void EepromRead(int add, float *var, int len);
extern void EepromWrite(int add, int *var, int len);
extern void EepromRead(int add, int *var, int len);
#endif
