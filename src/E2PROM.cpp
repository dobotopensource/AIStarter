#include "E2PROM.h"

/**********EEPROM******************/
void EepromWrite(int add, float *var, int len)
{
    uint8_t tempin[8];
    //float temptest1;
    memcpy(tempin, var, len);
    for (int i = 0; i < len; i++)
    {
        EEPROM.write(add + i, tempin[i]);
    }
}

void EepromRead(int add, float *var, int len)
{
    uint8_t tempout[8];
    for (int i = 0; i < len; i++)
    {
        tempout[i] = EEPROM.read(add + i);
    }
    memcpy(var, tempout, sizeof(float));
}
void EepromWrite(int add, int *var, int len)
{
    uint8_t tempin[8];
    //float temptest1;
    memcpy(tempin, var, len);
    for (int i = 0; i < len; i++)
    {
        EEPROM.write(add + i, tempin[i]);
    }
}

void EepromRead(int add, int *var, int len)
{
    uint8_t tempout[8];
    for (int i = 0; i < len; i++)
    {
        tempout[i] = EEPROM.read(add + i);
    }
    memcpy(var, tempout, sizeof(int));
}
