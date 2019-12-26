#ifndef __MMC5883L_H
#define __MMC5883L_H

#include "Arduino.h"
#include "SoftI2CMaster.h"
#include "E2PROM.h"

#define ADDRESS 0x30         //
#define MagnetcDeclination 2 //笔者所在地磁偏角，请根据情况自行百度
#define CalThreshold 0
#define EN_COEFX 1

#define MMC5883MA_REG_DATA 0x00
#define MMC5883MA_REG_TEMP 0x06
#define MMC5883MA_REG_STATUS 0x07
#define MMC5883MA_REG_CTRL0 0x08
#define MMC5883MA_REG_CTRL1 0x09
#define MMC5883MA_REG_CTRL2 0x0A
#define MMC5883MA_REG_PRODUCTID 0x2F

#define MMC5883MA_CMD_RESET 0x10
#define MMC5883MA_CMD_SET 0x08
#define MMC5883MA_CMD_TM_M 0x01
#define MMC5883MA_CMD_TM_T 0x02
#define MMC5883MA_PRODUCT_ID 0x0C

//defualt value??
#define OFFSETX 1669
#define OFFSETY -12649
#define XCOE 1.05

//offset threshhold
#define MI_X_MIN_MAX_THR 100
#define MI_Y_MIN_MAX_THR 100
#define MI_Z_MIN_MAX_THR 100

#define PINSW3 37
#define CALIB_END PINSW3 //for calibration end

extern int CompassSenorInit();
extern float CompassSenorDetect();
extern int CompassSenorCalibration();
#endif