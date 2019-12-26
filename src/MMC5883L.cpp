
#include "MMC5883L.h"

#define YAW_AXIS_IS_X 0
#define YAW_AXIS_IS_Y 0
#define YAW_AXIS_IS_Z 1

#define MIN_COEFFICIENT 0.5
#define MAX_COEFFICIENT 10.0
#define DEF_COEFFICIENT 1.0

static bool start = true;
int offsetY, offsetX, offsetZ;

#if EN_COEFX
float coefficientX, coefficientZ, coefficientY;
#endif

static bool limit_value(float *coeff, const float min, const float max, const float def)
{
    if (*coeff > max || *coeff < min || isnan(*coeff))
    {
        *coeff = def;
        return false;
    }
    return true;
}

int CompassSenorInit()
{
    i2c.beginTransmission(ADDRESS);
    i2c.send(MMC5883MA_REG_CTRL0);
    i2c.send(MMC5883MA_CMD_SET);
    i2c.endTransmission();
    delay(1);
    i2c.beginTransmission(ADDRESS);
    i2c.send(MMC5883MA_REG_CTRL0);
    i2c.send(MMC5883MA_CMD_TM_M);
    i2c.endTransmission();
    /*Read Eeprom*/
    EepromRead(CompassOffsetY, &offsetY, sizeof(offsetY));
    EepromRead(CompassOffsetX, &offsetX, sizeof(offsetX));
    EepromRead(CompassOffsetZ, &offsetZ, sizeof(offsetZ));
#if EN_COEFX
    EepromRead(CompassCoefficientX, &coefficientX, sizeof(coefficientX));
    EepromRead(CompassCoefficientY, &coefficientY, sizeof(coefficientY));
    EepromRead(CompassCoefficientZ, &coefficientZ, sizeof(coefficientZ));

    limit_value(&coefficientX, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT);
    limit_value(&coefficientY, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT);
    limit_value(&coefficientZ, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT);

#endif
    Serial.println("CompassInit!  ");
    Serial.print("offsetX:");
    Serial.print(offsetX);
    Serial.print("   offsetY:");
    Serial.print(offsetY);
    Serial.print("   offsetZ:");
    Serial.println(offsetZ);
#if EN_COEFX
    Serial.print("coefficientX:");
    Serial.print(coefficientX);
    Serial.print("   coefficientY:");
    Serial.print(coefficientY);
    Serial.print("   coefficientZ:");
    Serial.println(coefficientZ);
#endif

    return 0;
}

//axis2 is the geomagnetic direction??
int calculateHeading(int *axis1, int *axis2)
{
    float headingRadians = atan2((double)((*axis1)), (double)((*axis2)));
    // 保证数据在0-2*PI之间
    if (headingRadians < 0)
        headingRadians += 2 * PI;

    int headingDegrees = headingRadians * 180 / M_PI;
    headingDegrees += MagnetcDeclination; //磁偏角

    // 保证数据在0-360之间
    if (headingDegrees > 360)
        headingDegrees -= 360;

    return headingDegrees;
}

float CompassSenorDetect()
{
    int x, y, z;
    uint16_t tempx, tempy, tempz;
    //static int time = millis();

    i2c.beginTransmission(ADDRESS);
    i2c.send(MMC5883MA_REG_CTRL0);
    i2c.send(MMC5883MA_CMD_SET);
    i2c.endTransmission();
    delay(1);
    i2c.beginTransmission(ADDRESS);
    i2c.send(MMC5883MA_REG_CTRL0);
    i2c.send(MMC5883MA_CMD_TM_M);
    i2c.endTransmission();
    delay(1);

    i2c.beginTransmission(ADDRESS);
    i2c.send(0x00); //select register 3, X MSB register
    i2c.endTransmission();
    i2c.requestFrom(ADDRESS);

    tempx = i2c.receive();           //X msb
    tempx |= i2c.receive() << 8;     //X lsb
    tempy = i2c.receive();           //Y msb
    tempy |= i2c.receive() << 8;     //Y lsb
    tempz = i2c.receive();           //Z msb
    tempz |= i2c.receiveLast() << 8; //Z lsb
    i2c.endTransmission();

#if EN_COEFX

    x = coefficientX * (tempx - 32768 - offsetX);
    y = coefficientY * (tempy - 32768 - offsetY);
    z = coefficientZ * (tempz - 32768 - offsetZ);
#else
    x = tempx - 32768 - offsetX;
    y = tempy - 32768 - offsetY;
    z = tempz - 32768 - offsetZ;
#endif

    i2c.beginTransmission(ADDRESS);
    i2c.send(MMC5883MA_REG_CTRL0);
    i2c.send(MMC5883MA_CMD_TM_M);
    i2c.endTransmission();
#if YAW_AXIS_IS_X
    float angle = calculateHeading(&y, &z);
#elif YAW_AXIS_IS_Y
    float angle = calculateHeading(&x, &z);
#elif YAW_AXIS_IS_Z
    float angle = calculateHeading(&x, &y);
#endif

#if 1
    Serial.print("tempx: ");
    Serial.print(tempx);
    Serial.print(";tempy: ");
    Serial.print(tempy);
    Serial.print(";tempz: ");
    Serial.print(tempz);

    Serial.print(";x: ");
    Serial.print(x);
    Serial.print("  ;y: ");
    Serial.print(y);
    Serial.print("  ;z: ");
    Serial.print(z);

#if YAW_AXIS_IS_X
    Serial.print(" ;angle(y,z): ");
#elif YAW_AXIS_IS_Y
    Serial.print(" ;angle(x,z): ");
#elif YAW_AXIS_IS_Z
    Serial.print(" ;angle(x,y): ");
#endif

    Serial.println(angle);

#endif
    return angle;
}

//+-------------------+---------------------+
//                    ^ X
//              +-----------+
//              |     |     |
//              |    +++    |
//       Y <---------+X| Z  | car is heading to the z+ direction()
//              |    +-+    | ( perpendicular to the screen toward the inside)
//              |         O |
//              +-----------+
//+-----------------------------------------+
int CompassSenorCalibration()
{
    int x, y, z;
    static int yMax, yMin, xMax, xMin, zMax, zMin;
    uint16_t tempx, tempy, tempz;
    //static uint8_t count = 0;
    static int time = millis();
    Serial.println("CompassSenor Starting Calibration......");

    Serial.println("Press left button to finish");
    while (1)
    {
        if (millis() - time > 1000 && !digitalRead(CALIB_END))
        {
            Serial.println("Calibration finished");
            break;
        }

        i2c.beginTransmission(ADDRESS);
        i2c.send(MMC5883MA_REG_CTRL0);
        i2c.send(MMC5883MA_CMD_SET);
        i2c.endTransmission();
        delay(1);
        i2c.beginTransmission(ADDRESS);
        i2c.send(MMC5883MA_REG_CTRL0);
        i2c.send(MMC5883MA_CMD_TM_M);
        i2c.endTransmission();
        delay(1);

        i2c.beginTransmission(ADDRESS);
        i2c.send(0x00); //select register 3, X MSB register
        i2c.endTransmission();
        i2c.requestFrom(ADDRESS);

        tempx = i2c.receive();           //X msb
        tempx |= i2c.receive() << 8;     //X lsb
        tempy = i2c.receive();           //Y msb
        tempy |= i2c.receive() << 8;     //Y lsb
        tempz = i2c.receive();           //Z msb
        tempz |= i2c.receiveLast() << 8; //Z lsb
        i2c.endTransmission();

        x = tempx - 32768;
        y = tempy - 32768;
        z = tempz - 32768;

        if (start)
        {
            xMin = yMin = zMin = 32767;
            xMax = yMax = zMax = -32768;
            start = false;
        }
        xMax = (x > xMax) ? x : xMax;
        xMin = (x < xMin) ? x : xMin;
        yMax = (y > yMax) ? y : yMax;
        yMin = (y < yMin) ? y : yMin;
        zMax = (z > zMax) ? z : zMax;
        zMin = (z < zMin) ? z : zMin;

        i2c.beginTransmission(ADDRESS);
        i2c.send(MMC5883MA_REG_CTRL0);
        i2c.send(MMC5883MA_CMD_TM_M);
        i2c.endTransmission();

        delay(20);
    }

    offsetX = (xMax + xMin) / 2;
    offsetY = (yMax + yMin) / 2;
    offsetZ = (zMax + zMin) / 2;

    if (xMax - xMin < MI_X_MIN_MAX_THR)
    {
        Serial.print("offsetX calibration failed!!!");
    }
    else
    {
        EepromWrite(CompassOffsetX, &offsetX, sizeof(offsetX));
#if EN_COEFX
        coefficientX = 1;
        if (!limit_value(&coefficientX, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT))
        {
            Serial.print("coefficientX error!!!");
        }
        EepromWrite(CompassCoefficientX, &coefficientX, sizeof(coefficientX));
        Serial.print("coefficientX : ");
        Serial.println(coefficientX);
#endif
    }
    Serial.print("xMax : ");
    Serial.print(xMax);
    Serial.print("  xMin : ");
    Serial.print(xMin);
    Serial.print("  offsetX : ");
    Serial.println(offsetX);

    if (yMax - yMin < MI_Y_MIN_MAX_THR)
    {
        Serial.print("offsetY calibration failed!!!");
    }
    else
    {
        EepromWrite(CompassOffsetY, &offsetY, sizeof(offsetY));
#if EN_COEFX
        coefficientY = (xMax - xMin) / (yMax - yMin);
        if (!limit_value(&coefficientY, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT))
        {
            Serial.print("coefficientY error!!!");
        }
        EepromWrite(CompassCoefficientY, &coefficientY, sizeof(coefficientY));
        Serial.print("coefficientY : ");
        Serial.println(coefficientY);
#endif
    }
    Serial.print("yMax : ");
    Serial.print(yMax);
    Serial.print("  yMin : ");
    Serial.print(yMin);
    Serial.print("  offsetY : ");
    Serial.println(offsetY);

    if (zMax - zMin < MI_Z_MIN_MAX_THR)
    {
        Serial.print("offsetZ calibration failed!!!");
    }
    else
    {
        EepromWrite(CompassOffsetZ, &offsetZ, sizeof(offsetZ));
#if EN_COEFX
        coefficientZ = abs((float)(xMax - xMin) / (zMax - zMin)); //right order??
        if (!limit_value(&coefficientZ, MIN_COEFFICIENT, MAX_COEFFICIENT, DEF_COEFFICIENT))
        {
            Serial.print("coefficientZ error!!!");
        }
        EepromWrite(CompassCoefficientZ, &coefficientZ, sizeof(coefficientZ));
        Serial.print("coefficientZ : ");
        Serial.println(coefficientZ);
#endif
    }
    Serial.print("zMax : ");
    Serial.print(zMax);
    Serial.print("  zMin : ");
    Serial.print(zMin);
    Serial.print("  offsetZ : ");
    Serial.println(offsetZ);

    return 0;
}
