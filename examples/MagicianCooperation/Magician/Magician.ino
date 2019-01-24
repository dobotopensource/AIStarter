#include <Arduino.h>
#include <Dobot.h>
#include <Pixy2.h>
#include <Pixy2I2C.h>
#include <Pixy2CCC.h>
#include <TPixy2.h>

Pixy2I2C pixy;
bool flag = true;
#define InitPositionX  260
#define InitPositionY  0
#define InitPositionZ  60
#define InitPositionR  0

float AreaPoint[4][3] = {
  {137.05, -206.94, -39},
  {137.05, -244.31, -39},
  {100.50, -206.94, -39},
  {100.50, -244.31, -39}
};

float trayPoint[4][3] = {
  {308.12,  25.92,  28},
  {308.12, -15.92,  28},
  {258.12,  25.92,  28},
  {258.12, -15.92,  28}
};

/********** 从方块区吸取方块，放置托盘 **********/
void AreaToAIStarter()
{
    for(uint8_t i=0; i<4; i++){
        Dobot_SetPTPCmdEx(JUMP_XYZ, AreaPoint[i][0], AreaPoint[i][1], AreaPoint[i][2], 0);
        Dobot_SetEndEffectorSuctionCupEx(true);
        Dobot_SetPTPCmdEx(MOVL_XYZ, AreaPoint[i][0], AreaPoint[i][1], AreaPoint[i][2]+70, 0);
        Dobot_SetPTPCmdEx(JUMP_XYZ, trayPoint[i][0], trayPoint[i][1], trayPoint[i][2], 0);
        Dobot_SetEndEffectorSuctionCupEx(false);
        Dobot_SetPTPCmdEx(MOVL_XYZ, trayPoint[i][0], trayPoint[i][1], trayPoint[i][2]+30, 0);
    }
    Dobot_SetPTPCmdEx(MOVJ_XYZ, InitPositionX, InitPositionY, InitPositionZ, InitPositionR);
}

void setup()
{
    Serial.begin(115200);
    pixy.init();
    pixy.setLamp(1, 1);
    Dobot_Init();
    Dobot_SetEndEffectorParamsEx(59.7,0,0);
    Dobot_SetPTPCommonParamsEx(100,100);
    Dobot_SetPTPJumpParamsEx(25);
    Dobot_SetPTPCmdEx(MOVJ_XYZ, InitPositionX, InitPositionY, InitPositionZ, InitPositionR);
}

void loop()
{
    pixy.ccc.getBlocks();
    delay(10);
    pixy.ccc.getBlocks();
    delay(10);
    if(pixy.ccc.numBlocks == 0){
        delay(3000);
        Dobot_SetPTPCmdEx(MOVJ_XYZ, InitPositionX, InitPositionY, InitPositionZ, InitPositionR);
        return 0;
    } else if (pixy.ccc.numBlocks) {
        AreaToAIStarter();
    }
}
