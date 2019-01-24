#include <AIStarter.h>
#define carSpeed 100
String strCommand;
float rOffSet,lOffSet;
void setup()
{
    AIStarter_SmartBotInit();
}

void loop()
{
    strCommand = AIStarter_SmartBotXbeeRead();
    if(!AIStarter_SmartBotXbeeCompare(strCommand, "Ahead")) {
        rOffSet = 1;
        lOffSet = 1;
    } else if(!AIStarter_SmartBotXbeeCompare(strCommand, "Back")) {
        rOffSet = -1;
        lOffSet = -1;
    } else if(!AIStarter_SmartBotXbeeCompare(strCommand, "TurnLeft")) {
        rOffSet = 1;
        lOffSet = 0.5; 
    } else if(!AIStarter_SmartBotXbeeCompare(strCommand, "TurnRight")) {
        rOffSet = 0.5;
        lOffSet = 1;
    } else if(!AIStarter_SmartBotXbeeCompare(strCommand, "Stop")) {
        rOffSet = 0;
        lOffSet = 0;
    }
    delay(2);
    AIStarter_SmartBotSetMotor(MOTORR,rOffSet*carSpeed);
    AIStarter_SmartBotSetMotor(MOTORL,lOffSet*carSpeed);
    AIStarter_SmartBotXbeeClear();
}
