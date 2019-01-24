#include <AIStarter.h>
void setup()
{
    AIStarter_SmartBotInit();
    AIStarter_SmartBotServoAttach(SERVO1);
}

void loop() {
    int pmwServo = 0;
    for (pmwServo = 0; pmwServo <= 180; pmwServo += 1) {
        AIStarter_SmartBotServoWrite(SERVO1,pmwServo);
        delay(15);
    }
    for (pmwServo = 180; pmwServo >= 0; pmwServo -= 1) {
        AIStarter_SmartBotServoWrite(SERVO1,pmwServo);
        delay(15);
    }
}
