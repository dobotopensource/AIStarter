/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:             ObstacleAvoid
** Latest modified Date:  2018-12-5
** Latest Version:        V1.0.0
** Descriptions:          Ai-Starter Demo
*********************************************************************************************************/
#include <AIStarter.h>

#define  FRONTSPEED   100
#define  BACKSPEED    -100
#define  DIFSPEED     30
#define  BACKTIME     1000
#define  SWERVETIME   500
#define  DIS          5
//int MOTORR_tar_speed;
//int MOTORL_tar_speed;
enum{
    BACKRIGHT,
    BACKLEFT,
    AHEAD,
    ZERO
};

int motorStatus = AHEAD;
int rOffSet = 0;
int lOffSet = 0;
float dis[3];


/************* 初始化 **************/
void setup() 
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    pinMode(36, INPUT);
    
    AIStarter_SmartBotInit();
    Serial.println("SmartBotInit is completed");
    AIStarter_SmartBotSetLED(LED1,BLINK);
    Serial.println("*****");
    

}

void SET_MOTOR_SPEED(int MOTORR_tar_speed,int MOTORL_tar_speed)
{
    static int num = 0;
    static int MOTORL_last_tar_speed=0;
    static int MOTORL_last_speed=0;
    static int MOTORL_now_speed=0;
  
    static int MOTORR_last_tar_speed=0;
    static int MOTORR_last_speed=0;
    static int MOTORR_now_speed=0;
    if(MOTORR_tar_speed==MOTORR_last_tar_speed&&MOTORL_tar_speed==MOTORL_last_tar_speed){  
    }
    else{
        num=0;
        MOTORL_last_speed = MOTORL_now_speed;//保存当前速度
        MOTORR_last_speed = MOTORR_now_speed;//保存当前速度
        while(num<=10){
            num++;
            MOTORL_now_speed += (MOTORL_tar_speed-MOTORL_last_speed)/10;  //速度直线上升
            MOTORR_now_speed += (MOTORR_tar_speed-MOTORR_last_speed)/10;  //速度直线上升
            AIStarter_SmartBotSetMotor(MOTORR, MOTORR_now_speed);/*输出转速*/ 
            AIStarter_SmartBotSetMotor(MOTORL, MOTORL_now_speed);/*输出转速*/ 
            delay(50);
        }
        MOTORR_last_tar_speed = MOTORR_tar_speed;//保存上次传来的速度
        MOTORL_last_tar_speed = MOTORL_tar_speed;//保存上次传来的速度
    }
}
/************* 程序主循环 **************/
void loop() 
{
    if(!AIStarter_SmartBotGetKeyValue(37)) {
        delay(5);
        if(!AIStarter_SmartBotGetKeyValue(37)) {
            digitalWrite(BEEP,HIGH);
            delay(10);
        }
    } else {
        if(AIStarter_SmartBotGetKeyValue(37)) {
            digitalWrite(BEEP,LOW);
        }
    }
    // put your main code here, to run repeatedly:
    rOffSet = 0;
    lOffSet = 0;
    dis[0] = AIStarter_SmartBotGetSonar(SONAR1);
    dis[1] = AIStarter_SmartBotGetSonar(SONAR2);
    dis[2] = AIStarter_SmartBotGetSonar(SONAR3);
    do{
         if(dis[0] > 2*DIS ){
            motorStatus = AHEAD;
        }else if(dis[0] > DIS  && dis[0] < 2*DIS ){
            motorStatus = motorStatus;
        }else if(dis[0] > 0  && dis[0] < DIS ){
            motorStatus = BACKRIGHT;
            break;
        }   
        
        if(dis[1] > 2*DIS ){
            motorStatus = AHEAD;
        }else if(dis[1] > DIS  && dis[1] < 2*DIS ){
            motorStatus = motorStatus;
        }else if(dis[1] > 0  && dis[1] < DIS ){
            motorStatus = BACKRIGHT;
            break;
        }
        
        if(dis[2] > 2*DIS ){
            motorStatus = AHEAD;
        }else if(dis[2] > DIS  && dis[2] < 2*DIS ){
            motorStatus = motorStatus;
        }else if(dis[2] > 0  && dis[2] < DIS ){
            motorStatus = BACKLEFT;
            break;
        }
    }while(0);
   
    if((dis[2] < 0.001) && (dis[1] < 0.001) && (dis[0] < 0.001)){
        motorStatus = AHEAD;
    }
    if(promgramRun){
        switch(motorStatus){
            case BACKRIGHT:
                SET_MOTOR_SPEED(BACKSPEED,BACKSPEED);
                delay(BACKTIME);
                SET_MOTOR_SPEED(DIFSPEED,FRONTSPEED);
                delay(SWERVETIME);
            break;
            case BACKLEFT:
                SET_MOTOR_SPEED(BACKSPEED,BACKSPEED);
                delay(BACKTIME);
                SET_MOTOR_SPEED(FRONTSPEED,DIFSPEED);
                delay(SWERVETIME);
            break;
            case AHEAD:
                SET_MOTOR_SPEED(FRONTSPEED,FRONTSPEED);
            break;
            default:
            break;
        }
    }
    else{
      SET_MOTOR_SPEED(0,0);
    }

}
