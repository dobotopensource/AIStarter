#ifndef __HC_SR04_H
#define __HC_SR04_H

#define ECHO1 56 //PF2
#define TRIG1 46 //PL3

#define ECHO2 58 //PF4
#define TRIG2 6  //PH3

#define ECHO3 57 //PF3
#define TRIG3 45 //PL4

#define TIMEOUT 3000

enum Hcporttag
{
  SONAR1,
  SONAR2,
  SONAR3
};
extern int gSonarTime;
extern int SonarInit(int port);
extern float SonarDetect(int port);

#endif
