#ifndef __IRMODULE_H
#define __IRMODULE_H

#define IRPIN1 25
#define IRPIN2 26
#define IRPIN3 27
#define IRPIN4 28
#define IRPIN5 29
#define IRPIN6 39

enum
{
  IR1 = 0,
  IR2,
  IR3,
  IR4,
  IR5,
  IR6
};
extern int IRInit();
extern int IRDetectPort(int port);
extern int IRDetect();

#endif
