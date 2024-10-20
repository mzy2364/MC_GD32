#ifndef Parms_H
#define Parms_H
#include "system_define.h"

EXTERN struct 
{

	uint16_t LockTime;

	long EndSpeed;

} MotorParm;


bool InitMotorParm(void);

#endif




