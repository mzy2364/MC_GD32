/*
*******************************************************************************************************
*
* File Name : can.h
* Version   : V1.0
* Author    : mzy2364
* brief     : can header file
* 
*******************************************************************************************************
*/
#ifndef _CAN_H_
#define _CAN_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
/* select CAN baudrate */
/* 1MBps */
/* #define CAN_BAUDRATE  1000  */
/* 500kBps */
#define CAN_BAUDRATE  500
/* 250kBps */
/* #define CAN_BAUDRATE  250 */
/* 125kBps */
/* #define CAN_BAUDRATE  125 */
/* 100kBps */ 
/* #define CAN_BAUDRATE  100 */
/* 50kBps */ 
/* #define CAN_BAUDRATE  50 */
/* 20kBps */ 
/* #define CAN_BAUDRATE  20 */

/* VARIABLES ----------------------------------------------------------------------------------------*/
extern FlagStatus can0_receive_flag;
extern can_receive_message_struct receive_message;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void can_bus_init(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _CAN_H_ */

/***************************************** (END OF FILE) *********************************************/
