/*
*******************************************************************************************************
*
* File Name : main.c
* Version   : V1.0
* Author    : mzy2364
* brief     : main function file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_app.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
}

/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    BaseType_t ret;
    
    SCB->VTOR = FLASH_BASE | 0x4000;
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    
    /* Create Init Task */
    ret = xTaskCreate(
        pmsm_init,
        "pmsm_init_tsk",
        256u,
        NULL,
        4u,
        NULL
    );
    configASSERT(ret == pdPASS);

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();
    
    return 0;
}


/***************************************** (END OF FILE) *********************************************/
