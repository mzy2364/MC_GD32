/*
 * task.h
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

#ifndef _TASK_H_
#define _TASK_H_

#ifdef __cplusplus
extern "C"{
#endif

#define ADC_FILTER_COUNT_POWER  4
#define ADC_FILTER_COUNT        (1<<ADC_FILTER_COUNT_POWER)

void task_init(void);
void task_scheduler(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_TASK_H_ */
