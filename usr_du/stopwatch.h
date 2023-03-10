/*
 * stopwatch.h
 *
 *  Created on: Dec 11, 2022
 *      Author: 42077
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_


#define DEBUG_EXECUTION_TIME_A 1
#define DEBUG_EXECUTION_TIME_B 1

#if DEBUG_EXECUTION_TIME_A
TIM_HandleTypeDef htim2;

#define STOPWATCH_A_INIT()		(HAL_TIM_Base_Start(&htim2))
#define STOPWATCH_A_START() 	(htim2.Instance->CNT = 0)
#define STOPWATCH_A_STOP()  	(htim2.Instance->CNT)
#define STOPWATCH_A_PRINT(idx) 	(printf("%d:%lu\n",(idx), STOPWATCH_STOP()))
#define STOPWATCH_A_WAIT(microsec) ({htim2.Instance->CNT = 0; while((microsec) > htim2.Instance->CNT);})
#else
#define STOPWATCH_A_INIT()		({})
#define STOPWATCH_A_START() 	({})
#define STOPWATCH_A_STOP() 		({})
#define STOPWATCH_A_PRINT(idx)	({(idx);})
#define STOPWATCH_A_WAIT(microsec) ({(idx);})
#endif

#if DEBUG_EXECUTION_TIME_B
TIM_HandleTypeDef htim3;

#define STOPWATCH_B_INIT()		(HAL_TIM_Base_Start(&htim3))
#define STOPWATCH_B_START() 	(htim3.Instance->CNT = 0)
#define STOPWATCH_B_STOP()  	(htim3.Instance->CNT)
#define STOPWATCH_B_PRINT(idx) 	(printf("%d:%lu\n",(idx), STOPWATCH_STOP()))
#define STOPWATCH_B_WAIT(microsec) ({htim3.Instance->CNT = 0; while((microsec) != htim3.Instance->CNT);})
#else
#define STOPWATCH_B_INIT()		({})
#define STOPWATCH_B_START() 	({})
#define STOPWATCH_B_STOP() 		({})
#define STOPWATCH_B_PRINT(idx)	({(idx);})
#define STOPWATCH_B_WAIT(microsec) ({(idx);})
#endif

#endif /* STOPWATCH_H_ */
