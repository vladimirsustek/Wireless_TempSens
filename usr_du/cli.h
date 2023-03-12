/*
 * cli.h
 *
 *  Created on: Dec 12, 2022
 *      Author: 42077
 */

#ifndef CLI_H_
#define CLI_H_

#include <stdio.h>

#include "usart.h"
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif


#define CLI_BUFF_SIZE (uint32_t)(128)

typedef struct cli
{
	uint8_t* pBegin;
	uint32_t length;
}cli_t;

void cli_init(void);
void cli_deInit(void);
cli_t cli_process(void);

#endif /* CLI_H_ */
