/*
 * cli.h
 *
 *  Created on: Dec 12, 2022
 *      Author: 42077
 */

#ifndef CLI_H_
#define CLI_H_

#include "usart.h"

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
