/*
 * nrf24l01p_func.h
 *
 *  Created on: Nov 1, 2022
 *      Author: 42077
 */

#ifndef INC_NRF24L01P_FUNC_H_
#define INC_NRF24L01P_FUNC_H_

#include "nrf24l01p_driver.h"

NRF_DEV_s* new_NRFDevice(void);
void init_NRFReceiver(NRF_DEV_s* dev, const char * const adr);
void init_NRFTransmitter(NRF_DEV_s* dev, const char * const adr);

#endif /* INC_NRF24L01P_FUNC_H_ */
