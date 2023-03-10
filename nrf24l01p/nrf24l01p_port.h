/*
 * nrf24l01p_port.h
 *
 *  Created on: Nov 19, 2022
 *      Author: 42077
 */

#ifndef NRF24L01P_PORT_H_
#define NRF24L01P_PORT_H_

#include "nrf24l01p_defines.h"

uint8_t nrfport_initHostHW();
void nrfport_spiTransaction(uint8_t* tx, uint8_t *rx, uint32_t lng);
void nrfport_writeByte(uint8_t adr, uint8_t data);
uint8_t nrfport_readByte(uint8_t adr);
uint8_t nrfport_getIRQ(void);
void nrfport_ceHigh(void);
void nrfport_ceLow(void);
void nrfport_powerUp(void);
void nrfport_powerDown(void);
#endif /* NRF24L01P_PORT_H_ */
