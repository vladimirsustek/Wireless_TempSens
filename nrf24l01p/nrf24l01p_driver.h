/*
 * nrf24l01p_driver.h
 *
 *  Created on: Oct 31, 2022
 *      Author: 42077
 */

#ifndef INC_NRF24L01P_DRIVER_H_
#define INC_NRF24L01P_DRIVER_H_

#include "nrf24l01p_defines.h"

uint8_t NRF_initHostHW();
void NRF_configure(bool ptx_nprx);

 uint8_t NRF_getCONFIG(void);
 uint8_t NRF_getEN_AA(void);
 uint8_t NRF_getEN_RXADDR(void);
 uint8_t NRF_getSETUP_AW(void);
 uint8_t NRF_getSETUP_RETR(void);
 uint8_t NRF_getRF_CH(void);
 uint8_t NRF_getRF_SETUP(void);
 uint8_t NRF_getSTATUS(void);
 uint8_t NRF_getOBSERVE_TX(void);
 uint8_t NRF_getRPD(void);
 void NRF_getRX_ADDR_P0(uint8_t* arg, uint32_t lng);
 void NRF_getRX_ADDR_P1(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getRX_ADDR_P2(void);
 uint8_t NRF_getRX_ADDR_P3(void);
 uint8_t NRF_getRX_ADDR_P4(void);
 uint8_t NRF_getRX_ADDR_P5(void);
 void NRF_getTX_ADDR(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getRX_PW_P0(void);
 uint8_t NRF_getRX_PW_P1(void);
 uint8_t NRF_getRX_PW_P2(void);
 uint8_t NRF_getRX_PW_P3(void);
 uint8_t NRF_getRX_PW_P4(void);
 uint8_t NRF_getRX_PW_P5(void);
 uint8_t NRF_getFIFO_STATUS(void);
 uint8_t NRF_getDYNPD(void);
 uint8_t NRF_getFEATURE(void);

 void NRF_setCONFIG(uint8_t arg);
 void NRF_setEN_AA(uint8_t arg);
 void NRF_setEN_RXADDR(uint8_t arg);
 void NRF_setSETUP_AW(uint8_t arg);
 void NRF_setSETUP_RETR(uint8_t arg);
 void NRF_setRF_CH(uint8_t arg);
 void NRF_setRF_SETUP(uint8_t arg);
 void NRF_setSTATUS(uint8_t arg);
 void NRF_setOBSERVE_TX(uint8_t arg);
 void NRF_setRPD(uint8_t arg);
 void NRF_setRX_ADDR_P0(uint8_t* arg, uint32_t lng);
 void NRF_setRX_ADDR_P1(uint8_t* arg, uint32_t lngs);
 void NRF_setRX_ADDR_P2(uint8_t arg);
 void NRF_setRX_ADDR_P3(uint8_t arg);
 void NRF_setRX_ADDR_P4(uint8_t arg);
 void NRF_setRX_ADDR_P5(uint8_t arg);
 void NRF_setTX_ADDR(uint8_t* arg, uint32_t lng);
 void NRF_setFIFO_STATUS(uint8_t arg);
 void NRF_setDYNPD(uint8_t arg);
 void NRF_setFEATURE(uint8_t arg);

 void NRF_getR_RX_PAYLOAD(uint8_t* arg, uint32_t lng);
 void NRF_setW_TX_PAYLOAD(uint8_t* arg, uint32_t lng);
 void NRF_setFLUSH_TX(void);
 void NRF_setFLUSH_RX(void);
 void NRF_setREUSE_TX_PL(void);
 uint8_t NRF_getR_RX_PL_WID(void);
 void NRF_set_W_ACK_PAYLOAD(uint8_t pipe, uint8_t* arg, uint32_t lng);
 void NRF_setW_TX_PAYLOAD_NO_ACK(uint8_t* arg, uint32_t lng);
 uint8_t NRF_getNOP(void);
 void NRF_CEactivate(void);
 void NRF_CEdeactivate(void);

 uint8_t NRF_getIRQ(void);

 void NRF_powerUp(void);
 void NRF_powerDown(void);

 uint32_t NRF_activeRF(uint32_t (*msTickGet)(), void (*msDelay)(uint32_t), uint32_t timeOut);
 uint8_t NRF_postProcess(uint8_t pipe, uint8_t* rxBuff);
 uint32_t NRF_powerCycle(void (*msDelay)(uint32_t));
#endif /* INC_NRF24L01P_DRIVER_H_ */
