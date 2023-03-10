/*
 * nrf24l01p_func.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Vladimir Sustek
 */
#include <string.h>

#include "nrf24l01p_driver.h"


#define MAX_NRF_INSTANCES (uint32_t)(2)

static NRF_DEV_s nrfs[MAX_NRF_INSTANCES] = {0};
static int nrf_instances = 0;

NRF_DEV_s* new_NRFDevice(void)
{
	NRF_DEV_s* dev = NULL;

	if (nrf_instances == MAX_NRF_INSTANCES)
		return dev;

	dev = &(nrfs[nrf_instances]);
	nrf_instances++;

	dev->CEactivate = NRF_CEactivate;
	dev->CEdeactivate = NRF_CEdeactivate;

	dev->getCONFIG = NRF_getCONFIG;
	dev->getEN_AA = NRF_getEN_AA;
	dev->getEN_RXADDR = NRF_getEN_RXADDR;
	dev->getSETUP_AW = NRF_getSETUP_AW;
	dev->getSETUP_RETR = NRF_getSETUP_RETR;
	dev->getRF_CH = NRF_getRF_CH;
	dev->getRF_SETUP = NRF_getRF_SETUP;
	dev->getSTATUS = NRF_getSTATUS;
	dev->getOBSERVE_TX = NRF_getOBSERVE_TX;
	dev->getRPD = NRF_getRPD;
	dev->getRX_ADDR_P0 = NRF_getRX_ADDR_P0;
	dev->getRX_ADDR_P1 = NRF_getRX_ADDR_P1;
	dev->getRX_ADDR_P2 = NRF_getRX_ADDR_P2;
	dev->getRX_ADDR_P3 = NRF_getRX_ADDR_P3;
	dev->getRX_ADDR_P4 = NRF_getRX_ADDR_P4;
	dev->getRX_ADDR_P5 = NRF_getRX_ADDR_P5;
	dev->getTX_ADDR = NRF_getTX_ADDR;
	dev->getRX_PW_P0 = NRF_getRX_PW_P0;
	dev->getRX_PW_P1 = NRF_getRX_PW_P1;
	dev->getRX_PW_P2 = NRF_getRX_PW_P2;
	dev->getRX_PW_P3 = NRF_getRX_PW_P3;
	dev->getRX_PW_P4 = NRF_getRX_PW_P4;
	dev->getRX_PW_P5 = NRF_getRX_PW_P5;
	dev->getFIFO_STATUS = NRF_getFIFO_STATUS;
	dev->getDYNPD = NRF_getDYNPD;
	dev->getFEATURE = NRF_getFEATURE;

	dev->setCONFIG = NRF_setCONFIG;
	dev->setEN_AA = NRF_setEN_AA;
	dev->setEN_RXADDR = NRF_setEN_RXADDR;
	dev->setSETUP_AW = NRF_setSETUP_AW;
	dev->setSETUP_RETR = NRF_setSETUP_RETR;
	dev->setRF_CH = NRF_setRF_CH;
	dev->setRF_SETUP = NRF_setRF_SETUP;
	dev->setSTATUS = NRF_setSTATUS;
	dev->setOBSERVE_TX = NRF_setOBSERVE_TX;
	dev->setRPD = NRF_setRPD;
	dev->setRX_ADDR_P0 = NRF_setRX_ADDR_P0;
	dev->setRX_ADDR_P1 = NRF_setRX_ADDR_P1;
	dev->setRX_ADDR_P2 = NRF_setRX_ADDR_P2;
	dev->setRX_ADDR_P3 = NRF_setRX_ADDR_P3;
	dev->setRX_ADDR_P4 = NRF_setRX_ADDR_P4;
	dev->setRX_ADDR_P5 = NRF_setRX_ADDR_P5;
	dev->setTX_ADDR = NRF_setTX_ADDR;
	dev->setFIFO_STATUS = NRF_setFIFO_STATUS;
	dev->setDYNPD = NRF_setDYNPD;
	dev->setFEATURE = NRF_setFEATURE;

	return dev;
}

void init_NRFReceiver(NRF_DEV_s* dev, const char * const adr)
{
	if(strlen(adr) > NRF_MAX_ADR_LNG ||
			strlen(adr) << NRF_MIN_ADR_LNG ||
			dev == NULL ||
			adr == NULL)
		return;

/*
	cfg.CRCO = 1;
	cfg.PRIM_RX = 1;
	en_aa.ENAA_P0 = 1;
	en_rxadr.ERX_P0 = 1;
	setup_aw.AW = 0b11;
	setup_retr.ARD = 0b1111;
	setup_retr.ARC = 0b1111;
	rf_ch.RF_CH = 0b10;
	rf_setup.RF_DR_HIGH = 1;
	rf_setup.RF_PWR = 0b11;
	status.RX_DR = 1;
	status.TX_DS = 1;
	status.MAX_RT = 1;
	dynpd.DPL_P0 = 1;
	feature.EN_DPL = 1;

	dev->setCONFIG(cfg);
	dev->setEN_AA(en_aa);
	dev->setEN_RXADDR(en_rxadr);
	dev->setSETUP_AW(setup_aw);
	dev->setSETUP_RETR(setup_retr);
	dev->setRF_CH(rf_ch);
	dev->setRF_SETUP(rf_setup);
	dev->setSTATUS(status);
	dev->setRX_ADDR_P0((uint8_t*)adr, strlen((char*)adr));
	dev->setDYNPD(dynpd);
	dev->setFEATURE(feature);
*/
}


void init_NRFTransmitter(NRF_DEV_s* dev, const char * const adr)
{
	if(strlen(adr) > NRF_MAX_ADR_LNG ||
			strlen(adr) << NRF_MIN_ADR_LNG ||
			dev == NULL ||
			adr == NULL)
		return;

/*
	cfg.CRCO = 1;
	cfg.PRIM_RX = 1;
	en_aa.ENAA_P0 = 1;
	en_rxadr.ERX_P0 = 1;
	setup_aw.AW = 0b11;
	setup_retr.ARD = 0b1111;
	setup_retr.ARC = 0b1111;
	rf_ch.RF_CH = 0b10;
	rf_setup.RF_DR_HIGH = 1;
	rf_setup.RF_PWR = 0b11;
	status.RX_DR = 1;
	status.TX_DS = 1;
	status.MAX_RT = 1;
	dynpd.DPL_P0 = 1;
	feature.EN_DPL = 1;

	dev->setCONFIG(cfg);
	dev->setEN_AA(en_aa);
	dev->setEN_RXADDR(en_rxadr);
	dev->setSETUP_AW(setup_aw);
	dev->setSETUP_RETR(setup_retr);
	dev->setRF_CH(rf_ch);
	dev->setRF_SETUP(rf_setup);
	dev->setTX_ADDR((uint8_t*)adr, strlen((char*)adr));
	dev->setDYNPD(dynpd);
	dev->setSTATUS(status);
	dev->setFEATURE(feature);
*/
}
