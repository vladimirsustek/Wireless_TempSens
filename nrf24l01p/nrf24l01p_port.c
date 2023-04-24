/*
 * nrf24l01p_port.c
 *
 *  Created on: Nov 19, 2022
 *      Author: Vladimir Sustek, MSc
 */


/**
  * @brief  Porting layer (SPI, CE GPIO and ISR GPIO) for NRF24L01+.
  *
  * NRF24L01+ needs following peripheral:
  *
  * SPI to operate with pins (max SCK 10MHz):
  * - SCK
  * - MISO
  * - MOSI
  * - CHIP SELECT (May be a SW-controlled GPIO pin)
  *
  * GPIO to control Chip Enable (NRF24L01+ RF Active pin):
  * - a Chip Enable GPIO pin in push-pull mode 0-3V3
  *
  * IRQ pin to detect special cases of operation (received/transmitted):
  * - an IRQ pin with detection on falling edge (3V3 to 0V)
  *
  * For different platforms needs to be this layer implemented accordingly:
  *
  * 1) nrfport_spiTransaction(): correct Chip Select functions and SPI transaction
  * needed. This function is universal and may send/receive number of bytes (lng).
  *
  * 2) nrfport_ceHigh(), nrfport_ceLow(): correct GPIO function (or setting a
  * correct bit in GPIO register) must be assigned to fulfill  Chip Enable NRF's
  * functionality (Chip Enable represents RF activation - send or receive).
  *
  * 3) nrfport_getISQ(): correct GPIO function or reading directly GPIO register
  * to get NRF's IRQ pin state (Idle HIGH, when IRQ fires LOW).
  *
  * 4) nrfport_powerUp()/nrfport_powerDown(): functions controlling power supply
  * line of the NRF24L01+ (when implemented). Note that NRF24L01+ module has no
  * power line control so this function does not need to be called and implemented.
  * When intended to be implemented a correct mechanism e.g. usage of transistor
  * controlling the power supply line must be used.
  *
  * The advantage of powerUp/powerDown is hard reset as the NRF24L01+ may rarely
  * reach a state where does not respond and is not possible to reset via SPI.
  */

#include "nrf24l01p_defines.h"

/* Platform dependent includes - differ across different platforms */
#include "spi.h"
#include "gpio.h"


extern SPI_HandleTypeDef hspi1;

/**
  * @brief Initialization of SPI + Chip select , CE GPIOs and IRQ.
  *
  * @note This function must initialize all peripherals for SPI, CE
  * and IRQ functionality. However initialization may be handled
  * externally and thus this function may be keep mocked returning 0.
  *
  * @return 0 when initialization passed.
  */
uint8_t nrfport_initHostHW()
{
	uint8_t aux = 0;
	return aux;
}


/**
  * @brief Universal SPI write and read function including Chip Select handling.
  *
  * @param tx pointer to buffer for transmit
  * @param tx pointer to buffer for receive
  * @param tx length of data to be transmitted
  *
  * @note NRF24L01+ is sampling input data on rising edge with LOW clock in idle:
  * This SPI may be described as: SPI_POLARITY_LOW and SPI_PHASE_1EDGE.
  * During the transaction must be Chip select low from 1st to last byte of lng.
  */
void nrfport_spiTransaction(uint8_t* tx, uint8_t *rx, uint32_t lng)
{
	if(NULL == tx || NULL == rx) return;

	HAL_GPIO_WritePin(NRF_NCS_GPIO_Port, NRF_NCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, lng, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_NCS_GPIO_Port, NRF_NCS_Pin, GPIO_PIN_SET);
}


/**
  * @brief Universal SPI write a byte function.
  *
  * @param adr address of NRF24L01+ register
  * @param data content which will be in the register written
  *
  */
void nrfport_writeByte(uint8_t adr, uint8_t data)
{
	uint8_t rx[WR_BYTE_LNG] = {0};
	uint8_t tx[WR_BYTE_LNG] = {0};

	tx[WR_BYTE_CMD] = CMD_W_REGISTER | adr;
	tx[WR_BYTE_PLD] = data;

	nrfport_spiTransaction(tx, rx, WR_BYTE_LNG);
}


/**
  * @brief Universal SPI read a byte function.
  *
  * @param adr address of NRF24L01+ register
  *
  * @return data content of the register
  */
uint8_t nrfport_readByte(uint8_t adr)
{
	uint8_t rx[RD_BYTE_LNG] = {0};
	uint8_t tx[RD_BYTE_LNG] = {0};

	tx[RD_BYTE_CMD] = CMD_R_REGISTER | adr;

	nrfport_spiTransaction(tx, rx, RD_BYTE_LNG);

	return rx[RD_BYTE_PLD];
}


/**
  * @brief Set RF Active pin high.
  */
void nrfport_ceHigh(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}


/**
  * @brief Set RF Active pin low.
  */
void nrfport_ceLow(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Power up device's supply line (depends on implementation N-FET, P-FET NPN, PNP)
 */
void nrfport_powerUp(void)
{
	/* P-FET implementation */
	HAL_GPIO_WritePin(PWR_3V3_GPIO_Port, PWR_3V3_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Power down device's supply line (depends on implementation N-FET, P-FET NPN, PNP)
 */
void nrfport_powerDown(void)
{
	/* P-FET implementation */
	HAL_GPIO_WritePin(PWR_3V3_GPIO_Port, PWR_3V3_Pin, GPIO_PIN_SET);
}


/*
 * @brief Read the IRQ pin (IRQ fired when the pin is LOW).
 */
uint8_t nrfport_getIRQ(void)
{
	uint8_t irqFired = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin)) ? 1 : 0;
	return irqFired;
}
