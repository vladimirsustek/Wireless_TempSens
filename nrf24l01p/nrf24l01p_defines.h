/*
 * nrf24l01p_defines.h
 *
 *  Created on: Nov 1, 2022
 *      Author: 42077
 */

#ifndef INC_NRF24L01P_DEFINES_H_
#define INC_NRF24L01P_DEFINES_H_

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#define MAX_DEVICE_POWER_CYCLE_ATTEMPTS (uint32_t)(3)

#define WR_BYTE_LNG		 (uint32_t)(2)
#define WR_BYTE_PLD		 (uint32_t)(1)
#define WR_BYTE_CMD		 (uint32_t)(0)

#define RD_BYTE_LNG		 (uint32_t)(2)
#define RD_BYTE_PLD		 (uint32_t)(1)
#define RD_BYTE_CMD		 (uint32_t)(0)

#define NRF_MAX_ADR_LNG  (uint32_t)(5)
#define NRF_MIN_ADR_LNG  (uint32_t)(3)

#define NRF_ADR_MAX	     (uint32_t)(5)
#define NRF_ADR_MIN	 	 (uint32_t)(3)
#define RD_RX_ADR_P0_LNG (uint32_t)(6)

#define WR_RX_ADR_P0_LNG (uint32_t)(6)

#define PAYLOAD_MIN		 (uint32_t)(0)
#define PAYLOAD_MAX		 (uint32_t)(32)

// Addresses
#define CONFIG_ADR      (uint8_t)(0x00)
#define EN_AA_ADR       (uint8_t)(0x01)
#define EN_RXADDR_ADR   (uint8_t)(0x02)
#define SETUP_AW_ADR    (uint8_t)(0x03)
#define SETUP_RETR_ADR  (uint8_t)(0x04)
#define RF_CH_ADR       (uint8_t)(0x05)
#define RF_SETUP_ADR    (uint8_t)(0x06)
#define STATUS_ADR      (uint8_t)(0x07)
#define OBSERVE_TX_ADR  (uint8_t)(0x08)
#define RPD_ADR         (uint8_t)(0x09)
#define RX_ADR_P0_ADR   (uint8_t)(0x0A)
#define RX_ADR_P1_ADR   (uint8_t)(0x0B)
#define RX_ADR_P2_ADR   (uint8_t)(0x0C)
#define RX_ADR_P3_ADR   (uint8_t)(0x0D)
#define RX_ADR_P4_ADR   (uint8_t)(0x0E)
#define RX_ADR_P5_ADR   (uint8_t)(0x0F)
#define TX_ADR_ADR      (uint8_t)(0x10)
#define RX_PW_P0_ADR    (uint8_t)(0x11)
#define RX_PW_P1_ADR    (uint8_t)(0x12)
#define RX_PW_P2_ADR    (uint8_t)(0x13)
#define RX_PW_P3_ADR    (uint8_t)(0x14)
#define RX_PW_P4_ADR    (uint8_t)(0x15)
#define RX_PW_P5_ADR    (uint8_t)(0x16)
#define FIFO_STATUS_ADR (uint8_t)(0x17)
#define DYNPD_ADR       (uint8_t)(0x1C)
#define FEATURE_ADR     (uint8_t)(0x1D)


/* CONFIG */
#define CONFIG_RESERVED (uint8_t)(7)
#define MASK_RX_DR      (uint8_t)(6)
#define MASK_TX_DS      (uint8_t)(5)
#define MASK_MAX_RT     (uint8_t)(4)
#define EN_CRC          (uint8_t)(3)
#define CRCO            (uint8_t)(2)
#define PWR_UP          (uint8_t)(1)
#define PRIM_RX         (uint8_t)(0)


/* EN_AA */
#define EN_AA_RESERVED_7  (uint8_t)(7)
#define EN_AA_RESERVED_6  (uint8_t)(6)
#define ENAA_P5           (uint8_t)(5)
#define ENAA_P4           (uint8_t)(4)
#define ENAA_P3           (uint8_t)(3)
#define ENAA_P2           (uint8_t)(2)
#define ENAA_P1           (uint8_t)(1)
#define ENAA_P0           (uint8_t)(0)


/* EN_RXADDR */
#define EN_RXADDR_RESERVERD_1     (uint8_t)(7)
#define EN_RXADDR_RESERVERD_0     (uint8_t)(6)
#define ERX_P5                    (uint8_t)(5)
#define ERX_P4                    (uint8_t)(4)
#define ERX_P3                    (uint8_t)(3)
#define ERX_P2                    (uint8_t)(2)
#define ERX_P1                    (uint8_t)(1)
#define ERX_P0                    (uint8_t)(0)


/* SETUP_AW */
#define SETUP_AW_RESERVED_5  (uint8_t)(7)
#define SETUP_AW_RESERVED_4  (uint8_t)(6)
#define SETUP_AW_RESERVED_3  (uint8_t)(5)
#define SETUP_AW_RESERVED_2  (uint8_t)(4)
#define SETUP_AW_RESERVED_1  (uint8_t)(3)
#define SETUP_AW_RESERVED_0  (uint8_t)(2)
#define AW_1                 (uint8_t)(1)
#define AW_0                 (uint8_t)(0)


/* SEETUP_RETR */
#define ARD_3  (uint8_t)(7)
#define ARD_2  (uint8_t)(6)
#define ARD_1  (uint8_t)(5)
#define ARD_0  (uint8_t)(4)
#define ARC_3  (uint8_t)(3)
#define ARC_2  (uint8_t)(2)
#define ARC_1  (uint8_t)(1)
#define ARC_0  (uint8_t)(0)


/* RF_CH */
#define RF_CH_RESERVED    (uint8_t)(7)
#define RF_CH_6           (uint8_t)(6)
#define RF_CH_5           (uint8_t)(5)
#define RF_CH_4           (uint8_t)(4)
#define RF_CH_3           (uint8_t)(3)
#define RF_CH_2           (uint8_t)(2)
#define RF_CH_1           (uint8_t)(1)
#define RF_CH_0           (uint8_t)(0)


/* RF_SETUP */
#define CONST_WAVE        (uint8_t)(7)
#define RF_SETUP_RESERVED (uint8_t)(6)
#define RF_DR_LOW         (uint8_t)(5)
#define PLL_LOCK          (uint8_t)(4)
#define RF_DR_HIGH        (uint8_t)(3)
#define RF_PWR_1          (uint8_t)(2)
#define RF_PWR_0          (uint8_t)(1)
#define OBSOLETE          (uint8_t)(0)


/* STATUS */
#define STATUS_RESERVED   (uint8_t)(7)
#define RX_DR             (uint8_t)(6)
#define TX_DS             (uint8_t)(5)
#define MAX_RT            (uint8_t)(4)
#define RX_P_NO_2         (uint8_t)(3)
#define RX_P_NO_1         (uint8_t)(2)
#define RX_P_NO_0         (uint8_t)(1)
#define TX_FULL           (uint8_t)(0)


/* OBSERVE_TX */
#define PLOS_CNT_3        (uint8_t)(7)
#define PLOS_CNT_2        (uint8_t)(6)
#define PLOS_CNT_1        (uint8_t)(5)
#define PLOS_CNT_0        (uint8_t)(4)
#define ARC_CNT_3         (uint8_t)(3)
#define ARC_CNT_2         (uint8_t)(2)
#define ARC_CNT_1         (uint8_t)(1)
#define ARC_CNT_0         (uint8_t)(0)


/* RPD */
#define RPD_RESERVED_6      (uint8_t)(7)
#define RPD_RESERVED_5      (uint8_t)(6)
#define RPD_RESERVED_4      (uint8_t)(5)
#define RPD_RESERVED_3      (uint8_t)(4)
#define RPD_RESERVED_2      (uint8_t)(3)
#define RPD_RESERVED_1      (uint8_t)(2)
#define RPD_RESERVED_0      (uint8_t)(1)
#define RPD                 (uint8_t)(0)


/* FIFO_STATUS */
#define FIFO_STATUS_RESERVED   (uint8_t)(7)
#define TX_REUSE               (uint8_t)(6)
#define FIFO_TX_FULL           (uint8_t)(5)
#define TX_EMPTY               (uint8_t)(4)
#define FIFO_STATUS_RESERVED_1 (uint8_t)(3)
#define FIFO_STATUS_RESERVED_0 (uint8_t)(2)
#define RX_FULL                (uint8_t)(1)
#define RX_EMPTY               (uint8_t)(0)


/* DYNPD */
#define DYNDP_RESERVED_1       (uint8_t)(7)
#define DYNDP_RESERVED_0       (uint8_t)(6)
#define DPL_P5                 (uint8_t)(5)
#define DPL_P4                 (uint8_t)(4)
#define DPL_P3                 (uint8_t)(3)
#define DPL_P2                 (uint8_t)(2)
#define DPL_P1                 (uint8_t)(1)
#define DPL_P0                 (uint8_t)(0)


/* FEATURE */
#define FEATURE_RESERVED_4     (uint8_t)(7)
#define FEATURE_RESERVED_3     (uint8_t)(6)
#define FEATURE_RESERVED_2     (uint8_t)(5)
#define FEATURE_RESERVED_1     (uint8_t)(4)
#define FEATURE_RESERVED_0     (uint8_t)(3)
#define EN_DPL                 (uint8_t)(2)
#define EN_ACK_PAY             (uint8_t)(1)
#define EN_DYN_ACK             (uint8_t)(0)

// Commands
#define CMD_R_REGISTER			(uint8_t)(0b00000000)
#define CMD_W_REGISTER			(uint8_t)(0b00100000)
#define CMD_R_RX_PAYLOAD		(uint8_t)(0b01100001)
#define CMD_W_TX_PAYLOAD		(uint8_t)(0b10100000)

#define CMD_FLUSH_TX			(uint8_t)(0b11100001)
#define CMD_FLUSH_RX			(uint8_t)(0b11100010)
#define CMD_REUSE_TX_PL			(uint8_t)(0b11100011)
#define CMD_R_RX_PL_WID			(uint8_t)(0b01100000)
#define CMD_W_ACK_PAYLOAD		(uint8_t)(0b10101000)
#define CMD_W_TX_PAYLOAD_NO_ACK (uint8_t)(0b10110000)
#define CMD_NOP					(uint8_t)(0b11111111)

typedef struct NRF_DEV
{
	 void (*switchSPIcontext)(uint8_t);
	 void (*CEactivate)(void);
	 void (*CEdeactivate)(void);

	 uint8_t (*getCONFIG)(void);
	 uint8_t (*getEN_AA)(void);
	 uint8_t (*getEN_RXADDR)(void);
	 uint8_t (*getSETUP_AW)(void);
	 uint8_t (*getSETUP_RETR)(void);
	 uint8_t (*getRF_CH)(void);
	 uint8_t (*getRF_SETUP)(void);
	 uint8_t (*getSTATUS)(void);
	 uint8_t (*getOBSERVE_TX)(void);
	 uint8_t (*getRPD)(void);
	 void (*getRX_ADDR_P0)(uint8_t*, uint32_t lng);
	 void (*getRX_ADDR_P1)(uint8_t*, uint32_t lng);
	 uint8_t (*getRX_ADDR_P2)(void);
	 uint8_t (*getRX_ADDR_P3)(void);
	 uint8_t (*getRX_ADDR_P4)(void);
	 uint8_t (*getRX_ADDR_P5)(void);
	 void (*getTX_ADDR)(uint8_t*, uint32_t);
	 uint8_t (*getRX_PW_P0)(void);
	 uint8_t (*getRX_PW_P1)(void);
	 uint8_t (*getRX_PW_P2)(void);
	 uint8_t (*getRX_PW_P3)(void);
	 uint8_t (*getRX_PW_P4)(void);
	 uint8_t (*getRX_PW_P5)(void);
	 uint8_t (*getFIFO_STATUS)(void);
	 uint8_t (*getDYNPD)(void);
	 uint8_t (*getFEATURE)(void);

	 void (*setCONFIG)(uint8_t);
	 void (*setEN_AA)(uint8_t);
	 void (*setEN_RXADDR)(uint8_t);
	 void (*setSETUP_AW)(uint8_t);
	 void (*setSETUP_RETR)(uint8_t);
	 void (*setRF_CH)(uint8_t);
	 void (*setRF_SETUP)(uint8_t);
	 void (*setSTATUS)(uint8_t);
	 void (*setOBSERVE_TX)(uint8_t);
	 void (*setRPD)(uint8_t);
	 void (*setRX_ADDR_P0)(uint8_t*, uint32_t);
	 void (*setRX_ADDR_P1)(uint8_t*, uint32_t);
	 void (*setRX_ADDR_P2)(uint8_t);
	 void (*setRX_ADDR_P3)(uint8_t);
	 void (*setRX_ADDR_P4)(uint8_t);
	 void (*setRX_ADDR_P5)(uint8_t);
	 void (*setTX_ADDR)(uint8_t*, uint32_t);
	 void (*setFIFO_STATUS)(uint8_t);
	 void (*setDYNPD)(uint8_t);
	 void (*setFEATURE)(uint8_t);

	 void (*getR_RX_PAYLOAD)(uint8_t*, uint32_t);
	 void (*setW_TX_PAYLOAD)(uint8_t*, uint32_t);
	 void (*set_FLUSH_TX)(void);
	 void (*set_FLUSH_RX)(void);
	 void (*set_REUSE_TX_PL)(void);
	 uint8_t (*get_R_RX_PL_WID)(void);
	 void (*set_W_ACK_PAYLOAD)(uint8_t*, uint32_t);
	 void (*set_W_TX_PAYLOAD_NO_ACK)(void);
	 void (*get_NOP)(void);
}NRF_DEV_s;


#endif /* INC_NRF24L01P_DEFINES_H_ */
