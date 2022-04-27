/*
ucBusMacros.h

config / utes for the uart-clocked bus 

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2021

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/


#ifndef UCBUS_MACROS_H_
#define UCBUS_MACROS_H_

#include "./ucbus_config.h"
#include <Arduino.h>

// ---------------------------------------------- INFO 

/*
    assuming for now there is one bus PHY per micro, 
    this is for shared hardware config *and* macros to operate 
    / read / write on the bus 
*/

// ---------------------------------------------- BUFFER / DROP SIZES / RATES
// the channel count: 2
#define UB_CH_COUNT 2 
// the size of each buffer: also the maximum segment size 
#define UB_BUFSIZE 256
// max. # of drops on the bus, just swapping from top level config.h 
#define UB_MAX_DROPS UCBUS_MAX_DROPS
// with a fixed 2-byte header, we can have some max # of data bytes, 
// this is *probably* going to stay at 10, but might fluxuate a little 
#define UB_DATA_BYTES_PER_WORD 12
#define UB_HEAD_BYTES_PER_WORD (UB_DATA_BYTES_PER_WORD + 3)     // + 2 header, + 1 rare character
#define UB_DROP_BYTES_PER_WORD (UB_DATA_BYTES_PER_WORD + 2)     // + 2 header

// ---------------------------------------------- DATA WORDS -> INFO 

typedef union {
    struct {
        uint8_t CH0FC:1;    // bit: channel 0 reported flowcontrol (1: full, 0: cts)
        uint8_t CH1FC:1;    // bit: channel 1 reported flowcontrol 
        uint8_t DROPTAP:6;  // 0-63: time division drop 
        uint8_t CHSELECT:1; // bit: channel select: 1 for ch1, 0 ch0
        uint8_t RESERVED:3; // not currently used, 
        uint8_t TOKENS:4;   // 0-15: how many bytes in word are real data bytes 
    } bits;
    uint8_t bytes[2];
} UCBUS_HEADER_Type;

#define UCBUS_RARECHAR 0b10101010

// ---------------------------------------------- PORT / PIN CONFIGS 
#ifdef UCBUS_IS_D51
// ------------------------------------ D51 HAL
#define UB_SER_USART SERCOM1->USART
#define UB_SERCOM_CLK SERCOM1_GCLK_ID_CORE
#define UB_GCLKNUM_PICK 7
#define UB_COMPORT PORT->Group[0]
#define UB_TXPIN 16  // x-0
#define UB_TXBM (uint32_t)(1 << UB_TXPIN)
#define UB_RXPIN 18  // x-2
#define UB_RXBM (uint32_t)(1 << UB_RXPIN)
#define UB_RXPO 2 // RX on SER-2
#define UB_TXPERIPHERAL 2 // A: 0, B: 1, C: 2
#define UB_RXPERIPHERAL 2

// the data enable / reciever enable pins were modified between module circuit 
// revisions: the board w/ an SMT JTAG header is "the OG" module, 
// these are from board-level config
#ifdef IS_OG_MODULE 
#define UB_DE_PIN 16 // driver output enable: set HI to enable, LO to tri-state the driver 
#define UB_DE_PORT PORT->Group[1] 
#define UB_RE_PIN 19 // receiver output enable, set LO to enable the RO, set HI to tri-state RO 
#define UB_RE_PORT PORT->Group[0]
#else 
#define UB_DE_PIN 19 // driver output enable: set HI to enable, LO to tri-state the driver 
#define UB_DE_PORT PORT->Group[0] 
#define UB_RE_PIN 9 // receiver output enable, set LO to enable the RO, set HI to tri-state RO 
#define UB_RE_PORT PORT->Group[1]
#endif 

#define UB_TE_PIN 17  // termination enable, drive LO to enable to internal termination resistor, HI to disable
#define UB_TE_PORT PORT->Group[0]
#define UB_TE_BM (uint32_t)(1 << UB_TE_PIN)
#define UB_RE_BM (uint32_t)(1 << UB_RE_PIN)
#define UB_DE_BM (uint32_t)(1 << UB_DE_PIN)

#define UB_DRIVER_ENABLE UB_DE_PORT.OUTSET.reg = UB_DE_BM
#define UB_DRIVER_DISABLE UB_DE_PORT.OUTCLR.reg = UB_DE_BM
// ------------------------------------ END D51 HAL 
#endif 

#ifdef UCBUS_IS_D21
// ------------------------------------ D21 HAL 
#define UB_SER_USART SERCOM1->USART 
#define UB_PORT PORT->Group[0]
#define UB_TXPIN 16
#define UB_TXBM (uint32_t)(1 << UB_TXPIN)
#define UB_RXPIN 19
#define UB_RXBM (uint32_t)(1 << UB_RXPIN)
#define UB_RXPO 3 // RX is on SER1-3
#define UB_TXPERIPHERAL PERIPHERAL_C
#define UB_RXPERIPHERAL PERIPHERAL_C
// data enable, recieve enable pins 
#define UB_DEPIN 17
#define UB_DEBM (uint32_t)(1 << UB_DEPIN)
#define UB_REPIN 18
#define UB_REBM (uint32_t)(1 << UB_REPIN)
#define UB_DRIVER_ENABLE UB_PORT.OUTSET.reg = UB_DEBM
#define UB_DRIVER_DISABLE UB_PORT.OUTCLR.reg = UB_DEBM
#define UB_DE_SETUP UB_PORT.DIRSET.reg = UB_DEBM; UB_DRIVER_DISABLE
#define UB_RECIEVE_ENABLE UB_PORT.OUTCLR.reg = UB_REBM
#define UB_RECIEVE_DISABLE UB_PORT.OUTSET.reg = UB_REBM
#define UB_RE_SETUP UB_PORT.DIRSET.reg = UB_REBM; UB_RECIEVE_ENABLE
// ------------------------------------ END D21 HAL 
#endif 

#endif 