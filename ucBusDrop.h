/*
osap/drivers/ucBusDrop.h

beginnings of a uart-based clock / bus combo protocol

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2019

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/

#ifndef UCBUS_DROP_H_
#define UCBUS_DROP_H_

#include "./ucbus_config.h"

#ifdef UCBUS_IS_DROP

#include <Arduino.h>
#include "ucBusMacros.h"

// setup 
void ucBusDrop_setup(boolean useDipPick, uint8_t ID);
uint16_t ucBusDrop_getOwnID(void);

// isrs 
void ucBusDrop_rxISR(void);
void ucBusDrop_dreISR(void);
void ucBusDrop_txcISR(void);

// handlers (define in main.cpp, these are application interfaces)
void ucBusDrop_onRxISR(void);
void ucBusDrop_onPacketARx(uint8_t* inBufferA, volatile uint16_t len);

// the api, eh 
boolean ucBusDrop_ctrB(void);
size_t ucBusDrop_readB(uint8_t* dest);
boolean ucBusDrop_ctrA(void);
size_t ucBusDrop_readA(uint8_t* dest);

// drop cannot tx to channel A
boolean ucBusDrop_ctsB(void); // true if tx buffer empty, 
boolean ucBusDrop_isPresent(uint8_t rxAddr);
void ucBusDrop_transmitB(uint8_t *data, uint16_t len);


#endif
#endif 