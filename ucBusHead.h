/*
osap/drivers/ucBusHead.h

beginnings of a uart-based clock / bus combo protocol

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2019

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/

#ifndef UCBUS_HEAD_H_
#define UCBUS_HEAD_H_

#include "./ucbus_config.h"

#ifdef UCBUS_IS_HEAD

#include <Arduino.h>
#include "ucBusMacros.h"

// setup, 
void ucBusHead_setup(void);

// need to call the main timer isr at some rate, 
void ucBusHead_timerISR(void);
void ucBusHead_rxISR(void);
void ucBusHead_txISR(void);

// ub interface, 
boolean ucBusHead_ctr(uint8_t drop); // is there ahn packet to read at this drop 
size_t ucBusHead_read(uint8_t drop, uint8_t *dest);  // get 'them bytes fam 
//size_t ucBusHead_readPtr(uint8_t* drop, uint8_t** dest, unsigned long *pat); // vport interface, get next to handle... 
//void ucBusHead_clearPtr(uint8_t drop);
boolean ucBusHead_ctsA(void);  // return true if TX complete / buffer ready
boolean ucBusHead_ctsB(uint8_t drop);
boolean ucBusHead_isPresent(uint8_t drop); // have we heard from this drop recently ? 
void ucBusHead_transmitA(uint8_t *data, uint16_t len, uint8_t channel);  // ship bytes: broadcast to all 
void ucBusHead_transmitB(uint8_t *data, uint16_t len, uint8_t drop);  // ship bytes: 0-14: individual drop, 15: broadcast

#endif
#endif 