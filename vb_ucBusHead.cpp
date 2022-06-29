/*
osap/vb_ucBusHead.cpp

virtual port, bus head / host

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2019

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the osap project.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

#include "vb_ucBusHead.h"

#ifdef UCBUS_IS_HEAD
#ifdef UCBUS_ON_OSAP 

#include "ucBusHead.h"
#include "../osape/core/osap.h"

// locally, track which drop we shifted in a packet from last
uint8_t _lastDropHandled = 0;

// badness, should remove w/ direct copy in API eventually
uint8_t _tempBuffer[1024];

VBus_UCBusHead::VBus_UCBusHead(Vertex* _parent, String _name
): VBus (_parent, _name) {
  // no other ops, believe it or not, 
}

void VBus_UCBusHead::begin(void){
  // start ucbus
  ucBusHead_setup(); 
}

void VBus_UCBusHead::loop(void){
  // we need to shift items from the bus into the origin stack here
  // we can shift multiple in per turn, if stack space exists
  uint8_t drop = _lastDropHandled;
  for (uint8_t i = 1; i < UB_MAX_DROPS; i++) {
    drop++;
    if (drop >= UB_MAX_DROPS) {
      drop = 1;
    }
    if (ucBusHead_ctr(drop)) {
      // find a stack slot,
      if (stackEmptySlot(this, VT_STACK_ORIGIN)) {
        // copy it in, 
        uint16_t len = ucBusHead_read(drop, _tempBuffer);
        stackLoadSlot(this, VT_STACK_ORIGIN, _tempBuffer, len);
      } else {
        // no more empty spaces this turn, continue 
        return; 
      }
    }
  }
}

void VBus_UCBusHead::timerISR(void){
  ucBusHead_timerISR();
}

void VBus_UCBusHead::send(uint8_t* data, uint16_t len, uint8_t rxAddr) {
  if (rxAddr == 0) {
    OSAP::error("attempt to busf from head to self", MEDIUM);
  } else {  
    ucBusHead_transmitB(data, len, rxAddr);
  }
}

boolean VBus_UCBusHead::cts(uint8_t rxAddr){
  // mapping rxAddr in osap space (where 0 is head) to ucbus drop-id space...
  return ucBusHead_ctsB(rxAddr);
}

void VBus_UCBusHead::broadcast(uint8_t* data, uint16_t len, uint8_t broadcastChannel){
  OSAP::debug("UNWRITTEN");
}

boolean VBus_UCBusHead::ctb(uint8_t broadcastChannel){
  OSAP::debug("UNWRITTEN");
  return false;
}

boolean VBus_UCBusHead::isOpen(uint8_t rxAddr){
  return false;
}

#endif 
#endif