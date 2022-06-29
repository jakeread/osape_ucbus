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

void vb_ucBusHead_setup(void) {
  // start ucbus
  ucBusHead_setup();  // todo: rewrite as c object, not class
}

void vb_ucBusHead_loop(Vertex* vt) {
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
      uint8_t slot = 0;
      if (stackEmptySlot(vt, VT_STACK_ORIGIN)) {
        // copy it in, 
        uint16_t len = ucBusHead_read(drop, _tempBuffer);
        stackLoadSlot(vt, VT_STACK_ORIGIN, _tempBuffer, len);
      } else {
        // no more empty spaces this turn, continue 
        return; 
      }
    }
  }
}

boolean vb_ucBusHead_cts(VBus* vb, uint8_t rxAddr) {
  // mapping rxAddr in osap space (where 0 is head) to ucbus drop-id space...
  return ucBusHead_ctsB(rxAddr);
}

void vb_ucBusHead_send(VBus* vb, uint8_t* data, uint16_t len, uint8_t rxAddr) {
  if (rxAddr == 0) {
    OSAP::error("attempt to busf from head to self", MEDIUM);
  } else {
    ucBusHead_transmitB(data, len, rxAddr);
  }
}

#endif 
#endif