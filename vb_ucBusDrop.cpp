/*
osap/vport_ucbus_drop.cpp

virtual port, bus drop, ucbus 

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2020

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the osap project.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

#include "vb_ucBusDrop.h"

#ifdef UCBUS_IS_DROP
#ifdef UCBUS_ON_OSAP 

#include "ucBusDrop.h"

// badness, direct write in future 
uint8_t _tempBuffer[1024];

void vb_ucBusDrop_setup(VBus* vb, boolean useDipPick, uint8_t ID){
    // start it: use DIP 
    ucBusDrop_setup(useDipPick, ID);
    vb->ownRxAddr = ucBusDrop_getOwnID();
}

void vb_ucBusDrop_loop(Vertex* vt){
  // will want to shift(?) from ucbus inbuffer to vertex origin stack 
  if(ucBusDrop_ctrB()){
    // find a slot, 
    uint8_t slot = 0;
    if(stackEmptySlot(vt, VT_STACK_ORIGIN)){
      // copy in to origin stack 
      uint16_t len = ucBusDrop_readB(_tempBuffer);
      stackLoadSlot(vt, VT_STACK_ORIGIN, _tempBuffer, len);
    } else {
      // no empty space, will wait in bus 
    }
  }
}

boolean vb_ucBusDrop_cts(VBus* vb, uint8_t rxAddr){
  // immediately clear? & transmit only to head 
  if(rxAddr == 0 && ucBusDrop_ctsB()){
    return true;
  } else {
    return false;
  }
}

void vb_ucBusDrop_send(VBus* vb, uint8_t* data, uint16_t len, uint8_t rxAddr){
  // can't tx not-to-the-head, will drop pck 
  if(rxAddr != 0) return;
  // if the bus is ready, drop it,
  if(ucBusDrop_ctsB()){
    ucBusDrop_transmitB(data, len);
  } else {
    OSAP::error(2, "ubd tx while not clear");
  }
}

#endif 
#endif 