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
#include "../osape/core/osap.h"

// badness, direct write in future 
uint8_t _tempBuffer[UB_BUFSIZE];

VBus_UCBusDrop::VBus_UCBusDrop(Vertex* _parent, String _name
): VBus(_parent, _name){

}

void VBus_UCBusDrop::begin(void){
  ucBusDrop_setup(true, 0);
  ownRxAddr = ucBusDrop_getOwnID();
}

void VBus_UCBusDrop::begin(uint8_t _ownRxAddr){
  ucBusDrop_setup(false, _ownRxAddr);
  ownRxAddr = ucBusDrop_getOwnID();
}

void VBus_UCBusDrop::loop(void){
  // will want to shift(?) from ucbus inbuffer to vertex origin stack 
  if(ucBusDrop_ctrB()){
    // find a slot, 
    if(stackEmptySlot(this, VT_STACK_ORIGIN)){
      // copy in to origin stack 
      uint16_t len = ucBusDrop_readB(_tempBuffer);
      stackLoadSlot(this, VT_STACK_ORIGIN, _tempBuffer, len);
    } else {
      // no empty space, will wait in bus 
    }
  }
}

void VBus_UCBusDrop::send(uint8_t* data, uint16_t len, uint8_t rxAddr){
  // can't tx not-to-the-head, will drop pck 
  if(rxAddr != 0) return;
  // if the bus is ready, drop it,
  if(ucBusDrop_ctsB()){
    ucBusDrop_transmitB(data, len);
  } else {
    OSAP::error("ubd tx while not clear", MEDIUM);
  }
}

boolean VBus_UCBusDrop::cts(uint8_t rxAddr){
  // immediately clear? & transmit only to head 
  return (rxAddr == 0 && ucBusDrop_ctsB());
}

void VBus_UCBusDrop::broadcast(uint8_t* data, uint16_t len, uint8_t broadcastChannel){
  OSAP::debug("Broadcast is unwritten");
}

boolean VBus_UCBusDrop::ctb(uint8_t broadcastChannel){
  OSAP::debug("CTB is unwritten");
  return false;
}

boolean VBus_UCBusDrop::isOpen(uint8_t rxAddr){
  return ucBusDrop_isPresent(rxAddr);
}


#endif 
#endif 