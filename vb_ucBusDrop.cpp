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
  addrSpaceSize = UCBUS_MAX_DROPS;
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
  // can we shift-in from channel a / broadcast messages ?
  // also... stack 'em from the broadcast channel first, typically higher priority 
  if(ucBusDrop_ctrA()){
    // and if we have an empty space... 
    if(stackEmptySlot(this, VT_STACK_ORIGIN)){
    // get len & strip out the broadcastChannel, which was stuffed at [0]
    uint16_t len = ucBusDrop_readA(_tempBuffer);
    injestBroadcastPacket(&(_tempBuffer[1]), len - 1, _tempBuffer[0]);
    }
  }
  // can we shift-in from channel b / directed messages ? 
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
  OSAP::debug("Bus Drop CTB is unwritten");
  return false;
}

boolean VBus_UCBusDrop::isOpen(uint8_t rxAddr){
  return ucBusDrop_isPresent(rxAddr);
}

#endif 
#endif 