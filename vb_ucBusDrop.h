/*
osap/vport_ucbus_drop.h

virtual port, bus drop, ucbus 

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2020

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the osap project.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

#ifndef VBUS_UCBUS_HEAD_H_
#define VBUS_UCBUS_HEAD_H_

#include "./ucbus_config.h"

#ifdef UCBUS_IS_DROP
#ifdef UCBUS_ON_OSAP 

#include <Arduino.h>
#include "../osape/core/vertex.h"

class VBus_UCBusDrop : public VBus {
  public:
    void begin(void);
    void begin(uint8_t _ownRxAddr);
    void loop(void) override;
    void send(uint8_t* data, uint16_t len, uint8_t rxAddr) override;
    void broadcast(uint8_t* data, uint16_t len, uint8_t broadcastChannel) override;
    boolean cts(uint8_t rxAddr) override;
    boolean ctb(uint8_t broadcastChannel) override;
    boolean isOpen(uint8_t rxAddr);
    VBus_UCBusDrop(Vertex* _parent, String _name);
};

#endif 
#endif 
#endif 