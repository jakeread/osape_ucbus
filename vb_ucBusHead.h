/*
osap/vb_ucBusHead.h

virtual port, bus head, ucbus 

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2020

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the osap project.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

#ifndef VPORT_UCBUS_HEAD_H_
#define VPORT_UCBUS_HEAD_H_ 

#include "./ucbus_config.h"

#ifdef UCBUS_IS_HEAD
#ifdef UCBUS_ON_OSAP 

#include <Arduino.h>
#include "../osape/core/vertex.h"

class VBus_UCBusHead : public VBus {
  public:
    void begin(void);
    // loop to ferry data, 
    void loop(void) override;
    // fast loop, needs to be called in ~ 10kHz ISR 
    void timerISR(void);
    // ... bus : osap API 
    void send(uint8_t* data, uint16_t len, uint8_t rxAddr) override;
    void broadcast(uint8_t* data, uint16_t len, uint8_t broadcastChannel) override;
    boolean cts(uint8_t rxAddr) override;
    boolean ctb(uint8_t broadcastChannel) override;
    boolean isOpen(uint8_t rxAddr) override;
    //
    // -------------------------------- Constructors 
    VBus_UCBusHead(Vertex* _parent, String _name);
};

#endif
#endif 
#endif 