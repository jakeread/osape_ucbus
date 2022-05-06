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

void vb_ucBusHead_setup(void);
void vb_ucBusHead_loop(Vertex* vt);
boolean vb_ucBusHead_cts(VBus* vb, uint8_t rxAddr);
void vb_ucBusHead_send(VBus* vb, uint8_t* data, uint16_t len, uint8_t rxAddr);

#endif 
#endif
#endif 