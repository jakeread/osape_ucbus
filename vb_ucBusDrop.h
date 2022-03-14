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

#include <Arduino.h>
#include "../osape/core/vertex.h"

void vb_ucBusDrop_setup(VBus* vb, boolean useDipPick, uint8_t ID);
void vb_ucBusDrop_loop(Vertex* vt);
boolean vb_ucBusDrop_cts(VBus* vb, uint8_t rxAddr);
void vb_ucBusDrop_send(VBus* vb, uint8_t* data, uint16_t len, uint8_t rxAddr);

#endif 
#endif 