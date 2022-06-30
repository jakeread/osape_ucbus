/*
osap/drivers/ucBusDrop.cpp

beginnings of a uart-based clock / bus combo protocol

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2019

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/

#include "ucBusDrop.h"

#ifdef UCBUS_IS_DROP

#include "ucBusDipConfig.h"
#include "../indicators.h"
#include "../osape/core/osap.h"

// recieve buffers
uint8_t recieveBuffer[UB_CH_COUNT][UB_BUFSIZE];
volatile uint16_t recieveBufferWp[UB_CH_COUNT];
// tracking did-last-msg have token,
volatile boolean lastWordHadToken[UB_CH_COUNT];

// stash buffers (have to ferry data from rx buffer -> here immediately on rx, else next word can overwrite)
uint8_t inBuffer[UB_CH_COUNT][UB_BUFSIZE];
volatile uint16_t inBufferLen[UB_CH_COUNT];

// output buffer 
uint8_t outBuffer[UB_CH_COUNT][UB_BUFSIZE];
volatile uint16_t outBufferRp[UB_CH_COUNT];
volatile uint16_t outBufferLen[UB_CH_COUNT];

// receive word
UCBUS_HEADER_Type inHeader = { .bytes = { 0,0 } };
volatile uint8_t inWordWp = 0;
uint8_t inWord[UB_HEAD_BYTES_PER_WORD];

// outgoing word 
UCBUS_HEADER_Type outHeader = { .bytes = { 0,0 } };
uint8_t outWord[UB_DROP_BYTES_PER_WORD];
volatile uint8_t outWordRp = 0;

// reciprocal buffer space, for flowcontrol 
volatile uint8_t rcrxb[UB_CH_COUNT];

// our physical bus address, 
volatile uint8_t id = 0;

// available time count, in bus tick units 
volatile uint16_t timeTick = 0;
volatile uint64_t timeBlink = 0;
uint16_t blinkTime = 1000;

// baudrate 
uint32_t ub_baud_val = 0;

// we need to track interrupt states as well as setting the flags in the micro, 
// since the D21 fires only one ISR for all of the flags;
volatile boolean txcISR = false;
volatile boolean dreISR = false;

#define DRE_ISR_ON UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE; dreISR = true
#define DRE_ISR_OFF UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE; dreISR = false 
#define TXC_ISR_ON UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC; txcISR = true 
#define TXC_ISR_OFF UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC; txcISR = false 

#ifdef UCBUS_IS_D51 
// ------------------------------------ D51 SPECIFIC 
// hardware init (file scoped)
void setupBusDropUART(void){
  // set driver output LO to start: tri-state 
  UB_DE_PORT.DIRSET.reg = UB_DE_BM;
  UB_DRIVER_DISABLE;
  // set receiver output on, forever: LO to set on 
  UB_RE_PORT.DIRSET.reg = UB_RE_BM;
  UB_RE_PORT.OUTCLR.reg = UB_RE_BM;
  // termination resistor should be set only on one drop, 
  // or none and physically with a 'tail' cable, or something? 
  UB_TE_PORT.DIRSET.reg = UB_TE_BM;
  if(dip_readPin1()){
    UB_TE_PORT.OUTCLR.reg = UB_TE_BM;
  } else {
    UB_TE_PORT.OUTSET.reg = UB_TE_BM;
  }
  // rx pin setup
  UB_COMPORT.DIRCLR.reg = UB_RXBM;
  UB_COMPORT.PINCFG[UB_RXPIN].bit.PMUXEN = 1;
  if(UB_RXPIN % 2){
    UB_COMPORT.PMUX[UB_RXPIN >> 1].reg |= PORT_PMUX_PMUXO(UB_RXPERIPHERAL);
  } else {
    UB_COMPORT.PMUX[UB_RXPIN >> 1].reg |= PORT_PMUX_PMUXE(UB_RXPERIPHERAL);
  }
  // tx
  UB_COMPORT.DIRCLR.reg = UB_TXBM;
  UB_COMPORT.PINCFG[UB_TXPIN].bit.PMUXEN = 1;
  if(UB_TXPIN % 2){
    UB_COMPORT.PMUX[UB_TXPIN >> 1].reg |= PORT_PMUX_PMUXO(UB_TXPERIPHERAL);
  } else {
    UB_COMPORT.PMUX[UB_TXPIN >> 1].reg |= PORT_PMUX_PMUXE(UB_TXPERIPHERAL);
  }
  // ok, clocks, first line au manuel
  	// unmask clocks 
	MCLK->APBAMASK.bit.SERCOM1_ = 1;
  GCLK->GENCTRL[UB_GCLKNUM_PICK].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) | GCLK_GENCTRL_GENEN;
  while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(UB_GCLKNUM_PICK));
	GCLK->PCHCTRL[UB_SERCOM_CLK].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(UB_GCLKNUM_PICK);
  // then, sercom
  while(UB_SER_USART.SYNCBUSY.bit.ENABLE);
  UB_SER_USART.CTRLA.bit.ENABLE = 0;
  while(UB_SER_USART.SYNCBUSY.bit.SWRST);
  UB_SER_USART.CTRLA.bit.SWRST = 1;
  while(UB_SER_USART.SYNCBUSY.bit.SWRST);
  while(UB_SER_USART.SYNCBUSY.bit.SWRST || UB_SER_USART.SYNCBUSY.bit.ENABLE);
  // ctrla 
  UB_SER_USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD;
  UB_SER_USART.CTRLA.reg |= SERCOM_USART_CTRLA_RXPO(UB_RXPO) | SERCOM_USART_CTRLA_TXPO(0);
  //UB_SER_USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(1); // enable even parity 
  // ctrlb 
  while(UB_SER_USART.SYNCBUSY.bit.CTRLB);
  UB_SER_USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	// enable interrupts 
	NVIC_EnableIRQ(SERCOM1_2_IRQn); // rx interrupts 
  NVIC_EnableIRQ(SERCOM1_1_IRQn); // transmit complete interrupt 
	NVIC_EnableIRQ(SERCOM1_0_IRQn); // data register empty interrupts 
	// set baud 
  UB_SER_USART.BAUD.reg = ub_baud_val;
  // and finally, a kickoff
  while(UB_SER_USART.SYNCBUSY.bit.ENABLE);
  UB_SER_USART.CTRLA.bit.ENABLE = 1;
  // enable rx interrupt, disable dre, txc 
  UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE | SERCOM_USART_INTENCLR_TXC;
  // to enable tx complete, 
  //UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC; // now watch transmit complete
}

// DRE handler 
void SERCOM1_0_Handler(void){
  ucBusDrop_dreISR();
}

// TXC handler 
void SERCOM1_1_Handler(void){
  ucBusDrop_txcISR();
}

void SERCOM1_2_Handler(void){
	ucBusDrop_rxISR();
}
// ------------------------------------ END D51 SPECIFIC 
#endif 

#ifdef UCBUS_IS_D21 
// ------------------------------------ D21 SPECIFIC 
void setupBusDropUART(void){
  // ------------------------------------------ USART PIN CONFIG
  // setup pins as output or inputs,
  UB_PORT.DIRSET.reg = UB_TXBM;
  UB_PORT.DIRCLR.reg = UB_RXBM;
  // pincfg using wrconfig write, s/o
  // https://community.atmel.com/forum/sam-d21-spi-interface-bare-code
  PORT_WRCONFIG_Type wrconfig;  // make new write config object,
  wrconfig.bit.WRPMUX = 1;      // it will write to pmux
  wrconfig.bit.WRPINCFG = 1;    // it will write to pinconfig
  wrconfig.bit.PMUX = MUX_PA16C_SERCOM1_PAD0;  // with this pmux setting
                                                // (putting 16 on c, for ser1)
  wrconfig.bit.PMUXEN = 1;                     // enabling pin muxing
  wrconfig.bit.HWSEL = 1;  // writing to the upper half of the pins
                            // and (below) writing these pins, masked and
                            // shifted into the lower half
  wrconfig.bit.PINMASK = (uint16_t)((UB_TXBM | UB_RXBM) >> 16);
  UB_PORT.WRCONFIG.reg = wrconfig.reg;  // here's the one-shot write, using prep above
  // ------------------------------------------ Transmit Driver / Recieve
  // Driver Enable
  UB_DE_SETUP;
  UB_RE_SETUP;
  // ------------------------------------------ SPI CONFIG
  // now, lettuce unmask the peripheral SER1
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;
  // hook the peripheral up to our main CPU clock, which is running at 48mHz
  // on the D21
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_ID_SERCOM1_CORE;
  while (GCLK->STATUS.bit.SYNCBUSY);
  // now we can setup the actual sercom, first do a reset for posterity and
  // await complete
  UB_SER_USART.CTRLA.bit.SWRST = 1;
  while (UB_SER_USART.SYNCBUSY.bit.SWRST);
  // pinout: TX on SERx-0, RX on SERx-2
  UB_SER_USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |     // lsb first
                            SERCOM_USART_CTRLA_MODE(1) |  // internal clock
                            SERCOM_USART_CTRLA_TXPO(0) |  // tx on SERx-0
                            SERCOM_USART_CTRLA_RXPO(UB_RXPO);  // rx on SERx-3
  // enable reciever, transmit,
  UB_SER_USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;
  // set BAUD:
  UB_SER_USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(ub_baud_val);
  // we will use interrupts: not the highest priority (0), just under. 
  NVIC_EnableIRQ(SERCOM1_IRQn);
  NVIC_SetPriority(SERCOM1_IRQn, 1);
  // rx interrupt always
  UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE | SERCOM_USART_INTENCLR_TXC;
  // UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  // ok I think that's it?
  UB_SER_USART.CTRLA.bit.ENABLE = 1;
  while (UB_SER_USART.SYNCBUSY.bit.ENABLE);
}

void SERCOM1_Handler(void) {
  if (UB_SER_USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
    ucBusDrop_rxISR();
  } 
  if (UB_SER_USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE && dreISR) {
    ucBusDrop_dreISR();
  } 
  if (UB_SER_USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC && txcISR){
    ucBusDrop_txcISR();
  } 
} // ------------------------------------------------------ END SERCOM ISR
// ------------------------------------ END D21 SPECIFIC 
#endif 

void ucBusDrop_setup(boolean useDipPick, uint8_t ID) {
  #ifdef UCBUS_IS_D51
  dip_setup();
  if(useDipPick){
    // set our id, 
    id = dip_readLowerFive(); // should read lower 4, now that cha / chb 
  } else {
    id = ID;
  }
  #endif 
  #ifdef UCBUS_IS_D21
  id = ID;
  #endif 
  if(id > 31){ id = 31; }   // max 31 drops, logical addresses 1 - 31
  if(id == 0){ id = 1; }    // 0 'tap' is the clk reset, bump up... maybe cause confusion: instead could flash err light 
  // setup input / etc buffers 
  for(uint8_t ch = 0; ch < UB_CH_COUNT; ch ++){
    recieveBufferWp[ch] = 0;
    inBufferLen[ch] = 0;
    outBufferRp[ch] = 0;
    outBufferLen[ch] = 0;
    rcrxb[ch] = 0;
  }
  // pick baud, via top level config.h 
  // baud bb baud
  // 63019 for a very safe 115200
  // 54351 for a go-karting 512000
  // 43690 for a trotting pace of 1MHz
  // 21845 for the E30 2MHz
  // 0 for max-speed 3MHz
  switch(UCBUS_BAUD){
    case 1:
      ub_baud_val = 43690;
      break;
    case 2: 
      ub_baud_val = 21845;
      break;
    case 3: 
      ub_baud_val = 0;
      break;
    default:
      ub_baud_val = 43690;
  }
  // start the hardware 
  setupBusDropUART();
}

uint16_t ucBusDrop_getOwnID(void){
  return id;
}

void ucBusDrop_rxISR(void){
  // ------------------------------------------------------ DATA INGEST
  // get the data 
  uint8_t data = UB_SER_USART.DATA.reg;
  inWord[inWordWp ++] = data;
  // tracking delineation 
  if(inWordWp >= UB_HEAD_BYTES_PER_WORD){
    // always reset, never overwrite inWord[] tail
    inWordWp = 0;
    // is lastchar the rarechar ?
    if(inWord[UB_HEAD_BYTES_PER_WORD - 1] == UCBUS_RARECHAR){
      // carry on, 
    } else {
      // restart on appearance of rarechar 
      for(uint8_t b = 0; b < UB_HEAD_BYTES_PER_WORD; b ++){
        if(inWord[b] == UCBUS_RARECHAR){
          inWordWp = UB_HEAD_BYTES_PER_WORD - 1 - b;
          // in case the above ^ causes some wrapping case (?) don't think it does though 
          if(inWordWp >= UB_HEAD_BYTES_PER_WORD) inWordWp = 0;
          return;
        }
      }
    }
  } else {
    // was just data byte, bail for now 
    return;
  }
  // ------------------------------------------------------ TERMINAL BYTE CASE 
  // blink on count-of-words:
  timeTick ++;
  timeBlink ++;
  if(timeBlink >= blinkTime){
    CLKLIGHT_TOGGLE; 
    timeBlink = 0;
  }
  // extract the header, 
  inHeader.bytes[0] = inWord[0];
  inHeader.bytes[1] = inWord[1];
  // now, check for our-rx:
  if(inHeader.bits.DROPTAP == id){  // -------------------- OUR TAP, TX CASE 
    // read-in fc states, 
    rcrxb[0] = inHeader.bits.CH0FC;
    rcrxb[1] = inHeader.bits.CH1FC;
    // reset out header,
    outHeader.bytes[0] = 0; 
    outHeader.bytes[1] = 0;
    // write outgoing flowcontrol terms: if we have unread buffers on these chs, zero space avail:
    outHeader.bits.CH0FC = (inBufferLen[0] ?  0 : 1);
    outHeader.bits.CH1FC = (inBufferLen[1] ?  0 : 1);
    // write also our drop tap...
    outHeader.bits.DROPTAP = id;
    // check about tx state, 
    for(uint8_t ch = 0; ch < UB_CH_COUNT; ch ++){
      if(outBufferLen[ch] && rcrxb[ch] > 0){
        // can tx this ch, 
        uint8_t numTx = outBufferLen[ch] - outBufferRp[ch];
        if(numTx > UB_DATA_BYTES_PER_WORD) numTx = UB_DATA_BYTES_PER_WORD;
        // can fill ch-output, 
        outHeader.bits.CHSELECT = ch;
        outHeader.bits.TOKENS = numTx;
        // fill bytes,
        uint8_t* outB = outBuffer[ch];
        uint16_t outBRp = outBufferRp[ch];
        for(uint8_t b = 0; b < numTx; b ++){
          outWord[b + 2] = outB[outBRp + b];  // fill from ob[2], ob[0] and ob[1] are header 
        }
        outBufferRp[ch] += numTx;
        // if numTx < data bytes / frame, packet terminates this word, we reset 
        if(numTx < UB_DATA_BYTES_PER_WORD){
          outBufferLen[ch] = 0;
          outBufferRp[ch] = 0;
        }
        break; // don't check next ch, 
      }
    }
    // stuff header -> word
    outWord[0] = outHeader.bytes[0];
    outWord[1] = outHeader.bytes[1];
    // now setup the transmit action:
    // set driver on, ship 1st byte, tx rest on DRE edges 
    outWordRp = 1; // next is [1]
    UB_DRIVER_ENABLE;
    UB_SER_USART.DATA.reg = outWord[0];
    DRE_ISR_ON;
  } // ---------------------------------------------------- END TX CASE 

  // ------------------------------------------------------ BEGIN RX TERMS 
  // the ch that head tx'd to 
  uint8_t rxCh = inHeader.bits.CHSELECT;
  // and # bytes tx'd here 
  uint8_t numToken = inHeader.bits.TOKENS;
  // check for broken numToken count,
  if(numToken > UB_DATA_BYTES_PER_WORD) { 
    OSAP::error("ucbus-drop outsize numToken rx", MINOR); 
    return; 
  }
  // don't overfill recieve buffer: 
  if(recieveBufferWp[rxCh] + numToken > UB_BUFSIZE){
    recieveBufferWp[rxCh] = 0;
    OSAP::error("ucbus-drop rx overfull buffer", MINOR);
    return;
  }
  // so let's see, if we have any we write them in:
  if(numToken > 0){
    uint8_t* rxB = recieveBuffer[rxCh];
    uint16_t rxBWp = recieveBufferWp[rxCh]; 
    for(uint8_t i = 0; i < numToken; i ++){
      rxB[rxBWp + i] = inWord[2 + i];
    }
    recieveBufferWp[rxCh] += numToken;
    // set in-packet state,
    lastWordHadToken[rxCh] = true;
  }
  // to find the edge, if we have numToken < numDataBytes and have at least one previous
  // token in stream, we have pckt edge 
  if((numToken < UB_DATA_BYTES_PER_WORD) && lastWordHadToken[rxCh]){
    // reset token edge
    lastWordHadToken[rxCh] = false;
    // pckt edge on this ch, shift recieveBuffer -> inBuffer and reset write pointer 
    // unfortunately we have to do this literal-swap thing (some memcpy coming up here), 
    // but should be able to use a pointer-swapping approach later. here we check if the pck 
    // is actually for us, then if we can accept it (fc not violated) and then swap it in:
    if(recieveBuffer[rxCh][0] == id || recieveBuffer[rxCh][0] == 0){
      // we should accept this, can we?
      if(inBufferLen[rxCh] != 0){ // failed to clear before new arrival, FC has failed 
        recieveBufferWp[rxCh] = 0;
        OSAP::error("ucbus-drop rx fc fails", MINOR);
        return;
      } // end check-for-overwrite 
      // copy from rxbuffer to inbuffer, it's ours... now FC will go lo, head should not tx *to us*
      // before it is cleared with ucBusDrop_readB()
      memcpy(inBuffer[rxCh], recieveBuffer[rxCh], recieveBufferWp[rxCh]);
      inBufferLen[rxCh] = recieveBufferWp[rxCh];
      recieveBufferWp[rxCh] = 0;
      // if CH0, fire "RT" on-rx interrupt, this is where we should want RTOS in the future 
      if(rxCh == 0){
        ucBusDrop_onPacketARx(&(inBuffer[0][1]), inBufferLen[0] - 1);
        // assuming the interrupt is the exit for time being,
        inBufferLen[0] = 0;
      }
      //DEBUG1PIN_OFF;
    } else {
      // packet wasn't for us, ignore 
      recieveBufferWp[rxCh] = 0;
    }
  } // ---------------------------------------------------- END RX TERMS

  // finally (and a bit yikes) we call the onRxISR on *every* word, that's our 
  // synced system clock: fair warning though, we're firing this pretty late
  // esp. if we have also this time transmitted, read in a packet, etc... yikes 
  ucBusDrop_onRxISR();
} // end rx-isr 

void ucBusDrop_dreISR(void){
  UB_SER_USART.DATA.reg = outWord[outWordRp ++];
  if(outWordRp >= UB_DROP_BYTES_PER_WORD){
    DRE_ISR_OFF; // clear tx-empty int.
    TXC_ISR_ON;  // set tx-complete int.
  } 
}

void ucBusDrop_txcISR(void){
  UB_SER_USART.INTFLAG.reg = SERCOM_USART_INTFLAG_TXC;   // clear flag (so interrupt not called again)
  TXC_ISR_OFF;
  UB_DRIVER_DISABLE;
}

// -------------------------------------------------------- ASYNC API

boolean ucBusDrop_ctrB(void){
  // clear to read a packet when this buffer occupied... 
  return (inBufferLen[1] > 0);
}

size_t ucBusDrop_readB(uint8_t *dest){
  if(!ucBusDrop_ctrB()) return 0;
  // to read-out, we rm the 0th byte which is addr information
  size_t len = inBufferLen[1] - 1;
  memcpy(dest, &(inBuffer[1][1]), len);
  inBufferLen[1] = 0; // now it's empty 
  return len;
}

boolean ucBusDrop_ctsB(void){
  if(outBufferLen[1] == 0 && rcrxb[1] > 0){
    return true;
  } else {
    return false;
  }
}

void ucBusDrop_transmitB(uint8_t *data, uint16_t len){
  if(!ucBusDrop_ctsB()) return;
  // we don't need to decriment our count of the remote rcrxb here
  // because we get an update from the head on their actual rcrxb *each time we are tapped*
  // however, we cannot tx more than the bufsize, bruh 
  if(len > UB_BUFSIZE) return;
  // copy it into the outBuffer, 
  memcpy(&(outBuffer[1]), data, len);
  // needs to be interrupt safe: transmit could start between these lines
  __disable_irq();
  outBufferLen[1] = len;
  outBufferRp[1] = 0;
  __enable_irq();
}

#endif 