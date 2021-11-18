/*
osap/drivers/ucBusHead.cpp

uart-based clock / bus combo protocol

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2019

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/

#include "ucBusHead.h"

#ifdef UCBUS_IS_HEAD

#include <Arduino.h>

#include "../../../indicators.h"
#include "../../../syserror.h"
#include "../../peripheralNums.h"
#include "../../d51ClockBoss.h"
#include "../../osape/utils/cobs.h"

// input buffers / space 
uint8_t inBuffer[UB_CH_COUNT][UB_MAX_DROPS][UB_BUFSIZE];   // per-drop incoming bytes: 0 will be empty always, no drop here
volatile uint16_t inBufferWp[UB_CH_COUNT][UB_MAX_DROPS];   // per-drop incoming write pointer
volatile uint16_t inBufferLen[UB_CH_COUNT][UB_MAX_DROPS];  // per-drop incoming bytes, len of, set when EOP detected
volatile boolean lastWordHadToken[UB_CH_COUNT][UB_MAX_DROPS];

// transmit buffers 
uint8_t outBuffer[UB_CH_COUNT][UB_BUFSIZE];
volatile uint16_t outBufferRp[UB_CH_COUNT];
volatile uint16_t outBufferLen[UB_CH_COUNT];

// flow control, per ch per drop 
volatile uint8_t rcrxb[UB_CH_COUNT][UB_MAX_DROPS];     // if 0 donot tx on this ch / this drop 

// currently 'tapped' drop - we loop thru bus drops, 
volatile uint8_t currentDropTap = 1; // drop we are currently 'txing' to / drop that will reply on this cycle
volatile uint8_t lastDropTap = 1; 

// outgoing word / stuff info 
volatile UCBUS_HEADER_Type outHeader = { .bytes = { 0, 0 } };
uint8_t outWord[UB_HEAD_BYTES_PER_WORD];                // this goes on-the-line, 
volatile uint8_t outWordRp = 0;

// incoming word 
volatile UCBUS_HEADER_Type inHeader = { .bytes = { 0, 0 } };
uint8_t inWord[UB_DROP_BYTES_PER_WORD];
uint8_t inWordWp = 0;

// baudrate 
uint32_t ub_baud_val = 0;

// uart init (file scoped)
void setupBusHeadUART(void){
  // driver output is always on at head, set HI to enable
  UB_DE_PORT.DIRSET.reg = UB_DE_BM;
  UB_DE_PORT.OUTSET.reg = UB_DE_BM;
  // receive output is always on at head, set LO to enable
  UB_RE_PORT.DIRSET.reg = UB_RE_BM;
  UB_RE_PORT.OUTCLR.reg = UB_RE_BM;
  // termination resistor for receipt on bus head is always on, set LO to enable 
  UB_TE_PORT.DIRSET.reg = UB_TE_BM;
  UB_TE_PORT.OUTCLR.reg = UB_TE_BM;
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
  // then, sercom: disable and then perform software reset
  while(UB_SER_USART.SYNCBUSY.bit.ENABLE);
  UB_SER_USART.CTRLA.bit.ENABLE = 0;
  while(UB_SER_USART.SYNCBUSY.bit.SWRST);
  UB_SER_USART.CTRLA.bit.SWRST = 1;
  while(UB_SER_USART.SYNCBUSY.bit.SWRST);
  while(UB_SER_USART.SYNCBUSY.bit.SWRST || UB_SER_USART.SYNCBUSY.bit.ENABLE);
  // ok, CTRLA:
  UB_SER_USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD; // data order (1: lsb first) and mode (?) 
  UB_SER_USART.CTRLA.reg |= SERCOM_USART_CTRLA_RXPO(UB_RXPO) | SERCOM_USART_CTRLA_TXPO(0); // rx and tx pinout options 
  //UB_SER_USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(1); // turn on parity: parity is even by default (set in CTRLB), leave that 
  // CTRLB has sync bit, 
  while(UB_SER_USART.SYNCBUSY.bit.CTRLB);
  // recieve enable, txenable, character size 8bit, 
  UB_SER_USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
  // CTRLC: setup 32 bit on read and write:
  // UBH_SER_USART.CTRLC.reg = SERCOM_USART_CTRLC_DATA32B(3); 
	// enable interrupts 
	NVIC_EnableIRQ(SERCOM1_2_IRQn); // rx interrupts 
  NVIC_EnableIRQ(SERCOM1_1_IRQn); // transmit complete interrupt 
	NVIC_EnableIRQ(SERCOM1_0_IRQn); // data register empty interrupts 
	// set baud 
  UB_SER_USART.BAUD.reg = ub_baud_val;
  // and finally, a kickoff
  while(UB_SER_USART.SYNCBUSY.bit.ENABLE);
  UB_SER_USART.CTRLA.bit.ENABLE = 1;
  // enable the RXC interrupt, disable TXC, DRE
  UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE | SERCOM_USART_INTENCLR_TXC;
}

// TX Handler, for second bytes initiated by timer, 
// void SERCOM1_0_Handler(void){
// 	ucBusHead_txISR();
// }

// startup, 
void ucBusHead_setup(void){
  // clear buffers to begin,
  for(uint8_t ch = 0; ch < UB_CH_COUNT; ch ++){
    outBufferLen[ch] = 0;
    outBufferRp[ch] = 0;
    for(uint8_t d = 0; d < UB_MAX_DROPS; d ++){
      inBufferLen[ch][d] = 0; // zero all input buffers, write-in pointers
      inBufferWp[ch][d] = 0;
      rcrxb[ch][d] = 0;       // assume zero space to tx to all drops until they report otherwise 
      lastWordHadToken[ch][d] = false;
    }
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
  // start the uart, 
  setupBusHeadUART();
  // ! alert ! need to setup timer in main.cpp 
}

void ucBusHead_timerISR(void){
  // increment / wrap time division for drops  
  currentDropTap ++;
  if(currentDropTap > UB_MAX_DROPS){ // recall that tapping '0' should operate the clock reset, addr 0 doesn't exist 
    currentDropTap = 1;
  }
  // reset the outgoing header, 
  outHeader.bytes[0] = 0; 
  outHeader.bytes[1] = 0;
  // write in drop tap, flowcontrol rules 
  outHeader.bits.CH0FC = (inBufferLen[0][currentDropTap] ?  0 : 1);
  outHeader.bits.CH1FC = (inBufferLen[1][currentDropTap] ?  0 : 1);
  outHeader.bits.DROPTAP = currentDropTap;                
  // now we check if we can tx on either channel, 
  for(uint8_t ch = 0; ch < UB_CH_COUNT; ch ++){
    // do we have ahn pck to be tx'ing, and is flowcontrol condition met 
    // FC: outBuffer[x][0] is the 'addr' we are tx'ing to, so indexes relevant rcrxb as well
    // ! and, when we broadcast (addr '0') we ignore FC rules, so: 
    if(outBufferLen[ch] > 0 && (rcrxb[ch][outBuffer[ch][0]] | (outBuffer[ch][0] == 0))){
      // ch has incomplete-tx of some packet 
      // count them, max we will transmit is from word length: 
      uint8_t numTx = outBufferLen[ch] - outBufferRp[ch];
      if(numTx > UB_DATA_BYTES_PER_WORD) numTx = UB_DATA_BYTES_PER_WORD;
      // we can write the 2nd header byte (ch select and # of words)
      outHeader.bits.CHSELECT = ch;
      outHeader.bits.TOKENS = numTx;
      // fill bytes, 
      uint8_t *outB = outBuffer[ch];
      uint16_t outBRp = outBufferRp[ch];
      for(uint8_t b = 0; b < numTx; b ++){ 
        outWord[b + 2] = outB[outBRp + b];
      }
      outBufferRp[ch] += numTx;
      // if numTx < data words per packet, packet will terminate this frame, we can reset 
      // recipient uses the tailing '0' token-d byte to delineate packets (COBS for words)
      if(numTx < UB_DATA_BYTES_PER_WORD) {
        // flow control: we have tx'd to whichever drop... the head recieves updates from drops 
        // for rcrxb, but they're potentially spaced 1/64 turns of this ISR, 
        // so we need to update our accounting of their space-available-to-receive.
        // recall also that rcrxb is parallel per channel *and* per drop 
        rcrxb[ch][outBuffer[ch][0]] = 0; // 0 space available here now, 
        outBufferLen[ch] = 0; // reset also the outgoing buffer,
        outBufferRp[ch] = 0;  // and it's read-out ptr 
      }
      break; // don't check the next ch, outword occupied by this 
    }
  }
  // stuff header -> outWord
  outWord[0] = outHeader.bytes[0];
  outWord[1] = outHeader.bytes[1];
  // insert rarechar 
  outWord[UB_HEAD_BYTES_PER_WORD - 1] = UCBUS_RARECHAR;
  // now we transmit: 
  UB_SER_USART.DATA.reg = outWord[0];
  outWordRp = 1; // next up, 
  UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

// data register empty: bang next byte in 
void SERCOM1_0_Handler(void){
  UB_SER_USART.DATA.reg = outWord[outWordRp ++];
  if(outWordRp >= UB_HEAD_BYTES_PER_WORD){ // if we've transmitted them all, 
    UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE; // clear tx-data-empty interrupt 
    UB_SER_USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC; // set tx-complete interrupt 
  }
}

// transmit complete interrupt: delimit incoming words 
void SERCOM1_1_Handler(void){
  UB_SER_USART.INTFLAG.bit.TXC = 1;
  UB_SER_USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;
  // this means the latest word transmit is done, next byte on the line should be 1st in 
  // upstream pckt 
  lastDropTap = currentDropTap;
  inWordWp = 0;
}

// rx handler, for incoming
void SERCOM1_2_Handler(void){
	ucBusHead_rxISR();
}

void ucBusHead_rxISR(void){
	// shift the byte -> incoming, 
  inWord[inWordWp ++] = UB_SER_USART.DATA.reg;
  if(inWordWp >= UB_DROP_BYTES_PER_WORD){
    // that's ^ word delineation, so our drop tap should be:
    uint8_t rxDrop = lastDropTap; 
    // check that, 
    inHeader.bytes[0] = inWord[0];
    inHeader.bytes[1] = inWord[1];
    if(inHeader.bits.DROPTAP != rxDrop){ return; } // bail on mismatch, was a bad / misaligned word
    // update our buffer states, 
    rcrxb[0][rxDrop] = inHeader.bits.CH0FC;
    rcrxb[1][rxDrop] = inHeader.bits.CH1FC; 
    // the ch that drop tx'd on 
    uint8_t rxCh = inHeader.bits.CHSELECT;
    // has anything?
    uint8_t numToken = inHeader.bits.TOKENS;
    // check for broken numToken count,
    if(numToken > UB_DATA_BYTES_PER_WORD) { ERROR(1, "ucbus-head outsize numToken rx"); return; }
    // if we are filling this buffer, but it's already occupied, fc has failed and we
    if(inBufferLen[rxCh][rxDrop] != 0){ ERROR(0, "ucbus-head rx FC broken"); return; }
    // donot write past buffer size,
    if(inBufferWp[rxCh][rxDrop] + numToken > UB_BUFSIZE){
      inBufferWp[rxCh][rxDrop] = 0;
      ERROR(0, "ucbus-head rx packet too-long");
      return;
    }
    // shift bytes into rx buffer 
    uint8_t * inB = inBuffer[rxCh][rxDrop];
    uint16_t inBWp = inBufferWp[rxCh][rxDrop];
    for(uint8_t i = 0; i < numToken; i ++){
      inB[inBWp + i] = inWord[2 + i];
    }
    inBufferWp[rxCh][rxDrop] += numToken;
    // to find packet edge, if we have numToken > numDataBytes and at least 
    // one other in the stream, we have pckt edge
    if(numToken > 0) lastWordHadToken[rxCh][rxDrop] = true;
    if(numToken < UB_DATA_BYTES_PER_WORD && lastWordHadToken[rxCh][rxDrop]){
      // packet edge, reset token edge
      lastWordHadToken[rxCh][rxDrop] = false;
      // pckt edge is here, set fullness, otherwise we're done, 
      // application responsible for shifting it out and 
      // inBufferLen is what we read to determine FC condition 
      inBufferLen[rxCh][rxDrop] = inBufferWp[rxCh][rxDrop];
      inBufferWp[rxCh][rxDrop] = 0;
    }
  }
}

// -------------------------------------------------------- API 

// clear to read ? channel select ? 
#warning TODO: bus head read per-ch: yep, should be a or b, 
boolean ucBusHead_ctr(uint8_t drop){
  // called once per loop, so here's where this debug goes:
  //(rcrxb[1] > 0) ? DEBUG2PIN_OFF : DEBUG2PIN_ON; // for psu-breakout,
  //(rcrxb[2] > 0) ? DEBUG3PIN_OFF : DEBUG3PIN_ON; // pin off is light on
  if(drop >= UB_MAX_DROPS) return false;
  if(inBufferLen[1][drop] > 0){
    return true;
  } else {
    return false;
  }
}

#warning TODO: bus head osap-read-in per-ch ? currently fixed to chb osap reads 
size_t ucBusHead_read(uint8_t drop, uint8_t *dest){
  if(!ucBusHead_ctr(drop)) return 0;
  size_t len = inBufferLen[1][drop];
  memcpy(dest, inBuffer[1][drop], len);
  __disable_irq(); // again... do we need these ? big brain time 
  inBufferLen[1][drop] = 0;
  inBufferWp[1][drop] = 0;
  __enable_irq();
  return len;
}

boolean ucBusHead_ctsA(void){
	if(outBufferLen[0] == 0){ 
    // only condition is that our transmit buffer is zero, are not currently tx'ing on this channel 
		return true;
	} else {
		return false;
	}
}

boolean ucBusHead_ctsB(uint8_t drop){
  // escape states 
  if(outBufferLen[1] == 0 && rcrxb[1][drop] > 0){
    return true; 
  } else {
    return false;
  }
}

#warning TODO: we have this awkward +1 in the buffer / segsize, vs what the app. sees... 
void ucBusHead_transmitA(uint8_t *data, uint16_t len){
	if(!ucBusHead_ctsA()) return;
  if(len > UB_BUFSIZE + 1) return; // none over buf size 
  // 1st byte: broadcast identifier 
  outBuffer[0][0] = 0;
  // copy in @ 1th byte 
  // we *shouldn't* have to guard against the memcpy, god bless, since 
  // the bus shouldn't be touching this so long as our outBufferLen is 0,
  // which - we are guarded against that w/ the flowcontrol check above 
  memcpy(&(outBuffer[0][1]), data, len);
  // len set 
  __disable_irq();
  outBufferLen[0] = len + 1;
  outBufferRp[0] = 0;
  __enable_irq();
}

void ucBusHead_transmitB(uint8_t *data, uint16_t len, uint8_t drop){
  if(!ucBusHead_ctsB(drop)) return;
  if(len > UB_BUFSIZE + 1) return; // same as above
  __disable_irq();
  // 1st byte: drop identifier 
  outBuffer[1][0] = drop;
  // copy in @ 1th byte 
  memcpy(&(outBuffer[1][1]), data, len);
  // length set 
  outBufferLen[1] = len + 1; // + 1 for the addr... 
  // read-out ptr reset 
  outBufferRp[1] = 0;
  __enable_irq();
}

#endif 