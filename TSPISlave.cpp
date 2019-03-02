/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Contributors:
  Tim - https://github.com/Defragster
  Mike - https://github.com/mjs513

  Designed and tested for PJRC Teensy 3.2, 3.5, 3.6, and LC boards.

  Forum link : https://forum.pjrc.com/threads/54548-TSPISlave-Library

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "Arduino.h"
#include "TSPISlave.h"
#include "SPI.h"
TSPISlave* TSPISlave::spi_slaves[3] = { nullptr, nullptr, nullptr };
void tspi0_isr(void);
void tspi1_isr(void);
void tspi2_isr(void);

TSPISlave::TSPISlave(SPIClass& _port, uint8_t _miso, uint8_t _mosi, uint8_t _sck, uint8_t _cs, uint8_t _fmsz) {
  port = &_port;
  ( _fmsz <= 8 ) ? _fmsz = 8 : _fmsz = 16; 

  if ( &_port == &SPI ) {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    SIM_SCGC6 |= SIM_SCGC6_SPI0; // enable slave clock
    spi_map = 0x4002C000;
#elif defined(KINETISL)
    SIM_SCGC4 |= SIM_SCGC4_SPI0; // enable slave clock
    SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTC; // enable ports
    spi_map = 0x40076000;
#endif
    spi_irq = IRQ_SPI0;
    _VectorsRam[16 + IRQ_SPI0] = tspi0_isr;
    spi_slaves[0] = this;
  }
#if defined(KINETISL) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( &_port == &SPI1 ) {
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    SIM_SCGC6 |= SIM_SCGC6_SPI1; // enable slave clock
    spi_map = 0x4002D000;
#elif defined(KINETISL)
    SIM_SCGC4 |= SIM_SCGC4_SPI1; // enable slave clock
    SIM_SCGC5 |= SIM_SCGC5_PORTE; // enable port
    spi_map = 0x40077000;
#endif
    spi_irq = IRQ_SPI1;
    _VectorsRam[16 + IRQ_SPI1] = tspi1_isr;
    spi_slaves[1] = this;
  }
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( &_port == &SPI2 ) {
    SIM_SCGC3 |= SIM_SCGC3_SPI2; // enable slave clock
    spi_map = 0x400AC000;
    spi_irq = IRQ_SPI2;
    _VectorsRam[16 + IRQ_SPI2] = tspi2_isr;
    spi_slaves[2] = this;
  }
#endif

#if defined(KINETISL)
  (*(KINETISL_SPI_t *)spi_map).C1 = ((uint8_t)0b00000000); // disable spi
  (*(KINETISL_SPI_t *)spi_map).C2 = ((uint8_t)(( _fmsz == 8 ) ? 0x00 : 0x40));  //((uint8_t)0b01000000);
  (*(KINETISL_SPI_t *)spi_map).BR = ((uint8_t)0b00000000);
  (*(KINETISL_SPI_t *)spi_map).C1 = ((uint8_t)0b11101100);
#endif

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).MCR = 0x00000000;
  (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).CTAR0 = 0;
  (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).CTAR0 = SPI_CTAR_FMSZ(_fmsz - 1);
  (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).CTAR0 = (*(KINETISK_SPI_t *)spi_map).CTAR0 & ~(SPI_CTAR_CPOL | SPI_CTAR_CPHA);
  (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  (*(KINETISK_SPI_t *)spi_map).RSER = 0x00020000;
#endif

  setSlaveMOSI(mosi = _mosi);
  setSlaveMISO(miso = _miso);
  setSlaveCS(cs = _cs);
  setSlaveSCK(sck = _sck);

  NVIC_SET_PRIORITY(spi_irq, 1); // set priority
  NVIC_ENABLE_IRQ(spi_irq); // enable CS IRQ
}

bool TSPISlave::setSlaveMISO(uint8_t pin) {
  if ( pin == 1 ) {
    CORE_PIN1_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 5 ) {
    CORE_PIN5_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 8 ) {
    CORE_PIN8_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  else if ( pin == 12 ) {
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( pin == 51 ) {
    CORE_PIN51_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
#endif
  return 0;
}

bool TSPISlave::setSlaveMOSI(uint8_t pin) {
  if ( pin == 0 ) {
    CORE_PIN0_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 21 ) {
    CORE_PIN21_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 7 ) {
    CORE_PIN7_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    return 1;
  }
  else if ( pin == 11 ) {
    CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    return 1;
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( pin == 52 ) {
    CORE_PIN52_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    return 1;
  }
#endif
  return 0;
}

bool TSPISlave::setSlaveCS(uint8_t pin) {
  if ( pin == 2 ) { // SPI0, T3.X, LC
    CORE_PIN2_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 6 ) { // SPI1, LC
    CORE_PIN6_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 10 ) { // SPI0, T3.X, LC
    CORE_PIN10_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    return 1;
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( pin == 31 ) {
    CORE_PIN31_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 43 ) { // SPI2, T3.X
    CORE_PIN43_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    return 1;
  }
#endif
  return 0;
}

bool TSPISlave::setSlaveSCK(uint8_t pin) {
  if ( pin == 13 ) {
    CORE_PIN13_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 14 ) {
    CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 20 ) {
    CORE_PIN20_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( pin == 32 ) {
    CORE_PIN32_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
  if ( pin == 53 ) {
    CORE_PIN53_CONFIG = PORT_PCR_MUX(2);
    return 1;
  }
#endif
  return 0;
}

void TSPISlave::onReceive(_spi_ptr handler) {
  _spihandler = handler;
}

bool TSPISlave::available() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  return (*(KINETISK_SPI_t *)spi_map).SR & 0xF0;
#elif defined(KINETISL)
  (*(KINETISL_SPI_t *)spi_map).C1 &= ~SPI_C1_SPIE;
  return ((*(KINETISL_SPI_t *)spi_map).S & SPI_S_SPRF || (*(KINETISL_SPI_t *)spi_map).S & SPI_S_SPTEF);
#endif
}

bool TSPISlave::active() {
  return !digitalRead(cs);
}

void TSPISlave::pushr(uint16_t data) {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  (*(KINETISK_SPI_t *)spi_map).PUSHR = data;
#elif defined(KINETISL)
  (*(KINETISL_SPI_t *)spi_map).DH = data >> 8;
  (*(KINETISL_SPI_t *)spi_map).DL = data;
  (*(KINETISL_SPI_t *)spi_map).C1 &= ~SPI_C1_SPTIE;
#endif
}

uint16_t TSPISlave::popr() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  return (*(KINETISK_SPI_t *)spi_map).POPR;
#elif defined(KINETISL)
  return (((*(KINETISL_SPI_t *)spi_map).DH) << 8 | ((*(KINETISL_SPI_t *)spi_map).DL));
#endif
  return 1;
}

void tspi0_isr(void) {
  if ( TSPISlave::spi_slaves[0] && TSPISlave::spi_slaves[0]->_spihandler ) {
    TSPISlave::spi_slaves[0]->_spihandler();
  }

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else {
    SPI0_PUSHR_SLAVE = 0;
    SPI0_POPR;
  }
  SPI0_SR |= SPI_SR_RFDF;

#elif defined(KINETISL)
  else {
    KINETISL_SPI0.DH;
    KINETISL_SPI0.DL;
    KINETISL_SPI0.DH = 0;
    KINETISL_SPI0.DL = 0;
  }
  KINETISL_SPI0.C1 &= ~SPI_C1_SPTIE;
  KINETISL_SPI0.C1 |= SPI_C1_SPIE;
#endif
}

void tspi1_isr(void) {
  if ( TSPISlave::spi_slaves[1] && TSPISlave::spi_slaves[1]->_spihandler ) {
    TSPISlave::spi_slaves[1]->_spihandler();
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else {
    SPI1_PUSHR_SLAVE = 0;
    SPI1_POPR;
  }
  SPI1_SR |= SPI_SR_RFDF;

#elif defined(KINETISL)
  else {
    KINETISL_SPI1.DH;
    KINETISL_SPI1.DL;
    KINETISL_SPI1.DH = 0;
    KINETISL_SPI1.DL = 0;
  }
  KINETISL_SPI1.C1 &= ~SPI_C1_SPTIE;
  KINETISL_SPI1.C1 |= SPI_C1_SPIE;
#endif
}

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
void tspi2_isr(void) {
  if ( TSPISlave::spi_slaves[2] && TSPISlave::spi_slaves[2]->_spihandler ) {
    TSPISlave::spi_slaves[2]->_spihandler();
  }
  else {
    SPI2_PUSHR_SLAVE = 0;
    SPI2_POPR;
  }
  SPI2_SR |= SPI_SR_RFDF;
}
#endif
