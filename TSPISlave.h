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

#if !defined(_TSPISLAVE_H_)
#define _TSPISLAVE_H_

#include "Arduino.h"
#include <SPI.h>

typedef void (*_spi_ptr)();

class TSPISlave {

  public:
    TSPISlave(SPIClass& _port, uint8_t _miso, uint8_t _mosi, uint8_t _sck, uint8_t _cs, uint8_t _fmsz = 16);
    void onReceive(_spi_ptr handler);
    _spi_ptr _spihandler = nullptr;
    static TSPISlave* spi_slaves[3];
    volatile uint32_t spi_map;
    volatile uint32_t spi_irq;
    bool available();
    bool active();
    void pushr(uint16_t data);
    uint16_t popr();
    uint8_t sck, miso, mosi, cs;

  private:
    SPIClass *port;
    bool setSlaveMOSI(uint8_t pin);
    bool setSlaveMISO(uint8_t pin);
    bool setSlaveSCK(uint8_t pin);
    bool setSlaveCS(uint8_t pin);
};

#endif