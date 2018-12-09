#include "TSPISlave.h"

uint8_t miso = 51;
uint8_t mosi = 52;
uint8_t sck = 53;
uint8_t cs = 43;
uint8_t spimode = 16;

TSPISlave mySPI = TSPISlave(SPI2, miso, mosi, sck, cs, spimode);

void setup() {
  Serial.begin(115200);
  mySPI.onReceive(myFunc);
}

void loop() {
  delay(1000);
  Serial.println(millis());
}

void myFunc() {
  Serial.println("START: ");
  uint16_t arr[2] = { 0x1111, 0x2222 };
  uint8_t i = 0;
  while ( mySPI.active() ) {
    if (mySPI.available()) {
      mySPI.pushr(arr[i++]);
      Serial.print("VALUE: 0x");
      Serial.println(mySPI.popr(), HEX);
    }
  }
  Serial.println("END");
}
