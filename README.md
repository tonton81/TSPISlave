# TSPISlave
SPI Slave Library for Teensy LC/3.x


  SPI slave library designed for Teensy 3.x / LC. Allows one or more ports enabling with a unified, simple, sketch.
 
  SPI0, SPI1, and SPI2 are supported accross all boards.
 
  Supports multiple byte/word transfers.
 
  Supports 8 or 16bit wide transfers.
 
  
  LC connections:
   
    LC     HOST
    MISO -> MOSI
    MOSI -> MISO
  
  T3.x connections:
  
    T3     HOST
    MISO -> MISO
    MOSI -> MOSI
  
  
  Supported pin support for slaves' bus:
 
      Teensy LC:
        SPI:  2,10,11,12,13,14
        SPI1: 0,5,6,20
      Teensy 3x:
        SPI:  2,10,11,12,13,14
        SPI1: 0,1,31,32
        SPI2: 43,51,52,53
