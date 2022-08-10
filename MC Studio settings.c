/*
Example	setting avrdude with external tool
Tools -> External Tools -> Arguments:
avrdude -v -p m328p -c arduino -P com6 -b 115200 -D -U flash:w:"$(ProjectDir)Debug\$(ItemFileName).hex":i
*/

/*
Example setting for correct float conversion
Project properties -> Toolchain -> XC8 Linker -> Miscellaneous -> Other linked flags (just put it):
-Wl,-u,vfprintf -lprintf_flt -lm
*/

 /*
 Example setting for set Clock Frequency globally
 Project properties -> Toolchain -> XC8 C Compiler -> Symbols -> Defined Symbols (Add item):
 F_CPU=16000000UL
 */
 
  /*
Atmega328P SPI, for correct transmit need make sending 2 times
 */