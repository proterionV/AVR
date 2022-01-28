/*
Example	setting avrdude with external tool
Tools -> External Tools -> Argunebts:
avrdude -v -p m2560 -c stk500v2 -P com7 -b 115200 -D -U flash:w:"$(ProjectDir)Debug$(ItemFileName).hex":i
*/

/*
Example setting for correct float conversion
Project properties -> Toolchain -> XC8 Linker -> Miscellaneous -> Other linked flags (just put):
-Wl,-u,vfprintf -lprintf_flt -lm
*/

 /*
 Example setting for set Clock Frequency globally
 Project properties -> Toolchain -> XC8 C Compiler -> Symbols -> Defined Symbols (Add item):
 F_CPU=16000000UL
 */