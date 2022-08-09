/*
 * main.c
 *
 * Created: 8/9/2022 12:13:48 PM
 *  Author: igor.abramov
 */ 

#define F_CPU	16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define DDSOut	 (Check(PORTD, 7))
#define DDSOutInv Inv(PORTD, 7)

#define Button	(PIND  & (1<<2))

#define Init	 0
#define On		 1
#define Off		 2

#define FillCell    0xFF
#define Terminator  '$'
#define Eraser		' '
#define StringEnd	'\0'
#define CR			'\r'
#define LF			'\n'

#define Waiting		1
#define Record		2
#define Comparsion	3
#define Finished	4
#define Pause		5

#define Accel		10
#define Decel		20
#define Moving		30
#define Stop		40

#include <xc.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <util/delay.h>


int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
    }
}