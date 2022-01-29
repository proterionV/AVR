/*
 * main.c
 *
 * Created: 1/29/2022 6:18:04 PM
 *  Author: igor.abramov
 */ 

#define F_CPU    16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 7)
#define LedOn   High(PORTB, 7)
#define LedOff  Low(PORTB, 7)
#define LedInv  Inv(PORTB, 7)

#define Right    (~PINC & (1<<0))
#define Left     (~PINC & (1<<1))
#define Enter    (PINC & (1<<2))

#define Enable	 (!(PINL & (1<<2)))
#define Disable	 (PINL & (1<<2))

#define DDSOut	 (PORTB & (1<<0))
#define DDSOutInv PORTB ^= (1<<0);

#define Phase    (PORTB & (1<<2))
#define PhaseOn  PORTB |= (1<<2)
#define PhaseOff PORTB &= (0<<2)
#define PhaseInv PORTB ^= (1<<2)

#define True	 1
#define False	 0

#define Init	 2
#define On		 1
#define Off		 0

#define	Tension		1
#define Frequency 	0

#define Forward	    1
#define Reporcial 	0

#define FrequencyArraySize 150
#define TensionArraySize   30
#define RxBufferSize 100
#define TxBufferSize 100

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define StringEnd	'\0'

#include <xc.h>
#include <avr/io.h>
#include <float.h>
#include "lcd/lcd.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <avr/eeprom.h>

int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
    }
}