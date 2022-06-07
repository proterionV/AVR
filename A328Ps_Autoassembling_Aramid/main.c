/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 */ 

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define ImpOn  High(PORTD, 7)
#define ImpOff Low(PORTD, 7)
#define ImpInv Inv(PORTD, 7)

#define DirInv	(!Check(PIND, 2)) 

#define Counter 1
#define Off		0

#define Right 2
#define Left  3

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <avr/eeprom.h>
#include "lcd/lcdpcf8574/lcdpcf8574.h"

volatile struct
{
	unsigned int interval, ms16, ms992;
} MainTimer;

void Timer2(unsigned short mode)
{
	if (mode == Counter)
	{
		TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);
		TIMSK2 = (1 << TOIE2);
		TCNT2 = 0;
		return;
	}
	
	TCCR2B = (0 << CS22)|(0 << CS21)|(0 << CS20);
	TIMSK2 = (0 << TOIE2);
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{
	MainTimer.ms16++;
	if (MainTimer.interval > 0) MainTimer.interval--;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT2 = 5;
}

void EraseUnits(int x, int y, int offset, float count)
{
	char eraser = 32;

	if (count<10000)
	{
		lcd_gotoxy(x+offset+4,y);
		lcd_putc(eraser);
	}
	
	if (count<1000)
	{
		lcd_gotoxy(x+offset+3,y);
		lcd_putc(eraser);
	}
	
	if (count<100)
	{
		lcd_gotoxy(x+offset+2,y);
		lcd_putc(eraser);
	}
	
	if (count<10)
	{
		lcd_gotoxy(x+offset+1,y);
		lcd_putc(eraser);
	}
	
	
	lcd_gotoxy(x, y);
}

void DisplayPrint()
{
	//static char A[20] = { 0 }, P[20] = { 0 };
	
	//EraseUnits(0, 0, 3, Measure.Fa);
	//sprintf(A, "%.2f", Measure.Fa < 0 ? 0 : Measure.Fa);
	//lcd_puts(A);
	//
	//EraseUnits(0, 1, 3, Measure.Fp);
	//sprintf(P, "%.2f", Measure.Fp < 0 ? 0 : Measure.Fp);
	//lcd_puts(P);
	
	//if (Mode.alarm)
	//{
		//lcd_clrline(9, 1);
		//lcd_puts("Alarm");
	//}
}

void Initialization(const char* name)
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b11100011;
	PORTD = 0b00011100;
	
	lcd_init(LCD_DISP_ON);
	lcd_led(false);
	lcd_clrscr();
	lcd_home();
	
	lcd_puts(name);
	_delay_ms(2000);
	lcd_clrscr();
	lcd_home();
	
	Timer2(Counter);
	sei();
}

void Step(short direction)
{
	ImpOn;
	if (direction == Left)  _delay_us(200);
	if (direction == Right) _delay_ms(4);
	ImpOff;
	_delay_ms(5);
}

int main(void)
{
	Initialization("Hello world!");
	
    while(1)
    {
		if (DirInv) Step(Left);
		else Step(Right);
		
        if (MainTimer.ms992)
        {
			LedInv;
		    MainTimer.ms992 = 0;
        } 
    }
}