/*
 * main.c
 *
 * Created: 3/14/2022 3:07:47 PM
 *  Author: igor.abramov
 */ 

/*
  Frequency comparer with counter inputs
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

#define Right    (~PIND & (1<<2))
#define Left     (~PIND & (1<<3))
#define Enter    (PIND  & (1<<4))

#define BtnReset (Check(PINC, 6))

#define Activity (!Check(PIND, 5))

#define Phase    (Check(PORTD, 6))
#define PhaseOn  High(PORTD, 6)
#define PhaseOff Low(PORTD, 6)
#define PhaseInv Inv(PORTD, 6)

#define Init	 0
#define On		 1
#define Off		 2
#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define StartSPI	Low(PORTB, 2)
#define EndSPI		High(PORTB, 2)

#define MovAvgSize	50

#define RxBufferSize    100
#define TxBufferSize	100

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define Arrow		'>'
#define Eraser		' '
#define StringEnd	'\0'
#define CR			'\r'
#define LF			'\n'

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
#include "lcd/lcd.h"

volatile struct
{
	volatile unsigned int ms16, ms160, ms992;
} MainTimer;

volatile struct
{
	unsigned int pulseA, pulseP, ovf;
	float Fa, Fp;
} Measure;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(0 << CS00);
		TIMSK0 = (1 << TOIE0);
		TCNT0 = 0;
		return;
	}
	
	TCCR0B = (0 << CS02)|(0 << CS01)|(0 << CS00);
	TIMSK0 = (0 << TOIE0);
	TCNT0 = 0;
}

ISR(TIMER0_OVF_vect)
{
	Measure.ovf++;	
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(0 << CS10);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
	TIMSK1 = (0 << TOIE1)|(0 << ICIE1);
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1 << CS22)|(0 << CS21)|(1 << CS20);
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

	if (MainTimer.ms16 % 10 == 0) MainTimer.ms160++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT2 = 5;
}

float MovAvgAramid(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[MovAvgSize];
	static float result;
	
	if (reset)
	{
		memset(values, 0, MovAvgSize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % MovAvgSize;
	
	return result/MovAvgSize;
}

float MovAvgPolyamide(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[MovAvgSize];
	static float result;
	
	if (reset)
	{
		memset(values, 0, MovAvgSize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % MovAvgSize;
	
	return result/MovAvgSize;
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
	static char frequencyAramid[20], frequencyPolyamide[20];
	
	EraseUnits(0, 0, 0, Measure.Fa);
	sprintf(frequencyAramid, "A%.1f", Measure.Fa);
	lcd_puts(frequencyAramid);
	
	EraseUnits(0, 1, 0, Measure.Fp);
	sprintf(frequencyPolyamide, "P%.1f", Measure.Fp);
	lcd_puts(frequencyPolyamide);
}

void Initialization()
 {
	 DDRB = 0b00111111;
	 PORTB = 0b00000111;
	 
	 DDRC = 0b00111111;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b11001110;
	 PORTD = 0b00110011;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_clrscr();
	 lcd_home();
	 
	 MovAvgAramid(0, true);
	 MovAvgPolyamide(0, true);
	 
	 Timer0(true);
	 Timer1(true);
	 Timer2(true);
	 sei();
 }

void Calculation()
{
	Measure.Fa = MovAvgAramid(((255.*Measure.ovf)+TCNT0)*8.008064516129032, false);
	Measure.Fp = MovAvgPolyamide(TCNT1*8.008064516129032, false);
	
	TCNT0 = 0;
	TCNT1 = 0;
}

int main(void)
{
	Initialization();
	
	while(1)
	{	
		if (MainTimer.ms160)
		{
			
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			Calculation();
			DisplayPrint();	
			MainTimer.ms992 = 0;
			TCNT0 = 0;
			TCNT1 = 0;
		}
	}
}