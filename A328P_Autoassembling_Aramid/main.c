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

#define ForwardState	Check(PORTD, 6)
#define ForwardOn		High(PORTD, 6)
#define ForwardOff		Low(PORTD, 6)
#define ForwardInv		Inv(PORTD, 6)

#define BackyardState	Check(PORTD, 7)
#define BackyardOn		High(PORTD, 7)
#define BackyardOff		Low(PORTD, 7)
#define BackyardInv		Inv(PORTD, 7)

#define Stop		0
#define Forward	 	1
#define Backyard 	2
#define Inversion	3
#define Error		4

#define MovAvgSize	5

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
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
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
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
	TIMSK1 = (0 << TOIE1)|(0 << ICIE1);
}

void Timer2(bool enable)
{
	if (enable)
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

	if (MainTimer.ms16 % 10 == 0) MainTimer.ms160++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT2 = 5;
}

unsigned short CurrentMotorDirection()
{
	if (ForwardState && BackyardState) return Error;
	if (ForwardState)  return Forward;
	if (BackyardState) return Backyard;
	return Stop;
}

void MotorDirection(unsigned short option)
{
	static unsigned short direction;
	
	switch (option)
	{
		case Forward:
		BackyardOff;
		ForwardOn;
		break;
		case Backyard:
		ForwardOff;
		BackyardOn;
		break;
		case Inversion:
		direction = CurrentMotorDirection();
		MotorDirection(Stop);
		if (direction == Stop) MotorDirection(Forward);
		if (direction == Forward) MotorDirection(Backyard);
		if (direction == Backyard) MotorDirection(Forward);
		break;
		default:
		ForwardOff;
		BackyardOff;
		break;
	}
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
	static unsigned short direction;
	 
	EraseUnits(0, 0, 3, Measure.Fa);
	sprintf(frequencyAramid, "%.1f Hz", Measure.Fa);
	lcd_puts(frequencyAramid);
	
	EraseUnits(0, 1, 3, Measure.Fp);
	sprintf(frequencyPolyamide, "%.1f Hz", Measure.Fp);
	lcd_puts(frequencyPolyamide);
	
	lcd_clrline(9, 0);
	direction = CurrentMotorDirection();
	if (direction == Stop)	   lcd_puts("Stop");
	if (direction == Forward)  lcd_puts("Forth");
	if (direction == Backyard) lcd_puts("Back");
	if (direction == Error)	   lcd_puts("Error");	
}

void Initialization()
 {
	 DDRB = 0b00111111;
	 PORTB = 0b00000111;
	 
	 DDRC = 0b00111111;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b11001110;
	 PORTD = 0b00110011;
	 
	 ForwardOff;
	 BackyardOff;
	 
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
	Measure.Fa = MovAvgAramid(((255.*Measure.ovf)+TCNT0)*6.25, false);	 //1.008064516129032 
	Measure.Fp = MovAvgPolyamide(TCNT1*6.25, false);
	
	TCNT0 = 0;
	TCNT1 = 0;
}

void Regulator()
{
	static float difference = 0;
	
	difference = Measure.Fa - Measure.Fp;
	
	if (difference >= -1 && difference <= 1) 
	{
		if (CurrentMotorDirection() == Stop) return;
		MotorDirection(Stop);
		return;
	}
	
	if (difference > 1) MotorDirection(Forward); else MotorDirection(Backyard);	
}

int main(void)
{
	Initialization();
	
	while(1)
	{	
		if (MainTimer.ms160)
		{
			Calculation();
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			LedInv;
			Regulator();
			DisplayPrint();
			MainTimer.ms992 = 0;
		}
	}
}