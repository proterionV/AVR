/*
 * main.c
 *
 * Created: 3/17/2022 11:55:07 AM
 * Author: igor.abramov
 * Measure vibration level as softness of paper yarn
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

#define MovAvgSize	2

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
	signed int value;
	float voltage;
	bool done;
} Convert;

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

void Converter(unsigned short option)
{
	switch (option)
	{
		case Off:
		ADCSRA |= (0<<ADSC);
		break;
		case On:
		ADCSRA |= (1<<ADSC);
		break;
		default:
		ADCSRA = 0x8F;
		ADMUX = 0x41;
		ADCSRA |= (0<<ADSC);
		break;
	}
}

ISR(ADC_vect)
{
	Converter(Off);
	Convert.value = ADCW;
	Convert.done = true;
}

float MovAvg(float value, bool reset)
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
	static char voltage[20], adcw[20];
	
	EraseUnits(0, 0, 2, Convert.voltage);
	sprintf(voltage, "%.1f V", Convert.voltage);
	lcd_puts(voltage);
	
	EraseUnits(0, 1, 0, Convert.value);
	sprintf(adcw, "%d", Convert.value);
	lcd_puts(adcw);
}

void Initialization()
 {
	 DDRB = 0b00111111;
	 PORTB = 0b00000111;
	 
	 DDRC = 0b00111100;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b11001110;
	 PORTD = 0b00110011;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_clrscr();
	 lcd_home();
	 
	 MovAvg(0, true);
	 
	 Timer2(true);
	 Converter(Init);
	 sei();
 }

int main(void)
{
	Initialization();
	Converter(On);
	
	while(1)
	{	
		if (Convert.done)
		{
			Convert.voltage = MovAvg(Convert.value*0.0048828125, false);
			Convert.done = false;
			Converter(On);
		}
		
		if (MainTimer.ms160)
		{
			DisplayPrint();
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			LedInv;	
			MainTimer.ms992 = 0;
		}
	}
}