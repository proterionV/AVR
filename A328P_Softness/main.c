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
#include "lcd/lcdpcf8574/lcdpcf8574.h"

volatile struct
{
	volatile unsigned int ms200, ms1000, sec;
} MainTimer;

volatile struct
{
	signed int value;
	float voltage;
	bool done;
} Convert;
 
volatile struct
{
	float frequency;
	unsigned long int accum, increment;
} DDS;
 
volatile struct
{
	unsigned short mode, btn;
	bool active, action, changed;
} Mode;

volatile struct 
{
	float ungrinded, grinded;
	float difference;
} Measure;

const unsigned long int ACCUM_MAXIMUM = 1000000000;
const unsigned int		FREQUENCY_MAXIMUM = 7812;
const unsigned int		FREQUENCY = 2000;

void Timer1(unsigned short mode)
{
	switch(mode)
	{
		case On:
			TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
			TIMSK1 = (1 << TOIE1);
			TCNT1 = 62411;
			break;
		default:
			TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
			TIMSK1 = (0 << TOIE1);
			TCNT1 = 62411;
			break;
	}
}

ISR(TIMER1_OVF_vect)
{
	if (MainTimer.ms200 >= 5)
	{
		MainTimer.ms1000++;
		MainTimer.ms200 = 0;
	}
	
	MainTimer.ms200++;
	TCNT1 = 62411;
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // 1024 bit scaler
		TIMSK2 = (1<<TOIE2);
		return;
	}
	
	TCCR2B = (0<<CS22) | (0<<CS21) | (0<<CS20);
	TIMSK2 = (0<<TOIE2);
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{
	TCNT2 = 255;
	
	DDS.accum += DDS.increment;
	
	if (DDS.accum >= ACCUM_MAXIMUM)
	{
		DDSOutInv;
		DDS.accum -= ACCUM_MAXIMUM;
	}
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
			ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
			ADMUX = 0x41;
			break;
	}
}

ISR(ADC_vect)
{
	Converter(Off);
	Convert.value = ADCW;
	Convert.done = true;
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
	static char ungrinded[20], grinded[20], difference[20]; 
	
	EraseUnits(0, 0, 0, Measure.ungrinded);
	sprintf(ungrinded, "%.2f", Measure.ungrinded);
	lcd_gotoxy(0, 0);
	lcd_puts(ungrinded);
	
	EraseUnits(0, 1, 0, Measure.grinded);
	sprintf(grinded, "%.2f", Measure.grinded);
	lcd_gotoxy(0, 1);
	lcd_puts(grinded);
	
	EraseUnits(8, 0, 1, Measure.difference);
	sprintf(difference, "%.2f", Measure.difference);
	lcd_gotoxy(8, 0);
	lcd_puts(difference);
	lcd_putc('%');
	
	if (!Mode.changed) return;
	
	lcd_clrline(8, 1);
	
	switch(Mode.mode)
	{
		case Waiting:
			lcd_puts("Waiting");
			break;
		case Record:
			lcd_puts("Record");
			break;
		case Comparsion:
			lcd_puts("Comparsion");
			break;
		default:
			lcd_puts("Finished");
			break;	
	}
	
	Mode.changed = false;
}

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.frequency < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.frequency)/FREQUENCY_MAXIMUM)*divider;
}

void Initialization(const char* projectName)
 {
	 DDRB = 0b00111100;
	 PORTB = 0b00000011;
	 
	 DDRC = 0b00111100;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b11000010;
	 PORTD = 0b00111111;
	 
	 DDS.frequency = FREQUENCY;
	 DDS.increment = GetAddendum();
	 
	 Mode.mode = Waiting;
	 Mode.changed = true;

	 MainTimer.sec = 0;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_led(false);
	 lcd_gotoxy(4, 0);
	 lcd_puts(projectName);
	 _delay_ms(2000);
	 lcd_clrscr();
	 lcd_home();
	 
	 Timer1(true);
	 Timer2(false);
	 Converter(Init);
	 sei();
 }

void ModeControl()
{
	if (Button) Mode.btn = 0;
	{
		if (!Button) Mode.btn++;
		{
			if (Mode.btn == 1)
			{
				switch(Mode.mode)
				{
					case Waiting:
						Mode.mode = Record;
						Timer2(true);
						break;
					case Record:
						//Mode.mode = Comparsion;
						Mode.mode = Waiting;
						Timer2(false);
						break;
					case Comparsion:
						Mode.mode = Finished;
						break;
					default:
						Mode.mode = Finished;
						break;
				}
				
				Mode.changed = true;
			}
		}
	}
}

void Calculation()
{
	Convert.voltage = Convert.value*0.0048875855327468;
}

int main(void)
{
	Initialization("Softness");
	
	while(1)
	{	
		ModeControl();
		
		if (Convert.done)
		{
			Calculation();
			Converter(On);
			Convert.done = false;
		}
		
		if (MainTimer.ms1000)
		{
			DisplayPrint();
			if (Mode.mode == Waiting) LedInv;
			if (MainTimer.sec > 59) MainTimer.sec = 0;
			MainTimer.sec++;
			MainTimer.ms1000 = 0;
		}
	}
}