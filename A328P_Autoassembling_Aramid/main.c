/*
 * main.c
 *
 * Created: 3/14/2022 3:07:47 PM
 *  Author: igor.abramov
 */ 

/*
  Frequency comparer with counter inputs
  IR Receiver with continue receive will be down after 530 ms
*/

#define F_CPU	16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led			Check(PORTB, 5)
#define LedOn		High(PORTB, 5)
#define LedOff		Low(PORTB, 5)
#define LedInv		Inv(PORTB, 5)

#define Pulse		Check(PORTD, 7)
#define PulseOn		High(PORTD, 7)
#define PulseOff	Low(PORTD, 7)
#define PulseInv	Inv(PORTD, 7)

#define RightOn		(!Check(PIND, 2)) 
#define LeftOn		(!Check(PIND, 3))
#define Active		(!Check(PIND, 6))

#define Right	 		10
#define Left 			20
#define Stop			30
#define Short			40

#define Acceleration	1
#define Deceleration	2
#define Waiting			3
#define Process			4

#define AccelDelay	10
#define DecelDelay	5

#define MovAvgSize	100

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
	bool moving, done;
} MainTimer;

volatile struct
{
	unsigned int pulseA, pulseP, ovf;
	float Fa, Fp;
} Measure;

volatile struct
{
	unsigned short mode;
	unsigned short count;
	bool active;		
} Mode;

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
	
	if (MainTimer.done) 
	{
		MainTimer.moving = false;
		MainTimer.done = false;	
	}
	
	if (MainTimer.moving) MainTimer.done++; 
	
	TCNT2 = 5;
}

void delay_ms(int ms)
{
	while (0 < ms)
	{
		_delay_ms(1);
		--ms;
	}
}

void delay_us(int us)
{
	while (0 < us)
	{
		_delay_us(1);
		--us;
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
	
	if (Mode.mode == Waiting) return;
	 
	EraseUnits(0, 0, 3, Measure.Fa);
	sprintf(frequencyAramid, "%.2f", Measure.Fa < 0 ? 0 : Measure.Fa);
	lcd_puts(frequencyAramid);
	
	EraseUnits(0, 1, 3, Measure.Fp);
	sprintf(frequencyPolyamide, "%.2f", Measure.Fp < 0 ? 0 : Measure.Fp);
	lcd_puts(frequencyPolyamide);
}

void Initialization()
 {
	 DDRB = 0b00111111;
	 PORTB = 0b00000111;
	 
	 DDRC = 0b00111111;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b10000000;
	 PORTD = 0b01111111;
	 
	 PulseOff;
	 
	 Mode.mode = Waiting;
	 Mode.active = false;
	 Mode.count = 0;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_clrscr();
	 lcd_home();
	 
	 lcd_puts("0.0");
	 lcd_clrline(9, 0);
	 lcd_puts("Waiting");
	 
	 lcd_gotoxy(0, 1);
	 lcd_puts("0.0");
	 lcd_clrline(9, 1);
	 lcd_puts("Stop");
	 
	 MovAvgAramid(0, true);
	 MovAvgPolyamide(0, true);
	 
	 Timer0(false);
	 Timer1(false);
	 Timer2(true);
	 sei();	 
 }

void Calculation()
{
	// F = (k / q) * L * t = X m/s
	// k = 1000 ms / 160 ms = 6.25 (measure during 160 ms) 
	// q = 50 imp/rev for both impellers
	// La aramid roll D = 0.027 m, L = 0.0848 m (measured)
	// Lp polyamide roll D = 0.0512 m, L = 0.161 m (calculated)	
	// t = 60 seconds
	// in the same F and original sizes asm = -20
	// Lp experimantal v1 = 0.1570 // 1.1775 // asm = +4
	// Lp experimental v3 = 0.1572 // 1.1790 // asm =  ?
	// Lp experimental v3 = 0.1575 // 1,1812 // asm = -2
	// Lp experimantal v2 = 0.1580 // 1.1850 // asm = -6
																		   
	Measure.Fa = MovAvgAramid(((255.f*Measure.ovf)+TCNT0)*0.636, false); // (6.25/50.f * 0.0848 * 60 = 0.636 
	Measure.Fp = MovAvgPolyamide(TCNT1*1.1790, false); // 50 imp/rev // (6.25/50.f * 0.161 * 60 = 1.2075 
	
	TCNT0 = 0;
	TCNT1 = 0;
}

void Step(unsigned short direction)
{
	if (MainTimer.moving) return;
	
	switch (direction)
	{
		case Right:
			PulseOn;
			_delay_us(200);
			PulseOff;
			_delay_ms(70);
			break;
		case Left:
			PulseOn;
			_delay_ms(70);
			PulseOff;
			break;
		default:
			PulseOff;
			break;	
	}	 
	
	MainTimer.moving = true;
}

void Manual()
{
	static unsigned short key = Stop;
	
	if (!(RightOn | LeftOn)) 
	{
		if (key == Stop) return;
		
		lcd_clrline(9, 1);
		lcd_puts("Stop");
		key = Stop;
		return;
	}
	
	if (RightOn & LeftOn) 
	{
		if (key == Short) return;
		
		lcd_clrline(9, 1);
		lcd_puts("Short");
		key = Short;
		return;
	}
	
	if (RightOn) 
	{ 
		if (key == Right) 
		{
			Step(Right);
			return;
		}
		
		Step(Right);
		lcd_clrline(9, 1);
		lcd_puts("Right");
		key = Right;
		return; 
	}
	
	if (LeftOn) 
	{
		if (key == Left)
		{
			Step(Left);
			return;
		}
		
		Step(Left);
		lcd_clrline(9, 1);
		lcd_puts("Left");
		key = Left;
		return;
	}
}

void ModeControl()
{
	if (Active)
	{
		if (Mode.active) 
		{
			if (Mode.mode == Acceleration || Mode.mode == Deceleration)
			{
				if (Mode.count > 0 && Mode.mode == Acceleration) return;
				
				LedOn;
				Mode.mode = Process;
				Mode.count = 0;
				Timer0(true);
				Timer1(true);
				lcd_clrline(9, 0);
				lcd_puts("Process"); 
				return;
			}
			
			return;				
		}
		
		Mode.active = true;
		Mode.count = AccelDelay;
		Mode.mode = Acceleration;
		lcd_clrline(9, 0);
		lcd_puts("Acceler");
		return;
	}
	
	if (Mode.active)
	{
		if (Mode.mode == Process)
		{
			Mode.count = DecelDelay;
			Mode.mode = Deceleration;
			lcd_clrline(9, 0);
			lcd_puts("Deceler");
			return;	
		}
		
		if (Mode.mode == Deceleration || Mode.mode == Acceleration)
		{
			if (Mode.count > 0 && Mode.mode == Deceleration) return;
			
			LedOff;
			PulseOff;
			Mode.active = false;
			Mode.mode = Waiting;
			Mode.count = 0;
			
			lcd_clrscr();
			lcd_home();
			lcd_puts("0.0");
			lcd_clrline(9, 0);
			lcd_puts("Waiting");
			
			lcd_gotoxy(0, 1);
			lcd_puts("0.0");
			lcd_clrline(9, 1);
			lcd_puts("Stop");
			
			MovAvgAramid(0, true);
			MovAvgPolyamide(0, true);
			
			Timer0(false);
			Timer1(false);
			return;
		}
	}
}

void Regulator()
{
	static float difference = 0;
	
	if (RightOn || LeftOn) return;
	
	difference = Measure.Fa - Measure.Fp;
	
	if (difference >= -0.15 && difference <= 0.15) return;
	
	if (difference > 0.15) Step(Left); else Step(Right);
}

int main(void)
{
	Initialization();
	
	while(1)
	{	
	 	//Manual();
		ModeControl();
		if (Mode.mode == Process) Regulator();

		if (MainTimer.ms160)
		{
			if (Mode.mode == Process) 
			{
				Calculation();
			}
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			DisplayPrint();
			MainTimer.ms992 = 0;
			if (Mode.count > 0) Mode.count--;
			if (Mode.mode == Acceleration || Mode.mode == Deceleration) LedInv;
		}
	}
}