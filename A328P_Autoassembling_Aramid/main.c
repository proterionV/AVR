/*
 * main.c
 *
 * Created: 3/14/2022 3:07:47 PM
 * Author: igor.abramov
 */ 

/*
  Frequency comparer with counter inputs
  IR Receiver with continue receive will be down after 530 ms
  
  // F = (k / q) * L * t = X m/s
  // k = 1000 ms / 160 ms = 6.25 (measure during 160 ms)
  // q = 50 imp/rev for both impellers
  // La aramid roll D = 0.027 m, L = 0.0848 m (measured)
  // Lp polyamide roll D = 0.0512 m, L = 0.161 m (calculated)
  // Lp experimantal = 0.1579
  // t = 60 seconds
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

#define Stop		Check(PORTB, 3)
#define StopOn		High(PORTB, 3)
#define StopOff		Low(PORTB, 3)
#define StopInv		Inv(PORTB, 3)

#define Pulse		Check(PORTD, 7)
#define PulseOn		High(PORTD, 7)
#define PulseOff	Low(PORTD, 7)
#define PulseInv	Inv(PORTD, 7)

#define RightOn		(!Check(PIND, 2)) 
#define LeftOn		(!Check(PIND, 3))
#define Active		(!Check(PIND, 6))

#define Right	 		10
#define Left 			20
#define OK				30
#define Short			40

#define Acceleration	11
#define Waiting			22
#define Process			33

#define IntervalR		11
#define IntervalL		7
#define AccelDelay		40
#define AlarmDelay		600
#define RangeUp			0.005
#define RangeDown		-0.005
#define AArraySize		35
#define PArraySize		35
#define TArraySize		10
#define HArraySize		10
#define Overfeed		0

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <util/delay.h>
#include "lcd/lcd.h"

volatile struct
{
	volatile unsigned int interval, ms16, ms992;
} MainTimer;

volatile struct
{
	unsigned int ovf;
	float Fa, Fp;
} Measure;

volatile struct
{
	unsigned short current;
	unsigned int count, alarmCount;
	unsigned short key;
	bool alarm, run;		
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

float MovAvgAramid(float value)
{
	static unsigned short index = 0;
	static float array[AArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % AArraySize;
	
	return result/AArraySize;
}

float MovAvgPolyamide(float value)
{
	static unsigned short index = 0;
	static float array[PArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % PArraySize;
	
	return result/PArraySize;
}

void DisplayPrint()
{
	static char Va[20] = { 0 }, Vp[20] = { 0 };
	 
	EraseUnits(0, 0, 3, Measure.Fa);
	sprintf(Va, "%.2f", Measure.Fa < 0 ? 0 : Measure.Fa);
	lcd_puts(Va);
	
	EraseUnits(0, 1, 3, Measure.Fp);
	sprintf(Vp, "%.2f", Measure.Fp < 0 ? 0 : Measure.Fp);
	lcd_puts(Vp);
	
	if (Mode.alarm) 
	{
		lcd_clrline(15, 1);
		lcd_puts("A");
	}
}

void Initialization()
 {
	 DDRB = 0b00101111;
	 PORTB = 0b00000000;
	 
	 DDRC = 0b00111100;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b10000000;
	 PORTD = 0b01111111;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_clrscr();
	 lcd_home();
	 
	 PulseOff;
	 Mode.current = Waiting;
	 Mode.key = OK;
	 Mode.count = 0;
	 Mode.alarmCount = AlarmDelay;
	 Mode.alarm = false;
	 
	 Measure.Fa = 0;
	 Measure.Fp = 0;
	 
	 DisplayPrint();
	 Timer2(true);
	 sei();
	 StopOff;
 }

void Calculation()
{
	float a, p = 0;
	
	a = ((255.f*Measure.ovf)+TCNT0)*0.001709568;
	p =	TCNT1*0.003183264;
	Measure.Fa = MovAvgAramid(a*60); // (1.2096774 * 0.0848 = 0.10258
	Measure.Fp = MovAvgPolyamide(p*60); // 50 imp/rev // (1.2096774 * 0.1579 = 0.19052
}

void Step(unsigned short direction)
{
	if (MainTimer.interval) return;
	
	switch (direction)
	{
		case Right:
			PulseOn;
			_delay_us(200);
			PulseOff;
			_delay_ms(70);
			MainTimer.interval = IntervalR;
			break;
		case Left:
			PulseOn;
			_delay_ms(70);
			PulseOff;
			MainTimer.interval = IntervalL;
			break;
		default:
			PulseOff;
			break;	
	}	 
}

void Step2(short direction)
{
	PulseOn;
	if (direction == Left)  _delay_us(600);
	if (direction == Right) _delay_ms(4);
	PulseOff;
	_delay_ms(4);
}

void Manual()
{
	if (!(RightOn | LeftOn)) 
	{
		if (Mode.key == OK || Mode.current == Process) return;
		
		lcd_clrline(9, 0);
		lcd_puts("OK");
		Mode.key = OK;
		return;
	}
	
	if (RightOn & LeftOn) 
	{
		if (Mode.key == Short) return;
		
		lcd_clrline(9, 0);
		lcd_puts("Short");
		Mode.key = Short;
		return;
	}
	
	if (RightOn) 
	{ 
		if (Mode.key == Right) 
		{
			Step2(Right);
			return;
		}
		
		Step2(Right);
		lcd_clrline(9, 0);
		lcd_puts("Right");
		Mode.key = Right;
		return; 
	}
	
	if (LeftOn) 
	{
		if (Mode.key == Left)
		{
			Step2(Left);
			return;
		}
		
		Step2(Left);
		lcd_clrline(9, 0);
		lcd_puts("Left");
		Mode.key = Left;
		return;
	}
}

void ModeControl()
{	
	if (Active)
	{
		if (Mode.current == Waiting)
		{
			Mode.count = AccelDelay;
			Mode.current = Acceleration;
			Mode.alarmCount = AlarmDelay;
			Mode.alarm = false;
			Timer0(true);
			Timer1(true);
			return;
		}
		
		if (Mode.current == Acceleration && !Mode.count) Mode.current = Process;		
		return;
	}
	
	if (Mode.current == Waiting) return;
	
	PulseOff;
	Timer0(false);
	Timer1(false);
	
	for (int i = 0; i<40; i++) 
	{
		MovAvgAramid(0);
		MovAvgPolyamide(0);
	}
	
	TCNT0 = 0;
	TCNT1 = 0;
	Mode.current = Waiting;
	
	Measure.Fa = 0;
	Measure.Fp = 0;
	Measure.ovf = 0;
	DisplayPrint();	
	StopOff;
}

void Regulator()
{
	static float difference = 0, ratio = 0;
	
	//if (RightOn || LeftOn) return; 
	
	if (Mode.current == Waiting || Mode.current == Acceleration) 
	{
		if (Mode.key == OK) return;
		//lcd_clrline(9, 0);
		//lcd_puts("OK");
		Mode.key = OK;
		return;
	}
	
	ratio = 1 - ((Measure.Fa == 0 ? 1 : Measure.Fa) / (Measure.Fp == 0 ? 1 : Measure.Fp));
	difference = Overfeed - ratio;
	
	if (difference > RangeDown && difference < RangeUp) 
	{
		if (Mode.key == OK) return;
		if (Mode.alarmCount < AlarmDelay) Mode.alarmCount = AlarmDelay;
		//lcd_clrline(9, 0);
		//lcd_puts("OK");
		Mode.key = OK;
		return;
	}
	
	if (difference >= RangeUp) 
	{
		if (Mode.key == Left)
		{
			Step(Left);
			return;
		}
		
		Step(Left);
		//lcd_clrline(9, 0);
		//lcd_puts("Left");
		Mode.key = Left;
	}
	else 
	{
		if (Mode.key == Right)
		{
			Step(Right);
			return;
		}
		
		Step(Right);
		//lcd_clrline(9, 0);
		//lcd_puts("Right");
		Mode.key = Right;
	}
}

int main(void)
{		
	Initialization();
	
	while(1)
	{
		ModeControl();
		
		if (Mode.current == Process) Regulator();
		
		if (MainTimer.ms992)
		{	
			LedInv;
			
			if (Mode.current == Process || Mode.current == Acceleration)
			{
				Calculation();
				DisplayPrint();

				if (Mode.key != OK && Mode.alarmCount > 0 && !Mode.alarm) Mode.alarmCount--;
				if (!Mode.alarmCount && !Mode.alarm)
				{
					StopOn;
					Mode.alarm = true;
				}
			}
			
			if (Mode.count) Mode.count--;
			
			TCNT0 = 0;
			TCNT1 = 0;
			Measure.ovf = 0;
			MainTimer.ms992 = 0;
		}
	}
}