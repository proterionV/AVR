/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 */ 

#define F_CPU	16000000L
#define Spindle	10

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Aramid		Check(PIND, 4)
#define Polyamide   Check(PIND, 5)
#define Working		Check(PIND, 6)

#define Fault		Check(PORTB, 0)
#define FaultOn		High(PORTB, 0)
#define FaultOff	Low(PORTB, 0)
#define FaultInv	Inv(PORTB, 0)

#define Imp			Check(PORTB, 1)
#define ImpOn		High(PORTB, 1)
#define ImpOff		Low(PORTB, 1)
#define ImpInv		Inv(PORTB, 1)

#define Led			Check(PORTB, 5)
#define LedOn		High(PORTB, 5)
#define LedOff		Low(PORTB, 5)
#define LedInv		Inv(PORTB, 5)

#define Off				0
#define InternalCounter 1
#define ExternalCounter 2

#define Acceleration	11
#define Waiting			22
#define Process			33

#define Right	 		10
#define Left 			20
#define Stop			30 

#define AvgArraySize    35
#define IntervalR		9
#define IntervalL		4
#define AccelDelay		40
#define FaultDelay		1200
#define RangeUp			0.005
#define RangeDown		-0.005
#define Overfeed		0

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

volatile struct
{
	unsigned int ms16, ms992;
} MainTimer;

volatile struct
{
	unsigned int ovf;
	float Ua, Up;
} Measure;

volatile struct
{
	unsigned short operation;
	unsigned short current;
	unsigned int delay;
	unsigned int fuse;
	bool fault;
} Mode;

void Timer0(unsigned short mode)
{
	if (mode == ExternalCounter)
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

void Timer1(unsigned short mode)
{
	if (mode == ExternalCounter)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
}

void Timer2(unsigned short mode)
{
	if (mode == InternalCounter)
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
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT2 = 5;
}

float MovAvgAramid(float value)
{
	static unsigned short index = 0;
	static float array[AvgArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % AvgArraySize;
	
	return result/AvgArraySize;
}

float MovAvgPolyamide(float value)
{
	static unsigned short index = 0;
	static float array[AvgArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % AvgArraySize;
	
	return result/AvgArraySize;
}

void Calculation()
{
	Measure.Ua = MovAvgAramid(((255.f*Measure.ovf)+TCNT0)*0.001709568);
	Measure.Up = MovAvgPolyamide(TCNT1*0.003183264); 
}

void Initialization()
{
	DDRB = 0b11111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000001;
	PORTD = 0b00000011;
	
	MainTimer.ms16 = 0;
	MainTimer.ms992 = 0;
	
	Measure.Ua = 0;
	Measure.Up = 0;
	Measure.ovf = 0;
	
	Mode.fault = false;
	Mode.current = Waiting;
	Mode.fuse = FaultDelay;
	Mode.delay = AccelDelay;
	Mode.operation = Stop;
	
	FaultOn;
	_delay_ms(3000);
	FaultOff;
	
	Timer2(InternalCounter);
	sei();
}

void Control()
{
	if (Working)
	{
		if (Mode.current == Process) return;
		
		if (Mode.current == Waiting)
		{
			FaultOff;
			Mode.delay = AccelDelay;
			Mode.current = Acceleration;
			Mode.fuse = FaultDelay;
			Mode.fault = false;
			Timer0(ExternalCounter);
			Timer1(ExternalCounter);
			return;
		}
		
		if (Mode.current == Acceleration && !Mode.delay) Mode.current = Process;
		return;
	}
	
	if (Mode.current == Waiting) return;
	
	ImpOff;
	Timer0(Off);
	Timer1(Off);
	
	for (int i = 0; i<AvgArraySize; i++)
	{
		MovAvgAramid(0);
		MovAvgPolyamide(0);
	}
	
	TCNT0 = 0;
	TCNT1 = 0;
	Measure.Ua = 0;
	Measure.Up = 0;
	Measure.ovf = 0;
	Mode.current = Waiting;
}

void Step(short direction)
{
	ImpOn;
	if (direction == Left)  _delay_ms(1); // 1 ms
	if (direction == Right) _delay_ms(2); // 2 ms
	ImpOff;
	_delay_ms(1); // 1 ms
}

void Regulator()
{
	static float difference = 0, ratio = 0;
	
	if (Mode.current != Process)
	{
		if (Mode.operation == Stop) return;
		Mode.operation = Stop;
		return;
	}
	
	ratio = 1 - ((Measure.Ua == 0 ? 1 : Measure.Ua) / (Measure.Up == 0 ? 1 : Measure.Up));
	difference = Overfeed - ratio;
	
	if (difference > RangeDown && difference < RangeUp)
	{
		if (Mode.operation == Stop) return;
		if (Mode.fuse < FaultDelay) Mode.fuse = FaultDelay;
		Mode.operation = Stop;
		FaultOff;
		return;
	}
	
	if (difference >= RangeUp)
	{
		if (Mode.operation == Left)
		{
			Step(Left);
			return;
		}
		
		Step(Left);
		Mode.operation = Left;
	}
	else
	{
		if (Mode.operation == Right)
		{
			Step(Right);
			return;
		}
		
		Step(Right);
		Mode.operation = Right;
	}
}

int main(void)
{
	Initialization();
	
    while(1)
    {
		Control();
		Regulator();
		
        if (MainTimer.ms992)
        {	
			LedInv;
					
			if (Mode.current == Process || Mode.current == Acceleration)
			{
				Calculation();

				if (Mode.operation != Stop && Mode.fuse && !Mode.fault) Mode.fuse--;
				
				if (!Mode.fuse && !Mode.fault)
				{
					FaultOn;
					Mode.fault = true;
				}
			}
			
			if (Mode.delay) Mode.delay--;
			
			TCNT0 = 0;
			TCNT1 = 0;
			Measure.ovf = 0;
		    MainTimer.ms992 = 0;
        } 
    }
}