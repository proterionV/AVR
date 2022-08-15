/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 */ 

#define F_CPU	16000000L
#define Spindle	7		  // order number device = order number of spindle, can use as address of device, it should be positioned in RAM

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Fault		Check(PORTB, 0) // output for open contact of yarn brake
#define FaultOn		High(PORTB, 0)
#define FaultOff	Low(PORTB, 0)
#define FaultInv	Inv(PORTB, 0)

#define Led			Check(PORTB, 5)	// operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, 5)
#define LedOff		Low(PORTB, 5)
#define LedInv		Inv(PORTB, 5)

#define Imp			Check(PORTC, 0)	// control pulses of motor
#define ImpOn		High(PORTC, 0)
#define ImpOff		Low(PORTC, 0)
#define ImpInv		Inv(PORTC, 0)
 
#define Aramid		Check(PIND, 4) // aramid speed pulses input
#define Polyamide   Check(PIND, 5) // polyamide speed pulses input
#define Running		Check(PIND, 6) // spindle run input

#define Off				0
#define InternalCounter 1
#define ExternalCounter 2

#define Acceleration	11
#define Waiting			22
#define Process			33

#define Right	 		10
#define Left 			20
#define Stop			30

// these parameters also should be positioned in ROM
#define AvgArraySize    35		// Size of array to calculate average
#define HighIntervalR	1		// count 16 ms period of generation to right rotation
#define LowIntervalR	-0		// count 16 ms period of prohibited generation to right
#define HighIntervalL	1		// count 16 ms period of generation to left rotation
#define LowIntervalL 	-0		// count 16 ms period of prohibited generation to left
#define AccelDelay		40		// delay to start measuring after spindle start
#define FaultDelay		1200	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			0.005	// if ratio > range up then motor left
#define RangeDown		-0.005	// if ratio < range up then motor right; between = stop
#define Overfeed		0		// factor to keep wrong assembling (for example if we need asm - 10)

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
	unsigned short ms16;
	signed short interval;
	bool interrupt16;
	bool interrupt992;
} MainTimer;

volatile struct
{
	unsigned int ovf;
	float Ua;
	float Up;
} Measure;

volatile struct
{
	unsigned short operation;
	unsigned short current;
	unsigned int delay;
	unsigned int fuse;
	bool fault;
	bool run;
} Mode;

volatile struct
{
	bool ready;
	bool permission;	
} Signal;

void Timer0(unsigned short func)
{			  
	if (func == ExternalCounter)
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

void Timer1(unsigned short func)
{
	if (func == ExternalCounter)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
}

void Timer2(unsigned short func)
{
	if (func == InternalCounter)
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
	MainTimer.interrupt16 = true;

	if (MainTimer.ms16 >= 62)
	{
		MainTimer.interrupt992 = true;
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
	PORTD = 0b00110011;
	
	MainTimer.ms16 = 0;
	MainTimer.interval = 0;
	MainTimer.interrupt16 = false;
	MainTimer.interrupt992 = false;
	
	Measure.Ua = 0;
	Measure.Up = 0;
	Measure.ovf = 0;
	
	Mode.run = false;
	Mode.fault = false;
	Mode.current = Waiting;
	Mode.fuse = FaultDelay;
	Mode.delay = 0;
	Mode.operation = Stop;
	
	Signal.ready = false;
	Signal.permission = false;
	
	Timer2(InternalCounter);
	sei();
}

void Step(short direction)
{
	if (!Signal.permission || MainTimer.interval < 0) return;
	
	switch (direction)
	{
		case Right:
			ImpOn;
			_delay_ms(1);
			if (!MainTimer.interval) MainTimer.interval = HighIntervalR;
			break;
		case Left:
			ImpOn;
			_delay_ms(5);
			if (!MainTimer.interval) MainTimer.interval = HighIntervalL;
			break;
		default:
			ImpOff;
			break;
	}
	
	ImpOff;
	_delay_ms(5);
}

void Control()
 {
	 if (Mode.run)
	 {
		 if (Mode.current == Process) return;
		 
		 if (Mode.current == Waiting)
		 {
			 LedOn;
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
		Signal.ready = false;
		Signal.permission = false;
		MainTimer.interval = 0;
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
		
		Signal.ready = true;
		Mode.operation = Left;
	}
	else
	{
		if (Mode.operation == Right)
		{
			Step(Right);
			return;
		}
		
		Signal.ready = true;
		Mode.operation = Right;
	}
}

void InterruptMS16()
{
	if (MainTimer.interrupt16)
	{
		MainTimer.interrupt16 = false;
		if (Signal.permission)
		{
			if (MainTimer.interval < 0) MainTimer.interval++;
			
			if (MainTimer.interval > 0)
			{
				MainTimer.interval--;
				if (!MainTimer.interval)
				{
					if (Mode.operation == Right) MainTimer.interval = LowIntervalR;
					if (Mode.operation == Left) MainTimer.interval = LowIntervalL;
				}
			}
		}
		
		if (!Signal.permission && Signal.ready) Signal.permission = true;
	}	
}

void InterruptMS992()
{
	if (MainTimer.interrupt992)
	{			
		if (Mode.current == Process || Mode.current == Acceleration)
		{
			Calculation();

			if (Mode.operation != Stop && Mode.fuse && !Mode.fault) Mode.fuse--;
			
			if (!Mode.fuse && !Mode.fault)
			{
				FaultOn;
				Mode.fault = true;
			}
			
			TCNT0 = 0;
			TCNT1 = 0;
			Measure.ovf = 0;
		}
		else LedInv;
		
		if (Mode.delay) Mode.delay--;
		if (Running && !Mode.run) { LedOn; Mode.run == true; }
		if (!Running && Mode.run) Mode.run = false;
		MainTimer.interrupt992 = false;
	}	
};

int main(void)
{
	Initialization();
	
    while(1)
    {
		Control();		   // control current mode: waiting, acceleration, process
		Regulator();	   // ratio calculate, motor direction control: right, left, stop
		InterruptMS16();   // function of handling interrupt every 16 ms
        InterruptMS992();  // function of handling interrupt every 992 ms
    }
}