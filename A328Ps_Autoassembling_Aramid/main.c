/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 * k = Tcount*0.028*Pi*60/100
 */ 

#define F_CPU	16000000L
#define Spindle	3				// order number device = order number of spindle, can use as address of device, it should be positioned in RAM
				
#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Imp			Check(PORTB, PORTB0)	// control pulses of motor
#define ImpOn		High(PORTB, PORTB0)
#define ImpOff		Low(PORTB, PORTB0)
#define ImpInv		Inv(PORTB, PORTB0)

#define Fault		Check(PORTB, PORTB1) // output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB1)
#define FaultOff	Low(PORTB, PORTB1)
#define FaultInv	Inv(PORTB, PORTB1)

#define Led			Check(PORTB, PORTB5) // operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define Running		Check(PIND, PIND3)  // spindle run input
#define Aramid		Check(PIND, PIND4)  // aramid speed pulses input
#define Polyamide   Check(PIND, PIND5)  // polyamide speed pulses input

#define Off		 0
#define On		 1
#define Init	 2

#define Right	 		10
#define Left 			20
#define Locked			30
								// these parameters also should be positioned in ROM
#define FilterFactor    0.1		// Size of array to calculate average
#define StartDelay		10		// delay to start measuring after spindle start
#define FaultDelay		30  	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			0.01	// if ratio > range up then motor left
#define RangeDown		-0.01	// if ratio < range up then motor right; between = stop
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

struct TimeControl
{
	unsigned short ms;
	bool s;
} MainTimer;

struct Data
{
	unsigned int ovf;
	float Ua;
	float Up;
} Measure;

struct ModeControl
{
	unsigned short startDelay;
	unsigned int faultDelay;
	bool fault;
	bool run;
} Mode;

struct SignalControl
{
	bool ready;
	bool permission;
} Signal;

struct MotorControl
{
	bool isStep;
	unsigned short operation;
} Motor; 

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
	MainTimer.ms++;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.s = true;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

void Calculation()
{		
	Measure.Ua += ((256.f*Measure.ovf+TCNT0)*0.05277875658 - Measure.Ua) * FilterFactor;		 
	Measure.Up += (TCNT1*0.05277875658 - Measure.Up) * FilterFactor; 	   
}

void Initialization()
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000010;
	PORTD = 0b00110011;
	
	MainTimer.ms = 0;
	MainTimer.s = false;
	
	Measure.Ua = 0;
	Measure.Up = 0;
	Measure.ovf = 0;
	
	Mode.run = false;
	Mode.fault = false;
	Mode.faultDelay = FaultDelay;
	Mode.startDelay = 0;
	Motor.operation = Locked;
	
	Signal.ready = false;
	Signal.permission = false;
	
	Timer2(true);
	sei();
}

void Step()
{
		
}

void Regulation()
{
	static float difference = 0, ratio = 0;
	
	ratio = 1 - ((Measure.Ua == 0 ? 1 : Measure.Ua) / (Measure.Up == 0 ? 1 : Measure.Up));
	difference = Overfeed - ratio;
	
	if (Motor.isStep) return;
	
	if (difference > RangeDown && difference < RangeUp)
	{
		if (Mode.faultDelay < FaultDelay) Mode.faultDelay = FaultDelay;
		return;
	}
	
	if (difference >= RangeUp) Motor.operation = Left;
	else Motor.operation = Right;
	Motor.isStep = true;
}
							   
void HandleS()
{
	if (MainTimer.s)
	{	
		if (Mode.startDelay) Mode.startDelay--;
				
		if (Mode.run && !Mode.startDelay)
		{
			LedInv;
			
			if (Motor.isStep)
			{
				Motor.isStep = false;
				Motor.operation = Locked;
			}
			
			Calculation();
			Regulation();

			if (Motor.operation != Locked && Mode.faultDelay && !Mode.fault) Mode.faultDelay--;
			
			if (!Mode.faultDelay && !Mode.fault)
			{
				FaultOn;
				Mode.fault = true;
			}
			
			TCNT0 = 0;
			TCNT1 = 0;
			Measure.ovf = 0;
		}
		
		if (Running && !Mode.run)
		{
			FaultOff;
			Mode.run = true;
			Mode.startDelay = StartDelay;
			Mode.faultDelay = FaultDelay;
			Mode.fault = false;
			Timer0(true);
			Timer1(true);
			return;
		}
		 
		if (!Running && Mode.run)
		{
			ImpOff;
			Timer0(Off);
			Timer1(Off);
			Measure.Ua = 0;
			Measure.Up = 0;
			Measure.ovf = 0;
			Mode.run = false;
			Mode.fault = false;
			Mode.faultDelay = FaultDelay;
			Mode.startDelay = 0;
			Motor.operation = Locked;
		}
		
		MainTimer.s = false;
	}	
}

int main(void)
{
	Initialization();
	
    while(1)
    {	
        HandleS();		   // function of handling interrupt every second
		if (Motor.isStep) Step();
    }
}