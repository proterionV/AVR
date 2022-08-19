/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 */ 

#define F_CPU	16000000L
#define Spindle	3					// order number device = order number of spindle, can use as address of device, it should be positioned in RAM
				
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

#define Led			Check(PORTB, PORTB5)	// operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define Running		Check(PIND, PIND3) // spindle run input
#define Aramid		Check(PIND, PIND4) // aramid speed pulses input
#define Polyamide   Check(PIND, PIND5) // polyamide speed pulses input

#define Off		 0
#define On		 1
#define Init	 2

#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define InternalCounter 1
#define ExternalCounter 2

#define Acceleration	11
#define Waiting			22
#define Process			33

#define Right	 		10
#define Left 			20
#define Stop			30

#define TxBufferSize	100

// these parameters also should be positioned in ROM
#define FilterFactor    0.1		// Size of array to calculate average
#define HighIntervalR	1		// count 16 ms period of generation to right rotation
#define LowIntervalR	-0		// count 16 ms period of prohibited generation to right
#define HighIntervalL	1		// count 16 ms period of generation to left rotation
#define LowIntervalL 	-0		// count 16 ms period of prohibited generation to left
#define AccelDelay		40		// delay to start measuring after spindle start
#define FaultDelay		1200	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			0.007	// if ratio > range up then motor left
#define RangeDown		-0.007	// if ratio < range up then motor right; between = stop
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

void USART(unsigned short option)
{
	switch (option)
	{
		case TxOn:
			UCSR0B |= (1 << TXEN0);
			break;
		case TxOff:
			UCSR0B |= (0 << TXEN0);
			break;
		case RxOn:
			UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
			break;
		case RxOff:
			UCSR0B = (1 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			break;
		case On:
			UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
			break;
		case Off:
			UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			break;
		default:
			UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
			UBRR0  =  3;
			break;
	}
}

void TxChar(unsigned char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

void Transmit()
{
	static char buffer[TxBufferSize] = { 0 };
	static char A[20], P[20];
	
	memset(buffer, 0, TxBufferSize);
	
	sprintf(A, "$A%.1f ", Measure.Ua);
	sprintf(P, "$P%.1f ", Measure.Up);
	strcat(buffer, A);
	strcat(buffer, P);
	
	TxString(buffer);
}

float MovAvgAramid(float value, bool reset)
{
	static float result = 0;
	
	if (reset) result = 0;
	
	result += (value - result) * FilterFactor;
	
	return result;
}

float MovAvgPolyamide(float value, bool reset)
{
	static float result = 0;
	
	if (reset) result = 0;
	
	result += (value - result) * FilterFactor;
	
	return result;
}

void Calculation(){					 // 0.087964*1.008*60/100
	Measure.Ua = MovAvgAramid(((255.f*Measure.ovf)+TCNT0+1)*0.0532009867, false);
	Measure.Up = MovAvgPolyamide(TCNT1*0.0532009867, false); 
}

void Initialization()
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000010;
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
	
	MovAvgAramid(0, true);
	MovAvgPolyamide(0, true);
	
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
			_delay_ms(7);
			if (!MainTimer.interval) MainTimer.interval = HighIntervalR;
			break;
		case Left:
			ImpOn;
			_delay_ms(1);
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
			 FaultOff;
			 LedOn;
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
	 
	 MovAvgAramid(0, true);
	 MovAvgPolyamide(0, true);
	 
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
			if (!Led) LedOn;
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
		if (Running && !Mode.run) Mode.run = true;
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