/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 * k = Tcount*0.028*Pi*60/100
 */ 

#define F_CPU	16000000L
				
#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Imp			Check(PORTB, PORTB0)	// control pulses of motor
#define ImpOn		High(PORTB, PORTB0)
#define ImpOff		Low(PORTB, PORTB0)
#define ImpInv		Inv(PORTB, PORTB0)

#define Fault		Check(PORTB, PORTB1)	// output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB1)
#define FaultOff	Low(PORTB, PORTB1)
#define FaultInv	Inv(PORTB, PORTB1)

#define Led			Check(PORTB, PORTB5)	// operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define TxEnable 	Check(PIND, PIND2)		// if connected every sec data transmit
#define Running		Check(PIND, PIND3)		// spindle run input
#define Aramid		Check(PIND, PIND4)		// aramid speed pulses input
#define Polyamide   Check(PIND, PIND5)		// polyamide speed pulses input
#define LcdEnable	Check(PIND, PIND6)		// 

#define Off				0
#define On				1
#define Init			2

#define Right	 		10
#define Left 			20
#define Locked			30

#define ArraySize		  64		// these parameters also should be positioned in ROM
#define StartDelay		  5			// delay to start measuring after spindle start
#define FaultDelay		  1200  	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			  0.005		// if ratio > range up then motor left
#define RangeDown		  -0.005
#define LeftStepDuration  4			// seconds	 sp5 - 4	   sp3 - 2	   rest 3
#define RightStepDuration 4			// seconds		sp5 - 4				    rest 3
#define PauseBetweenSteps 32		// seconds
#define Overfeed		  0			// factor to keep wrong assembling (for example if we need asm - 10%)

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <avr/wdt.h>

struct TimeControl
{
	unsigned int ms;
	bool handle;
} MainTimer = { 0, false };

struct Data
{
	unsigned int ovf;
	float Fa, Fp, d;
} Measure = { 0, 0, 0, 0 };

struct ModeControl
{
	unsigned int startDelay, faultDelay;
	bool fault, run;
} Mode = { 0, FaultDelay, false, false };

struct MotorControl
{
	unsigned int isDelay, isStep, operation;
	bool isFirstPulse;
} Motor = { 0, 0, Locked, true }; 

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
		High(TIMSK0, TOIE0);
		TCNT0 = 0;
		return;
	}
	
	TCCR0B = 0x00;
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
		TCNT1 = 0;
		return;
	}
	
	TCCR1B = 0x00;
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
	
	TCCR2B = 0x00;
	Low(TIMSK2,TOIE2);
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{	
	MainTimer.ms++;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.handle = true;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

void USART(unsigned short option)
{
	switch (option)
	{
		case On:
		UCSR0B |= (1 << TXEN0);
		break;
		case Off:
		UCSR0B |= (0 << TXEN0);
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
	static char d[8] = {}, a[8] = {}, p[8] = {}, buffer[32] = {}; 

	sprintf(a, "A%.1f$", Measure.Fa);
	sprintf(p, "P%.1f$", Measure.Fp);
	sprintf(d, "D%.3f", Measure.d);
	strcat(buffer, a);
	strcat(buffer, p);
	strcat(buffer, d);
	TxString(buffer);
	
	for (int i=0; i<32; i++) buffer[i] = 0;
}

float Average(float difference, bool isReset)
{
	static float values[ArraySize] = { 0 };
	static int index = 0;
	static float result = 0;
	
	if (isReset)
	{
		for (int i = 0; i<ArraySize; i++) values[i] = 0;
		index = 0;
		result = 0;
		return 0;
	}
	
	if (++index >= ArraySize) index = 0;
	
	result -= values[index];
	result += difference;
	values[index] = difference;
	
	return result / ArraySize;
}

void Calculation()
{	
	Measure.Fa = (float)TCNT0 + Measure.ovf*256;
	Measure.Fp = (float)TCNT1;
	Measure.d = Average(Overfeed - (1 - (Measure.Fa == 0 ? 1 : Measure.Fa) / (Measure.Fp == 0 ? 1 : Measure.Fp)), false);		
}

void Initialization()
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000010;
	PORTD = 0b00000011;
	
	Timer2(true);
	USART(Init);
	USART(On);
	sei();
	
	wdt_enable(WDTO_8S);
}

void StartOrStop()
{
	if (Running && !Mode.run)
	{
		FaultOff;
		Mode.run = true;
		Mode.startDelay = StartDelay;
		Mode.faultDelay = FaultDelay;
		Mode.fault = false;
		Timer0(true);
		Timer1(true);
	}
	
	if (!Running && Mode.run)
	{
		LedOff;
		ImpOff;
		Timer0(false);
		Timer1(false);
		Average(0, true);
		Measure.Fa = 0;
		Measure.Fp = 0;
		Measure.d = 0;
		Mode.run = false;
		Mode.fault = false;
		Mode.faultDelay = FaultDelay;
		Mode.startDelay = 0;
		Motor.operation = Locked;
	}
}			  

void Step3()
{
	ImpOn;
	
	if (Motor.operation == Right)
	{
		if (Motor.isFirstPulse)
		{
			_delay_ms(5);
			Motor.isFirstPulse = false;
			return;
		}
		
		_delay_ms(1);
		ImpOff;
		_delay_ms(5);
		return;
	}
	
	if (Motor.operation == Left)
	{
		_delay_ms(5);
		ImpOff;
		_delay_ms(1);
	}
}

void Step4()
{
	ImpOn;
	
	if (Motor.operation == Left)
	{
		if (Motor.isFirstPulse)
		{
			_delay_ms(5);
			Motor.isFirstPulse = false;
			return;
		}
		
		_delay_ms(1);
		ImpOff;
		_delay_ms(5);
		return;
	}
	
	if (Motor.operation == Right)
	{
		_delay_ms(5);
		ImpOff;
		_delay_ms(1);
	}
}

void Step5()
{
	ImpOn;
	
	if (Motor.operation == Right)
	{
		if (Motor.isFirstPulse)
		{
			_delay_ms(1);
			Motor.isFirstPulse = false;
			return;
		}
		
		_delay_us(600);
		ImpOff;
		_delay_ms(5);
		return;
	}
	
	if (Motor.operation == Left)
	{
		_delay_ms(5);
		ImpOff;
		_delay_ms(1);
	}
}

void Step()
{
	ImpOn;
	_delay_us(450);
	ImpOff;
	_delay_ms(5);
}

void Regulation()
{
	if (Motor.isStep) return;
	
	if ((Measure.d > RangeDown && Measure.d < RangeUp))
	{
		Mode.faultDelay = FaultDelay;
		Motor.operation = Locked;
		return;
	}
	
	if (Motor.isDelay) return;
	
	if (Measure.d >= RangeUp) 
	{
		Motor.operation = Left;
		Motor.isStep = LeftStepDuration;
		Motor.isFirstPulse = true; 
	}
	else 
	{
		Motor.operation = Right;
		Motor.isStep = RightStepDuration;
	}
}

void Process()
{
	if (Mode.run && !Mode.startDelay)
	{
		LedInv;
		
		Calculation();
		
		if (TxEnable) Transmit();
		
		if (Motor.isDelay > 0) Motor.isDelay--;
		
		if (Motor.isStep)
		{
			Motor.isStep--;
			if (!Motor.isStep) Motor.isDelay = PauseBetweenSteps;
		}
		
		Regulation();

		if (Motor.operation != Locked && Mode.faultDelay && !Mode.fault) Mode.faultDelay--;
		
		if (!Mode.faultDelay && !Mode.fault)
		{
			FaultOn;
			Mode.fault = true;
		}
	}
	
	if (Mode.run)
	{
		TCNT0 = 0;
		TCNT1 = 0;
		Measure.ovf = 0;
	}
}
							   					
int main()
{
	Initialization();
	
    while(1)
    {		
		if (MainTimer.handle)
		{	
			if (Mode.startDelay) Mode.startDelay--;
			
			StartOrStop();
			Process();
			
			MainTimer.handle = false;
		}
		
		if (Motor.isStep) Step5();
		
		wdt_reset();
    }
}