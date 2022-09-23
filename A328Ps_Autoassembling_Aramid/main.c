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

#define Off				0
#define On				1
#define Init			2

#define Right	 		10
#define Left 			20
#define Locked			30
								// these parameters also should be positioned in ROM
#define FilterFactor      0.09	// Size of array to calculate average
#define StartDelay		  10	// delay to start measuring after spindle start
#define FaultDelay		  900  	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			  0.01	// if ratio > range up then motor left
#define RangeDown		  -0.01	// if ratio < range up then motor right; between = stop
#define Overfeed		  0		// factor to keep wrong assembling (for example if we need asm - 10)
#define	InstantRangeUp	  0.02
#define InstantRangeDown  -0.02
#define LeftStepDuration  3		 // inv 1
#define RightStepDuration 3		 // in 2
#define PauseBetweenSteps 2		 // inv 5

//#define LeftStepDuration  1		 // inv 1
//#define RightStepDuration 2		 // in 2
//#define PauseBetweenSteps 5		 // inv 5

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
	bool ms100, s;
} MainTimer;

struct Data
{
	unsigned short ovf;
	unsigned int Fa;
	unsigned int Fp;
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
	bool instantDifferenceIsOver;
} Signal;

struct MotorControl
{
	unsigned short isDelay;
	unsigned short isStep;
	unsigned short operation;
} Motor; 

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
		High(TIMSK0, TOIE0);
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

	if (MainTimer.ms % 100 == 0) MainTimer.ms100 = true;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.s = true;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

ISR(ANALOG_COMP_vect)
{
	Measure.Fp++;
}

void Comparator(unsigned int option)
{
	switch(option)
	{
		case On:
			High(ACSR, ACIE);
			break;
		case Off:
			Low(ACSR, ACIE);
		default:
			Low(ADCSRA, ADEN);
			Low(ADCSRB, ACME);
			Low(ACSR, ACI);
			High(ACSR, ACBG);
			Low(ACSR, ACIE);
			High(ACSR, ACIS1);
			break;
	}
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
	static char fa[20], fp[20];
	static char buffer[100];
		
	memset(buffer, 0, 100);
	
	sprintf(fa, "A%.2f$", Measure.Ua);
	sprintf(fp, "P%.2f$", Measure.Up);
	strcat(buffer, fa);
	strcat(buffer, fp);
	TxString(buffer);
}

float KalmanAramid(float aramidFrequecy, bool reset)
{
	static float measureVariation = 90, estimateVariation = 1, speedVariation = 0.02;
	static float CurrentEstimate = 0;
	static float LastEstimate = 0;
	static float Gain = 0;
	
	if (reset)
	{
		CurrentEstimate = 0;
		LastEstimate = 0;
		Gain = 0;
	}
	
	Gain = estimateVariation / (estimateVariation + measureVariation);
	CurrentEstimate = LastEstimate + Gain * (aramidFrequecy - LastEstimate);
	estimateVariation = (1.0 - Gain) * estimateVariation + fabs(LastEstimate - CurrentEstimate) * speedVariation;
	LastEstimate = CurrentEstimate;
	return CurrentEstimate;	
}

float KalmanPolyamide(float polyamideFrequency, bool reset)
{
	static float measureVariation = 90, estimateVariation = 1, speedVariation = 0.02;
	static float CurrentEstimate = 0;
	static float LastEstimate = 0;
	static float Gain = 0;
	
	if (reset)
	{
		CurrentEstimate = 0;
		LastEstimate = 0;
		Gain = 0;
	}
	
	Gain = estimateVariation / (estimateVariation + measureVariation);
	CurrentEstimate = LastEstimate + Gain * (polyamideFrequency - LastEstimate);
	estimateVariation = (1.0 - Gain) * estimateVariation + fabs(LastEstimate - CurrentEstimate) * speedVariation;
	LastEstimate = CurrentEstimate;
	return CurrentEstimate;	
}

void Calculation()
{	
	static float speedA = 0, speedP = 0;
	
	speedA = KalmanAramid(TCNT1*10, false);
	speedP = KalmanPolyamide((TCNT0+Measure.ovf*256)*10, false);
	//speedP = KalmanPolyamide(Measure.Fp*5, false);
	
	Measure.Ua = speedA*0.05277875658;
	Measure.Up = speedP*0.05277875658;		
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
	Measure.Fp = 0;
	
	Mode.run = false;
	Mode.fault = false;
	Mode.faultDelay = FaultDelay;
	Mode.startDelay = 0;
	Motor.operation = Locked;
	
	Signal.instantDifferenceIsOver = false;
	
	KalmanAramid(0, true);
	KalmanPolyamide(0, true);
	
	Timer2(true);
	//Comparator(Init);
	USART(Init);
	USART(On);
	sei();
}

void Step()
{
	ImpOn;
	
	 //sp1
	if (Motor.operation == Right) _delay_us(500);		// for non-inverted circuit 500 us, inverted - 1 ms
	if (Motor.operation == Left) _delay_ms(5);		// for non-inverted circuit 5 ms, inverted - 7 ms
	
	// rest
	//if (Motor.operation == Left) _delay_ms(1);		// for non-inverted circuit 500 us, inverted - 1 ms
	//if (Motor.operation == Right) _delay_ms(5);		// for non-inverted circuit 500 us, inverted - 1 ms

	
	ImpOff;
	_delay_ms(5); // both
}

void Regulation()
{
	static float difference = 0, ratio = 0;
	
	ratio = 1 - ((Measure.Ua == 0 ? 1 : Measure.Ua) / (Measure.Up == 0 ? 1 : Measure.Up));
	difference = Overfeed - ratio;
	
	if ((difference > RangeDown && difference < RangeUp))
	{
		Mode.faultDelay = FaultDelay;
		Motor.operation = Locked;
		Motor.isStep = 0;
		Motor.isDelay = false;
		return;
	}
	
	if (Motor.isStep || Motor.isDelay) return;
	
	if (difference >= RangeUp) 
	{
		Motor.operation = Left;
		Motor.isStep = LeftStepDuration;
	}
	else 
	{
		Motor.operation = Right;
		Motor.isStep = RightStepDuration;
	}
}
							   					
int main(void)
{
	Initialization();
	
    while(1)
    {	
		if (MainTimer.ms100)
		{
			if (Mode.run && !Mode.startDelay)
			{
				Calculation();
				Transmit();
			}
			
			if (Mode.run)
			{
				TCNT0 = 0;
				TCNT1 = 0;
				Measure.Fp = 0;
				Measure.ovf = 0;
			}
			
			MainTimer.ms100 = false;
		}
		
		if (MainTimer.s)
		{
			if (Mode.startDelay) Mode.startDelay--;
			
			if (Mode.run && !Mode.startDelay)
			{
				LedInv;
				
				Regulation();
				
				if (Motor.isDelay > 0) Motor.isDelay--;
				
				if (Motor.isStep) 
				{
					Motor.isStep--;
					if (!Motor.isStep) Motor.isDelay = PauseBetweenSteps;
				}
				
				//Calculation();
				//Regulation();
				//Transmit();

				if (Motor.operation != Locked && Mode.faultDelay && !Mode.fault) Mode.faultDelay--;
				
				if (!Mode.faultDelay && !Mode.fault)
				{
					FaultOn;
					Mode.fault = true;
				}
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
				//Comparator(On);
			}
			
			if (!Running && Mode.run)
			{
				LedOff;
				ImpOff;
				Timer0(false);
				Timer1(false);
				//Comparator(Off);
				KalmanAramid(0, true);
				KalmanPolyamide(0, true);
				Measure.Ua = 0;
				Measure.Up = 0;
				Measure.Fp = 0;
				Mode.run = false;
				Mode.fault = false;
				Mode.faultDelay = FaultDelay;
				Mode.startDelay = 0;
				Motor.operation = Locked;
			}
			
			MainTimer.s = false;
		}
		
		if (Motor.isStep)
		{
			Step();
		}
    }
}