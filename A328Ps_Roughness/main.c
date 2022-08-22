/*
 * main.c
 *
 * Created: 8/9/2022 12:13:48 PM
 *  Author: igor.abramov
 */ 

#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define DDSOut	 (Check(PORTB, 0))
#define DDSOutInv Inv(PORTB, 0)

#define Right    (~PIND & (1<<2))
#define Left     (~PIND & (1<<3))
#define Enter    (PIND  & (1<<4))

#define Init	 0
#define On		 1
#define Off		 2

#define Waiting		1
#define Record		2
#define Comparsion	3
#define Finished	4
#define Pause		5

#define Accel		10
#define Decel		20
#define Move		30
#define Stop		40

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

enum
{
	Measurment,
	Parameters,
	Manual,
	Default		
} MenuItem;

volatile struct
{
	unsigned int ms200, ms1000, sec;
} MainTimer;

volatile struct
{
	float setting, frequency;
	unsigned long int accum, increment;
} DDS;

volatile struct
{
	signed int value;
	float voltage;
	bool done;
} Convert;

volatile struct
{
	float ungrinded, grinded;
	float difference;
} Measure;

volatile struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
} Encoder;

const unsigned long int ACCUM_MAXIMUM = 1000000000;
const unsigned int		FREQUENCY_MAXIMUM = 31250;

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
		TIMSK1 = (1 << TOIE1);
		TCNT1 = 62411;
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
	TIMSK1 = (0 << TOIE1);
	TCNT1 = 62411;
}

ISR(TIMER1_OVF_vect)
{
	if (MainTimer.ms200 >= 5)
	{
		MainTimer.ms200 = 0;
		MainTimer.ms1000++;
	}
	
	MainTimer.ms200++;
	TCNT1 = 62411;
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20); // 1024 bit scaler
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
		ADMUX = 0x40;
		break;
	}
}

ISR(ADC_vect)
{
	Converter(Off);
	Convert.value = ADCW;
	Convert.done = true;
}

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.frequency < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.frequency)/FREQUENCY_MAXIMUM)*divider;
}

void SetOptionDDS(short direction)
{
	//if (Menu.mode == Manual)
	//{
		//if (direction > 0) DDS.frequency += DDS.frequency + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ? Encoder.addendumValues[Encoder.addendum] : 0;
		//if (direction < 0) DDS.frequency -= DDS.frequency - Encoder.addendumValues[Encoder.addendum] < 0 ? DDS.frequency : Encoder.addendumValues[Encoder.addendum];
		//DDS.increment = GetAddendum();
		//if (DDS.frequency < 0.1) { PhaseOff; Timer2(false); } else { Timer2(true); PhaseOn; }
		//return;
	//}
	//
	//if (!direction)
	//{
		//DDS.frequency = Measure.frequency * Encoder.multiplier;
		//DDS.increment = GetAddendum();
		//return;
	//}
	//
	//if (direction > 0)
	//{
		//Encoder.multiplier += DDS.frequency >= FREQUENCY_MAXIMUM ? 0 : Encoder.addendumValues[Encoder.addendum];
		//eeprom_update_float((float*)1, Encoder.multiplier);
		//Encoder.multiplierChanged = true;
	//}
	//
	//if (direction < 0)
	//{
		//Encoder.multiplier -= Encoder.multiplier <= Encoder.addendumValues[Encoder.addendum] ? Encoder.multiplier : Encoder.addendumValues[Encoder.addendum];
		//eeprom_update_float((float*)1, Encoder.multiplier);
		//Encoder.multiplierChanged = true;
	//}
}

void EncoderHandler(void)
{
	if (Right) Encoder.forward = 0;
	{
		if (!Right) Encoder.forward++;
		{
			if (Encoder.forward == 1 && Left)
			{
				SetOptionDDS(1);
				return;
			}
		}
	}
	
	if (Left) Encoder.backward = 0;
	{
		if (!Left) Encoder.backward++;
		{
			if (Encoder.backward == 1 && Right)
			{
				SetOptionDDS(-1);
				return;
			}
		}
	}
	
	if (Enter) Encoder.button = 0;
	{
		if (!Enter) Encoder.button++;
		{
			if (Encoder.button == 1)
			{
				
			}
		}
	}
}

void Initialization()
{
	DDRB = 0xFF;
	PORTB = 0x00;
	
	DDRD = 0x00;
	PORTD = 0xFF;
	
	DDS.frequency = 2000;
	DDS.increment = GetAddendum();
	
	Timer1(true);
	Timer2(true);
	sei();	
}

int main(void)
{
	Initialization();
	
    while(1)
    {
		if (MainTimer.ms1000)
		{
			LedInv;
			MainTimer.ms1000 = 0;
		}   
    }
}