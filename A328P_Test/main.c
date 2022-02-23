/*
 * main.c
 *
 * Created: 1/25/2022 2:34:40 PM
 *  Author: igor.abramov
 */ 

#define F_CPU   16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT)) 
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))  

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define True	1
#define False	0

#define Forward  0
#define Backyard 1

#define Init 2
#define On	 1
#define Off  0

#define NextLine 0x0A
#define FillCell 0xFF
#define Terminator '$'

#define RxBufferSize 100
#define TxBufferSize 100

#define DDSOut	 (Check(PORTD, 7))
#define DDSOutInv Inv(PORTD, 7)

#define ServoUp		 High(PORTB, 1)
#define	ServoDown 	 Low(PORTB, 1)
#define ServoCommand (Check(PINC, 0))

#define Counter	0
#define Oscillator 1

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
#include "lcd/lcdpcf8574/lcdpcf8574.h"

const unsigned long int		ACCUM_MAXIMUM = 1875000000;
//const unsigned int		FREQUENCY_MAXIMUM = 7812; // timer2 divider 1024
const unsigned int	    FREQUENCY_MAXIMUM = 31250; // timer2 divider 256
//const unsigned long int   FREQUENCY_MAXIMUM = 62500; // timer2 divider 128

struct
{
	unsigned int ms40, ms200, ms1000;
	unsigned int ms16, ms992;
	bool isr;
} MainTimer;

struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
	float multiplier;
	float addendumValues[4];
	enum Addendums
	{
		one,
		ten,
		hundred,
		thousand
	} addendum;
} Encoder;

struct
{
	unsigned short sec, min, hour, day, week, month, year;
} Watch;

struct
{
	unsigned char byte;
	unsigned short byteReceived;
} Rx;

struct
{
	unsigned long int ticksCurrent,ticksPrevious,ticks;
	unsigned long int overflows,ticksBuffer;
	unsigned short action, periodicMeasure, ovfFlag;
	unsigned short done, index, valuesFull, method, zero;
	unsigned int average;
	float values[100];
	float period, frequency, previousFrequency, bufFrequency, pulseCount;
} Measure;

struct
{
	float setting;
	unsigned long int increment, accum, frequency;
} DDS;

struct
{
	float Kpid;
	float Kp;
	float Ki;
	float Kd;
	
} Factors;

volatile struct
{
	float tension;
	int value;
	unsigned short done;
} Convert;

ISR(TIMER0_OVF_vect)
{
	MainTimer.ms16++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT0 = 5;
}

ISR(TIMER1_OVF_vect)
{
	if (MainTimer.ms40 % 5 == 0) MainTimer.ms200++;
	
	if (MainTimer.ms40 >= 25)
	{
		MainTimer.ms1000++;
		MainTimer.ms40 = 0;
	}
	
	MainTimer.isr = true;
	TCNT1 = 64911;
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

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;	
}

ISR(ADC_vect)
{
	ADCSRA |= (0<<ADSC);
	Convert.value = ADCW;
	Convert.done++;
}

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(0 << CS01)|(1 << CS00);
		TIMSK0 = (1 << TOIE0);
		TCNT0 = 0;
		return;
	}
	
	TCCR0B = (0 << CS02)|(0 << CS01)|(0 << CS00);
	TIMSK0 = (0 << TOIE0);
	TCNT0 = 0;
}

void Timer1(unsigned short mode)
{
	switch(mode)
	{
		case Counter:
			TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
			TIMSK1 = (1 << TOIE1);
			TCNT1 = 62411;
			break;
		case Oscillator:
			TCCR1A|=(1<<COM1A1)|(1<<WGM11);        //NON Inverted PWM
			TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
			ICR1=4999;  //fPWM=50Hz	
			break;
		default:
			TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
			TIMSK1 = (0 << TOIE1);
			TCNT1 = 62411;
			break;
	}
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20); // 128 bit scaler
		TIMSK2 = (1<<TOIE2);
		return;
	}
	
	TCCR2B = (0<<CS22) | (0<<CS21) | (0<<CS20);
	TIMSK2 = (0<<TOIE2);
	TCNT2 = 0;
}

void EraseUnits(int x, int y, int offset, float count)
{
	static unsigned char eraser = 32;
	
	if (count<1000000000 || count < 0)
	{
		lcd_gotoxy(x+offset+9,y);
		lcd_putc(eraser);
	}
	
	if (count<100000000 || count < 0)
	{
		lcd_gotoxy(x+offset+8,y);
		lcd_putc(eraser);
	}
	
	if (count<10000000 || count < 0)
	{
		lcd_gotoxy(x+offset+7,y);
		lcd_putc(eraser);
	}
	
	if (count<1000000 || count < 0)
	{
		lcd_gotoxy(x+offset+6,y);
		lcd_putc(eraser);
	}
	
	if (count<100000 || count < 0)
	{
		lcd_gotoxy(x+offset+5,y);
		lcd_putc(eraser);
	}
	
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
}

void DisplayPrint()
{
	static char frequency[20];
	
	EraseUnits(0, 0, 0, DDS.setting);
	sprintf(frequency, "F%.1f$", DDS.setting);
	lcd_gotoxy(0, 0);
	lcd_puts(frequency);
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
			ADCSRA = 0x8F;
			ADMUX = 0x41;
			ADCSRA |= (0<<ADSC);
			break;
	}
}

void USART(unsigned short option)
{
	switch (option)
	{
		case 0:
			UCSR0B |= (0 << TXEN0);
			break;
		case 1:
			UCSR0B |= (1 << TXEN0);
			break;
		default:
			UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
			UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
			UBRR0L = 0;	 // 1 MBit/s
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
	static char frequency[20], tension[20];
	sprintf(frequency, "F%.1f$", DDS.setting);
	sprintf(tension, "Tn%.2f", Convert.tension);
	TxString(tension);
}

void Receive()
 {
	 static char RxBuffer[RxBufferSize];
	 static char TxBuffer[TxBufferSize] = "error: ";
	 static char RxCharBuffer[2];
	 
	 RxCharBuffer[0] = Rx.byte;
	 
	 if (RxCharBuffer[0] != Terminator)
	 { 
		 strcat(RxBuffer, RxCharBuffer);
		 return;
	 }
	 
	 if (!(strcasecmp(RxBuffer, "led")))
	 {
		 LedInv;
	 }
	 else
	 {
		 strcat(TxBuffer, RxBuffer);
		 TxString(TxBuffer);
		 memset(TxBuffer, 9, TxBufferSize);
	 }
	 
	 memset(RxBuffer, 0, RxBufferSize);
 }

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.setting < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.setting)/FREQUENCY_MAXIMUM)*divider;
}

void SetOptionDDS(short direction)
{
	if (!direction)
	{
		DDS.setting = Measure.frequency;// * Encoder.multiplier;
		DDS.increment = GetAddendum();
		return;
	}
	
	if (direction > 0)
	{
		Encoder.multiplier += DDS.setting >= FREQUENCY_MAXIMUM ? 0 : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
	
	if (direction < 0)
	{
		Encoder.multiplier -= Encoder.multiplier <= Encoder.addendumValues[Encoder.addendum] ? Encoder.multiplier : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
}

void RegulatorInit(float Kpid, float Kp, float Ki, float Kd)
{
	Factors.Kpid = Kpid;
	Factors.Kp = Kp;
	Factors.Ki = Ki;
	Factors.Kd = Kd;
}

void StepperStep()
{
	static unsigned short direction = Forward;

	if (DDS.setting <= 0 && direction == Backyard) direction = Forward;
	if (DDS.setting >= 20000 && direction == Forward) direction = Backyard;
	if (direction == Forward) Measure.frequency++;// else Measure.frequency--;
	SetOptionDDS(0);
}

void Regulator(void)
{
	static float I,previousError;
	float P,D,regulationError,factor;

	DDS.setting = (Measure.frequency*Encoder.multiplier)*Factors.Kpid;
	regulationError = DDS.setting - DDS.frequency;
	
	P = regulationError*Factors.Kp;
	I = (I+(regulationError*0.08))*Factors.Ki;
	D = ((regulationError-previousError)/0.08)*Factors.Kd;
	
	factor = P+I+D;
	previousError = regulationError;
	
	DDS.frequency = factor < 0 ? factor*(-1) : factor;
	
	DDS.increment = GetAddendum();
}

void ServoStep(unsigned int direction)
{
	static unsigned short action = 0;
	
	if (ServoCommand) action = 0;
	{
		if (!ServoCommand) action++;
		{
			if (action == 1)
			{	
				ServoUp;
				_delay_ms(1);
				ServoDown;
				action = 0;
			}
		}
	}
}

int main(void)
{
	float voltage = 0;
	
	DDRB = 0b00111110;
	PORTB = 0b00000001;
	
	DDRC = 0b00111100;
	PORTC = 0b11000000;
	
	DDRD = 0b11000010;
	PORTD = 0b00111111;
	
	Timer0(false);
	Timer1(Counter);
	Timer2(false);
	USART(Init);
	Converter(Init);
	sei();
	   
	while(1)
	{	
		if (Rx.byteReceived)
		{
			Receive();
			Rx.byteReceived = 0;
		}
		
		if (MainTimer.isr)
		{
			StepperStep();
			MainTimer.ms40++;
			MainTimer.isr = false;
		}
		
		if (Convert.done)
		{
			voltage	= Convert.value*0.0048828125;
			Convert.tension = voltage >= 0.11 ? (voltage-0.11)*1445.f : 0;
			Convert.done = 0;
		}
		
		if (MainTimer.ms200) 
		{
			Converter(On);
			MainTimer.ms200 = 0;
		}
		
		if (MainTimer.ms1000) 
		{	
			LedInv;
			Transmit();
			MainTimer.ms1000 = 0;
		}
	}
}