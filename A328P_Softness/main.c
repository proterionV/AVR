/*
 * main.c
 *
 * Created: 3/17/2022 11:55:07 AM
 * Author: igor.abramov
 * Measure vibration level as softness of paper yarn
 */ 

#define F_CPU	16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define Right    (~PIND & (1<<2))
#define Left     (~PIND & (1<<3))
#define Enter    (PIND  & (1<<4))

#define DDSOut	 (Check(PORTD, 7))
#define DDSOutInv Inv(PORTD, 7)

#define Init	 0
#define On		 1
#define Off		 2
#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define MovAvgSize		10
#define RMSArraySize	100

#define RxBufferSize    100
#define TxBufferSize	100

#define FillCell    0xFF
#define Terminator  '$'
#define Eraser		' '
#define StringEnd	'\0'
#define CR			'\r'
#define LF			'\n'

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
} MainTimer;

struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
	bool addendumChanged, speedChanged;
	float multiplier;
	float addendumValues[4];
	enum Addendums
	{
		tenths,
		units,
		dozens,
		hundredths
	} addendum;
} Encoder;

volatile struct
{
	signed int value;
	float voltage, avg, kalman, rms;
	bool done;
} Convert;

volatile struct
{
	unsigned char byte;
	bool byteReceived;
} Rx;
 
volatile struct
{
	float frequency;
	unsigned long int accum, increment;
} DDS;
 
const unsigned long int ACCUM_MAXIMUM = 1000000000;
const unsigned int		FREQUENCY_MAXIMUM = 7812;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(10 << CS01)|(1 << CS00);
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
	MainTimer.ms16++;

	if (MainTimer.ms16 % 10 == 0) MainTimer.ms160++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT0 = 5;
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // 1024 bit scaler
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
		ADCSRA = 0x8F;
		ADMUX = 0x41;
		ADCSRA |= (0<<ADSC);
		break;
	}
}

ISR(ADC_vect)
{
	Converter(Off);
	Convert.value = ADCW;
	Convert.done = true;
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

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;
}

float MovAvg(float value, bool reset)
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

float Kalman(float value, bool reset)
{
	static float measureVariation = 5, estimateVariation = 0.20, speedVariation = 0.01;
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
	CurrentEstimate = LastEstimate + Gain * (value - LastEstimate);
	estimateVariation = (1.0 - Gain) * estimateVariation + fabs(LastEstimate - CurrentEstimate) * speedVariation;
	LastEstimate = CurrentEstimate;
	return CurrentEstimate;
}

float RMS(float value, bool reset)
{
	static float values[RMSArraySize];
	static unsigned short index = 0;
	static float result;
	
	if (reset)
	{
		memset(values, 0, RMSArraySize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += (value*value) - values[index];
	values[index] = value*value;
	index = (index + 1) % RMSArraySize;
	
	return sqrt(result/RMSArraySize);
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
	static char rms[20], addendum[20], speed[20];
	
	if (DDS.frequency > 0)
	{
		EraseUnits(0, 0, 0, Convert.rms);
		sprintf(rms, "%.1f", Convert.rms);
		lcd_puts(rms);
	}
	else
	{
		if (Convert.rms > 0)
		{
			Convert.rms = 0;
			EraseUnits(0, 0, 0, Convert.rms);
			sprintf(rms, "%.1f", Convert.rms);
			lcd_puts(rms);	
		}
	}
	
	if (Encoder.addendumChanged)
	{
		sprintf(addendum, "%.1f", Encoder.addendumValues[Encoder.addendum]);
		EraseUnits(10, 1, 0, Encoder.addendumValues[Encoder.addendum]);
		lcd_gotoxy(10, 1);
		lcd_puts(addendum);
		Encoder.addendumChanged = false;
	}
	
	if (Encoder.speedChanged)
	{
		sprintf(speed, "%.1f", DDS.frequency);
		EraseUnits(0, 1, 2, DDS.frequency);
		lcd_gotoxy(0, 1);
		lcd_puts(speed);
		Encoder.speedChanged = false;
	}
}

void Initialization(const char* projectName)
 {
	 DDRB = 0b00111100;
	 PORTB = 0b00000011;
	 
	 DDRC = 0b00111100;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b11000010;
	 PORTD = 0b00111111;
	 
	 Encoder.addendumValues[tenths] = 0.1;
	 Encoder.addendumValues[units] = 1;
	 Encoder.addendumValues[dozens] = 10;
	 Encoder.addendumValues[hundredths] = 100;
	 Encoder.addendum = hundredths;
	 
	 Encoder.addendumChanged = true;
	 Encoder.speedChanged = true;
	 
	 Convert.rms = 1;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_gotoxy(4, 0);
	 lcd_puts(projectName);
	 _delay_ms(2000);
	 lcd_clrscr();
	 lcd_home();
	 
	 MovAvg(0, true);
	 Kalman(0, true);
	 RMS(0, true);
	 
	 USART(Init);
	 USART(Off);
	 Timer0(true);
	 Converter(Init);
	 sei();
 }

void Transmit(float value)
{
	char voltage[20] = { 0 };
	sprintf(voltage, "%.1f,", value);
	TxString(voltage);	
}

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.frequency < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.frequency)/FREQUENCY_MAXIMUM)*divider;
}

void SetOptionDDS(short direction)
{
	if (direction > 0) DDS.frequency += DDS.frequency + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ? Encoder.addendumValues[Encoder.addendum] : 0;
	if (direction < 0) DDS.frequency -= DDS.frequency - Encoder.addendumValues[Encoder.addendum] < 0 ? DDS.frequency : Encoder.addendumValues[Encoder.addendum];
	DDS.increment = GetAddendum();
	if (DDS.frequency < 0.1) { Timer2(false); Converter(Off); } else { Timer2(true); Converter(On); }
	Encoder.speedChanged = true;
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
				switch(Encoder.addendum)
				{
					case tenths:
					Encoder.addendum = units;
					break;
					case units:
					Encoder.addendum = dozens;
					break;
					case dozens:
					Encoder.addendum = hundredths;
					break;
					default:
					Encoder.addendum = tenths;
					break;
				}
				Encoder.addendumChanged = true;
			}
		}
	}
}

int main(void)
{
	Initialization("Softness");
	
	while(1)
	{	
		EncoderHandler();
		
		if (Convert.done)
		{
			Convert.voltage = Convert.value*0.0048875855327468;
			Convert.rms = RMS(Convert.voltage, false);
			Convert.done = false;
			Converter(On);
		}
		
		if (MainTimer.ms160)
		{
			 
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			LedInv;	
			DisplayPrint();
			MainTimer.ms992 = 0;
		}
	}
}