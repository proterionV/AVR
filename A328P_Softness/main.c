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

#define Active	(!Check(PIND, 5))

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

#define Acceleration	1
#define Deceleration	2
#define Waiting			3
#define Process			4

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
	volatile unsigned int ms16, ms160, ms992, sec, min;
} MainTimer;

struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
	bool addendumChanged;
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
	float voltage;
	bool done;
} Convert;
 
volatile struct
{
	float setting, frequency;
	unsigned long int accum, increment;
	bool settingChanged, frequencyChanged;
} DDS;
 
volatile struct
{
	unsigned short mode;
	bool active, action;
} Mode;

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
			ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
			ADMUX = 0x41;
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
	static char voltage[20], addendum[20], setting[20], frequency[20];
	
	if (DDS.frequency > 0)
	{
		EraseUnits(0, 0, 2, Convert.voltage);
		sprintf(voltage, "%.2f", Convert.voltage);
		lcd_puts(voltage);
	}
	else
	{
		if (Convert.voltage > 0)
		{
			Convert.voltage = 0;
			EraseUnits(0, 0, 2, Convert.voltage);
			sprintf(voltage, "%.2f", Convert.voltage);
			lcd_puts(voltage);	
		}
	}
	
	if (DDS.frequencyChanged)
	{
		sprintf(frequency, "%.1f", DDS.frequency);
		EraseUnits(0, 1, 2, DDS.frequency);
		lcd_gotoxy(0, 1);
		lcd_puts(frequency);
		DDS.frequencyChanged = false;
	}
	
	if (DDS.settingChanged)
	{
		sprintf(setting, "%.1f", DDS.setting);
		EraseUnits(10, 0, 0, DDS.setting);
		lcd_gotoxy(10, 0);
		lcd_puts(setting);
		DDS.settingChanged = false;
	}
	
	if (Encoder.addendumChanged)
	{
		sprintf(addendum, "%.1f", Encoder.addendumValues[Encoder.addendum]);
		EraseUnits(10, 1, 0, Encoder.addendumValues[Encoder.addendum]);
		lcd_gotoxy(10, 1);
		lcd_puts(addendum);
		Encoder.addendumChanged = false;
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
	 DDS.settingChanged = true;
	 DDS.frequencyChanged = true;
	 DDS.setting = eeprom_read_float((float*)1);
	 
	 Mode.mode = Waiting;
	 
	 MainTimer.sec = 0;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_gotoxy(4, 0);
	 lcd_puts(projectName);
	 _delay_ms(2000);
	 lcd_clrscr();
	 lcd_home();
	 
	 lcd_gotoxy(0, 0);
	 lcd_puts("0.0");
	 
	 USART(Init);
	 Timer0(true);
	 Timer2(false);
	 Converter(Init);
	 sei();
 }

void Transmit(float value)
{
	char voltage[20] = { 0 };
	sprintf(voltage, "%.2f,", value);
	TxString(voltage);	
}

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.frequency < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.frequency)/FREQUENCY_MAXIMUM)*divider;
}

void SetSetting(short direction)
{
	if (Mode.mode != Waiting) return;
	
	if (direction > 0) DDS.setting += DDS.setting + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ? Encoder.addendumValues[Encoder.addendum] : 0;
	if (direction < 0) DDS.setting -= DDS.setting - Encoder.addendumValues[Encoder.addendum] < 0 ? DDS.setting : Encoder.addendumValues[Encoder.addendum];
	eeprom_update_float((float*)1, DDS.setting);
	DDS.settingChanged = true;
}

void SetFrequency(short direction)
{
	static short addendum = 1;
	if (direction > 0) DDS.frequency += DDS.frequency + addendum <= FREQUENCY_MAXIMUM ? addendum : 0;
	if (direction < 0) DDS.frequency -= DDS.frequency - addendum < 0 ? DDS.frequency : addendum;
	DDS.increment = GetAddendum();
	DDS.frequencyChanged = true;
}

void EncoderHandler(void)
{
	if (Right) Encoder.forward = 0;
	{
		if (!Right) Encoder.forward++;
		{
			if (Encoder.forward == 1 && Left)
			{
				SetSetting(1);
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
				SetSetting(-1);
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

void ModeControl()
{
	if (Active && MainTimer.sec < 60)
	{
		if (DDS.setting < 1) return;
		
		if (Mode.active)
		{
			if (Mode.mode == Acceleration && DDS.frequency == DDS.setting)
			{	
				LedOn;
				Mode.mode = Process;
				Converter(On);
				USART(TxOn);
				return;
			}
			
			return;
		}
		
		Mode.active = true;
		Mode.action = false;
		Mode.mode = Acceleration;
		Timer2(true);
		return;
	}
	
	if (Mode.mode == Process)
	{
		Mode.mode = Deceleration;
		Converter(Off);
		USART(Off);
		return;
	}
	
	if (Mode.mode == Deceleration && DDS.frequency < 1)
	{
		LedOff;
		Mode.active = false;
		Mode.mode = Waiting;
		Timer2(false);
		if (!Active) MainTimer.sec = 0;
		return;
	}
	
	if (!Active && MainTimer.sec > 0) MainTimer.sec = 0;
}

int main(void)
{
	Initialization("Softness");
	
	while(1)
	{	
		EncoderHandler();
		ModeControl();
		
		if (Mode.mode == Acceleration) SetFrequency(1);
		if (Mode.mode == Deceleration) SetFrequency(-1);
		
		if (Convert.done)
		{
			Convert.voltage = Convert.value*0.0048875855327468;
			Transmit(Convert.voltage);
			Converter(On);
			Convert.done = false;
		}
		
		if (MainTimer.ms992)
		{
			if (Mode.mode == Process && MainTimer.sec < 60) MainTimer.sec++;
			if (Mode.mode == Waiting) LedInv;
			DisplayPrint();
			MainTimer.ms992 = 0;
		}
	}
}