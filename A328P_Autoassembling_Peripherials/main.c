/*
 * main.c
 *
 * Created: 3/4/2022 3:28:44 PM
 *  Author: igor.abramov
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

#define BtnReset (Check(PINC, 6))

#define Activity (!Check(PIND, 5))

#define Phase    (Check(PORTD, 6))
#define PhaseOn  High(PORTD, 6)
#define PhaseOff Low(PORTD, 6)
#define PhaseInv Inv(PORTD, 6)

#define Init	 0
#define On		 1
#define Off		 2
#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define Frequency 	0
#define	Tension		1

#define TnArraySize 20

#define RxBufferSize    100
#define TxBufferSize	100

#pragma region Symbols

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define Arrow		'>'
#define Eraser		' '
#define StringEnd	'\0'

#pragma endregion Symbols

#pragma region Includes

#include <xc.h>
#include <avr/io.h>
#include <float.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#include "lcd/lcd.h"
#include "dht/dht.h"

#pragma endregion Includes

#pragma region Structs

volatile struct 
{
	volatile unsigned int ms16, ms160, ms992;
} MainTimer;

volatile struct 
{
	float period, frequency, bufFrequency;
	unsigned long int ticksCurrent,ticksPrevious,ticks;
	unsigned long int overflows,ticksBuffer;
	bool done, zero;
} Measure;

volatile struct 
{
	float tension;
	signed int value;
	bool done;	
} Convert;

volatile struct 
{
	unsigned char byte;
	bool byteReceived;	
} Rx;

#pragma endregion Structs

#pragma region Inits, Interrupts 

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

ISR(TIMER0_OVF_vect)
{
	MainTimer.ms16++;

	if (MainTimer.ms16 % 10 == 0)MainTimer.ms160++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT0 = 5;
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << ICNC1)|(1 << ICES1)|(0 << CS12)|(1 << CS11)|(1 << CS10);
		TIMSK1 = (1 << TOIE1)|(1 << ICIE1);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
	TIMSK1 = (0 << TOIE1)|(0 << ICIE1);
}

ISR(TIMER1_OVF_vect)
{
	Measure.overflows++;
	
	if (Measure.overflows > 2)
	{
		Measure.zero = true;
		Measure.done = true;
	}
}

ISR(TIMER1_CAPT_vect)
{
	Measure.ticksBuffer = ICR1;
	Measure.done = true;
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
		 ADMUX = 0x40;
		 ADCSRA |= (0<<ADSC);
		 break;
	 }
 }

ISR(ADC_vect)
{
	Converter(Off);
	Convert.value = ADCW;
	Convert.done++;
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
		UBRR0  =  0;
		break;
	}
}

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;
}

#pragma endregion Inits and Interrupts

#pragma region Common functions

void Initialization()
{
	DDRB = 0b00111100;
	PORTB = 0b00000011;
	
	DDRC = 0b00111100;
	PORTC = 0b01000000;
	
	DDRD = 0b11011110;
	PORTD = 0b00100011;
	
	lcd_init(LCD_DISP_ON);
	Timer0(true);
	Timer1(false);
	sei();
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

float Kalman(float value, bool reset)
{
	static float measureVariation = 40, estimateVariation = 0.20, speedVariation = 0.001;
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

float MovAvgTns(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[TnArraySize];
	static float result;
	
	if (reset)
	{
		memset(values, 0, TnArraySize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % TnArraySize;
	
	return result/TnArraySize;
}

#pragma endregion Common functions

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
}

void GetOneWireData()
{
	static char bufferT[20], bufferH[20];
	static float temperature, humidity;
	
	if(dht_gettemperaturehumidity(&temperature, &humidity) != -1)
	{
		EraseUnits(0, 0, 2, temperature);
		sprintf(bufferT, "%.1f C", temperature);
		lcd_gotoxy(0, 0);
		lcd_puts(bufferT);
		
		EraseUnits(0, 1, 2, humidity);
		sprintf(bufferH, "%.1f ", humidity);
		lcd_gotoxy(0, 1);
		lcd_puts(bufferH);
		lcd_putc('%');
	}
	else
	{
		lcd_clrscr();
		lcd_home();
		lcd_puts("error");
	}
}

void Calculation(unsigned short parameter)
{
	static int adc = 0;
	
	if (parameter == Tension)
	{
		adc = Convert.value-18;
		Convert.tension = MovAvgTns(adc < 1 ? 0 : (adc*7.)-7, false);
		adc = 0;
		
		return;
	}
		
	Measure.ticksCurrent = ((Measure.overflows * 65536L) + Measure.ticksBuffer) - Measure.ticksPrevious;
	Measure.ticksPrevious = Measure.ticksBuffer;
	Measure.period = Measure.ticksCurrent*0.000004;
	Measure.bufFrequency = Measure.period >= 1 ? 0 : Measure.period <= 0 ? 0 : 1.f/Measure.period;
	Measure.frequency = Kalman(Measure.zero > 0 ? 0 : Measure.bufFrequency < 1 ? Measure.frequency : Measure.bufFrequency, false);
	Measure.overflows = 0;
	Measure.zero = false;
}

int main(void)
{
	Initialization();
	
    while(1)
    {
        if (MainTimer.ms160)
        {
			
	        MainTimer.ms160 = 0;
        }
		
		if (MainTimer.ms992)
		{
			LedInv;
			GetOneWireData();
			MainTimer.ms992 = 0;
		}
    }
}