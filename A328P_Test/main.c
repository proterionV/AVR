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

#define NextLine 0x0A
#define FillCell 0xFF
#define Terminator '$'

#define SizeReceiveBuffer 100
#define SizeTransmitBuffer 100

#define DDSOut	 (Check(PORTD, 7))
#define DDSOutInv Inv(PORTD, 7)

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
//const unsigned int	    FREQUENCY_MAXIMUM = 15625; // timer2 divider 256
const unsigned long int	FREQUENCY_MAXIMUM = 62500; // timer2 divider 128
unsigned short direction = 0;

struct
{
	unsigned int ms200, ms200a, ms1000;
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
	char bytes[SizeReceiveBuffer];	
	unsigned short byteReceived, dataHandled;
} Receive;

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
	unsigned long int accum, increment, counter, frequency;
} DDS;

struct
{
	float Kpid;
	float Kp;
	float Ki;
	float Kd;
	
} Factors;

ISR(TIMER1_OVF_vect)
{
	TCNT1 = 64911;
	MainTimer.ms200++;
	//MainTimer.ms200a++;
	//
	//if (MainTimer.ms200 >= 5)
	//{
		//MainTimer.ms1000++;
		//MainTimer.ms200 = 0;
	//}
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
	Receive.byte = UDR0;
	Receive.byteReceived++;	
}

void Timer1()
{
	TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
	TIMSK1 = (1 << TOIE1);
	TCNT1 = 62411;
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (0<<CS21) | (1<<CS20); // 128 bit scaler
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

void DisplayPrint(unsigned short cancel)
{
	static char sec[10];
	
	if (cancel) return;
	
	EraseUnits(0, 1, 0, Watch.sec);
	sprintf(sec,"%.d", Watch.sec);
	lcd_gotoxy(0, 1);
	lcd_puts(sec);
}

void UART()
{
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0L = 0;
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
	static char frequency[20];
	sprintf(frequency, "F%.1f$", DDS.setting);
	TxString(frequency);
}

unsigned short UART_ReceiveHandler()
{
	static unsigned short queue = 0;
	static char undefined[SizeTransmitBuffer];
	
	if (Receive.byte != Terminator) 
	{
		Receive.bytes[queue] = Receive.byte;
		queue = (queue + 1) % SizeReceiveBuffer;
		return False;			
	}
	
	Receive.bytes[++queue] = 0;
	
	if (!(strcasecmp(Receive.bytes, "led")))
	{
		if (Led) LedOff; else LedOn;
	}
	else if (!(strcasecmp(Receive.bytes, "print")))
	{
		EraseUnits(0, 0, 0, 0);
		lcd_gotoxy(0, 0);
		lcd_puts(Receive.bytes);
	}
	else
	{
		for (int i=0; i<SizeTransmitBuffer; i++) undefined[0] = 0;
		strcat(undefined, "Undefined command: \"");
		strcat(undefined, Receive.bytes);
		strcat(undefined, "\"");
		TxString(undefined);	
	}
	
	for (int i=0; i<SizeReceiveBuffer; i++) Receive.bytes[i] = 0;
	queue = 0;
	return True;
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
		//DDS.setting = Measure.frequency * Encoder.multiplier;
		DDS.increment = GetAddendum();
		return;
	}
	
	if (direction > 0)
	{
		Encoder.multiplier += DDS.setting >= 31250 ? 0 : Encoder.addendumValues[Encoder.addendum];
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

int main(void)
{
	DDRB = 0xFF;
	PORTB = 0x00;

	DDRD = 0xFF;
	PORTD = 0x00;

	Timer1();
	Timer2(true);
	UART();
	sei();

	while(1)
	{		
		if (MainTimer.ms200)
		{
			//if (DDS.setting <= 1000) direction = 0;
			if (DDS.setting >= 26000) direction++;
			if (!direction) DDS.setting += 5; // else DDS.setting -= 62;
			SetOptionDDS(0);
			Transmit();
			MainTimer.ms200 = 0;
		}
		
		if (MainTimer.ms1000)
		{
			LedInv;
			MainTimer.ms1000 = 0;
		}  
	}
}