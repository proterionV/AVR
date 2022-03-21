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

#define StartSPI	Low(PORTB, 2)	 
#define EndSPI		High(PORTB, 2)	

#define DDSOut		(Check(PORTB, 4))
#define DDSOutInv	Inv(PORTB, 4)

#define Frequency 	200
#define	Tension		201

#define TnArraySize 20
#define TArraySize	10
#define HArraySize	10

#define RxBufferSize    250
#define TxBufferSize	100
#define TempBufferSize  20

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define Arrow		'>'
#define Eraser		' '
#define BufferEnd   '#'
#define StringEnd	'\0'
#define CR			'\r'
#define LF			'\n'

#define Before	30
#define Between 31
#define Inside	32
#define After	33

#define Timeout 2

#include <xc.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <float.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include "lcd/lcd.h"
#include "dht/dht.h"

volatile struct 
{
	volatile unsigned int ms16, ms160, ms992, s;
} MainTimer;

volatile struct 
{
	float period, frequency, bufFrequency, temperature, humidity, tension;
	unsigned long int ticksCurrent,ticksPrevious,ticks;
	unsigned long int overflows,ticksBuffer;
	bool done, zero;
} Measure;

volatile struct 
{
	signed int value;
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
	unsigned long int increment, accum;
	unsigned long int accumMax, frequencyMax;	
} DDS;

volatile struct
{
	bool tryToConnect, connected;
	bool receiving, handling;
	bool building, sending;
	bool reset;
	unsigned short delay, timeout;
		
} Server;

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
	
	if (DDS.accum >= DDS.accumMax)
	{
		DDSOutInv;
		DDS.accum -= DDS.accumMax;
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
			UBRR0  =  3;
			break;
	}
}

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;
}

void SPI(unsigned short option)
{
	switch (option)
	{
		case On:
			SPCR |= (1<<SPE);
			break;
		case Off:
			SPCR |= (0<<SPE);
			break;
		default:
			SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (0<<SPR0);
			SPDR = 0b00000000;
			break;		
	}
}

void Initialization()
{
	DDRB = 0b00111110;
	PORTB = 0b00000001;
	
	DDRC = 0b00111111;
	PORTC = 0b00000000;
	
	DDRD = 0b11001110;
	PORTD = 0b00110011;
	
	lcd_init(LCD_DISP_ON);
	
	Server.tryToConnect = true;
	Server.delay = -1;
	
	DDS.frequencyMax = 7812;
	DDS.accumMax = 1000000000;
	DDS.increment = 0;

	Timer0(true);
	Timer1(true);
	Timer2(true);
	Converter(On);
	
	USART(Init);
	USART(On);
	SPI(Off);
	sei();
}

void WriteBytes(unsigned int word)
{
	unsigned char LSB = word & 0xff;			
	unsigned char MSB = word >> 8;  
	
	StartSPI;

	SPDR = LSB;
	while (!(SPSR & (1<<SPIF)));

	SPDR = MSB;					
	while (!(SPSR & (1<<SPIF)));			

	EndSPI;	
	
	StartSPI;

	SPDR = MSB;							// 	send First 8 MS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy
	
	SPDR = LSB;							// 	send Last 8 LS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy

	EndSPI;
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

float MovAvgTemp(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[TArraySize];
	static float result;
	
	if (reset)
	{
		memset(values, 0, TArraySize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % TArraySize;
	
	return result/TArraySize;
}

float MovAvgHum(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[HArraySize];
	static float result;
	
	if (reset)
	{
		memset(values, 0, HArraySize);
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % HArraySize;
	
	return result/HArraySize;
}

void SetIncrement(unsigned int frequency)
{
	static unsigned int divider = 0;
	
	DDS.frequency = frequency;
	divider = DDS.frequency < 11000 ? 10000 : 100000;
	DDS.increment = (((DDS.accumMax/divider)*DDS.frequency)/DDS.frequencyMax)*divider;
}

void EraseUnits(int x, int y, int offset, float count)
{
	if (count<10000)
	{
		lcd_gotoxy(x+offset+4,y);
		lcd_putc(Eraser);
	}
	
	if (count<1000)
	{
		lcd_gotoxy(x+offset+3,y);
		lcd_putc(Eraser);
	}
	
	if (count<100)
	{
		lcd_gotoxy(x+offset+2,y);
		lcd_putc(Eraser);
	}
	
	if (count<10)
	{
		lcd_gotoxy(x+offset+1,y);
		lcd_putc(Eraser);
	}
	
	
	lcd_gotoxy(x, y);
}

void DisplayPrint()
{
	static char frequency[20];
	
	EraseUnits(8, 0, 0, Measure.frequency);
	sprintf(frequency, "F%.1f", Measure.frequency);
	lcd_puts(frequency);
}

void GetOneWireData(bool enable)
{
	static float temperature, humidity;
	
	if (!(enable)) return;
	
	if (dht_gettemperaturehumidity(&temperature, &humidity) != -1)
	{
		Measure.temperature = MovAvgTemp(temperature, false);
		Measure.humidity = MovAvgHum(humidity, false);
		return;	
	}
	
	Measure.temperature = MovAvgTemp(-1, false);
	Measure.humidity = MovAvgHum(-1, false);
}

void Calculation(unsigned short parameter)
{
	static int adc = 0;
	
	if (parameter == Tension)
	{
		adc = Convert.value-18;
		Measure.tension = MovAvgTns(adc < 1 ? 0 : (adc*7.)-7, false);
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

unsigned short GetDataSize(char *buffer)
{
	int count = 0;
	
	for (int i=0;i<100; i++)
	{
		if (buffer[i] == BufferEnd) break;
		count++;
	}
	
	return count;	
}

void ConnectToServer(bool connect)
{
	Server.connected = false;
	Server.tryToConnect = true;
	if (connect) TxString("AT+CIPSTART=\"TCP\",\"192.168.43.108\",11000\r\n");
	else TxString("AT+CIPCLOSE\r\n");
	Server.receiving = true;
	Server.delay++;
}

void Transmit()
{
	static char TxBuffer[TxBufferSize] = { 0 };
	static char Temp[TempBufferSize] = { 0 };
	
	if (Server.building)
	{
		char Command[TempBufferSize] = "AT+CIPSEND=";
		
		sprintf(Temp, "F%.1f$", Measure.frequency);
		strcat(TxBuffer, Temp);
		sprintf(Temp, "Tn%.1f$", Measure.tension);
		strcat(TxBuffer, Temp);
		sprintf(Temp, "T%.1f$", Measure.temperature);
		strcat(TxBuffer, Temp);
		sprintf(Temp, "H%.1f$", Measure.humidity);
		strcat(TxBuffer, Temp);
		sprintf(Temp, "K%.3f#", DDS.frequency);
		strcat(TxBuffer, Temp);
		
		sprintf(Temp, "%d\r\n", GetDataSize(TxBuffer));
		strcat(Command, Temp);
		
		TxString(Command);
		
		Server.building = false;
		return;	
	}
	
	if (Server.sending)
	{
		TxString(TxBuffer);
		Server.sending = false;
		memset(TxBuffer, 0, TxBufferSize);
	}		
}

void Receive()
{
	static unsigned int charIndex = 0, wordIndex = 0, position = Before;
	static char RxBuffer[RxBufferSize] = { 0 };
	static char Response[10][80] = { 0 };
	
	if (Server.receiving)
	{ 
		RxBuffer[charIndex++] = Rx.byte; 
		return; 
	}
	
	if (Server.connected && !Server.receiving && !Server.handling)
	{
		RxBuffer[charIndex++] = Rx.byte;
		Server.receiving = true;
		Server.delay++;
		return;		
	}
	
	if (charIndex < 1) return;
	
	RxBuffer[charIndex] = StringEnd;
	wordIndex = 0;
	charIndex = 0;
	position = Before;
	
	for(int i=0; i<sizeof(RxBuffer); i++)
	{	 
		if (position == Before)
		{
			if (RxBuffer[i] == StringEnd) break;
			if (RxBuffer[i] == CR || RxBuffer[i] == LF) continue;
			position = Inside;
			i--;		
		}
		else if (position == Inside)
		{
			if (RxBuffer[i] == CR || RxBuffer[i] == LF)
			{
				Response[wordIndex][charIndex] = StringEnd;
				position = Between;
				charIndex = 0;
				continue;
			}
			
			Response[wordIndex][charIndex++] = RxBuffer[i];
			if (RxBuffer[i] == StringEnd) position = After;
			 
		}
		else if (position == Between)
		{
			if (RxBuffer[i] == StringEnd) position = After;
			if (RxBuffer[i] == CR || RxBuffer[i] == LF) continue;
			position = Inside;
			wordIndex++;	
			i--;	
		}
		else 
		{
				
		}
	}
	
	Server.timeout = 0;
	charIndex = 0;
	
	if (!Server.connected && !strcasecmp(Response[1], "CONNECT") && !strcasecmp(Response[2], "OK"))
	{
		Server.connected = true;
		Server.tryToConnect = false;
		return;
	}
	
	if (!strcasecmp(Response[1], "CLOSED") && !strcasecmp(Response[2], "OK"))
	 {
		Server.connected = false;
		return;
	}
	 
	if (!strcasecmp(Response[0], "CLOSED")) 
	{ 
		Server.connected = false; 
		Server.tryToConnect = true;
		return;	
	}
	
	for (int i=0; i<=wordIndex; i++)
	{
		if (!strcasecmp(Response[i], "ERROR"))
		{
			lcd_clrline(0, 0, Response[i]);
			return;
		}
	}
	
	if (!strcasecmp(Response[0], "+IPD,3:Get"))
	{
		Server.building = true;
		return;			
	}
	
	if (Response[2][0] == Arrow)
	{
		Server.sending = true;
		return;
	}
}

int main(void)
{
	Initialization();											  
	
    while(1)
    {
		if (Measure.done)
		{
			Calculation(Frequency);
			Measure.done = 0;
		}
		
		if (Convert.done)
		{
			Calculation(Tension);
			Convert.done = 0;
		}
		
		if (Rx.byteReceived || Server.handling)
		{
			Receive();
			Rx.byteReceived = false;
			Server.handling = false;
		}
		
        if (MainTimer.ms160)
        {
			
	        MainTimer.ms160 = 0;
        }
		
		if (MainTimer.ms992)
		{
			LedInv;
			GetOneWireData(true);
			Converter(On);
			Transmit();
			
			if (Server.delay == Timeout) 
			{ 
				Server.receiving = false; 
				Server.handling = true;
				Server.delay = -1; 
			}
			
			if (Server.delay >= 0) Server.delay++;  
			
			if (MainTimer.s >= 10)
			{
				if (Server.tryToConnect) ConnectToServer(true);
				MainTimer.s = 0;
			}
			
			if (Server.timeout >= 30)
			{
				ConnectToServer(false);
				Server.timeout = 0;
			}
			
			MainTimer.s++;
			Server.timeout++;
			MainTimer.ms992 = 0;
		}
    }
}