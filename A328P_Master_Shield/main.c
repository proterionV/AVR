/*
 * main.c
 *
 * Created: 10/7/2022 8:22:24 PM
 *  Author: prote
 */ 

#define F_CPU 16000000L

#include <xc.h>
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <util/delay.h>
#include "lcd/lcd.h"

#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Led			Check(PORTB, PORTB5)
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)

#define Off	 0
#define On	 1
#define Init 2

#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define RxBufferSize    250
#define TxBufferSize	100
#define TempBufferSize  20
#define Before			30
#define Between			31
#define Inside			32
#define After			33
#define Timeout			2

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define Arrow		'>'
#define Eraser		' '
#define BufferEnd   '#'
#define StringEnd	'\0'
#define CR			'\r'
#define LF			'\n'

#define Down	1
#define Up		2
#define	Right	3
#define Left	4
#define Select	5
#define None	0 

#define Config		1
#define Connection  10
#define Settings	11

#define Observe		2
#define	Devices		20
#define Monitor		21

#define Test		3
#define Test1		30
#define Test2		31  

struct TimeControl
{
	unsigned int ms, s;
	bool ms50, ms200, sec;
} MainTimer;

struct Interface
{
	unsigned short x, y, arrowPosition, button;
	unsigned short page, item, subitem, row;
	bool printed;
} Menu;

struct Analog
{
	unsigned int value;
	bool complete;
} Convertion;

struct Receiving
{
	unsigned char byte;
	bool byteReceived;
} Rx;

struct TCP
{
	bool tryToConnect, connected;
	bool receiving, handling;
	bool building, sending;
	bool reset;
	unsigned short delay, timeout;
	
} Client;

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
	
	if (!(MainTimer.ms % 50)) MainTimer.ms50 = true;
	if (!(MainTimer.ms % 200)) MainTimer.ms200 = true;
	
	if (MainTimer.ms >= 1000) 
	{
		MainTimer.sec = true;
		MainTimer.s++;
		if (MainTimer.s >= 60) MainTimer.s = 0;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;			   
}

void Converter(unsigned short option)
{
	switch (option)
	{
		case Off:
			Low(ADCSRA, ADSC);
			break;
		case On:
			High(ADCSRA, ADSC);
			break;
		default:
			High(ADMUX, REFS0);
			ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
			break;
	}
}

ISR(ADC_vect)
{
	Converter(Off);
	Convertion.value = ADCW;
	Convertion.complete = true;
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

void TxChar(unsigned char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
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
	Client.connected = false;
	Client.tryToConnect = true;
	if (connect) TxString("AT+CIPSTART=\"TCP\",\"192.168.43.108\",11000\r\n");
	else TxString("AT+CIPCLOSE\r\n");
	Client.receiving = true;
	Client.delay++;
}

void Transmit()
{
	static char TxBuffer[TxBufferSize] = { 0 };
	static char Temp[TempBufferSize] = { 0 };
	
	if (Client.building)
	{
		char Command[TempBufferSize] = "AT+CIPSEND=";
		
		sprintf(Temp, "r%.2f#", 3.14);
		//strcat(TxBuffer, Temp);
		//sprintf(Temp, "Tn%.1f$", Measure.tension);
		//strcat(TxBuffer, Temp);
		//sprintf(Temp, "T%.1f$", Measure.temperature);
		//strcat(TxBuffer, Temp);
		//sprintf(Temp, "H%.1f$", Measure.humidity);
		//strcat(TxBuffer, Temp);
		//sprintf(Temp, "K%.3f#", DDS.frequency);
		//strcat(TxBuffer, Temp);
		
		sprintf(Temp, "%d\r\n", GetDataSize(TxBuffer));
		strcat(Command, Temp);
		
		TxString(Command);
		
		Client.building = false;
		return;
	}
	
	if (Client.sending)
	{
		TxString(TxBuffer);
		Client.sending = false;
		memset(TxBuffer, 0, TxBufferSize);
	}
}

void Receive()
{
	static unsigned int charIndex = 0, wordIndex = 0, position = Before;
	static char RxBuffer[RxBufferSize] = { 0 };
	static char Response[10][80] = { 0 };
	
	if (Client.receiving)
	{
		RxBuffer[charIndex++] = Rx.byte;
		return;
	}
	
	if (Client.connected && !Client.receiving && !Client.handling)
	{
		RxBuffer[charIndex++] = Rx.byte;
		Client.receiving = true;
		Client.delay++;
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
	
	Client.timeout = 0;
	charIndex = 0;
	
	if (!Client.connected && !strcasecmp(Response[1], "CONNECT") && !strcasecmp(Response[2], "OK"))
	{
		Client.connected = true;
		Client.tryToConnect = false;
		return;
	}
	
	if (!strcasecmp(Response[1], "CLOSED") && !strcasecmp(Response[2], "OK"))
	{
		Client.connected = false;
		return;
	}
	
	if (!strcasecmp(Response[0], "CLOSED"))
	{
		Client.connected = false;
		Client.tryToConnect = true;
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
		Client.building = true;
		return;
	}
	
	if (Response[2][0] == Arrow)
	{
		Client.sending = true;
		return;
	}
}

void Initialization()
{
	DDRB = 0b00100011;
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b00000000;
	
	DDRD = 0b11110000;
	PORTD = 0b00000000;
	
	Menu.page = Config;
	Menu.item = None;
	Menu.subitem = None;
	Menu.row = 0;
	Menu.printed = false;
	
	lcd_init();
	lcd_clear();
	lcd_gotoxy(0, 0);
	lcd_putc(Arrow);
	
	Timer2(true);
	Converter(Init);
	sei();
}

void ButtonsCheck()
{	
	if (Menu.button == Up)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Up");
	}
	else if (Menu.button == Down)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Down");
	}
	else if (Menu.button == Right)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Right");
		return;
	}
	else if (Menu.button == Left)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Left");
	}
	else if (Menu.button == Select)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Select");
	}
	else
	{
		lcd_gotoxy(0, 1);
		lcd_puts("None");
	}	
}

void DisplayPrint()
{
	static char data[10] = { 0 };
	
	sprintf(data, "%d", Convertion.value);
	lcd_gotoxy(12, 1);
	lcd_puts(data);
}

void SetRow(unsigned short direction)
{
	if (direction == Up)
	{
		lcd_gotoxy(0, 1);
		lcd_putc(Eraser);
		lcd_gotoxy(0, 0);
		lcd_putc(Arrow);
		return;
	}
	
	if (direction == Down)
	{
		lcd_gotoxy(0, 0);
		lcd_putc(Eraser);
		lcd_gotoxy(0, 1);
		lcd_putc(Arrow);
	}
}

void KeypadControl()
{	   
	if (Convertion.value > 800) 
	{ 
		if (Menu.button == None) return; 
		Menu.button = None;
		return;
	}
	else if (Convertion.value > 700)
	{
		if (Menu.button == Select) return;
		Menu.button = Select;
		return;
	}
	else if (Convertion.value > 450)
	{
		if (Menu.button == Left) return;
		Menu.button = Left;
		return;
	} 
	else if (Convertion.value > 250)
	{
		if (Menu.button == Down) return;
		Menu.button = Down;
		return;
	}
	else if (Convertion.value > 100)
	{
		if (Menu.button == Up) return;
		Menu.button = Up;
		return;
	}
	else
	{
		if (Menu.button == Right) return;
		Menu.button = Right;
	}
}

void ConfigHandler()
{
	if (!Menu.printed)
	{
		SetRow(Menu.row);
		lcd_gotoxy(1, 0);
		lcd_puts("Connection");
		lcd_gotoxy(1, 1);
		lcd_puts("Settings");
		Menu.printed = true;
	}
}

void ObserveHandler()
{
	if (!Menu.printed)
	{
		SetRow(Menu.row);
		lcd_gotoxy(1, 0);
		lcd_puts("Devices");
		lcd_gotoxy(1, 1);
		lcd_puts("Monitor");
		Menu.printed = true;
	}
}

void TestHandler()
{
	if (!Menu.printed)
	{
		SetRow(Menu.row);
		lcd_gotoxy(1, 0);
		lcd_puts("Test1");
		lcd_gotoxy(1, 1);
		lcd_puts("Test2");
		Menu.printed = true;
	}
}

void ConnectionHandler()
{
	
}

void SettingsHandler()
{
	
}

void InterfaceControl()
{
	if (Menu.button == Right) 
	{
		Menu.page++;
		if (Menu.page > 3) Menu.page = Config; 
		Menu.printed = false;
		lcd_clear();
		SetRow(Up);
	}
	
	else if (Menu.button == Left)
	{
		Menu.page--;
		if (Menu.page < 1) Menu.page = Test;
		Menu.printed = false;
		lcd_clear();
		SetRow(Up);
	}
	
	switch(Menu.page)
	{
		case Config:
			ConfigHandler();
			break;
		case Observe:
			ObserveHandler();
			break;
		case Test:
			TestHandler();
			break;
	}
	
	SetRow(Menu.button);
}

void Handler50()
{
	if (MainTimer.ms50)
	{
		Converter(On);		
		MainTimer.ms50 = false;
	}	
}

void Handler200()
{
	if (MainTimer.ms200)
	{
		KeypadControl();
		InterfaceControl();	
		MainTimer.ms200 = false;
	}
};

void Handler1000()
{
	 if (MainTimer.sec)
	 {
		 LedInv;
		 
		 MainTimer.sec = false;
	 }
}

int main(void)
{
	Initialization();
		
    while(1)
    {	
		Handler50();
		Handler200();
		Handler1000();
    }
}