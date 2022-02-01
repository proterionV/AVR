/*
 * main.c
 *
 * Created: 1/29/2022 6:18:04 PM
 *  Author: igor.abramov
 */ 

#define F_CPU    16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define Right    (~PINC & (1<<1))
#define Left     (~PINC & (1<<2))
#define Enter    (PINC & (1<<3))

#define BtnReset (PINC & (1<<6))

#define Enable	 (!(PINL & (1<<2)))
#define Disable	 (PINL & (1<<2))

#define DDSOut	  (PORTB & (1<<1))
#define DDSOutInv Inv(PORTB, 1)

#define Phase    (PORTB & (1<<2))
#define PhaseOn  PORTB |= (1<<2)
#define PhaseOff PORTB &= (0<<2)
#define PhaseInv PORTB ^= (1<<2)

#define Init	 2
#define On		 1
#define Off		 0

#define	Tension		1
#define Frequency 	0

#define Forward	    1
#define Reporcial 	0

#define FrequencyArraySize 150
#define TensionArraySize   30
#define RxBufferSize 100
#define TxBufferSize 100

#define NextLine    0x0A
#define FillCell    0xFF
#define Terminator  '$'
#define Arrow		'>'
#define Eraser		' '
#define StringEnd	'\0'

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
#include "lcd/lcdpcf8574/lcdpcf8574.h"

const unsigned long int ACCUM_MAXIMUM = 1875000000;
const unsigned int		FREQUENCY_MAXIMUM = 62500;

volatile struct
{
	unsigned int ms200, ms1000;
} MainTimer;

struct
{
	unsigned short X,Y,lcd;
	bool resetDelay, manualActive, autoActive, mainActive;
	unsigned short arrowPosition, resetHold, resetCount;
	char modeNames[3][10];

	enum Modes
	{
		Manual,
		Auto,
		Main
	} mode;
} Menu;

struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
	unsigned short addendumChanged;
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

volatile struct
{
	float setting;
	unsigned long int accum, increment, counter;
} DDS;

volatile struct
{
	float frequency;	
} Measure;

ISR(TIMER1_OVF_vect)
{
	TCNT1 = 62411;
	MainTimer.ms200++;
	
	if (MainTimer.ms200 >= 5)
	{
		MainTimer.ms1000++;
		MainTimer.ms200 = 0;
	}	
}

ISR(TIMER2_OVF_vect)
{
	TCNT2 = 254;
	
	DDS.accum += DDS.increment;
	
	if (DDS.accum >= ACCUM_MAXIMUM)
	{
		DDSOutInv;
		DDS.accum -= ACCUM_MAXIMUM;
	}
}

void Timer1()
{
	TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
	TIMSK1 = (1 << TOIE1);
	TCNT1 = 62411;		
}

void Timer2(unsigned int enable)
{
	if (enable)
	{
		TCCR2B = (1<<CS22) | (0<<CS21) | (0<<CS20); /// 256 bit scaler 
		TIMSK2 = (1<<TOIE2);
		return;
	}
	
	TCCR2B = (0<<CS22) | (0<<CS21) | (0<<CS20); 
	TIMSK2 = (0<<TOIE2);
	TCNT2 = 0;	
}

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.setting < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.setting)/FREQUENCY_MAXIMUM)*divider;
}

void SetOptionDDS(short direction)
{
	if (Menu.mode == Manual) 
	{
		if (direction > 0) DDS.setting += DDS.setting + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ? Encoder.addendumValues[Encoder.addendum] : 0;
		if (direction < 0) DDS.setting -= DDS.setting - Encoder.addendumValues[Encoder.addendum] < 0 ? DDS.setting : Encoder.addendumValues[Encoder.addendum];  
		DDS.increment = GetAddendum();
		return;
	}
	
	if (!direction)
	{
		DDS.setting = Measure.frequency * Encoder.multiplier;
		DDS.increment = GetAddendum();
		return;
	}
	
	if (direction > 0)
	{
		Encoder.multiplier += DDS.setting >= 62500 ? 0 : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
	
	if (direction < 0)
	{
		Encoder.multiplier -= Encoder.multiplier <= Encoder.addendumValues[Encoder.addendum] ? Encoder.multiplier : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
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
					case one:
						Encoder.addendum = ten;
						break;
					case ten:
						Encoder.addendum = hundred;
						break;
					case hundred:
						Encoder.addendum = thousand;
						break;
					default:
						Encoder.addendum = one;
						break;
				}
				Encoder.addendumChanged = true;
			}
		}
	}
}

void EraseUnits(int x, int y, int offset, float count)
{
	char eraser = 32;
	
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
	static char setting[20];
	static char addendum[10];
	static char multiplier[10];

	if (Menu.mode == Main) return;

	sprintf(setting, "%.1f Hz", DDS.setting);
	EraseUnits(0, 0, 3, DDS.setting);
	lcd_gotoxy(0, 0);
	lcd_puts(setting);
	
	if (Menu.mode == Auto)
	{
		EraseUnits(9, 0, 0, Encoder.multiplier);
		sprintf(multiplier,"x%.3f",Encoder.multiplier);
		lcd_gotoxy(9,0);
		lcd_puts(multiplier);
		
		if (Encoder.addendumChanged)
		{
			sprintf(addendum, "%.3f", Encoder.addendumValues[Encoder.addendum]);
			EraseUnits(9, 1, 0, Encoder.addendumValues[Encoder.addendum]);
			lcd_gotoxy(9, 1);
			lcd_puts(addendum);
			Encoder.addendumChanged = false;
		}
		
		return;
	}
	
	if (Encoder.addendumChanged)
	{
		sprintf(addendum, "%.f", Encoder.addendumValues[Encoder.addendum]);
		EraseUnits(9, 1, 0, Encoder.addendumValues[Encoder.addendum]);
		lcd_gotoxy(9, 1);
		lcd_puts(addendum);
		Encoder.addendumChanged = false;
	}
}

void ManualHandle()
{
	EncoderHandler();

	if (Menu.manualActive) return;
	Encoder.addendumValues[one] = 1;
	Encoder.addendumValues[ten] = 10;
	Encoder.addendumValues[hundred] = 100;
	Encoder.addendumValues[thousand] = 1000;
	Encoder.addendum = one;
	DDS.setting = 0;
	SetOptionDDS(0);
	lcd_clrscr();
	lcd_home();
	Timer2(On);
	Menu.manualActive = true;
	Encoder.addendumChanged = true;
}

void AutoHandle()
{
	EncoderHandler();
	
	if (Menu.autoActive) return;
	Encoder.addendumValues[one] = 1;
	Encoder.addendumValues[ten] = 0.1;
	Encoder.addendumValues[hundred] = 0.01;
	Encoder.addendumValues[thousand] = 0.001;
	Encoder.addendum = one;
	lcd_clrscr();
	lcd_home();
	Timer2(On);
	Encoder.multiplier = eeprom_read_float((float*)1);
	Menu.autoActive = true;
	Encoder.addendumChanged = true;	
}

void SetOption(enum Modes mode)
{	
	Menu.mode = mode;
	Menu.mainActive = false;
	Menu.manualActive = false;
	Menu.autoActive = false;
	Menu.resetHold = 0;
}

void SetArrow(short stepDirection)
{
	lcd_gotoxy(0,Menu.arrowPosition);
	lcd_putc(Eraser);
	
	Menu.arrowPosition += stepDirection > 0 ?
	Menu.arrowPosition < 1 ? stepDirection : 0
	:
	Menu.arrowPosition > 0 ? stepDirection : 0;
	
	lcd_gotoxy(0,Menu.arrowPosition);
	lcd_putc(Arrow);
}

void MainHandle()
{
	if (Right) Encoder.forward = 0;
	{
		if (!Right) Encoder.forward++;
		{
			if (Encoder.forward == 1 && Left)
			{
				SetArrow(1);
			}
		}
	}
	
	if (Left) Encoder.backward = 0;
	{
		if (!Left) Encoder.backward++;
		{
			if (Encoder.backward == 1 && Right)
			{
				SetArrow(-1);
			}
		}
	}
	
	if (Enter) Encoder.button = 0;
	{
		if (!Enter) Encoder.button++;
		{
			if (Encoder.button == 1 && !Menu.resetDelay)
			{
				SetOption(Menu.arrowPosition);
				return;
			}
		}
	}
		
	if (Menu.mainActive) return;
	Menu.mainActive = true;
	DDS.setting = 0;
	Timer2(Off); 
	lcd_clrscr();
	lcd_home();
	SetArrow(0);
	lcd_gotoxy(1, 0);
	lcd_puts(Menu.modeNames[Manual]);
	lcd_gotoxy(1, 1);
	lcd_puts(Menu.modeNames[Auto]);	
}

void DisplayReinit()
{
	if (BtnReset) Menu.lcd = 0;
	{
		if (!Enter) Menu.lcd++;
		{
			if (Menu.lcd)
			{
				lcd_init(LCD_DISP_ON);
				lcd_led(LCD_CLR);
				lcd_clrscr();
				lcd_home();
				SetOption(Main);
			}
		}
	}
}

void Initialization()
{
	DDRB = 0b00101110;
	PORTB = 0b00010001;
	
	DDRC = 0x30;
	PORTC = 0x4E;
	
	DDRD = 0xFF;
	PORTD = 0x00;

	strcpy(Menu.modeNames[0], "Manual"); // where aString is either an array or pointer to char
	strcpy(Menu.modeNames[1], "Auto");
	strcpy(Menu.modeNames[2], "Main");

	Menu.arrowPosition = 0;
	Menu.resetHold = 0;
	Menu.mode = Main;
}

int main(void)
{
	Initialization();
	lcd_init(LCD_DISP_ON);
	lcd_led(LCD_CLR);
	lcd_clrscr();
	lcd_home();

	Timer1();
	sei();
	
    while(1)
    {
		switch (Menu.mode)
		{
			case Manual:
				ManualHandle();
				break;
			case Auto:
				AutoHandle();
				break;
			case Main:
				MainHandle();
				break;
			default:
				lcd_clrscr();
				lcd_home();
				lcd_puts("Range out");
				break;
		}
		
		if (MainTimer.ms1000)
		{
			LedInv;
			DisplayPrint();
			MainTimer.ms1000 = 0;
			
			if (Menu.resetDelay) Menu.resetCount++; 
			if (Menu.resetCount >= 3) Menu.resetDelay = false;
			if (Menu.mode == Main) continue;
			if (Enter && Menu.resetHold > 0) Menu.resetHold = 0; 
			if (!Enter) Menu.resetHold++;
			if (Menu.resetHold >= 3)
			{
				SetOption(Main);
				Menu.resetDelay = true;
			}
		}
    }
}