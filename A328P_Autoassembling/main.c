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
#define Low(REG,BIT)   (REG &= (0<<BIT)) //(REG &= (0<<BIT))

#define Led     Check(PORTB, 5)
#define LedOn   High(PORTB, 5)
#define LedOff  Low(PORTB, 5)
#define LedInv  Inv(PORTB, 5)

#define Right    (~PIND & (1<<2))
#define Left     (~PIND & (1<<3))
#define Enter    (PIND  & (1<<4))

#define BtnReset (Check(PINC, 6))

#define Activity (!Check(PIND, 5))

#define DDSOut	 (Check(PORTB, 1))
#define DDSOutInv Inv(PORTB, 1)

#define Phase    (Check(PORTD, 6))
#define PhaseOn  High(PORTD, 6)
#define PhaseOff Low(PORTD, 6)
#define PhaseInv Inv(PORTD, 6)

#define Init	 2
#define On		 1
#define Off		 0

#define Start	 1
#define Stop	 0

#define	Tension		1
#define Frequency 	0

#define Forward	    1
#define Reporcial 	0

#define FreqArraySize	150
#define TensArraySize   50
#define RxBufferSize    100
#define TxBufferSize	100

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
	unsigned int ms16, ms16s, ms160, ms992;
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
	bool addendumChanged;
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
	float period, frequency, bufFrequency, pulseCount;
	unsigned long int ticksCurrent,ticksPrevious,ticks;
	unsigned long int overflows,ticksBuffer;
	unsigned short method;
	bool done, zero;	
} Measure;

volatile struct 
{
	unsigned short stopDelay, startDelay, delayCount;
	bool stateChanged, stateChanging;
	enum States
	{
		Acceleration,
		Deceleration,
		Regulation,
		Waiting
	} state;		
} AutoMode;

volatile struct
{
	float tension;
	signed int value;
	unsigned short done;
} Convert;

volatile struct
{
	unsigned char byte;
	unsigned short byteReceived, dataHandled;
} Rx;

ISR(TIMER0_OVF_vect)
{
	MainTimer.ms16++;
	MainTimer.ms16s++;

	if (MainTimer.ms16 >= 10)
	{
		if ((Menu.mode == Manual) || (Menu.mode == Auto && !(AutoMode.state == Waiting))) MainTimer.ms160++;
		MainTimer.ms16 = 0;
	}
	
	if (MainTimer.ms16s >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16s = 0;
	}
	
	TCNT0 = 5;	
}

ISR(TIMER1_OVF_vect)
{
	if (Measure.method == Forward) return;
	Measure.overflows++;
	
	if (Measure.overflows > 2)
	{
		Measure.zero = true;
		Measure.done = true;
	}
}

ISR(TIMER1_CAPT_vect)
{
	if (Measure.method == Reporcial)
	{
		Measure.ticksBuffer = ICR1;
		Measure.done = true;
		return;
	}
	
	Measure.pulseCount++;
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

ISR(ADC_vect)
{
	ADCSRA |= (0<<ADSC);
	Convert.value = ((signed)ADCW-21);
	Convert.done++;
}

ISR(USART0_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;
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

void Timer2(bool enable)
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

void Converter(unsigned short option)
{
	switch (option)
	{
		case 0:
			ADCSRA |= (0<<ADSC);
			break;
		case 1:
			ADCSRA |= (1<<ADSC);
			break;
		default:
			ADCSRA = 0x8F;
			ADMUX = 0x40;
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
			UCSR0B = (0 << RXEN0) | (0 << TXEN0) | (0 << RXCIE0);
			UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
			UBRR0L = 0;
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
	static char buffer[TxBufferSize];
	static char tension[20], frequency[20];
	
	if (Menu.mode == Manual && !Phase) return;
	
	memset(buffer, 0, TxBufferSize);
	
	sprintf(frequency, "F%.1f$", DDS.setting);
	sprintf(tension, "Tn%.1f", Convert.tension);
	strcat(buffer, frequency);
	strcat(buffer, tension);
	
	TxString(buffer);
}

float Kalman(float value, bool reset)
{
	static float measureVariation = 40, estimateVariation = 0.20, speedVariation = 0.009;
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

float MovAvgFrq(float value, bool reset)
{
	static float values[TensArraySize];
	static unsigned short index = 0;
	static float result;
	
	if (reset)
	{
		for (int i=0; i < TensArraySize; i++) values[i] = 0;
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % TensArraySize;
	
	return result/TensArraySize;
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
		if (DDS.setting < 0.1) PhaseOff; else PhaseOn;
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
	static char tension[20];
	static char addendum[10];
	static char multiplier[10];

	if (Menu.mode == Main) return;

	sprintf(setting, "%.1f Hz", DDS.setting);
	EraseUnits(0, 0, 3, DDS.setting);
	lcd_gotoxy(0, 0);
	lcd_puts(setting);
	
	sprintf(tension, "%.1f cN", Convert.tension);
	EraseUnits(0, 1, 3, Convert.tension);
	lcd_gotoxy(0, 1);
	lcd_puts(tension);
	
	if (Encoder.addendumChanged)
	{
		sprintf(addendum, Menu.mode == Manual ? "%.1f" : "%.3f", Encoder.addendumValues[Encoder.addendum]);
		EraseUnits(10, 1, 0, Encoder.addendumValues[Encoder.addendum]);
		lcd_gotoxy(10, 1);
		lcd_puts(addendum);
		Encoder.addendumChanged = false;
	}
	
	if (Menu.mode == Manual) return;
	
	sprintf(multiplier,"x%.3f",Encoder.multiplier);
	EraseUnits(9, 0, 0, Encoder.multiplier);
	lcd_gotoxy(9,0);
	lcd_puts(multiplier);
}

void Calculation(unsigned short parameter)
{
	if (parameter == Tension)
	{
		if (AutoMode.state == Deceleration)
		{
			Convert.tension = MovAvgFrq(0, false);
			return;
		}
		
		if (AutoMode.state == Waiting)
		{
			Convert.tension = MovAvgFrq(0, false);
			return;
		}
		
		Convert.tension = Menu.mode == Main ? 0 : MovAvgFrq(Convert.value < 1 ? 0 : (Convert.value*0.0048828125)*2908.f, 0);
		return;
	}
	
	if (Measure.method == Reporcial)
	{
		if (AutoMode.state == Deceleration)	
		{
			Measure.frequency = Kalman(0, false);
			return;
		}
		
		Measure.ticksCurrent = ((Measure.overflows * 65536L) + Measure.ticksBuffer) - Measure.ticksPrevious;
		Measure.ticksPrevious = Measure.ticksBuffer;
		Measure.period = Measure.ticksCurrent*0.000004;
		Measure.bufFrequency = Measure.period >= 1 ? 0 : Measure.period <= 0 ? 0 : 1.f/Measure.period;
		Measure.frequency = Kalman(Measure.zero > 0 ? 0 : Measure.bufFrequency < 1 ? Measure.frequency : Measure.bufFrequency, false);
		Measure.overflows = 0;
		Measure.zero = false;
		
		return; // return 65536 - (Measure.ticksCurrent + 10); for generation during got period (timer 3 must be On)
	}
	
	Measure.frequency = MovAvgFrq((float)Measure.pulseCount*5.f, 0);
	Measure.pulseCount = 0;
	return;	 // 0;
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
	PhaseOff;
	LedOff;
	Menu.mainActive = true;
	Timer1(false);
	Timer2(false);
	USART(Off);
	Kalman(0, true);
	MovAvgFrq(0, true);
	DDS.setting = 0;
	DDS.increment = 0;
	Convert.value = 0;
	Convert.tension = 0;
	lcd_clrscr();
	lcd_home();
	lcd_gotoxy(1, 0);
	lcd_puts(Menu.modeNames[Manual]);
	lcd_gotoxy(1, 1);
	lcd_puts(Menu.modeNames[Auto]);
	SetArrow(0);
}

void ManualHandle()
{
	EncoderHandler();

	if (Menu.manualActive) return;
	Encoder.addendumValues[one] = 0.1;
	Encoder.addendumValues[ten] = 1;
	Encoder.addendumValues[hundred] = 10;
	Encoder.addendumValues[thousand] = 100;
	Encoder.addendum = one;
	lcd_clrscr();
	lcd_home();
	Timer2(true);
	USART(On);
	Menu.manualActive = true;
	Encoder.addendumChanged = true;
}

bool AutoInit()
{
	Menu.autoActive = true;
	Encoder.addendumChanged = true;
	Encoder.addendumValues[one] = 1;
	Encoder.addendumValues[ten] = 0.1;
	Encoder.addendumValues[hundred] = 0.01;
	Encoder.addendumValues[thousand] = 0.001;
	Encoder.addendum = one;
	AutoMode.state = Waiting;
	AutoMode.stateChanging = false;
	AutoMode.stateChanged = false;
	AutoMode.startDelay = 10; // eeprom_read_dword(2);
	AutoMode.stopDelay = 10;  // eeprom_read_dword(3);
	AutoMode.delayCount = 0;
	Encoder.multiplier = eeprom_read_float((float*)1);
	lcd_clrscr();
	lcd_home();	
	return true;
}

void AutoHandle()
{
	if (!Menu.autoActive) Menu.autoActive = AutoInit();
	
	EncoderHandler();
	
	if (Measure.done)
	{
		Calculation(Frequency);
		SetOptionDDS(0);
		Measure.done = 0;
	}
	
	if (Activity && AutoMode.state == Waiting)	
	{
		AutoMode.state = Acceleration;
		Timer1(true);
		Timer2(true);
		USART(On);
	}
	
	if ((AutoMode.state == Acceleration) && (AutoMode.delayCount > AutoMode.startDelay))
	{
		AutoMode.state = Regulation;
		AutoMode.delayCount = 0;
		PhaseOn;
		LedOn;
	}
	
	if (!Activity && AutoMode.state == Regulation)
	{
		AutoMode.state = Deceleration;		
	}
	
	if ((AutoMode.state == Deceleration) && (AutoMode.delayCount > AutoMode.stopDelay))
	{
		PhaseOff;
		LedOff;
		Timer1(false);
		Timer2(false);
		USART(Off);
		Kalman(0, true);
		MovAvgFrq(0, true);
		AutoMode.state = Waiting;
		AutoMode.delayCount = 0;
		DDS.setting = 0;
		Measure.frequency = 0;
		Convert.value = 0;
		Convert.tension = 0;
	}
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
				Menu.lcd = 0;
			}
		}
	}
}

void Initialization(unsigned int method)
{
	DDRB = 0b00101110;
	PORTB = 0b00010001;
	
	DDRC = 0b00110000;
	PORTC = 0b01000000;
	
	DDRD = 0b11000010;
	PORTD = 0b00111111;

	strcpy(Menu.modeNames[0], "Manual"); // where aString is either an array or pointer to char
	strcpy(Menu.modeNames[1], "Auto");
	strcpy(Menu.modeNames[2], "Main");

	Menu.arrowPosition = 0;
	Menu.resetHold = 0;
	Menu.mode = Main;
	
	Measure.method = method;
	Measure.done = 0;
}

void MenuReset()
{
	if (Menu.resetDelay) Menu.resetCount++;
	if (Menu.resetCount >= 3) Menu.resetDelay = false;
	if (Menu.mode == Main) return;
	if (Enter && Menu.resetHold > 0) Menu.resetHold = 0;
	if (!Enter) Menu.resetHold++;
	if (Menu.resetHold >= 3)
	{
		SetOption(Main);
		Menu.resetDelay = true;
	}
}

int main(void)
{
	Initialization(Reporcial);
	lcd_init(LCD_DISP_ON);
	lcd_led(LCD_CLR);
	lcd_clrscr();
	lcd_home();
	Timer0(true);
	Converter(Init);
	USART(Init);
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
		
		if (Convert.done)
		{
			Calculation(Tension);
			Convert.done = 0;
		}
		
		if (MainTimer.ms160)
		{
			Converter(Start);
			Transmit();
			MainTimer.ms160 = 0;
		}
		
		if (MainTimer.ms992)
		{
			DisplayPrint();
			MainTimer.ms992 = 0;
			
			if (Menu.mode == Auto && ((AutoMode.state == Acceleration) || (AutoMode.state == Deceleration)))
			{
				AutoMode.delayCount++;
				LedInv;
			} 			
			
			MenuReset();
		}
    }
}