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

#define Enable	 (!(PINL & (1<<2)))
#define Disable	 (PINL & (1<<2))

#define DDSOut	  (PORTB & (1<<1))
#define DDSOutInv Inv(PORTB, 1)

#define Phase    (PORTB & (1<<2))
#define PhaseOn  PORTB |= (1<<2)
#define PhaseOff PORTB &= (0<<2)
#define PhaseInv PORTB ^= (1<<2)

#define True	 1
#define False	 0

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
	unsigned short X,Y;
	unsigned short arrowPosition, reset;
	unsigned short manualActive, autoActive, mainActive;
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
	if (direction > 0) DDS.setting += DDS.setting + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ? Encoder.addendumValues[Encoder.addendum] : 0;
	if (direction < 0) DDS.setting -= DDS.setting - Encoder.addendumValues[Encoder.addendum] < 0 ? DDS.setting : Encoder.addendumValues[Encoder.addendum];  
	DDS.increment = GetAddendum();
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
				Encoder.addendumChanged = True;
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
	static char setting[20], addendum[10];

	if (Menu.mode == Manual)
	{
		sprintf(setting, "%.1f Hz", DDS.setting);
		EraseUnits(0, 0, 3, DDS.setting);
		lcd_gotoxy(0, 0);
		lcd_puts(setting);
		
		if (Encoder.addendumChanged)
		{
			sprintf(addendum, "x%.f", Encoder.addendumValues[Encoder.addendum]);
			EraseUnits(0, 1, 0, Encoder.addendumValues[Encoder.addendum]);
			lcd_gotoxy(0, 1);
			lcd_puts(addendum);
			Encoder.addendumChanged = False;
		}
	}
}

void ManualHandle()
{
	EncoderHandler();

	if (Menu.manualActive) return;
	Encoder.addendum = one;
	Encoder.addendumValues[one] = 1;
	Encoder.addendumValues[ten] = 10;
	Encoder.addendumValues[hundred] = 100;
	Encoder.addendumValues[thousand] = 1000;
	DDS.setting = 0;
	SetOptionDDS(0);
	lcd_clrscr();
	lcd_home();
	Timer2(On);
	Menu.manualActive = True;
	Encoder.addendumChanged = True;
}

void AutoHandle()
{
	if (Menu.autoActive) return;
	lcd_clrscr();
	lcd_home();
	lcd_puts("Auto mode");
	Menu.autoActive = True;	
}

void SetOption(enum Modes mode)
{	
	Menu.mainActive = False;
	Menu.manualActive = False;
	Menu.autoActive = False;
	Menu.mode = mode;
	Menu.reset = 0;
	DDS.setting = 0;
	Timer2(Off);
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

void MenuHandle()
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
			if (Encoder.button == 1)
			{
				SetOption(Menu.arrowPosition);
			}
		}
	}
		
	if (Menu.mainActive) return;
	Timer2(Off); 
	lcd_clrscr();
	lcd_home();
	SetArrow(0);
	lcd_puts(Menu.modeNames[Manual]);
	lcd_gotoxy(1,1);
	lcd_puts(Menu.modeNames[Auto]);
	Menu.mainActive = True;	
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
	Menu.reset = 0;
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
	Timer2(Off);
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
				MenuHandle();
				break;
			default:
				lcd_clrscr();
				lcd_home();
				lcd_puts("Range out");
				lcd_gotoxy(0, 1);
				lcd_puts("Reboot pls");
				break;
		}
		
		if (MainTimer.ms1000)
		{
			LedInv;
			DisplayPrint();
			MainTimer.ms1000 = 0;
			
			if (Menu.mode == Main) continue;
			if (Enter && Menu.reset > 0) Menu.reset = 0; 
			if (!Enter) Menu.reset++;
			if (Menu.reset >= 5) SetOption(Main);
		}
    }
}























/*

#define F_CPU   16000000UL

#define Led     PORTB &  (1<<7);
#define OnLed   PORTB |= (1<<7);
#define OffLed  PORTB &= (0<<7);
#define InvLed  PORTB ^= (1<<7);

#define OnADC   ADCSRA |= (1<<ADSC);
#define OffADC  ADCSRA |= (0<<ADSC);

#define SwPort  PORTF

#define Right   (~PINC & (1<<0))
#define Left    (~PINC & (1<<1))      
#define Enter   (PINC & (1<<2))            

#include <mega2560.h>
#include <mega2560_bits.h>
#include <stdio.h>
#include <delay.h>
#include <stdlib.h>
#include <string.h>
#include <alcd.h>
#include <stdint.h>
#include <spi.h>
#include "eeprom.h"

const unsigned long int ACCUM_MAXIMUM = 1875000000; // original 1875000000 / divider 132720000
const unsigned int FREQUENCY_MAXIMUM = 31250;
const int arrow = 62;

struct 
{
    unsigned short X,Y;
    unsigned short arrowPosition; 
    unsigned short mainActive,manualActive,autoActive,parametersActive;
    char modeNames[4][10];

    enum Modes
    {  
        Manual,
        Auto,
        Parameters,
        Reset,
        Main
    } mode;
    
    enum Modes preselect;
} Menu;

struct 
{
	unsigned int ms200,ms1000;	
} MainTimer;

struct 
{
    unsigned int setting;
	unsigned long int accum, increment;
} DDS;

struct 
{
	unsigned long int frequency,previousFrequency;
	unsigned long int multiplier,interruptCount,impulsesCount;
} Measure;

struct
{
    unsigned short forward;
    unsigned short backward;
    unsigned short button;
    unsigned short reducerSetting;
    unsigned short reducer;
    unsigned short addendumValues[4];
    
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
    float multiplier;
    float addendumValues[4];
    unsigned short clear;
    unsigned short update;
} Autotune;

struct 
{
	unsigned long int ticksCurrent,ticksPrevious;
    unsigned int overflows; 
    unsigned short action;
    float frequency,period;
} MeasureFrequency;

volatile unsigned short transientComplete = 1;
volatile short loadPosition;

interrupt [TIM0_OVF] void t0_isr (void)
{
	Measure.interruptCount++;
}

interrupt [TIM1_OVF] void t1_isr (void)
{
	TCNT1 = 62411;
	MainTimer.ms200++;
//    Autotune.update++;
	if (!transientComplete) loadPosition++; 
    
	if (MainTimer.ms200 >= 5)
	{   
        if (!Autotune.clear) Autotune.clear = 1;
		Measure.impulsesCount = TCNT0;
		Measure.multiplier = Measure.interruptCount;
		TCNT0 = 0;
		MainTimer.ms200 = 0;
		MainTimer.ms1000++;
	}
}

interrupt [TIM5_OVF] void t5_isr (void)
{
	TCNT5 = 65535;
    
    MeasureFrequency.frequency++;
    
	DDS.accum += DDS.increment;
	
	if (DDS.accum >= ACCUM_MAXIMUM)
	{
		DDS.accum -= ACCUM_MAXIMUM;
		PORTB^=(1<<0);
	}
}

void init_timer0(void)
{
	TCCR0B = (1<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = 1;
}

void init_timer1(void)   // set initialization 8 bit timer 0
{
	TCNT1 = 62411;  // 62411 = 200 ms
	TCCR1B = 5;   // scaler 10 bit
	TIMSK1 = 1;
}

void init_timer5(void)   // set initialization 16 bit timer 5
{
	TCCR5B = (1<<CS52) | (0<<CS51) | (0<<CS50);  // scaler 64
	TIMSK5 = 1;
	TCNT5 = 0;
}

void Plug(unsigned short run)
{
    if (!run) return;
    lcd_gotoxy(0,0);
    lcd_puts("In developing");
}

void EraseUnits(int x, int y, int offset, float count)
{
	char eraser = 32;
	
    if (count<1000000000 || count < 0)
    {
        lcd_gotoxy(x+offset+9,y);
        lcd_putchar(eraser);
    }
    
    if (count<100000000 || count < 0)
    {
        lcd_gotoxy(x+offset+8,y);
        lcd_putchar(eraser);
    }
    
    if (count<10000000 || count < 0)
    {
        lcd_gotoxy(x+offset+7,y);
        lcd_putchar(eraser);
    }
    
    if (count<1000000 || count < 0)
    {
        lcd_gotoxy(x+offset+6,y);
        lcd_putchar(eraser);
    }
    
	if (count<100000 || count < 0)
	{
		lcd_gotoxy(x+offset+5,y);
		lcd_putchar(eraser);
	}
	
	if (count<10000)
	{
		lcd_gotoxy(x+offset+4,y);
		lcd_putchar(eraser);
	}
	
	if (count<1000)
	{
		lcd_gotoxy(x+offset+3,y);
		lcd_putchar(eraser);
	}
	
	if (count<100)
	{
		lcd_gotoxy(x+offset+2,y);
		lcd_putchar(eraser);
	}
	
	if (count<10)
	{
		lcd_gotoxy(x+offset+1,y);
		lcd_putchar(eraser);
	}
}

void ManualPrint(void)
{
    char setting[10], frequency[10], addendum[10];

    EraseUnits(0, 0, 3, DDS.setting);
	sprintf(setting,"%.lu Hz",DDS.setting);
	lcd_gotoxy(0,0);
	lcd_puts(setting);
    
    EraseUnits(11, 0, 0, Encoder.addendumValues[Encoder.addendum]);
	sprintf(addendum,"x%.u",Encoder.addendumValues[Encoder.addendum]);
	lcd_gotoxy(11,0);
	lcd_puts(addendum);
	
	EraseUnits(0, 1, 3, MeasureFrequency.frequency);
	sprintf(frequency,"%.lu Hz",MeasureFrequency.frequency);
	lcd_gotoxy(0,1);
	lcd_puts(frequency);
}

void AutoPrint(void)
{
    char frequency[10], multiplier[10], setting[10], addendum[10];
    
    EraseUnits(0, 0, 3, Measure.frequency);
	sprintf(frequency,"%.lu Hz",Measure.frequency);
	lcd_gotoxy(0,0);
	lcd_puts(frequency);


    EraseUnits(9, 0, 0, Autotune.multiplier);
	sprintf(multiplier,"x%.3f",Autotune.multiplier);
	lcd_gotoxy(9,0);
	lcd_puts(multiplier);
    
    EraseUnits(0, 1, 3, DDS.setting);
	sprintf(setting,"%.lu Hz",DDS.setting);
	lcd_gotoxy(0,1);
	lcd_puts(setting);
    
    EraseUnits(9, 1, 0, Autotune.addendumValues[Encoder.addendum]);
	sprintf(addendum,"x%.3f",Autotune.addendumValues[Encoder.addendum]);
	lcd_gotoxy(9,1);
	lcd_puts(addendum);	
}

unsigned long int GetAddendum(void)
{
    unsigned int divider = DDS.setting < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.setting)/31250)*divider;    
}

void SwitchDevice(void)
{
    SwPort = SwPort == 0b10000000 ? 0x01 : SwPort << 1;    
}

unsigned short WaitManualTransientProcesses()
{
    char cell = 0xFF;

    if (loadPosition < 0) return 0;

    if (loadPosition >= 16) 
    {
        lcd_clear();
        loadPosition = 0;
        return 1;
    }        
    
    lcd_gotoxy(loadPosition,1); 
    lcd_putchar(cell);
    return 0;         
}

void SetOptionDDS(short direction)
{
    DDS.setting += direction > 0 ?
        DDS.setting + Encoder.addendumValues[Encoder.addendum] <= FREQUENCY_MAXIMUM ?
            Encoder.addendumValues[Encoder.addendum] : FREQUENCY_MAXIMUM - DDS.setting
        :
        DDS.setting >= Encoder.addendumValues[Encoder.addendum] ? 
            Encoder.addendumValues[Encoder.addendum]*-1 : DDS.setting*-1; 
  
    DDS.increment = GetAddendum();  
}

void SetOptionAutoDDS(short direction)
{
    if (direction)
    {
        Autotune.multiplier += direction > 0 ?
            Autotune.addendumValues[Encoder.addendum]
            :
            Autotune.addendumValues[Encoder.addendum]*-1; 
    }
        
    DDS.setting = Measure.frequency *Autotune.multiplier; 
    DDS.increment = GetAddendum(); 
}

void ManualInitialization(unsigned short reducer, enum Addendums addendum, unsigned int setting, short position)
{  
    unsigned int index = 0;
    
    init_timer0();
	init_timer1();
    init_timer5();
    #asm("sei");
    
    Encoder.addendumValues[index] = 1;
    for (index=1;index<sizeof(Encoder.addendumValues);index++) 
        Encoder.addendumValues[index] = Encoder.addendumValues[index-1] * 10; 
    
    lcd_gotoxy(1,0);
    lcd_puts("Initialization"); 
    
    Encoder.reducerSetting = reducer;
    DDS.setting = setting;
    DDS.increment = GetAddendum();
    Encoder.addendum = addendum;
    loadPosition = position; 
    transientComplete = 0;
    
    Menu.manualActive++;
}

void ManualHandler(void)
{
    if (transientComplete) 
    {
        if (Encoder.reducer >= Encoder.reducerSetting)
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
                    }
                }
            } 
         
            Encoder.reducer = 0;
        }
                    
        Encoder.reducer++;
    }
                    
    if (MainTimer.ms1000 > 0)
    {        
        TCNT0 = 0;
        Measure.frequency = (Measure.multiplier*256L)+Measure.impulsesCount;
        Measure.multiplier = 0;
        Measure.interruptCount = 0;
        MainTimer.ms1000 = 0;
        if (transientComplete) ManualPrint();
    }
                
    if (!transientComplete)
    {
        transientComplete = WaitManualTransientProcesses();
    }    
}

void AutoInitialization(unsigned short reducer, enum Addendums addendum)
{
//    unsigned int index = 0;
    
    init_timer0();
	init_timer1();
    init_timer5();
    #asm("sei");
    
//    Autotune.addendumValues[index] = 1.f;
//    for (index=1;index<sizeof(Autotune.addendumValues);index++) 
//        Autotune.addendumValues[index] = (float) Autotune.addendumValues[index-1] / 10;
 
    Autotune.addendumValues[0] = (float)1;
    Autotune.addendumValues[1] = (float)0.1;
    Autotune.addendumValues[2] = (float)0.01;
    Autotune.addendumValues[3] = (float)0.001;
    
    Encoder.reducerSetting = reducer;
    Encoder.addendum = addendum;
    Autotune.clear = 0;
    
    Menu.autoActive++;
    
    SetOptionDDS(1);
}

void AutoHandler(void)
{
    Encoder.reducer++;
    
    if (Encoder.reducer >= Encoder.reducerSetting)
    {                         
        if (Right) Encoder.forward = 0;
        {                                  
            if (!Right) Encoder.forward++;
            {  
                if (Encoder.forward == 1 && Left)
                {
                    SetOptionAutoDDS(1);
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
                    SetOptionAutoDDS(-1);
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
                }
            }
        } 
         
        Encoder.reducer = 0;
    }
     
    if (Autotune.update)
    {   
        SetOptionAutoDDS(0);
        Autotune.update = 0;
    }
         
    if (MainTimer.ms1000 > 0)
    {  
        TCNT0 = 0;
        Measure.previousFrequency = Measure.frequency;
        Measure.frequency = (Measure.multiplier*256L)+Measure.impulsesCount;  
        
        if (((Measure.frequency+1) == Measure.previousFrequency) || 
            ((Measure.frequency-1) == Measure.previousFrequency) ||
            (Measure.frequency == Measure.previousFrequency)) Autotune.update = 0;\
        else Autotune.update = 1;
        
        Measure.multiplier = 0;
        Measure.interruptCount = 0;
        MainTimer.ms1000 = 0; 
        if (Autotune.clear == 1) 
        {   
            Autotune.clear++;
            lcd_clear();
        } 
        if (Autotune.clear > 1) AutoPrint();
        MeasureFrequency.frequency = 0;
    }     
}

void SetArrow(short stepDirection)  // соотнести позицию в соответствии с нумератором пунктов меню
{
    char eraser = 32;

    lcd_gotoxy(0,Menu.arrowPosition);
    lcd_putchar(eraser);   
    
    Menu.arrowPosition += stepDirection > 0 ? 
        Menu.arrowPosition < 1 ? stepDirection : 0 
        :
        Menu.arrowPosition > 0 ? stepDirection : 0;
    
    lcd_gotoxy(0,Menu.arrowPosition);
    lcd_putchar(arrow);
}

void SetOption(enum Modes mode)
{
//    int index = 0;                                     
    
//    for (index = 0; index < sizeof(Menu.mode); index++)  check enum exist
//    {
//        if (index == mode) 
//        {
//            Menu.arrowPosition = 0;
//            Menu.mainActive = 0; 
//            Menu.mode = mode;  
//            break;  
//        }
//    }
    
    Menu.arrowPosition = 0;
    Menu.mainActive = 0; 
    Menu.mode = mode;
    lcd_clear();    
}

void MainMenu(void)
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
            if (Encoder.button == 1)
            {       
                SetOption(Menu.arrowPosition);            
            }
        }
    }            
}

void MenuInitialization(void)
{
    strcpy(Menu.modeNames[0], "Manual"); // where aString is either an array or pointer to char
    strcpy(Menu.modeNames[1], "Auto");
    strcpy(Menu.modeNames[2], "Parameters");
    strcpy(Menu.modeNames[3], "Reset"); 
    
    Menu.arrowPosition = 0;
    Menu.mode = Main;    
}  

void main(void)
{
	DDRA = 0xFF;                  
	PORTA = 0x00;
    
    DDRB = 0xFF;
    PORTB = 0x00;
	
	DDRC = 0x00;
	PORTC = 0xFF;
	
	DDRD = 0x00;
	PORTD = 0xFF;
    
    DDRF = 0xFF;
    PORTF = 0x01; 
    
    DDRL = 0x00;
    PORTL = 0xFF; 
    
	SwitchDevice();
    
	lcd_init(16);
	lcd_clear();
    lcd_gotoxy(0,0);
     
    Plug(0);
    MenuInitialization();
    
	while(1)
	{   
        switch (Menu.mode)
        {
            case Manual: 
                if (!Menu.manualActive) ManualInitialization(25, one, 0, -1);
                ManualHandler();
                break;
            case Auto:
                if (!Menu.autoActive) AutoInitialization(25,one);
                AutoHandler();                
                break;
            case Parameters:
            
                break;
            case Reset:
            
                break;
            default:
                #asm("cli");
                if (!Menu.mainActive)
                {     
                    SetArrow(0);
                    lcd_puts(Menu.modeNames[0]);
                    lcd_gotoxy(1,1);
                    lcd_puts(Menu.modeNames[1]);
                    Menu.mainActive++;
                }
                MainMenu();
                break;
        }
	}
}

*/