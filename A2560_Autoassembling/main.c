/*
 * main.c
 *
 * Created: 1/25/2022 1:10:22 PM
 *  Author: igor.abramov
 */ 

/* ****************Description************** 

     Fword = (ACCUM*F)/(Fosc/4/T5prescaler)
     Fmax = Fosc/2/T5prescaler

*/

#define F_CPU    16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led     Check(PORTB, 7)
#define LedOn   High(PORTB, 7)
#define LedOff  Low(PORTB, 7)
#define LedInv  Inv(PORTB, 7)

#define Right    (~PINC & (1<<0))
#define Left     (~PINC & (1<<1))      
#define Enter    (PINC & (1<<2)) 

#define Enable	 (!(PINL & (1<<2)))
#define Disable	 (PINL & (1<<2)) 
 
#define DDSOut	 (PORTB & (1<<0))
#define DDSOutInv PORTB ^= (1<<0);
 
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
#define StringEnd	'\0'

#include <xc.h>
#include <avr/io.h>
#include <float.h>
#include "lcd/lcd.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <avr/eeprom.h>
 
const unsigned long int ACCUM_MAXIMUM = 1875000000; 
const unsigned int		FREQUENCY_MAXIMUM = 62500;
const unsigned short	startDelay = 10;
const unsigned short	stopDelay = 10;

volatile struct
{
	unsigned int ms40, ms200transmit, ms200, ms1000;
	unsigned short counter, start, stop, breaking;
} MainTimer;

volatile struct
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

volatile struct
{
    unsigned long int ticksCurrent,ticksPrevious,ticks;
    unsigned long int overflows,ticksBuffer;      
    unsigned short done, index, method, zero;
    float period, frequency, bufFrequency, pulseCount;	
} Measure;

volatile struct
{
    float setting;
    unsigned long int accum, increment, counter;
} DDS;

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
} Receive;

ISR(TIMER1_OVF_vect)
{
	//TCNT1 = 64911; // 64911 - 40 ms
	//MainTimer.ms40++;
	
	TCNT1 = 62411; // 62411 - 200 ms 
	MainTimer.ms200++;
	MainTimer.ms200transmit++;
	if (Enable) ADCSRA |= (1<<ADSC);
	
	if (MainTimer.ms200 >= 5 || MainTimer.ms40 >= 25)
	{
		MainTimer.ms40 = 0;
		MainTimer.ms200 = 0;
		MainTimer.ms1000++;
	}
}

ISR(TIMER3_OVF_vect)
{
	DDS.setting = 0;
	DDS.increment = 0;
	Measure.frequency = 0;
}

ISR(TIMER4_OVF_vect) 
{
	if (Measure.method == Forward) return; 
    Measure.overflows++;
	
	if (Measure.overflows >= 4) 
	{
		Measure.zero++;
		Measure.done++;
	}
}

ISR(TIMER4_CAPT_vect)
{  
	if (Measure.method == Reporcial) 
	{
		Measure.ticksBuffer = ICR4;
		Measure.done++;
		return;
	}
	
	Measure.pulseCount++;
}

ISR(TIMER5_OVF_vect)
{
    TCNT5 = 65535;
    
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
	Receive.byte = UDR0;
	Receive.byteReceived++;	
}

float FilterMovingAverageFrequency(float value, unsigned short reset)
 {
	 static float values[FrequencyArraySize];
	 static unsigned short index = 0;
	 static float result;
	 
	 if (reset)
	 {
		 for (int i=0; i < FrequencyArraySize; i++) values[i] = 0;
		 result = 0;
		 index = 0;
		 return 0;
	 }
	 
	 result += value - values[index];
	 values[index] = value;
	 index = (index + 1) % FrequencyArraySize;
	 
	 return result/FrequencyArraySize;
 }

float FilterMovingAverageTension(float value, unsigned short reset)
 {
	 static float values[TensionArraySize];
	 static unsigned short index = 0;
	 static float result;
	 
	 if (reset)
	 {
		 for (int i=0; i < TensionArraySize; i++) values[i] = 0;
		 result = 0;
		 index = 0;
		 return 0;
	 }
	 
	 result += value - values[index];
	 values[index] = value;
	 index = (index + 1) % TensionArraySize;
	 
	 return result/TensionArraySize;
 }

float FilterKalman(float value, unsigned short reset)
 {
	 static float measureVariation = 40, estimateVariation = 0.20, speedVariation = 0.003;
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

void Timer1(void)
{
    TCNT1 = 62411;  // 62411 = 200 ms
    TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);   // scaler 10 bit
    TIMSK1 = (1 << TOIE1);
}

void Timer3(unsigned short enable)
{
	if (enable)
	{
		TCNT3 = 0;
		TCCR3B = (0 << CS32)|(1 << CS31)|(1 << CS30);
		TIMSK3 = (1 << TOIE3);
		return;
	}
	
	TCCR3B = (0 << CS32)|(0 << CS31)|(0 << CS30);
	TIMSK3 = (0 << TOIE3);
}

void Timer4(unsigned short enable)
{
	if (enable)
	{
		TCCR4B = (1 << ICNC4)|(1 << ICES4)|(0 << CS42)|(1 << CS41)|(1 << CS40); 
		TIMSK4 = (1 << TOIE4)|(1 << ICIE4);
		return;
	}
	
	TCCR4B = (0 << CS42)|(0 << CS41)|(0 << CS40);
	TIMSK4 = (0 << TOIE4)|(0 << ICIE4);
}

void Timer5(unsigned short enable)
{
	if (enable)
	{
		TCCR5B = (1<<CS52) | (0<<CS51) | (0<<CS50);  // scaler 64
		TIMSK5 = (1<<TOIE5);
		return;
	}
	
	 TCCR5B = (0<<CS52) | (0<<CS51) | (0<<CS50);  // scaler 0
	 TIMSK5 = (0<<TOIE5);
	 TCNT5 = 0;
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

void EraseUnits(int x, int y, int offset, float count)
{
	char eraser = 32;
	
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

void DisplayPrint(void)
{
	static char multiplier[10];
	static char addendum[10];
	static char setting[10];
	static char tension[10];
	
	EraseUnits(0, 0, 1, DDS.setting);
	sprintf(setting, "F %.1f", DDS.setting);
	lcd_gotoxy(0, 0);
	lcd_puts(setting);
	
	EraseUnits(9, 0, 0, Encoder.multiplier);
	sprintf(multiplier,"x%.3f",Encoder.multiplier);
	lcd_gotoxy(9,0);
	lcd_puts(multiplier);
	
	EraseUnits(0, 1, 1, Convert.tension);
	sprintf(tension,"T %.1f",Convert.tension);
	lcd_gotoxy(0,1);
	lcd_puts(tension);
	
	sprintf(addendum,"%.3f",Encoder.addendumValues[Encoder.addendum]);
	lcd_gotoxy(10,1);
	lcd_puts(addendum);
}

void TransmitChar(unsigned char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TransmitString(const char* s)
{
	for (int i=0; s[i]; i++) TransmitChar(s[i]);
}

void Transmitting()
{
	static char buffer[TxBufferSize];
	static char tension[20], frequency[20];
	
	memset(buffer, 0, TxBufferSize);
	
	sprintf(frequency, "F%.1f$", DDS.setting);
	sprintf(tension, "Tn%.1f", Convert.tension);
	strcat(buffer, frequency);
	strcat(buffer, tension);
	
	TransmitString(buffer);
}

void Receiving()
{	
	static char buffer[RxBufferSize];
	static unsigned int index = 0;
	
	if (Receive.byte == Terminator)
	{
		buffer[index] = StringEnd;
		TransmitString(buffer);
		index = 0;	
		return;
	}
	
	buffer[index] = Receive.byte;
	index++;
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
            }
        }
    } 
}

void Initialization(unsigned short Method, enum Addendums addendum)
{
	LedOff;
	PhaseOff;
	
	Measure.method = Method;
	
	MainTimer.ms200 = 0;
	MainTimer.ms1000 = 0;
	MainTimer.start = 0;
	MainTimer.stop = 0;
	MainTimer.counter = 0;
	MainTimer.breaking = 0;
	
	Encoder.addendum = addendum;
	Encoder.addendumValues[0] = 1;
	Encoder.addendumValues[1] = 0.1;
	Encoder.addendumValues[2] = 0.01;
	Encoder.addendumValues[3] = 0.001;
	Encoder.multiplier = eeprom_read_float((float*)1);
}

void Calculation(unsigned short parameter)
{
	if (parameter == Tension)
	{
		Convert.tension = FilterMovingAverageTension(Convert.value < 1 ? 0 : (Convert.value*0.0048828125)*2908.f, 0);
		return;
	}
	
	if (Measure.method == Reporcial)
	{
		Measure.ticksCurrent = ((Measure.overflows * 65536L) + Measure.ticksBuffer) - Measure.ticksPrevious;
		Measure.ticksPrevious = Measure.ticksBuffer;
		Measure.period = Measure.ticksCurrent*0.000004;
		Measure.bufFrequency = Measure.period >= 1 ? 0 : Measure.period <= 0 ? 0 : 1.f/Measure.period;
		Measure.frequency = FilterKalman(Measure.zero > 0 ? 0 : Measure.bufFrequency < 1 ? Measure.frequency : Measure.bufFrequency, 0);
		Measure.overflows = 0;
		Measure.zero = 0;
		
		return; // return 65536 - (Measure.ticksCurrent + 10); for generation during got period (timer 3 must be On)
	}
	
	Measure.frequency = FilterMovingAverageFrequency((float)Measure.pulseCount*5.f, 0);
	Measure.pulseCount = 0;
	return;	 // 0;
}

void Reset()
{
	FilterMovingAverageFrequency(0, True);
	FilterMovingAverageTension(0, True);
	FilterKalman(0, True);
	
	Encoder.button = 0;
	Encoder.forward = 0;
	Encoder.backward = 0;
	
	Measure.method = 0;
	Measure.index = 0;
	Measure.done = 0;
	Measure.period = 0;
	Measure.frequency = 0;
	Measure.overflows = 0;
	Measure.pulseCount = 0;
	Measure.ticksCurrent = 0;
	Measure.ticksPrevious = 0;
	
	DDS.accum = 0;
	DDS.setting = 0;
	DDS.increment = 0;
	
	Convert.done = 0;
	Convert.value = 0;
	Convert.tension = 0;
}

void ModeDefiner()
{
	if (Enable && !MainTimer.start) 
	{
		LedOn;
		PhaseOn;
		
		Timer4(On);
		Timer5(On);
		USART(On);
		Converter(On);
		
		TCNT1 = 62411;
		MainTimer.ms200 = 0;
		MainTimer.counter = 0;
		MainTimer.stop = 0;	
		MainTimer.start++;	
	}
	
	if (Disable && !MainTimer.stop)
	{
		LedOff;
		PhaseOff;
		
		Timer4(Off);
		Timer5(Off);
		USART(Off);
		Converter(Off);
		Reset();
		
		MainTimer.stop++;
		MainTimer.counter = 0;
		MainTimer.start = 0;
	}
}

void ModeDelayer()
{
	if (MainTimer.start && (MainTimer.counter < startDelay));
	{
		LedInv;
		MainTimer.counter++;
	}

	if ((MainTimer.counter >= startDelay) && !Phase)
	{
		LedOn;
		PhaseOn;
	}
}

int main(void)
{
    DDRA = 0xFF;                  
    PORTA = 0x00;
    
    DDRB = 0xFF;
    PORTB = 0x00;
    
    DDRC = 0x00;
    PORTC = 0xFF;
    
    DDRL = 0x00;
    PORTL = 0xFF; 
	
	lcd_init(LCD_DISP_ON);
	USART(Init);
	Converter(Init);
	Initialization(Reporcial, thousand);
	Reset();
	Timer1();
    sei();
	
    while(1)
    {   
		ModeDefiner();
		EncoderHandler();

		if (MainTimer.ms1000 > 0)
		{
			DisplayPrint();
			MainTimer.ms1000 = 0;
		}

		if (Measure.done)
		{
			Calculation(Frequency);
			SetOptionDDS(0);
			Measure.done = 0;
		}

		if (Convert.done)
		{
			Calculation(Tension);
			Convert.done = 0;
		}

		if (Enable && MainTimer.ms200transmit)
		{
			Transmitting();
			MainTimer.ms200transmit = 0;
		}
    }
}