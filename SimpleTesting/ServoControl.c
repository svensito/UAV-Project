/*
 * ServoControl.c
 *
 * Created: 12.04.2013 11:40:02
 *  Author: snfu
 */ 

#include "ServoControl.h"
#include <avr/io.h>
#include <avr/interrupt.h>



// With these settings we know that we have 1 step per 2° of Servo Angle change

volatile long POSITION1 = NEUTRAL;
volatile long POSITION2 = NEUTRAL;
volatile long POSITION3 = NEUTRAL;
volatile long POSITION4 = NEUTRAL;
volatile long POSITION5 = FLAP_UP; // This is the FLAP Servo with particular neutral setting...

volatile long ctrl_out[5]={NEUTRAL,NEUTRAL,NEUTRAL,NEUTRAL,FLAP_UP};

int8_t ctrl = 0;	// valid are 0 to 2*servo channels (5) => 10
// used for reading the remote control
uint8_t channel = 0;				// variable for the channels
int first_pulse = FALSE;			// variable to tag the first pulse
volatile int ctrl_in[9] = {0,0,0,0,0,0,0,0,0};	// array for writing the counter values
int Timer_time = 0;

void servo_init()
{
	// ---------------------
	// PWM Timer for the Servos
	// REMEMBER: Whenever Interrupt Service Routines are configured, 
	// we need to disable the Interrupts and enable them afterwards!
	// Remark: here we do it globally in the Simple Testing Init
	
	// Defining the Output Ports
	DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB4);
	
	PORTB &= ~((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4));		// All servo signals are pulled down initially
	
	// Using Timer 1 (16 bit Timer)
	// Prescaler = 8	(CS Bits)
	// CTC Mode	(WGM10 - WGM13)
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
	
	// TIMSK is set in the main routine, as problems were discovered when not setting this globally
	// TIMSK  = (0<<TICIE1)|(1<<OCIE1A)|(0<<OCIE1B)|(0<<TOIE1);
	OCR1A = NEUTRAL;	// Give Neutral position in the beginning
	// -----------------------
	
	// -----------------------
	// INPUT CONTROL 
	// We use interrupt Pin INT0 to control the TIMER2
	
	// General Interrupt Control Register
	// setting one of the bits enables the Interrupt Request
	GICR = (0<<INT1)|(1<<INT0)|(0<<INT2);
	
	// MCU Control Register to activate INT0 Pin
	// ISC11, ISC10 used for INT1 control
	// ISC01, ISC00 used for INT0 control	-> setting both to 1, gives rising edge interrupt
	MCUCR = (0<<ISC11)|(0<<ISC10)|(1<<ISC01)|(1<<ISC00);
	
	// TCCR2
	// we activate the timer 2 in normal mode
	// with Prescaler of 128 we achieve an overflow at 2,7ms
	// and the signal from the receiver has a resolution of 0.01ms/bit
	TCCR2 = (0<<WGM20)|(0<<WGM21)|(1<<CS22)|(0<<CS21)|(1<<CS20);
	TCNT2 = 0;
	write_string_ln("ServoTimerStarted");
		
}

ISR(TIMER1_COMPA_vect)
{
	switch (ctrl)
	{
		// We might need to disable interrupts and enable them again...
		
		case 0:
			//cli();
			PORTB |= (1<<PB0);
			OCR1A = ctrl_out[motor];
			ctrl++;		// increase the ctrl pointer
			//sei();
		break;
		
		case 1:
			//cli();
			PORTB &= ~(1<<PB0);
			OCR1A = PULSE_WIDTH - ctrl_out[motor];
			ctrl++;
			//sei();
		break;
		
		case 2:
			//cli();
			PORTB |= (1<<PB1);
			OCR1A = ctrl_out[aileron];
			ctrl++;
			//sei();
		break;
		
		case 3:
			//cli();
			PORTB &= ~(1<<PB1);
			OCR1A = PULSE_WIDTH - ctrl_out[aileron];
			ctrl++;
			//sei();
		break;
		
		case 4:
			//cli();
			PORTB |= (1<<PB2);
			OCR1A = ctrl_out[elevator];
			ctrl++;
			//sei();
		break;
		
		case 5:
			//cli();
			PORTB &= ~(1<<PB2);
			OCR1A = PULSE_WIDTH - ctrl_out[elevator];
			ctrl++;
			//sei();
		break;
		
		case 6:
			//cli();
			PORTB |= (1<<PB3);
			OCR1A = ctrl_out[rudder];
			ctrl++;
			//sei();
		break;
		
		case 7:
			//cli();
			PORTB &= ~(1<<PB3);
			OCR1A = PULSE_WIDTH - ctrl_out[rudder];
			ctrl++;
			//sei();
		break;
		
		case 8:
			//cli();
			PORTB |= (1<<PB4);
			OCR1A = ctrl_out[flap];
			ctrl++;
			//sei();
		break;
		
		case 9:
			//cli();
			PORTB &= ~(1<<PB4);
			OCR1A = PULSE_WIDTH - ctrl_out[flap];
			ctrl= 0;	// reset the ctrl pointer
			//sei();
		break;
	}
	
}

ISR(INT0_vect)
{
	if(first_pulse == TRUE)
	{
		first_pulse = FALSE;
	}
	else
	{
		//write_ubyte_ln(channel);
		ctrl_in[channel] = TCNT2;
		channel++;
	}
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{
	channel = 0;
	first_pulse = TRUE;
	//write_string_ln("OVF");
}