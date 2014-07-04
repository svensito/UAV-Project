/*
 * DifferentialPressure.c
 *
 * Created: 18.06.2013 15:43:52
 *  Author: SNFU
 */ 

/*	This is used to activate an ADC that is reading the differential pressure.
*	The differential pressure is used to determine the airspeed.
*/

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"
#include "DifferentialPressure.h"

//##############################
// variables
int16_t v_raw = 0;		// raw voltage value, comes as 10 bit value
volatile double speed = 0;		// speed as floating value
int16_t speed_tx = 0;	// we will calculate a speed to have an understanding of how fast the A/C is
double delta_p = 0;		// delta pressure
double density = 1.185;

int ADC_cal_flag = FALSE;
int16_t adc_offset = 0;

//##############################
						// The ADC will be connected to PIN PA0 (ADCO)
						// the VCC will need to be connected to 5VDC
						// there are several ways to determine the reference voltage
						// within ADMUX register those can be selected
						// by default we are at 00, this will use AREF as reference voltage

void ADC_start(void)
{
						// first the direction of the Port needs to be set to input:
	DDRA = 0x00;

						//DDRA = (0 << DDA0);	// this is implicitly defining Pin AO as an input

						// Datasheet ATMEGA page 216 / 217
						// Register: ADCSRA Register -> ADC Control Status Register A
						// Bit 7: ADEN	-> ADC Enable (enabling the ADC)
						// Bit 6: ADSC	-> ADC start conversion
						// Bit 5: ADATE	-> ADC auto trigger enable
						// Bit 4: ADIF	-> ADC interrupt flag
						// Bit 3: ADIE	-> ADC interrupt enable
						// Bit 2 - 0: ADPS	-> ADC Prescaler Bits: division factor between XTAL and input clock to ADC

						// The prescaler needs to be set to not overrun the ADC conversion frequency
						// Frequency should end up between 50 kHz and 200kHz
						// therefore we determine our minimum and maximum prescaler values:
						// Division factor min = F_CPU / 200 kHz
						// Division factor max = F_CPU / 50 kHz
						// with our CPU clock of 12MHz we will end up with: min 60	, max 240
						// we choose a prescaler of: 128 ( as per datasheet page 217 -> setting all three bits)
						
						// Within the ADMUX Register we need to set which Input is actually needed.
						// We connected the Diff Pressure to PA3 (???)
						
						// The REFS0 needs to be set to 1, otherwise there will be terrible noise and not useable data!
	ADMUX = (0 << REFS1)|(1 << REFS0)|(0 << MUX4)|(0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(1 << MUX0);
													
	ADCSRA = (1 << ADEN)|(1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0)|(0 << ADATE)|(0 << ADIF)|(0 << ADIE);
	
	write_string_ln("ADC ok");
}

						// when ADATE is set, then the unit will look for the Bits in SFIOR, ADTS2 - ADTS0
						// as ADTS2 - ADTS0 are set to 000 by default, the unit is in free running mode -> 
						// will permanently read and convert the input

						// Output Register:
						// ADCL
						// ADCH
						// if ADLAR = 0 (as default) the result is right adjusted -> 10 bits with LSB at the very right
						// need to combine both registers to get the value!

						// setting the Start conversion bit
int16_t ADC_read_voltage(void)
{						
	ADCSRA |=	(1<<ADSC);
	
	// maybe we check for the interrupt flag?
	while((ADCSRA & (1 << ADSC)));		// while the Start Bit is not reset, wait
	
	v_raw = (ADCL);
	v_raw = (ADCH << 8) | v_raw;		// it was mentioned to read out Lower Bytes before
	
	// sensor reads 2.5VDC at 0bar differential pressure
	// 2.5VDC will give a value of ((2^10) / 2)-1 = (1024 / 2) -1 = 512 - 1 = 511
	
	// Speed calculation
	// 511 = 0
	// 100 000 Pa = 1 bar
	// the datasheet says the pressure range is +-2 kPa = 2000 Pa = 0,02 bar
	// 1 Pa = 1N/m^2
	// means that 2000/511 Pa per bit = 3,914 Pa

	
	//speed = v_raw;	// we will use the ADC value to run autopilot functions...(more precise)
	delta_p = (v_raw -510)*(2000/510);
	if(ADC_cal_flag == TRUE)
	{
		return speed_tx = (v_raw-adc_offset);
	}	
	// speed = SQRT((2*dp) / rho)
	
	return v_raw;
}

void ADC_calibration()
{
	int cal_count = 0;
	
	while(cal_count < 50)
	{
		adc_offset += ADC_read_voltage();
		cal_count++;
	}
	adc_offset = adc_offset / cal_count;
	write_string("ADC_offset: ");write_var_ln(adc_offset);
	ADC_cal_flag = TRUE;
	_delay_ms(500);
}