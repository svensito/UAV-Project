/*
 * SimpleTesting.c
 *
 * Created: 09.04.2013 09:50:30
 *  Author: snfu
 */ 



//------------------------------------------------------
// DEFINE SECTION
//#define F_CPU	12000000UL



//------------------------------------------------------

//------------------------------------------------------
// INCLUDE SECTION
#include "GeneralSettings.h"
#include <avr/io.h>
#include <avr/delay.h>
#include "uart.h"
#include "ServoControl.h"
#include "port.h"
#include <avr/interrupt.h>
#include <string.h>
#include "Gyro.h"
#include "Accelerometer.h"
#include <avr/wdt.h>
//------------------------------------------------------

//------------------------------------------------------
// Local declarations and variables

// These are the task flags, set them to activate / deactivate task
int task_gyro	= TRUE;
int task_acc	= TRUE;
int task_mag	= TRUE;
int task_temp	= FALSE;
int task_baro	= TRUE;
int task_speed	= TRUE;

// Control Mode
uint8_t Ctrl_Mode = DIRECT_CTRL;		// First Test with Direct Law Only: DIRECT_LAW or NORMAL_LAW possible
uint8_t Ctrl_Mode_prev = DIRECT_CTRL;	// Initialize as the same Mode
uint8_t state_change = FALSE;			// Needed as a flag for state change

// Control Knob
uint8_t Knob_State = 0;
uint8_t prev_Knob_State = 0;
uint16_t Knob_Stepwidth = 400;

// Control Algorithm Variables
uint8_t k_eta_q = 5;

//long test	= 0;
//------------------------------------------------------


//------------------------------------------------------
// Task Timer definitions
volatile uint8_t task_flag = 0;		// this flag will be set every 20ms and reset in the application
uint8_t comp_count = 0;

//------------------------------------------------------
// Task Timer for the tasks
void Task_Timer(void)
{
	// Configuring the Timer 0, that is used to create the Task Timer
	// WGM Bits: Define CTC mode
	// CS Bits: Prescaler of 1024 selected
	// desired Frequency: 50Hz => period of 20ms
	// FCPU = 12MHz
	// Steps for clearing the timer: (F_CPU/Prescaler)/(Freq) = 234
	TCCR0 = (0 << FOC0)|(0 << WGM00)|(0 << COM01)|(0 << COM00)|(1 << WGM01)|(1 << CS02)|(0 << CS01)|(1 << CS00);
	
	// we will set TIMSK globally (deactivating all and activating all interrupts at once
	// TIMSK = (1<<OCIE0);
	// Due to the 8 bit timer, the maximum value can be 255, that would reflect 21,7 ms
	// The OCR cannot be set to higher than 255
	// Corrected: The Overflow could be used to count how many times the Timer was overflowing (also ISR)
	// if ISRs want to be avoided, then the overflow is as well not the choice No1
	
	// Corrected: Using the ISR to count up. Therefore multiples of the timer OCR value are possible
	// with OCR0 as 195 => 16,6ms count up time => if counter = 3 then 50ms task time to be realised
	OCR0 = 195;//0x75;
	write_string_ln("TaskTimerStarted");
}	

ISR(TIMER0_COMP_vect)
{
	comp_count++;
	if (comp_count >= 3)
	{
		// Setting the task flag
		task_flag = 1;
		comp_count = 0; // Resetting comp count
	}
	
}

//------------------------------------------------------

//-----------------------------------
// Watchdog
// Is used to secure in case there is a disturbance and the controller does not react.
// Enabling the Watchdog
void WDT_on(void)
{
	// Set the Watchdog Enable Bit
	// Seting the Prescaler Bits: 101 will give around 0.5 seconds timeout
		WDTCR |= (1<<WDE)|(1<<WDP2)|(0<<WDP1)|(1<<WDP0);
}

// Disabling the Watchdog
void WDT_off(void)
{
	/* command to Toggle the Watchdog */
	wdt_reset();
	
	/* Write logical one to WDTOE and WDE */
	WDTCR |= (1<<WDTOE) | (1<<WDE);
	/* Turn off WDT */
	WDTCR = 0x00;
	write_string_ln("WdogEnabl");
}
//-----------------------------------


int main(void)
{	
	
	// Initializing
	init_uart();
	
	// Checking the Reset Status
	if (MCUCSR & (1<<WDRF)) write_string("WdogRst");
	else if(MCUCSR & (1<<PORF)) write_string("PwrUpRst");
	else if(MCUCSR & (1<<BORF)) write_string("BrwnOutRst");
	else if(MCUCSR & (1<<EXTRF)) write_string("ExtRst");
	else write_var(MCUCSR);
	// Resetting the Status Register
	MCUCSR &= ~((1<<WDRF)|(1<<PORF)|(1<<BORF)|(1<<EXTRF));
	_delay_ms(500);
	WDT_on();
	
	i2c_initialize();
	
	
	
	if(task_gyro == TRUE)	
	{
		gyro_start();
		gyro_calibration();
	}		
	if(task_acc == TRUE)	acc_start();
	if(task_baro == TRUE)	
	{
		baro_start();
		baro_calibration();
	}		
	if(task_mag == TRUE)	mag_start();
	if(task_speed == TRUE)	
	{
		ADC_start();
		ADC_calibration();
	}		
	
	//_delay_ms(2000);
	// Starting the Servo PWM signal setting
	
	cli();	// disable interrupts
	Task_Timer();
	servo_init();
	// setting all needed timers to Interrupt mode
	TIMSK  = (0<<TICIE1)|(1<<OCIE1A)|(0<<OCIE1B)|(0<<TOIE1)|(1<<OCIE0)|(1<<TOIE2);
	sei();	//enable interrupts
	
	int testflag = 0;
	// Turn rate Data
	int8_t p_raw,q_raw,r_raw,p_filt,q_filt,r_filt,p_filt_prev,q_filt_prev,r_filt_prev = 0; // raw turn rates, filtered turn rates,previous turn rates
	float alpha_turn = 0.7;	
	// Altitude Data
	int16_t altitude_raw,altitude_filt,alt_hold = 0;	// raw altitude and filtered altitude
	int16_t alt_error = 0;
	int16_t alt_error_sum = 0;
	int16_t alt_error_prev = 0;
	float alpha_alt = 0.3; // alpha element [0;1] -> alpha 0: only raw input (noise free)
										   // -> alpha 1: only filtered input (only noise)
	int8_t Kp_alt = 5;
	int8_t Ki_alt = 5;
	int8_t Kd_alt = 5;	
	long trimmed_elevator = 0;				   
	// Speed Data
	int16_t speed_raw,speed_filt = 0;	// raw speed and filtered speed
	float alpha_speed = 0.8; // alpha element [0;1] -> alpha 0: only raw input (noise free)
												// -> alpha 1: only filtered input (only noise)
	float speed = 0;
	// Euler Angles Data
	float Theta = 0;
	float Phi = 0;
	float Psi = 0;
	float Phi_hold = 0;
	float Phi_error = 0;
	float Phi_error_sum = 0;
	float Phi_error_prev = 0;
	int8_t Kp_Phi = 5;
	int8_t Ki_Phi = 5;
	int8_t Kd_Phi = 5;
	long trimmed_aileron = 0;
	// Acceleration data
	int16_t acc_x_raw, acc_y_raw, acc_z_raw,acc_x_filt, acc_y_filt, acc_z_filt,acc_x_filt_prev, acc_y_filt_prev, acc_z_filt_prev = 0;
	float alpha_acc = 0.3;
	// Mag Data
	int16_t mag = 0;
	//struct three_values test;
	int8_t bla_cnt = 0;
	int8_t send_cnt = 0;
	
	while(1)
    {
		// Program Code (infinite loop)
			
			
			if(task_flag == 1)
			{
				// Resetting the task call:
				task_flag = 0;
				//bla_cnt++;
				
				// Defining struct to read the gyro channels
				struct three_elements_obj turn_rate;
				if(task_gyro == TRUE)	
				{
					// turn rates
					turn_rate = gyro_read();
					// result in degrees per second
					p_raw = turn_rate.p;
					q_raw = turn_rate.q;
					r_raw = turn_rate.r;
					// low pass filter on the turn rates
					p_filt = p_raw * (1-alpha_turn) + (alpha_turn*p_filt);
					q_filt = q_raw * (1-alpha_turn) + (alpha_turn*q_filt);
					r_filt = r_raw * (1-alpha_turn) + (alpha_turn*r_filt);
					
					Theta += (q_filt_prev+ (q_filt - q_filt_prev)/2) * 0.02;	// 20 ms sample Time
					Phi += (p_filt_prev+ (p_filt - p_filt_prev)/2) * 0.02;
					Psi += (r_filt_prev+ (r_filt - r_filt_prev)/2) * 0.02;
					q_filt_prev = q_filt;
					p_filt_prev = p_filt;
					r_filt_prev = r_filt;
				}
				struct acc_readings_obj acc_val;
				if(task_acc == TRUE)	
				{
					acc_val = acc_reading();
					acc_x_raw = acc_val.a_x;
					acc_y_raw = acc_val.a_y;
					acc_z_raw = acc_val.a_z;
					//write_string("acc: "); write_var_ln(acc_reading());
				}
				if(task_baro == TRUE)// && (bla_cnt==100))	
				{
					altitude_raw = baro_read();
					// low pass filter on the altitude reading
					altitude_filt = altitude_raw*(1-alpha_alt) + (alpha_alt*altitude_filt);
					//bla_cnt = 0;
				}
						
				if(task_mag == TRUE)	mag = mag_read();
				if(task_speed == TRUE)
				{
					speed_raw = ADC_read_voltage();
					// low pass filter on the speed reading
					speed_filt = speed_raw*(1-alpha_speed) + (alpha_speed*speed_filt);
				}
				
				// Reading the input from the knob to change control modes
				if(ctrl_in[4]<135 && ctrl_in[4]>120) Ctrl_Mode = DIRECT_CTRL;
				else if(ctrl_in[4]<150 && ctrl_in[4]>135) Ctrl_Mode = HOLD_CTRL;
				else if(ctrl_in[4]<160 && ctrl_in[4]>150) Ctrl_Mode = HOLD_CTRL;
										
				if(Ctrl_Mode_prev != Ctrl_Mode)
				{
					// If a state change happens
					state_change = TRUE;
				}
				else
				{
					// Otherwise no state change
					state_change = FALSE;
				}
				
				switch(Ctrl_Mode)
				{
					case DIRECT_CTRL:
					// Direct Law is used for direct Remote Control input -> Servos
					// + / - in front of the long bracket inverses the Control
						if(state_change == TRUE)
						{
							write_string_ln("DIRECT CTRL");
						}
						 POSITION1 = NEUTRAL+((ctrl_in[0]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);			// Motor	increase:	POSITION++	reduce: POSITION--
						 POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);		// Aileron	right:		POSITION++	left: POSITION--
						 POSITION3 = NEUTRAL+((ctrl_in[2]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);	// Elevator	up:			POSITION++	down: POSITION--
						 POSITION4 = NEUTRAL+((ctrl_in[3]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);		// Rudder	left:		POSITION++	right: POSITION--
						 if(ctrl_in[5]>144) POSITION5 = FLAP_FULL;										// Flaps
						 else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) POSITION5 = FLAP_1;
						 else POSITION5 = FLAP_UP;
					break;
					
					case NORMAL_CTRL:
					// Normal Law will be used for semi automatic flight (supportive Damping etc.)
						 //POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON) - (10*turn_rate.p);
							bla_cnt++;
							if(bla_cnt == 100)
							{
								write_string(GPS_string[GPS_MODE]);write_string(" ");
								write_string(GPS_string[GPS_LATITUDE]);write_string(" ");
								write_string(GPS_string[GPS_LONGITUDE]);write_string(" ");
								write_string(GPS_string[GPS_DATE]);write_string(" ");
								write_string_ln(GPS_string[GPS_TIME]);
								bla_cnt = 0;
							}									
					break;
					
					case DAMPED_CTRL:
						POSITION1 = NEUTRAL+((ctrl_in[0]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);			// Motor (not for Easyglider)
						POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);		// Aileron
						POSITION3 = (NEUTRAL+((ctrl_in[2]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR))- (k_eta_q*turn_rate.q);	// Elevator
						POSITION4 = NEUTRAL-((ctrl_in[3]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);		// Rudder
						if(ctrl_in[5]>144) POSITION5 = FLAP_FULL;
						else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) POSITION5 = FLAP_1;
						else POSITION5 = FLAP_UP;
					break;
					
					case HOLD_CTRL:
						// entry condition
						if(state_change == TRUE)
						{
							write_string_ln("HOLD CONTROL");
							alt_hold = altitude_filt; // Setting current values as control values
							Phi_hold = Phi;
							trimmed_elevator = POSITION3; // Setting current input as "trimmed input"
							write_string("Trimmed Elevator: ");write_var_ln(trimmed_elevator);
							trimmed_aileron = POSITION2;
							write_string("Trimmed Aileron: ");write_var_ln(trimmed_aileron);
							write_string("Target Alt: ");write_var_ln(alt_hold);
						}
						
						// Other controls as usual
						POSITION1 = NEUTRAL+((ctrl_in[0]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);		// Motor
						POSITION4 = NEUTRAL+((ctrl_in[3]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);		// Rudder
						if(ctrl_in[5]>144) POSITION5 = FLAP_FULL;
						else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) POSITION5 = FLAP_1;
						else POSITION5 = FLAP_UP;						
							
						// altitude control with elevator (Position 3)
						alt_error = altitude_filt - alt_hold; // Positive Error shall give negative control (Elevator down)
						alt_error_sum += alt_error;
						POSITION3 = ( trimmed_elevator - (Kp_alt*alt_error + Ki_alt*alt_error_sum*0.05 + Kd_alt*(alt_error_prev - alt_error)/0.05));
						if(POSITION3 > RIGHT) POSITION3 = RIGHT;
						else if (POSITION3 < LEFT) POSITION3 = LEFT;
						alt_error_prev = alt_error;
						
							
						// bank angle control with aileron (Position 2)
						Phi_error = Phi - Phi_hold;		// Positive Error shall give negative control (Aileron roll left)
						Phi_error_sum += Phi_error;
						POSITION2 = (trimmed_aileron - (Kp_Phi*Phi_error + Ki_Phi*Phi_error_sum*0.05 + Kd_Phi*(Phi_error_prev - Phi_error)/0.05));
						Phi_error_prev = Phi_error;
						if(POSITION2 > RIGHT) POSITION2 = RIGHT;
						else if (POSITION2 < LEFT) POSITION2 = LEFT;	
			
						

					break;
												
					
				}
					// writing all data to serial port
					write_var(POSITION1);write_string(";");write_var(POSITION2);write_string(";");
					write_var(POSITION3);write_string(";");write_var(POSITION4);write_string(";");
					write_var(POSITION5);write_string(";");
					write_var(p_filt);write_string(";");write_var(Phi);write_string(";");
					write_var(q_filt);write_string(";");write_var(Theta);write_string(";");
					write_var(r_filt);write_string(";");write_var(Psi);write_string(";");
					write_var(acc_x_raw);write_string(";");write_var(acc_y_raw);write_string(";");write_var(acc_z_raw);write_string(";");
					write_var(mag);write_string(";");
					write_var(altitude_filt);write_string(";");write_var(speed_filt);write_string(";");
					write_var(alt_error);write_string(";");write_var(Phi_error);write_string_ln(";");
					
				
				Ctrl_Mode_prev = Ctrl_Mode;	// Setting previous State
			}		
			
			// Toggling the Watchdog (Reset)
			wdt_reset();
    }
}