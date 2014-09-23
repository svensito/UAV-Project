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

//******************************************************************
// These are the task flags, set them to activate / deactivate task
int task_gyro	= TRUE;		// reading gyro data enabled (TRUE) /disabled (FALSE)
int task_acc	= TRUE;		// reading acc data enabled (TRUE) /disabled (FALSE)
int task_mag	= TRUE;		// reading mag data enabled (TRUE) /disabled (FALSE)
int task_temp	= FALSE;	// reading temp data enabled (TRUE) /disabled (FALSE)
int task_baro	= TRUE;		// reading baro data enabled (TRUE) /disabled (FALSE)
int task_speed	= TRUE;		// reading ADC speed data enabled (TRUE) /disabled (FALSE)
int serial_log	= FALSE;		// serial output enabled (TRUE) /disabled (FALSE)
//******************************************************************

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
uint8_t control_gain = 1;

//long test	= 0;
//------------------------------------------------------


//------------------------------------------------------
// Task Timer definitions
volatile uint8_t task_flag = 0;		// this flag will be set every 20ms and reset in the application
uint8_t comp_count = 0;

// Timer Sample time
float sample_time = 0;


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
	
	// Update: want to achieve 20ms to get better resolution...Using 10 ms time and counter to count 2 times...
	// 10 ms -> 117 count value, in the interrupt count til 2!
	OCR0 = 117;//0x75;
	sample_time = 0.03;	// ALWAYS change when changing the task time!
	write_string_ln("TaskTimerStarted");
}	

ISR(TIMER0_COMP_vect)
{
	comp_count++;
	if (comp_count >= 3)	// This gives multiple of 10ms	-> Change sample time accordingly
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
	write_string_ln("WdogDisabl");
}
//-----------------------------------


// Ascii to integer
uint32_t atol_new(const char* str) {
	uint32_t num = 0;
	char digit;
	while ((digit = *str++) != '\0') {
		if (digit < '0' || digit > '9') {
			return num;  /* No valid conversion possible */
		}
		num *= 10;
		num += digit - '0';
	}
	return num;
}

//-----------------------------------


int main(void)
{	
	// Setting the LED port to output
	//DDRC |= (1<<PC2);	// LED on PC2 pin as output
	
	//LED off
	//PORTC &= ~(1<<PC2);
	//LED on
	//PORTC |= (1<<PC2);
	
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
		ADC_speed_cal();
	}		
	
	//_delay_ms(2000);
	// Starting the Servo PWM signal setting
	
	cli();	// disable interrupts
	Task_Timer();
	servo_init();
	// setting all needed timers to Interrupt mode
	TIMSK  = (0<<TICIE1)|(1<<OCIE1A)|(0<<OCIE1B)|(0<<TOIE1)|(1<<OCIE0)|(1<<TOIE2);
	sei();	//enable interrupts
	
	
	
	// Circle Number Pi
	float Pi =  3.1415926535;
	
	int testflag = 0;
	// Turn rate Data
	int8_t p_raw,q_raw,r_raw,p_filt,q_filt,r_filt,p_filt_prev,q_filt_prev,r_filt_prev = 0; // raw turn rates, filtered turn rates,previous turn rates
	float alpha_turn = 0.7;	
	// Altitude Data
	int16_t altitude_raw,altitude_filt,alt_hold, alt_hold_0 = 0;	// raw altitude and filtered altitude
	int16_t alt_error = 0;
	int16_t alt_error_sum = 0;
	int16_t alt_error_prev = 0;
	float alpha_alt = 0.3; // alpha element [0;1] -> alpha 0: only raw input (noise free)
										   // -> alpha 1: only filtered input (only noise)
	int8_t Kp_alt = 1;
	int8_t Ki_alt = 10;
	int8_t Kd_alt = 10;
	long trimmed_elevator = 0;				   
	// Speed Data
	int16_t speed_raw,speed_filt, speed_filt_1, speed_filt_2, speed_filt_3, speed_filt_4, speed_filt_5 = 0;	// raw speed and filtered speed
	float alpha_speed_1 = 0.3; // alpha element [0;1] -> alpha 0: only raw input (noise free)
	float alpha_speed_2 = 0.3;											// -> alpha 1: only filtered input (only noise)
	float alpha_speed_3 = 0.3;											
	float alpha_speed_raw = 0.1;									
												
	float speed = 0;
	float speed_hold, speed_error, speed_error_sum, speed_errror_prev = 0;
	int8_t Kp_speed = 10;	
	//int8_t Kd_speed = 1;	// No differential part as signal too noisy
	int8_t Ki_speed = 5;
	long trimmed_motor = 0;
	
	// Euler Angles Data
	float Theta, Theta_hold, Theta_error, Theta_error_sum, Theta_error_prev = 0;
	int8_t Kp_Theta = 10;	// as per simulation in SCILAB this are very good gains for a wide range of speed...
	int8_t Kd_Theta = 5;
	int8_t Ki_Theta = 10;
	int8_t K_p_q = 9;
	float Phi = 0;
	float Psi = 0;
	float Phi_hold, Phi_hold_0 = 0;
	float Phi_error = 0;
	float Phi_error_sum = 0;
	float Phi_error_prev = 0;
	int8_t Kp_Phi = 10;	// as per simulation in scilab
	int8_t Kd_Phi = 1;
	int8_t Ki_Phi = 5;
	
	long trimmed_aileron = 0;
	long trimmed_rudder = 0;
	// Acceleration data
	int16_t acc_x_raw, acc_y_raw, acc_z_raw,acc_x_filt, acc_y_filt, acc_z_filt,acc_x_filt_prev, acc_y_filt_prev, acc_z_filt_prev = 0;
	float alpha_acc = 0.3;
	float Theta_acc = 0;	// Theta based on the Accelerometer reading
	float Phi_acc = 0;		// Phi based on the Accelerometer reading
	// Kalman Filter Data
	// weighing matrices ???
	float Q_angle = 0.005;
	float Q_gyro = 0.005;
	float R_angle = 0.02;
	// Estimation of Theta (Kalman)
	float q_bias = 0;
	float P_00_Theta, P_01_Theta, P_10_Theta, P_11_Theta = 0;
	float Theta_temp, S_Theta, K_0_Theta, K_1_Theta = 0;
	// Estimation of Phi (Kalman)
	float p_bias = 0;
	float P_00_Phi, P_01_Phi, P_10_Phi, P_11_Phi = 0;
	float Phi_temp, S_Phi, K_0_Phi, K_1_Phi = 0;
	// Estimation of Psi (Kalman)
	float r_bias = 0;
	float P_00_Psi, P_01_Psi, P_10_Psi, P_11_Psi = 0;
	float Psi_temp, S_Psi, K_0_Psi, K_1_Psi = 0;
	
	// Mag Data
	int16_t heading = 0;
	int16_t heading_target = 0;
	int16_t heading_hold, heading_error, heading_error_prev, heading_error_sum  = 0;
	int8_t Kp_head = 1;
	int8_t Kd_head = 1;
	int8_t Ki_head = 0;		// simulation showed that PD is convenient for Heading Track
	
	// GPS navigation data
	int32_t GPS_POS_CURRENT_X, GPS_POS_CURRENT_Y, GPS_POS_TARGET_X, GPS_POS_TARGET_Y, GPS_POS_DIF_X, GPS_POS_DIF_Y = 0;
	int8_t GPS_POS_LAT_DEG, GPS_POS_LAT_MIN = 0;
	int32_t GPS_POS_LAT_SEC = 0;
	int32_t GPS_DIS_TO_TARGET = 0;
			
	// random counter
	int8_t bla_cnt = 0;
	int8_t send_cnt = 0;
	int8_t tune_cnt = 0;
	int8_t test_cnt = 0;
	
	// random flags
	int8_t knob_flag = 0;
	
	// Turn On the Watchdog
	WDT_on();
	
	while(1)
    {
			// Program Code (infinite loop)
			
			if(UART_READY_FLAG == TRUE)
			{
				if(strcmp(GPS_string[0],"GPRMC")==0)
				{
					memcpy(GPS_RMC,GPS_string,sizeof(GPS_RMC));
					memset(GPS_string,0,sizeof(GPS_string));
					
				}
				if(strcmp(GPS_string[0],"GPGGA")==0)
				{
					memcpy(GPS_GGA,GPS_string,sizeof(GPS_GGA));
					memset(GPS_string,0,sizeof(GPS_string));
				}
				/*
				write_string_ln(GPS_RMC[GPS_RMC_GROUNDSPEED]);
				write_string_ln(GPS_GGA[GPS_GGA_ALTMSL]);
				*/
				UART_READY_FLAG = FALSE;
			}
			
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
					Theta_acc = atan2(acc_x_raw,-acc_z_raw)*180/Pi;
					Phi_acc = atan2(-acc_y_raw,-acc_z_raw)*180/Pi;
					//write_string("acc: "); write_var_ln(acc_reading());
					
				}
				if(task_baro == TRUE)// && (bla_cnt==100))	
				{
					altitude_raw = baro_read();
					// low pass filter on the altitude reading
					altitude_filt = altitude_raw*(1-alpha_alt) + (alpha_alt*altitude_filt);
					//bla_cnt = 0;
				}
						
				if(task_mag == TRUE)	heading = mag_read();
				if(task_speed == TRUE)
				{
					
					if (bla_cnt == 8)		// to reduce the sampling time of the speed reading...
					{
					
					speed_raw = ADC_read_speed();
					// low pass filter on the speed reading
								// current reading				// prev reading			//
			
					speed_filt = ((-3*speed_filt_1) + (12*speed_filt_2) + (17*speed_filt_3) + (12*speed_filt_4) + (-3*speed_filt_5))/35;// + (alpha_speed_2*speed_filt_2) + (alpha_speed_3*speed_filt_3);
					speed_filt_5 = speed_filt_4;
					speed_filt_4 = speed_filt_3;
					speed_filt_3 = speed_filt_2;
					speed_filt_2 = speed_filt_1;
					speed_filt_1 = speed_raw;
					
					bla_cnt = 0;
					}
					
					bla_cnt++;
					//write_var(speed_raw);write_string(";");write_var(speed_filt);write_string_ln(";");
					
					
				}
				
				// Reading the input from the knob to change control modes
				/*
				if(ctrl_in[4]<135 && ctrl_in[4]>120) Ctrl_Mode = DIRECT_CTRL;
				else if(ctrl_in[4]<150 && ctrl_in[4]>135) Ctrl_Mode = HOLD_CTRL;
				else if(ctrl_in[4]<160 && ctrl_in[4]>150) Ctrl_Mode = HOLD_CTRL;
				*/	
				
				// Calculating the attitude by using Kalman Filtering
				Theta += sample_time * (q_filt-q_bias);
				
				P_00_Theta +=  - sample_time * ((P_10_Theta + P_01_Theta) - Q_angle);
				P_01_Theta +=  - sample_time * P_11_Theta;
				P_10_Theta = P_01_Theta;
				P_11_Theta +=  + Q_gyro * sample_time;
				
				
				Phi += sample_time * (p_filt-p_bias);
				P_00_Phi +=  - sample_time * ((P_10_Phi + P_01_Phi) - Q_angle);
				P_01_Phi +=  - sample_time * P_11_Phi;
				P_10_Phi = P_01_Phi;
				P_11_Phi +=  + Q_gyro * sample_time;
				
			
				/*
				Psi += sample_time * (r_filt-r_bias);
				P_00_Psi +=  - sample_time * (P_10_Psi + P_01_Psi) + Q_angle * sample_time;
				P_01_Psi +=  - sample_time * P_11_Psi;
				P_10_Psi +=  - sample_time * P_11_Psi;
				P_11_Psi +=  + Q_gyro * sample_time;
				*/
				Theta_temp = Theta_acc - Theta;
				S_Theta = P_00_Theta + R_angle;
				K_0_Theta = P_00_Theta / S_Theta;
				K_1_Theta = P_10_Theta / S_Theta;
				
				Phi_temp = Phi_acc - Phi;
				S_Phi = P_00_Phi + R_angle;
				K_0_Phi = P_00_Phi / S_Phi;
				K_1_Phi = P_10_Phi / S_Phi;
				/*
				Psi_temp = mag - Psi;
				S_Psi = P_00_Psi + R_angle;
				K_0_Psi = P_00_Psi / S_Psi;
				K_1_Psi = P_10_Psi / S_Psi;
				*/
				Theta +=  K_0_Theta * Theta_temp;
				q_bias  +=  K_1_Theta * Theta_temp;
				P_00_Theta -= K_0_Theta * P_00_Theta;
				P_01_Theta -= K_0_Theta * P_01_Theta;
				P_10_Theta -= K_1_Theta * P_00_Theta;
				P_11_Theta -= K_1_Theta * P_01_Theta;
				
				Phi +=  K_0_Phi * Phi_temp;
				p_bias  +=  K_1_Phi * Phi_temp;
				P_00_Phi -= K_0_Phi * P_00_Phi;
				P_01_Phi -= K_0_Phi * P_01_Phi;
				P_10_Phi -= K_1_Phi * P_00_Phi;
				P_11_Phi -= K_1_Phi * P_01_Phi;
				/*
				Psi +=  K_0_Psi * Psi_temp;
				r_bias  +=  K_1_Psi * Psi_temp;
				P_00_Psi -= K_0_Psi * P_00_Psi;
				P_01_Psi -= K_0_Psi * P_01_Psi;
				P_10_Psi -= K_1_Psi * P_00_Psi;
				P_11_Psi -= K_1_Psi * P_01_Psi;
				*/
				
				/*
				write_var(Phi);write_string(";");
				write_var(Theta);write_string_ln(";");
				*/
				

				
				//******************************************
				// Detecting current position and required heading
				
				
				//GPS_POS_CURRENT_Y = "123456";
				// Target coordinates: Berlin
				GPS_POS_TARGET_X = 132400290;	// 10 Latitude digits
				GPS_POS_TARGET_Y = 523138036;	// 10 Longitude digits
								
				GPS_POS_CURRENT_X = atol_new(GPS_RMC[GPS_RMC_LONGITUDE]);
				GPS_POS_CURRENT_Y = atol_new(GPS_RMC[GPS_RMC_LATITUDE]);
				//write_var(GPS_POS_CURRENT_X);write_string(";");write_var(GPS_POS_CURRENT_Y);write_string(";");
				GPS_POS_DIF_X = GPS_POS_TARGET_X - GPS_POS_CURRENT_X;	// define errors in such way that the course is correct!
				GPS_POS_DIF_Y = GPS_POS_TARGET_Y - GPS_POS_CURRENT_Y;	
				//write_var(GPS_POS_DIF_X);write_string(";");write_var(GPS_POS_DIF_Y);write_string_ln(";");
				//heading_target = atan2(GPS_POS_DIF_Y,GPS_POS_DIF_X)*(180/PI);
				heading_target = atan2((GPS_POS_DIF_X),(GPS_POS_DIF_Y))*(180/PI);
				if (heading_target < 0)
				{
					heading_target += 360;
				}
				// Distance to target:
				// calculate the latitude difference. one degree latitude is 111km distance (wiki). divide by cos(angle).
				// 1 deg = 111km
				// 1 minute = 1,85km
				// 1 second = 0,031km
				GPS_POS_LAT_DEG = GPS_POS_DIF_Y/10000000;
				GPS_POS_LAT_MIN = (GPS_POS_DIF_Y/100000)-(GPS_POS_LAT_DEG*100);
				GPS_POS_LAT_SEC = (GPS_POS_DIF_Y)-((GPS_POS_LAT_DEG*10000000)+(GPS_POS_LAT_MIN*100000));
				GPS_DIS_TO_TARGET = GPS_POS_LAT_DEG*111000 + GPS_POS_LAT_MIN*1850 + (((GPS_POS_LAT_SEC*60*31)/100000));///cos(heading_target);
				//write_var(GPS_POS_LAT_MIN);write_string(";");write_var(GPS_POS_LAT_SEC);write_string_ln(";");
				//write_var_ln(atol_new(GPS_RMC[GPS_RMC_LATITUDE][0])*10 + atol_new(GPS_RMC[GPS_RMC_LATITUDE][1]));
				
				
				//*******************************************
				// Control of Modes
				// changing from knob to normal switch for mode control
				if(ctrl_in[5]>144) Ctrl_Mode = HOLD_CTRL;								// Dn Position
				else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) Ctrl_Mode = DIRECT_CTRL;	// Middle Position
				else Ctrl_Mode = DAMPED_CTRL;											// Up Position
								
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
				
				//******************************************						
				// use this flag to switch between long / lat tuning mode
				uint8_t mode = 3;
				
				switch(Ctrl_Mode)
				{
					case DIRECT_CTRL:
					// Direct Law is used for direct Remote Control input -> Servos
					// + / - in front of the long bracket inverses the Control
						if(state_change == TRUE)
						{
							write_string_ln("DIRECT CTRL");
						}
							if (ctrl_in > ctrl_in_prev)
							{
								
							}
						 
						 ctrl_out[motor]	=	NEUTRAL+((ctrl_in[stick_l_up_down]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);		
						 ctrl_out[aileron]	=	NEUTRAL+((ctrl_in[stick_r_left_right]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);
						 ctrl_out[elevator] =	NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
						 ctrl_out[rudder]	=	NEUTRAL+((ctrl_in[stick_l_left_right]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);
						 
						 // Flap Delay
						 uint16_t flap_target = FLAP_UP;	// defining the target flap position
						 uint8_t flap_delta = 10;	// giving the speed of flap extension 
													// 10 will give around 2 seconds for movement in between two positions
						 
						 // Flap Control
						 if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120)		flap_target = FLAP_UP;
						 else if(ctrl_in[rotary_knob]<150 && ctrl_in[rotary_knob]>135)	flap_target = FLAP_1;
						 else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150)	flap_target = FLAP_FULL;
						 
						 // drive flap with sample_time steps delay 
						 if(ctrl_out[flap] < flap_target)		ctrl_out[flap]+= flap_delta;
						 else if(ctrl_out[flap] > flap_target)	ctrl_out[flap]-= flap_delta;
						 else ctrl_out[flap] = flap_target;
						 
						 // camera gimbal servo
						 ctrl_out[camera_y] = NEUTRAL;
						 ctrl_out[camera_z] = SERVO_GIMB_Z_STOP;	// the z axis Servo is a hacked 360 deg Servo without position information
																	// There is no feedback from the servo on its position.	
																	// For leaving and reaching a new position, only time integration would work...
						 
					break;
					
					case TUNE_CTRL:
						// Tuning control mode
							

						
						//###########################################
						// Tuning Longitudinal Mode Open Loop Tuning
						if (mode == 1)
						{
							
							// Make a step change and check the response
							if(state_change == TRUE)
							{
								write_string_ln("*TUNING LONG STEP*");
								trimmed_motor = ctrl_out[motor];
								trimmed_aileron = ctrl_out[aileron];
								trimmed_elevator = ctrl_out[elevator];
								trimmed_rudder = ctrl_out[rudder];
								ctrl_out[elevator] = trimmed_elevator;
								ctrl_out[motor] = trimmed_motor;
								ctrl_out[aileron] = trimmed_aileron;
								ctrl_out[rudder] = trimmed_rudder;
								tune_cnt = 0;
							}

							if(tune_cnt < 40) tune_cnt++;
							if(tune_cnt == 40)	// after 2 seconds it will make a step input to the elevator
							{
								write_string_ln("Step Ele");
								ctrl_out[elevator] +=  200;
								tune_cnt += 1; // to leave the if clause
							}
						}
						
						//###########################################
						// Tuning Lateral Mode Open Loop Tuning
						if (mode == 2)
						{
							
							// Make a step change and check the response
							if(state_change == TRUE)
							{
								write_string_ln("*TUNING LAT IMPULSE*");
								trimmed_motor = ctrl_out[motor];
								trimmed_aileron = ctrl_out[aileron];
								trimmed_elevator = ctrl_out[elevator];
								trimmed_rudder = ctrl_out[rudder];
								ctrl_out[elevator] = trimmed_elevator;
								ctrl_out[motor] = trimmed_motor;
								ctrl_out[aileron] = trimmed_aileron;
								ctrl_out[rudder] = trimmed_rudder;
								tune_cnt = 0;
							}

							if(tune_cnt < 40) tune_cnt++;
							if(tune_cnt == 40)	// after 2 seconds it will make an impulse to the aileron
							{
								write_string_ln("Step Ail");
								ctrl_out[aileron] +=  500;
								tune_cnt += 1;
							}
							if(tune_cnt >= 41 && tune_cnt < 57) tune_cnt++;
							if(tune_cnt == 57)	// after 210 mseconds it will release the aileron
							{
								write_string_ln("Step Ail");
								ctrl_out[aileron] -=  500;
								tune_cnt += 1;
							}
						}
						
						//###########################################
						// Tuning Longitudinal Mode Closed Loop Tuning
						if (mode == 3)
							{
						
							if(state_change == TRUE)
							{
								write_string_ln("*Speed Control*");
								speed_error = 0;
								speed_error_sum = 0;
								speed_hold = speed_filt;
								
							}
							// Trying of Speed control
						
							ctrl_out[aileron]	=	NEUTRAL+((ctrl_in[stick_r_left_right]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);
							ctrl_out[elevator] =	NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
							ctrl_out[rudder]	=	NEUTRAL+((ctrl_in[stick_l_left_right]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);
							
						//**********************************************
						// Speed control (PI only -> noisy signal) with motor
						speed_error = speed_hold - speed; // Positive Error (too slow) shall give positive input (motor increase)
						speed_error_sum += speed_error;	
						// when too slow (bracket turns positive) the motor gets increased
						ctrl_out[motor] = trimmed_motor + (Kp_speed*speed_error + Ki_speed * speed_error_sum * sample_time);
						if(ctrl_out[motor] > RIGHT) ctrl_out[motor] = RIGHT;
						else if (ctrl_out[motor] < LEFT) ctrl_out[motor] = LEFT;
						speed_errror_prev = speed_error;
						}
				
						//###########################################
						// Tuning the Lateral Mode Closed loop tuning
						if (mode == 4)
						{
							
							if(state_change == TRUE)
							{
								write_string_ln("TUNING LAT");
								trimmed_motor = ctrl_out[motor];
								trimmed_aileron = ctrl_out[aileron];
								trimmed_elevator = ctrl_out[elevator];
								
								Phi_hold = Phi;
								Phi_error = 0;
								Phi_error_prev = 0;
								Phi_error_sum = 0;
								Kp_Phi = 0;
								
								Theta_hold = Theta;
								Theta_error = 0;
								Theta_error_prev = 0;
								Theta_error_sum = 0;
								
							}
							// Tuning of PID Parameters
							// Tuning the Longitudinal Mode
							
							ctrl_out[motor] = trimmed_motor;	// lets assume the motor setting will keep the speed more or less
							
							// Theta control with elevator
													
							Theta_error = Theta_hold - Theta; // Positive Error (theta too flat) shall give positive value control output (elevator up)
							Theta_error_sum += Theta_error;
							// including the speed as parameter to reduce the control Gains
							Kp_Theta = (Kp_Theta-speed_filt > 0)?(Kp_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
							Ki_Theta = (Ki_Theta-speed_filt > 0)?(Ki_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
							Kd_Theta = (Kd_Theta-speed_filt > 0)?(Kd_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
													
							
							// ctrl_out[elevator] = (trimmed_elevator - (Kp_Theta*-Theta_error + Ki_Theta*Theta_error_sum*sample_time + Kd_Theta*(Theta_error_prev - Theta_error)/sample_time));
							ctrl_out[elevator] = (trimmed_elevator + (Kp_Theta*Theta_error + Ki_Theta*Theta_error_sum*sample_time + Kd_Theta*(Theta_error_prev - Theta_error)/sample_time));
							if(ctrl_out[elevator] > RIGHT) ctrl_out[elevator] = RIGHT;
							else if (ctrl_out[elevator] < LEFT) ctrl_out[elevator] = LEFT;
							Theta_error_prev = Theta_error;
							
							// Tuning the K_p of the Aileron P closed loop
							Kp_Phi += ctrl_in[stick_r_up_down]-SERVO_TRIM_AILERON;
							if (Kp_Phi < 0) Kp_Phi = 0;
							if (Kp_Phi	 > 254) Kp_Phi = 255;
							
							Phi_error = Phi - Phi_hold;
							ctrl_out[aileron] = trimmed_aileron - (Kp_Phi*Phi_error_sum);
						}
										
					break;
					
					case DAMPED_CTRL:
					// Heading control test
					// entry condition
					
					if(state_change == TRUE)
					{
						write_string_ln("DAMPED CONTROL");
						// Fly 90 degrees to the right
						heading_hold = heading;
						Theta_hold = Theta;
						Phi_hold_0 = Phi;	// This is introduced to have an outer loop trimmed Phi value
						// Setting current input as "trimmed input"
						trimmed_elevator = ctrl_out[elevator];
						trimmed_aileron = ctrl_out[aileron];
						trimmed_motor = ctrl_out[motor];
						trimmed_rudder = ctrl_out[rudder];
						//resetting the errors
						heading_error = 0;
						heading_error_sum = 0;
						speed_error = 0;
						speed_error_sum = 0;
						Theta_error = 0;
						Theta_error_sum = 0;
						Phi_error = 0;
						Phi_error_sum = 0;
						write_string("Heading hold - Trim Ail: ");write_var(heading_hold);write_string(" - ");write_var(trimmed_aileron);write_string_ln("");
						// Delay timer
						test_cnt = 0;
					}
					
					if(test_cnt < 132) test_cnt++;
					if(test_cnt == 132)	// after 4 seconds (base: sample time) it will make a step change in the heading command
					{
						heading_hold += 90;
						write_string("Step heading + 90: ");write_var(heading_hold);write_string_ln("");
						test_cnt++;
					}
					
					// keep trimmed control for rudder and motor
					ctrl_out[motor] = trimmed_motor;
					ctrl_out[rudder] = trimmed_rudder;
					
					ctrl_out[elevator] =	NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
					
					//********************************************
					// Theta control with elevator
					// Problem with altitude control is the poor resolution of 1 Hz.
// 											
// 					Theta_error = Theta_hold - Theta; // Positive Error (theta too flat) shall give positive value control output (elevator up)
// 					Theta_error_sum += Theta_error;
// 					// including the speed as parameter to reduce the P control Gain
// 					//Kp_Theta = (Kp_Theta-speed_filt > 0)?(Kp_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
// 					// control formula
// 					ctrl_out[elevator] = (trimmed_elevator + (Kp_Theta*Theta_error + Ki_Theta*Theta_error_sum*sample_time));
// 					if(ctrl_out[elevator] > RIGHT) ctrl_out[elevator] = RIGHT;
// 					else if (ctrl_out[elevator] < LEFT) ctrl_out[elevator] = LEFT;
					
					
					//**********************************************	
					// Heading hold control	
					// input: current heading, heading_com
					// control: Phi	
					// controller: PI control	(heading reading is very noisy...diff part gives too noisy outputs)
					// when Phi positive we do a right turn, when Phi negative, we do a left turn
						
					// calculating the heading error
					heading_error = heading_hold - heading;
					// to identify which way to go we calculate (do not travel 270 deg left, when shortest is 90 deg right)
					int16_t heading_ctrl_2 = 360 + heading_error;
					if (heading_ctrl_2 < 180)
					{
						heading_error = heading_ctrl_2;
					}
					// error sum for I control
					heading_error_sum += heading_error;
					// controller formula
					Phi_hold = Phi_hold_0 + (Kp_head*heading_error + Kd_head*(heading_error-heading_error_prev)*sample_time);
					// error previous for d control
					heading_error_prev = heading_error;
					
					// Limiting the Bank Angle command to +- 20 maximum
					int8_t bank_limit_2 = 20;
					if (Phi_hold < -bank_limit_2) Phi_hold = -bank_limit_2;
					else if (Phi_hold > bank_limit_2) Phi_hold = bank_limit_2;
					
						
					// *********************************************	
					// Bank angle control with aileron
					// input: Phi
					// output: aileron command
					
					Phi_error = Phi - Phi_hold;		// Positive Error (too hard bank right)shall give negative control (Aileron roll left)
					Phi_error_sum += Phi_error;
					
					// including the speed as parameter to reduce the control Gains
					//Kp_Phi = 5;	// if (K-speed > 0) then take (K-speed) else take (1)
					
					// controller formula						
					ctrl_out[aileron] = (trimmed_aileron - (Kp_Phi*Phi_error + Ki_Phi*Phi_error_sum*sample_time));// + Kd_Phi*(Phi_error_prev - Phi_error)/sample_time));
					// limiting controls
					// limiting contr);
					if(ctrl_out[aileron] > RIGHT) ctrl_out[aileron] = RIGHT;
					else if (ctrl_out[aileron] < LEFT) ctrl_out[aileron] = LEFT;
					
						
					break;
					
					case HOLD_CTRL:
						// entry condition
						if(state_change == TRUE)
						{
							write_string_ln("HOLD CONTROL");
							// Setting current values as control values
							// High level loops
							//alt_hold = altitude_filt;		// controlling barometric altitude
							alt_hold = altitude_filt;
							alt_hold_0 = alt_hold;			// used for finding previous altitude again
							heading_hold = heading_target;	// controlling heading
							
							// middle loops (if required)
							Phi_hold_0 = Phi;			// using current phi as phi_0
							Theta_hold = Theta;			// using current theta as theta_0
							speed_hold = speed;							
							
							// Setting current input as "trimmed input"
							trimmed_elevator = ctrl_out[elevator]; 
							trimmed_aileron = ctrl_out[aileron];
							trimmed_motor = ctrl_out[motor];
							trimmed_rudder = ctrl_out[rudder];
							
							//****************************
							// Resetting the errors
							// altitude
							alt_error = 0;
							alt_error_sum = 0;
							alt_error_prev = 0;
							// speed
							speed_error = 0;
							speed_error_sum = 0;
							// Theta
							Theta_error = 0;
							Theta_error_sum = 0;
							Theta_error_prev = 0;
							// Phi
							Phi_error = 0;
							Phi_error_sum = 0;
							Phi_error_prev = 0;
							// *****************************
							write_string("Target Alt - Heading - Speed: ");write_var(alt_hold);write_string(" - ");write_var(heading_hold);write_string(" - ");write_var_ln(speed_hold);
						}
						// speed control not yet established (keeping it as is)
						ctrl_out[motor] = trimmed_motor ;
						
						// Using Rotary knob for altitude hold control (testing)
						if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120 && knob_flag == 1)	// NEUTRAL (Up Middle
						{
							// normal position -> taking alt as when state was entered
							alt_hold = alt_hold_0;
							knob_flag = 0;
							write_string("ALT Hold ");write_var_ln(alt_hold);

						}
						else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150 && knob_flag == 0) // Left One Click
						{
							// when knob turned -> Altitude = actual + 10
						 	alt_hold = alt_hold + 10;
							knob_flag = 1; 
							write_string(" Alt Hold ");write_var_ln(alt_hold);
						}
						
						//******************************
						// LONGITUDINAL AUTOPILOT
						
 						//******************************************************
 						// Altitude hold
 						// Input: Altitude reading (poor 1Hz resolution of barometer)
 						// output: Theta_com (Pitch Angle command)
 						// Controller: P control (as per SCILAB the most efficient for Altitude hold outer loop)
 						alt_error = alt_hold - altitude_filt;
 						Theta_hold = Kp_alt*(alt_error);
 						// Limiting Theta Angle command to +20 -10 maximum (= Saturation)
 						int8_t pitch_limit = 20;
 						if (Theta_hold < -(pitch_limit/2)) Theta_hold = -(pitch_limit/2); // only 10 deg neg due to speed gaining
 						else if (Theta_hold > pitch_limit) Theta_hold = pitch_limit;
						
 						//******************************************************
 						
 						//******************************************************
 						// Theta control with elevator
 						// Input: Theta command from Altitude Hold loop
 						// Output: elevator command pid loop
 						// Controller: PID Controller
						// Gains: K_p = 10 K_i = 10 K_d = 5
 						Theta_error = Theta - Theta_hold; // as per Simulation in SCILAB this convention is best
 						Theta_error_sum += Theta_error;
 						
						 // controller formula for the necessary control change						 
 						ctrl_out_PID[elevator] = (Kp_Theta*Theta_error + Ki_Theta*Theta_error_sum*sample_time + Kd_Theta*(Theta_error_prev - Theta_error)/sample_time);
 
 						Theta_error_prev = Theta_error;
						//******************************************************
						
						//******************************************************
						// Pitch damping
						// Input: turn rate q
						// Output: elevator command damping
						// Gains: K_p_q = 9
						ctrl_out_DAMP[elevator] = K_p_q * (q_filt);
						//******************************************************
						
						//******************************************************
						// Combining the calculated inputs elevator and limiting elevator command
						ctrl_out[elevator] = trimmed_elevator + ctrl_out_PID[elevator] - ctrl_out_DAMP[elevator];// + ctrl_out_DAMP[elevator]);
						if(ctrl_out[elevator] > RIGHT) ctrl_out[elevator] = RIGHT;
						else if (ctrl_out[elevator] < LEFT) ctrl_out[elevator] = LEFT;
						//******************************************************
						
						//**********************************************
// 						// LATERAL AUTOPILOT

// 						//**********************************************	
// 						// Heading hold control	
// 						// input: current heading, heading_com
// 						// control: Phi	
// 						// controller: PI control	(heading reading is very noisy...diff part gives too noisy outputs)
// 						// when Phi positive we do a right turn, when Phi negative, we do a left turn
// 						
// 						// Using Rotary knob for heading control (testing)
// 						if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120)
// 						{
// 							//heading_hold += 90;
// 							
// 						}
// 						else if(ctrl_in[rotary_knob]<150 && ctrl_in[rotary_knob]>135)
// 						{
// 							// normal position
// 						}
// 						else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150)
// 						{
// 							//heading_hold -= 90;
// 						}
// 						
// 						// calculating the heading error
// 						heading_error = heading_hold - heading;
// 						// to identify which way to go we calculate (do not travel 270 deg left, when shortest is 90 deg right)
// 						int16_t heading_ctrl = 360 + heading_error;
// 						if (heading_ctrl < 180)
// 						{
// 							heading_error = heading_ctrl;
// 						}
// 						// error sum for I control
// 						heading_error_sum += heading_error;
// 						// controller formula
// 						Phi_hold = Phi_hold_0 + (Kp_head*heading_error + Kd_head*(heading_error-heading_error_prev)*sample_time);
// 						// error previous for d control
// 						heading_error_prev = heading_error;
// 						
// 						// Limiting the Bank Angle command to +- 20 maximum (= Saturation)
// 						int8_t bank_limit = 20;
// 						if (Phi_hold < -bank_limit) Phi_hold = -bank_limit;
// 						else if (Phi_hold > bank_limit) Phi_hold = bank_limit;
// 												
// 						
// 						// *********************************************	
// 						// Bank angle control with aileron
// 						
// 						Phi_error = Phi - Phi_hold;		// Positive Error (too hard bank right)shall give negative control (Aileron roll left)
// 						Phi_error_sum += Phi_error;
// 						// including the speed as parameter to reduce the control Gains
// 						//Kp_Phi = (Kp_Phi-speed_filt > 0)?(Kp_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
// 						//Ki_Phi = (Ki_Phi-speed_filt > 0)?(Ki_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
// 						//Kd_Phi = (Kd_Phi-speed_filt > 0)?(Kd_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
// 						
// 						ctrl_out[aileron] = (trimmed_aileron - (Kp_Phi*Phi_error + Ki_Phi*Phi_error_sum*sample_time));// + Kd_Phi*(Phi_error_prev - Phi_error)/sample_time));
// 						Phi_error_prev = Phi_error;
// 						if(ctrl_out[aileron] > RIGHT) ctrl_out[aileron] = RIGHT;
// 						else if (ctrl_out[aileron] < LEFT) ctrl_out[aileron] = LEFT;	
// 						
// 						//**********************************************
// 						// Speed control (PI only -> noisy signal) with motor
// 						speed_error = speed_hold - speed; // Positive Error (too slow) shall give positive input (motor increase)
// 						speed_error_sum += speed_error;	
// 						// when too slow (bracket turns positive) the motor gets increased
// 						ctrl_out[motor] = trimmed_motor + (Kp_speed*speed_error + Ki_speed * speed_error_sum * sample_time);
// 						if(ctrl_out[motor] > RIGHT) ctrl_out[motor] = RIGHT;
// 						else if (ctrl_out[motor] < LEFT) ctrl_out[motor] = LEFT;
// 						speed_errror_prev = speed_error;
// 						
// 						// Damping of yaw
// 						int8_t yaw_damping = 2;	// needs to be checked
// 						if(speed_filt>20) yaw_damping /= 2;
// 						ctrl_out[rudder] = trimmed_rudder + (r_filt*yaw_damping);
// 						
						
						
						// camera gimbal servo
// 						ctrl_out[camera_y] = NEUTRAL_GIMB_Y + ((ctrl_in[stick_r_up_down]-SERVO_TRIM_GIMB_Y)*SERVO_GAIN_GIMB_Y);//+((ctrl_in[stick_r_up_down]-SERVO_TRIM_GIMB_Y)*SERVO_GAIN_ELEVATOR);
// 						if (ctrl_out[camera_y] < MAX_GIMB_Y_UP) ctrl_out[camera_y] = MAX_GIMB_Y_UP;
// 						if (ctrl_out[camera_y] > MAX_GIMB_Y_DN) ctrl_out[camera_y] = MAX_GIMB_Y_DN;
// 									
// 						ctrl_out[camera_z] = SERVO_GIMB_Z_STOP-((ctrl_in[stick_r_left_right]-141)*10);
						// the z axis Servo is a hacked 360 deg Servo without position information
						// There is no feedback from the servo on its position.
						// For leaving and reaching a new position, only time integration would work...
						
					break;
												
					
				}
					// writing all data to serial port if enabled
					if(serial_log == TRUE)	// set serial log variable to enable / disable output
					{
						write_var(ctrl_out[motor]);write_string(";");
						write_var(ctrl_out[aileron]);write_string(";");
						write_var(ctrl_out[elevator]);write_string(";");
						write_var(ctrl_out[rudder]);write_string(";");
						write_var(ctrl_out[flap]);write_string(";");
						write_var(Phi);write_string(";");
						write_var(Theta);write_string(";");
						write_var(heading);	write_string(";");
						write_var(altitude_filt);write_string(";");
						write_var(speed_filt);write_string(";");
						write_var(alt_error);write_string(";");
						write_var(heading_error);
						
						// In case at least once a GPS Signal has been received, the GPS Info will also be printed
					
						if(strcmp(GPS_RMC[GPS_RMC_LONGITUDE],"")!=0)
						{
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_TIME]);
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_LONGITUDE]);
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_LATITUDE]);
							//write_string(";");
							//write_string(GPS_RMC[GPS_RMC_PATH]);
							write_string(";");
							write_string(GPS_GGA[GPS_GGA_ALTMSL]);
							write_string(";");
							write_var(heading_target);
						}
					
						write_string_ln(";");
					}
				Ctrl_Mode_prev = Ctrl_Mode;		// Setting previous State
				
				ctrl_in_prev[9] = ctrl_in;
				//GPS_string_prev[GPS_LONGITUDE][] = GPS_string[GPS_LONGITUDE];
			}		
			
			
			// Toggling the Watchdog (Reset)
			wdt_reset();
    }
}