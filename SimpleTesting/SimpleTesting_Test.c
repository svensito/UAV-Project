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
int task_gyro	= FALSE;
int task_acc	= FALSE;
int task_mag	= FALSE;
int task_temp	= FALSE;
int task_baro	= FALSE;
int task_speed	= FALSE;

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
	
	// Timer Sample time
	float sample_time = 0.05;
	
	// Circle Number Pi
	float Pi =  3.1415926535;
	
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
	int8_t Kp_alt = 10;
	int8_t Ki_alt = 10;
	int8_t Kd_alt = 10;	
	long trimmed_elevator = 0;				   
	// Speed Data
	int16_t speed_raw,speed_filt = 0;	// raw speed and filtered speed
	float alpha_speed = 0.8; // alpha element [0;1] -> alpha 0: only raw input (noise free)
												// -> alpha 1: only filtered input (only noise)
	float speed = 0;
	float speed_hold = 0;
	// Euler Angles Data
	float Theta, Theta_hold, Theta_error, Theta_error_sum, Theta_error_prev = 0;
	int8_t Kp_Theta = 30;
	int8_t Kd_Theta = 1;
	int8_t Ki_Theta = 30;
	float Phi = 0;
	float Psi = 0;
	float Phi_hold = 0;
	float Phi_error = 0;
	float Phi_error_sum = 0;
	float Phi_error_prev = 0;
	int8_t Kp_Phi = 30;
	int8_t Kd_Phi = 1;
	int8_t Ki_Phi = 30;
	
	long trimmed_aileron = 0;
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
	int16_t mag = 0;
	int16_t mag_hold = 0;
	//struct three_values test;
	int8_t tune_cnt = 0;
	int16_t bla_cnt = 0;
	int8_t send_cnt = 0;
	
	
		_delay_ms(1000);
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
			
			if(task_flag == 2)
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
					
					/*
					Theta += (q_filt_prev+ (q_filt - q_filt_prev)/2) * 0.02;	// 20 ms sample Time
					Phi += (p_filt_prev+ (p_filt - p_filt_prev)/2) * 0.02;
					Phi += (r_filt_prev+ (r_filt - r_filt_prev)/2) * 0.02;
					*/
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
						
				if(task_mag == TRUE)	mag = mag_read();
				if(task_speed == TRUE)
				{
					speed_raw = ADC_read_speed();
					// low pass filter on the speed reading
					speed_filt = speed_raw*(1-alpha_speed) + (alpha_speed*speed_filt);
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
				// changing from knob to normal switch for mode control
				if(ctrl_in[5]>144) Ctrl_Mode = HOLD_CTRL;
				else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) Ctrl_Mode = DIRECT_CTRL;
				else Ctrl_Mode = DIRECT_CTRL;				
										
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
							if (ctrl_in > ctrl_in_prev)
							{
								
							}
						 
						 ctrl_out[motor]	=	NEUTRAL+((ctrl_in[stick_l_up_down]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);		
						 ctrl_out[aileron]	=	NEUTRAL+((ctrl_in[stick_r_left_right]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);
						 ctrl_out[elevator] =	NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
						 ctrl_out[rudder]	=	NEUTRAL+((ctrl_in[stick_l_left_right]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);
						 
						 // Flap Delay
						 uint16_t flap_target = 0;	// defining the target flap position
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
					
					case NORMAL_CTRL:
							
					break;
					
					case DAMPED_CTRL:
						// Take Off Control
						// Motor Max
						// Hold Magnetic Heading
						// Hold Theta
						// Hold Phi
						// Hold Psi / r
						// correct with rudder
						// reach target speed -> pull up
					break;
					
					case HOLD_CTRL:
						// entry condition
						if(state_change == TRUE)
						{
							write_string_ln("HOLD CONTROL");
							// Setting current values as control values
							alt_hold = altitude_filt; 
							Phi_hold = Phi;
							Theta_hold = Theta;
							mag_hold = mag;
							speed_hold = speed;
							// Setting current input as "trimmed input"
							trimmed_elevator = ctrl_out[elevator]; 
							trimmed_aileron = ctrl_out[aileron];
							//resetting the errors
							alt_error = 0;
							Theta_error = 0;
							Phi_error = 0;
							alt_error_sum = 0;
							Theta_error_sum = 0;
							alt_error_prev = 0;
							Theta_error_prev = 0;
							Phi_error_sum = 0;
							Phi_error_prev = 0;
							write_string("Trim El / Ail: ");write_var(trimmed_elevator);write_string(" - ");write_var_ln(trimmed_aileron);
							write_string("Target Alt / Bank: ");write_var(Theta_hold);write_string(" - ");write_var_ln(Phi_hold);
						}
						
						// Other controls as usual
						//POSITION1 = NEUTRAL+((ctrl_in[0]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);		// Motor
						//POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);		// Aileron	right:		POSITION++	left: POSITION--
						//POSITION3 = NEUTRAL+((ctrl_in[2]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);	// Elevator	up:			POSITION++	down: POSITION--
						//POSITION4 = NEUTRAL+((ctrl_in[3]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);		// Rudder
						ctrl_out[motor] = NEUTRAL+((ctrl_in[stick_l_up_down]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);
						//ctrl_out[aileron] = NEUTRAL+((ctrl_in[stick_r_left_right]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);
						//ctrl_out[elevator] = NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
						ctrl_out[rudder] = NEUTRAL+((ctrl_in[stick_l_left_right]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);
						
						// Flying without Flaps only!
						// We use the knob to increase gain of the control
						if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120) 
						{
							control_gain = 1;
						}
						else if(ctrl_in[rotary_knob]<150 && ctrl_in[rotary_knob]>135) 
						{
							control_gain = 2;
						}
						else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150) 
						{
							control_gain = 3;
						}
						// Theta control with elevator
						
						Theta_error = Theta_hold - Theta; // 
						Theta_error_sum += Theta_error;
						// including the speed as parameter to reduce the control Gains
						Kp_Theta = (Kp_Theta-speed_filt > 0)?(Kp_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						Ki_Theta = (Ki_Theta-speed_filt > 0)?(Ki_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						Kd_Theta = (Kd_Theta-speed_filt > 0)?(Kd_Theta-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						
						ctrl_out[elevator] = (trimmed_elevator - control_gain * (Kp_Theta*-Theta_error + Ki_Theta*Theta_error_sum*sample_time + Kd_Theta*(Theta_error_prev - Theta_error)/sample_time));
						if(ctrl_out[elevator] > RIGHT) ctrl_out[elevator] = RIGHT;
						else if (ctrl_out[elevator] < LEFT) ctrl_out[elevator] = LEFT;
						Theta_error_prev = Theta_error;
						
						// altitude control with elevator (Position 3)
						/*
						alt_error = altitude_filt - alt_hold; // Positive Error shall give negative control (Elevator down)
						alt_error_sum += alt_error;
						Theta_hold = (Theta_hold - control_gain * (Kp_alt*alt_error + Ki_alt*alt_error_sum*0.05 + Kd_alt*(alt_error_prev - alt_error)/0.05));
						alt_error_prev = alt_error;
						*/
							
						// bank angle control with aileron (Position 2)
						
						Phi_error = Phi - Phi_hold;		// Positive Error shall give negative control (Aileron roll left)
						Phi_error_sum += Phi_error;
						// including the speed as parameter to reduce the control Gains
						Kp_Phi = (Kp_Phi-speed_filt > 0)?(Kp_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						Ki_Phi = (Ki_Phi-speed_filt > 0)?(Ki_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						Kd_Phi = (Kd_Phi-speed_filt > 0)?(Kd_Phi-speed_filt):1;	// if (K-speed > 0) then take (K-speed) else take (1)
						
						ctrl_out[aileron] = (trimmed_aileron - control_gain * (Kp_Phi*Phi_error + Ki_Phi*Phi_error_sum*sample_time + Kd_Phi*(Phi_error_prev - Phi_error)/sample_time));
						Phi_error_prev = Phi_error;
						if(ctrl_out[aileron] > RIGHT) ctrl_out[aileron] = RIGHT;
						else if (ctrl_out[aileron] < LEFT) ctrl_out[aileron] = LEFT;	
			
						

					break;
												
					
				}
					// writing all data to serial port
					
					write_var(ctrl_out[motor]);write_string(";");write_var(ctrl_out[aileron]);write_string(";");
					write_var(ctrl_out[elevator]);write_string(";");write_var(ctrl_out[rudder]);write_string(";");
					write_var(ctrl_out[flap]);write_string(";");
					write_var(Phi);write_string(";");
					write_var(Theta);write_string(";");
					write_var(mag);write_string(";");write_var(altitude_filt);write_string(";");
					write_var(speed_filt);
					// In case at least once a GPS Signal has been received, the GPS Info will also be printed
					if(strcmp(GPS_RMC[GPS_RMC_MODE],"")!=0 || strcmp(GPS_GGA[GPS_GGA_MODE],"")!=0) 
					{
						write_string(";");
						write_string(GPS_RMC[GPS_RMC_LONGITUDE]);
						write_string(";");
						write_string(GPS_RMC[GPS_RMC_LATITUDE]);
					
					}
					write_string_ln(";");
					
				Ctrl_Mode_prev = Ctrl_Mode;		// Setting previous State
				ctrl_out_prev[5] = ctrl_out;	// Setting previous controls
				ctrl_in_prev[9] = ctrl_in;
				//GPS_string_prev[GPS_LONGITUDE][] = GPS_string[GPS_LONGITUDE];
			}
			
			
			if (task_flag == 1)
			{
				task_flag = 0;
				
				if(strcmp(GPS_RMC[GPS_RMC_LONGITUDE],"")!=0)
				{
					Ctrl_Mode = TUNE_CTRL;
				}
				
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
				
				switch (Ctrl_Mode)
				{
					case DIRECT_CTRL:
							write_string_ln("Direct");
					break;
					
					case TUNE_CTRL:
							// Make a step change and check the response
							if(state_change == TRUE)
							{
								state_change = FALSE;
								write_string_ln("*TUNING LONG STEP*");
								//trimmed_motor = ctrl_out[motor];
								trimmed_aileron = ctrl_out[aileron];
								trimmed_elevator = ctrl_out[elevator];
								ctrl_out[elevator] = trimmed_elevator;
								//ctrl_out[motor] = trimmed_motor;
								ctrl_out[aileron] = trimmed_aileron;
								Phi_hold = Phi;
								Phi_error = 0;
								Phi_error_prev = 0;
								Phi_error_sum = 0;
								Theta_error = 0;
								Theta_hold = Theta;
								Kp_Theta = 0;
							}

									
							if(tune_cnt < 40) tune_cnt++;
							if(tune_cnt == 40)	// after 2 seconds it will make a step input to the elevator
							{
								write_string_ln("Step");
								ctrl_out[elevator] +=  200;
								tune_cnt += 1;
							}
							write_var_ln(ctrl_out[elevator]);
					
					break;
					
				}
				
				Ctrl_Mode_prev = Ctrl_Mode;
			}
			// camera gimbal servo
			/*
			ctrl_out[camera_y] = NEUTRAL_GIMB_Y + ((ctrl_in[stick_r_up_down]-SERVO_TRIM_GIMB_Y)*SERVO_GAIN_GIMB_Y);//+((ctrl_in[stick_r_up_down]-SERVO_TRIM_GIMB_Y)*SERVO_GAIN_ELEVATOR);
			if (ctrl_out[camera_y] < MAX_GIMB_Y_UP) ctrl_out[camera_y] = MAX_GIMB_Y_UP;
			if (ctrl_out[camera_y] > MAX_GIMB_Y_DN) ctrl_out[camera_y] = MAX_GIMB_Y_DN;
						
			ctrl_out[camera_z] = SERVO_GIMB_Z_STOP-((ctrl_in[stick_r_left_right]-141)*10);
			*/
			// the z axis Servo is a hacked 360 deg Servo without position information
			// There is no feedback from the servo on its position.
			// For leaving and reaching a new position, only time integration would work...
			
			/*
			if(strcmp(uart_word,"left")==1) ctrl_out[camera_z] = SERVO_GIMB_Z_LEFT;
			if(strcmp(uart_word,"stop")==1) ctrl_out[camera_z] = SERVO_GIMB_Z_STOP;
			if(strcmp(uart_word,"right")==1) ctrl_out[camera_z] = SERVO_GIMB_Z_RIGHT;
			*/			
			// Toggling the Watchdog (Reset)
			wdt_reset();
    }
}