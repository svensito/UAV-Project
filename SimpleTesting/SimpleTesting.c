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
#include "OLED_display.h"
//------------------------------------------------------

//------------------------------------------------------
// Local declarations and variables

//******************************************************************
// These are the task flags, set them to activate / deactivate task
int task_gyro	= TRUE;		// reading gyro data enabled (TRUE) /disabled (FALSE)
int task_acc	= TRUE;		// reading acc data enabled (TRUE) /disabled (FALSE)
//int task_mag	= FALSE;	// reading mag data enabled (TRUE) /disabled (FALSE) -> too unreliable for navigation
//int task_temp	= FALSE;	// reading temp data enabled (TRUE) /disabled (FALSE)
int task_baro	= FALSE;		// reading baro data enabled (TRUE) /disabled (FALSE)
int task_speed	= FALSE;		// reading ADC speed data enabled (TRUE) /disabled (FALSE)
int task_OLED	= TRUE;		// OLED task
int serial_log	= FALSE;		// serial output enabled (TRUE) /disabled (FALSE)
//******************************************************************

/********************
 General Variables
 ********************/



/********************
 Control Modes
 ********************/
uint8_t Ctrl_Mode = DIRECT_CTRL;		// First Test with Direct Law Only: DIRECT_LAW or NORMAL_LAW possible
uint8_t Ctrl_Mode_prev = DIRECT_CTRL;	// Initialize as the same Mode
uint8_t state_change = FALSE;			// Needed as a flag for state change

// Control Knob
uint8_t Knob_State = 0;
uint8_t prev_Knob_State = 0;
uint16_t Knob_Stepwidth = 400;
int8_t knob_flag = 0;

/********************
 Control surface trims
 ********************/
long trimmed_elevator = 0;
long trimmed_motor = 0;
long trimmed_aileron = 0;
long trimmed_rudder = 0;

/********************
 Flap Controls
 ********************/
// Flap Delay
uint16_t flap_setpoint = FLAP_UP;	// defining the desired setpoint flap position
uint8_t flap_delta = 10;	// giving the speed of flap extension
// 10 will give around 2 seconds for movement in between two positions

/********************
 Altitude Data
 ********************/
int16_t altitude_raw,altitude_filt,alt_hold, alt_hold_0 = 0;	// raw altitude and filtered altitude
int16_t alt_error = 0;
int16_t alt_error_sum = 0;
int16_t alt_error_prev = 0;
float alpha_alt = 0.3; // alpha element [0;1] -> alpha 0: only raw input (noise free)
						// -> alpha 1: only filtered input (only noise)
int8_t Kp_alt = 1;
int8_t Ki_alt = 10;
int8_t Kd_alt = 10;


/********************
 Speed Data
 ********************/
int16_t speed_raw,speed_filt, speed_filt_1, speed_filt_2, speed_filt_3, speed_filt_4, speed_filt_5 = 0;	// raw speed and filtered speed
float alpha_speed_1 = 0.3; // alpha element [0;1] -> alpha 0: only raw input (noise free)
float alpha_speed_2 = 0.3;											// -> alpha 1: only filtered input (only noise)
float alpha_speed_3 = 0.3;
float alpha_speed_raw = 0.1;
float speed, speed_cal = 0;
float speed_hold, speed_error, speed_error_sum, speed_error_prev = 0;
int8_t Kp_speed = 1;
//int8_t Kd_speed = 1;	// No differential part as signal too noisy
int8_t Ki_speed = 1;
uint8_t speed_cnt = 0;	// needed for decreasing speed reading resolution


/********************
 Euler Angles Data
 ********************/
// Turn rates Data
int8_t p_raw,q_raw,r_raw,p_filt,q_filt,r_filt = 0; // raw turn rates, filtered turn rates,previous turn rates
float alpha_turn = 0.7;
// Pitch Angle Theta
float Theta, Theta_hold, Theta_error, Theta_error_sum, Theta_error_prev = 0;
int8_t Kp_Theta = 10;	// as per simulation in SCILAB this are very good gains for a wide range of speed...
int8_t Kd_Theta = 1;
int8_t Ki_Theta = 10;
int8_t K_p_q = 9;
// Roll Angle Phi
float Phi = 0;
float Phi_hold, Phi_hold_0 = 0;
float Phi_error = 0;
float Phi_error_sum = 0;
float Phi_error_prev = 0;
int8_t Kp_Phi = 20;	// as per simulation in scilab
int8_t Kd_Phi = 3;
int8_t Ki_Phi = 10;
// Yaw Angle Psi
float Psi = 0;

/********************
 Acceleration Data
 ********************/
// Acceleration data
int16_t acc_x_raw, acc_y_raw, acc_z_raw,acc_x_filt, acc_y_filt, acc_z_filt,acc_x_filt_prev, acc_y_filt_prev, acc_z_filt_prev = 0;
float alpha_acc = 0.3;
float Theta_acc = 0;	// Theta based on the Accelerometer reading
float Phi_acc = 0;		// Phi based on the Accelerometer reading

/********************
 Kalman Filter Data
 ********************/
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

/********************
 Magnetometer Data
 ********************/
int16_t heading, heading_GPS = 0;
int16_t heading_goal = 0;
int16_t heading_hold, heading_error, heading_error_prev, heading_error_sum  = 0;
int8_t Kp_head = 1;
int8_t Kd_head = 1;
int8_t Ki_head = 0;		// simulation showed that PD is convenient for Heading Track

/********************
 GPS Data
 ********************/
int32_t GPS_POS_CURRENT_X, GPS_POS_CURRENT_Y, GPS_POS_GOAL_X, GPS_POS_GOAL_Y,GPS_POS_HOME_X, GPS_POS_HOME_Y, GPS_POS_DIF_X, GPS_POS_DIF_Y = 0;
int32_t GPS_POS_GOAL_WP[5][3] = {{0,0,0},		// Waypoint 1: POSX, POSY, ALT
								 {0,0,0},		// Waypoint 2: POSX, POSY, ALT
								 {0,0,0},		// Waypoint 3: POSX, POSY, ALT
								 {0,0,0},		// Waypoint 4: POSX, POSY, ALT
								 {0,0,0}};		// Waypoint 5: POSX, POSY, ALT
#define WP1	0
#define WP2	1
#define WP3	2
#define WP4	3
#define WP5	4
#define WPX 0
#define WPY 1

int8_t GPS_POS_LAT_DEG, GPS_POS_LAT_MIN = 0;
int32_t GPS_POS_LAT_SEC = 0;
int32_t GPS_DIS_TO_GOAL = 0;
uint8_t GPS_home_set = FALSE;

/********************
 Navigation Lights
 ********************/
uint8_t lights_enabled = FALSE;			// Lights Enable Flag
uint8_t lights_commanded	= FALSE;	// Flag to check if the lights have been commanded already
uint8_t flash_count = 0;				// Strobe light counter

/********************
 Counter
 ********************/
uint8_t send_cnt = 0;
int8_t tune_cnt = 0;
int8_t test_cnt = 0;
uint8_t cal_cnt = 0;


// Task Timer definitions
volatile uint8_t task_flag = 0;		// this flag will be set every 20ms and reset in the application
uint8_t comp_count = 0;
// Timer Sample time
float sample_time = 0;


//------------------------------------------------------

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
	
	// Initialize I2C communication
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
	//if(task_mag == TRUE)	mag_start();
	if(task_speed == TRUE)	
	{
		ADC_start();
	}		
	// OLED
 	if (task_OLED == TRUE)
	{
		OLED_init();
		OLED_clear();
// 		OLED_send_string("GPS");
// 		OLED_set_position(0,1);
// 		OLED_send_string("HOME");
// 		OLED_set_position(0,2);
// 		OLED_send_string("LAT");
// 		OLED_set_position(0,3);
// 		OLED_send_string("LON");
	}
	
	
	// Starting the Servo PWM signal setting
	cli();	// disable interrupts
	Task_Timer();
	servo_init();
	// setting all needed timers to Interrupt mode
	TIMSK  = (0<<TICIE1)|(1<<OCIE1A)|(0<<OCIE1B)|(0<<TOIE1)|(1<<OCIE0)|(1<<TOIE2);
	sei();	//enable interrupts
	
	
				
	// Turn On the Watchdog
	WDT_on();
	
	// Define Navigation Lights Output Port D
	DDRD |= (1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7);
	// Turn off LEDs at instance
	PORTD &= ~((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
	
	// Turn on Light for check
	PORTD |= (1<<PD4);
	

		
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
					p_filt = p_raw;// * (1-alpha_turn) + (alpha_turn*p_filt);
					q_filt = q_raw;// * (1-alpha_turn) + (alpha_turn*q_filt);
					r_filt = r_raw;// * (1-alpha_turn) + (alpha_turn*r_filt);
										
				}
				struct acc_readings_obj acc_val;
				if(task_acc == TRUE)	
				{
					acc_val = acc_reading();
					acc_x_raw = acc_val.a_x;
					acc_y_raw = acc_val.a_y;
					acc_z_raw = acc_val.a_z;
					Theta_acc = atan2(acc_x_raw,-acc_z_raw)*180/PI;
					Phi_acc = atan2(-acc_y_raw,-acc_z_raw)*180/PI;
					//write_string("acc: "); write_var_ln(acc_reading());

				}
				if(task_baro == TRUE)// && (bla_cnt==100))	
				{
					altitude_raw = baro_read();
					// low pass filter on the altitude reading
					altitude_filt = altitude_raw*(1-alpha_alt) + (alpha_alt*altitude_filt);
					//bla_cnt = 0;
				}
						
				//if(task_mag == TRUE)	heading = mag_read(Phi,Theta); // heading is too unreliable for navigation
				if(task_speed == TRUE)
				{
					
					if (speed_cnt == 8)		// to reduce the sampling time of the speed reading...
					{
					
					speed_raw = ADC_read_speed();
					//write_var_ln(speed_raw);
					// low pass filter on the speed reading
								// current reading				// prev reading			//
					if (cal_cnt == 0)
					{
						while(cal_cnt < 10)
						{
							cal_cnt++;
							speed_cal += ADC_read_speed();
							
						}
						speed_cal /= 10;
					}
					speed_filt = (speed_raw-speed_cal)*0.1 + 0.9*speed_filt_1;
					speed_filt_1 = speed_filt;
					speed_cnt = 0;
					}
					//write_var(speed_raw);write_string(";");write_var_ln(speed_filt);
					speed_cnt++;
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
				
				write_var(Phi);write_string(";");
				write_var(Theta);write_string_ln(";");

				
				

				
				//******************************************
				// Detecting current position and current heading from GPS reading
				
				//GPS_POS_CURRENT_Y = "123456";
							
				GPS_POS_CURRENT_X = atol_new(GPS_RMC[GPS_RMC_LONGITUDE]);
				GPS_POS_CURRENT_Y = atol_new(GPS_RMC[GPS_RMC_LATITUDE]);
				
				heading_GPS = atol_new(GPS_RMC[GPS_RMC_PATH])/100;
				// Distance to goal location:
				// calculate the latitude difference. one degree latitude is 111km distance (wiki). divide by cos(angle).
				// 1 deg = 111km
				// 1 minute = 1,85km
				// 1 second = 0,031km
				GPS_POS_LAT_DEG = GPS_POS_DIF_Y/10000000;
				GPS_POS_LAT_MIN = (GPS_POS_DIF_Y/100000)-(GPS_POS_LAT_DEG*100);
				GPS_POS_LAT_SEC = (GPS_POS_DIF_Y)-((GPS_POS_LAT_DEG*10000000)+(GPS_POS_LAT_MIN*100000));
				GPS_DIS_TO_GOAL = GPS_POS_LAT_DEG*111000 + GPS_POS_LAT_MIN*1850 + (((GPS_POS_LAT_SEC*60*31)/100000));
				//write_var(GPS_POS_LAT_MIN);write_string(";");write_var(GPS_POS_LAT_SEC);write_string_ln(";");
				//write_var_ln(atol_new(GPS_RMC[GPS_RMC_LATITUDE][0])*10 + atol_new(GPS_RMC[GPS_RMC_LATITUDE][1]));
				
				// Setting the current location as "home position"
				// left stick: left down
				// right stick: right down
				/* _______________			 _______________
				   |			 |			 |			   |
				   |			 |			 |			   |
				   |	  '		 |			 |		'	   |
				   |  0			 |			 |			 0 |
				   |_____________|			 |_____________|
				
				*/
				if ((ctrl_in[stick_l_up_down]    <	110) &&		// limit down left stick
					(ctrl_in[stick_l_left_right] >	175) &&		// limit left left stick
					(ctrl_in[stick_r_up_down]	>	170) &&		// limit down right stick
					(ctrl_in[stick_r_left_right] >	170) &&		// limit right right stick
					(GPS_home_set == FALSE))
				{
					GPS_home_set = TRUE;
					GPS_POS_HOME_X = atol_new(GPS_RMC[GPS_RMC_LONGITUDE]);
					GPS_POS_HOME_Y = atol_new(GPS_RMC[GPS_RMC_LATITUDE]);
					write_string("Home Set Lat ");write_var(GPS_POS_HOME_Y);write_string(" Long ");write_var_ln(GPS_POS_HOME_X);
					
					// Determination of Waypoints
					// 1 second = 1/60 min = 0,01667 mins
					// 1 second = 1/60 min = 1/60 * 1850m = 30,8m
					// offset: 5 seconds = 150 meters = 5*0,01667 = 0,08335 minutes
					// Within the GPS sentence: add 8335 to get an offset of 5 seconds = 150 around the Home Position
					// GPS_POS Format: DDMMSSSSS -> 0.SSSSS * 60
					GPS_POS_GOAL_WP[WP1][WPX] = GPS_POS_HOME_X + 8335;			// WP1 -> X (LONG) Position |
					GPS_POS_GOAL_WP[WP1][WPY] = GPS_POS_HOME_Y;					// WP1 -> Y (LAT) Position	|	WP1 = EAST of HOME Pos
					GPS_POS_GOAL_WP[WP2][WPX] = GPS_POS_HOME_X;					// WP2 -> X Position		|
					GPS_POS_GOAL_WP[WP2][WPY] = GPS_POS_HOME_Y + 8335;			// WP2 -> Y Position		|	WP2 = NORTH of HOME Pos
					GPS_POS_GOAL_WP[WP3][WPX] = GPS_POS_HOME_X - 8335;			// WP3 -> X Position		|
					GPS_POS_GOAL_WP[WP3][WPY] = GPS_POS_HOME_Y;					// WP3 -> Y Position		|	WP2 = WEST of HOME Pos
					GPS_POS_GOAL_WP[WP4][WPX] = GPS_POS_HOME_X;					// WP4 -> X Position		|
					GPS_POS_GOAL_WP[WP4][WPY] = GPS_POS_HOME_Y-8335;			// WP4 -> Y Position		|	WP4 = SOUTH of HOME Pos
					GPS_POS_GOAL_WP[WP5][WPX] = GPS_POS_HOME_X;	// SPARE
					GPS_POS_GOAL_WP[WP5][WPY] = GPS_POS_HOME_Y;	// SPARE
					
				}
				//*******************************************
				
				//*******************************************
				/* Control of Navigation Lights
				   - Enabling of Lights
				   - Flash Lights
				   - Position Lights
				   - Landing Light
				   - Using Port D Pins 4,5,6,7 as Light controls
				   - 4: Strobe Lights
				   - 5: Nav Lights
				   - 6: Landing Lights
				*/
				
				// Enabling of lights
				// left stick: left down
				// right stick: right up
				/* _______________			 _______________
				   |			 |			 |			 0 |
				   |			 |			 |			   |
				   |	  '		 |			 |		'	   |
				   |  0			 |			 |			   |
				   |_____________|			 |_____________|
				
				*/
				if ((ctrl_in[stick_l_up_down]    <	110) &&		// limit down left stick
					(ctrl_in[stick_l_left_right] >	175) &&		// limit left left stick
					(ctrl_in[stick_r_up_down]	<	90) &&		// limit up right stick
					(ctrl_in[stick_r_left_right] >	170))		// limit right right stick
					{
						if (lights_enabled == FALSE && lights_commanded == FALSE)
						{
							lights_enabled = TRUE;
						}
						else if (lights_enabled == TRUE && lights_commanded == FALSE)
						{
							lights_enabled = FALSE;
						}
					}
					else lights_commanded = FALSE;
				
				lights_enabled = TRUE;
				if (lights_enabled == TRUE)
				{
					// Strobe light flash value
					uint8_t flash_value = 110;
					// Lights output:
					PORTD |= (1<<PD5);		// NAV LIGHTS ON
					if ((flap_setpoint == FLAP_1)||(flap_setpoint == FLAP_FULL))
					{
						PORTD |= (1<<PD6);		// LANDING LIGHTS ON
					}
					
					flash_count++;
					if (flash_count == flash_value)
					{
						// Strobe On
						PORTD |= (1<<PD4);
					}
					if (flash_count == (flash_value+2))
					{
						// Strobe off
						PORTD &= ~(1<<PD4);
					}
					if (flash_count == flash_value+6)
					{
						//Strobe on
						PORTD |= (1<<PD4);
					}
					if (flash_count == (flash_value+8))
					{
						//Strobe off
						PORTD &= ~(1<<PD4);
						flash_count = 0;
					}
					
				}
				else if(lights_enabled == FALSE)
				{
					// Keep all LEDS off
					PORTD &= ~((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
				}
				
				//*******************************************
				
				
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
						 
						 // Flap Control
						 if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120)		flap_setpoint = FLAP_UP;
						 else if(ctrl_in[rotary_knob]<150 && ctrl_in[rotary_knob]>135)	flap_setpoint = FLAP_1;
						 else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150)	flap_setpoint = FLAP_FULL;
						 
						 // drive flap with sample_time steps delay 
						 if(ctrl_out[flap] < flap_setpoint)		ctrl_out[flap]+= flap_delta;
						 else if(ctrl_out[flap] > flap_setpoint)	ctrl_out[flap]-= flap_delta;
						 else ctrl_out[flap] = flap_setpoint;
						 
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
						speed_error_prev = speed_error;
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
							write_string_ln("Damped CONTROL");
							// Setting current values as control values
							// High level loops
							//alt_hold = altitude_filt;		// controlling barometric altitude
							alt_hold = altitude_filt;
							alt_hold_0 = alt_hold;			// used for finding previous altitude again
							heading_hold = heading;	// controlling heading
							
							// middle loops (if required)
							Phi_hold_0 = Phi;			// using current phi as phi_0
							Phi_hold = Phi;				// using current phi as hold value
							Theta_hold = Theta;			// using current theta as theta_0
							speed_hold = speed_filt;	// current speed reading for speed control				
							
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
							write_string("Goal Alt - Heading - Speed: ");write_var(alt_hold);write_string(" - ");write_var(heading_hold);write_string(" - ");write_var_ln(speed_hold);
						}
						
 						//**********************************************
 						// SPEED CONTROL AUTOPILOT
 						
 						//**********************************************
 						// speed control not yet established (keeping it as is)
 						//ctrl_out[motor] = trimmed_motor ;
 						ctrl_out[aileron]	=	NEUTRAL+((ctrl_in[stick_r_left_right]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);
 						ctrl_out[elevator] =	NEUTRAL+((ctrl_in[stick_r_up_down]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);
 						ctrl_out[rudder]	=	NEUTRAL+((ctrl_in[stick_l_left_right]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);
 						// Speed control (PI only -> noisy signal) with motor
  						speed_error = speed_hold - speed_filt; // Positive Error (too slow) shall give positive input (motor increase)
  						speed_error_sum += speed_error;	
  						// when too slow (bracket turns positive) the motor gets increased
  						ctrl_out[motor] = trimmed_motor + Kp_speed*speed_error + (Ki_speed * speed_error_sum * sample_time);
 						//write_var(Ki_speed * speed_error_sum * sample_time);write_string(";");
 						 if(ctrl_out[motor] > MOTOR_LIM_HI) 
 						 {
 							 ctrl_out[motor] = MOTOR_LIM_HI;
 							 //speed_error_sum -= speed_error; // to stop 
 						 }
  						else if (ctrl_out[motor] < MOTOR_LIM_LOW) 
						 {
 							 ctrl_out[motor] = MOTOR_LIM_LOW;
 							 //speed_error_sum -= speed_error;
 						 }
  						speed_error_prev = speed_error;
 						
						//***********************************************
 						

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
							heading_hold = heading_GPS;	// controlling heading
							
							// middle loops (if required)
							Phi_hold_0 = Phi;			// using current phi as phi_0
							Phi_hold = Phi;				// using current phi as hold value
							Theta_hold = Theta;			// using current theta as theta_0
							speed_hold = speed;			// current speed reading for speed control				
							
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
							write_string("Goal Alt - Heading - Speed: ");write_var(alt_hold);write_string(" - ");write_var(heading_hold);write_string(" - ");write_var_ln(speed_hold);
						}
						
						// Using Poti to change between two altitudes
						// direct poti value might be too noisy for altitude values
						if ((ctrl_in[poti]-155) < 30)
						{
							alt_hold = alt_hold_0;
						}
						else
						{
							alt_hold = alt_hold_0 + 10;
						}
						
						
						//******************************************************
						//******************************************************
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
						ctrl_out[elevator] = trimmed_elevator + ctrl_out_PID[elevator] - ctrl_out_DAMP[elevator];
						if(ctrl_out[elevator] > RIGHT) ctrl_out[elevator] = RIGHT;
						else if (ctrl_out[elevator] < LEFT) ctrl_out[elevator] = LEFT;
						//******************************************************
						
						//******************************************************
						//******************************************************
 						// LATERAL AUTOPILOT

						// Heading problem: Compass is giving faulty results when roll angle <> 0
						// Heading solution: Using GPS heading information (GPS_RMC[GPS_RMC_PATH])
						
						//**********************************************
						// GPS GOAL control
						// Using Rotary knob for GPS and hold control (testing)
						if(ctrl_in[rotary_knob]<135 && ctrl_in[rotary_knob]>120 && knob_flag == 1)	// NEUTRAL (Up Middle
						{
							knob_flag = 0;
							// normal position -> normal hold control of actual course
							heading_hold = heading_GPS;

							write_string("Heading Hold ");write_var_ln(heading_hold);

						}
						else if(ctrl_in[rotary_knob]<160 && ctrl_in[rotary_knob]>150 && knob_flag == 0) // Left One Click
						{
							knob_flag = 1;
							// when knob turned -> GPS -> Return to home -> Home Position needs to be defined before (see above)
							GPS_POS_GOAL_X = GPS_POS_HOME_X;
							GPS_POS_GOAL_Y = GPS_POS_HOME_Y;
							

							//write_var(GPS_POS_CURRENT_X);write_string(";");write_var(GPS_POS_CURRENT_Y);write_string(";");
							GPS_POS_DIF_X = GPS_POS_GOAL_X - GPS_POS_CURRENT_X;	// define errors in such way that the course is correct!
							GPS_POS_DIF_Y = GPS_POS_GOAL_Y - GPS_POS_CURRENT_Y;
							//write_var(GPS_POS_DIF_X);write_string(";");write_var(GPS_POS_DIF_Y);write_string_ln(";");
							//heading_goal = atan2(GPS_POS_DIF_Y,GPS_POS_DIF_X)*(180/PI);
							
							// Test Goal coordinates: Berlin
							//GPS_POS_GOAL_X = 132400290;	// 10 Latitude digits
							//GPS_POS_GOAL_Y = 523138036;	// 10 Longitude digits
							
							heading_goal = atan2((GPS_POS_DIF_X),(GPS_POS_DIF_Y))*(180/PI);
							if (heading_goal < 0)
							{
								heading_goal += 360;
							}
							
							heading_hold = heading_goal;

							write_string("Return Home Head ");write_var_ln(heading_hold);
						}
											
 						//**********************************************	
 						// Heading hold control	
 						// input: current heading, heading_command
 						// control: Phi	
 						// controller: PD control (as per Scilab the most efficient controller)
 						// when Phi positive we do a right turn, when Phi negative, we do a left turn
 						heading_error = heading_GPS - heading_hold;	// as per Simulation
 						// to identify which way to go we calculate commanded heading (do not travel 270 deg left, when shortest is 90 deg right)
 						int16_t heading_ctrl = 360 + heading_error;
						if (heading_ctrl < 180) heading_error = heading_ctrl;
 						heading_error_sum += heading_error;
 						// The Phi_hold 0 does make sense when the sensor is not properly installed. 
 						// Otherwise it does not make sense!
 						Phi_hold = -(Kp_head*heading_error + Kd_head*(heading_error-heading_error_prev)*sample_time);
 						heading_error_prev = heading_error;
 						// Limiting the Bank Angle command to +- 10 maximum (= Saturation)
 						int8_t bank_limit = 10;
 						if (Phi_hold < -bank_limit) Phi_hold = -bank_limit;
 						else if (Phi_hold > bank_limit) Phi_hold = bank_limit;
						//**********************************************						
						
						// *********************************************	
						// Bank angle control with aileron
						// Input: Phi_hold = Phi_command
						// Output: Aileron Command

						Phi_error =  Phi_hold - Phi;		// as per Simulation in Scilab
						Phi_error_sum += Phi_error;
						ctrl_out[aileron] = (trimmed_aileron + (Kp_Phi*Phi_error + Ki_Phi*Phi_error_sum*sample_time) + Kd_Phi*(Phi_error_prev - Phi_error)/sample_time);
						Phi_error_prev = Phi_error;
						if(ctrl_out[aileron] > RIGHT) ctrl_out[aileron] = RIGHT;
						else if (ctrl_out[aileron] < LEFT) ctrl_out[aileron] = LEFT;	
						//***********************************************
						
						//*********************************************
						// Damping of yaw	-> Yaw Damping is not needed
						//int8_t yaw_damping = 2;	// needs to be checked
						//if(speed_filt>20) yaw_damping /= 2;
						//ctrl_out[rudder] = trimmed_rudder + (r_filt*yaw_damping);
						//**********************************************
						
						
						//**********************************************
						// SPEED CONTROL AUTOPILOT
						
 						// Speed control (PI only -> noisy signal) with motor
 						speed_error = speed_hold - speed_filt; // Positive Error (too slow) shall give positive input (motor increase)
 						speed_error_sum += speed_error;
 						// when too slow (bracket turns positive) the motor gets increased
 						ctrl_out[motor] = trimmed_motor + Kp_speed*speed_error + (Ki_speed * speed_error_sum * sample_time);
 						//write_var(Ki_speed * speed_error_sum * sample_time);write_string(";");
 						if(ctrl_out[motor] > MOTOR_LIM_HI)
 						{
	 						ctrl_out[motor] = MOTOR_LIM_HI;
	 						//speed_error_sum -= speed_error; // to stop
 						}
 						else if (ctrl_out[motor] < MOTOR_LIM_LOW)
 						{
	 						ctrl_out[motor] = MOTOR_LIM_LOW;
	 						//speed_error_sum -= speed_error;
 						}
 						speed_error_prev = speed_error;
 						
 						//***********************************************
						
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
						write_var(heading_GPS);	write_string(";");
						write_var(altitude_filt);write_string(";");
						write_var(speed_filt);write_string(";");
						write_var(alt_hold);write_string(";");
						write_var(Phi_hold);write_string(";");
						write_var(Theta_hold);write_string(";");
						write_var(speed_hold);
						
						// In case at least once a GPS Signal has been received, the GPS Info will also be printed
					
						if(strcmp(GPS_RMC[GPS_RMC_LONGITUDE],"")!=0)
						{
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_TIME]);
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_LONGITUDE]);
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_LATITUDE]);
							write_string(";");
							write_string(GPS_RMC[GPS_RMC_PATH]);
							write_string(";");
							write_string(GPS_GGA[GPS_GGA_ALTMSL]);
						}
					
						write_string_ln(";");
					}
					
					// Writing to OLED
 						if (task_OLED == TRUE)
							{
								send_cnt++;
								
								if (send_cnt==160)
								{
									OLED_send_char("C");
									
// 									OLED_send_byte(0x7F);
// 									OLED_send_byte(0x09);
// 									OLED_send_byte(0x09);
// 									OLED_send_byte(0x09);
// 									OLED_send_byte(0x01);
// 									OLED_send_byte(0x00);
									send_cnt = 0;
								}
								

								//OLED_set_position(0,1);
								//OLED_send_string("TEST2");
							}

					
					
				Ctrl_Mode_prev = Ctrl_Mode;		// Setting previous State
				
				ctrl_in_prev[9] = ctrl_in;
				//GPS_string_prev[GPS_LONGITUDE][] = GPS_string[GPS_LONGITUDE];
			}		
			
			
			// Toggling the Watchdog (Reset)
			wdt_reset();
    }
}