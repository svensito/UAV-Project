/*
 * ServoControl.h
 *
 * Created: 12.04.2013 11:39:15
 *  Author: snfu
 */ 


#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_


// defines
#define NEUTRAL			2250	//
#define RIGHT			3000	// Min Limit
#define LEFT			1000
#define PULSE_WIDTH		6000	// This value value will give 4ms of pulswidth

// FLAP SERVO LIMITS
#define FLAP_FULL		2300	// Extension Flap Servo Limit
#define FLAP_1			1900
#define FLAP_UP			1510	// Slightly different Neutral Position	

#define TRIM_NEUTRAL	140		// is the counter value of TCNT2 for neutral position of control

#define SERVO_GAIN_MOTOR	14	// Normal GAIN would be 31 (full resolution for Servos)
#define SERVO_GAIN_RUDDER	14		
#define SERVO_GAIN_ELEVATOR	15
#define SERVO_GAIN_AILERON	15

#define SERVO_TRIM_MOTOR	TRIM_NEUTRAL 
#define SERVO_TRIM_RUDDER	TRIM_NEUTRAL	// 
#define SERVO_TRIM_ELEVATOR	TRIM_NEUTRAL -10	// 
#define SERVO_TRIM_AILERON	TRIM_NEUTRAL 

#define TRUE			1
#define FALSE			0


// Functions
void servo_init();
void servo_start_signal();

// Variables
volatile long POSITION1;
volatile long POSITION2;
volatile long POSITION3;
volatile long POSITION4;
volatile long POSITION5;

volatile int ctrl_in[9];
volatile int ctrl_in_offset[9];
volatile long ctrl_out[5];

typedef enum
	{
	motor = 0,
	aileron,
	elevator,
	rudder,
	flap
	} controls_out;
	
typedef enum
{
	stick_l_up_down = 0,
	stick_r_left_right,
	stick_r_up_down,
	stick_l_left_right,
	rotary_knob,
	three_way_switch,
	spare	
	} controls_in;

#endif /* SERVOCONTROL_H_ */