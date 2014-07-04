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
//------------------------------------------------------

//------------------------------------------------------
// Local declarations and variables

// These are the task flags, set them to activate / deactivate task
int task_gyro	= TRUE;
int task_acc	= FALSE;
int task_mag	= FALSE;
int task_temp	= FALSE;
int task_baro	= TRUE;
int task_speed	= TRUE;

// Control Mode
uint8_t Ctrl_Mode = DIRECT_LAW;	// First Test with Direct Law Only: DIRECT_LAW or NORMAL_LAW possible

// Control Knob
uint8_t Knob_State = 0;
uint8_t prev_Knob_State = 0;
uint16_t Knob_Stepwidth = 400;
//long test	= 0;
//------------------------------------------------------

//------------------------------------------------------
// Task Timer definitions


volatile int task_flag = 0;		// this flag will be set every 20ms and reset in the application


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
	OCR0 = 0x75;
	write_string_ln("TaskTimerStarted");
}	

ISR(TIMER0_COMP_vect)
{
	// Setting the task flag
	task_flag = 1;
}

//------------------------------------------------------

int main(void)
{	
	
	// Initializing
	init_uart();
	
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
	int8_t Phi = 0;
	int8_t Theta =0;
	int8_t Psi = 0;
	int16_t altitude = 0;
	int16_t speed = 0;
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
				bla_cnt++;
				
				// Defining struct to read the gyro channels
				struct three_elements_obj turn_rate;
				if(task_gyro == TRUE)	
				{
					// turn rates and euler angles
					turn_rate = gyro_read();
					Phi		+= turn_rate.p/10;
					Theta	+= turn_rate.q/10;
					Psi		+= turn_rate.r/10;
				}
				
				if(task_acc == TRUE)	acc_read();
				if(task_baro == TRUE)// && (bla_cnt==100))	
				{
					altitude = baro_read();
					//bla_cnt = 0;
				}
						
				if(task_mag == TRUE)	mag_read();
				if(task_speed == TRUE)
				{
					speed = ADC_read_voltage();
				}
				
				switch(Ctrl_Mode)
				{
					case DIRECT_LAW:
					// Direct Law is used for direct Remote Control input -> Servos
					// + / - in front of the long bracket inverses the Control
						 POSITION1 = NEUTRAL+((ctrl_in[0]-SERVO_TRIM_MOTOR)*SERVO_GAIN_MOTOR);			// Motor (not for Easyglider)
						 POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON);		// Aileron
						 POSITION3 = NEUTRAL+((ctrl_in[2]-SERVO_TRIM_ELEVATOR)*SERVO_GAIN_ELEVATOR);	// Elevator
						 POSITION4 = NEUTRAL-((ctrl_in[3]-SERVO_TRIM_RUDDER)*SERVO_GAIN_RUDDER);		// Rudder
						 if(ctrl_in[5]>144) POSITION5 = FLAP_FULL;
						 else if (ctrl_in[5] > 139 && ctrl_in[5] < 144) POSITION5 = FLAP_1;
						 else POSITION5 = FLAP_UP;
						 
						 //POSITION5 = -162*ctrl_in[5]+24662;
						 
						 // to determine the position of the knob
						 /*
						 if(ctrl_in[4]<135 && ctrl_in[4]>120) Knob_State = 1;
						 else if(ctrl_in[4]<150 && ctrl_in[4]>135) Knob_State = 2;
						 else if(ctrl_in[4]<160 && ctrl_in[4]>150) Knob_State = 3;
						 else Knob_State = 0;
						 
						
						 switch(Knob_State)
						 {
							 case 1:
								if(prev_Knob_State == 2) 
								{
									
								}
								else if (prev_Knob_State == 3)
								{

								}
								else ;
								prev_Knob_State = 1;
							 break;
							 case 2:
								if(prev_Knob_State == 3)
								{

								}
								else if (prev_Knob_State == 1)
								{

								}
								else ;
								prev_Knob_State = 2;
							 break;
							 case 3:
							 	if(prev_Knob_State == 1)
							 	{

							 	}
							 	else if (prev_Knob_State == 2)
							 	{

							 	}
							 	else ;
								prev_Knob_State = 3;
							 break;
						 }
						 */
						 //POSITION5 = NEUTRAL-;
						 // Sending Start Short (16 bit):
						 //send_ushort(10101);//send_ubyte(128);send_ubyte(255);
						 // Sending Payload:
// 						 send_ushort(POSITION1);send_ushort(POSITION2);
// 						 send_ushort(POSITION3);send_ushort(POSITION4);
// 						 send_sbyte(Phi);send_sbyte(Theta);send_sbyte(Psi);
// 						 send_sshort(altitude);
// 						 send_sshort(speed); write_string_ln(";");
						if(send_cnt == 5)
						{
 						 write_var(POSITION1);write_string(";");write_var(POSITION2);write_string(";");
 						 write_var(POSITION3);write_string(";");write_var(POSITION4);write_string(";");
 						 write_var(POSITION5);write_string(";");
						 write_var(turn_rate.p);write_string(";");write_var(Phi);write_string(";");
						 write_var(turn_rate.q);write_string(";");write_var(Theta);write_string(";");
						 write_var(turn_rate.r);write_string(";");write_var(Psi);write_string(";");
						 write_var(altitude);write_string(";");write_var_ln(speed);
						 send_cnt = 0;
						}
						send_cnt++;					
					break;
					
					case NORMAL_LAW:
					// Normal Law will be used for semi automatic flight (supportive Damping etc.)
						 POSITION2 = NEUTRAL+((ctrl_in[1]-SERVO_TRIM_AILERON)*SERVO_GAIN_AILERON) - (10*turn_rate.p);
									
					break;
					
				}
							
			}		
		
    }
}