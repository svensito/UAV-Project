/*
 * ADC_Reading.h
 *
 * Created: 18.06.2013 15:43:46
 *  Author: SNFU
 */ 


#ifndef ADCREADING_H_
#define ADCREADING_H_

// initializing the ADC
void ADC_start(void);
// reading the voltage function
volatile int16_t ADC_read_speed(void);
volatile int16_t ADC_read_RSSI(void);
volatile int16_t ADC_read_ultrasound(void);
void ADC_speed_cal(void);

#endif /* ADCREADING_H_ */