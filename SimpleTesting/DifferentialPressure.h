/*
 * DifferentialPressure.h
 *
 * Created: 18.06.2013 15:43:46
 *  Author: SNFU
 */ 


#ifndef DIFFERENTIALPRESSURE_H_
#define DIFFERENTIALPRESSURE_H_

// initializing the ADC
void ADC_start(void);
// reading the voltage function
volatile int16_t ADC_read_voltage(void);
void ADC_calibration(void);

#endif /* DIFFERENTIALPRESSURE_H_ */