

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define KHZ 1000

#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include "adc_analog.h"
#include "../utils/bit_tools.h"
#include "../usart/usart.h"

extern char bufferDummy[20];
/************************************************************************/
/* @obtain prescaler at ADc frequancy specified                                                                     */
/************************************************************************/
uint8_t ADC_calcPreScaler(uint8_t adcClk){ // ne precisa depois alterase
	
	// CHECK THIS// If used adcClk*KHZ we can avoid pass 16 bit vars.
	uint8_t presc =F_CPU/((uint32_t)(adcClk)*KHZ); //must cast adcClk in order to divide
	
		if (presc<128){
			if (presc<2) return ADC_PRESC_DIV2;
			if (presc<4) return ADC_PRESC_DIV4;
			if (presc<8) return ADC_PRESC_DIV8;
			if (presc<16) return ADC_PRESC_DIV16;
			if (presc<32) return ADC_PRESC_DIV32;
			if (presc<64) return ADC_PRESC_DIV64;
			if (presc<128) return ADC_PRESC_DIV128;	
	}else return ADC_PRESC_DIV128;//return the max prescaler value just to try, not guaranteed to work because it's over max allowed

}



/************************************************************************/
/* @init ADC                                                                    */
/************************************************************************/
void ADC_init(uint8_t mode, uint8_t voltageRef,uint8_t adcClk){
	
	ADCSRA |=mode;
	ADCSRA |= ADC_calcPreScaler(adcClk);
	ADMUX = voltageRef;
	
}


/*
 * convert an adc value to a resistance value
 */
long ADC_getResistence(uint16_t adcread, uint16_t adcbalanceresistor)
{
	if(adcread == 0)
		return -1;
	else
		return (long)((long)(ADC_RESOLUTION*(long)adcbalanceresistor)/adcread-(long)adcbalanceresistor);
}


/*
 * exponential moving avarage filter
 *
 * "newvalue" new adc read value
 * "value" old adc filtered value
 * return a new filtered value
 *
 * References:
 *   Guillem Planissi: Measurement and filtering of temperatures with NTC
 */
#define ADC_EMAFILTERALPHA 30
unsigned int ADC_emaFilter(unsigned int newvalue, unsigned int val)
{
	//use exponential moving avarate Y=(1-alpha)*Y + alpha*Ynew, alpha between 1 and 0
	//in uM we use int math, so Y=(63-63alpha)*Y + 63alpha*Ynew  and  Y=Y/63 (Y=Y>>6)
	val = (64-ADC_EMAFILTERALPHA)*val+ADC_EMAFILTERALPHA*newvalue;
	val = (val>>6);
	return val;
}



/************************************************************************/
/* @ CHECK how to do best                                                                     */
/************************************************************************/
uint16_t ADC_readFromINT(void){
	
	uint16_t val;
	
	val = ADCL;
	val |= (ADCH <<8);
	// clear flags for next run
	
	return val;
}



/************************************************************************/
/* @fire ADC                                                                     */
/************************************************************************/
void ADC_startConversion(uint8_t ch){
	
	ch&= 0b00000111;
	ADMUX &= 0xF8 |ch;
	
	//-- Start conv
	ADCSRA |= ADC_START_CONV;
	
	// wait until complete
	while(ADCSRA & ADC_START_CONV);
	
}



/************************************************************************/
/* @adc check if conversion finished and read                                                                     */
/************************************************************************/
uint8_t ADC_checkStatusAndRead(uint16_t *val){
	
	// wait until complete
	if(ADCSRA & ADC_START_CONV) return 0;
	
	// grab values é ler pra uma var adcl e adch
	*val = ADCL;
	*val |= (ADCH <<8);
	return 1;
	
}


/************************************************************************/
/* @read adc                                                                     */
/************************************************************************/
uint16_t ADC_readAndWAIT(uint8_t ch){
uint16_t val;
	
	
	ch &= 0b00000111;  // AND operation with 7 (7 channels can be used in single ended input) more on differential input
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits and select the channel(ch)
	
	
	//-- Start conv
	ADCSRA |= ADC_START_CONV;
	
	// wait until complete
	while(ADCSRA & ADC_START_CONV);
	
	// grab values é ler pra uma var adcl e adch
	//ADCL Must be read always first
	val = ADCL;
	val |= (ADCH <<8);
	return val;
	 
}