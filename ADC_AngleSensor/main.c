#include "stm32f4xx.h"                  // Device header
#include <math.h>

// using PA1, ADC1 to read Angle sensor value (Pot on the pendulum)
// setting 10 bit resolution may change depending on perfomance
// continous coversion


int value;


int main(void){
	
	RCC->AHB1ENR |= 0x9; // clk to PA and PD
	GPIOA->MODER |= 0xC; // set PA1 to analog mode
	GPIOA->MODER |= 0xC000;
	
	GPIOD->MODER |= 0x555;
	RCC->APB2ENR |= 0x100; // clk to ADC1
	
	ADC1->CR2 = 0;
	ADC1->CR1 |= 1<<24; // setting 10bit resolution
	ADC1->CR2 |= 0; // right align data writing 0 to 11th bit
	ADC1->CR2 |= 2; // sett contionous conversion mode
	//ADC1->SQR2 |= 1;
	ADC1->SQR3 |= 1; // select channel 1 and as the 1st conversion
	ADC1->CR2 |= ADC_CR2_ADON; // switch adc on
	

	while(1){
		
		ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
		while(!((ADC1->SR & 0x2) == 0)){
			value = ADC1->DR;
			
			
		}
		
	}
}
