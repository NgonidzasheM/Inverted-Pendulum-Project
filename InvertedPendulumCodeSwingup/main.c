#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include <math.h>

//#define ENCODER_TIM_PERIOD 65535

//Initialization prototypes********************************************/
void InitPWM(void);                                                  //*
void InitAngleSensorADC(void);                                       //*
void Init_Encoder(void);                                             //*
void InitDAC(void);                                                  //*
void InitControlTim4(void);                                          //*
/**********************************************************************/
//calculation functions                                              //*
double xposition(int encoderV);                                      //*
double angleFn(int adcValue);                                        //*
/**********************************************************************/

//varaible initializations********************************************//
int ADCvalue;                                                        //*
uint32_t ARR = 0xFFFFFFFF;                                           //*
int EncValue = 0;                                                    //*
double x;                                                            //*
double theta;                                                        //*
//float myDacValf = 2.8;                                             //*
uint16_t myDacx;                                                     //*
int pwmValue = 0;                                                   //*
/**********************************************************************/		



// initial conditions 
float xRef = 0;
float thetaRef = 0;
float xEprevious = 0;
float xError;
float uPrevious = 0;
float uCurrent = 0;

int count = 0;

int swingup = 1; // to be removed



int main(void){
	
	// Initializations
	InitPWM();
	InitAngleSensorADC();
	Init_Encoder();
	InitDAC();
	InitControlTim4();

// Press Button to Start	
		while((GPIOA->IDR & 0x1) == 0){
		}	
		
		
	while(1){		
		EncValue = TIM2->CNT;                             // the encoder value
		// Getting the ADC value
		ADC1->CR2 |= ADC_CR2_SWSTART;                     // start conversion
		while(!((ADC1->SR & 0x2) == 0)){
			ADCvalue = ADC1->DR;                            // the adc value
						
		}
		
// Setting swing up to 0 if the angle reaches a point the pndulum can balance		
		theta = angleFn(ADCvalue);                        // the pendulum angle*/
		if((theta<0.1) && (theta > -0.1)){
			swingup = 0;
		}
		else{
			swingup = 1;
		}

		x = xposition(EncValue);
		//xRef = 0.45/2;
		xError = xRef - x;
		
		// Position controller
		uCurrent = uCurrent + 21.8*xError - 21.793*xEprevious;
		
		if(uCurrent > 2.7){
		
			uCurrent = 2.7;
		}
		if(uCurrent < -2.7){
			uCurrent = -2.7;
		}
		
		pwmValue = (int)(100*uCurrent/3);
		if(uCurrent > 0){
			GPIOD->BSRR |= GPIO_BSRR_BR3;
			GPIOD->BSRR |= GPIO_BSRR_BS2;
			TIM3->CCR2 = pwmValue;
		}
		else if(uCurrent < 0){
			GPIOD->BSRR |= GPIO_BSRR_BS3;
			GPIOD->BSRR |= GPIO_BSRR_BR2;
			TIM3->CCR2 = pwmValue*-1;
		}
		else{
			GPIOD->BSRR |= GPIO_BSRR_BR3;
			GPIOD->BSRR |= GPIO_BSRR_BR2;
			TIM3->CCR2 = 0;
		}
		
		uPrevious = uCurrent;
		xEprevious = xError;

		//uCurrent = 0;
		
		
		
		
		myDacx = (uint16_t)((x*20/9)*4095);               // getting the value x in voltage (Vvalue/4095) as unsigned int16
		DAC->CR |= 1<<0;                                  // start the dac 
		DAC->CR |= 1<<16;                                 // start the dac for channel 2		
		DAC->DHR12R2 = myDacx;                            // value of voltage on PA5 to record the position*/
		
	}


}

// using timer 4 to implement control

void TIM4_IRQHandler(void){
	TIM4->SR &= ~(0x1);                                 // clear interrupt


	
// Implemetation of the swing up controller	
	if(swingup == 1){
		if(count < 120){     // when 120 it means it stays at position until 120*5ms and then changes position	
			xRef = -0.06; 
			count = count+1;
		}
		else{
			xRef = 0.08;       // Positive setpoint of the cart position
			count = count + 1;
			if(count>240){
				count = 0;
			}
		}
	}	

	
}


void InitControlTim4(void){
	RCC->APB1ENR |= 1<<2;
	TIM4->PSC = 16000-1;                                // 1kHz
	TIM4->ARR = 5-1;                                    // set control frequency (now 5ms)
	TIM4->DIER |= 0x1;                                  // enable update interrupt
	
	NVIC_EnableIRQ(TIM4_IRQn);
	TIM4->CR1 |= 0x1;                                   // enable timer/ start counter
}

void InitPWM(void){

	RCC->AHB1ENR |= 0x4;                // clk to port c
	RCC->AHB1ENR |= 0x9;                // clk to port a and d
	
	
	
	GPIOD->MODER |= 0x55;               // to control enable pins 1, 2, 3, 4 on the motor driver
	GPIOD->MODER |= 0x55000000; 
	GPIOA->MODER &= ~(0x3);             // select PA0 to be input (switch) 00 in the moder/ masking out
	
	
	GPIOC->MODER |= 0xA000;             // pin 6, 7, 8 and 9 to alternate function
	GPIOC->AFR[0] |= 0x22000000;        // map p6 and 7 to AF2

	
	RCC->APB1ENR |= 0x2;                // clk to timer 3
	TIM3->PSC = 16-1;                   // set prescaler (CNT counts at 0.01ms) 100kHz
	TIM3->ARR = 100-1;                  // to set frequency of overflow to 10000Hz / 10kHz ideal for motor control
	
	TIM3->CCMR1 &= ~(0x303);            // channel 1 and 2 set to output mode
	
	TIM3->CCMR1 |= 0x6060;              // set channel 1 and 2 to PWM mode 1
	
	TIM3->CCMR1 &= ~(0x1010);           // clear 4 and 12th bit to set mode 1 PWM
	
	TIM3->CR1 &= ~(0x60);               // enable edge align
	TIM3->CCER &= ~(0xAA);              /* mask out capture compare CCxP and CCxNP output polarity (x = channel #) 
	                                    |setting channel as active*/
													 
	TIM3->CCER |= 0x11;                 // enable capture compare output CCxE
	TIM3->CR1 |= 0x1;                   // start timer or counter
	
}


void InitAngleSensorADC(void){
	
	RCC->AHB1ENR |= 0x9;                 // clk to PA and PD
	GPIOA->MODER |= 0xC;                 // set PA1 to analog mode
	GPIOA->MODER |= 0xC000;
	
	GPIOD->MODER |= 0x555;
	RCC->APB2ENR |= 0x100;               // clk to ADC1
	
	ADC1->CR2 = 0;
	//ADC1->CR1 |= 1<<24;                  // setting 10bit resolution
	ADC1->CR2 |= 0;                      // right align data writing 0 to 11th bit
	ADC1->CR2 |= 2;                      // sett contionous conversion mode
	//ADC1->SQR2 |= 1;
	ADC1->SQR3 |= 1; // select channel 1 and as the 1st conversion
	ADC1->CR2 |= ADC_CR2_ADON;           // switch adc on
	
}


void Init_Encoder(void){
	

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // CLK for GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // CLK to tim2
	
	// Encoder Tim2 Ch1 PA15, Ch2 PB3
	GPIOA->MODER |= GPIO_MODER_MODE15_1; // Altenate function PA15
	GPIOB->MODER |= GPIO_MODER_MODE3_1; // Alternate function PB3
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15; // 100Mhz high speed
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3; // same for PB3
	
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD15_0; // setting the pins to pull up
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_0;
	
	GPIOA->AFR[1] |= 0x10000000;
	GPIOB->AFR[0] |= 0x1000;
	
	TIM2->ARR = ARR;
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0;
	 
	TIM2->SMCR |= TIM_SMCR_SMS_0|TIM_SMCR_SMS_1;
	TIM2->SMCR |= 0x300;  // filter (which ever works)
	TIM2->CCMR1 |= 0x3030; // input filter
	TIM2->CR1 |= TIM_CR1_CEN;

}

void InitDAC(void){

	RCC->AHB1ENR |= 1<<0; // clock to port A
	GPIOA->MODER |= 0xF00; // set PA4 and 5 to analog mode
	
	RCC->APB1ENR |= 1<<29; // enable clock for DAC
	DAC->CR &= ~(0x1FFF<<16); // unmask all unused function for channel 2
	DAC->CR &= ~(0x1FFF<<1); // unamask all unused function for channel 1

}


double xposition(int encoderV){
	double m = 0.45/4256;
	double position = 0;
	
	
	
	if(position < 0){	
		position = 0;  // when the encoder gives out a negative value change it to distance 0
	}
	else{
		position = m*encoderV;	
	}
	return position;
}

double angleFn(int adcValue){
	
	double m1 = (2*3.1429)/4095;

	double angle = 0;
	
	angle = m1*adcValue - 3.1429;
	
	return angle;
}

int getADC(void){
	int myValue;
	ADC1->CR2 |= ADC_CR2_SWSTART;                     // start conversion
	while(!((ADC1->SR & 0x2) == 0)){
	ADCvalue = ADC1->DR; 
}
	return myValue;
}
	
