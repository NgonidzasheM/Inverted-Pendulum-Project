#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include <math.h>


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
int pwmValue = 0;                                                    //*
/**********************************************************************/		

int getADC(void);

// initial conditions 
float xRef = 0;
float thetaRef = 0;
float xEprevious = 0;
float xError;
float uPrevious = 0;
float uCurrent = 0;
double v;
double xp;
double thetap;
double angv;

int avgADC;

double u;

int count = 0;


// filter initialisations
// estimated values 
double x_est;
double v_est;
double theta_est;
double omega_est;
// previous estimates
double xp_est = 0;
double vp_est = 0;
double thetap_est = 0;
double omegap_est = 0;

// A matrix
double a11 = 1;      double a12 = 0.0046;   double a13 = 0;      double a14 = 0;
double a21 = 0;      double a22 = 0.8479;   double a23 = 0;      double a24 = 0;
double a31 = 0;      double a32 = 0.0016;   double a33 = 1.003;  double a34 = 0.005;
double a41 = 0;      double a42 = 0.6276;   double a43 = 1.125;  double a44 = 1.001;

// B Matrix
double b1 = 0.0002;
double b2 = 0.07375;
double b3 = -0.00078;
double b4 = -0.3043;

// Kalman Filter Gains
double k11 = 0.9999;     double k12 = 0;           double k13 = 0;      double k14 = 0;
double k21 = 0;          double k22 = 0.9532;      double k23 = 0;      double k24 = 0.0012;
double k31 = 0;          double k32 = 0.0000;      double k33 = 0.9998; double k34 = 0.005;
double k41 = 0;          double k42 = 0.0002;      double k43 = 0;      double k44 = 0.9904;

int main(void){
	
	// Initializations
	InitPWM();
	InitAngleSensorADC();
	Init_Encoder();
	InitDAC();
	InitControlTim4();

	int adc = 0;
	int adcsum = 0;
	
	float k1 = -8.4254;
	float k2 = -5.6779;
	float k3 = -33.9159;
	float k4 = -2.2590;
	
	EncValue = TIM2->CNT;                             // the encoder value
	xp = xposition(EncValue);
		
	
	ADCvalue = getADC();
	//avgADC = ADCvalue;
	thetap = angleFn(ADCvalue);
	
	// Press Button to Start	
	while((GPIOA->IDR & 0x1) == 0){
	}	
		
		
	while(1){		
		EncValue = TIM2->CNT;                             // the encoder value
		// Getting the ADC value
		ADCvalue = getADC();

		// Moving average FIR Filter
		for(uint32_t i = 0; i<1000; i++){
			adc = getADC();
			adcsum = adcsum + adc;
			avgADC = adcsum/(i+1);
		}
		adcsum = 0;
		
		// Controller u			
		u = 1*(k1*(0) + k2*0 + k3*(theta) + k4*0);
		
// implementing Saturation		
		if(u > 2.8){
			u = 2.8;
		}
		else if (u < -2.8){
			u = -2.8;
		}
		// setting PWM control value
		pwmValue = (int)(fabs(u)*100/3);
		
		if(u > 0){
			GPIOD->BSRR |= GPIO_BSRR_BR3;
			GPIOD->BSRR |= GPIO_BSRR_BS2;
			TIM3->CCR2 = 5 + pwmValue;        // add 5 to implement dead band
		}
		else if(u < 0){
			GPIOD->BSRR |= GPIO_BSRR_BS3;
			GPIOD->BSRR |= GPIO_BSRR_BR2;
			TIM3->CCR2 = 5 + pwmValue;        // will add 5 to compensated dead band
		}
		else{
			GPIOD->BSRR |= GPIO_BSRR_BR3;
			GPIOD->BSRR |= GPIO_BSRR_BR2;
			TIM3->CCR2 = 0;
		}
		
		// Implement Filter here
		x_est     = (a11-k11)*xp_est + (a12-k12)*vp_est + (a13-k13)*thetap_est + (a14-k14)*omegap_est + k11*x + k12*v + k13*theta + k14*angv + b1*u;
		v_est     = (a21-k21)*xp_est + (a22-k22)*vp_est + (a23-k23)*thetap_est + (a24-k24)*omegap_est + k21*x + k22*v + k23*theta + k24*angv + b2*u;
		theta_est = (a31-k31)*xp_est + (a32-k32)*vp_est + (a33-k33)*thetap_est + (a34-k34)*omegap_est + k31*x + k32*v + k33*theta + k34*angv + b3*u;
		omega_est = (a41-k41)*xp_est + (a42-k42)*vp_est + (a43-k43)*thetap_est + (a44-k44)*omegap_est + k41*x + k42*v + k43*theta + k44*angv + b4*u;
		
		xp_est = x_est;
		vp_est = v_est;
		thetap_est = theta_est;
		omegap_est = omega_est;
				
	}
}



// using timer 4 to implement control
void TIM4_IRQHandler(void){
	TIM4->SR &= ~(0x1);                                 // clear interrupt
	
	// Calculate cart velocity
	x = xposition(EncValue);		
	v = (x-xp)/0.005;		
	xp = x;
	
	// Calculate angular velocity of the pendulum
	theta = angleFn(avgADC);
	angv = (theta-thetap)/0.005;
	thetap = theta;	
	
	
}


void InitControlTim4(void){
	RCC->APB1ENR |= 1<<2;
	TIM4->PSC = 16000-1;                                // 1kHz
	TIM4->ARR = 1-1;                                    // set control frequency (now 5ms)
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
	
		position = m*encoderV;	
	
	return position;
}

double angleFn(int adcValue){
	
	double m1 = (2*3.1415927)/4095;

	double angle = 0;
	
	angle = m1*adcValue - 3.1415927;
	
	return angle;
}

int getADC(void){
	int myValue;
	ADC1->CR2 |= ADC_CR2_SWSTART;                     // start conversion
	while(!((ADC1->SR & 0x2) == 0)){
	myValue = ADC1->DR; 
}
	return myValue;
}
	
