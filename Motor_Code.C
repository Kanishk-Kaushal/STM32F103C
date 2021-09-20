// DEVICE HEADER
#include"stm32f10x.h"

#include "math.h"
#include "stdlib.h"

volatile int myTicks = 0;
volatile uint16_t samples[2] ={ 0, 0 };

// FUNCTION PROTOYPES
void Ports_Clocks(void);
void Timers_Init(void);
void ADC_Init(void);
int map(float k, float l, float h, float L, float H);
int ADC_2ch_read(int k);
void Motor_Code(int x, int y);




int main()
{
	// CONFIGURE STM32
	Ports_Clocks();
	Timers_Init();
	ADC_Init();

	int x_joy, y_joy;
	
	while (1)
	{
		// MAPPED X AXIS VALUE
		x_joy = ADC_2ch_read(0);   
		
		// MAPPED Y AXIS VALUE
		y_joy = ADC_2ch_read(1);   

		Motor_Code(x_joy, y_joy);
	}
}

void Ports_Clocks()
{
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // ENABLE CLOCK | PORT A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // ENABLE CLOCK | PORT B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // ENABLE ALTERNATE FUNCTION

	// PORT A | PIN 1 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);   
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1);   

	// PORT A | PIN 2 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);   
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);   

	// PORT A | PIN 4 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE4;   
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   

	// PORT A | PIN 5 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE5;   
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   

	
	// PORT B | PIN 6 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE6;   
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	// PORT B | PIN 7 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE7;  
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

}

void Timers_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   // TIMER 4 ENABLE
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // CHANNEL 1 & CHANNEL 2 -> OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   // ENABLE ARPE | ARPE -> AUTO PRE-LOAD ENABLE

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   // CHANNEL 1 PRELOAD ENABLE
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   // CHANNEL 2 PRELOAD ENABLE
	
	// OUPUT COMPARE 1 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	
	// OUPUT COMPARE 2 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1; // PRESCALAR
	TIM4->ARR = 4095;  // AUTO-RELOAD VALUE -> (2^16 - 1)  
	TIM4->CCR1 = 0; // CAPTURE/COMPARE REGISTERS
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   // BEFORE STARTING TIMER -> INITIALIZE ALL REGISTERS
	TIM4->CR1 |= TIM_CR1_CEN;   // COUNTER ENABLE
}

void ADC_Init()
{
	GPIOA->BSRR |= 1 << 4 | 1 << 5;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   //  PRESCLAR -> 72MHz/6
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1 CLOCK ENABLE
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;   // DMA1 CLOCK ENABLE

	// SAMPLING RATES FOR CHANNEL 1 & CHANNEL 2
	ADC1->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0;   
	ADC1->SMPR2 |= ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0; 
  
	// SET CHANNEL 1 & 2 | SET SEQUENCE
	ADC1->SQR1 |= 1 << 20;   
	ADC1->SQR3 |= ADC_SQR3_SQ1_0;
	ADC1->SQR3 |= ADC_SQR3_SQ2_1;
	
	ADC1->CR1 |= ADC_CR1_SCAN;   // SCAN MODE ENABLE
	ADC1->CR2 |= ADC_CR2_DMA;   // DMA MODE ENABLE

	
	// DMA CONFIGURATIONS
	
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));   
	DMA1_Channel1->CMAR = (uint32_t) samples;   
	DMA1_Channel1->CNDTR = 2;   

	DMA1_Channel1->CCR |= DMA_CCR1_CIRC;   
	DMA1_Channel1->CCR |= DMA_CCR1_MINC;   
	DMA1_Channel1->CCR |= DMA_CCR1_PSIZE_0;   
	DMA1_Channel1->CCR |= DMA_CCR1_MSIZE_0;   

	DMA1_Channel1->CCR |= DMA_CCR1_EN;   // DMA1 ENABLE
	
	ADC1->CR2 |= ADC_CR2_ADON;   // TURN ADC ON
	ADC1->CR2 |= ADC_CR2_CONT;   // ADC CONTINUOS MODE
		
	ADC1->CR2 |= ADC_CR2_ADON; // TURN ADC ON AGAIN | GIVEN IN REF MANUAL
	ADC1->CR2 |= ADC_CR2_CAL; // RUN CALIBRATION
	
}

int map(float k, float l, float h, float L, float H)
{
	return ((k - l) / (h - l)) * (H - L) + L;
}

int ADC_2ch_read(int k)
{
	int Joyval = 0;
	
	Joyval = samples[k];
	
	if(k == 0)
	{
		Joyval = map(Joyval, 0, 4095, -4095, 4095);
	}
		

	else
	{
		Joyval = map(Joyval, 0, 4095, 4095, -4095);
		
		//  BUFFERS FOR JOYSTICK EDGES		
		if (Joyval < -3950)
		{
			Joyval = -4095;
		}
		
		if (Joyval > 3950)
		{
			Joyval = 4095;
		}
			
	}
	
	// BUFFERS FOR JOYSTICVK CENTRES
	if (abs(Joyval) < 200)
		Joyval = 0;
	
	return Joyval;
}

void Motor_Code(int x, int y)
{


	// STOP
	if (abs(x) < 20 && abs(y) < 20)
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = 0;  //Left SPEED
	 TIM4->CCR2 = 0;   //Right SPEED
	}		
	
  
	// FORWARD MAX
	else if(abs(x) < 10 && y > 0)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y) ;  //Left SPEED
	  TIM4->CCR2 = (uint32_t) abs(y);   //Right SPEED
		
	}	
	
	// BACKWARD MAX
	else if(abs(x) < 10 && y < 0) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(y));  //Left SPEED
	  TIM4->CCR2 = (uint32_t)  abs(y);   //Right SPEED
	}	
	
	// SPOT LEFT
	else if (x < 0 && abs(y) <= 10) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x);  //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(x);   //Right SPEED
	}		
	
	// SPOT RIGHT
	else if (x > 0 && abs(y) <= 10)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x);		//Left SPEED
	  TIM4->CCR2 = (uint32_t) abs(x);   //Right SPEED		
	}		
		
	// OCTET 1
	else if(x > 0 && y > 0 && x > y) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
	 TIM4->CCR1 = (uint32_t) abs(x);   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y));   //Right SPEED
	}		

	// OCTET 2
	else if(x > 0 && y > 0 && x < y) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y);   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y));   //Right SPEED
	}		

	// OCTET 3
	else if(x < 0 && y > 0 && abs(x) < y){
	GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y));   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(y);   //Right SPEED  
	}

  // OCTET 4
	else if(x < 0 && y > 0 && abs(x) >= y) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y));   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(x);   //Right SPEED
	}	

  // OCTET 5	
	else if(x < 0 && y < 0 && abs(x) > abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x);   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y));   //Right SPEED
	}
	
	// OCTET 6
	else if(x < 0 && y < 0 && abs(x) < abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y);   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y));   //Right SPEED
	}
	
	// OCTET 7
	else if(x > 0 && y < 0 && abs(x) < abs(y)) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y));   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(y);   //Right SPEED
	}

	// OCTET 8
	else if(x > 0 && y < 0 && abs(x) > abs(y)) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y));   //Left SPEED
	 TIM4->CCR2 = (uint32_t) abs(x);   //Right SPEED
	}
}	
