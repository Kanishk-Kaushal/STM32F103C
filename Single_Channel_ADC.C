// DEVICE HEADER
#include "stm32f10x.h"

volatile static uint16_t myTicks = 0;
volatile static uint32_t val = 0;
volatile static uint32_t mapfdone = 0;

// FUNCTION PROTOTYPES
uint16_t map(uint32_t z,float input_l,float input_h,float output_l,float output_h);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void ADC1_2_IRQHandler(void);
void ADC_Init(void);
void Timer4_B9_Init(void);

int main(void)
{
	ADC_Init();
	
	Timer4_B9_Init();
		
	while(1)
	{
		TIM4 ->CCR4 = map(val, 0, 4095, 0, 1000);
		delayMs(5);
		
	}
		
}

// INITIALISE ADC
void ADC_Init(void)
{
	// SETTING CLOCK FOR ADC
	RCC ->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	RCC ->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

  // SET ADC PIN -> A5
	GPIOA ->CRL |= GPIO_CRL_CNF5_1;
	GPIOA ->CRL &= ~(GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);
	
	// ENABLE EOC INTERRUPT | EOCIE -> END OF CONVERSION INTERRUPT ENABLE
	ADC1 ->CR1 |= ADC_CR1_EOCIE;
	NVIC_EnableIRQ(ADC1_2_IRQn);
	
	// SET SAMPLE RATE
	ADC1 ->SMPR2 |= ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_2;
	
	// SET THE CHANNEL
	ADC1 ->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2;
	
	// ADC CONTINUOS MODE 
	ADC1 ->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;
	
	// SET DELAY
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	delayMs(1);
	
	// TURN ADC ON AGAIN | GIVEN IN REF MANUAL
	ADC1 ->CR2 |= ADC_CR2_ADON;
	delayMs(1);
	
	// CONFIGURE CALLIBRATION
	ADC1 ->CR2 |= ADC_CR2_CAL;
	while(ADC1 ->CR2 & ADC_CR2_CAL); 
}	

// SET UP TIMER 4, PORT B9 & PWM
void Timer4_B9_Init(void)
{
	// INITIALIZE TIMER 4 | GPIOB | ENABLE ALT FUNCTION
	RCC ->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	RCC ->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// GPIOC | OUTPUT | PUSH-PULL MODE | MAX SPEED = 50Hz
	GPIOB ->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOB ->CRH &= ~(GPIO_CRH_CNF9_0);
			
	TIM4 -> PSC = 8; // PRESCALAR 
	TIM4 -> ARR = 1000; // AUTO RELOAD REGISTER
	TIM4 -> CCR4 = 0; // CAPTURE/COMPARE REGISTER
	   
	TIM4 -> CCER |= TIM_CCER_CC4E; // OUTPUT POLARITY SET TO ACTIVE HIGH
	TIM4 -> CR1 |= TIM_CR1_ARPE; // CR -> CONTROL REGISTER | ARPE -> AUTO RELOAD PRELOAD REGISTER | ARPE -> SET => TIMER COUNTER (TIM_CNT) IS BUFFERED
	TIM4 -> CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // OUPUT COMPARE 4 MODE SET AS PWM MODE 1
	TIM4 -> CCMR2 |= TIM_CCMR2_OC4PE; // ENABLE THE CORRESPONDING PRELOAD REGISTER
	
	TIM4 -> EGR |= TIM_EGR_UG; // BEFORE STARTING TIMER -> INITIALIZE ALL REGISTERS
	TIM4 -> CR1 |= TIM_CR1_CEN; // START TIMER
}

// ADC INTERRUPT REQUEST HANDLER
void ADC1_2_IRQHandler(void)
{
	if(ADC1 ->SR & ADC_SR_EOC)
	{
		val = ADC1 ->DR;
	}
		 
}

// MAP FUNCTION AS IN ARDUINO
uint16_t map(uint32_t z,float input_l,float input_h,float output_l,float output_h)
{
	float mapf = (z - input_l)/(input_h-input_l)*(output_h-output_l)+output_l;
	mapfdone = (uint16_t)mapf;
		
	return(mapfdone);
}

// SYSTEM TIMER
void SysTick_Handler(void)
{
	myTicks++;
}

// 1 MILI SECOND DELAY
void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

