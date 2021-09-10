#include "stm32f10x.h" // DEVICE HEADER

// FUNCTION PROTOYPES
void Delay(int x); 
void Ports_Init(void);
void Timer_Init(void);

int main(){

  Ports_Init();
	
	Timer_Init();
	
	while(1)
	{
		
		int i;
		for(i = 0; i < 2000; i++)
		{
			TIM4 -> CCR4 = i;
			Delay(1);
			
		}
    		
		int j;
		for(j = 2000; j >= 0; j--)
		{
			TIM4 -> CCR4 = j;
			Delay(1);
		}
			
	}

}


void Delay(int rep) // RANDOM DELAY FUNCTION
{
	int l;
	for (l = 0; l < rep; l++ )
	{
		for(int k = 0; k<10000; k++)
		{
			__NOP(); // PAUSE CPU FOR 1 CYCLE
		}
	}
	
}

void Ports_Init()
{
	// INITIALIZE TIMER 4 | GPIOB | ENABLE ALT FUNCTION
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// GPIOC | OUTPUT | PUSH-PULL MODE | MAX SPEED = 50Hz
	GPIOB -> CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
	GPIOB -> CRH &= ~(GPIO_CRH_CNF9_0);
}

void Timer_Init()
{
	TIM4 -> PSC = 79; // PRESCALAR = 1MHz
	TIM4 -> ARR = 2000; // AUTO RELOAD REGISTER
	TIM4 -> CCR4 = 0; // CAPTURE/COMPARE REGISTER
	   
	TIM4 -> CCER |= TIM_CCER_CC4E; // OUTPUT POLARITY SET TO ACTIVE HIGH
	TIM4 -> CR1 |= TIM_CR1_ARPE; // CR -> CONTROL REGISTER | ARPE -> AUTO RELOAD PRELOAD REGISTER | ARPE -> SET => TIMER COUNTER (TIM_CNT) IS BUFFERED
	TIM4 -> CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // OUPUT COMPARE 4 MODE SET AS PWM MODE 1
	TIM4 -> CCMR2 |= TIM_CCMR2_OC4PE; // ENABLE THE CORRESPONDING PRELOAD REGISTER
	
	TIM4 -> EGR |= TIM_EGR_UG; // BEFORE STARTING TIMER -> INITIALIZE ALL REGISTERS
	TIM4 -> CR1 |= TIM_CR1_CEN; // START TIMER
}
