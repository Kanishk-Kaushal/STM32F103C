#include "stm32f10x.h"

int val;
 volatile uint16_t samples[2]={0,0};
volatile uint16_t msticks;

 long int MAP(long int IN, long int INmin, long int INmax, long int OUTmin, long int OUTmax)
 {
	 return ((((IN - INmin)*(OUTmax - OUTmin))/(INmax - INmin)) + OUTmin);
 }

void delayms(uint16_t ms)
{
	msticks=0;
	while(msticks<ms);
}

void SysTick_Handler(void)
{
	msticks++;
}


	void ADC1_2_IRQHandler(void)
	{
		if(ADC1->SR & ADC_SR_EOC)
		{
			val=ADC1->DR;
		}
	
	}


int main(){

	//change prescaler for adc (<14MHz)
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	
	//enable clocks for adc and alt functions
	RCC->APB2ENR |=RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |=(1<<4);
	RCC->APB1ENR |=RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN;
	//enable dma1
	RCC->AHBENR |=RCC_AHBENR_DMA1EN;
	//RCC->APB2ENR |=RCC_APB2ENR_TIM1EN;
	SysTick_Config(SystemCoreClock/1000);
	
	//configure pin as input push pull
	GPIOA->CRL |=GPIO_CRL_CNF5_1;
	GPIOA->CRL &=~(GPIO_CRL_CNF5_0);
	GPIOA->CRL |=GPIO_CRL_CNF7_1;
	GPIOA->CRL &=~(GPIO_CRL_CNF7_0);
	//OUTPUT
	GPIOB->CRH|= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
	GPIOB->CRH &= ~(GPIO_CRH_CNF9_0);
	//GPIOB->CRH|= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1;
	//GPIOB->CRH &= ~(GPIO_CRH_CNF8_0);
	GPIOB->CRL|= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF1_0);
	
	//enable end of conversion interrupt
	//ADC1->CR1 |=ADC_CR1_EOCIE;
	//enable interrupt in NVIC
	//NVIC_EnableIRQ(ADC1_2_IRQn);
	
	//set the sampling rate
	ADC1->SMPR2 |=ADC_SMPR2_SMP5_2 | ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_1;
	ADC1->SMPR2 |=ADC_SMPR2_SMP7_2 | ADC_SMPR2_SMP7_0 | ADC_SMPR2_SMP7_1;
	//set channel in sequence register
	ADC1->SQR3 |=ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2;
	ADC1->SQR1 |=1<<20; //FOR 2 SEQ
	ADC1->SQR3 |=7<<5;
	
	//adc dma enable and scan
	ADC1->CR1 |=ADC_CR1_SCAN;
	ADC1->CR2 |=ADC_CR2_DMA;
	
	//dma settings
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	DMA1_Channel1->CMAR = (uint32_t)samples;
	DMA1_Channel1->CNDTR = 2;
	DMA1_Channel1->CCR |= DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	
	
	//enable adc and continuous mode
	ADC1->CR2 |=ADC_CR2_ADON | ADC_CR2_CONT;
	delayms(1);
	
	//turn adc 2nd time
	ADC1->CR2 |=ADC_CR2_ADON;
	delayms(1);
	//RUN THE CALIBRATION
	ADC1->CR2 |=ADC_CR2_CAL;
	delayms(2);
	while(ADC1 ->CR2 & ADC_CR2_CAL);
	
	//TIMER
	
	TIM3->CR1 |=TIM_CR1_ARPE;
	TIM3->CCER |= TIM_CCER_CC4E;
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
	
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->CCER |= TIM_CCER_CC4E;
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
	
	TIM4->PSC=0;
	TIM4->ARR=1000;
	TIM4->CCR4 =0; //PB9
	
	TIM3->PSC=0;
	TIM3->ARR=1000;
	TIM3->CCR4 =0; //PB1
	
	//TIM4->CCER |= TIM_CCER_CC3E;
	//TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	//TIM4->CCR3=0; //PB8
	
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |=TIM_CR1_CEN;
	
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |=TIM_CR1_CEN;
	
	
	while(1)
	{
		
		
			//TIM4->CCR4 = (int)(MAP(samples[0],4080,0,1440,0));
				TIM4 -> CCR4 = (int)(MAP(samples[0],4080,0,1440,0));
				TIM3->CCR4 = (int)(MAP(samples[1],4080,0,1440,0));
			
		
	}
	
}
