// DEVICE HEADER FILE
#include "stm32f10x.h"

volatile static uint16_t myTicks=0;

// FUNCTION PROTOTYPES
void Ports_Init(void);
void USART_Rx_Init(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void Blink(int n);
unsigned char USART_Receive(void);
void Timers_Init(void);

int main(void)
{
	Ports_Init();
	Timers_Init();
	USART_Rx_Init();
		
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	while(1)
	{
		unsigned char rv = USART_Receive();
		
		if(rv == 'f')
	{
		
			GPIOA -> BSRR |= 1 << 4;   //Turn on LEFT LED
			GPIOA -> BSRR |= 1 << 5;   //Turn on RIGHT LED
			TIM4 -> CCR2 = 1024;       //Right PWM
			TIM4 -> CCR1 = 1024;	     // Left PWM      
			Blink(3);
		
		
	}
	
	else if(rv == 's')
	{
		GPIOA -> BRR |= 1 << 4;   //Turn off LEFT LED
		GPIOA -> BRR |= 1 << 5;   //Turn off RIGHT LED
		TIM4 -> CCR2 = 0;       //Right PWM
		TIM4 -> CCR1 = 0;	     //Left PWM
		Blink(3);
	}
	
	else if(rv == 'r')
	{
		GPIOA -> BRR |= 1 << 4;   //Turn off LEFT LED
		GPIOA -> BRR |= 1 << 5;   //Turn off RIGHT LED
		TIM4 -> CCR2 = 1024;       //Right PWM
		TIM4 -> CCR1 = 1024;	     //Left PWM
		Blink(3);
	}
		
			
	}
	
}


// FUNCTION DEFINITIONS

void Ports_Init(void)
{
	//Enable Clocks:
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
  RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function

	//Setup PA4:
	GPIOA -> CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA -> CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA -> CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA -> CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB -> CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB -> CRL |= GPIO_CRL_CNF6_1;
	GPIOB -> CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB -> CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB -> CRL |= GPIO_CRL_CNF7_1;
	GPIOB -> CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA -> CRH |= GPIO_CRH_CNF10;   //Input with pull-up/pull-down (10)
	GPIOA -> CRH &= ~(GPIO_CRH_CNF10_0);

	//PB4 Setup:
	//Disable SWD & JTAG:
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);
	GPIOB->CRL |= (GPIO_CRL_MODE4);   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR = 1 << (4);   //Mandatory turn off
	
	// CONFIGURE PORT C PIN 13 -> ONBOARD LED
	RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC -> CRH |= GPIO_CRH_MODE13;
	GPIOC -> CRH &= ~(GPIO_CRH_CNF13);
}

void Timers_Init(void)
{
	RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4 -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4 -> CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4 -> CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4 -> CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4 -> CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4 -> CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4 -> CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4 -> CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4 -> PSC = 1;   //freq/1 = 8 Mhz
	TIM4 -> ARR = 8000;
	TIM4 -> CCR1 = 0;
	TIM4 -> CCR2 = 0;

	TIM4 -> EGR |= TIM_EGR_UG;   //Update Registers
	TIM4 -> CR1 |= TIM_CR1_CEN;   //Start Counting
}

void USART_Rx_Init(void)
{
	// CONFIGURE UART | TX -> A9 | RX -> A10
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

	// RX - A10 | FLOATING INPUT
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA -> CRH |= GPIO_CRH_CNF10_0;
	
	// TX - A9 | ALTERNATE OUTPUT PUSH-PULL
	GPIOA -> CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA -> CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
	
	// BAUD RATE
	USART1 -> BRR |= 0x271; // 115200
	
	// CONTROL REGISTER | USART ENABLE | TRANSMIT ENABLE | RECEIVE ENABLE
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; 
}

void SysTick_Handler(void)
{
	myTicks++;
}

void Blink(int n)
{
  while(n!=0)
	{
		GPIOC -> ODR |= 0x2000; // SETS 
		delayMs(500);
		
		GPIOC -> ODR &= ~0x2000; // RESETS 
		delayMs(500);
		
		n--;
	}

}	

unsigned char USART_Receive()
{
  while((USART1 -> SR & USART_SR_RXNE) == 0); // SR -> STATUS REGISTER | RXNE -> RECEIVE NOT EMPTY FLAG
	
	unsigned char val = USART1 -> DR; // DR -> DATA REGISTER | 8-BIT 
	
	return val;
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

