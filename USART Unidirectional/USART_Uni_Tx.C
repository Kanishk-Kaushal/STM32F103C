// DEVICE HEADER FILE
#include "stm32f10x.h"

volatile static uint16_t myTicks=0;

// FUNCTION PROTOTYPES
void Ports_Init(void);
void USART_Tx_Init(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void USART_Transmit(unsigned char value);

int main(void)
{

	Ports_Init();
	USART_Tx_Init(); 
	
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	while(1)
	{
		if(GPIOB ->IDR & GPIO_IDR_IDR5)
		{
      USART_Transmit('B'); 
			
		}
		else
		{
			USART_Transmit('A');
			
		}
				
	}
	
}


// FUNCTION DEFINITIONS

void Ports_Init(void)
{
	// CONFIGURE PORT B PIN 5 -> PUSHBUTTON
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB -> CRL &= ~(GPIO_CRL_MODE5);
	GPIOB -> CRL |= GPIO_CRL_CNF5_1;
	GPIOB -> CRL &= ~(GPIO_CRL_CNF5_0);
}

void USART_Tx_Init(void)
{
	// CONFIGURE UART | TX -> A9 | RX -> A10
	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

	// RX - A10 | FLOATING INPUT
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA -> CRH |= GPIO_CRH_CNF10_0;
	
	// TX - A9 | ALTERNATE OUTPUT PUSH-PULL
	GPIOA -> CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA -> CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
	
	// BAUD RATE
	USART1 -> BRR |= 0x2580; // 9600
	
	// CONTROL REGISTER | USART ENABLE | TRANSMIT ENABLE | RECEIVE ENABLE
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void SysTick_Handler(void)
{
	myTicks++;
}

void USART_Transmit(unsigned char value)
{
  USART1 -> DR = value;
	while((USART1 ->SR & USART_SR_TXE) == 0); // TXE -> TRANSMIT EMPTY
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}
