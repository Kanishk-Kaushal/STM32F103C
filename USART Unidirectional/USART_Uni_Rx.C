// DEVICE HEADER FILE
#include "stm32f10x.h"

volatile static uint16_t myTicks=0;

// FUNCTION PROTOTYPES
void Ports_Init(void);
void USART_Rx_Init(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void Blink(int n);
void USART_Receive(void);

int main(void)
{
	Ports_Init();
	USART_Rx_Init();
		
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	while(1)
	{
		USART_Receive();
			
	}
	
}


// FUNCTION DEFINITIONS

void Ports_Init(void)
{
	
	// CONFIGURE PORT C PIN 13 -> ONBOARD LED
	RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC -> CRH |= GPIO_CRH_MODE13;
	GPIOC -> CRH &= ~(GPIO_CRH_CNF13);
}

void USART_Rx_Init(void)
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

void USART_Receive()
{
  while((USART1 -> SR & USART_SR_RXNE) == 0); // SR -> STATUS REGISTER | RXNE -> RECEIVE NOT EMPTY FLAG
	
	unsigned char val = USART1 -> DR; // DR -> DATA REGISTER | 8-BIT 
	
	if(val == 'B')
	{
		Blink(2);
	}
		
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}
