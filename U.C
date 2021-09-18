//transmitter
#include "stm32f10x.h"
unsigned char temp='a';
volatile static uint16_t myTicks = 0;
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void ports(void);
void SysTick_Handler(void)
{
	myTicks++;
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

int main()
{
	//usart1 clock enable / gpioa
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
	

	
	//Setting up delay
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	// declare tx (a9) and rx (a10) pins
  ports();
  
	//usart div value 
	USART1->BRR = 0x14DC; //for 72MGHz
	
	//             tx enable       rx enable     usart enable 
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	
	while(1)
	{
	
		if(GPIOA -> IDR & 0x00000001) // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{
			USART1->DR = temp;
			while(!(USART1->SR & USART_SR_TXE));
		}
		
		
		
	}
		
	
}



void ports(void)
{
	//Setting up GPIO pin A9
	RCC ->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOA ->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA ->CRH &= ~(GPIO_CRH_CNF9_0);
	
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A TO 0
	
	GPIOA -> CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0

}



//reciever
#include "stm32f10x.h"
volatile static uint16_t myTicks = 0;




void SysTick_Handler(void);
void delayMs(uint16_t ms);
void ports(void);
void SysTick_Handler(void)
{
	myTicks++;
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

void blink ()
{
	int t=0;
	while (t<2)
	{
		GPIOC -> ODR |= 0x2000;
		delayMs(100);
		GPIOC ->  ODR &= ~0x2000;
		delayMs(100);
		t++;
	}
}

int main()
{
	//usart1 clock enable / gpioa
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
	

	
	//Setting up delay
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	// declare tx (a9) and rx (a10) pins
  ports();
  
	//usart div value 
	USART1->BRR = 0x14DC; //for 72MGHz
	
	//             tx enable       rx enable     usart enable 
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	
  ports();
 

	
	while(1)
	{
		if( USART1->SR & USART_SR_RXNE)
		{
			unsigned char temp=USART1->DR;
			if(temp=='a')
				blink();
		}

		
	}
		
	
}

void ports(void)
{
//Setting up GPIO pin A9
	RCC ->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOA ->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA ->CRH &= ~(GPIO_CRH_CNF9_0);
	
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
}

