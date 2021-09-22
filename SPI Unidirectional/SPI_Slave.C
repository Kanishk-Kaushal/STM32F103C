#include "stm32f10x.h"
volatile static myTicks=0;
char rec_data;

//prototype start
void spi(void);
void ports(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
char recieve(void);
void CS_Disable (void);
void CS_Enable (void);
void blink (void);
// prototype end

int main()
{
  SysTick_Config(SystemCoreClock / 1000);
	
	spi();
	ports();
	


	while(1)
	{

		rec_data=recieve();
		
		if(rec_data=='A') // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{
			blink();
		}

   
	}


}



void ports(void)
{ //c13 output
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	//slave mode 
	RCC ->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA-> CRL |=(1<<25);
	GPIOA->CRL  &=~(1u<<24);
	GPIOA->CRL |= (1<<26) | (1<<27);
	
	

}
void spi()
{      
  RCC->APB2ENR |= (1<<12);
SPI1->CR1 |= (1<<10) | (1<<15);
SPI1->CR1 &= ~(0x4); // Slave Mode
SPI1->CR1 |= 0x31; // fclk / 265
SPI1->CR2 |= (1<<0);
SPI1->CR1 |= 0x40; // Enabling SPI SPI periph
}

char recieve(void)
{
 char rx_val;
  		while(SPI1->SR & 0x1)
                  {
                     rx_val = SPI1->DR;
                  }
 return rx_val;
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
void SysTick_Handler(void)
{
	myTicks++;
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}
