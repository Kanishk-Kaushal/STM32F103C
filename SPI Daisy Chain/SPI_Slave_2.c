// HEADER FILE

#include "stm32f10x.h"

volatile static myTicks=0;
char rec_data;

// FUNCTION PROTOTYPES

void spi(void);
void ports(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void send(char tx_char);
void CS_Disable (void);
void CS_Enable (void);
void blink (int b);
char recieve(void);


int main()
{
  SysTick_Config(SystemCoreClock / 1000);
	spi();
	ports();
	
	while(1)
	{
		
	  // TRANSMITS TO MASTER
		
		if (GPIOA -> IDR & 0x00000001) // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{ 
			CS_Enable();
			
			send('C');
			
			blink(1);
			
			CS_Disable();
			
			delayMs(10);
		}
		
		// RECEIVES DATA
		
		rec_data = recieve();
		
		if(rec_data == 'B')
		{
			blink(3);
		}
		
		else if(rec_data == 'A')
		{
			send(rec_data);
			blink(1);
		}			
   
	}


}

// FUNCTION DEFINITIONS 

char recieve(void)
{
	SPI1 -> CR1 &= ~(1<<14);
	
 char rx_val = 0;
	
  while(SPI1 -> SR & 0x1)
     {
        rx_val = SPI1 -> DR;
     }
 return rx_val;
}

void ports(void)
{
	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;  // GPIOA CLOCK ENABLE
	RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;  // GPIOC CLOCK ENABLE
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	
	// SPI PORTS
	
	GPIOA->CRL |= (1<<21);
	GPIOA->CRL &= ~(1<<20);
	GPIOA->CRL |= (1<<22) | (1<<23);
	GPIOA->CRL |= (1<<29);
	GPIOA->CRL &= ~(1<<28);
	GPIOA->CRL |= (1<<30) | (1U<<31);
	
	// PUSHBUTTONS
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A0  TO 0 
	GPIOA -> CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0
	
	GPIOA -> CRL &= 0xFFFFFF0F; // RESET PORT A1 TO 0
	GPIOA -> CRL |= 0x128; // INPUT MODE | PUSH-PULL| PIN A1

}

void spi()
{
	RCC -> APB2ENR |= (1<<12);  // ENABLE SPI1 CLOCK

	SPI1 -> CR1 |=  (1<<9);
	SPI1 -> CR1 |= 0x4; // MASTER
	SPI1 -> CR1 |= 0x31; // PRESCALAR -> fclk / 265
	SPI1 -> CR2 |= 0x4;
	SPI1 -> CR1 |= 0x40; // ENABLE SPI PERIPHERALS
	
}

void send(char tx_char)
{
    SPI1 -> CR1 |= (1<<14);
    SPI1 -> DR = tx_char;
	
		while(SPI1 -> SR & 0x80);
	  
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

void CS_Enable (void)
{
	GPIOA -> BSRR |= (1<<4)<<16;
}

void CS_Disable (void)
{
	GPIOA -> BSRR |= (1<<4);
}

void blink (int k)
{
	int t = 0;
	
	while (t < k)
	{
		GPIOC -> ODR |= 0x2000;
		delayMs(100);
		GPIOC ->  ODR &= ~0x2000;
		delayMs(100);
		t++;
	}
}

