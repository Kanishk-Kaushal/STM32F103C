#include "stm32f10x.h"  // DEVICE HEADER

void Delays(int time);

int main(void)
{
	RCC -> APB2ENR |= 4; // ENABLE PORT A
	
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A TO 0
	
	GPIOA -> CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0
	
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	
	while(1)
	{
		if(GPIOA -> IDR & 0x00000001) // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{
			Delays(10);
			GPIOC -> ODR ^= 0x2000; // CHANGE THE PIN STATE
			Delays(10);
		}
		
		else
		{
			GPIOC->ODR = 0x2000; // SET PC13 TO HIGH
		}
	}
	
}


void Delays(int time) // RANDOM DELAY FUNCTION
{
	int t;
	
	for(;time>0;time--)
	{
	 for(t=0;t<100000;t++)
		{
			
		}
	}
}

