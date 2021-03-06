#include "stm32f10x.h"  // DEVICE HEADER

void Delays(int time);


unsigned char debounce(void);

int main(void)
{
	RCC -> APB2ENR |= 4; // ENABLE PORT A
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A PIN TO 0
	GPIOA -> CRL |= 0x8; // PIN A0 | INPUT | PUSH PULL
	
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13 TO 0
	GPIOC -> CRH |= 0x00300000; // PIN C13 | OUTPUT MODE | PUSH PULL | MAX SPEED -> 50Hz

	unsigned char CurrentState = 0;
	unsigned char LastState = 0;
	
	
	
	while(1)
	{
		CurrentState = debounce();
		
		if((CurrentState == 1) & (LastState == 0))
		{
			GPIOC -> ODR ^= 0x2000;
			Delays(10);
		}
						
		LastState = CurrentState;
	}
	
	
	
}


void Delays(int time) // RANDOM DELAY FUNCTION
{
	int t;
	
	for(;time>0;time--)
	{
	 for(t = 0; t < 100000; t++)
		{
			__NOP(); // PAUSE CPU CYCLE
		}
	}
}

unsigned char debounce() // TO PREVENT MECHANICAL ERROR FROM BUTTON
{
	if(GPIOA -> IDR & 0x00000001)
	{
		Delays(10);
			
		if(GPIOA -> IDR & 0x00000001)
		{
			return 1;
		}
	}
	return 0;
}
