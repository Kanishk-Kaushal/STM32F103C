#include "stm32f10x.h" // DEVICE HEADER

void Delays(int time);

int signal = 0;

void EXTI0_IRQHandler(void);

unsigned char debounce(void);

int main(void)
{
	
	
	RCC -> APB2ENR |= 4; // ENABLE PORT A
	
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A TO 0
	
	GPIOA -> CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0
	
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	
	__disable_irq(); // IRQ -> INTERRUPT REQUEST
	
	AFIO -> EXTICR[0] = 0x00; // ACTIVATE INTERRUPTS FOR PORT A PIN 0
	
	EXTI -> IMR |= 1; // ENABLE INTERRUPT FOR EXTI0 | IMR -> INTERRUPT MASK REGISTER
	
	EXTI -> RTSR |= 1; // GENERATE SYSTEM INTERRUPT AT RISING EDGE | RTSR -> RISING TRIGGER SELECTION REGISTER
	
	NVIC_EnableIRQ(EXTI0_IRQn); // ENABLE GLOBAL INTERRUPT FUNCTION
	
	__enable_irq();
	
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
	
	for(; time > 0; time--)
	{
	 for(t = 0; t < 100000; t++)
		{
			__NOP(); // PAUSE CPU FOR 1 CYCLE
		}
	}
}

void EXTI0_IRQHandler() // INTERRUPT HANDLER FUNCTION OF PA0
{
	EXTI->PR |= 1; // PR -> PENDING REGISTER | SET | PR -> 1 => PRESCENCE OF INTERRUPT REQUEST
	
	if(signal)
	{
		signal = 0;
	}
	else
	{
		signal = 1;
	}
}


unsigned char debounce() // TO PREVENT MECHANICAL ERROR FROM BUTTON
{
	if(signal)
	{
		Delays(10);
			
		if(signal)
		{
			return 1;
		}
	}
	return 0;
}
