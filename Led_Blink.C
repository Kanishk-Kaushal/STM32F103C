
#include "stm32f10x.h" // DEVICE HEADER

void delay(int rep);

int main(void)
{
	RCC -> APB2ENR |= 0x10; // 0b10000 -> ENABLING THE IOPCEN BIT | IOPCEN -> INPUT OUTPUT PORT C ENABLE
	
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PORT C PIN 13
	
	GPIOC -> CRH |= 0x00300000; // OUTPUT MODE | MAX SPEED = 50hz | GENERAL PUSH PULL
	
	GPIOC -> ODR |= 0x2000; // ODR -> OUTPUT DATA REGISTER | SETS ODC13 PIN | IN-BUILT LED (PC13) -> OFF 
	

	
	while(1)
	{
		// BLINKING THE PC13 LED
		
		GPIOC -> ODR |= 0x2000;
		delay(10);
		GPIOC -> ODR &= ~0x2000; // ODR13 -> CLEAR | LED -> ON
		delay(10);
		
	}
	

}  

void delay(int rep) // RANDOM TIME DELAY FUNCTION
{
		
	for(; rep > 0; rep--)
	{
		
		int i;
		for(i = 0; i < 100000; i++)
		{
			__NOP();
		}
	}
	
	
}
