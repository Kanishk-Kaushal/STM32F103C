#include "stm32f10x.h" // DEVICE HEADER

// FUNCTION PROTOTYPES
void Delays(int time);
void DelayMs(unsigned long t);
void DelayMillis(void);
void systick_init(void);

int main(void)
{
	systick_init();
	
		
	RCC -> APB2ENR |= 0x10; // ENABLE PORT C			
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13	
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	
	while(1)
	{
		// BLINKING THE PC13 LED
		
		GPIOC -> ODR |= 0x2000;  // ODR13 -> SET | LED (ON-BOARD) -> OFF | LED (BREADBOARD) -> ON
		DelayMs(1000);
		GPIOC -> ODR &= ~0x2000; // ODR13 -> CLEAR | LED (ON-BOARD) -> ON | LED (BREADBOARD) -> OFF
		DelayMs(1000);
				
	}
	
}

void Delays(int time) // RANDOM DELAY FUNCTION
{
	int t;
	for(; time > 0; time--)
	{
	 for(t = 0; t < 100000; t++)
		{
			
		}
	}
}


void systick_init(void) // INITIALISING SYSTICK -> ARM MPU TIMER
{
	// RESET VALUES FOR EACH REGISTER (PRESENT BY DEFAULT)
	SysTick -> CTRL = 0; // CTRL -> CONTROL AND STATUS REGISTER
	SysTick -> LOAD = 0x00FFFFFF; // LOAD -> RELOAD VALUE REGISTER
	SysTick -> VAL = 0; // VAL -> CURRENT VALUE REGISTER
	
  // NEW DESIRED VALUE
	SysTick -> CTRL |= 5; // 5 = 0b101 | 0th BIT = ENABLE -> HIGH | 2nd BIT = CLK_SOURCE -> HIGH => CLOCK SOURCE = AHB
}

void DelayMillis(void) // DELAY FOR EXACTLY 1 MILI-SECOND
{
	SysTick -> LOAD = 0x11940; // Ox11940 = 72000 (Decimal)
	SysTick -> VAL = 0; // STARTING FROM 0
	while((SysTick -> CTRL & 0x00010000) == 0); // CHECKS THE "COUNT FLAG" BIT | 1ms TO EXIT THIS LOOP
}

void DelayMs(unsigned long t) // FUCNTION TO REPEAT THE "DelayMillis()" FUNCTION BY DESIRED NUMBER OF TIMES
{
	for(; t > 0; t--)
		{
			DelayMillis();
		}
}

