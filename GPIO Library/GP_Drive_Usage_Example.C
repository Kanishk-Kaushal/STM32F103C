#include "stm32f10x.h" // DEVICE HEADER

#include "GP_Drive.h"  // SELF MADE HEADER FILE FOR GPIOs

void Delays(int time); // FUNCTION PROTOTYPE 

int main(void)
{
	init_GP(PA, 0, IN, I_PP); // PORT A | PIN 0 | INPUT MODE | PULL-UP / PULL-DOWN
	
	init_GP(PC, 13, OUT50, O_GP_PP); // PORT C | PIN 13 | OUPUT MODE | PUSH-PULL | MAX SPEED -> 50Hz
	
	while(1)
	{
    
		if(R_GP(PA, 0) == 1) // CHECK STATUS OF PORT A, PIN 0
		{
			Delays(10);
			toggle_GP(PC, 13); // TOGGLE PIN STATE
			Delays(10);
		}
    
		else
		{
			W_GP(PC, 13, 1); // SET PC13 -> HIGH
		}
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
