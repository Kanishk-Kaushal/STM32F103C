/*
 Code implements Motor Code on LEDs thru PWM with Timer4, by reading Joystick values
 from a phone via LAN2UART.
 Phone Joystick->Switcher->(LAN2UART)->STMF1
 Setup:
 PWM LEDs: PB6(Ch1), PB7(Ch2)
 UART: PA9(Tx), PA10(Rx)
       Left           Right
 LED:  PA4            PA5
 PWM:  PB6            PB7
 PB4 By default is HIGH, so we disable it in GPIO_Initialize()
 */

#include "stm32f10x.h"
#include "stdlib.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

void GPIO_Initialize()
{
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function

	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH |= GPIO_CRH_CNF10;   //Input with pull-up/pull-down (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);

	//PB4 Setup:
	//Disable SWD & JTAG:
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);
	GPIOB->CRL |= (GPIO_CRL_MODE4);   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR = 1 << (4);   //Mandatory turn off
}

void Timer_Initialize()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;   //freq/1 = 8 Mhz
	TIM4->ARR = 8000;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void UART_Initilaize()
{
	/*
    	//PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 4<<4 | 5<<0;   //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      Tx Enable	  UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
	*/
	
	
	// CONFIGURE UART | TX -> A9 | RX -> A10
	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN;

	// RX - A10 | FLOATING INPUT
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA -> CRH |= GPIO_CRH_CNF10_0;
	
	// TX - A9 | ALTERNATE OUTPUT PUSH-PULL
	GPIOA -> CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA -> CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
	
	// BAUD RATE
	USART1 -> BRR |= 0x271; // 115200
	
	// CONTROL REGISTER | USART ENABLE | TRANSMIT ENABLE | RECEIVE ENABLE
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; 
	
}

uint8_t getuval()   //Reads UART Values
{
	uint8_t data;
	while(!(USART1->SR & USART_SR_RXNE));   //Check Status Register if all is Recieved
	data = USART1->DR;
	return data;
}

void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear)
{
	if (DL == 1)
		GPIOA->BSRR |= 1 << 4;   //Turn on LEFT LED
	else
		GPIOA->BRR |= 1 << 4;   //Turn off LEFT LED

	if (DR == 1)
		GPIOA->BSRR |= 1 << 5;   //Turn on RIGHT LED
	else
		GPIOA->BRR |= 1 << 5;   //Turn off RIGHT LED

	TIM4->CCR1 = (uint32_t) abs(abs(a*X) - abs(b*Y))*(gear*0.1);   //Left PWM
	TIM4->CCR2 = (uint32_t) abs(abs(p*X) - abs(q*Y))*(gear*0.1);   //Right PWM
}

void MotorCode(int x, int y, float g)
{
//void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, int g)

	if (abs(x) < 20 && abs(y) < 20)   //No Motion
		Drive(0,0,0,0,0,0,0,0,0);

	else if(abs(x) < 10 && y < 0)   //Full Backward
		Drive(0,0,0,1,0,1,x,y,g);

	else if(abs(x) < 10 && y > 0)   //Full Forward
		Drive(1,1,0,1,0,1,x,y,g);

	else if (x < 0 && abs(y) <= 10)   //Spot Turn Left
		Drive(0,1,1,0,1,0,x,y,g);

	else if (x > 0 && abs(y) <= 10)   //Spot Turn Right
		Drive(1,0,1,0,1,0,x,y,g);

	else if(x > 0 && y > 0 && x > y)   //Octet 1
		Drive(1,0,1,0,1,1,x,y,g);

	else if(x > 0 && y > 0 && x < y)   //Octet 2
		Drive(1,1,0,1,1,1,x,y,g);

	else if(x < 0 && y > 0 && abs(x) < y)   //Octet 3
		Drive(1,1,1,1,0,1,x,y,g);

	else if(x < 0 && y > 0 && abs(x) >= y)   //Octet 4
		Drive(0,1,1,1,1,0,x,y,g);

	else if(x < 0 && y < 0 && abs(x) > abs(y))   //Octet 5
	 	Drive(0,1,1,0,1,1,x,y,g);

	else if(x < 0 && y < 0 && abs(x) < abs(y))   //Octet 6
	 	Drive(0,0,0,1,1,1,x,y,g);

	else if(x > 0 && y < 0 && abs(x) < abs(y))   //Octet 7
	 	Drive(0,0,1,1,0,1,x,y,g);

	else if(x > 0 && y < 0 && abs(x) > abs(y))   //Octet 8
	 	Drive(1,0,1,1,1,0,x,y,g);

	//Test Drive:
	//Drive(1,1,1,0,0,1,x,y,g);
}

int main()
{
	GPIO_Initialize();
	Timer_Initialize();
  	UART_Initilaize();
	
	int x = 0, y = 0;
  	int trash = 0;
	float gear = 1.0;
	while (1)
  	{
		//Read LAN2UART Values
		if(getuval() == 'm')
		{
			gear = (int) ((getuval() - '0') + 1);   //Get gear value
			if(getuval() == 's')
			{
				x = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //x value
			}
			if(getuval() == 'f')
			{
				y = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //y value
			}
			trash = getuval();   //This is actually Mast CAM values but we're ignoring it for now
		}
		else
		{
			trash = trash + 1 - 1;   //Random values
		}
		x = x - 8000;
		y = y - 8000;

		if(abs(x) < 500)
			x = 0;
		if(abs(y) < 500)
			y = 0;

		MotorCode(x, y, gear);   //Run MotorCode

		}
}
/**/


