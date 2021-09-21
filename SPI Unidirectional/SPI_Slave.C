#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "delay.h"


#define SPIx_RCC      RCC_APB1Periph_SPI2
#define SPIx          SPI2
#define SPI_GPIO_RCC  RCC_APB2Periph_GPIOB
#define SPI_GPIO      GPIOB
#define SPI_PIN_MOSI  GPIO_Pin_15
#define SPI_PIN_MISO  GPIO_Pin_14
#define SPI_PIN_SCK   GPIO_Pin_13
#define SPI_PIN_SS    GPIO_Pin_12


void NVIC_Configuration(void);
void SPIx_Init(void);
uint8_t is_SPIx_EnableSlave(void);
void USART1_Init(void);
 
 
volatile uint8_t receivedStr = 0;
uint8_t buffRx[100]= "";

volatile uint8_t RxIdx = 0;
volatile uint8_t TxCnt = 0;

void SPI2_IRQHandler(void)
{
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_TXE) != RESET)
  {
		TxCnt++;
	}
	
	if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) != RESET)
	{
		char c = SPI_I2S_ReceiveData(SPIx);		
		buffRx[RxIdx] = c;
		RxIdx++;	
		if( (buffRx[RxIdx-1] == '\n') || (RxIdx == (sizeof(buffRx)-1)))
		{
			RxIdx = 0;
			receivedStr = 1;
			//SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, DISABLE);
		}
	}
}

int main(void)
{
		int tog = 0;
    DelayInit();

	  NVIC_Configuration();
    
		SPIx_Init();

    while (1)
    {
			if(is_SPIx_EnableSlave())
			{
				if(receivedStr)
				{				 
					receivedStr = 0;				
					RxIdx = 0;
					//SPI_I2S_SendData(SPIx,'S');
					//SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
					tog ^= 1;
					GPIO_WriteBit(GPIOC, GPIO_Pin_13,(BitAction)tog);
				}		
			}			
    }
}

void NVIC_Configuration()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  /* Configure and enable SPI_MASTER interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void SPIx_Init()
{
    // Initialization struct
    SPI_InitTypeDef SPI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // Step 1: Initialize SPI
    RCC_APB1PeriphClockCmd(SPIx_RCC, ENABLE);
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStruct); 
    SPI_Cmd(SPIx, ENABLE);
		
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
	  //SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
		
		// Step 2: Initialize GPIO
    RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE);
    // GPIO pins for MOSI, MISO, and SCK
		
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_MOSI | SPI_PIN_SCK;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
    // GPIO pin for MISO
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_MISO;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    // GPIO pin for SS
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_SS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    // LED GPIO Init				
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}


uint8_t is_SPIx_EnableSlave()
{
    // Set slave SS pin low
    if(SPI_GPIO->IDR & SPI_PIN_SS)
			return 0;
		else
			return 1;
}
