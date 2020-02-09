/////// SPI header file

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H
#include <stdint.h>
/*MCU specific headerfile for stm32f446xx base STM32F446RE nucleo */
#include"stm32f446xx.h"

/*

		ENABLE Clock for SPI peripherals

*/

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<13))

/*

		DISABLE Clock for SPI peripherals

*/

#define SPI1_PCLK_DISABLE() (RCC->APB2RSTR |= (1<<12))
#define SPI2_PCLK_DISABLE() (RCC->APB1RSTR |= (1<<14))
#define SPI3_PCLK_DISABLE() (RCC->APB1RSTR |= (1<<15))
#define SPI4_PCLK_DISABLE() (RCC->APB2RSTR |= (1<<13))

/*

		RESET Clock for SPI peripherals

*/
#define SPI1_PCLK_RESET() do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_PCLK_RESET() do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB2RSTR &= ~(1<<14));}while(0)
#define SPI3_PCLK_RESET() do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB2RSTR &= ~(1<<15));}while(0)
#define SPI4_PCLK_RESET() do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)

//// Some general marcro
#define ENABLE 					1
#define DISABLE 				0
#define SET 						ENABLE
#define RESET 					DISABLE
#define	FLAG_SET				SET
#define FLAG_RESET			RESET

//// Mode Device (MSTR)
#define SPI_Device_mode_master     1
#define SPI_Device_mode_slave      0


//// Bus Configuration
#define SPI_Bus_config_FD          1    // Full DUPLEX
#define SPI_Bus_config_HD          2		// Half DUPLEX
#define SPI_Bus_config_RxOnly      3    // Simple Read only

//// Clock Speed
#define SPI_CLK_DIV2                	0    // it means that SPI clock speed will be 16/2=8 MHz
#define SPI_CLK_DIV4                  1		 // it means that SPI clock speed will be 16/4=4 MHz
#define SPI_CLK_DIV8                  2		 //......
#define SPI_CLK_DIV16                 3    
#define SPI_CLK_DIV32                 4 
#define SPI_CLK_DIV64                 5
#define SPI_CLK_DIV128                6
#define SPI_CLK_DIV256                7     //....

//// Mode 8bit or 16bit (DFF) Data frame format
#define SPI_Mode_8bits                0     // by defaut
#define SPI_Mode_16bits							  1

//// Clock Polarity 
#define SPI_CPOL_High									1     // CK to 0 when idle 
#define SPI_CPOL_Low									0			// CK to 1 when idle

//// Phase Polarity 
#define SPI_CPHA_High									1     // The first clock transition is the first data capture edge 
#define SPI_CPHA_Low									0			// The second clock transition is the first data capture edge

//// Software slave management (SSM)
#define SPI_SSM_Enable                1      // 
#define SPI_SSM_Disable								0      // 

//// Set FLAG SPI
#define TXE_FLAG											(1 << 1)
#define RXNE_FLAG											(1 << 0)
#define BUSY_FLAG											(1 << 7)



/* Configuration structure for SPIx Peripheral
  ***********************************************/


typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_CLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_config_t;

/* Handle structure for SPIx Peripheral
  ***********************************************/
typedef struct
{
	SPI_TypeDef *pSPIx;
	SPI_config_t SPIConfig;
}SPI_Handle_t;
/******************************************************************************/
/*        APIs supported by this driver                                       */
/*      																		                                  */
/*                                                                            */
/******************************************************************************/
// Get flage
uint8_t Get_FLAG(SPI_TypeDef *pSPIx,uint32_t Flagname);

/*
		Peripheral clock setup
*/
void SPI_PeripheralClockControl(SPI_TypeDef *pSPIx,uint8_t EnorDi);
 
   //Peripheral setup
void SPI_Peripheral(SPI_TypeDef *pSPIx,uint8_t EnorDi);
	 //SSI configure to keep MSTR to 1
void SPI_SSIConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi);
	 //SSOE configure to enable NSS output enable
void SPI_SSOEConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi);
/*
		Init and De_Init
*/
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_TypeDef *pSPIx);
/*
		Data send and receive
*/
void SPI_Send_Data(SPI_TypeDef *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_Send_Data(SPI_TypeDef *pSPIx,uint8_t *pRxBuffer,uint32_t Len);
/*
		IRQ Configuration and ISR Handling
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPiorityConfig(uint8_t IRQNumber,uint8_t Piority);
void SPI_ISRHandling(SPI_TypeDef *pSPIx);


#endif
