#include "stm32f446xx.h"
#include "SPI_driver.h"
#include <stdint.h>

///////////0
uint8_t Get_FLAG(SPI_TypeDef *pSPIx,uint32_t Flagname)
{
	if (pSPIx->SR & (Flagname))
	{
		return FLAG_SET;
	}
	else 
		return FLAG_RESET;
}
/////////// 1.1
void SPI_Peripheral(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << 6);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 6);
	}
}
/////////// 1.2
void SPI_SSIConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << 8);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 8);
	}
}

/////////// 1.3
void SPI_SSOEConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << 2);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << 2);
	}
}


///////////1
void SPI_PeripheralClockControl(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			{
			SPI1_PCLK_EN();
			}
		else if (pSPIx == SPI2)
			{
			SPI2_PCLK_EN();
			}
		else if (pSPIx == SPI3)
			{
			SPI3_PCLK_EN();
			}
		else if (pSPIx == SPI4)
			{
			SPI4_PCLK_EN();
			}
	}
	else
	{
		if(pSPIx == SPI1)
			{
			SPI1_PCLK_DISABLE();
			}
		else if (pSPIx == SPI2)
			{
			SPI2_PCLK_DISABLE();
			}
		else if (pSPIx == SPI3)
			{
			SPI3_PCLK_DISABLE();
			}
		else if (pSPIx == SPI4)
			{
			SPI4_PCLK_DISABLE();
			}
	}
}

////////////////////2
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	uint32_t temp_reg;
	// configuration device mode
	temp_reg |= pSPI_Handle->SPIConfig.SPI_DeviceMode 	<< 2;
	// Bus configuration
	if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_Bus_config_FD)
	{
		temp_reg &= (1 << 15);
	}
	else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_Bus_config_HD)
	{
		temp_reg |= (1 << 15);
	}
	else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_Bus_config_RxOnly)
	{
		// only receive
		temp_reg |= (1 << 10);
	}
	// Clock speed
	temp_reg |= pSPI_Handle->SPIConfig.SPI_CLKSpeed   	<<3;
	// Data frame format 
	temp_reg |= pSPI_Handle->SPIConfig.SPI_DFF					<<11;
	// configuration Clock Polarity
	temp_reg |= pSPI_Handle->SPIConfig.SPI_CPOL  				<<1;
	// Configuration Phase Polarity 
	temp_reg |= pSPI_Handle->SPIConfig.SPI_CPHA					<<0;
	// Enable or Disable SPI slave management
	temp_reg |= pSPI_Handle->SPIConfig.SPI_SSM					<<9;
	
	// Set  SPI_CR1
	pSPI_Handle->pSPIx->CR1 = temp_reg;
}


//////////////////3
void SPI_DeInit(SPI_TypeDef *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_PCLK_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_PCLK_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_PCLK_RESET();
	}
}



//////////////////4
////////////////// Fonction blocking call
void SPI_Send_Data(SPI_TypeDef *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len!=0)
	{
		while(Get_FLAG(pSPIx,TXE_FLAG) == FLAG_RESET);
		if (pSPIx->CR1 & (1 << 11)) // if DFF in mode 16bits
			{
				pSPIx->DR = *((uint16_t*)pTxBuffer);// load the data to the data register and change data from 8bits to 16bits
				Len--;
				Len--;
				pTxBuffer ++;
			}
		else
			{
				pSPIx->DR = *(pTxBuffer);
				Len--;
				pTxBuffer ++;
			}
	}
} 

	
