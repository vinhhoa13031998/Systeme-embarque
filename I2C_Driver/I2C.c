#include "stm32f446xx.h"
#include "I2C_driver.h"
#include <stdint.h>


uint16_t AHB_PrescTable[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_PrescTable[4] = {2,4,8,16};

///////
static void I2C_GenerateSTARTcondi(I2C_TypeDef *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_TypeDef *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRF(I2C_TypeDef *pI2Cx);
static void I2C_GenerateSTOPcondi(I2C_TypeDef *pI2Cx);

////////// Generate the START condition
static void I2C_GenerateSTARTcondi(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 8);
}	

////////// Generate the STOP condition
void I2C_GenerateSTOPcondi(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 9);
}	


/////////// Get Address in write mode
static void I2C_ExecuteAddressPhaseWriteMode(I2C_TypeDef *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);   // in write mode (=0)
	pI2Cx->DR = SlaveAddr;
}

/////////// Get Address in write mode
static void I2C_ExecuteAddressPhaseReadMode(I2C_TypeDef *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1<<0);   // in read mode (=1)
	pI2Cx->DR = SlaveAddr;
}

//////////////////// Clear the ADDR Flag
static void I2C_ClearADDRF(I2C_TypeDef *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

//////////////////// Get Flag of I2C
uint8_t Get_FLAG_I2C(I2C_TypeDef *pI2Cx,uint32_t Flagname)
{
	if (pI2Cx->SR1 & (Flagname))
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//////////////////// Manage ACK
void I2C_ACKorNACK(I2C_TypeDef *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 10);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

///////////////////// PLL mode
uint32_t RCC_GetPLLOutput()
{
	return 0;
}

//////////////////// Get the clock speed
uint32_t I2C_GetPLCK1()
{
	uint32_t pclk1;
	uint32_t CLK;
	uint8_t clkSrc,temp1,temp2,AHBP,APBP;

	
	// Get the Frequence of system
	clkSrc = ((RCC->CFGR >> 2) & 0x03);       // bit 3:2 of CFGR
	if (clkSrc == 0)						 							// IF that is on mode HSI 16Mhz
	{
		CLK = 16000000;
	}
	else if (clkSrc == 1)  									  // IF that is on mode HSE 8Mhz
	{
		CLK = 8000000;
	}
	else if (clkSrc == 2)
	{
		CLK = RCC_GetPLLOutput();  							// IF that is on mode PLL, we have to calculate
	}
	
	// Read the prescalar of AHB bus for frequence
	temp1 = ((RCC->CFGR >> 4) & 0xF);         // bit 7:4 HPRE
	if (temp1 < 8) 
	{
		AHBP = 1;
	}
	else 
	{
		AHBP = AHB_PrescTable[temp1-8];
	}
	
	// Read the prescalar of APB bus for frequence
	temp2 = ((RCC->CFGR >> 10) & 0x7);         // bit 12:10 PPRE1
	if (temp2 < 4) 
	{
		APBP = 1;
	}
	else 
	{
		APBP = APB_PrescTable[temp2-4];
	}
		
	pclk1 = CLK /(APBP*AHBP);
	
	return pclk1;
}


///////////1
void I2C_PeripheralClockControl(I2C_TypeDef *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DISABLE();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DISABLE();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DISABLE();
		}
	}	
}

////////////////////2
void I2C_Init(I2C_Handle_t *pI2C_Handle)
{
	uint32_t tempREG =0;
	
	// Enable the Peripheral clock control
	I2C_PeripheralClockControl(pI2C_Handle->pI2Cx,ENABLE);
	
	// ACK control
	pI2C_Handle->pI2Cx->CR1 |= 1 << 10;
	//tempREG |= pI2C_Handle->I2C_config.I2C_ACKControl << 10;
	//pI2C_Handle->pI2Cx->CR1 = tempREG;
	
	// configure the FREQ of CR2
	tempREG =0;
	tempREG |= I2C_GetPLCK1()/ 1000000U; 
	pI2C_Handle->pI2Cx->CR2 = ( tempREG & (0x3F));
	
	
	//program the device own address
	tempREG = 0;
	tempREG |= pI2C_Handle->I2C_config.I2C_DeviceAddresse << 1;
	tempREG |= 1 << 14;
	pI2C_Handle->pI2Cx->OAR1 = tempREG;
	
	// configure CCR of CCR register
	tempREG =0;
	if (pI2C_Handle->I2C_config.I2C_SCLSpeed <= I2C_CLKSpeed_STANDARD)
	{
		//mode is standard mode
		tempREG |= (I2C_GetPLCK1() / (2*pI2C_Handle->I2C_config.I2C_SCLSpeed) & 0xFFF);
		// tempREG &= ~(1 << 15); its not neccesaire
	}
	else
	{
		//mode is fast mode
	
		tempREG |= (1 << 15);
		//Fm duty cycle
		tempREG |= (pI2C_Handle->I2C_config.I2C_FmDutyCycle << 14);
		
		if (pI2C_Handle->I2C_config.I2C_FmDutyCycle == I2C_MODE_DUTY_2)
		{
			tempREG |= (I2C_GetPLCK1() / (3*pI2C_Handle->I2C_config.I2C_SCLSpeed) & 0xFFF);	
		}
		else 
		{
			tempREG |= (I2C_GetPLCK1() / (25*pI2C_Handle->I2C_config.I2C_SCLSpeed) & 0xFFF);
		}
	}
	pI2C_Handle->pI2Cx->CCR = tempREG;
	tempREG =0;
	if (pI2C_Handle->I2C_config.I2C_SCLSpeed <= I2C_CLKSpeed_STANDARD)
	{
		tempREG = (I2C_GetPLCK1() /1000000U) +1;
	}
	else
	{
		tempREG = ((I2C_GetPLCK1()*300) / 1000000U) +1;
	}
	pI2C_Handle->pI2Cx->TRISE = (tempREG & 0x3F);
}


//////////////////3
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_PCLK_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_PCLK_RESET();
	}
}


////////////////// 4 Send data to slave
void I2C_Send_Data(I2C_Handle_t *pI2C_Handle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddress)
{
	// 1.Generate the start condition
	I2C_GenerateSTARTcondi(pI2C_Handle->pI2Cx);
	
	// 2.Confirm that start generate is completed by checking the SB flag in SR1
	// Note : Until SB is cleared SCL will be streched (pull to LOW)
	while (!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_SB_FLAG));
	
	// 3.Send the address of slave and r/w mode
	I2C_ExecuteAddressPhaseWriteMode(pI2C_Handle->pI2Cx,SlaveAddress);
	
	// 4.Confirm that ADD is sent by checking the ADDR	 flag in SR1
	while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_ADDR_FLAG));
	
	// 5.Clear the ADD FLag
	I2C_ClearADDRF(pI2C_Handle->pI2Cx);
	
	// 6.Send data until len = 0
	
	while(Len > 0)
	{
		while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_TXE_FLAG)); // wait till TXE is set
		pI2C_Handle->pI2Cx->DR = (uint32_t)*pTxBuffer;
		Len--;
		pTxBuffer ++;
	}
	
	// 7. When Len =0, wait for TXE =1 and BTF =1 before send the bit STOP
	// Note: TXE =1 and BTF =1 mean that both SR and DR are empty
	// When the BTF = 1 SCL will be streched
	while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_TXE_FLAG)); // wait till TXE is set
	while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_BTF_FLAG)); // wait till BTF is set
	
	// 8. Generate the STOP condition
	I2C_GenerateSTOPcondi(pI2C_Handle->pI2Cx);
}

//////////////////// 5 Read data from slave
void I2C_Receive_Data(I2C_Handle_t *pI2C_Handle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddress)
{
	
	// 1.Generate the start condition
	I2C_GenerateSTARTcondi(pI2C_Handle->pI2Cx);
	
	// 2.Confirm that start generate is completed by checking the SB flag in SR1
	// Note : Until SB is cleared SCL will be streched (pull to LOW)
	while (!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_SB_FLAG));
	
	// 3.Send the address of slave and r/w mode
	I2C_ExecuteAddressPhaseReadMode(pI2C_Handle->pI2Cx,SlaveAddress);
	
	// 4.Confirm that ADD is sent by checking the ADDR	 flag in SR1
	while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_ADDR_FLAG));
	
	// 5. Receive data
	
	/////// Case Lenght = 1
	if (Len == 1)
	{
		// disable ACK
		I2C_ACKorNACK(pI2C_Handle->pI2Cx,DISABLE);
		
		// Clear the ADD FLag
		I2C_ClearADDRF(pI2C_Handle->pI2Cx);
		
		// Wait until RXNE become 1
		while(!Get_FLAG_I2C(pI2C_Handle->pI2Cx,I2C_RXNE_FLAG));
		
		// Generate the STOP condition
		I2C_GenerateSTOPcondi(pI2C_Handle->pI2Cx);
		
		// Read the date
		*pRxBuffer = pI2C_Handle->pI2Cx->DR;
	}
	else if (Len >1)
	{
		// Clear the ADD FLag
		I2C_ClearADDRF(pI2C_Handle->pI2Cx);
		for (uint8_t i = Len; i >0; i--)
		{
			//Wait until RXNE become 1
			if(Len == 2)
			{
				// disable ACK
				I2C_ACKorNACK(pI2C_Handle->pI2Cx,DISABLE);
				
				// Generate the STOP condition
				I2C_GenerateSTOPcondi(pI2C_Handle->pI2Cx);	
			}
			// Read the date
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;
			
			// increase the buffer address
			pRxBuffer++;
		}
	}
	// Re-Enable the ACK
	if (pI2C_Handle->I2C_config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ACKorNACK(pI2C_Handle->pI2Cx,ENABLE);
	}
}
	

