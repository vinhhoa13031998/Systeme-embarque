#include "stm32f446xx.h"
#include "GPIO.h"
#include <stdint.h>

///////////1
void GPIO_PeripheralClockControl(GPIO_TypeDef *pGPIOx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			{
			GPIOA_PCLK_EN();
			}
		else if(pGPIOx == GPIOB)
			{
			GPIOB_PCLK_EN();
			}
		else if(pGPIOx == GPIOC)
			{
			GPIOC_PCLK_EN();
			}
		else if(pGPIOx == GPIOD)
			{
			GPIOD_PCLK_EN();
			}
		else if(pGPIOx == GPIOE)
			{
			GPIOE_PCLK_EN();
			}
		else if(pGPIOx == GPIOF)
			{
			GPIOF_PCLK_EN();
			}
		else if(pGPIOx == GPIOG)
			{
			GPIOG_PCLK_EN();
			}
		else if(pGPIOx == GPIOH)
			{
			GPIOH_PCLK_EN();
			}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
			GPIOA_PCLK_DI();
			}
		else if(pGPIOx == GPIOB)
			{
			GPIOB_PCLK_DI();
			}
		else if(pGPIOx == GPIOC)
			{
			GPIOC_PCLK_DI();
			}
		else if(pGPIOx == GPIOD)
			{
			GPIOD_PCLK_DI();
			}
		else if(pGPIOx == GPIOE)
			{
			GPIOE_PCLK_DI();
			}
		else if(pGPIOx == GPIOF)
			{
			GPIOF_PCLK_DI();
			}
		else if(pGPIOx == GPIOG)
			{
			GPIOG_PCLK_DI();
			}
		else if(pGPIOx == GPIOH)
			{
			GPIOH_PCLK_DI();
			}
		}
}


/////// 2
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp_reg = 0;
	// Enable peripheral clock
	GPIO_PeripheralClockControl(pGPIO_Handle->pGPIOx,ENABLE);
	// configuration pin mode register
	if (pGPIO_Handle->GPIOConfig.GPIO_Pin_mode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		temp_reg = pGPIO_Handle->GPIOConfig.GPIO_Pin_mode << (2*pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
		pGPIO_Handle->pGPIOx->MODER &= ~((0x3) << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
		pGPIO_Handle->pGPIOx->MODER |= temp_reg;
		temp_reg = 0;   //reset valeur de temp_register
	}
	else
	{
		//interrupt mode
		if(pGPIO_Handle->GPIOConfig.GPIO_Pin_mode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
		}
		else if (pGPIO_Handle->GPIOConfig.GPIO_Pin_mode == GPIO_MODE_IT_RT)
		{
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
		}	
		else if (pGPIO_Handle->GPIOConfig.GPIO_Pin_mode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
		}
		
		////// configure the GPIO port selection in SYSCFG_EXTI
		uint8_t temp1,temp2,temp3;
		temp1 = pGPIO_Handle->GPIOConfig.GPIO_Pin_number / 4;
		temp3 = pGPIO_Handle->GPIOConfig.GPIO_Pin_number % 4;
		if (pGPIO_Handle->pGPIOx == GPIOA)
		{
			temp2 = 0;
		}
		else if (pGPIO_Handle->pGPIOx == GPIOB)
		{
			temp2 = 1;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOC)
		{
			temp2 = 2;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOD)
		{
			temp2 = 3;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOE)
		{
			temp2 = 4;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOF)
		{
			temp2 = 5;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOG)
		{
			temp2 = 6;
		} 
		else if (pGPIO_Handle->pGPIOx == GPIOH)
		{
			temp2 = 7;
		} 
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (temp2 << (4*temp3));
		
		
		
		////// Enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number); // Interrupt request from line is not masked
	}	
	
	
	// configuration Output type register
	temp_reg = pGPIO_Handle->GPIOConfig.GPIO_Pin_OPType <<(pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(1 << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);  //clearing
	pGPIO_Handle->pGPIOx->OTYPER |= temp_reg;
	temp_reg = 0;
	
	// configuration Output speed register
	temp_reg = pGPIO_Handle->GPIOConfig.GPIO_PinSpeed <<(2*pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~((0x3) << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);  //clearing
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp_reg;
	temp_reg = 0;
	
	// configuration Pull up, Pull Down register
	temp_reg = pGPIO_Handle->GPIOConfig.GPIO_Pin_Pupdcontrol << (2*pGPIO_Handle->GPIOConfig.GPIO_Pin_number);
	pGPIO_Handle->pGPIOx->PUPDR &= ~((0x3) << pGPIO_Handle->GPIOConfig.GPIO_Pin_number);    // clearing
	pGPIO_Handle->pGPIOx->PUPDR |= temp_reg;
	temp_reg = 0;
	
	// configuration the ALT functionality
	if(pGPIO_Handle->GPIOConfig.GPIO_Pin_mode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1,temp2;
		temp1 = pGPIO_Handle->GPIOConfig.GPIO_Pin_number / 8;
		temp2 = pGPIO_Handle->GPIOConfig.GPIO_Pin_number % 8;
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~((0xF) << (4*temp2));  //clearing
		pGPIO_Handle->pGPIOx->AFR[temp1] |= pGPIO_Handle->GPIOConfig.GPIO_PinAltFunMode << (4*temp2);
	}
}

///////// 3
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
		if(pGPIOx == GPIOA)
			{
			GPIOA_PCLK_RESET();
			}
		else if(pGPIOx == GPIOB)
			{
			GPIOB_PCLK_RESET();
			}
		else if(pGPIOx == GPIOC)
			{
			GPIOC_PCLK_RESET();
			}
		else if(pGPIOx == GPIOD)
			{
			GPIOD_PCLK_RESET();
			}
		else if(pGPIOx == GPIOE)
			{
			GPIOE_PCLK_RESET();
			}
		else if(pGPIOx == GPIOF)
			{
			GPIOF_PCLK_RESET();
			}
		else if(pGPIOx == GPIOG)
			{
			GPIOG_PCLK_RESET();
			}
		else if(pGPIOx == GPIOH)
			{
			GPIOH_PCLK_RESET();
			}
}

//////// 4
/*
		Read data from input PIN
*/
uint8_t GPIO_ReadfromPin(GPIO_TypeDef *pGPIOx,uint8_t Pin_Number)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> (Pin_Number)) & 0x00000001); // cai mui ten la chi huong dich chuyen va ta chi nhan gia tri cua LSB(tuc la bit cuoi cung cua port)
	return value;
}
/*
		Read data from input PORT
*/
uint16_t GPIO_ReadfromPort(GPIO_TypeDef *pGPIOx)	
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}


//////// 5

/*
		Write data to output PIN
*/
void GPIO_WriteToPin(GPIO_TypeDef *pGPIOx,uint16_t Pin_Number,uint8_t Value)
{
	if (Value == SET)
	{
		pGPIOx->ODR |= (1 << Pin_Number);
	}
	else 
	{
		pGPIOx->ODR &= ~(1 << Pin_Number);
	}
}
/*
		Write data to output PORT
*/

void GPIO_WriteToPort(GPIO_TypeDef *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
		Toggle Output PIN
*/

void GPIO_TogglePin(GPIO_TypeDef *pGPIOx,uint8_t Pin_Number)
{
	pGPIOx->ODR ^= (1 << Pin_Number);
}

/*
		Confugure the IRQ interrupt
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64 && IRQNumber > 31)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 95)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64 );
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64 && IRQNumber > 31)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber < 64 && IRQNumber > 31)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64 );
		}
	}
}

/*
		Configure the IRQ Priority interrupt
*/
void GPIO_IRQPiorityConfig(uint8_t IRQNumber,uint8_t Piority)
{
	uint8_t temp1,temp2;
	temp1 = IRQNumber / 4;
	temp2 = IRQNumber % 4;
	*(NVIC_PR_BASE_ADDR + (4*temp1)) |= (IRQNumber << (8*temp2+4));
}

/*
		Handling the IRQ
*/
void GPIO_IRQHandling(uint8_t Pin_Number)
{
	if(EXTI->PR & (1 << Pin_Number))
	{
		EXTI->PR |= (1 << Pin_Number);
	}
}
