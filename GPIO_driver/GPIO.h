/////// SPI header file

#ifndef GPIO_H
#define GPIO_H
#include <stdint.h>

/*MCU specific headerfile for stm32f446xx base STM32F446RE nucleo */
#include"stm32f446xx.h"

#define __vo volatile
	
/*
 *                ARM Cortex Mx processor NVIC ISERx register Addresse
 */

#define NVIC_ISER0                       ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                       ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                       ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                       ((__vo uint32_t*)0xE000E10C)

/*
 *                ARM Cortex Mx processor NVIC ICERx register Addresse
 */
#define NVIC_ICER0                       ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                       ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                       ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                       ((__vo uint32_t*)0xE000E18C)

/*
 *                ARM Cortex Mx processor NVIC Priority register Addresse
 */
#define NVIC_PR_BASE_ADDR		 ((__vo uint32_t*)0xE000E400)



/*
	LED and Button
*/
#define LED_GREEN   			 GPIO_Pin5  // port A
#define Button				 GPIO_Pin13 // port C

/*GPIO select mode*/

#define GPIO_MODE_IN         		 ((uint32_t) 0x00)
#define GPIO_MODE_OUT			 ((uint32_t) 0x01)
#define GPIO_MODE_ALTFN			 ((uint32_t) 0x02)
#define GPIO_MODE_ANALOG		 ((uint32_t) 0x03)
#define GPIO_MODE_IT_FT			 4   // falling edge trigger
#define GPIO_MODE_IT_RT		         5   // Rising edge trigger
#define GPIO_MODE_IT_RFT		 6   // falling + Rising edge trigger

/*GPIO Output type selection*/

#define GPIO_PIN_OP_PushPull		((uint32_t) 0x00)
#define GPIO_PIN_OP_OPEN_DRAIN		((uint32_t) 0x01)

/*GPIO ouput select speed type*/

#define GPIO_PIN_LOW_SPEED		((uint32_t) 0x00)
#define GPIO_PIN_MEDIUM_SPEED		((uint32_t) 0x01)
#define GPIO_PIN_FAST_SPEED		((uint32_t) 0x02)
#define GPIO_PIN_HIGH_SPEED		((uint32_t) 0x03)

/*GPIO port select PULL-UP and PULL-DOWN type*/

#define GPIO_PIN_NO_PULL		((uint32_t) 0x00)
#define GPIO_PIN_PULL_UP		((uint32_t) 0x01)
#define GPIO_PIN_PULL_DOWN		((uint32_t) 0x02)
#define GPIO_PIN_RESERVED		((uint32_t) 0x03)

/*GPIO Pin number */

#define GPIO_Pin0			0
#define GPIO_Pin1		 	1
#define GPIO_Pin2			2
#define GPIO_Pin3			3
#define GPIO_Pin4			4
#define GPIO_Pin5			5
#define GPIO_Pin6			6
#define GPIO_Pin7			7
#define GPIO_Pin8			8
#define GPIO_Pin9			9
#define GPIO_Pin10			10
#define GPIO_Pin11			11
#define GPIO_Pin12			12
#define GPIO_Pin13			13
#define GPIO_Pin14			14
#define GPIO_Pin15			15



//// Some general marcro
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_Pin_Set				SET
#define GPIO_Pin_Reset				Reset	
/*

		ENABLE Clock for GPIO peripherals

*/

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))
/*

		DISABLE Clock for GPIO peripherals

*/
#define GPIOA_PCLK_DI() (RCC->AHB1RSTR |= (1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1RSTR |= (1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1RSTR |= (1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1RSTR |= (1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1RSTR |= (1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1RSTR |= (1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1RSTR |= (1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1RSTR |= (1<<7))
/*

		RESET Clock for GPIO peripherals

*/
#define GPIOA_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<0)) ; (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<1)) ; (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<2)) ; (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<3)) ; (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<4)) ; (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<5)) ; (RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<6)) ; (RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_PCLK_RESET() do{(RCC->AHB1RSTR |= (1<<7)) ; (RCC->AHB1RSTR &= ~(1<<7));} while(0)

/*

		Enable Clock for SYSCFG peripherals

*/
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1<<14))

/*

		Disable Clock for SYSCFG peripherals

*/
#define SYSCFG_PCLK_DI()    (RCC->APB2RSTR |= (1<<14))


/* Handle structure for SPIx Peripheral
  ***********************************************/
typedef struct
{
	uint8_t GPIO_Pin_number;
	uint8_t GPIO_Pin_mode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_Pin_Pupdcontrol;
	uint8_t GPIO_Pin_OPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_config_t;


typedef struct
{
	GPIO_TypeDef *pGPIOx;
	GPIO_config_t	GPIOConfig;
}GPIO_Handle_t;




/******************************************************************************/
/*        APIs supported by this driver                                       */
/*      																		                                  */
/*                                                                            */
/******************************************************************************/


/*
		Peripheral clock setup
*/

void GPIO_PeripheralClockControl(GPIO_TypeDef *pGPIOx,uint8_t EnorDi);


/*
		Init and De_Init
*/
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);


/*
		Read from input pin and port
*/

uint8_t GPIO_ReadfromPin(GPIO_TypeDef *pGPIOx,uint8_t Pin_Number);
uint16_t GPIO_ReadFromPort(GPIO_TypeDef *pGPIOx);

/*
		Write to input pin and port
*/

void GPIO_WriteToPin(GPIO_TypeDef *pGPIOx,uint16_t Pin_Number,uint8_t Value);
void GPIO_WriteToPort(GPIO_TypeDef *pGPIOx,uint16_t Value);

/*
		Toggle Pin
*/
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx,uint8_t Pin_Number);


/*
		IRQ Configuration and ISR Handling
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPiorityConfig(uint8_t IRQNumber,uint8_t Piority);
void GPIO_IRQHandling(uint8_t Pin_Number);




#endif
