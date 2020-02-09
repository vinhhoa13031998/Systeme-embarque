/////// I2C header file
/////// Peripheral control bit (I2C_CR1bits.bit0)

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H
#include <stdint.h>


/*MCU specific headerfile for stm32f446xx base STM32F446RE nucleo */
#include"stm32f446xx.h"

/*

		ENABLE Clock for I2C peripherals

*/
#define I2C1_PCLK_EN()       (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()       (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()       (RCC->APB1ENR |= (1 << 23))

/*

		DISABLE Clock for I2C peripherals

*/
#define I2C1_PCLK_DISABLE()       (RCC->APB1RSTR |= (1 << 21))
#define I2C2_PCLK_DISABLE()       (RCC->APB1RSTR |= (1 << 22))
#define I2C3_PCLK_DISABLE()       (RCC->APB1RSTR |= (1 << 23))

/*

		RESET Clock for I2C peripherals

*/
#define I2C1_PCLK_RESET()    do {(RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21));} while(0)
#define I2C2_PCLK_RESET()    do {(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_PCLK_RESET()    do {(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)


//// Some general marcro
#define ENABLE 					1
#define DISABLE 				0
#define SET 						ENABLE
#define RESET 					DISABLE
#define	FLAG_SET				SET
#define FLAG_RESET			RESET

//// Mode Device (MSTR)
#define I2C_Device_MASTER			     1
#define I2C_Device_SLAVE           0

//// Clock Speed
#define I2C_CLKSpeed_STANDARD			100000
#define I2C_CLKSpeed_FASE					400000

//// ACK control
#define I2C_ACK_ENABLE						ENABLE
#define I2C_ACK_DISABLE						DISABLE

//// I2C Duty
#define I2C_MODE_DUTY_2						0   //Th + Tl = 3*CCR*PCLK 	
#define I2C_MODE_DUTY_16_9				1		//Th + Tl = 25*CCR*PCLK 	

//// I2C master mode selection
#define I2C_FastMode							1
#define I2C_StandardMode          0

//// Some FLAG in I2C
#define I2C_SB_FLAG								(1 << 0)
#define I2C_TXE_FLAG							(1 << 7)
#define I2C_RXNE_FLAG							(1 << 6)
#define I2C_BTF_FLAG							(1 << 2)
#define I2C_ADDR_FLAG             (1 << 1)
#define I2C_STOPF_FLAG						(1 << 4)
#define I2C_ADD10_FLAG            (1 << 3)
#define I2C_AF_FLAG               (1 << 10)
#define I2C_OVR_FLAG              (1 << 11)
#define I2C_PERERR_FLAG           (1 << 12)

/* Configuration structure for I2Cx Peripheral
  ***********************************************/


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddresse;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FmDutyCycle;
	uint8_t  I2C_MasterMode;
}I2C_config_t;

/* Handle structure for SPIx Peripheral
  ***********************************************/

typedef struct
{
	I2C_config_t I2C_config;
	I2C_TypeDef *pI2Cx;
}I2C_Handle_t;
/******************************************************************************/
/*        APIs supported by this driver                                       */
/*      																		                                  */
/*                                                                            */
/******************************************************************************/


// Get flage I2C
uint8_t Get_FLAG_I2C(I2C_TypeDef *pI2Cx,uint32_t Flagname);

/*
		Peripheral clock setup
*/
void I2C_PeripheralClockControl(I2C_TypeDef *pI2Cx,uint8_t EnorDi);

	 //Get Plck speed
uint32_t I2C_GetPLCK1(void);

   //Peripheral setup
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx,uint8_t EnorDi);
	 //I2C manage ACKING
void I2C_ACKorNACK(I2C_TypeDef *pI2Cx,uint8_t EnorDi);

/*
		Init and De_Init
*/
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

/*
		Data send and receive
*/

void I2C_Send_Data(I2C_Handle_t *pI2C_Handle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddress);
void I2C_Receive_Data(I2C_Handle_t *pI2C_Handle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddress);
/*
		IRQ Configuration and ISR Handling
*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQPiorityConfig(uint8_t IRQNumber,uint8_t Piority);

	


#endif
