/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 16, 2024
 *      Author: sherlyhartono
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * Handle structure is to initialize the GPIO port.
 * The pin will be specified in the pin config variable.
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * 1. Peripheral Clock setup.
 * This will call a macro.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * 2. Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // This will set the reset register to 1.


/*
 * 3. Data read
 */
// This will only return 0 or 1
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// There are 16 pins so we need uint16. Return the content of the input data register.
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*
 * 4. Data write
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/*
 * 5. Toggle output pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * 6. IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
