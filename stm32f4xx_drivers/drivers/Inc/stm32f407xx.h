/*
 * stm32f407xx.h
 *
 *  Created on: Feb 16, 2024
 *      Author: sherly hartono
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#define __vo volatile
#include <stdint.h>

/**
 * 0. Generic macros
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         	RESET
#define FLAG_SET 			SET
/**
 * 1. BASE ADDRESSES
 */

/**
 * Base address of Flash and SRAM memories.
 */
#define FLASH_BASEADDR	0x08000000U
#define SRAM1_BASEADDR	0x20000000U
#define SRAM2_BASEADDR	0x20001C00U
#define ROM_BASEADDR	0x20001C00U
#define SRAM SRAM1_BASEADDR

/**
 * AHB and APBx Bus Peripheral base address
 */
#define PERIPH_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASE
#define APB2PERIPH_BASEADDR 0x40010000U

#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

/**
 * Base address of the peripherals for AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x00000000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x00000400)

/**
 * Base address of peripherals for APB1 bus
 */
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x00005400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x00005800)

/**
 * Base address of peripherals for APB2 bus
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x00003C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x00003000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x00003800)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x00001000)

/**
 * 2. PERIPHERALS REGISTER DEFINITION STRUCTURES
 */

/**
 * 2A. Peripheral structure for GPIO
 * Some registers are highly volatile
 */
typedef struct {
	// This has to be in order.
	__vo uint32_t MODER;
	__vo uint32_t OTYPER; // Output type
	__vo uint32_t OSPEEDR; // Output speed
	__vo uint32_t PUPDR; // pull-up/pull-down register
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR; // bit set/reset register
	__vo uint32_t AFR[2]; // for low and high register

} GPIO_RegDef_t;

/*
 * 2B. peripheral register definition structure for RCC
 */
typedef struct {
	__vo uint32_t CR; /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t PLLCFGR; /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t CFGR; /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CIR; /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t AHB1RSTR; /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t AHB2RSTR; /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t AHB3RSTR; /*!< TODO,     										Address offset: 0x18 */
	uint32_t RESERVED0; /*!< Reserved, 0x1C                                                       */
	__vo uint32_t APB1RSTR; /*!< TODO,     										Address offset: 0x20 */
	__vo uint32_t APB2RSTR; /*!< TODO,     										Address offset: 0x24 */
	uint32_t RESERVED1[2]; /*!< Reserved, 0x28-0x2C                                                  */
	__vo uint32_t AHB1ENR; /*!< TODO,     										Address offset: 0x30 */
	__vo uint32_t AHB2ENR; /*!< TODO,     										Address offset: 0x34 */
	__vo uint32_t AHB3ENR; /*!< TODO,     										Address offset: 0x38 */
	uint32_t RESERVED2; /*!< Reserved, 0x3C                                                       */
	__vo uint32_t APB1ENR; /*!< TODO,     										Address offset: 0x40 */
	__vo uint32_t APB2ENR; /*!< TODO,     										Address offset: 0x44 */
	uint32_t RESERVED3[2]; /*!< Reserved, 0x48-0x4C                                                  */
	__vo uint32_t AHB1LPENR; /*!< TODO,     										Address offset: 0x50 */
	__vo uint32_t AHB2LPENR; /*!< TODO,     										Address offset: 0x54 */
	__vo uint32_t AHB3LPENR; /*!< TODO,     										Address offset: 0x58 */
	uint32_t RESERVED4; /*!< Reserved, 0x5C                                                       */
	__vo uint32_t APB1LPENR; /*!< TODO,     										Address offset: 0x60 */
	__vo uint32_t APB2LPENR; /*!< RTODO,     										Address offset: 0x64 */
	uint32_t RESERVED5[2]; /*!< Reserved, 0x68-0x6C                                                  */
	__vo uint32_t BDCR; /*!< TODO,     										Address offset: 0x70 */
	__vo uint32_t CSR; /*!< TODO,     										Address offset: 0x74 */
	uint32_t RESERVED6[2]; /*!< Reserved, 0x78-0x7C                                                  */
	__vo uint32_t SSCGR; /*!< TODO,     										Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR; /*!< TODO,     										Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR; /*!< TODO,     										Address offset: 0x88 */
	__vo uint32_t DCKCFGR; /*!< TODO,     										Address offset: 0x8C */
	__vo uint32_t CKGATENR; /*!< TODO,     										Address offset: 0x90 */
	__vo uint32_t DCKCFGR2; /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/**
 * 3. Peripheral definitions
 */

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)

/**
 * 4. Clock Enable Macros (function) for GPIO peripherals
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0)) // Turn on the 0th bit
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))

/**
 * 5. Clock Disable Macros (function) for GPIO peripherals
 */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0)) // Turn off the 0th bit
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))

#endif /* INC_STM32F407XX_H_ */
