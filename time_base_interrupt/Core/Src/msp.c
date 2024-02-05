/**
 * msp.c
 * initializing and configuring the low-level hardware peripherals of the microcontroller.
 * MSP stands for MCU Support Package. MSPs are user callback functions that perform
 * initializations at the system level,
 * such as GPIOs, interrupts, DMA, and clock.
 *
 * Created on: Jan 23, 2024
 *      Author: sherlyhartono
 */

#include "stm32f4xx_hal.h"

void HAL_MspInit(void) {
	// 1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// 2. Enable the required system exceptions of the arm cortex mx processor
	// The SHCSR enables the system handlers, and indicates:
	// the pending status of the BusFault, MemManage fault, and SVC exceptions
	// the active status of the system handlers.
	// Insert 111 into the bit 16, 17, 18.
	SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions

	// 3. Configure the priority for the system exceptions. (Optional)
	// Highest priority for all.
	// Systick is already set up by hal init. Don't need to set priority here.
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}


/**
 * Base timer low level initialization.
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer)
{

	//1. Enable the clock for the TIM6 peripheral
	__HAL_RCC_TIM6_CLK_ENABLE();

	//2. Enable the IRQ of TIM6. This is the interrupt handler.
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	//3. setup the priority for TIM6_DAC_IRQn
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);

}
