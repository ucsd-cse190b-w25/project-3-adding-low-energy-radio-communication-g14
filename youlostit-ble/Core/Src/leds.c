
/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
	/* Enable the clock for GPIOA and GPIOB */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable clock for GPIOA
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable clock for GPIOB

	/* Configure PA5 as an output by clearing all bits and setting the mode */
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

	/* Configure the GPIO to use low speed mode */
	GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

	/* Turn off the LED */
	GPIOA->ODR &= ~GPIO_ODR_OD5;

	GPIOB->MODER &= ~GPIO_MODER_MODE14;     // Clear mode bits
	GPIOB->MODER |= GPIO_MODER_MODE14_0;    // Set PB14 as output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;     // Push-pull output
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14;     // No pull-up/pull-down
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos); // Low speed mode
	GPIOB->ODR &= ~GPIO_ODR_OD14;           // Turn off LED2 (PB14)
}

void leds_set(uint8_t led)
{
  // TODO implement this
	if((led & 0xFF) == 0x01){
		GPIOA->ODR |= GPIO_ODR_OD5; // Set PA5 to turn on LED1
	}
	else if((led & 0xFF) == 0x02){
		GPIOB->ODR |= GPIO_ODR_OD14;  // Set PB14 to turn on LED2
	}
	else if((led & 0xFF) == 0x03){
		GPIOA->ODR &= ~GPIO_ODR_OD5; // Set PA5 to turn off LED1
	}
	else if(((led & 0xFF) == 0x04)){
		GPIOB->ODR &= ~GPIO_ODR_OD14; // Clear PB14 to turn off LED2
	}
}
