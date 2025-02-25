
/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
	// Enable clock for TIM2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Disable the timer
    timer->CR1 &= ~TIM_CR1_CEN;

    // Reset counter and status register
    timer->CNT = 0;
    timer->SR = 0;

    // Set auto-reload value for 50 ms (assuming 4 MHz clock)
    timer->ARR = 0xFFFF;   // 50 ms period (1 kHz clock -> 50 ticks)

    // Enable update interrupt
    timer->DIER |= TIM_DIER_UIE;

    // Enable TIM2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    // Enable the timer
    timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this
	timer->CNT = 0;

}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // TODO implement this
	uint32_t timer_clock = 4000000; // Timer clock after prescaler
	uint32_t period_ticks = (timer_clock / 1000) * period_ms; // Calculate period in ticks
	timer->ARR = period_ticks - 1; // Set auto-reload value
	timer_reset(TIM2);
}
