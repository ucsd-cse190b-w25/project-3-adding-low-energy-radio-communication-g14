#include "lptimer.h"

/**
 * @brief Initializes the Low-Power Timer (LPTIM1) with LSI clock source.
 * @param timer Pointer to the LPTIM instance.
 */
void lptimer_init(LPTIM_TypeDef* timer)
{
    // Enable LSI clock and wait until it is stable
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));

    // Select LSI as the clock source for LPTIM1
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;

    // Enable LPTIM1 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    // Keep LPTIM1 clock enabled during sleep mode
    RCC->APB1SMENR1 |= RCC_APB1SMENR1_LPTIM1SMEN;

    // Disable the timer before configuration
    timer->CR &= ~LPTIM_CR_ENABLE;
    while (timer->CR & LPTIM_CR_ENABLE);

    // Configure timer to use the internal clock
    timer->CFGR &= ~LPTIM_CFGR_CKSEL;
    timer->CFGR &= ~LPTIM_CFGR_COUNTMODE;

    // Reset the counter
    timer->CNT = 0;

    // Set prescaler to 32 (Prescaler = 2^(PRESC+1), where PRESC=5 → 2^5 = 32)
    timer->CFGR &= ~LPTIM_CFGR_PRESC;
    timer->CFGR |= (LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2);

    // Disable external trigger
    timer->CFGR &= ~LPTIM_CFGR_TRIGEN;

    // Clear interrupt flags
    timer->ICR = LPTIM_ICR_ARRMCF | LPTIM_ICR_CMPMCF | LPTIM_ICR_EXTTRIGCF;

    // Enable interrupt for autoreload match
    timer->IER |= LPTIM_IER_ARRMIE;

    // Enable LPTIM1 interrupt in NVIC with highest priority
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 0);

    // Enable the timer and start counting
    timer->CR |= LPTIM_CR_ENABLE;
    while (!(timer->CR & LPTIM_CR_ENABLE));

    timer->CR |= LPTIM_CR_CNTSTRT;
}

/**
 * @brief Resets the LPTIM counter.
 * @param timer Pointer to the LPTIM instance.
 */
void lptimer_reset(LPTIM_TypeDef* timer)
{
    timer->CNT = 0;
}

/**
 * @brief Sets the autoreload value for a given period in milliseconds.
 * @param timer Pointer to the LPTIM instance.
 * @param period Time in milliseconds (converted to LPTIM ticks internally).
 */
void lptimer_set_ms(LPTIM_TypeDef* timer, uint16_t period)
{
    timer->ARR = period - 1;
}
