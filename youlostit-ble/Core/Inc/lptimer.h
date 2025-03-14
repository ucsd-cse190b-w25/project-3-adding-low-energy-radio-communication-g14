#ifndef INC_LPTIMERH
#define INC_LPTIMERH

#include <stm32l475xx.h>

void lptimer_init(LPTIM_TypeDef* timer);
void lptimer_reset(LPTIM_TypeDef* timer);
void lptimer_set_ms(LPTIM_TypeDef* timer, uint16_t period);

#endif /* INC_LPTIMERH */
