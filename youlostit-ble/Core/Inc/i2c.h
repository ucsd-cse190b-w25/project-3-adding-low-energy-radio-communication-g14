
#ifndef i2c_H
#define i2c_H
#include <stm32l475xx.h>

void i2c_init();

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);

#endif /* i2c_H */
