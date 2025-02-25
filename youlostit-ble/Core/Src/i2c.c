
#include "i2c.h"
#define I2C_SPEED 400000
#define I2C_TIMEOUT 1000000
void i2c_init() {
	// trying to configure noise filters then enable timer for I2C by setting the PE bit of the register
	// wanna change the baud rate to 400 khz
	// configure the pins on the MCU so they are connected to the correct I2C peripheral rather than operating as GPIO pins

	// Enable GPIOB clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// enable i2c2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

	// Set PB10 and PB11 to Alternate Function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11); // Clear mode bits
	GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1); // Set to Alternate Function mode

	// Set PB10 and PB11 to Open-Drain Output
	GPIOB->OTYPER |= (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);

	// Enable Pull-Up resistors for PB10 and PB11
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11); // Clear
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0); // Enable pull-ups

	// Configure PB10 and PB11 to use Alternate Function 4 (AF4) for I2C2
	//GPIOB->AFRH &= ~((0xF << GPIO_AFRH_AFSEL10_Pos) | (0xF << GPIO_AFRH_AFSEL11_Pos)); // Clear AFR bits
	GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL10_Pos) | (4 << GPIO_AFRH_AFSEL11_Pos));     // Set AF4 for PB10, PB11

	I2C2->TIMINGR = 0x10909CEC;	// sets up timing for communication, that values is for 100khz
	I2C2->CR1 |= I2C_CR1_PE; // Enable I2C2


}
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
	volatile uint32_t timeout;

	    // Clear any existing errors
	    I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF;

	    printf("Starting I2C transaction - ISR: 0x%lx, CR2: 0x%lx\n", I2C2->ISR, I2C2->CR2);

	    if (dir == 0) {  // Writing data
	        // Configure the transfer
	        I2C2->CR2 = 0; // Clear CR2 first
	        I2C2->CR2 = (address << 1) | (len << 16) | I2C_CR2_START;

	        printf("Write CR2 configured: 0x%lx\n", I2C2->CR2);

	        for (uint8_t i = 0; i < len; i++) {
	            timeout = I2C_TIMEOUT;
	            while (!(I2C2->ISR & I2C_ISR_TXIS)) {
	                if (I2C2->ISR & I2C_ISR_NACKF) {
	                    printf("Error: NACK received. ISR=0x%lx\n", I2C2->ISR);
	                    I2C2->CR2 |= I2C_CR2_STOP;
	                    return 6;
	                }

	                if (--timeout == 0) {
	                    printf("Error 1: TXIS timeout. ISR=0x%lx, CR2=0x%lx\n",
	                           I2C2->ISR, I2C2->CR2);
	                    I2C2->CR2 |= I2C_CR2_STOP;
	                    return 1;
	                }
	            }
	            I2C2->TXDR = data[i];
	            printf("Wrote byte %d: 0x%02x\n", i, data[i]);
	        }

	        timeout = I2C_TIMEOUT;
	        while (!(I2C2->ISR & I2C_ISR_TC)) {
	            if (--timeout == 0) {
	                printf("Error 2: TC timeout. ISR=0x%lx\n", I2C2->ISR);
	                return 2;
	            }
	        }

	        I2C2->CR2 |= I2C_CR2_STOP;
	    }
	else {	// 1 for reading data
		I2C2->CR2 = ((address << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_RD_WRN; // Set the device address and indicate a read
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START; // Set how many bytes we’re reading and start the communication

		for (uint8_t i = 0; i < len; i++) {
			timeout = I2C_TIMEOUT;
			while (!(I2C2->ISR & I2C_ISR_RXNE)) { // Wait until data is received
				if (--timeout == 0) {
					printf("error 3");
					return 3; // If we wait too long, return an error
				}
			}
			data[i] = I2C2->RXDR; // Store the received data
		}

		timeout = I2C_TIMEOUT;
		while (!(I2C2->ISR & I2C_ISR_TC)) { // Wait until the transfer is complete
			if (--timeout == 0){
				printf("error 4");
				return 4; // If we wait too long, return an error
			}
		}

		I2C2->CR2 |= I2C_CR2_STOP; // Send a stop condition to end the communication
	}

	return 0; // If we got here, everything worked!
}

