/*-----------------------------------------------------*\
|  USI I2C Slave Driver                                 |
|                                                       |
| This library provides a robust, interrupt-driven I2C  |
| slave implementation built on the ATTiny Universal    |
| Serial Interface (USI) hardware.  Slave operation is  |
| implemented as a register bank, where each 'register' |
| is a pointer to an 8-bit variable in the main code.   |
| This was chosen to make I2C integration transparent   |
| to the mainline code and making I2C reads simple.     |
| This library also works well with the Linux I2C-Tools |
| utilities i2cdetect, i2cget, i2cset, and i2cdump.     |
|                                                       |
| Adam Honse (GitHub: CalcProgrammer1) - 7/29/2012      |
|            -calcprogrammer1@gmail.com                 |
\*-----------------------------------------------------*/
#ifndef USI_I2C_SLAVE_H
#define USI_I2C_SLAVE_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//USI I2C Initialize
//  address - the slave address (8 bit)
void USI_I2C_Slave_Init(char address);

#define USI_SLAVE_REGISTER_COUNT 8
extern volatile uint8_t* USI_Slave_register_buffer[USI_SLAVE_REGISTER_COUNT];

#endif
