/*! \file

USI I2C Slave Driver

copyright (c) 2015 Joerg Albert <jal2@gmx.de>

This implementation is based on the one from
Adam Honse (https://github.com/CalcProgrammer1/Stepper-Motor-Controller)
but uses function pointer to process and generate the
data received/transmitted.

	DESCRIPTION:

	Call for initialisation

		USI_I2C_Slave_Init(slave_addr, read_proc, write_proc)

	where
		slave_addr - the TWI slave address (8 bit)
		read_proc - gets called on every read access from the master; the signature is
		            uint8_t read_proc(uint8_t index)
			    where index is the number of the data byte in the current read access
		write_proc - gets called on every write access from the master; the signature is
			   bool write_proc(uint8_t data, uint8_t index)
			   where index is the number of the data byte in the current write access
			   and data is the byte just written by the master
			   This procedure returns false if the slave shall assert NAK immediately
*/

#ifndef USI_I2C_SLAVE_H
#define USI_I2C_SLAVE_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef uint8_t (*ReadProc_t)(uint8_t);
typedef bool (*WriteProc_t)(uint8_t, uint8_t);

void USI_I2C_Slave_Init(uint8_t address, ReadProc_t rproc, WriteProc_t wproc);

#endif /* #ifndef USI_I2C_SLAVE_H */

