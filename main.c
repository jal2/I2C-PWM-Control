/*! \file
  This file implements a simple i2c to pwm converter.
  Initial state is 0% duty cycle for the PWM (on PWM_PIN).
  You send a single byte where 255 is the maximum duty cycle.

  copyright (c) 2015 Joerg Albert <jal2@gmx.de>
*/
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usi_i2c_slave.h"     		

#if defined(__AVR_ATtiny85__)
/* pin config for ATtiny85:
   PB0 - SDA (DIL: pin 7)
   PB1 - PWM (OC0B) (DIL: pin 6)
   PB2 - SCL (DIL: pin 5)
   PB3 - LED (DIL: pin 2)
*/
#define LED_PIN PB3
#define PWM_PIN PB1
#else
# error "no pin config provided"
#endif

/* 8bit slave address */
#define 	SLAVE_ADDR       0x56


#ifndef 	F_CPU
# error "F_CPU undefined"
#endif

volatile static uint8_t new_pwm_level; /*!< new pwm duty cycle set in the IRQ handler */
static uint8_t pwm_level; /*!< last pwm duty cycle seen in the main loop */
uint8_t led_on; /* if > 0 the LED is on */

static void init_pwm(void)
{
	DDRB |= (1<<PWM_PIN);
	TCCR0A = (1<<COM0B1)|(1<<WGM00);  // mode #1
	TCCR0B = (1<<CS01);  // div8 (any speed would do)
	OCR0B = 0;          // off at init
}

/* set LED on/off

   LED on == pin high, i.e. connect LED to ground.
*/
static inline void set_led(bool on)
{
	if (on)
		PORTB |= (1 << LED_PIN);
	else
		PORTB &= ~(1 << LED_PIN);
}

/*! set PWM level

  \param[in] lvl PWM level (0...255)
*/
static inline set_pwm(uint8_t lvl)
{
	OCR0B = lvl;
}

/*! procedure called for a i2c master read access.

Be aware that it is called in IRQ context, so be fast here!

\param index the number of the data byte requested, starting with 0
\return data byte to be passed to the i2c master
 */
static uint8_t master_read(uint8_t index)
{
	return new_pwm_level;
}

/*! procedure called for a i2c master write access.

Be aware that it is called in IRQ context, so be fast here!

\param data the data byte
\param index the number of the data byte in the current master write access
\return true if we will assert ACK on the i2c bus after the next data byte written by the master
 */
static bool master_write(uint8_t data, uint8_t index)
{
	new_pwm_level = data;
	return false;
}

int main(void)
{
	int i;

	USI_I2C_Slave_Init(SLAVE_ADDR, master_read, master_write);

	while (1) {
		if (pwm_level != new_pwm_level) {
			/* a new level was set by a write from twi master */
			pwm_level = new_pwm_level;
			set_led(true);
			set_pwm(pwm_level);
			led_on=9; /* with a delay of 250ms, the LED will be on for 2s for each change in the PWM value */
		}

		if (led_on > 0)
			led_on--;
		else
			set_led(false);
		_delay_ms(250);
	}
	return 0;
}



