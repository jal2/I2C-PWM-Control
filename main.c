
//#include <stdlib.h>
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

volatile static uint8_t new_pwm_level;
static uint8_t pwm_level;
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

int main(void)
{
	int i;

	for(i=0; i < USI_SLAVE_REGISTER_COUNT; i++)
		USI_Slave_register_buffer[i] = &new_pwm_level;
	USI_I2C_Slave_Init(SLAVE_ADDR);

	while (1) {
		if (pwm_level != new_pwm_level) {
			/* level was set by a write from twi master */
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



