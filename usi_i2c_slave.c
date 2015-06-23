/*! \file

USI I2C Slave Driver

copyright (c) 2015 Joerg Albert <jal2@gmx.de>

This implementation is based on the one from
Adam Honse (https://github.com/CalcProgrammer1/Stepper-Motor-Controller)
but uses function pointer to process and generate the
data received/transmitted. This make the i2c communication shorter for simple
devices, e.g. our i2c2pwm converter, which needs a single byte only.

Furthermore this code uses an 8bit slave address during initialisation.

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

#include <stdbool.h>

#include "usi_i2c_slave.h"

//Microcontroller Dependent Definitions
#if defined (__AVR_ATtiny24__) || \
	defined (__AVR_ATtiny44__) || \
	defined (__AVR_ATtiny84__)
#define DDR_USI	        DDRA
#define PORT_USI	PORTA
#define PIN_USI		PINA
#define PORT_USI_SDA	PA6
#define PORT_USI_SCL	PA4
#define PIN_USI_SDA	PINA6
#define PIN_USI_SCL	PINA4
#define USI_START_VECTOR    USI_STR_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect  
#endif

#if defined(__AVR_AT90Tiny2313__) || \
	defined(__AVR_ATtiny2313__)
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PB5
#define PORT_USI_SCL        PB7
#define PIN_USI_SDA         PINB5
#define PIN_USI_SCL         PINB7
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if 	defined( __AVR_ATtiny25__ ) ||		\
	defined( __AVR_ATtiny45__ ) ||		\
	defined( __AVR_ATtiny85__ )
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PB0
#define PORT_USI_SCL        PB2
#define PIN_USI_SDA         PINB0
#define PIN_USI_SCL         PINB2
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif


#if !defined(DDR_USI)
# error "unsupported MCU -please add defines"
#endif

static unsigned char i2c_slave_address; /*!< the i2c slave address we acted upon, 8 bit */
static ReadProc_t read_proc;
static WriteProc_t write_proc;
static unsigned char byte_nr; /*!< count the number of bytes received or sent (excl. the device address) during the current access */

///////////////////////////////////////////////////////////////////////////////////////////////////
////USI Slave States///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


enum
{
  USI_SLAVE_CHECK_ADDRESS,
  USI_SLAVE_SEND_DATA,
  USI_SLAVE_SEND_DATA_ACK_WAIT,
  USI_SLAVE_SEND_DATA_ACK_CHECK,
  USI_SLAVE_RECV_DATA_WAIT,
  USI_SLAVE_RECV_DATA_ACK_SEND
} USI_I2C_Slave_State;

/////////////////////////////////////////////////
////USI Register Setup Values////////////////////
/////////////////////////////////////////////////

#define USI_SLAVE_COUNT_ACK_USISR			0b01110000 | (0x0E << USICNT0)	//Counts one clock (ACK)
#define USI_SLAVE_COUNT_BYTE_USISR			0b01110000 | (0x00 << USICNT0)	//Counts 8 clocks (BYTE)
#define USI_SLAVE_CLEAR_START_USISR			0b11110000 | (0x00 << USICNT0)  //Clears START flag
#define USI_SLAVE_SET_START_COND_USISR		0b01110000 | (0x00 << USICNT0)
#define USI_SLAVE_SET_START_COND_USICR		0b10101000
#define USI_SLAVE_STOP_DID_OCCUR_USICR		0b10111000
#define USI_SLAVE_STOP_NOT_OCCUR_USICR		0b11101000

/////////////////////////////////////////////////
////USI Direction Macros/////////////////////////
/////////////////////////////////////////////////

#define USI_SET_SDA_OUTPUT()	{ DDR_USI |=  (1 << PORT_USI_SDA); }
#define USI_SET_SDA_INPUT() 	{ DDR_USI &= ~(1 << PORT_USI_SDA); }

#define USI_SET_BOTH_INPUT() 	{ DDR_USI &= ~((1 << PORT_USI_SDA) | (1 << PORT_USI_SCL)); }

////////////////////////////////////////////////////////////////////////////////////////////////////

void USI_I2C_Slave_Init(uint8_t address, ReadProc_t rproc, WriteProc_t wproc)
{
	PORT_USI &= ~(1 << PORT_USI_SCL);
	PORT_USI &= ~(1 << PORT_USI_SDA);

	i2c_slave_address = address;
	read_proc = rproc;
	write_proc = wproc;

	USI_SET_BOTH_INPUT();
	
	USICR = (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) | (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
}

/////////////////////////////////////////////////////////////////////////////////
// ISR USI_START_vect - USI Start Condition Detector Interrupt                 //
//                                                                             //
//  This interrupt occurs when the USI Start Condition Detector detects a      //
//  start condition.  A start condition marks the beginning of an I2C          //
//  transmission and occurs when SDA has a high->low transition followed by an //
//  SCL high->low transition.  When a start condition occurs, the I2C slave    //
//  state is set to check address mode and the counter is set to wait 8 clocks //
//  (enough for the address/rw byte to be transmitted) before overflowing and  //
//  triggering the first state table interrupt.  If a stop condition occurs,   //
//  reset the start condition detector to detect the next start condition.     //
/////////////////////////////////////////////////////////////////////////////////

ISR(USI_START_VECTOR)
{
	USI_I2C_Slave_State = USI_SLAVE_CHECK_ADDRESS;

	USI_SET_SDA_INPUT();

	// wait for SCL to go low to ensure the Start Condition has completed (the
	// start detector will hold SCL low ) - if a Stop Condition arises then leave
	// the interrupt to prevent waiting forever - don't use USISR to test for Stop
	// Condition as in Application Note AVR312 because the Stop Condition Flag is
	// going to be set from the last TWI sequence
	while((PIN_USI & (1 << PIN_USI_SCL)) && !((PIN_USI & (1 << PIN_USI_SDA))));

	if(!(PIN_USI & (1 << PIN_USI_SDA)))
	{
		// a Stop Condition did not occur
		USICR = USI_SLAVE_STOP_NOT_OCCUR_USICR;
	}
	else
	{
		// a Stop Condition did occur
    	USICR = USI_SLAVE_STOP_DID_OCCUR_USICR;
	}

	USISR = USI_SLAVE_CLEAR_START_USISR;
}

/////////////////////////////////////////////////////////////////////////////////
// ISR USI_OVERFLOW_vect - USI Overflow Interrupt                              //
//                                                                             //
//  This interrupt occurs when the USI counter overflows.  By setting this     //
//  counter to 8, the USI can be commanded to wait one byte length before      //
//  causing another interrupt (and thus state change).  To wait for an ACK,    //
//  set the counter to 1 (actually -1, or 0x0E) it will wait one clock.        //
//  This is used to set up a state table of I2C transmission states that fits  //
//  the I2C protocol for proper transmission.                                  //
/////////////////////////////////////////////////////////////////////////////////

ISR(USI_OVERFLOW_VECTOR)
{
	switch (USI_I2C_Slave_State)
	{
		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_CHECK_ADDRESS                                        //
		//                                                                     //
		//  The first state after the start condition, this state checks the   //
		//  received byte against the stored slave address as well as the      //
		//  global transmission address of 0x00.  If there is a match, the R/W //
		//  bit is checked to branch either to sending or receiving modes.     //
		//  If the address was not for this device, the USI system is          //
		//  re-initialized for start condition.                                //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_CHECK_ADDRESS:

			if((USIDR == 0) || ((USIDR & ~1) == i2c_slave_address))
			{				
				if (USIDR & 0x01)
				{
					USI_I2C_Slave_State = USI_SLAVE_SEND_DATA;
				}
				else
				{
					byte_nr=0;
					USI_I2C_Slave_State = USI_SLAVE_RECV_DATA_WAIT;
				}

				//Set USI to send ACK
				USIDR = 0;
				PORT_USI |= (1 << PORT_USI_SDA);
				USI_SET_SDA_OUTPUT();
				USISR = USI_SLAVE_COUNT_ACK_USISR;
			}
			else
			{
				//Set USI to Start Condition Mode
				USICR = USI_SLAVE_SET_START_COND_USICR;
				USISR = USI_SLAVE_SET_START_COND_USISR;
			}
			break;

		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_SEND_DATA_ACK_WAIT                                   //
		//                                                                     //
		//  Wait 1 clock period for the master to ACK or NACK the sent data	   //
		//  If master NACK's, it means that master doesn't want any more data. //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_SEND_DATA_ACK_WAIT:

			//After sending, immediately shut off PORT = 1 to prevent driving
			//the line high (I2C should *NEVER* drive high, and could damage
			//connected devices if operating at different voltage levels)
			PORT_USI &= ~(1 << PORT_USI_SDA);

			USI_I2C_Slave_State = USI_SLAVE_SEND_DATA_ACK_CHECK;
			USI_SET_SDA_INPUT();
			USISR = USI_SLAVE_COUNT_ACK_USISR;
			break;

		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_SEND_DATA_ACK_CHECK                                  //
		//                                                                     //
		//  Check USIDR to see if master sent ACK or NACK.  If NACK, set up    //
		//  a reset to START conditions, if ACK, fall through into SEND_DATA   //
		//  to continue sending data.                                          //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_SEND_DATA_ACK_CHECK:
			
			if(USIDR)
			{
				//The master sent a NACK, indicating that it will not accept
				//more data.  Reset into START condition state
				USICR = USI_SLAVE_SET_START_COND_USICR;
				USISR = USI_SLAVE_SET_START_COND_USISR;
				return;
			}
			//else: fall through into SEND_DATA

		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_SEND_DATA                                            //
		//                                                                     //
		//  Set USIDR to the data to be sent, then set up SDA registers to     //
		//  enable data transmission in the next 8 clocks.  Set to wait 8      //
		//  clocks and proceed to wait for ACK.                                //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_SEND_DATA:

			USIDR = (*read_proc)(byte_nr++);
	
			USI_I2C_Slave_State = USI_SLAVE_SEND_DATA_ACK_WAIT;

			//To send data, DDR for SDA must be 1 (Output) and PORT for SDA
			//must also be 1 (line drives low on USIDR MSB = 0 or PORT = 0)
			PORT_USI |= (1 << PORT_USI_SDA);
			USI_SET_SDA_OUTPUT();
			USISR = USI_SLAVE_COUNT_BYTE_USISR;
			break;

		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_RECV_DATA_WAIT                                       //
		//                                                                     //
		//  Prepares to wait 8 clocks to receive a data byte from the master.  //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_RECV_DATA_WAIT:

			USI_I2C_Slave_State = USI_SLAVE_RECV_DATA_ACK_SEND;

			USI_SET_SDA_INPUT();
			USISR = USI_SLAVE_COUNT_BYTE_USISR;
			break;

		/////////////////////////////////////////////////////////////////////////
		// Case USI_SLAVE_RECV_DATA_ACK_SEND                                   //
		//                                                                     //
		//  After waiting for the master to finish transmission, this reads    //
		//  USIDR into either the i2c buffer or internal address, then sends   //
		//  an acknowledgement to the master.                                  //
		/////////////////////////////////////////////////////////////////////////
		case USI_SLAVE_RECV_DATA_ACK_SEND:

			USI_I2C_Slave_State = USI_SLAVE_RECV_DATA_WAIT;
			
			if ((*write_proc)(USIDR, byte_nr++)) {
				/* write an ACK */
				USIDR = 0;
				PORT_USI |= (1 << PORT_USI_SDA);
				USI_SET_SDA_OUTPUT();
			} else {
				/* NAK to the master */
				USI_SET_SDA_INPUT();
			}
			USISR = USI_SLAVE_COUNT_ACK_USISR;
			break;
	}
}
