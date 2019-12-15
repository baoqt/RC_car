// Header file for RN4871 BLE module
//
// Uses the following pins of PORTB:
//

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "utils/UARTstdio.h"

#define VL53L0X_ADDRESS	0x00000052

void VL53L0X_init(void);
void Test_I2C0_Connection(void);

void ConfigureI2C0(void);

void ConfigureI2C0(void)
{
	volatile signed long delay;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_I2C0;							// Enable I2C0 module.
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;						// Enable GPIOB module.
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_2 | ~GPIO_PIN_3;		// Disable analog
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_2 | GPIO_PIN_3;			// Enable alternate function for PORTB2 and PORTB3
	GPIO_PORTB_ODR_R |=  GPIO_PIN_3;										// Enable open drain for PORTB3 - I2C0SDA
	// Configure PMC for PORTB2 and PORTB3
	GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB2_I2C0SCL | GPIO_PCTL_PB3_I2C0SDA;
	GPIO_PORTB_DEN_R |= GPIO_PIN_2 | GPIO_PIN_3;				// Enable digital I/O on PORTB2 and PORTB3
	
	I2C0_MCR_R = I2C_MCR_MFE;														// Initiailize I2C0 in master mode
	I2C0_MTPR_R = 0x00000009;														// SCL clock speed set to 100Kbps
}

void VL53L0X_init(void)
{
	ConfigureI2C0();
	//Test_I2C0_Connection();
}

void Test_I2C0_Connection(void)
{
	UARTprintf("----------\n");
	UARTprintf("Testing I2C0 connection...\n");
	I2C0_MSA_R = VL53L0X_ADDRESS;												// Set slave address in transmit mode in write mode
	UARTprintf("To Slave Address: 0x%x\n", VL53L0X_ADDRESS);
	I2C0_MDR_R = 0xC0;																	// Set data and start transmission
	UARTprintf("Register Address: 0xC0\n");
	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
	UARTprintf("Sending register read request...\n\n");

	while (I2C0_MCS_R & I2C_MCS_BUSY)										// Check if master is busy
	{
	}
	
	if (!(I2C0_MCS_R & I2C_MCS_DATACK))
	{
		UARTprintf("Slave data ACK received\n");
	}
	else
	{
		UARTprintf("Error receiving slave data ACK\n");
	}
	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
	{
		UARTprintf("Slave address ACK received\n");
	}
	else
	{
		UARTprintf("Error receiving slave address ACK\n");
	}
	
	I2C0_MSA_R = VL53L0X_ADDRESS + 1;										// Set slave address in transmit mode in read mode
	UARTprintf("\nRetrieving register value...\n\n");
	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
	
	while (I2C0_MCS_R & I2C_MCS_BUSY)										// Check if master is busy
	{
	}
	
	if (!(I2C0_MCS_R & I2C_MCS_DATACK))
	{
		UARTprintf("Slave data ACK received\n");
	}
	else
	{
		UARTprintf("Error receiving slave data ACK\n");
	}
	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
	{
		UARTprintf("Slave address ACK received\n");
	}
	else
	{
		UARTprintf("Error receiving slave address ACK\n");
	}
	
	UARTprintf("\nValue Received: %d\n", I2C0_MDR_R);
	if (I2C0_MCS_R & (I2C_MCS_DATACK | I2C_MCS_ADRACK))
	{
		UARTprintf("Transmission failed\n");
	}
	else
	{
		UARTprintf("Transmission success\n");
	}
	UARTprintf("I2C0 test concluded\n----------\n");
}
