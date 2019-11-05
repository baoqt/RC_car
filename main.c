#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"

#include "utils/uartstdio.h"
#include "LCD.h"
#include "BLE.h"
#include "VL53L0X.h"

#define VL53L0X_ADDRESS	0x00000052

//#ifdef DEBUG
//void __error__(char *pcFilename, uint32_t ui32Line)
//{
//}
//#endif

void ConfigureUART(void)
{
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;								// Enable UART0 module.
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOA;						// Enable GPIOA module.

  GPIOPinConfigure(GPIO_PA0_U0RX);										// Configure GPIO pins for UART mode.
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);		// Use the internal 16MHz oscillator as the UART clock source.

  UARTStdioConfig(0, 115200, 16000000);								// Initialize the UART for console I/O.
}

void ConfigureI2C(void)
{
	volatile signed long delay;
	
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;							// Enable I2C0 module.
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;						// Enable GPIOB module.
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTB_LOCK_R |= 0x4C4F434B;										// Unlock PORTB
	GPIO_PORTB_CR_R = 0xFF;															// Allow changes to PORTB
	
	GPIO_PORTB_AMSEL_R = 0x00;													// Disable analog
	GPIO_PORTB_AFSEL_R = 0x0C;													// Enable alternate function for PORTB2 and PORTB3
	GPIO_PORTB_ODR_R |= 0x08;														// Enable open drain for PORTB3 - I2C0SDA
	GPIO_PORTB_DEN_R |= 0x0C;														// Enable digital I/O on PORTB2 and PORTB3
	GPIO_PORTB_PCTL_R =0x00003300;											// Configure PMC for PORTB2 and PORTB3
	
	I2C0_MCR_R = I2C_MCR_MFE;														// Initiailize I2C0 in master mode
	I2C0_MTPR_R = 0x00000009;														// SCL clock speed set to 100Kbps
	I2C0_MSA_R = VL53L0X_ADDRESS;												// Set slave address
}

void ConfigurePORTF()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;								// Activate clock for PORTF
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;											// Unlock PORTF
	GPIO_PORTF_CR_R = 0xFF;															// Allow changes to PORTF
	
	GPIO_PORTF_AMSEL_R = 0x00;													// Disable analog
	GPIO_PORTF_PCTL_R  = 0x00000000;										// PTCL GPIO on PORTF
	GPIO_PORTF_DIR_R |= 0x0E;														// Set PORTF1-PORTF3 as outputs (RGB LEDs)
	GPIO_PORTF_DIR_R &= ~0x11;													// SET PORTF0 & PORTF4 as inputs (SW1 and SW2)
	GPIO_PORTF_AFSEL_R = 0x00;													// Disable alternate function on PORTF
	GPIO_PORTF_PUR_R |= 0x11;														// Enable PUR on PORTF0 and PORTF4
	GPIO_PORTF_DEN_R |= 0xFF;														// Enable digital I/O on PORTF
	GPIO_PORTF_DATA_R &= ~0x0E;													// Turn off LEDs
}

int main()
{
	//
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //
  FPULazyStackingEnable();

  //
  // Set the clocking to run directly from the crystal. 50MHz clock, 20ns period
  //
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	//
	// Start module startup configuration
	//
	ConfigureUART();
	UARTprintf("----------\nUART configured\n");
	ConfigureI2C();
	UARTprintf("I2C configured\n");
	ConfigurePORTF();
	UARTprintf("Tiva LEDs configured\n");
	LCD_init();
	UARTprintf("LCD initialized\n");
	
	UARTprintf("----------\n\n");
	
	//
	// Turn off on board RGB LED.
	//
	GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	
	I2C0_MDR_R = 0xAA;
	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
	
	while (I2C0_MCS_R & I2C_MCS_BUSBSY);														// Fix this maybe
	
	if (!(I2C0_MCS_R & I2C_MCS_ERROR))
	{
		UARTprintf("Slave ACK received\n");
	}
	else
	{
		UARTprintf("Error receiving slave ACK\n");
	}
	
	while (1)
	{
		
	}
}
