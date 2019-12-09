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
#include "HC-SR04.h"

#define VL53L0X_ADDRESS	0x00000052

//#ifdef DEBUG
//void __error__(char *pcFilename, uint32_t ui32Line)
//{
//}
//#endif

void ConfigureUART0(void)
{
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;								// Enable UART0 module.
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;						// Enable GPIOA module.

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
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;						// Enable GPIOB module.
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTB_AMSEL_R &= GPIO_PIN_2 | GPIO_PIN_3;			// Disable analog
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_2 | GPIO_PIN_3;			// Enable alternate function for PORTB2 and PORTB3
	GPIO_PORTB_ODR_R |=  GPIO_PIN_3;										// Enable open drain for PORTB3 - I2C0SDA
	GPIO_PORTB_DEN_R |= GPIO_PIN_2 | GPIO_PIN_3;				// Enable digital I/O on PORTB2 and PORTB3
	GPIO_PORTB_PCTL_R |= 0x00003300;										// Configure PMC for PORTB2 and PORTB3
	
	I2C0_MCR_R = I2C_MCR_MFE;														// Initiailize I2C0 in master mode
	I2C0_MTPR_R = 0x00000009;														// SCL clock speed set to 100Kbps
}

void ConfigurePORTFLEDs()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;								// Activate clock for PORTF
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;											// Unlock PORTF
	GPIO_PORTF_CR_R = 0xFF;															// Allow changes to PORTF
	
	// Disable analog
	GPIO_PORTF_AMSEL_R &= GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	// PTCL GPIO on PORTF
	GPIO_PORTF_PCTL_R  = 0x00000000;
	// Set PORTF1-PORTF3 as outputs (RGB LEDs)
	GPIO_PORTF_DIR_R |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	// Disable alternate function on PORTF
	GPIO_PORTF_AFSEL_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	// Enable digital I/O on PORTF
	GPIO_PORTF_DEN_R |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	// Turn off LEDs
	GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
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
	ConfigureUART0();
	UARTprintf("----------\nUART0 configured\n");
	//ConfigureI2C();
	//UARTprintf("I2C configured\n");
	HCSR04_init();
	UARTprintf("HC-SR04 initialized\n");
	ConfigurePORTFLEDs();
	UARTprintf("Tiva LEDs configured\n");
	LCD_init();
	UARTprintf("LCD initialized\n");
	BLE_init();
	UARTprintf("BLE initialized\n");
	UARTprintf("----------\n\n");
	
//	I2C0_MSA_R = VL53L0X_ADDRESS;												// Set slave address in transmit mode in write mode
//	I2C0_MDR_R = 0x0C0;																	// Set data and start transmission
//	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
//	
//	while (I2C0_MCS_R & I2C_MCS_BUSY)										// Check if master is busy
//	{
//	}
//	
//	if (!(I2C0_MCS_R & I2C_MCS_DATACK))
//	{
//		UARTprintf("Slave data ACK received\n");
//	}
//	else
//	{
//		UARTprintf("Error receiving slave data ACK\n");
//	}
//	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
//	{
//		UARTprintf("Slave address ACK received\n");
//	}
//	else
//	{
//		UARTprintf("Error receiving slave address ACK\n");
//	}
//	
//	I2C0_MSA_R = VL53L0X_ADDRESS + 1;										// Set slave address in transmit mode in read mode
//	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
//	
//	while (I2C0_MCS_R & I2C_MCS_BUSY)										// Check if master is busy
//	{
//	}
//	
//	if (!(I2C0_MCS_R & I2C_MCS_DATACK))
//	{
//		UARTprintf("Slave data ACK received\n");
//	}
//	else
//	{
//		UARTprintf("Error receiving slave data ACK\n");
//	}
//	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
//	{
//		UARTprintf("Slave address ACK received\n");
//	}
//	else
//	{
//		UARTprintf("Error receiving slave address ACK\n");
//	}
//	
//	UARTprintf("Received: %d\n", I2C0_MDR_R); 
	
	//GPIO_PORTE_DATA_R |= GPIO_PIN_0 | GPIO_PIN_4;				// Turn dc motor to a known position (0-4)
		
	while (1)																						// Lowest priority is the dc motor commutation loop. Interrupted by any other communications.
	{																										// Placeholder pin names for hall effect sensor input and motor driver outputs
	}
//		switch (GPIO_PORTF_DATA_R |= GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
//		{
//			case 0:																					// 6 step commutation cycle
//			{
//				GPIO_PORTE_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_0;							// (0-4)
//				break;
//			}
//			case 1:
//			{
//				GPIO_PORTE_DATA_R &= ~GPIO_PIN_3 & ~GPIO_PIN_4 & ~GPIO_PIN_5;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_5;							// (0-5)
//				break;
//			}
//			case 2:
//			{
//				GPIO_PORTE_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_1;							// (1-5)
//				break;
//			}
//			case 3:
//			{
//				GPIO_PORTE_DATA_R &=~GPIO_PIN_3 & ~GPIO_PIN_4 & ~GPIO_PIN_5;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_3;							// (1-3)
//				break;
//			}
//			case 4:
//			{
//				GPIO_PORTE_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_2;							// (2-3)
//				break;
//			}
//			case 5:
//			{
//				GPIO_PORTE_DATA_R &=~GPIO_PIN_3 & ~GPIO_PIN_4 & ~GPIO_PIN_5;
//				GPIO_PORTE_DATA_R |= GPIO_PIN_4;							// (2-4)
//				break;
//			}
//			default:																				// Unknown state, turn off all outputs
//			{
//				GPIO_PORTE_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_4 & ~GPIO_PIN_5;
//			}
//		}
	}
//	for (int i = 0; i < 0xFE; i++)
//	{
	//	UARTprintf("Sending register address for read\n");
	//	I2C0_MDR_R = 0xC0;
	//	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
	//	
	//	while (I2C0_MCS_R & I2C_MCS_BUSBSY);														// Fix this maybe
	//	
	//	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
	//	{
	//		UARTprintf("Slave address ACK received\n");
	//	}
	//	else
	//	{
	//		UARTprintf("Error receiving slave address ACK\n");
	//	}
	//	
	//	UARTprintf("Sending read request\n");
	//	I2C0_MSA_R = VL53L0X_ADDRESS + 1;
	//	I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
	//	
	//	while (I2C0_MCS_R & I2C_MCS_BUSBSY);														// Fix this maybe
	//	
	//	if (!(I2C0_MCS_R & I2C_MCS_ADRACK))
	//	{
	//		UARTprintf("Slave address ACK received\n");
	//		break;
	//	}
	//	else
	//	{
	//		UARTprintf("Error receiving slave address ACK\n");
	//	}
//	}
	
//	char buffer[128] = "";
//	char receiveChar;
//	uint8_t i = 0;
	
//	while (1)
//	{	
//		while (UART1_FR_R & UART_FR_RXFE);
		
//		receiveChar = UART1_DR_R;
		
//		if ((receiveChar == '%') || (receiveChar == '\r'))
//		{
//			UARTprintf("< %s\n", buffer);
//			strcpy(buffer, "");
//		}
//		else
//		{
//			strcat(buffer, &receiveChar);
//			i++;
//		}
//	}
//}


