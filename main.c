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

#define PTCL					0x0000052C

#define PIN0          	0x00000001
#define PIN1          	0x00000002
#define PIN2          	0x00000004
#define PIN3          	0x00000008
#define PIN4          	0x00000010
#define PIN5          	0x00000020
#define PIN6          	0x00000040
#define PIN7          	0x00000080

#define VL53L0X_ADDRESS	0x00000052

//#ifdef DEBUG
//void __error__(char *pcFilename, uint32_t ui32Line)
//{
//}
//#endif

void ConfigureUART(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;								// Enable GPIOA module.
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;								// Enable UART0 module.

  GPIOPinConfigure(GPIO_PA0_U0RX);										// Configure GPIO pins for UART mode.
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);		// Use the internal 16MHz oscillator as the UART clock source.

  UARTStdioConfig(0, 115200, 16000000);								// Initialize the UART for console I/O.
}

void ConfigureI2C(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;								// Enable GPIOB module.
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;							// Enable I2C0 module.
	
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);									// Configure GPIO pins for I2C mode.
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	
	I2CMasterInitExpClk(VL53L0X_ADDRESS, 50000000, false);
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
	
	return 0;
}
