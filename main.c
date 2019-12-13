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

volatile unsigned long DELAY ;
uint8_t STATE;

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

void ConfigurePORTFLEDs()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;								// Activate clock for PORTF
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;											// Unlock PORTF
	GPIO_PORTF_CR_R = 0xFF;															// Allow changes to PORTF
	
	// Disable analog
	GPIO_PORTF_AMSEL_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	// PTCL GPIO on PORTF
	GPIO_PORTF_PCTL_R  &= ~GPIO_PCTL_PF1_M & ~GPIO_PCTL_PF2_M & ~GPIO_PCTL_PF3_M;
	// Set PORTF1-PORTF3 as outputs (RGB LEDs)
	GPIO_PORTF_DIR_R |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	// Disable alternate function on PORTF
	GPIO_PORTF_AFSEL_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	// Enable digital I/O on PORTF
	GPIO_PORTF_DEN_R |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	// Turn off LEDs
	GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
}

void BLDC_init()
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
	
	GPIO_PORTE_AMSEL_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	GPIO_PORTE_PCTL_R &= ~GPIO_PCTL_PE1_M & ~GPIO_PCTL_PE2_M & ~GPIO_PCTL_PE3_M;
	GPIO_PORTE_DIR_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	GPIO_PORTE_AFSEL_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
	GPIO_PORTE_DEN_R |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	
	GPIO_PORTD_LOCK_R = 0x4C4F434B;
	GPIO_PORTD_CR_R = 0xFF;
	
	GPIO_PORTD_AMSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
	GPIO_PORTD_DIR_R |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_PORTD_AFSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
	GPIO_PORTD_DEN_R |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
}

void Configure_TIMER2(unsigned long period)
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER2;							// Activate TIMER2 module
	delay = SYSCTL_RCGC1_R;															// Dummy read
	
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN;										// Disable TIMER2 during setup
	TIMER2_CFG_R = 0x00000000;													// Configure for 32-bit mode
	TIMER2_TAMR_R |= 0x00000002;												// Configure for periodic mode
	TIMER2_TAMR_R &= ~TIMER_TAMR_TACDIR;								// Configure for count down mode
	TIMER2_TAILR_R = period - 1;												// Load start value
	NVIC_PRI5_R &= ~0xE0000000; 	 											// configure Timer2 interrupt priority as 0
	NVIC_EN0_R |= 0x00800000;     											// enable interrupt 23 in NVIC (Timer2A)
	TIMER2_IMR_R |= TIMER_IMR_TATOIM;										// Clear interrupt flag.
	
	TIMER2_CTL_R |= TIMER_CTL_TAEN;											// Enable TIMER2.
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
	VL53L0X_init();
	UARTprintf("VL53L0X initialized\n");
	HCSR04_init();
	UARTprintf("HC-SR04 initialized\n");
	ConfigurePORTFLEDs();
	UARTprintf("Tiva LEDs configured\n");
	LCD_init();
	UARTprintf("LCD initialized\n");
	BLE_init();
	UARTprintf("BLE initialized\n");
	BLDC_init();
	UARTprintf("BLDC initialized\n");
	Configure_TIMER2(0x4000000);												// Adjust this period to change distance polling frequency.
	UARTprintf("TIMER2 configured\n");
	UARTprintf("----------\n");
	Test_I2C0_Connection();
	
	//GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_3;				// Turn dc motor to a known position (0-4)
		
	while (1)																						// Lowest priority is the dc motor commutation loop. Interrupted by any other communications.
	{																										// Placeholder pin names for hall effect sensor input and motor driver outputs
		if (DELAY > 0x0000FFFF)
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1;
			STATE = (GPIO_PORTE_DATA_R & (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)) >> 1;
			switch (STATE)
			{
				case 1:																					// 6 step commutation cycle
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_2 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
					GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_3;	// (1-3)
					break;
				}
				case 3:
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
					GPIO_PORTD_DATA_R |= GPIO_PIN_2 | GPIO_PIN_3;	// (2-3)
					break;
				}
				case 2:
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_3 & ~GPIO_PIN_7;
					GPIO_PORTD_DATA_R |= GPIO_PIN_2 | GPIO_PIN_6;	// (2-6)
					break;
				}
				case 6:
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_7;
					GPIO_PORTD_DATA_R |= GPIO_PIN_0 | GPIO_PIN_6;	// (0-6)
					break;
				}
				case 4:
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6;
					GPIO_PORTD_DATA_R |= GPIO_PIN_0 | GPIO_PIN_7;	// (0-7)
					break;
				}
				case 5:
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6;
					GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_7;	// (1-7)
					break;
				}
				default:																				// Unknown state, turn off all outputs
				{
					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
				}
			}
		}
		else
		{
			GPIO_PORTF_DATA_R |= GPIO_PIN_1;
		}
	}
}

void TIMER2A_Handler(void)
{
	DELAY = HCSR04_Get_Distance();
//	UARTprintf("Delay: %04X.%04X\n", ((0xFFFF0000 & DELAY) >> 16), (0x0000FFFF & DELAY));
//	UARTprintf("State: %d\n", STATE);
//	UARTprintf("Upper: %d\n", (GPIO_PORTD_DATA_R & (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)));
//	UARTprintf("Lower: %d\n\n", ((GPIO_PORTD_DATA_R & (GPIO_PIN_6 | GPIO_PIN_7)) >> 5) | (GPIO_PORTD_DATA_R & GPIO_PIN_3) >> 2);
}
