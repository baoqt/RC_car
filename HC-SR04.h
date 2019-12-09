// Header file for HC-SR04 ultrasonic distance sensor module
//
// Uses the following pins of PORTA:
// PORTA6		-		TRIG
// PORTB7 	- 	ECHO
//
// Uses TIMER2 to set distance polling frequency
// Uses TIMER3 to get measurement value
////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "utils/UARTstdio.h"

unsigned long DELAY;

void HCSR04_init(void);

void Configure_PORTA(void);
void Configure_TIMER2(unsigned long period);
void Configure_TIMER3(void);

void Configure_PORTA()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;								// Activate clock for PORTA
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTA_AMSEL_R &= ~GPIO_PIN_6 & ~GPIO_PIN_7;		// Disable analog on the two onboard pushbuttons.
	GPIO_PORTA_PCTL_R &= ~0x11000000;										// PTCL GPIO on PORTF
	GPIO_PORTA_DIR_R &= ~GPIO_PIN_7;										// SET PORTF7 as input (ECHO)
	GPIO_PORTA_DIR_R |= GPIO_PIN_6;											// Set PORTF6 as output (TRIG)
	GPIO_PORTA_AFSEL_R &= ~GPIO_PIN_6 & ~GPIO_PIN_7;		// Disable alternate function on PORTA
	GPIO_PORTA_PUR_R |= GPIO_PIN_6;											// Enable open drain on PORTA6 (TRIG)
	GPIO_PORTA_DEN_R |= GPIO_PIN_6 | GPIO_PIN_7;				// Enable digital output on PORTA6 and PORTA7
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

void Configure_TIMER3()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER3;							// Activate TIMER3 module.
	delay = SYSCTL_RCGC1_R;															// Dummy read.
	
	TIMER3_CTL_R &= ~TIMER_CTL_TAEN;										// Disable TIMER3 during setup.
	TIMER3_CFG_R = 0x00000000;													// Configure for 32-bit mode.
	TIMER3_TAMR_R |= 0x00000001;												// Configure for one-shot mode.
	TIMER3_TAMR_R |= TIMER_TAMR_TACDIR;									// Configure for count up mode.
	TIMER3_TAILR_R = 0xFFFFFFFF;												// Load upper bound value.
}

void HCSR04_init()
{
	Configure_PORTA();
	Configure_TIMER2(0x4000000);												// Adjust this period to change distance polling frequency.
	Configure_TIMER3();
}

void TIMER2A_Handler(void)
{
	TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
	
	GPIO_PORTA_DATA_R &= ~GPIO_PIN_6;										// Lower TRIG, preparing for a measurement.
	TIMER3_CTL_R |= TIMER_CTL_TAEN;											// Enable TIMER3 to count pulsewidth.
	
	SysCtlDelay(4000);																	// Short delay to allow for sensor latency.
	while (GPIO_PORTA_DATA_R & GPIO_PIN_7);							// Wait until measurement is finished.
	
	TIMER3_CTL_R &= ~TIMER_CTL_TAEN;										// Disable TIMER3 to read delay.
	DELAY = TIMER3_TAV_R;																// Get delay value.
	TIMER3_TAV_R = 0;																		// Reset TIMER3's value.
	GPIO_PORTA_DATA_R |= GPIO_PIN_6;										// Raise TRIG, ending the measurement.
	
	UARTprintf("Delay: %04X.%04X\n", ((0xFFFF0000 & DELAY) >> 16), (0x0000FFFF & DELAY));
}
