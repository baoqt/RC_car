////////////////////////////////////////////////////////////
//	Main file for the BLE controlled RC car project.
// 
//	Contains function calls to initialization and configuration
//	to the specific hardware modules from the included header
//	files.
//
//	Main loops checks the periodically fetched measurement
//	from the distance sensor and disables the PWM driving the
//	main servo motor.
//
//	Uses TIMER2 to periodically get the distance measurement
//	as well as pop off any BLE received commands from the command
//	stack.
//
//	BLDC commutation loop is working on the TIVA side, but
//	driver board is unresponsive. Hall effect sensor inputs
//	and gate high and low outputs follow expected waveforms.
////////////////////////////////////////////////////////////
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
#include "driverlib/pwm.h"

#include "utils/uartstdio.h"
#include "LCD.h"
#include "BLE.h"
#include "VL53L0X.h"
#include "HC-SR04.h"

#define LENGTH 255

volatile unsigned long DELAY ;
uint8_t STATE = 0;
uint8_t CMD = 0;
char CMD_BUFFER[LENGTH];
bool DEBOUNCE = false;

////////////////////////////////////////////////////////////
//	Configures the UART0 module, used to send debugging messages
//	through the TIVA's evaluation board USB cable onto a serial
//	COM port.
//
//	Uses the following pins of PORTA:
//	PORTA0		-		UART0 RX
//	PORTA1		-		UART0 TX
////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////
//	Configures the TIVA's onboard push buttons; PB0 and PB1.
//
//	Uses the following pins of PORTF:
//	PORTF0		-		PB0
//	PORTF4		-		PB1
//
//	Uses internal pull up resistors so a logic high corresponds
//	with 0V and a logic low corresponds with 3V3.
////////////////////////////////////////////////////////////
void ConfigurePORTFPushButtons()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;								// Activate clock for PORTF
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;											// Unlock PORTF
	GPIO_PORTF_CR_R = 0xFF;															// Allow changes to PORTF
	
	GPIO_PORTF_AMSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;		// Disable analog on the two onboard pushbuttons.
	GPIO_PORTF_PCTL_R = 0x00000000;											// PTCL GPIO on PORTF
	GPIO_PORTF_DIR_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;			// SET PORTF0 & PORTF4 as inputs (SW1 and SW2)
	GPIO_PORTF_AFSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;		// Disable alternate function on PORTF
	GPIO_PORTF_PUR_R |= GPIO_PIN_0 | GPIO_PIN_4;				// Enable PUR on PORTF0 and PORTF4
	GPIO_PORTF_DEN_R |= GPIO_PIN_0 | GPIO_PIN_4;				// Enable digital I/O on PORTF
}

////////////////////////////////////////////////////////////
//	Configures interrupts that trigger on a negative edge
//	on the two previously mentioned buttons.
////////////////////////////////////////////////////////////
void ConfigurePORTFInterrupts()
{
	GPIO_PORTF_IM_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;				// Disable interrupts on PORTF0 and PORTF4
	GPIO_PORTF_IS_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;				// PORTF0 and PORTF4 configured edge sensitive
	GPIO_PORTF_IBE_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;			// PORTF0 and PORTF4 generation controlled by IEV register
	GPIO_PORTF_IEV_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;			// PORTF0 and PORTF4 configured falling edge
	GPIO_PORTF_ICR_R = 0x00;														// Clear interrupt flag
	NVIC_PRI7_R &= ~0x00E00000;													// Set interrupt 30 to priority 0.
	NVIC_EN0_R |= 0x40000000;														// Enable interrupt 30 in NVIC (GPIOF)
	GPIO_PORTF_IM_R |= 0x11;														// Enable interrupts on PORTF0 and PORTF4
}

////////////////////////////////////////////////////////////
//	Configures the TIVA's onboard RGB LED.
//
//	Uses the following pins of PORTF:
//	PORTF1		-		RED
//	PORTF2		-		BLUE
//	PORTF3		-		GREEN
////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////
//	Configures the necessary pins for a simple BLDC motor driver.
//
//	Uses the following pins of PORTD:
//	PORTD0		-		Gate A high
//	PORTD1		-		Gate B high
//	PORTD2		-		Gate C high
//	PORTD3		-		Gate A low
//	PORTD6		-		Gate B low
//	PORTD7		-		Gate C low
//
//	Uses the following pins of PORTE:
//	PORTE1		-		Hall effect sensor A
//	PORTE2		-		Hall effect sensor B
//	PORTE3		-		Hall effect sensor C
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

////////////////////////////////////////////////////////////
//	Configures the necessary pins and modules for a continuous
//	rotation servo control.
//
//	Uses the PWM0 module
//
//	Uses the following pins of PORTB:
//	PORTB6		-		PWM0_0 output
////////////////////////////////////////////////////////////
void Servo_init()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;
	delay = SYSCTL_RCGC2_R;
	
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	
	GPIO_PORTB_LOCK_R = 0x4C4F4e4B;
	GPIO_PORTB_CR_R = 0xFF;

	GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_6;
	GPIO_PORTB_DEN_R |= GPIO_PIN_6;
	GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0;
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_6;
	
	PWM0_0_CTL_R &= ~PWM_0_CTL_ENABLE;
	PWM0_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
	PWM0_0_LOAD_R = 0x000003FF;
	PWM0_0_CMPA_R = 0x0000000E;
	PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;
}

////////////////////////////////////////////////////////////
//	Configures TIMER2 to periodically get a distance measurement
//	from the ultrasonic distance sensor as well as pop off a
//	BLE transmitted motor control command if any exists.
////////////////////////////////////////////////////////////
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
	UARTprintf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	UARTprintf("----------\nUART0 configured\n");
	//BLE_init(CMD_BUFFER, LENGTH);
	//UARTprintf("BLE initialized\n");
	//VL53L0X_init();
	//UARTprintf("VL53L0X initialized\n");
	HCSR04_init();
	UARTprintf("HC-SR04 initialized\n");
	ConfigurePORTFPushButtons();
	UARTprintf("Tiva push buttons configured\n");
	ConfigurePORTFInterrupts();
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;							// Activate TIMER0 module
	volatile unsigned long delay = SYSCTL_RCGC1_R;															// Dummy read
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN;										// Disable TIMER0 during setup
	TIMER0_CFG_R = 0x00000000;													// Configure for 32-bit mode
	TIMER0_TAMR_R |= 0x00000001;												// Configure for one-shot mode
	TIMER0_TAMR_R &= ~TIMER_TAMR_TACDIR;								// Configure for count down mode
	TIMER0_TAILR_R = 0xEFFFFF;													// Load start value
	NVIC_PRI4_R &= ~0xE0000000; 	 											// configure Timer0 interrupt priority as 0
	NVIC_EN0_R |= 0x00080000;     											// enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= TIMER_IMR_TATOIM;										// enable time out interrupt mask
	UARTprintf("PORTF interrupts configured\n");
	ConfigurePORTFLEDs();
	UARTprintf("Tiva LEDs configured\n");
	//LCD_init();
	//UARTprintf("LCD initialized\n");
	//BLDC_init();
	//UARTprintf("BLDC initialized\n");
	Servo_init();
	UARTprintf("Servo motor intialized\n");
	Configure_TIMER2(0x00FFFFFF);												// Adjust this period to change distance polling frequency.
	UARTprintf("TIMER2 configured\n");
	UARTprintf("----------\n");
	
	//GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_3;				// Turn dc motor to a known position (0-4)
		
	while (1)																						// Lowest priority is the dc motor commutation loop. Interrupted by any other communications.
	{																										// Placeholder pin names for hall effect sensor input and motor driver outputs	
		if (DELAY > 0x0000FFFF)
		{
			PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;
//			//GPIO_PORTF_DATA_R &= ~GPIO_PIN_1;
//			STATE = (GPIO_PORTE_DATA_R & (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)) >> 1;
//			switch (STATE)
//			{
//				case 1:																					// 6 step commutation cycle
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_2 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_3;	// (1-3)
//					break;
//				}
//				case 3:
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_2 | GPIO_PIN_3;	// (2-3)
//					break;
//				}
//				case 2:
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_3 & ~GPIO_PIN_7;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_2 | GPIO_PIN_6;	// (2-6)
//					break;
//				}
//				case 6:
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_7;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_0 | GPIO_PIN_6;	// (0-6)
//					break;
//				}
//				case 4:
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_0 | GPIO_PIN_7;	// (0-7)
//					break;
//				}
//				case 5:
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6;
//					GPIO_PORTD_DATA_R |= GPIO_PIN_1 | GPIO_PIN_7;	// (1-7)
//					break;
//				}
//				default:																				// Unknown state, turn off all outputs
//				{
//					GPIO_PORTD_DATA_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3 & ~GPIO_PIN_6 & ~GPIO_PIN_7;
//				}
//			}
		}
		else
		{
			PWM0_ENABLE_R &= ~PWM_ENABLE_PWM0EN;
		}
	}
}
void TIMER0A_Handler(void)
{
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;									// Set acknowlegement flag for TIMER0.
	GPIO_PORTF_IM_R |= 0x11;
	DEBOUNCE = false;																// Timeout triggered.
}

void GPIOF_Handler(void)
{
	GPIO_PORTF_ICR_R |= 0x11;
	
	if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4) && !DEBOUNCE)
	{
		DEBOUNCE = true;
		GPIO_PORTF_IM_R &= ~0x11;
		UARTprintf("ACC\n");
		strcat(CMD_BUFFER, "ACC|");
		TIMER0_CTL_R |= TIMER_CTL_TAEN;
//		BLE_command("$$$");
//		//BLE_wait_for_RX();
	}
	else if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0) && !DEBOUNCE)
	{
		DEBOUNCE = true;
		GPIO_PORTF_IM_R &= ~0x11;
		UARTprintf("DEC\n");
		strcat(CMD_BUFFER, "DEC|");
		TIMER0_CTL_R |= TIMER_CTL_TAEN;
//		BLE_command("B");
//		//BLE_wait_for_RX();
	}
}

void TIMER2A_Handler(void)
{
	DELAY = HCSR04_Get_Distance();
	
	if (strcmp(CMD_BUFFER, "") != 0)
	{
		if (strstr(CMD_BUFFER, "ACC|"))
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
			GPIO_PORTF_DATA_R |= GPIO_PIN_1;
			
			PWM0_0_CMPA_R = ((PWM0_0_CMPA_R << 1) < PWM0_0_LOAD_R) ? PWM0_0_CMPA_R << 1 : PWM0_0_CMPA_R;
			CMD = 1;
		}
		else if (strstr(CMD_BUFFER, "DEC|"))
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
			GPIO_PORTF_DATA_R |= GPIO_PIN_2;
			
			PWM0_0_CMPA_R = ((PWM0_0_CMPA_R >> 1) > 0) ? PWM0_0_CMPA_R >> 1 : PWM0_0_CMPA_R;
			CMD = 2;
		}
		else if (strstr(CMD_BUFFER, "LFT|"))
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
			GPIO_PORTF_DATA_R |= GPIO_PIN_3;
			CMD = 3;
		}
		else if (strstr(CMD_BUFFER, "RGT|"))
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
			GPIO_PORTF_DATA_R |= GPIO_PIN_1 | GPIO_PIN_2;
			CMD = 4;
		}
		else if (strstr(CMD_BUFFER, "ATO|"))
		{
			GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;
			GPIO_PORTF_DATA_R |= GPIO_PIN_2 | GPIO_PIN_3;
			CMD = 5;
		}
		else
		{
			CMD = 0;
		}
		strcpy(CMD_BUFFER, "");
	}
	
//	UARTprintf("Delay: %04X.%04X\n", ((0xFFFF0000 & DELAY) >> 16), (0x0000FFFF & DELAY));
//	UARTprintf("State: %d\n", STATE);
//	UARTprintf("Upper: %d\n", (GPIO_PORTD_DATA_R & (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)));
//	UARTprintf("Lower: %d\n\n", ((GPIO_PORTD_DATA_R & (GPIO_PIN_6 | GPIO_PIN_7)) >> 5) | (GPIO_PORTD_DATA_R & GPIO_PIN_3) >> 2);
}
