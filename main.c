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

bool DEBOUNCE_FLAG = 0;

void ConfigureUART(void)
{
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;								// Enable UART0 module.
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;						// Enable GPIOA module.

  GPIOPinConfigure(GPIO_PA0_U0RX);										// Configure GPIO pins for UART mode.
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);		// Use the internal 16MHz oscillator as the UART clock source.

  UARTStdioConfig(0, 115200, 16000000);								// Initialize the UART for console I/O.
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1;								// Set up UART1 module.
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
	UART1_CTL_R &= ~UART_CTL_UARTEN;										// Disable UART1 for configuration.
	
	GPIO_PORTB_LOCK_R = 0x4C4F434B;											// Unlock PORTB.
	GPIO_PORTB_CR_R = 0xFF;															// Allow changes to PORTB
	
	GPIO_PORTB_AMSEL_R = 0x00;
	GPIO_PORTB_DEN_R |= GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_0 | GPIO_PIN_1;			// Set PORTB0 and PORTB1 to alternate function.
	GPIO_PORTB_DR2R_R |= GPIO_PIN_0 | GPIO_PIN_1;				// Set drive strengh to 2mA
	GPIO_PORTB_PCTL_R |= 0x00000011;
	
	UART1_IBRD_R = 8;																		// Set baud rate to 115200 with 16MHz clock
	UART1_FBRD_R = 54;
	UART1_LCRH_R = UART_LCRH_WLEN_8;										// Set data length to 8 bits.
	UART1_LCRH_R |= UART_LCRH_FEN;											// Enable FIFO.
	UART1_CC_R = UART_CC_CS_PIOSC;											// Set clock source to PIOSC.
	UART1_CTL_R |= UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;
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
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_2 | GPIO_PIN_3;			// Enable alternate function for PORTB2 and PORTB3
	GPIO_PORTB_ODR_R |=  GPIO_PIN_3;										// Enable open drain for PORTB3 - I2C0SDA
	GPIO_PORTB_DEN_R |= GPIO_PIN_2 | GPIO_PIN_3;				// Enable digital I/O on PORTB2 and PORTB3
	GPIO_PORTB_PCTL_R |= 0x00003300;											// Configure PMC for PORTB2 and PORTB3
	
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

void ConfigurePORTFInterrupts()
{
	GPIO_PORTF_IM_R &= ~0x11;														// Disable interrupts on PORTF0 and PORTF4
	GPIO_PORTF_IS_R &= ~0x11;														// PORTF0 and PORTF4 configured edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11;													// PORTF0 and PORTF4 generation controlled by IEV register
	GPIO_PORTF_IEV_R &= ~0x11;													// PORTF0 and PORTF4 configured falling edge
	GPIO_PORTF_RIS_R = 0x00;														// Clear interrupt flag
	NVIC_PRI7_R &= ~0x00E00000;
	NVIC_EN0_R |= 0x40000000;														// Enable interrupt 30 in NVIC (GPIOF)
	GPIO_PORTF_IM_R |= 0x11;														// Enable interrupts on PORTF0 and PORTF4
}

void Timer0_Init(unsigned long period)
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;							// Activate TIMER0 module
	delay = SYSCTL_RCGC1_R;															// Dummy read
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN;										// Disable TIMER0 during setup
	TIMER0_CFG_R = 0x00000000;													// Configure for 32-bit mode
	TIMER0_TAMR_R |= 0x00000001;												// Configure for one-shot mode
	TIMER0_TAMR_R &= ~TIMER_TAMR_TACDIR;								// Configure for count down mode
	TIMER0_TAILR_R = period - 1;												// Load start value
	NVIC_PRI4_R &= ~0xE0000000; 	 											// configure Timer0 interrupt priority as 0
	NVIC_EN0_R |= 0x00080000;     											// enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= TIMER_IMR_TATOIM;										// enable time out interrupt mask
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
	ConfigurePORTFInterrupts();
	UARTprintf("Interrupts configured\n");
	Timer0_Init(0xEFFFFF);
	UARTprintf("Timers configured\n");
	LCD_init();
	UARTprintf("LCD initialized\n");
	BLE_init();
	UARTprintf("BLE initialized\n");
	
	UARTprintf("----------\n\n");
	
	//
	// Turn off on board RGB LED.
	//
	GPIO_PORTF_DATA_R &= ~GPIO_PIN_1 & ~GPIO_PIN_2 & ~GPIO_PIN_3;

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
//	}
//	else
//	{
//		UARTprintf("Error receiving slave address ACK\n");
//	}
	
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
	
	char buffer[128] = "";
	char receiveChar;
	uint8_t i = 0;
	
	while (1)
	{	
		while (UART1_FR_R & UART_FR_RXFE);
		
		receiveChar = UART1_DR_R;
		
		if ((receiveChar == '%') || (receiveChar == '\r'))
		{
			UARTprintf("< %s\n", buffer);
			strcpy(buffer, "");
		}
		else
		{
			strcat(buffer, &receiveChar);
			i++;
		}
	}
}

void GPIOF_Handler(void)
{
	GPIO_PORTF_ICR_R |= 0x11;
	
	if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4) && (DEBOUNCE_FLAG == 0))
	{
		BLE_command("$$$");
		DEBOUNCE_FLAG = 1;
		GPIO_PORTF_IM_R &= ~0x11;
		TIMER0_CTL_R |= TIMER_CTL_TAEN;
	}
	else if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0) && (DEBOUNCE_FLAG == 0))
	{
		BLE_command("");
		DEBOUNCE_FLAG = 1;
		GPIO_PORTF_IM_R &= ~0x11;
		TIMER0_CTL_R |= TIMER_CTL_TAEN;
	}
}

void TIMER0A_Handler(void)
{
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;									// Set acknowlegement flag for TIMER0
	
	DEBOUNCE_FLAG = 0;																	// Clear debounce
	GPIO_PORTF_IM_R |= 0x11;
}
