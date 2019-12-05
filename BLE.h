// Header file for RN4871 BLE module
//
// Uses the following pins of PORTB:
// PORTB0		-		UART1 RX
// PORTB1 	- 	UART1 TX

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "utils/UARTstdio.h"

bool DEBOUNCE_FLAG = 0;
char buffer[128] = "";
char RXChar;
uint8_t i = 0;

void BLE_init(void);
void BLE_command(const char* pCommand);

void ConfigureUART1(void);
void ConfigurePORTF(void);
void ConfigurePORTFInterrupts(void);
void Timer0_init(unsigned long period);

void ConfigureUART1(void)
{
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
	UART1_IM_R |= UART_IM_TXIM | UART_IM_RXIM;					// Enable FIFO interruptps for RX and TX.
	UART1_ICR_R = 0x00;																	// Clear interrupt flags
	NVIC_PRI1_R &= ~0x00800000;													// Set interrupt 6 priority to level 3.
	NVIC_PRI1_R |= 0x00600000;
	NVIC_EN0_R |= 0x00000040;														// Enable interrupt 6 in NVIC (UART1)
	UART1_CTL_R &= UART_CTL_TXE;												// Disable TX until FIFO interrupt.
	UART1_CTL_R |= UART_CTL_RXE | UART_CTL_UARTEN;			// Enable UART1 RX.
}

void ConfigurePORTF()
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;								// Activate clock for PORTF
	delay = SYSCTL_RCGC2_R;															// Dummy read to allow clock to activate
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;											// Unlock PORTF
	GPIO_PORTF_CR_R = 0xFF;															// Allow changes to PORTF
	
	GPIO_PORTF_AMSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;		// Disable analog on the two onboard pushbuttons.
	GPIO_PORTF_PCTL_R  = 0x00000000;										// PTCL GPIO on PORTF
	GPIO_PORTF_DIR_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;			// SET PORTF0 & PORTF4 as inputs (SW1 and SW2)
	GPIO_PORTF_AFSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_4;		// Disable alternate function on PORTF
	GPIO_PORTF_PUR_R |= GPIO_PIN_0 | GPIO_PIN_4;				// Enable PUR on PORTF0 and PORTF4
	GPIO_PORTF_DEN_R |= GPIO_PIN_0 | GPIO_PIN_4;				// Enable digital I/O on PORTF
}

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

void BLE_init()
{
	ConfigureUART1();
	ConfigurePORTF();
	ConfigurePORTFInterrupts();
	Timer0_Init(0xEFFFFF);
}

void BLE_command(const char* pCommand)
{
	
	for (uint8_t i = 0; i < strlen(pCommand); i++)
	{
		UART1_DR_R = pCommand[i];
	}
	
	UART1_DR_R = '\r';
	
	UARTprintf("> %s\n", pCommand);
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
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;									// Set acknowlegement flag for TIMER0.
	
	DEBOUNCE_FLAG = 0;																	// Clear debounce.
	GPIO_PORTF_IM_R |= 0x11;
}

void UART1_Handler(void)
{
	if (UART1_MIS_R & UART_MIS_TXMIS)
	{
		UART1_ICR_R = 0x00;
		UART1_CTL_R |= UART_CTL_TXE;											// Enable TX.
		
		while (!(UART1_FR_R & UART_FR_TXFE));							// Wait until TX FIFO buffer is empty.
		
		UART1_CTL_R &= ~UART_CTL_TXE;											// Disable TX until next interrupt
	}
}
