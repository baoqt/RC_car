// Header file for RN4871 BLE module
//
// Uses the following pins of PORTB:
// PORTB0		-		UART1 RX
// PORTB1 	- 	UART1 TX
//
// Uses TIMER0 to timeout on getting UART1 RX
// Uses TIMER1 to periodically forward UART1 to UART0 for debugging
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

bool TIMEOUT_COUNTER = 0;															// Flag used to timeout while waiting for RX.
bool DEBOUNCE_FLAG = 0;

char TXbuffer[256];																		// UART1 -> UART0 forwarding buffer.
char RXbuffer[256];																		// UART0 -> UART1 forwarding buffer.
uint8_t wordIndex = 0;																// Word index.
uint8_t charIndex = 0;																// Char index.
uint8_t bufferLower = 0;															// Buffer word lower index pointer.

void BLE_init(void);																	// Configured required modules.
void BLE_command(const char* pCommand);								// Sends a Bluetooth ASCII command over UART1.
void BLE_flush(void);																	// Flushes out UART1 RX FIFO buffer.
void BLE_forward(void);																// Forwards UART1 RX -> UART0 TX for COM port debugging.
int BLE_wait_for_RX(void);														// Gets UART1 RX, uses TIMER0 as a timeout counter.

void ConfigureUART1(void);														// Configures UART1 module for Bluetooth communication.
void Timer0_init(unsigned long period);								// Configures timeout for waiting on RX.
void Timer1_init(unsigned long period);								// Configures UART1 buffer to UART0 COM terminal forwarding.

void ConfigureUART1(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1;								// Enable UART1 module.
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;								// Enable PORTB module
	delay = SYSCTL_RCGC2_R;
	UART1_CTL_R &= ~UART_CTL_UARTEN;										// Disable UART1 for configuration.
	
	GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_0 & ~GPIO_PIN_1;
	GPIO_PORTB_DEN_R |= GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX;		// DEBUG: Check if UART1 is still transmitting after this change.
	GPIO_PORTB_AFSEL_R |= GPIO_PIN_0 | GPIO_PIN_1;			// Set PORTB0 and PORTB1 to alternate function.
	
	
	UART1_IBRD_R = 8;																		// Set baud rate to 115200 with 16MHz clock
	UART1_FBRD_R = 54;
	UART1_LCRH_R = UART_LCRH_WLEN_8;										// Set data length to 8 bits.
	UART1_LCRH_R |= UART_LCRH_FEN;											// Enable FIFO. 
	UART1_CC_R = UART_CC_CS_PIOSC;											// Set clock source to PIOSC.
	UART1_ICR_R = 0x00;																	// Clear interrupt flags
	
	NVIC_PRI1_R &= ~0x00800000;													// Set interrupt 6 priority to level 3.
	NVIC_PRI1_R |= 0x00600000;
	NVIC_EN0_R |= 0x00000040;														// Enable interrupt 6 in NVIC (UART1)
	UART1_IM_R |= UART_IM_TXIM | UART_IM_RXIM;					// Enable FIFO interruptps for RX and TX.
	UART1_CTL_R |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;			// Enable UART1 RX.
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

void Timer1_Init(unsigned long period)
{
	volatile unsigned long delay;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER1;
	delay = SYSCTL_RCGC1_R;
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
	TIMER1_CFG_R = 0x00000000;
	TIMER1_TAMR_R |= 0x00000002;
	TIMER1_TAMR_R &= ~TIMER_TAMR_TACDIR;
	TIMER1_TAILR_R = period - 1;
	NVIC_PRI5_R &= ~0x0000E000;
	NVIC_PRI5_R |= 0x00006000;
	NVIC_EN0_R |= 0x00200000;
	TIMER1_IMR_R |= TIMER_IMR_TATOIM;
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void BLE_init()
{
	bool config = 1;
	
	ConfigureUART1();
	Timer0_Init(0x2FFFFFF);
	Timer1_Init(0x8FFFFFF);
	UARTprintf("----------\n");
	UARTprintf("Configuring BLE service profile...\n< ");
	config &= BLE_wait_for_RX();
	BLE_command("$$$");
	config &= BLE_wait_for_RX();
	BLE_command("SR,0100");
	config &= BLE_wait_for_RX();
	BLE_command("SS,C0");
	config &= BLE_wait_for_RX();
	if (config)
	{
		UARTprintf("\nBLE configuration successful.\n");
	}
	else
	{
		UARTprintf("\nBLE configuration failed.\n");
	}
	UARTprintf("----------\n");
}

void BLE_command(const char* pCommand)
{
	
	for (uint8_t i = 0; i < strlen(pCommand); i++)
	{
		UART1_DR_R = pCommand[i];
	}
	
	if (strcmp(pCommand, "$$$") != 0)
	{
		UART1_DR_R = '\r';
	}
	
	UARTprintf("%s\n< ", pCommand);
}

void BLE_flush(void)
{
	while (!(UART1_FR_R & UART_FR_RXFE))								// Flush out any remaining characters in UART buffer into burst buffer
		{
			char RXChar = UART1_DR_R;
			
			if ((RXChar == '%') || (RXChar == '\n'))				// Detected string delimiter.
			{
				strcat(TXbuffer, "\n< ");											// Format RX messages.
			}
			else if (RXChar == '>')
			{
				strcat(TXbuffer, "\n>");
			}
			else
			{																								// Add received char to buffer.
				strcat(TXbuffer, &RXChar);
			}
		}
}

void BLE_forward(void)
{
	if (strcmp(TXbuffer, "") != 0)											// If burst buffer is not empty
	{
		UARTprintf(TXbuffer);															// Burst forward UART1 -> UART0
		strcpy(TXbuffer, "");															// Clear burst buffer.
	}
}

int BLE_wait_for_RX(void)															// Wait on an RX with a timeout.
{
	TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;									// Disable periodic flush and forward interrupt.
	UART1_IM_R &= ~UART_IM_RXIM;												// Disable RX FIFO interrupt.
	TIMEOUT_COUNTER = 0;
	TIMER0_CTL_R |= TIMER_CTL_TAEN;											// Start timeout counter.
	
	while (UART1_FR_R & UART_FR_RXFE)										// Wait until RX starts.
	{
		if (TIMEOUT_COUNTER)
		{
			return 0;																				// Timeout.
		}
	}
	
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
	TIMER0_TAV_R = TIMER0_TAILR_R;
	TIMER0_CTL_R |= TIMER_CTL_TAEN;
	
	while (!(UART1_FR_R & UART_FR_RXFE))								// Wait until RX finishes.
	{
		BLE_flush();																			// Flush FIFO buffer.
		SysCtlDelay(50000);																// Wait for any follow up RX.
		
		if (TIMEOUT_COUNTER)
		{
			return 0;																				// Timeout.
		}
	}
	
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
	TIMER0_TAV_R = TIMER0_TAILR_R;
	
	BLE_forward();																			// Forward UART1 -> UART0.
	UART1_IM_R |= UART_IM_RXIM;													// Reenable RX FIFO interrupt.
	TIMER1_IMR_R |= TIMER_IMR_TATOIM;										// Reenable periodic flush and forward interrupt.
	
	return 1;																						// Successful RX.
}



void TIMER0A_Handler(void)
{
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;									// Set acknowlegement flag for TIMER0.
	
	TIMEOUT_COUNTER = 1;																// Timeout triggered.
//	DEBOUNCE_FLAG = 0;
//	GPIO_PORTF_IM_R |= 0x11;
}

void TIMER1A_Handler(void)
{
	TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
	
	BLE_flush();
	BLE_forward();
	
//	UARTgets(RXbuffer, 256);
//	
//	if ((strchr(RXbuffer, '\n') != NULL) || (strstr(RXbuffer, "$$$") != NULL))
//	{
//		BLE_command(RXbuffer);
//		strcpy(RXbuffer, "");
//	}
}

void UART1_Handler(void)
{
	if (UART1_MIS_R & UART_MIS_TXMIS)										// TX interrupt.
	{
		UART1_ICR_R |= UART_ICR_TXIC;											// Clear TX interrupt flag.
	}
	if (UART1_MIS_R & UART_MIS_RXMIS)										// RX interrupt.
	{
		UART1_ICR_R = UART_ICR_RXIC;											// Clear RX interrupt flag.
		TIMER1_CTL_R &= ~TIMER_CTL_TAEN;									// Disable UART forwarding interrupts.
		
		BLE_flush();
		
		TIMER1_CTL_R |= TIMER_CTL_TAEN;										// Reenable UART forwarding interrupts.
	}
}
