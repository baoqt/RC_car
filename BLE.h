// Header file for RN4871 BLE module
//
// Uses the following pins of PORTB:
// PORTB0		-		UART1 RX
// PORTB1 	- 	UART1 TX

#include <stdint.h>
#include <string.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"

#include "utils/UARTstdio.h"

void BLE_init(void);
void BLE_command(const char* pCommand);

void BLE_init()
{
//	BLE_command("GK");
//	SysCtlDelay(1000000);
//	BLE_command("GNR");
//	SysCtlDelay(1000000);
//	BLE_command("C");
//	SysCtlDelay(1000000);
//	BLE_command("M");
//	SysCtlDelay(1000000);
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
