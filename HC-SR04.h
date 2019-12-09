// Header file for HC-SR04 ultrasonic distance sensor module
//
// Uses the following pins of PORTA:
// PORTA6		-		TRIG
// PORTB7 	- 	ECHO

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
