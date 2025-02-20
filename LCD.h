// Header file for NHD-C12832A1Z-NSW-BBW-3V3 LCD module
//
// Uses the following pins of PORTA:
// PORTA2		-		SCL
// PORTA5		-		STX
//
// Uses the following pins of PORTB:
// PORTB5		-		message mode, 0 = command, 1 = data (A0)
////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#define displayOn 0xAF
#define displayOff 0xAE
#define ADCSelectNormal 0xA0
#define ADCSelectReverse 0xA1
#define displaySelectNormal 0xA7
#define displaySelectReverse 0xA1
#define allPointsOn 0xA5
#define normalPoints 0xA4
#define biasSet 0xA2
#define modeSelectNormal 0xC0
#define controlSet 0x2F
#define ratioSet 0x21
#define volumeModeSet 0x81
#define volumeRegSet 0x20

void command(uint8_t command);
void data(uint8_t data);

void LCD_init(void);
void LCD_clear(void);
void LCD_go_to(uint8_t page, uint8_t offset);
void LCD_write_char(char character);
void LCDprintf(char stirng[]);

void Configure_SSI0(void);
void Configure_PORTB(void);

void Configure_SSI0(void)
{
	volatile unsigned long delay;
	
	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA5_SSI0TX;
	GPIO_PORTA_DEN_R |= GPIO_PIN_2 | GPIO_PIN_5;
	GPIO_PORTA_PUR_R |= GPIO_PIN_2 | GPIO_PIN_5;
	
	SSI0_CR1_R &= SSI_CR1_SSE;
	SSI0_CR1_R = 0x00000000;
	SSI0_CC_R |= SSI_CC_CS_PIOSC;
	SSI0_CPSR_R |= 0x00000002;
	SSI0_CR0_R |= 0x00000100; 
	SSI0_CR0_R |= SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_DSS_8;
	
	SSI0_CR1_R |= SSI_CR1_SSE;
}

void Configure_PORTB(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_5;
	GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PA5_M;
	GPIO_PORTB_DIR_R |= GPIO_PIN_5;
	GPIO_PORTB_AFSEL_R &= ~GPIO_PIN_5;
	GPIO_PORTB_DEN_R |= GPIO_PIN_5;
}

void command(uint8_t command)
{
	GPIO_PORTB_DATA_R &= ~GPIO_PIN_5;											// Pull PORTB5 low for data
	SSI0_DR_R = command;
	
	return;
}

void data(uint8_t data)
{
	GPIO_PORTB_DATA_R |= GPIO_PIN_5;											// Pull PORTB5 high for data
	SSI0_DR_R = data;
	
	return;
}

void LCD_init()
{
	Configure_SSI0();
	Configure_PORTB();
	
	command(displayOff);
	command(ADCSelectNormal);
	command(modeSelectNormal);
	command(biasSet);
	command(controlSet);
	command(ratioSet);
	command(volumeModeSet);
	command(volumeRegSet);
	command(displayOn);
	
	LCD_go_to(2, 0);
	LCDprintf("AYY");
	return;
}

void LCD_clear()
{
	for (int8_t j = 3; j >= 0; j--)
	{
		LCD_go_to(j, 0);								// Loop through each page
		
		for (uint8_t n = 0; n < 132; n++)
		{
			data(0x00);										// Clear row
		}
	}
	
	return;
}

void LCD_go_to(uint8_t page, uint8_t offset)
{
	command(0xB0 + page);
	command(0x10 + (offset & 0xF));
	command(0x00 + (offset & 0x0F));
	
	return;
}

void LCD_write_char(char character)
{
	switch (character)
	{
		case 'A' :
		{
			data(0x3F);
			data(0x48);
			data(0x48);
			data(0x48);
			data(0x3F);
			data(0x00);
			break;
		}
		case 'B' :
		{
			data(0x7F);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x36);
			data(0x00);
			break;
		}
		case 'C' :
		{
			data(0x3E);
			data(0x41);
			data(0x41);
			data(0x41);
			data(0x22);
			data(0x00);
			break;
		}
		case 'D' :
		{
			data(0x7F);
			data(0x41);
			data(0x41);
			data(0x41);
			data(0x3E);
			data(0x00);
			break;
		}
		case 'E' :
		{
			data(0x7F);
			data(0x49);
			data(0x49);
			data(0x41);
			data(0x41);
			data(0x00);
			break;
		}
		case 'F' :
		{
			data(0x7F);
			data(0x50);
			data(0x50);
			data(0x40);
			data(0x40);
			data(0x00);
			break;
		}
		case 'G' :
		{
			data(0x3E);
			data(0x41);
			data(0x49);
			data(0x49);
			data(0x2E);
			data(0x00);
			break;
		}
		case 'H' :
		{
			data(0x7F);
			data(0x08);
			data(0x08);
			data(0x08);
			data(0x7F);
			data(0x00);
			break;
		}
		case 'I' :
		{
			data(0x00);
			data(0x41);
			data(0x7F);
			data(0x41);
			data(0x00);
			data(0x00);
			break;
		}
		case 'J' :
		{
			data(0x06);
			data(0x41);
			data(0x41);
			data(0x41);
			data(0x7E);
			data(0x00);
			break;
		}
		case 'K' :
		{
			data(0x7F);
			data(0x08);
			data(0x14);
			data(0x22);
			data(0x41);
			data(0x00);
			break;
		}
		case 'L' :
		{
			data(0x7F);
			data(0x01);
			data(0x01);
			data(0x01);
			data(0x01);
			data(0x00);
			break;
		}
		case 'M' :
		{
			data(0x7F);
			data(0x20);
			data(0x10);
			data(0x20);
			data(0x7F);
			data(0x00);
			break;
		}
		case 'N' :
		{
			data(0x7F);
			data(0x10);
			data(0x08);
			data(0x04);
			data(0x7F);
			data(0x00);
			break;
		}
		case 'O' :
		{
			data(0x3E);
			data(0x41);
			data(0x41);
			data(0x41);
			data(0x3E);
			data(0x00);
			break;
		}
		case 'P' :
		{
			data(0x7F);
			data(0x48);
			data(0x48);
			data(0x48);
			data(0x30);
			data(0x00);
			break;
		}
		case 'Q' :
		{
			data(0x3E);
			data(0x41);
			data(0x41);
			data(0x42);
			data(0x3D);
			data(0x00);
			break;
		}
		case 'R' :
		{
			data(0x7F);
			data(0x48);
			data(0x48);
			data(0x4C);
			data(0x33);
			data(0x00);
			break;
		}
		case 'S' :
		{
			data(0x32);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x26);
			data(0x00);
			break;
		}
		case 'T' :
		{
			data(0x40);
			data(0x40);
			data(0x7F);
			data(0x40);
			data(0x40);
			data(0x00);
			break;
		}
		case 'U' :
		{
			data(0x7E);
			data(0x01);
			data(0x01);
			data(0x01);
			data(0x7E);
			data(0x00);
			break;
		}
		case 'V' :
		{
			data(0x70);
			data(0x0C);
			data(0x03);
			data(0x0C);
			data(0x70);
			data(0x00);
			break;
		}
		case 'W' :
		{
			data(0x7C);
			data(0x03);
			data(0x1C);
			data(0x03);
			data(0x7C);
			data(0x00);
			break;
		}
		case 'X' :
		{
			data(0x63);
			data(0x14);
			data(0x08);
			data(0x14);
			data(0x63);
			data(0x00);
			break;
		}
		case 'x' :
		{
			data(0x09);
			data(0x12);
			data(0x0E);
			data(0x09);
			data(0x12);
			data(0x00);
			break;
		}
		case 'Y' :
		{
			data(0x60);
			data(0x10);
			data(0x0F);
			data(0x10);
			data(0x60);
			data(0x00);
			break;
		}
		case 'Z' :
		{
			data(0x43);
			data(0x45);
			data(0x49);
			data(0x51);
			data(0x61);
			data(0x00);
			break;
		}
		case '0' :
		{
			data(0x3E);
			data(0x51);
			data(0x49);
			data(0x45);
			data(0x3E);
			data(0x00);
			break;
		}
		case '1' :
		{
			data(0x00);
			data(0x21);
			data(0x7F);
			data(0x01);
			data(0x00);
			data(0x00);
			break;
		}
		case '2' :
		{
			data(0x33);
			data(0x45);
			data(0x49);
			data(0x49);
			data(0x31);
			data(0x00);
			break;
		}
		case '3' :
		{
			data(0x22);
			data(0x41);
			data(0x49);
			data(0x49);
			data(0x36);
			data(0x00);
			break;
		}
		case '4' :
		{
			data(0x78);
			data(0x08);
			data(0x08);
			data(0x08);
			data(0x7F);
			data(0x00);
			break;
		}
		case '5' :
		{
			data(0x7A);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x46);
			data(0x00);
			break;
		}
		case '6' :
		{
			data(0x3E);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x26);
			data(0x00);
			break;
		}
		case '7' :
		{
			data(0x60);
			data(0x41);
			data(0x46);
			data(0x58);
			data(0x60);
			data(0x00);
			break;
		}
		case '8' :
		{
			data(0x36);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x36);
			data(0x00);
			break;
		}
		case '9' :
		{
			data(0x32);
			data(0x49);
			data(0x49);
			data(0x49);
			data(0x3E);
			data(0x00);
			break;
		}
		case ' ' :
		{
			data(0x00);
			data(0x00);
			data(0x00);
			data(0x00);
			data(0x00);
			data(0x00);
			break;
		}
		case ':' :
		{
			data(0x00);
			data(0x00);
			data(0x24);
			data(0x00);
			data(0x00);
			data(0x00);
			break;
		}
		case '.' :
		{
			data(0x00);
			data(0x00);
			data(0x01);
			data(0x00);
			data(0x00);
			data(0x00);
			break;
		}
		default :
		{
			data(0xFF);
			data(0xFF);
			data(0xFF);
			data(0xFF);
			data(0xFF);
			data(0x00);
		}
	}
	
	return;
}

void LCDprintf(char string[])
{
	for (uint8_t i = 0; i < strlen(string); i++)
	{
		LCD_write_char(string[i]);
	}
	
	return;
}
