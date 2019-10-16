// Header file for NHD-C12832A1Z-NSW-BBW-3V3 LCD module
//
// Uses the following pins of PORTA:
// PORTA0		-		!chip select (!CS)
// PORTA1		-		!reset (!RS)
// PORTA2		-		message mode, 0 = command, 1 = data (A0)
// PORTA3		-		clock (SCL)
// PORTA4		-		serial input (SI)
//
// Uses timer 2A for communication clocking

#include <stdint.h>
#include <string.h>

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
void LCD_write(char stirng[]);

void command(uint8_t command)
{
	
	return;
}

void data(uint8_t data)
{
	
	return;
}

void LCD_init()
{
	command(displayOff);
	command(ADCSelectNormal);
	command(modeSelectNormal);
	command(biasSet);
	command(controlSet);
	command(ratioSet);
	command(volumeModeSet);
	command(volumeRegSet);
	command(displayOn);
	
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

void LCD_write(char string[])
{
	for (uint8_t i = 0; i < strlen(string); i++)
	{
		LCD_write_char(string[i]);
	}
	
	return;
}
