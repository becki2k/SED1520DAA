//-------------------------------------------------------------------------------------------------
#include "SED1520.h"
#include <stdlib.h>
//-------------------------------------------------------------------------------------------------
unsigned char lcd_x = 0, lcd_y = 0;
//-------------------------------------------------------------------------------------------------
extern unsigned char GLCD_WaitForStatus(unsigned char, unsigned char);
extern void GLCD_WriteCommand(unsigned char, unsigned char);
extern void GLCD_WriteDatta(unsigned char);
extern unsigned char GLCD_ReadData(void);
extern void GLCD_InitPorts(void);


//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_GoTo(unsigned char x,unsigned char y)			//GLCD_GoTo(column, page)
{
lcd_x = x;
lcd_y = y;

if(x < (SCREEN_WIDTH/2))
  {
  GLCD_WriteCommand(COLUMN_ADDRESS_SET | lcd_x, 0);		
  GLCD_WriteCommand(PAGE_ADDRESS_SET | lcd_y, 0);
  GLCD_WriteCommand(COLUMN_ADDRESS_SET | 0, 1);			//2nd Chip Column0, prepare for switch
  GLCD_WriteCommand(PAGE_ADDRESS_SET | lcd_y, 1);		//also set page of 2nd Chip, prepare for switch
  }
else
  {
  GLCD_WriteCommand(COLUMN_ADDRESS_SET | (lcd_x - (SCREEN_WIDTH/2)), 1);
  GLCD_WriteCommand(PAGE_ADDRESS_SET | lcd_y, 1);
  }
}
//-------------------------------------------------------------------------------------------------
// 
//-------------------------------------------------------------------------------------------------
void GLCD_ClearScreen(void)
{
char j, i;
for(j = 0; j < 4; j++)
  {
  GLCD_GoTo(0, j);
  for(i = 0; i < SCREEN_WIDTH; i++)
    {
	GLCD_WriteData(0);
	}
  }
GLCD_GoTo(0, 0);
}
//
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_WriteChar(char x)
{
char i; 
x -= 32; 									//refers to the font.h library (not all ASCII-signs are used)
for(i = 0; i < 5; i++) 
  GLCD_WriteData(pgm_read_byte(font5x7 + (5 * x) + i));
GLCD_WriteData(0x00);						//space between two char-signs
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_WriteString(char * s)
{
while(*s)
  {
  GLCD_WriteChar(*s++);
  }
}
//-------------------------------------------------------------------------------------------------
// GLCD_WriteInt
//-------------------------------------------------------------------------------------------------
void GLCD_WriteInt(int16_t zahl)
{
	char buffer[7];
	itoa( zahl, buffer, 10 ); 				// 10 fuer radix -> Dezimalsystem
	GLCD_WriteString( buffer );
}
//-------------------------------------------------------------------------------------------------
// GLCD_WriteHex
//-------------------------------------------------------------------------------------------------
inline unsigned char nibble_zu_hex(unsigned char nib) {
 if (nib < 0x0A) nib = nib + '0';
 else nib = (nib - 0x0A) + 'A';
 return nib;
}

void GLCD_WriteHex(unsigned char zahl)
{
	unsigned char temp; 
	temp = (zahl & 0x0F); 
	zahl = (zahl>>4); 
	GLCD_WriteChar(nibble_zu_hex(zahl)); 
	GLCD_WriteChar(nibble_zu_hex(temp)); 
}
//-------------------------------------------------------------------------------------------------
// GLCD_SetPixel
//-------------------------------------------------------------------------------------------------
void GLCD_SetPixel(unsigned char x, unsigned char y, unsigned char color)  //color=1 => Pixel=Black
{
	unsigned char temp;  
	GLCD_GoTo(x, y/8); 	
	
	GLCD_ReadData();
	temp = GLCD_ReadData();						//save old state 
	
	GLCD_GoTo(x, y/8); 							//Chips increments automatically, jump back
	if(color)
	  GLCD_WriteData(temp | (1 << (y % 8)));    
	else
	  GLCD_WriteData(temp & ~(1 << (y % 8))); 
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Bitmap(char * bmp, unsigned char x, unsigned char y, unsigned char dx, unsigned char dy)
{
	unsigned char i, j;
	for(j = 0; j < dy / 8; j++)
	  {
	  GLCD_GoTo(x,y + j);
	  for(i = 0; i < dx; i++)
	    GLCD_WriteData(pgm_read_byte(bmp++)); 
	  }
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Init(void)
{
	GLCD_InitPorts();
	GLCD_WriteCommand(RESET, 0);
	GLCD_WriteCommand(RESET, 1);
	GLCD_WaitForStatus(0x10, 0);
	GLCD_WaitForStatus(0x10, 1);
	GLCD_WriteCommand(DISPLAY_ON, 0);
	GLCD_WriteCommand(DISPLAY_ON, 1);
	GLCD_WriteCommand(DISPLAY_START_LINE | 0, 0);
	GLCD_WriteCommand(DISPLAY_START_LINE | 0, 1);
}
//-------------------------------------------------------------------------------------------------

