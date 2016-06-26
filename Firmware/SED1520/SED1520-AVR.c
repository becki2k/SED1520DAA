
/*********************************** Configuration ***********************************/

#include <avr/io.h>
#define F_CPU 100000
#include <util/delay.h>

#define SCREEN_WIDTH	122

#define SED1520_DATA_PORT 	PORTD
#define SED1520_DATA_DDR 	DDRD
#define SED1520_DATA_PIN 	PIND

#define SED1520_CONTROL_PORT 	PORTC
#define SED1520_CONTROL_DDR 	DDRC


#define SED1520_RW 		(1 << 0)
#define SED1520_Enable  (1 << 1)
#define SED1520_CS1 	(1 << 2)
#define SED1520_CS2 	(1 << 3)
#define SED1520_A0 		(1 << 4)
#define SED1520_RES 	(1 << 5)

/********************************* End Configuration **********************************/


#define RW_h			SED1520_CONTROL_PORT |= SED1520_RW;
#define RW_l			SED1520_CONTROL_PORT &= ~SED1520_RW;
#define Enable_h		SED1520_CONTROL_PORT |= SED1520_Enable;
#define Enable_l		SED1520_CONTROL_PORT &= ~SED1520_Enable;
#define CS1_h			SED1520_CONTROL_PORT |= SED1520_CS1;
#define CS1_l			SED1520_CONTROL_PORT &= ~SED1520_CS1;
#define CS2_h			SED1520_CONTROL_PORT |= SED1520_CS2;
#define CS2_l			SED1520_CONTROL_PORT &= ~SED1520_CS2;
#define A0_h			SED1520_CONTROL_PORT |= SED1520_A0;
#define A0_l			SED1520_CONTROL_PORT &= ~SED1520_A0;
#define RES_h			SED1520_CONTROL_PORT |= SED1520_RES;
#define RES_l			SED1520_CONTROL_PORT &= ~SED1520_RES;


extern unsigned char lcd_x, lcd_y;			//global Variables for current Position
//-------------------------------------------------------------------------------------------------
// Default-Setting
//-------------------------------------------------------------------------------------------------
void GLCD_InitPorts(void)
{
	SED1520_DATA_DDR = 0xFF;				//Output
	
	// Set Controll-Bits to Output
	SED1520_CONTROL_DDR |= (SED1520_CS2 | SED1520_CS1 | SED1520_RW | SED1520_A0 | SED1520_RES | SED1520_Enable);
	_delay_ms(1);	
	RES_h;				//High (always)==> 68 Family MPU
	
	//Set Default-State of Controll-Bits
	CS1_h; CS2_h; Enable_l;  
}
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Check Busy-Flag
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_WaitForStatus(unsigned char status, unsigned char controller)
{
	unsigned char tmp;
	SED1520_DATA_DDR = 0x00; 			//Data-Input
	SED1520_DATA_PORT = 0xFF; 			//Activate uC's intern PullUps
	A0_l;								//Instruction
	RW_h; 								//Read from GLCD 
	do
	  {
	  
	  if(controller == 0) {CS1_l;} 		
	  else {CS2_l;} 
	  
	  _delay_us(1); 
	   Enable_h;
	  _delay_us(1);
	  tmp = SED1520_DATA_PIN; 
	  _delay_us(1); 
	  Enable_l; 
	  CS1_h; CS2_h; 
	  
	  }while(tmp & status);
	
	SED1520_DATA_PORT = 0x00; 
	SED1520_DATA_DDR = 0xFF; 	
	return tmp; 
}
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Write command
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite,unsigned char ctrl)
{
	GLCD_WaitForStatus(0x80, ctrl);
	_delay_ms(10); 
	SED1520_DATA_DDR = 0xFF; 
	A0_l; 
	RW_l;

	if(ctrl) {CS2_l;}
	else {CS1_l;}
	
	_delay_us(1); 
	Enable_h; 
	_delay_us(1); 
	SED1520_DATA_PORT = commandToWrite;
	_delay_us(1); 
	Enable_l; 
	CS1_h; CS2_h; 
	}
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Write data
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite, unsigned char ctrl)
{
	GLCD_WaitForStatus(0x80, ctrl);
	SED1520_DATA_DDR = 0xFF; 
	A0_h; 
	RW_l; 

	if(lcd_x < 61) {CS1_l;}
	else { CS2_l; }
	
	_delay_us(1); 
	Enable_h; 
	_delay_us(1); 
	SED1520_DATA_PORT = dataToWrite; 
	_delay_us(1); 
	Enable_l; 
	CS1_h; CS2_h; 
	lcd_x++;
	
	if(lcd_x >= SCREEN_WIDTH)lcd_x = 0;
}
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Read data
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
	unsigned char tmp;	

	if (lcd_x < 61) GLCD_WaitForStatus(0x80, 0);
	else GLCD_WaitForStatus(0x80, 1);
	
	SED1520_DATA_DDR = 0x00;
	SED1520_DATA_PORT = 0xFF;
	
	RW_h;
	A0_h;
	
	if (lcd_x < 61) {CS1_l;}
	 else { CS2_l;}
	
	_delay_us(1);
	Enable_h;
	_delay_us(1);
	tmp = SED1520_DATA_PIN; 
	_delay_us(1);
	Enable_l;  
	CS1_h; CS2_h;
	SED1520_DATA_PORT = 0x00;
	SED1520_DATA_DDR = 0xFF; 

	return tmp; 
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
