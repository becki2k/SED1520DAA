#include "font.h"


//Commands
#define DISPLAY_ON 0xAF
#define DISPLAY_OFF 0xAE
#define DISPLAY_START_LINE 0xC0		//Default: Start-Adress (Row 0)
#define PAGE_ADDRESS_SET 0xB8		//Cursor-Pos. Row (0xB8:Row0, 0xB9:Row1, 0xBA:Row2, 0xBB:Row3)
#define COLUMN_ADDRESS_SET 0x00		//Cursor-Pos. Column (0x00:C(olumn)0, 0x01:Column1, 0x6F:Column79)
#define ADC_CLOCKWISE 0xA0
#define ADC_COUNTERCLOCKWISE 0xA1
#define STATIC_DRIVE_ON 0xA5
#define STATIC_DRIVE_OFF 0xA4
#define DUTY_RATIO_16 0xA8
#define DUTY_RATIO_32 0xA9
#define READ_MODIFY_WRITE 0xE0
#define END_READ_MODIFY 0xEE
#define RESET 0xE2


#define SCREEN_WIDTH	122

unsigned char GLCD_WaitForStatus(unsigned char,unsigned char);
void GLCD_WriteCommand(unsigned char,unsigned char);
void GLCD_GoTo(unsigned char,unsigned char);
void GLCD_WriteData(unsigned char);
unsigned char GLCD_ReadData(void);
void GLCD_ClearScreen(void);
void GLCD_WriteChar(char);
void GLCD_WriteString(char *);
void GLCD_WriteHex(unsigned char zahl);
void GLCD_WriteInt(int16_t zahl);
void GLCD_SetPixel(unsigned char, unsigned char, unsigned char);
void GLCD_Init(void);
void GLCD_Bitmap(char * , unsigned char, unsigned char, unsigned char, unsigned char);


