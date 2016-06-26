/**
  ******************************************************************************
  * @file    SED-1520DAA.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    11-January-2016
  * @brief   CORTEX HAL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the CORTEX:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *          
  *  @verbatim    
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================

    1.
		2.
		3.
   
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <SED-1520DAA.h>
#include "tim.h"

#define DEBUG 1

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LCD_ADC_FORWARD					0xA0
#define LCD_ADC_REVERSE					0xA1
#define LCD_DISPLAY_OFF					0xAF
#define LCD_DISPLAY_ON					0xAE
#define LCD_DISPLAY_STARTLINE		0xC0	// & 3..0 Line Adress		[0..31]
#define LCD_DUTY_1_16						0xA9
#define LCD_DUTY_1_32						0xA9
#define LCD_END									0xEE
#define LCD_RESET								0xE2
#define LCD_SET_PAGE_ADRESS			0xB8	// & 1..0 Page Adress		[0..3]
#define LCD_SET_COLUMN_ADDRESS	0x00	// & 6..0 Column Adress	[0..127]
#define LCD_STATIC_DRIVER_ON		0xA4
#define LCD_STATIC_DRIVER_OFF		0xA5


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t status;
/* Private function prototypes -----------------------------------------------*/
void send_cmd(LCD_Handle *pDisplay, GPIO_PinState ctrl_A0, GPIO_PinState ctrl_RW, uint8_t Command);
uint8_t data_read(LCD_Handle *pDisplay, GPIO_PinState ctrl_A0, GPIO_PinState ctrl_RW, uint8_t Command, uint8_t Recieve[]);
void data_write(LCD_Handle *pDisplay, uint8_t Command[]);
void init_GPIO_Input(GPIO_TypeDef *Port, uint16_t Pin);
void init_GPIO_Output(GPIO_TypeDef *Port, uint16_t Pin);

/* Private functions ---------------------------------------------------------*/
/**
 *	@param ctrl_RW Indicates if command is a read- or write-operation. (High=Read, Low=Write)
 */
void send_cmd(LCD_Handle *pDisplay, GPIO_PinState ctrl_A0, GPIO_PinState ctrl_RW, uint8_t Command)
{
	HAL_GPIO_WritePin(pDisplay->A_0.Port, pDisplay->A_0.Pin, ctrl_A0);
	HAL_GPIO_WritePin(pDisplay->nRD.Port, pDisplay->nRD.Pin, (GPIO_PinState)~ctrl_RW);
	HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, ctrl_RW);
	HAL_GPIO_WritePin(pDisplay->CS1.Port, pDisplay->CS1.Pin, GPIO_PIN_RESET);
	
	HAL_Delay(5);
	HAL_GPIO_WritePin(pDisplay->DB0.Port, pDisplay->DB0.Pin, (GPIO_PinState)((Command >> 0) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB1.Port, pDisplay->DB1.Pin, (GPIO_PinState)((Command >> 1) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB2.Port, pDisplay->DB2.Pin, (GPIO_PinState)((Command >> 2) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB3.Port, pDisplay->DB3.Pin, (GPIO_PinState)((Command >> 3) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB4.Port, pDisplay->DB4.Pin, (GPIO_PinState)((Command >> 4) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB5.Port, pDisplay->DB5.Pin, (GPIO_PinState)((Command >> 5) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB6.Port, pDisplay->DB6.Pin, (GPIO_PinState)((Command >> 6) & 0x01));
	HAL_GPIO_WritePin(pDisplay->DB7.Port, pDisplay->DB7.Pin, (GPIO_PinState)((Command >> 7) & 0x01));
	//GPIOA->ODR = Command & 0x00FF;
	
	switch(ctrl_RW){
		case GPIO_PIN_SET:
			HAL_GPIO_WritePin(pDisplay->nRD.Port, pDisplay->nRD.Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
			HAL_GPIO_WritePin(pDisplay->nRD.Port, pDisplay->nRD.Pin, GPIO_PIN_SET);
		break;
		case GPIO_PIN_RESET:
			HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, GPIO_PIN_SET);
		break;
	}
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(pDisplay->CS1.Port, pDisplay->CS1.Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, GPIO_PIN_SET);
}


/**
*
*/
uint8_t LCD_init(LCD_Handle *pDisplay)
{
	//HAL_GPIO_WritePin(pDisplay->CS1.Port, pDisplay->CS1.Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(pDisplay->CS2.Port, pDisplay->CS2.Pin, GPIO_PIN_RESET);

	while(read_status(pDisplay, 0) >= 0x80){
		status = read_status(pDisplay, 0);
		if(DEBUG)printf("Status: %d\n", status);
	}
	
	if(DEBUG)printf("Start Init-Funktion\n");
	printf("Start Init-Funktion\n");
	uint8_t status = 0;
	while(read_status(pDisplay, 0) >= 0x80){status = read_status(pDisplay, 0);		if(DEBUG)printf("Status: %d\n", status);}
	
//(a)Display off
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_DISPLAY_OFF);
	/*while(read_status(pDisplay) >= 0x80){
		status = read_status(pDisplay);
		if(DEBUG)printf("Status: %d\n", status);
	}*/
	
	//(-)reset
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_RESET);
	/*while(read_status(pDisplay) >= 0x80){
		status = read_status(pDisplay);
		if(DEBUG)printf("Status: %d\n", status);
	}*/
	
	//(b)Display start line register: First line
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_DISPLAY_STARTLINE + 0x00);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(c)Static drive off
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_STATIC_DRIVER_OFF);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(d)Column address counter: Address 0
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_SET_COLUMN_ADDRESS);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(e)Page address register: Page 0
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_SET_PAGE_ADRESS + 0x00);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(f)Select duty: 1/32
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_DUTY_1_32);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(g)Select ADC: Forward (ADC command D0 = “0”, ADC status flag = “1”)
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_ADC_FORWARD);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	//(h)Read modify write off
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_END);
	//while(read_status(pDisplay) >= 0x80){status = read_status(pDisplay);		if(DEBUG)printf("Status: %d\n", status);}
	
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_DISPLAY_ON);
	while(read_status(pDisplay, 0) >= 0x80){status = read_status(pDisplay, 0);		if(DEBUG)printf("Status: %d\n", status);}
	

	return status;
}

uint8_t read_status(LCD_Handle *pDisplay, uint8_t Controller)
{
	//Init GPIO as Input
	init_GPIO_Input(pDisplay->DB0.Port, pDisplay->DB0.Pin);
	init_GPIO_Input(pDisplay->DB1.Port, pDisplay->DB1.Pin);
	init_GPIO_Input(pDisplay->DB2.Port, pDisplay->DB2.Pin);
	init_GPIO_Input(pDisplay->DB3.Port, pDisplay->DB3.Pin);
	init_GPIO_Input(pDisplay->DB4.Port, pDisplay->DB4.Pin);
	init_GPIO_Input(pDisplay->DB5.Port, pDisplay->DB5.Pin);
	init_GPIO_Input(pDisplay->DB6.Port, pDisplay->DB6.Pin);
	init_GPIO_Input(pDisplay->DB7.Port, pDisplay->DB7.Pin);
	
	// Instruction Read
	HAL_GPIO_WritePin(pDisplay->A_0.Port, pDisplay->A_0.Pin, (GPIO_PinState)0);
	HAL_GPIO_WritePin(pDisplay->nWR.Port, pDisplay->nWR.Pin, (GPIO_PinState)1);
	//HAL_GPIO_WritePin(pDisplay->nRD.Port, pDisplay->nRD.Pin, (GPIO_PinState)0);
	uint8_t read = 0;
	
	switch(Controller){
		case 0:	HAL_GPIO_WritePin(pDisplay->CS1.Port, pDisplay->CS1.Pin, (GPIO_PinState)0);
						break;
		case 1:	HAL_GPIO_WritePin(pDisplay->CS2.Port, pDisplay->CS2.Pin, (GPIO_PinState)0);
						break;
	}
	
	read = (pDisplay->DB0.Port->IDR & 0xFF);
	printf("Read Status: %X\n", read);
	
	HAL_GPIO_WritePin(pDisplay->CS1.Port, pDisplay->CS1.Pin, (GPIO_PinState)1);
	HAL_GPIO_WritePin(pDisplay->CS2.Port, pDisplay->CS2.Pin, (GPIO_PinState)1);
	
	init_GPIO_Output(pDisplay->DB0.Port, pDisplay->DB0.Pin);
	init_GPIO_Output(pDisplay->DB1.Port, pDisplay->DB1.Pin);
	init_GPIO_Output(pDisplay->DB2.Port, pDisplay->DB2.Pin);
	init_GPIO_Output(pDisplay->DB3.Port, pDisplay->DB3.Pin);
	init_GPIO_Output(pDisplay->DB4.Port, pDisplay->DB4.Pin);
	init_GPIO_Output(pDisplay->DB5.Port, pDisplay->DB5.Pin);
	init_GPIO_Output(pDisplay->DB6.Port, pDisplay->DB6.Pin);
	init_GPIO_Output(pDisplay->DB7.Port, pDisplay->DB7.Pin);

	return read;
}
/**
*
*/
void LCD_Write_Pxl(LCD_Handle *pDisplay, uint8_t x, uint8_t y, LCD_PXL_STATE State)
{
	//DISABLE VOLTAGE
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_SET_PAGE_ADRESS + 2);
	send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_RESET, LCD_SET_COLUMN_ADDRESS + 5);
	send_cmd(pDisplay, GPIO_PIN_SET, GPIO_PIN_RESET, 0xFF);
	//ENABLE VOLTAGE
	
	//send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_SET, State);
}

uint8_t data_read(LCD_Handle *pDisplay, GPIO_PinState ctrl_A0, GPIO_PinState ctrl_RW, uint8_t Command, uint8_t Recieve[]){
	
	return 0;
}

void data_write(LCD_Handle *pDisplay, uint8_t Command[])
{
	for(int i=0; Command[i]!='\0';i++){
		send_cmd(pDisplay, GPIO_PIN_RESET, GPIO_PIN_SET, Command[i]);
	}
	
}

void init_GPIO_Output(GPIO_TypeDef *Port, uint16_t Pin){
  GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}


void init_GPIO_Input(GPIO_TypeDef *Port, uint16_t Pin){
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
