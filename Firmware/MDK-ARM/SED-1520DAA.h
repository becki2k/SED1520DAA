/**
  ******************************************************************************
  * @file    SED-1520DAA_H.h
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    11-January-2016
  * @brief   This file contains all the functions prototypes for the HAL 
  *          module driver.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SED_1520DAA_H
#define __SED_1520DAA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */
	 
/* Exported types ------------------------------------------------------------*/
typedef enum{
	LCD_PXL_RESET,
	LCD_PXL_SET
}LCD_PXL_STATE;
	 
struct Pin_Def{
	GPIO_TypeDef	*	Port;
	uint32_t				Pin;
};

typedef struct {
	struct Pin_Def A_0;
	struct Pin_Def CS2;
	struct Pin_Def CS1;
	struct Pin_Def nRD;
	struct Pin_Def nWR;
	struct Pin_Def DB0;
	struct Pin_Def DB1;
	struct Pin_Def DB2;
	struct Pin_Def DB3;
	struct Pin_Def DB4;
	struct Pin_Def DB5;
	struct Pin_Def DB6;
	struct Pin_Def DB7;
}LCD_Handle;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint8_t LCD_init(LCD_Handle *pDisplay);
void LCD_Write_Pxl(LCD_Handle *pDisplay, uint8_t x, uint8_t y, LCD_PXL_STATE State);
/* Initialization and de-initialization functions  ******************************/
/* Peripheral Control functions  ************************************************/




/**
  * @}
  */ 
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
