/*
 * LIS302DL.h
 *
 *  Created on: May 12, 2020
 *      Author: andrii
 */

#ifndef LIS302DL_LIS302DL_H_
#define LIS302DL_LIS302DL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

#define LIS302DL_SPI_PORT hspi1

#define LIS302DL_CS_pin			GPIO_PIN_3
#define LIS302DL_CS_GPIO_Port 	GPIOE

/*** Redefine if necessary ***/
//#define ST7735_SPI_PORT hspi3
//
//
//#define ST7735_RES_Pin       GPIO_PIN_1 // PD1
//#define ST7735_RES_GPIO_Port GPIOD 		//GPIOD
//#define ST7735_CS_Pin        GPIO_PIN_4
//#define ST7735_CS_GPIO_Port  GPIOA
//#define ST7735_DC_Pin        GPIO_PIN_8
//#define ST7735_DC_GPIO_Port  GPIOC

/*** LIS302DL Register addresses ***/
#define LIS302DL_CTRL_REG1 		0x20
#define LIS302DL_WHOAMI 		0x0F
#define LIS302DL_OUT_X 			0x29
#define LIS302DL_OUT_Y 			0x2B
#define LIS302DL_OUT_Z 			0x2D


void LIS302DL_Init();
void LIS302DL_GetId(uint8_t * output);
int8_t LIS302DL_GetX();
int8_t LIS302DL_GetY();
int8_t LIS302DL_GetZ();


#endif /* LIS302DL_LIS302DL_H_ */
