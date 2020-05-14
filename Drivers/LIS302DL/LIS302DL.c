/*
 * LIS302DL.c
 *
 *  Created on: May 12, 2020
 *      Author: andrii
 */

#include "LIS302DL.h"

extern SPI_HandleTypeDef LIS302DL_SPI_PORT;

static uint8_t out_cmd_buf;
static uint8_t in_data_buf;

static void LIS302DL_Select(void) {
	HAL_GPIO_WritePin(LIS302DL_CS_GPIO_Port, LIS302DL_CS_pin, GPIO_PIN_RESET);
}

static void LIS302DL_Unselect(void) {
	HAL_GPIO_WritePin(LIS302DL_CS_GPIO_Port, LIS302DL_CS_pin, GPIO_PIN_SET);
}

void LIS302DL_Init(void) {
	LIS302DL_Select();
	uint8_t cmd_buf[2];
	cmd_buf[0] = LIS302DL_CTRL_REG1;
	cmd_buf[1] = 0x47;
	HAL_SPI_Transmit(&LIS302DL_SPI_PORT, &(cmd_buf[0]), 2, HAL_MAX_DELAY);
	LIS302DL_Unselect();
}

void LIS302DL_GetId(uint8_t * output) {
	out_cmd_buf = (LIS302DL_WHOAMI | (0x80));
	LIS302DL_Select();
	HAL_SPI_Transmit(&LIS302DL_SPI_PORT, &out_cmd_buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LIS302DL_SPI_PORT, output, 1, HAL_MAX_DELAY);
	LIS302DL_Unselect();
}
int8_t LIS302DL_GetX(void) {
	uint8_t out_x;
	out_cmd_buf = (LIS302DL_OUT_X | (0x80));
	LIS302DL_Select();
	HAL_SPI_Transmit(&LIS302DL_SPI_PORT, &out_cmd_buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LIS302DL_SPI_PORT, &out_x, 1, HAL_MAX_DELAY);
	LIS302DL_Unselect();
	return (int8_t)out_x;
}
int8_t LIS302DL_GetY(void) {
	uint8_t out_y;
	out_cmd_buf = (LIS302DL_OUT_Y | (0x80));
	LIS302DL_Select();
	HAL_SPI_Transmit(&LIS302DL_SPI_PORT, &out_cmd_buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LIS302DL_SPI_PORT, &out_y, 1, HAL_MAX_DELAY);
	LIS302DL_Unselect();
	return (int8_t)out_y;
}
int8_t LIS302DL_GetZ(void) {
	uint8_t out_z;
	out_cmd_buf = (LIS302DL_OUT_Z | (0x80));
	LIS302DL_Select();
	HAL_SPI_Transmit(&LIS302DL_SPI_PORT, &out_cmd_buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LIS302DL_SPI_PORT, &out_z, 1, HAL_MAX_DELAY);
	LIS302DL_Unselect();
	return (int8_t)out_z;
}

