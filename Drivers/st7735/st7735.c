/* vim: set ai et ts=4 sw=4: */
#include "st7735.h"


#define DELAY 0x80

//extern uint16_t screen_buf[128*160];
//extern SemaphoreHandle_t DisplaySPI_Busy;

extern SPI_HandleTypeDef ST7735_SPI_PORT;
extern SemaphoreHandle_t sDisplaySPI;
static uint16_t data_buf[128*160];
static uint8_t cmd_buf;

// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay

static void ST7735_Select() {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

void ST7735_Unselect() {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset() {
	ST7735_Select();
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
    ST7735_Unselect();
}

static void ST7735_WriteCommand(uint8_t cmd) {
	//Copy the command to the protected screen buffer;
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	cmd_buf = cmd;
	ST7735_Select();
	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, &cmd_buf, sizeof(cmd));
    //while (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX);
}

static void ST7735_WriteData(uint8_t* buff, size_t buff_size) {
	//Copy the data to the protected screen buffer;
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	for(int i = 0; i < buff_size; i++) {
		((uint8_t *)data_buf)[i] = buff[i];
	}
	ST7735_Select();
	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, (uint8_t *)data_buf, buff_size);
    //while (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX);
}

static void ST7735_ExecuteCommandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // column address set
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    ST7735_WriteData(data, sizeof(data));

    // row address set
    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    ST7735_WriteData(data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init() {
    //ST7735_Select();
    ST7735_Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    //ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    //ST7735_Select();

    ST7735_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(data, sizeof(data));

    //ST7735_Unselect();
}

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            }
        }
    }
}

void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    //ST7735_Select();

    while(*str) {
        if(x + font.width >= ST7735_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    //ST7735_Unselect();
}

void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
	if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
	if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
	if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

	//uint8_t data[] = { color >> 8, color & 0xFF };
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	xSemaphoreGive(sDisplaySPI);
	for(y = h; y > 0; y--) {
		for(x = w; x > 0; x--) {
			data_buf[(y - 1)*w + x -1] = (color >> 8) + ((color & 0xFF) << 8);

		}
	}
	ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
	ST7735_WriteData((uint8_t*)&data_buf, w*h*(sizeof(uint16_t)));
}

void ST7735_FillScreen(uint16_t color) {
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    //ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    //ST7735_Unselect();
}

void ST7735_InvertColors(bool invert) {
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}

//=====================================================================================================================

void ST7735_Refresh() {
	ST7735_SetAddressWindow(0, 0, ST7735_WIDTH, ST7735_HEIGHT);
	ST7735_WriteData((uint8_t*)&data_buf, ST7735_WIDTH*ST7735_HEIGHT*(sizeof(uint16_t)));
}


void ST7735_AddPixel(uint8_t x, uint8_t y, uint16_t color) {
	if(y > ST7735_HEIGHT || y < 0 || x > ST7735_WIDTH || x < 0) return;
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	data_buf[y*ST7735_WIDTH + x] = (color >> 8) + ((color & 0xFF) << 8);
	xSemaphoreGive(sDisplaySPI);

}

void ST7735_AddHorLine(uint8_t y, uint8_t x_start, uint8_t x_end, uint8_t width, uint16_t color, LineType_t type) {
	if(y > ST7735_HEIGHT || y < 0) return;
	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	if(type == SOLID) {
		xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
		for(uint8_t x = x_start; x <= x_end; x++)
			data_buf[y*ST7735_WIDTH + x] = color__;
		xSemaphoreGive(sDisplaySPI);
	} else {
		xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
		for(uint8_t x = x_start; x <= x_end; x++)
			if((x & 0x02) == 0) data_buf[y*ST7735_WIDTH + x] = color__;
		xSemaphoreGive(sDisplaySPI);
	}
}
void ST7735_AddVerLine(uint8_t x, uint8_t y_start, uint8_t y_end, uint8_t width, uint16_t color, LineType_t type) {
	if(x > ST7735_WIDTH || x < 0) return;
	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	if(type == SOLID) {
		xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
		for(int y = y_start; y <= y_end; y++)
			data_buf[y*ST7735_WIDTH + x] = color__;
		xSemaphoreGive(sDisplaySPI);
	} else {
		xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
		for(int y = y_start; y <= y_end; y++)
			if((y & 0x02) == 0)data_buf[y*ST7735_WIDTH + x] = color__;
		xSemaphoreGive(sDisplaySPI);
	}
}
void ST7735_AddRectangle(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint16_t color) {
	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	for(int y = y_start; y <= y_end; y++) {
		for(int x = x_start; x <= x_end; x++) {
			data_buf[y*ST7735_WIDTH + x] = color__;
		}
	}
	xSemaphoreGive(sDisplaySPI);

}
void ST7735_AddFill(uint16_t color) {
	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	for(int i = 0; i < ST7735_WIDTH*ST7735_HEIGHT; i++) {
		data_buf[i] = color__;
	}
	xSemaphoreGive(sDisplaySPI);
}
static void ST7735_AddLineLow(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint16_t color__, LineType_t type) {

	int dx = (int)x_end - (int)x_start;
	int dy = (int)y_end - (int)y_start;
	int yi = 1;
	if(dy < 0) {
		yi = -1;
		dy = -dy;
	}
	int D = 2*dy - dx;
	uint8_t y = y_start;
	for(uint8_t x = x_start; x <= x_end; x++) {
		data_buf[y*ST7735_WIDTH + x] = color__;
		if(D > 0) {
			y = y + yi;
			D = D - 2*dx;
		}
		D = D + 2*dy;

	}
}

static void ST7735_AddLineHigh(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint16_t color__, LineType_t type_) {
	int dx = (int)x_end - (int)x_start;
	int dy = (int)y_end - (int)y_start;
	int xi = 1;
	if(dx < 0) {
		xi = -1;
		dx = -dx;
	}
	int D = 2*dx - dy;
	uint8_t x = x_start;
	for(uint8_t y = y_start; y <= y_end; y++) {
		data_buf[y*ST7735_WIDTH + x] = color__;
		if(D > 0) {
			x = x + xi;
			D = D - 2*dy;
		}
		D = D + 2*dx;

	}
}
void ST7735_AddLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint8_t width, uint16_t color, LineType_t type) {
	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	int abs_dx = x_start >= x_end ? x_start - x_end : x_end - x_start;
	int abs_dy = y_start >= y_end ? y_start - y_end : y_end - y_start;
	if(abs_dy < abs_dx) {
		if(x_start > x_end)
			ST7735_AddLineLow(x_end, y_end, x_start, y_start, color__, type);
		else
			ST7735_AddLineLow(x_start, y_start, x_end, y_end, color__, type);
	} else {
		if(y_start > y_end)
			ST7735_AddLineHigh(x_end, y_end, x_start, y_start, color__, type);
		else
			ST7735_AddLineHigh(x_start, y_start, x_end, y_end, color__, type);
	}

}


static void ST7735_AddChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color__, uint16_t bgcolor__) {
    uint32_t i, b, j;

    //ST7735_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
            	data_buf[(y+i)*ST7735_WIDTH + x + j] = color__;
            }
        }
    }
}

void ST7735_AddString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {

	uint16_t color__ = (color >> 8) + ((color & 0xFF) << 8);
	uint16_t bgcolor__ = (bgcolor >> 8) + ((bgcolor & 0xFF) << 8);
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
    while(*str) {
        if(x + font.width >= ST7735_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_AddChar(x, y, *str, font, color__, bgcolor__);
        x += font.width;
        str++;
    }
    xSemaphoreGive(sDisplaySPI);
}

void pushChannelData(int8_t newData, ChannelData_t * dataStruct) {
	dataStruct->values[dataStruct->head] = newData;
	if(dataStruct->cnt < 127) dataStruct->cnt++;
	dataStruct->head++;
	dataStruct->head &= 0x7F;
}

void printChannelData(uint8_t x_start, uint8_t y_start, ChannelData_t * dataStruct, uint16_t color) {

	if(dataStruct->cnt == 0) return;
	uint16_t color_norm__ = (color >> 8) + ((color & 0xFF) << 8);
	uint16_t color_clip__ = ((ST7735_RED >> 8) + ((ST7735_RED & 0xFF) << 8));
	int i = dataStruct->head - 1;
	int cnt = 0;
	xSemaphoreTake(sDisplaySPI, portMAX_DELAY);
	while(cnt < dataStruct->cnt) {
		uint16_t color__;
		if(dataStruct->values[i] >= (int8_t)71) {
			dataStruct->values[i] = (int8_t)71;
			color__ = color_clip__;
		} else if(dataStruct->values[i] <= (int8_t)(-70)) {
			dataStruct->values[i] = (int8_t)(-70);
			color__ = color_clip__;
		}
		else
			color__ = color_norm__;
		uint8_t y = y_start + (dataStruct->values[i] >> 2);
		data_buf[y*ST7735_WIDTH + x_start + cnt] = color__;//dataStruct.values[i];
		i--;
		i &= 0x7F;
		cnt++;
	}
	xSemaphoreGive(sDisplaySPI);

}



