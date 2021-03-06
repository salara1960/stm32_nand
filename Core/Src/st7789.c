//#include "main.h"
#include "st7789.h"

//-----------------------------------------------------------------------------------------

const uint32_t waits = 150;
//#ifdef SET_WITH_DMA
//	uint8_t *frm_buf = NULL;
//	const uint8_t total_blk_mem = 8;//240*240*2=115200 / 8 = 14400 | for stm32f407 sram - 192Кб
//#endif
//-----------------------------------------------------------------------------------------
static void ST7789_WriteCommand(uint8_t cmd)
{
	ST7789_DC_Clr();
	if (HAL_SPI_Transmit(ipsPort, &cmd, sizeof(cmd), waits) != HAL_OK) devError |= devSPI;
}
//-----------------------------------------------------------------------------------------
static void ST7789_WriteCommands(uint8_t *cmds, size_t cnt)
{
	ST7789_DC_Clr();
	if (HAL_SPI_Transmit(ipsPort, cmds, cnt, waits * 10) != HAL_OK) devError |= devSPI;
}
//-----------------------------------------------------------------------------------------
static void ST7789_WriteData(uint8_t *buff, size_t buff_size)
{
HAL_StatusTypeDef rt = HAL_OK;

	ST7789_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
#ifdef SET_WITH_DMA
		spiRdy = false;
		rt |= HAL_SPI_Transmit_DMA(ipsPort, buff, chunk_size);
		while (!spiRdy) HAL_Delay(1);
#else
		rt |= HAL_SPI_Transmit(ipsPort, buff, chunk_size, waits);
#endif
		buff += chunk_size;
		buff_size -= chunk_size;
	}

	if (rt != HAL_OK) devError |= devSPI;
}
//-----------------------------------------------------------------------------------------
static void ST7789_WriteDataLine(uint8_t *buff, size_t line_size)
{
HAL_StatusTypeDef rt = HAL_OK;

	ST7789_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once
	//while (buff_size > 0) {
		//uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
#ifdef SET_WITH_DMA
		spiRdy = false;
		rt = HAL_SPI_Transmit_DMA(ipsPort, buff, line_size);
		while (!spiRdy) HAL_Delay(1);
#else
		rt = HAL_SPI_Transmit(ipsPort, buff, line_size, waits);
#endif
		//buff += chunk_size;
		//buff_size -= chunk_size;
	//}

	if (rt != HAL_OK) devError |= devSPI;
}
//-----------------------------------------------------------------------------------------
static void ST7789_WriteSmallData(uint8_t data)
{
	ST7789_DC_Set();
	if (HAL_SPI_Transmit(ipsPort, &data, sizeof(data), waits) != HAL_OK) devError |= devSPI;
}
//-----------------------------------------------------------------------------------------
void ipsOn(uint8_t act)
{
	if (act) ST7789_WriteCommand(ST7789_DISPON);
	    else ST7789_WriteCommand(ST7789_DISPOFF);
}
//-----------------------------------------------------------------------------------------
void ST7789_SetRotation(uint8_t m)
{
	ST7789_WriteCommand(ST7789_MADCTL);	// MADCTL
	switch (m) {
	case 0:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		break;
	case 1:
		ST7789_WriteSmallData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	case 2:
		ST7789_WriteSmallData(ST7789_MADCTL_RGB);
		break;
	case 3:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	default:
		break;
	}
}
//-----------------------------------------------------------------------------------------
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;
	
	/* Column Address set */
	ST7789_WriteCommand(ST7789_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}

	/* Row Address set */
	ST7789_WriteCommand(ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}
	/* Write to RAM */
	ST7789_WriteCommand(ST7789_RAMWR);
}
//-----------------------------------------------------------------------------------------
void ST7789_Reset()
{
	HAL_Delay(20);//25
    ST7789_RST_Clr();
    HAL_Delay(10);//25
    ST7789_RST_Set();
    HAL_Delay(20);//50
}
//-----------------------------------------------------------------------------------------
void ST7789_Init(uint16_t bkColor)
{
		
    ST7789_WriteCommand(ST7789_COLMOD);		//	Set color mode
    ST7789_WriteSmallData(ST7789_COLOR_MODE_16bit);//
  	ST7789_WriteCommand(0xB2);				//	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		ST7789_WriteData(data, sizeof(data));
	}
	ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)
	
	/* Internal LCD Voltage generator settings */
    ST7789_WriteCommand(0xB7);				//	Gate Control
    ST7789_WriteSmallData(0x35);			//	Default value
    ST7789_WriteCommand(0xBB);				//	VCOM setting
    ST7789_WriteSmallData(0x19);			//	0.725v (default 0.75v for 0x20)
    ST7789_WriteCommand(0xC0);				//	LCMCTRL	
    ST7789_WriteSmallData(0x2C);			//	Default value
    ST7789_WriteCommand(0xC2);				//	VDV and VRH command Enable
    ST7789_WriteSmallData(0x01);			//	Default value
    ST7789_WriteSmallData(0xff);            //	Default value
    ST7789_WriteCommand(0xC3);				//	VRH set
    ST7789_WriteSmallData(0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
    ST7789_WriteCommand(0xC4);				//	VDV set
    ST7789_WriteSmallData(0x20);			//	Default value
    ST7789_WriteCommand(0xC6);				//	Frame rate control in normal mode
    ST7789_WriteSmallData(0x0F);			//	Default value (60HZ)
    ST7789_WriteCommand(0xD0);				//	Power control
    ST7789_WriteSmallData(0xA4);			//	Default value
    ST7789_WriteSmallData(0xA1);			//	Default value
	/**************** Division line ****************/

	ST7789_WriteCommand(0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

    ST7789_WriteCommand(0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

  	uint8_t cmds[] = {ST7789_INVON, ST7789_SLPOUT, ST7789_NORON, ST7789_DISPOFF};
  	ST7789_WriteCommands(cmds, sizeof(cmds));


	ST7789_Fill_Color(bkColor);				//	Fill with Black.
}
//-----------------------------------------------------------------------------------------
void ST7789_Fill_Color(uint16_t color)
{
/*
#ifdef SET_WITH_DMA
	int total_buf_size = (ST7789_WIDTH * ST7789_HEIGHT);//uint16_t[57600]
	int len = total_buf_size / total_blk_mem;//14400
	frm_buf = (uint8_t *)calloc(1, len);
	if (!frm_buf) return;
	int i = -1;
	while (++i < len) {
		frm_buf[i++] = color >> 8;
		frm_buf[i++] = color & 0xff;
	}
	uint16_t blk = (total_buf_size / len) << 1;
	uint16_t blk_buf = (len / ST7789_WIDTH) >> 1;

	uint16_t cx = 0, cy = 0, cw = ST7789_WIDTH - 1, ch = ST7789_HEIGHT - 1;//blk_buf - 1;
	ST7789_SetAddressWindow(cx, cy, cw, ch);
	ST7789_DC_Set();
	for (uint16_t i = 0; i < (blk<<1); i++) {
		//ST7789_SetAddressWindow(cx, cy, cw, ch);
		spiRdy = false;
		HAL_SPI_Transmit_DMA(ipsPort, frm_buf, len);
		while (!spiRdy) HAL_Delay(1);
		//cy += blk_buf;
		//ch += blk_buf;

	}

	if (frm_buf) free(frm_buf);

#else
*/
	ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);

	uint8_t data[ST7789_WIDTH << 1];
	uint16_t i = 0, j;
	for (j = 0; j < ST7789_WIDTH; j++) {
		*(uint16_t *)(data + i) = HTONS(color);
		i += 2;
	}
	for (j = 0; j < ST7789_HEIGHT; j++) ST7789_WriteDataLine(data, sizeof(data));

//#endif
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x < 0) || (x >= ST7789_WIDTH) ||
		 (y < 0) || (y >= ST7789_HEIGHT)) return;
	
	ST7789_SetAddressWindow(x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};

	ST7789_WriteData(data, sizeof(data));
}
//-----------------------------------------------------------------------------------------
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
	if ((xEnd < 0) || (xEnd >= ST7789_WIDTH) ||
		 (yEnd < 0) || (yEnd >= ST7789_HEIGHT))	return;

	uint16_t i, j;
	uint8_t data[] = {color >> 8, color & 0xFF};
	ST7789_SetAddressWindow(xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++) {
		for (j = xSta; j <= xEnd; j++) ST7789_WriteData(data, sizeof(data));
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawPixel_4px(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x <= 0) || (x > ST7789_WIDTH) || (y <= 0) || (y > ST7789_HEIGHT))	return;

	ST7789_Fill(x - 1, y - 1, x + 1, y + 1, color);
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
	uint16_t swap;
    uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);

    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            ST7789_DrawPixel(y0, x0, color);
        } else {
            ST7789_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	ST7789_DrawLine(x1, y1, x2, y1, color);
	ST7789_DrawLine(x1, y1, x1, y2, color);
	ST7789_DrawLine(x1, y2, x2, y2, color);
	ST7789_DrawLine(x2, y1, x2, y2, color);
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(x0 + x, y0 + y, color);
		ST7789_DrawPixel(x0 - x, y0 + y, color);
		ST7789_DrawPixel(x0 + x, y0 - y, color);
		ST7789_DrawPixel(x0 - x, y0 - y, color);

		ST7789_DrawPixel(x0 + y, y0 + x, color);
		ST7789_DrawPixel(x0 - y, y0 + x, color);
		ST7789_DrawPixel(x0 + y, y0 - x, color);
		ST7789_DrawPixel(x0 - y, y0 - x, color);
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
	if ((x + w - 1) >= ST7789_WIDTH) return;
	if ((y + h - 1) >= ST7789_HEIGHT) return;

	ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	ST7789_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
}
//-----------------------------------------------------------------------------------------
void ST7789_InvertColors(uint8_t invert)
{
	ST7789_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
}
//-----------------------------------------------------------------------------------------
void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
uint32_t i, b, j;

	ST7789_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);
	uint8_t cdata[] = {color >> 8, color & 0xFF};
	uint8_t bdata[] = {bgcolor >> 8, bgcolor & 0xFF};
	uint8_t *uk = NULL;

	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) {
			if ((b << j) & 0x8000) {
				uk = cdata;
			} else {
				uk = bdata;
			}
			ST7789_WriteData(uk, sizeof(cdata));
			/*HAL_SPI_Transmit_DMA(portOLED, uk, sizeof(cdata));
			while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY) {
				if (HAL_SPI_GetState(portOLED) == HAL_SPI_STATE_BUSY_TX) break;
			}*/
		}
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	if (!str) return;

	while (*str) {
		if (x + font.width >= ST7789_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7789_HEIGHT) break;

			if (*str == ' ') {// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		if (*str != '\n') {
			ST7789_WriteChar(x, y, *str, font, color, bgcolor);
			x += font.width;
		} else {
			x = 0;
			y += font.height;
		}
		str++;
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
uint8_t i;

	/* Check input parameters */
	if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;

	/* Check width and height */
	if ((x + w) >= ST7789_WIDTH) w = ST7789_WIDTH - x;
	if ((y + h) >= ST7789_HEIGHT) h = ST7789_HEIGHT - y;

	/* Draw lines */
	for (i = 0; i <= h; i++) ST7789_DrawLine(x, y + i, x + w, y + i, color);
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	ST7789_DrawLine(x1, y1, x2, y2, color);
	ST7789_DrawLine(x2, y2, x3, y3, color);
	ST7789_DrawLine(x3, y3, x1, y1, color);
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
		yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
		curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7789_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);
	ST7789_DrawLine(x0 - r, y0, x0 + r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
		ST7789_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

		ST7789_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
		ST7789_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
	}
}
//-----------------------------------------------------------------------------------------
void ST7789_TearEffect(uint8_t tear)
{
	ST7789_WriteCommand(tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
}
//-----------------------------------------------------------------------------------------
/*
void ST7789_Test(void)
{
	ST7789_Fill_Color(WHITE);
	HAL_Delay(1000);
	ST7789_WriteString(10, 20, "Speed Test", Font_11x18, RED, WHITE);
	HAL_Delay(1000);
	ST7789_Fill_Color(CYAN);
    HAL_Delay(500);
	ST7789_Fill_Color(RED);
    HAL_Delay(500);
	ST7789_Fill_Color(BLUE);
    HAL_Delay(500);
	ST7789_Fill_Color(GREEN);
    HAL_Delay(500);
	ST7789_Fill_Color(YELLOW);
    HAL_Delay(500);
	ST7789_Fill_Color(BROWN);
    HAL_Delay(500);
	ST7789_Fill_Color(DARKBLUE);
    HAL_Delay(500);
	ST7789_Fill_Color(MAGENTA);
    HAL_Delay(500);
	ST7789_Fill_Color(LIGHTGREEN);
    HAL_Delay(500);
	ST7789_Fill_Color(LGRAY);
    HAL_Delay(500);
	ST7789_Fill_Color(LBBLUE);
    HAL_Delay(500);
	ST7789_Fill_Color(WHITE);
	HAL_Delay(500);

	ST7789_WriteString(10, 10, "Font test.", Font_16x26, GBLUE, WHITE);
	ST7789_WriteString(10, 50, "Hello Steve!", Font_7x10, RED, WHITE);
	ST7789_WriteString(10, 75, "Hello Steve!", Font_11x18, YELLOW, WHITE);
	ST7789_WriteString(10, 100, "Hello Steve!", Font_16x26, MAGENTA, WHITE);
	HAL_Delay(1000);

	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Rect./Line.", Font_11x18, YELLOW, RED);
	ST7789_DrawRectangle(30, 30, 100, 100, WHITE);
	HAL_Delay(1000);

	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Filled Rect.", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledRectangle(30, 30, 50, 50, WHITE);
	HAL_Delay(1000);


	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Circle.", Font_11x18, YELLOW, RED);
	ST7789_DrawCircle(60, 60, 25, WHITE);
	HAL_Delay(1000);

	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Filled Cir.", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledCircle(60, 60, 25, WHITE);
	HAL_Delay(1000);

	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Triangle", Font_11x18, YELLOW, RED);
	ST7789_DrawTriangle(30, 30, 30, 70, 60, 40, WHITE);
	HAL_Delay(1000);

	ST7789_Fill_Color(RED);
	ST7789_WriteString(10, 10, "Filled Tri", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledTriangle(30, 30, 30, 70, 60, 40, WHITE);
	HAL_Delay(1000);


	//	If FLASH cannot storage anymore datas, please delete codes below.
//	ST7789_Fill_Color(WHITE);
//	uint16_t x = (240 - 128)/2, y = (240 - 128)/2;
//	ST7789_DrawImage(x, y, 128, 128, (uint16_t *)saber);
//	//ST7789_DrawImage(0, 0, 240, 240, (uint16_t *)knky);
//	//ST7789_DrawImage(0, 0, 240, 240, (uint16_t *)tek);
//	//HAL_Delay(1000);
//	//ST7789_DrawImage(0, 0, 240, 240, (uint16_t *)adi1);

}
*/
//-----------------------------------------------------------------------------------------
char *mkLineCenter(char *str, uint16_t width)
{
char st[128] = {0};

	memset(st, 0x20, 127);
	int8_t k = strlen(str);
	if (k < width) {
		int8_t n = (width - k)/2;
		memcpy((char *)&st[n], (char *)str, k);
		st[k + n*2 + 1] = '\0';
		strcpy(str, st);
	}

	return str;
}
//-----------------------------------------------------------------------------------------
