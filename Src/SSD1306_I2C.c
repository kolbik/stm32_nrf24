#include <stm32f1xx_hal.h>
#include <stdlib.h>
#include <string.h>
#include "SSD1306_I2C.h"
#include "font6x8.h"
#include "font12x16.h"

extern I2C_HandleTypeDef hi2c1;

unsigned char ssd1306_buf1[SSD1306_WIDTH + 1];
unsigned char ssd1306_buf2[2];

void swap(unsigned char *x, unsigned char *y)
{
  unsigned char t;

  t = *x;
  *x = *y;
  *y = t;
}

void ssd1306_SendCommand(unsigned char command)
{
#if 0
	I2C2TRN = addr;		//Адрес микросхемы
	I2C2TRN = reg;		//Адрес регистра
	I2C2TRN = data1;		//Данные
#endif
	uint8_t mas[2];

	mas[0] = SSD1306_COMMAND_MODE;
	mas[1] = command;
	HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDRESS, mas, 2, 100);
}

void ssd1306_SendData(unsigned char *data, unsigned char count)
{ 
	HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDRESS, data, count, 100);
}

void ssd1306_SetContrast(unsigned char contrast)
{
  ssd1306_SendCommand(SSD1306_SET_CONTRAST);
  ssd1306_SendCommand(contrast);
}

void ssd1306_SetCursor(unsigned char x, unsigned char p)
{
  ssd1306_SendCommand(SSD1306_SET_LCOL_START_ADDRESS | (x & 0x0F));
  ssd1306_SendCommand(SSD1306_SET_HCOL_START_ADDRESS | (x >> 4));
  ssd1306_SendCommand(SSD1306_SET_PAGE_START_ADDRESS | p);
}

void ssd1306_FillDisplay(unsigned char data)
{
  unsigned char page, x;
  
  ssd1306_buf1[0] = SSD1306_DATA_MODE;      // fist item "send data mode"
  for (page=0; page<8; page++)
  {	
    ssd1306_SetCursor(0, page);     
    for (x=0; x<SSD1306_WIDTH; x++)
    {
      ssd1306_buf1[x + 1] = data;
    };
    ssd1306_SendData(ssd1306_buf1, SSD1306_WIDTH + 1);
  };
  ssd1306_SetCursor(0, 0);

}

void ssd1306_DrawPixel(unsigned char x, unsigned char y, unsigned char clear)
{
  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) return;
  ssd1306_SetCursor(x, y >> 3); 
  ssd1306_buf2[0] = SSD1306_DATA_MODE;
  if (clear)
    ssd1306_buf2[1] = ~(1 << (y & 7));
  else
    ssd1306_buf2[1] = (1 << (y & 7));
  ssd1306_SendData(ssd1306_buf2, 2);
}

void ssd1306_DrawLine(unsigned char x1, unsigned char y1, unsigned char x2, 
                      unsigned char y2, unsigned char clear)
{
  unsigned char dx, dy, i, check, x, y;
  signed char sx, sy;
  signed int e;

  dx = abs(x1 - x2);
  dy = abs(y1 - y2);
  if (x1 < x2) sx = 1; else sx = -1;
  if (y1 < y2) sy = 1; else sy = -1;
  x = x1;
  y = y1;
  check = 0;
  if (dy > dx)
  {
    swap(&dx, &dy);
    check = 1;
  }
  e = 2 * dy - dx;
  for (i=0; i<dx; i++)
  {
    ssd1306_DrawPixel(x, y, clear);
    if (e >= 0)
    {  
      if (check == 1) x += sx; else y += sy;
      e -= 2 * dx;
    };
    if (check == 1) y += sy; else x += sx;
    e += 2 * dy;
  };
}

void ssd1306_DrawHLine(unsigned char x, unsigned char y, unsigned char sx)
{
  unsigned char p, n, i;
  
  if (((x + sx) > SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) return;
  
  p = y / 8;                  // page index
  n = y % 8;                  // offset form begin of page
  
  ssd1306_buf1[0] = SSD1306_DATA_MODE; // fist item "send data mode"  
  ssd1306_SetCursor(x, p);
  for (i=0; i<sx; i++)
  {
    ssd1306_buf1[i + 1] = (1 << n);
  };
  ssd1306_SendData(ssd1306_buf1, sx + 1); // send the buf to display
  
}

void ssd1306_DrawVLine(unsigned char x, unsigned char y, unsigned char sy)
{
  unsigned char p, p0, p1, n, n1, b;
  
  if ((x >= SSD1306_WIDTH) || ((y + sy) > SSD1306_HEIGHT)) return;
  
  p0 = y / 8;                  // page index
  p1 = (y + sy - 1) / 8;       // last page index
  n = y % 8;                   // offset form begin of page
  n1 = (y + sy) % 8;
  if (n1) n1 = 8 - n1;
  for (p=p0; p<(p1+1); p++)    // for each page...
  {
    if (p == p0) // first page
    {
      b = (0xFF << n) & 0xFF;
    } else if (p == p1) // last page
    {
      b = (0xFF >> n1) & 0xFF;
    } else       // internal pages
    {
      b = 0xFF;
    };
    ssd1306_SetCursor(x, p);
    ssd1306_buf2[0] = SSD1306_DATA_MODE;
    ssd1306_buf2[1] = b;
    ssd1306_SendData(ssd1306_buf2, 2);
  };
}

void ssd1306_DrawRect(unsigned char x, unsigned char y, unsigned char sx, 
                      unsigned char sy)
{
  ssd1306_DrawVLine(x, y, sy);
  ssd1306_DrawVLine(x + sx - 1, y, sy);
  // горизонтальные отрезки короче на 1 пикс. с каждой стороны
  ssd1306_DrawHLine(x + 1, y, sx - 2);
  ssd1306_DrawHLine(x + 1, y + sy - 1, sx - 2);
}

// x,y - top left coordinates; sx,sy - width and height of image
void ssd1306_DrawImage(unsigned char x, unsigned char y, unsigned char sx, 
                       unsigned char sy, const unsigned char img[], 
                       unsigned char invert)
{
  unsigned int j, t;
  unsigned char i, p, p0, p1, n, n1, b;

  if (((x + sx) > SSD1306_WIDTH) || ((y + sy) > SSD1306_HEIGHT) || 
      (sx == 0) || (sy == 0)) return;

  // Total bytes of the image array
  if (sy % 8)
    t = (sy / 8 + 1) * sx;
  else
    t = (sy / 8) * sx;
  p0 = y / 8;                 // first page index
  p1 = (y + sy - 1) / 8;      // last page index
  n = y % 8;                  // offset form begin of page
  
  n1 = (y + sy) % 8;
  if (n1) n1 = 8 - n1;
  
  j = 0;                      // bytes counter [0..t], or [0..(t+sx)]
  ssd1306_buf1[0] = SSD1306_DATA_MODE; // fist item "send data mode" 
  for (p=p0; p<(p1+1); p++)   // for each page...
  {
    ssd1306_SetCursor(x, p);   
    for (i=x; i<(x+sx); i++)
    {
      if (p == p0) // first page
      {
        b = (img[j] << n) & 0xFF;
      } else if ((p == p1) && (j >= t)) // last page
      {
        b = (img[j - sx] >> n1) & 0xFF;
      } else       // internal pages
      {
        b = ((img[j - sx] >> (8 - n)) & 0xFF) | ((img[j] << n) & 0xFF);
      };
      if (invert)
        ssd1306_buf1[i - x + 1] = ~b;
      else
        ssd1306_buf1[i - x + 1] = b;
      j++;
    };
    ssd1306_SendData(ssd1306_buf1, sx + 1); // send the buf to display
  };

}

// x,y - top left coordinates; sx,sy - width and height of rect
void ssd1306_FillRect(unsigned char x, unsigned char y, unsigned char sx, 
                      unsigned char sy, unsigned char data)
{
  unsigned char i, p, p0, p1;
  
  if (((x + sx) > SSD1306_WIDTH) || ((y + sy) > SSD1306_HEIGHT) || 
      (sx == 0) || (sy == 0)) return;
  p0 = y / 8;                 // first page index
  p1 = (y + sy - 1) / 8;      // last page index

  ssd1306_buf1[0] = SSD1306_DATA_MODE; // fist item "send data mode"   
  for (p=p0; p<(p1+1); p++)   // for each page...
  {
    ssd1306_SetCursor(x, p);
    for (i=x; i<(x+sx); i++)
    {
       ssd1306_buf1[i - x + 1] = data;
    };
    ssd1306_SendData(ssd1306_buf1, sx + 1); // send the buf to display   
  };

}

void ssd1306_Draw6x8Str(unsigned char x, unsigned char p, const char str[], 
                        unsigned char invert, unsigned char underline)
{
  unsigned char i, j, b, buf[FONT6X8_WIDTH + 1];
  unsigned int c;
  
  i = 0;
  buf[0] = SSD1306_DATA_MODE; // fist item "send data mode"    
  while (str[i] != '\0')
  {
    if (str[i] > 191)
      c = (str[i] - 64) * FONT6X8_WIDTH;
    else
      c = str[i] * FONT6X8_WIDTH;
    if (x > (SSD1306_WIDTH - FONT6X8_WIDTH - 1))
    {
      x = 0;
      p++;
    };
    if (p > 7) p = 0;
    ssd1306_SetCursor(x, p);
    for (j=0; j<FONT6X8_WIDTH; j++)
    {
      if (underline)
        b = font6x8[(unsigned int)(c + j)] | 0x80;
      else
        b = font6x8[(unsigned int)(c + j)];
      if (invert)
        buf[j + 1] = ~b;
      else
        buf[j + 1] = b;
    };
    ssd1306_SendData(buf, FONT6X8_WIDTH + 1); // send the buf to display
    x += FONT6X8_WIDTH;
    i++;
  };
}

void ssd1306_Draw12x16Str(unsigned char x, unsigned char y, const char str[], 
                          unsigned char invert)
{
  unsigned char i;
  unsigned int c;

  i = 0;
  while (str[i] != '\0')
  {
    if (str[i] > 191)
      c = (str[i] - 64) * FONT12X16_WIDTH * 2;
    else
      c = str[i] * FONT12X16_WIDTH * 2;
    ssd1306_DrawImage(x, y, 12, 16, (unsigned char *) &font12x16[c], invert);
    i++;
    x += 12;
  };
}

void ssd1306_HorzScroll(unsigned char dir_left, unsigned char start_page, 
                        unsigned char end_page, unsigned char speed)
{
  if (dir_left)
    ssd1306_SendCommand(SSD1306_LEFT_SCROLL_SETUP);
  else
    ssd1306_SendCommand(SSD1306_RIGHT_SCROLL_SETUP);
  ssd1306_SendCommand(0x00);  // dummy
  ssd1306_SendCommand(start_page);
  ssd1306_SendCommand(speed);
  ssd1306_SendCommand(end_page);
  ssd1306_SendCommand(0x00);  // dummy
  ssd1306_SendCommand(0xFF);  // dummy
  ssd1306_SendCommand(SSD1306_ACTIVATE_SCROLL);
}

void ssd1306_DiagScroll(unsigned char dir_left, unsigned char rows_fixed, 
                        unsigned char rows_scroll, unsigned char start_page, 
                        unsigned char end_page, unsigned char speed, 
                        unsigned char vert_offset)
{
  ssd1306_SendCommand(SSD1306_SET_VERT_SCROLL_AREA);        
  ssd1306_SendCommand(rows_fixed);
  ssd1306_SendCommand(rows_scroll);
  if (dir_left)
    ssd1306_SendCommand(SSD1306_VERT_LEFT_SCROLL_SETUP);
  else
    ssd1306_SendCommand(SSD1306_VERT_RIGHT_SCROLL_SETUP);
  ssd1306_SendCommand(0x00); 	// dummy
  ssd1306_SendCommand(start_page);
  ssd1306_SendCommand(speed);
  ssd1306_SendCommand(end_page);
  ssd1306_SendCommand(vert_offset);
  ssd1306_SendCommand(SSD1306_ACTIVATE_SCROLL);
}

void ssd1306_Init(unsigned char bus_prescale)
{
  ssd1306_SendCommand(SSD1306_DISPLAY_OFF);
  ssd1306_SendCommand(SSD1306_SET_DISPLAY_CLOCK_DIV);
  ssd1306_SendCommand(0x80);  
  ssd1306_SendCommand(SSD1306_SET_MULTIPLEX_RATIO);
  ssd1306_SendCommand(0x3F);
  ssd1306_SendCommand(SSD1306_SET_DISPLAY_OFFSET);
  ssd1306_SendCommand(0x00);
  ssd1306_SendCommand(SSD1306_SET_START_LINE | 0x00);
  ssd1306_SendCommand(SSD1306_SET_CHARGE_PUMP);
  ssd1306_SendCommand(0x14);
  ssd1306_SendCommand(SSD1306_MEMORY_ADDRESS_MODE);
  ssd1306_SendCommand(SSD1306_SET_LCOL_START_ADDRESS);
  ssd1306_SendCommand(SSD1306_SEGMENT_REMAP | 0x01);
  ssd1306_SendCommand(SSD1306_COM_SCAN_INVERSE);
  ssd1306_SendCommand(SSD1306_SET_COM_PINS_CONFIG);
  ssd1306_SendCommand(0x12);
  ssd1306_SendCommand(SSD1306_SET_CONTRAST);
  ssd1306_SendCommand(0xC8);
  ssd1306_SendCommand(SSD1306_SET_PRECHARGE_PERIOD);
  ssd1306_SendCommand(0xF1);
  ssd1306_SendCommand(SSD1306_SET_VCOM_DESELECT_LVL);
  ssd1306_SendCommand(0x40);
  ssd1306_SendCommand(SSD1306_ENTIRE_DISPLAY_RESUME);
  ssd1306_SendCommand(SSD1306_NORMAL_DISPLAY);
  ssd1306_SendCommand(SSD1306_DISPLAY_ON);

  ssd1306_SetCursor(0,0);

  ssd1306_FillDisplay(0x01);
}

