#ifndef SSD1306_I2C_H
#define SSD1306_I2C_H

#define SSD1306_ADDRESS                0x78
#define SSD1306_WIDTH                  128
#define SSD1306_HEIGHT                 64
#define SSD1306_COMMAND_MODE           0x00
#define SSD1306_DATA_MODE              0x40

// �������
#define SSD1306_SET_CONTRAST           0x81
#define SSD1306_ENTIRE_DISPLAY_RESUME  0xA4
#define SSD1306_ENTIRE_DISPLAY_ON      0xA5
#define SSD1306_NORMAL_DISPLAY         0xA6
#define SSD1306_INVERSE_DISPLAY        0xA7
#define SSD1306_DISPLAY_OFF            0xAE
#define SSD1306_DISPLAY_ON             0xAF
#define SSD1306_SET_LCOL_START_ADDRESS 0x00
#define SSD1306_SET_HCOL_START_ADDRESS 0x10
#define SSD1306_MEMORY_ADDRESS_MODE    0x20
#define SSD1306_SET_COLUMN_ADDRESS     0x21
#define SSD1306_SET_PAGE_ADDRESS       0x22
#define SSD1306_SET_PAGE_START_ADDRESS 0xB0
#define SSD1306_SET_START_LINE         0x40
#define SSD1306_SEGMENT_REMAP          0xA0
#define SSD1306_SET_MULTIPLEX_RATIO    0xA8
#define SSD1306_COM_SCAN_NORMAL        0xC0
#define SSD1306_COM_SCAN_INVERSE       0xC8
#define SSD1306_SET_DISPLAY_OFFSET     0xD3
#define SSD1306_SET_COM_PINS_CONFIG    0xDA
#define SSD1306_SET_DISPLAY_CLOCK_DIV  0xD5
#define SSD1306_SET_PRECHARGE_PERIOD   0xD9
#define SSD1306_SET_VCOM_DESELECT_LVL  0xDB
#define SSD1306_NOP                    0xE3
#define SSD1306_SET_CHARGE_PUMP        0x8D

// ������ �������������
#define SSD1306_RIGHT_SCROLL_SETUP      0x26
#define SSD1306_LEFT_SCROLL_SETUP       0x27
#define SSD1306_VERT_RIGHT_SCROLL_SETUP 0x29
#define SSD1306_VERT_LEFT_SCROLL_SETUP  0x2A
#define SSD1306_ACTIVATE_SCROLL         0x2F
#define SSD1306_DEACTIVATE_SCROLL       0x2E
#define SSD1306_SET_VERT_SCROLL_AREA    0xA3

// �������� ������������� (������ - �������)
#define SSD1306_2_FRAMES   0x07
#define SSD1306_3_FRAMES   0x04
#define SSD1306_4_FRAMES   0x05
#define SSD1306_5_FRAMES   0x00
#define SSD1306_25_FRAMES  0x06
#define SSD1306_64_FRAMES  0x01
#define SSD1306_128_FRAMES 0x02
#define SSD1306_256_FRAMES 0x03

// ������� �������
void ssd1306_SendCommand(unsigned char command);
// ������� ������
void ssd1306_SendData(unsigned char *data, unsigned char count);
// ���������/������� �������
#define ssd1306_DisplayOff() ssd1306_SendCommand(SSD1306_DISPLAY_OFF)
#define ssd1306_DisplayOn()  ssd1306_SendCommand(SSD1306_DISPLAY_ON)
// �������� �������
#define ssd1306_Clear()  ssd1306_FillDisplay(0x00)
// ����������/��������� �����������
#define ssd1306_DisplayNormal()  ssd1306_SendCommand(SSD1306_NORMAL_DISPLAY)
#define ssd1306_DisplayInverce() ssd1306_SendCommand(SSD1306_INVERSE_DISPLAY)
// ���������� ��������
void ssd1306_SetContrast(unsigned char contrast);
// ��������� �������������
#define ssd1306_DeactivateScroll() ssd1306_SendCommand(SSD1306_DEACTIVATE_SCROLL)
// �������� �������������� ������������� (dir_left - ����������� [�����/������];
// start_page, end_page - ��������� � �������� �������� [0..7];
// speed - �������� �� �������)
void ssd1306_HorzScroll(unsigned char dir_left, unsigned char start_page, 
                        unsigned char end_page, unsigned char speed);
// �������� �������������� � ������������ ������������� (dir_left - ���. 
// ����������� [�����/������]; rows_fixed, rows_scroll - ���. ����. (������) � 
// ������������ �����; start_page, end_page - �������. � ������. �������� [0..7];
// speed - �������� �� �������)
void ssd1306_DiagScroll(unsigned char dir_left, unsigned char rows_fixed, 
                        unsigned char rows_scroll, unsigned char start_page, 
                        unsigned char end_page, unsigned char speed, 
                        unsigned char vert_offset);
// ���������� ������� ������ (x - �� ����������� (� ��������);
// p - �� ��������� (� ���������))
void ssd1306_SetCursor(unsigned char x, unsigned char p);
// ��������� ���� ������� ������� ��-��������� (0xFF - ������; 0x00 - ��������)
void ssd1306_FillDisplay(unsigned char data);
// ������������� ����������
void ssd1306_Init(unsigned char bus_prescale);
// ����������/�������� (clear) ������ (x,y - ���������� � ��������)
void ssd1306_DrawPixel(unsigned char x, unsigned char y, unsigned char clear);
// ����������/������� (clear) �������: (x1,y1)-(x2,y2)- ���������� � ��������
void ssd1306_DrawLine(unsigned char x1, unsigned char y1, 
                      unsigned char x2, unsigned char y2, unsigned char clear);
// ���������� ������� ��������: (x,y) - �����. (� ��������) ��-����;
// (sx,sy) - ������ � ������ � ��������; invert - ��������� �����
void ssd1306_DrawImage(unsigned char x, unsigned char y, 
                       unsigned char sx, unsigned char sy,
                       const unsigned char img[], unsigned char invert);
// ������ ������������� data: (x,y) - �����. (� ��������) ��-����;
// (sx,sy) - ������ � ������ � ��������
void ssd1306_FillRect(unsigned char x, unsigned char y, 
                      unsigned char sx, unsigned char sy, unsigned char data);
// ������� ������ ������� 6�8 �� �������� p � ��������. �����. x (� ��������);
// str[] - ������ ��� ������; invert - ��������; underline - � ��������������
void ssd1306_Draw6x8Str(unsigned char x, unsigned char p, const char str[], 
                     unsigned char invert, unsigned char underline);
// ������� ������ ������� 12�16: x, y - �����. � ��������; str[] - ������ ���
// ������; invert - ��������
void ssd1306_Draw12x16Str(unsigned char x, unsigned char y, const char str[],
                          unsigned char invert);
// ���������� �������������� ����� �� ����� (x,y) ������ sx
void ssd1306_DrawHLine(unsigned char x, unsigned char y, unsigned char sx);
// ���������� ������������ ����� �� ����� (x,y) ������ sy
void ssd1306_DrawVLine(unsigned char x, unsigned char y, unsigned char sy);
// ���������� ������������� �������������: (x,y) - �����. (� ��������) ��-����;
// (sx,sy) - ������ � ������ � ��������
void ssd1306_DrawRect(unsigned char x, unsigned char y, 
                      unsigned char sx, unsigned char sy);

#endif
