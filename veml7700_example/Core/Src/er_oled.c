//adapted by David Ramsay for STM32

/***************************************************
//Web: http://www.buydisplay.com
EastRising Technology Co.,LTD
****************************************************/

#include "er_oled.h"
#include "stm32wbxx_hal_i2c.h"
#include "stm32wbxx_hal.h"


void I2C_Write_Byte(uint8_t value, uint8_t Cmd)
{
  uint16_t Addr = 0x3C << 1;
  uint8_t Data[2];
  Data[0] = Cmd;
  Data[1] = value;

  HAL_I2C_Master_Transmit(&ER_OLED_I2C_PORT, Addr, Data, 2, HAL_MAX_DELAY);
}

void er_oled_begin()
{
    command(0xae);//--turn off oled panel

    command(0xd5);//--set display clock divide ratio/oscillator frequency
    command(0x80);//--set divide ratio

    command(0xa8);//--set multiplex ratio
    command(0x27);//--1/40 duty

    command(0xd3);//-set display offset
    command(0x00);//-not offset

    command(0xad);//--Internal IREF Setting
    command(0x30);//--

    command(0x8d);//--set Charge Pump enable/disable
    command(0x14);//--set(0x10) disable

    command(0x40);//--set start line address

    command(0xa6);//--set normal display

    command(0xa4);//Disable Entire Display On

    command(0xa1);//--set segment re-map 128 to 0

    command(0xC8);//--Set COM Output Scan Direction 64 to 0

    command(0xda);//--set com pins hardware configuration
    command(0x12);

    command(0x81);//--set contrast control register
    command(0xaf);

    command(0xd9);//--set pre-charge period
    command(0x22);

    command(0xdb);//--set vcomh
    command(0x20);

    command(0xaf);//--turn on oled panel

}

void er_oled_clear(uint8_t* buffer)
{
	int i;
	for(i = 0;i < WIDTH * HEIGHT / 8;i++)
	{
		buffer[i] = 0;
	}
}

void er_oled_pixel(int x, int y, char color, uint8_t* buffer)
{
    if(x > WIDTH || y > HEIGHT)return ;
    if(color)
        buffer[x+(y/8)*WIDTH] |= 1<<(y%8);
    else
        buffer[x+(y/8)*WIDTH] &= ~(1<<(y%8));
}

void er_oled_char1616(uint8_t x, uint8_t y, uint8_t chChar, uint8_t* buffer)
{
	uint8_t i, j;
	uint8_t chTemp = 0, y0 = y, chMode = 0;

	for (i = 0; i < 32; i++) {
		chTemp = Font1612[chChar - 0x30][i];
		for (j = 0; j < 8; j++) {
			chMode = chTemp & 0x80? 1 : 0;
			er_oled_pixel(x, y, chMode, buffer);
			chTemp <<= 1;
			y++;
			if ((y - y0) == 16) {
				y = y0;
				x++;
				break;
			}
		}
	}
}

void er_oled_char(unsigned char x, unsigned char y, char acsii, char size, char mode, uint8_t* buffer)
{
    unsigned char i, j, y0=y;
    uint16_t temp;
    uint16_t position = 0x80;
    uint8_t maxindex = 8;
    unsigned char ch = acsii - ' ';

    if (size == 32){
    	position = 0x8000;
    	maxindex =16;
    }

    for(i = 0;i<size;i++) {
        if(size == 12)
        {
            if(mode)temp = Font1206[ch][i];
            else temp = ~Font1206[ch][i];
        }
        else if(size == 16)
        {
            if(mode)temp = Font1608[ch][i];
            else temp = ~Font1608[ch][i];
        }
        else {
            if(mode) {
            	temp = Font3216[ch - 16][2*i] << 8;
                temp |= Font3216[ch-16][2*i + 1];
            }
            else {
            	temp = ~Font3216[ch - 16][2*i] << 8;
            	temp |= ~Font3216[ch - 16][2*i + 1];
            }
        }
        for(j =0;j<maxindex;j++)
        {
            if(temp & position) er_oled_pixel(x, y, 1, buffer);
            else er_oled_pixel(x, y, 0, buffer);
            temp <<= 1;
            y++;
            if((y-y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void er_oled_string(uint8_t x, uint8_t y, const char *pString, uint8_t Size, uint8_t Mode, uint8_t* buffer)
{
    while (*pString != '\0') {
        if (x > (WIDTH - Size / 2)) {
            x = 0;
            y += Size;
            if (y > (HEIGHT - Size)) {
                y = x = 0;
            }
        }

        er_oled_char(x, y, *pString, Size, Mode, buffer);
        x += Size / 2;
        pString++;
    }
}

void er_oled_char3216(uint8_t x, uint8_t y, uint8_t chChar, uint8_t* buffer)
{
    uint8_t i, j;
    uint8_t chTemp = 0, y0 = y, chMode = 0;

    for (i = 0; i < 64; i++) {
        chTemp = Font3216[chChar - 0x30][i];
        for (j = 0; j < 8; j++) {
            chMode = chTemp & 0x80? 1 : 0;
            er_oled_pixel(x, y, chMode, buffer);
            chTemp <<= 1;
            y++;
            if ((y - y0) == 32) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void er_oled_bitmap(uint8_t x,uint8_t y,const uint8_t *pBmp, uint8_t chWidth, uint8_t chHeight, uint8_t* buffer)
{
	uint8_t i, j, byteWidth = (chWidth + 7)/8;
	for(j = 0;j < chHeight;j++){
		for(i = 0;i <chWidth;i++){
			if((pBmp[j * byteWidth + i / 8]) & (128 >> (i & 7))){
				er_oled_pixel(x + i,y + j, 1, buffer);
			}
		}
	}
}

void er_oled_display(uint8_t* pBuf)
{    uint8_t page,i;
    for (page = 0; page < PAGES; page++) {
        command(0xB0 + page);/* set page address */
        command(0x0c);   /* set low column address */
        command(0x11);  /* set high column address */
        for(i = 0; i< WIDTH; i++ ) {
          data(pBuf[i+page*WIDTH]);// write data one
        }
    }
}

void er_oled_time(const char *pString)
{
	uint8_t oled_buf[WIDTH * HEIGHT / 8];

	er_oled_clear(oled_buf);
    er_oled_char( 0, 4, *pString++,  32, 1, oled_buf);
    er_oled_char(16, 4, *pString++ , 32, 1, oled_buf);
    er_oled_char(40, 4, *pString++ , 32, 1, oled_buf);
    er_oled_char(56, 4, *pString   , 32, 1, oled_buf);

    er_oled_pixel(36, 12, 1, oled_buf);
	er_oled_pixel(36, 13, 1, oled_buf);
	er_oled_pixel(36, 14, 1, oled_buf);
	er_oled_pixel(36, 28, 1, oled_buf);
	er_oled_pixel(36, 27, 1, oled_buf);
	er_oled_pixel(36, 26, 1, oled_buf);

	er_oled_display(oled_buf);
}

void er_oled_print_2digit(int value){
  uint8_t oled_buf[WIDTH * HEIGHT / 8] = {0};
  char c = (char) ( ((int) '0') + (value % 10));
  char d = (char) ( ((int) '0') + (value / 10));
  er_oled_char(27, 14, d, 12, 1, oled_buf);
  er_oled_char(36, 14, c, 12, 1, oled_buf);
  er_oled_display(oled_buf);

}

void er_oled_print_3digit(int value){
  uint8_t oled_buf[WIDTH * HEIGHT / 8] = {0};
  char c = (char) ( ((int) '0') + (value % 10));
  char d = (char) ( ((int) '0') + ((value % 100) / 10));
  char e = (char) ( ((int) '0') + (value / 100));
  er_oled_char(27, 14, e, 12, 1, oled_buf);
  er_oled_char(36, 14, d, 12, 1, oled_buf);
  er_oled_char(45, 14, c, 12, 1, oled_buf);
  er_oled_display(oled_buf);

}
