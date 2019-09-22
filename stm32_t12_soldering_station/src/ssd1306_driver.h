/*
 * ssd1306_driver.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __SSD1306_DRIVER_H__
#define __SSD1306_DRIVER_H__

#include "ssd1306_fonts.h"
/*
#define SLAVE_ADDRESS                        0x78
#define SSD1306_WIDTH                           132
#define SSD1306_HEIGHT                           64
#define SSD1306_COLOR_BRIGHT                     0xFF
#define SSD1306_COLOR_DARK                        0x00
*/
/* SSD1306 commands. */

/*
#define SSD1306_SETLOWCOLUMN                      0x00
#define SSD1306_EXTERNALVCC                         0x01
#define SSD1306_SWITCHCAPVCC                      0x02
#define SSD1306_SETHIGHCOLUMN                      0x10
#define SSD1306_MEMORYMODE                         0x20
#define SSD1306_COLUMNADDR                         0x21
#define SSD1306_PAGEADDR                           0x22
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL                0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL       0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL       0x2A
#define SSD1306_DEACTIVATE_SCROLL                   0x2E
#define SSD1306_ACTIVATE_SCROLL                      0x2F
#define SSD1306_SETSTARTLINE                      0x40
#define SSD1306_SETCONTRAST                      0x81
#define SSD1306_CHARGEPUMP                         0x8D
#define SSD1306_SEGREMAP                         0xA0
#define SSD1306_SET_VERTICAL_SCROLL_AREA                0xA3
#define SSD1306_DISPLAYALLON_RESUME                   0xA4
#define SSD1306_DISPLAYALLON                      0xA5
#define SSD1306_NORMALDISPLAY                      0xA6
#define SSD1306_INVERTDISPLAY                      0xA7
#define SSD1306_SETMULTIPLEX                      0xA8
#define SSD1306_DISPLAYOFF                         0xAE
#define SSD1306_DISPLAYON                         0xAF
#define SSD1306_SETPAGESTARTADDRESS                  0xB0
#define SSD1306_COMSCANINC                         0xC0
#define SSD1306_COMSCANDEC                         0xC8
#define SSD1306_SETDISPLAYOFFSET                      0xD3
#define SSD1306_SETDISPLAYCLOCKDIV                   0xD5
#define SSD1306_SETPRECHARGE                      0xD9
#define SSD1306_SETCOMPINS                         0xDA
#define SSD1306_SETVCOMDESELECT                     0xDB
*/

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        (0x3C << 1)
//#define SSD1306_I2C_ADDR        0x78
#endif

// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           132
// some LEDs don't display anything in first two columns
 //#define SSD1306_WIDTH           130
#endif



// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;


// Procedure definitions
void ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);


#endif /* __SSD1306_DRIVER_H__ */
