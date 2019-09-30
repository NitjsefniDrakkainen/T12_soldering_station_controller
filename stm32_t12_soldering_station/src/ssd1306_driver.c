/*
 * ssd1306_driver.c
 *
 *  Created on: 16.09.2019
 *      Author: pciechelski
 */

#include "stm32f10x.h"
#include "i2c_interface.h"
#include "ssd1306_driver.h"
#include "m_snprintf.h"

static SSD1306_t SSD1306;
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
#if 0
static uint8_t Init_Table[]=
{
      SSD1306_DISPLAYOFF,
      SSD1306_SETLOWCOLUMN,
      SSD1306_SETHIGHCOLUMN,
      SSD1306_SETPAGESTARTADDRESS,
      SSD1306_SETSTARTLINE,
      SSD1306_SEGREMAP | 0x01,
      SSD1306_SETCOMPINS,
      0x02,   /* Set com pins data. */
      SSD1306_SETDISPLAYOFFSET,
      0x00,   /* Set display offset data. No offset. */
      SSD1306_COMSCANDEC,
      SSD1306_NORMALDISPLAY,
      SSD1306_DISPLAYALLON_RESUME,
      SSD1306_SETCONTRAST,
      0x01,   /* Set contrast data. */
      SSD1306_MEMORYMODE,
      0x00,   /* Memory addressing mode data. Horizontal addressing. */
      SSD1306_SETMULTIPLEX,
      0x1F,   /* Set MUX ratio data. 1/32 duty cycle. */
      SSD1306_SETPRECHARGE,
      0xF1,   /* Set pre-charge period data. */
      SSD1306_SETVCOMDESELECT,
      0x40,   /* Set V com deselect data. */
      SSD1306_CHARGEPUMP,
      0x14,   /* Charge pump setting data. */
      SSD1306_DISPLAYON,
};
#endif
/*
static void ssd1306_Reset(void)
{

}
*/
static void ssd1306_WriteCommand(uint8_t byte) {
	uint8_t data[3];
	data[0] = 0x80;
	data[1] = byte;

	I2C_StartTransmission(I2C1, I2C_Direction_Transmitter, SSD1306_I2C_ADDR);
	I2C_WriteData(I2C1, data[0]);

	I2C_WriteData(I2C1, data[1]);

	I2C_GenerateSTOP(I2C1, ENABLE);
}

static void ssd1306_WriteData(uint8_t* buffer, uint8_t buff_size) {
	int i;
	const uint8_t* data = (uint8_t*) buffer;
	I2C_StartTransmission(I2C1, I2C_Direction_Transmitter, SSD1306_I2C_ADDR);
	I2C_WriteData(I2C1, 0x40);
	for (i = 0; i < buff_size; ++i) {
		I2C_WriteData(I2C1, data[i]);
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

// Screen object
static SSD1306_t SSD1306;

// Initialize the oled screen
void ssd1306_Init(void) {
	volatile uint32_t delay_t = 0;
	/*TODO delay function*/
	while (1) {
		if ( ++delay_t > 1400000 ) {
			break;
		}
	}

	delay_t = 0;

    // Init OLED
    ssd1306_WriteCommand(0xAE); //display off

    ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
    ssd1306_WriteCommand(0x10); // 00,Horizontal Addressing Mode; 01,Vertical Addressing Mode;
                                // 10,Page Addressing Mode (RESET); 11,Invalid

    ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
    ssd1306_WriteCommand(0xC0); // Mirror vertically
#else
    ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

    ssd1306_WriteCommand(0x00); //---set low column address
    ssd1306_WriteCommand(0x10); //---set high column address

    ssd1306_WriteCommand(0x40); //--set start line address - CHECK

    ssd1306_WriteCommand(0x81); //--set contrast control register - CHECK
    ssd1306_WriteCommand(0x0F);

#ifdef SSD1306_MIRROR_HORIZ
    ssd1306_WriteCommand(0xA0); // Mirror horizontally
#else
    ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
    ssd1306_WriteCommand(0xA7); //--set inverse color
#else
    ssd1306_WriteCommand(0xA6); //--set normal color
#endif

    ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
    ssd1306_WriteCommand(0x3F); //

    ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssd1306_WriteCommand(0xD3); //-set display offset - CHECK
    ssd1306_WriteCommand(0x00); //-not offset

    ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
    ssd1306_WriteCommand(0xF0); //--set divide ratio

    ssd1306_WriteCommand(0xD9); //--set pre-charge period
    ssd1306_WriteCommand(0x22); //

    ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
    ssd1306_WriteCommand(0x12);

    ssd1306_WriteCommand(0xDB); //--set vcomh
    ssd1306_WriteCommand(0x20); //0x20,0.77xVcc

    ssd1306_WriteCommand(0x8D); //--set DC-DC enable
    ssd1306_WriteCommand(0x14); //

    /*
    ssd1306_WriteCommand(0xD4); // Set Display Clock Divide Ratio / OSC Frequency
    ssd1306_WriteCommand(0x80); // Display Clock Divide Ratio / OSC Frequency

    ssd1306_WriteCommand(0xA8); // Set Multiplex Ratio
    ssd1306_WriteCommand(0x3F); // Multiplex Ratio for 128x64 (64-1)

    ssd1306_WriteCommand(0xD3); // Set Display Offset
    ssd1306_WriteCommand(0x00); // Display Offset

    ssd1306_WriteCommand(0x40); // Set Display Start Line

    ssd1306_WriteCommand(0x8D); // Set Charge Pump
    ssd1306_WriteCommand(0x14); // Charge Pump (0x10 External, 0x14 Internal DC/DC)

    ssd1306_WriteCommand(0xA1); // Set Segment Re-Map
    ssd1306_WriteCommand(0xC8); // Set Com Output Scan Direction

    ssd1306_WriteCommand(0xDA); // Set COM Hardware Configuration
    ssd1306_WriteCommand(0x12); // COM Hardware Configuration

    ssd1306_WriteCommand(0x81); // Set Contrast
    ssd1306_WriteCommand(0xCF); // Contrast

    ssd1306_WriteCommand(0xD9); // Set Pre-Charge Period
    ssd1306_WriteCommand(0xF1); // Set Pre-Charge Period (0x22 External, 0xF1 Internal)

    ssd1306_WriteCommand(0xDB); // Set VCOMH Deselect Level
    ssd1306_WriteCommand(0x40); // VCOMH Deselect Level

    ssd1306_WriteCommand(0xA4); // Set all pixels OFF
    ssd1306_WriteCommand(0xA6); // Set display not inverted
	*/
    ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel

    // Clear screen
    ssd1306_Fill(Black);

    // Flush buffer to screen
    ssd1306_UpdateScreen();

    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    SSD1306.Initialized = 1;
}

// Fill the whole screen with the given color
void ssd1306_Fill(SSD1306_COLOR color) {
    /* Set memory */
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

// Write the screenbuffer with changed to the screen
void ssd1306_UpdateScreen(void) {
    uint8_t i;
    uint16_t n;
    for(i = 0; i < 8; i++) {
        ssd1306_WriteCommand(0xB0 + i);
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0x10);
        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
    }
    for (n = 0; n < (sizeof(SSD1306_Buffer)/sizeof(SSD1306_Buffer[0])); ++n) {
    	SSD1306_Buffer[n] = 0;
    }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if(SSD1306.Inverted) {
        color = (SSD1306_COLOR)!color;
    }

    // Draw in the right color
    if(color == White) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

// Draw 1 char to the screen buffer
// ch         => char om weg te schrijven
// Font     => Font waarmee we gaan schrijven
// color     => Black or White
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color) {
    uint32_t i, b, j;

    // Check remaining space on current line
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for(i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for(j = 0; j < Font.FontWidth; j++) {
            if((b << j) & 0x8000)  {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
            } else {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;

    // Return written char for validation
    return ch;
}

// Write full string to screenbuffer
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color) {
    // Write until null-byte
    while (*str) {
        if (ssd1306_WriteChar(*str, Font, color) != *str) {
            // Char could not be written
            return *str;
        }

        // Next char
        str++;
    }

    // Everything ok
    return *str;
}

// Position the cursor
void ssd1306_SetCursor(uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

