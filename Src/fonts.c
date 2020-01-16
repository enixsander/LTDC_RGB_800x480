#include "main.h"
#include "fonts.h"
#include "fonts_8x8.h"
#include "fonts_8x14.h"
#include "fonts_8x15_rus.h"
#include "fonts_8x16.h"
#include <string.h>

extern DMA2D_HandleTypeDef hdma2d;

typedef struct {
	uint8_t b,g,r;
} typeRGB;

typeRGB rgb = {0,0,0};

void draw_font8x14(const char *string, uint16_t x, uint16_t y, uint32_t color_text, uint32_t color_background) {

	#define SYMB_LENGTH 8
	#define NUMBER_OF_LINES 14
	uint32_t start_pos = (WIDTH * y + x) * 3;
	typeRGB letter[NUMBER_OF_LINES * SYMB_LENGTH] = {0};

	//Output Offset
	DMA2D->OOR = WIDTH - SYMB_LENGTH;

	for(uint16_t symbol = 0; symbol < strlen(string); symbol++) 
	{
		for (uint8_t i = 0; i < NUMBER_OF_LINES; i++) 
		{
			uint8_t byte = seabios8x14[string[symbol] * NUMBER_OF_LINES + i];
			uint8_t mask = 0x80;
			for (uint8_t offset = 0; offset < 8; offset++) 
			{
				uint8_t position = (SYMB_LENGTH * i) +  offset;
				if(byte & mask)
					set_pixel_color(color_text);
				else
					set_pixel_color(color_background);
				letter[position] = rgb;
				mask >>= 1;
			}
		}
		if (HAL_DMA2D_Start(&hdma2d, (uint32_t)letter, SDRAM_BANK_ADDR + start_pos, SYMB_LENGTH, NUMBER_OF_LINES) == HAL_OK)
	  {
	    HAL_DMA2D_PollForTransfer(&hdma2d, 10);
	  }
	  start_pos += SYMB_LENGTH * 3;	//длина символа
	}
}

void set_pixel_color(uint32_t color) {
	rgb.r = (uint8_t)(color >> 16);
	rgb.g = (uint8_t)(color >> 8);
	rgb.b = (uint8_t)color;
}
