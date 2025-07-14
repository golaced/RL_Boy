/*****************************************************************************
* | File      	:   OLED_0in96_rgb_test.c
* | Author      :   Waveshare team
* | Function    :   0.96inch RGB OLED Module Drive function
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2023-06-16
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "test.h"
#include "OLED_0in96_rgb.h"

int OLED_0in96_rgb_test(void)
{
	printf("0.96binch RGB OLED test demo\n");
        if(System_Init() != 0) {
        return -1;
    }
	printf("OLED_Init()...\r\n");
	OLED_0in96_rgb_Init();
	Driver_Delay_ms(500); 
	OLED_0in96_rgb_Clear();
	Driver_Delay_ms(500);   

	//1.Create a new image size
	UBYTE *BlackImage;
	
	UWORD Imagesize = OLED_0in96_rgb_WIDTH * OLED_0in96_rgb_HEIGHT * 2;
	if((BlackImage = (UBYTE *)malloc(Imagesize/2)) == NULL) {
			printf("Failed to apply for black memory...\r\n");
			return -1;
	}

	printf("Paint_NewImage\r\n");
	Paint_NewImage(BlackImage, OLED_0in96_rgb_WIDTH/2, OLED_0in96_rgb_HEIGHT, 90, BLACK);  
	Paint_SetScale(65);
	printf("Drawing\r\n");
	//1.Select Image
	Paint_SelectImage(BlackImage);
	Driver_Delay_ms(500);
	Paint_Clear(BLACK);

	printf("Brush white\r\n");
	OLED_0in96_rgb_Clear_color(white);  
	Driver_Delay_ms(1000); 

	printf("Brush red\r\n");
	OLED_0in96_rgb_Clear_color(red);  
	Driver_Delay_ms(1000); 

	printf("Brush green\r\n");
	OLED_0in96_rgb_Clear_color(green);  
	Driver_Delay_ms(1000); 

	printf("Brush blue\r\n");
	OLED_0in96_rgb_Clear_color(blue);  
	Driver_Delay_ms(1000); 

	printf("Brush black\r\n");
	OLED_0in96_rgb_Clear_color(black);  
	Driver_Delay_ms(1000);

	while(1)
	{
        // Note that when the rotation Angle is set to 90/270, 
        // the X-axis of the GUI function is opposite to the Y-axis of the display function
		Paint_NewImage(BlackImage, OLED_0in96_rgb_WIDTH/2, OLED_0in96_rgb_HEIGHT, 90, BLACK);
        Paint_SetScale(65);
        
		// 2.Drawing on the image		
		printf("Drawing:page 1\r\n");
		Paint_DrawPoint(20, 10, BLUE, DOT_PIXEL_1X1, DOT_STYLE_DFT);
		Paint_DrawPoint(30, 10, BRED, DOT_PIXEL_2X2, DOT_STYLE_DFT);
		Paint_DrawPoint(40, 10, GRED, DOT_PIXEL_3X3, DOT_STYLE_DFT);
		Paint_DrawLine(10, 10, 10, 20, GBLUE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
		Paint_DrawLine(20, 20, 20, 30, RED, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        OLED_0in96_rgb_Display_Part(BlackImage, 32, 0, 64, 128);
        Paint_Clear(BLACK);
		Paint_DrawLine(30, 10, 30, 20, MAGENTA, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
		Paint_DrawLine(40, 20, 40, 30, GREEN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
		Paint_DrawCircle(90, 16, 15, CYAN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);	
		Paint_DrawRectangle(50, 20, 60, 30, BROWN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
		OLED_0in96_rgb_Display_Part(BlackImage, 0, 0, 32, 128);
		Driver_Delay_ms(2000);			
		Paint_Clear(BLACK);

		// Drawing on the image
		printf("Drawing:page 2\r\n");
		for(UBYTE i=0; i<8; i++){
			Paint_DrawRectangle(0, 4*i, 128, 4*(i+1), i*4095, DOT_PIXEL_2X2, DRAW_FILL_FULL);
		}			
		OLED_0in96_rgb_Display_Part(BlackImage, 32, 0, 64, 128);
        for(UBYTE i=8; i<16; i++){
			Paint_DrawRectangle(0, 4*(i-8), 128, 4*(i-7), i*4095, DOT_PIXEL_2X2, DRAW_FILL_FULL);
		}
        OLED_0in96_rgb_Display_Part(BlackImage, 0, 0, 32, 128);
		Driver_Delay_ms(2000);	
		Paint_Clear(BLACK);	

		// Drawing on the image
		printf("Drawing:page 3\r\n");			
		Paint_DrawString_EN(10, 0, "waveshare", &Font12, BLACK, BLUE);
		Paint_DrawString_EN(10, 17, "hello world", &Font8, BLACK, MAGENTA);
        OLED_0in96_rgb_Display_Part(BlackImage, 32, 0, 64, 128);
        Paint_Clear(BLACK);
		Paint_DrawNum(10, 0, 123.456789, &Font8, 5, RED, BLACK);
		Paint_DrawNum(10, 15, 987654, &Font12, 4, YELLOW, BLACK);
		OLED_0in96_rgb_Display_Part(BlackImage, 0, 0, 32, 128);
		Driver_Delay_ms(2000);	
		Paint_Clear(BLACK);		

		// Drawing on the image
		printf("Drawing:page 4\r\n");
		Paint_DrawString_CN(0, 0,"ÄãºÃabc", &Font12CN, BROWN, BLACK);
        OLED_0in96_rgb_Display_Part(BlackImage, 32, 0, 64, 128);
		Driver_Delay_ms(2000);		
		Paint_Clear(BLACK);	

		printf("Drawing:page 5\r\n");
        // Show image on page5
		OLED_0in96_rgb_Display(gImage_0in96_rgb);
		Driver_Delay_ms(2000);		
		Paint_Clear(BLACK);

		printf("Brush black\r\n");
		OLED_0in96_rgb_Clear_color(black);  
		Driver_Delay_ms(5000); 
	}	
}

