#include "test.h"
#include "OLED_1in5_b.h"


int OLED_1in5_B_test(void)
{
	Driver_Delay_ms(100);
	System_Init();
	Driver_Delay_ms(100);
  OLED_1in5_B_Init();
  Driver_Delay_ms(500);
  OLED_1in5_B_Clear();
  // 0.Create a new image cache
  UBYTE *BlackImage;
  UWORD Imagesize = ((OLED_1in5_B_WIDTH == 0) ? (OLED_1in5_B_WIDTH) : (OLED_1in5_B_WIDTH + 1)) * OLED_1in5_B_HEIGHT;
	  if ((BlackImage = (UBYTE *)malloc(Imagesize / 8)) == NULL)
  { // No enough memory
    return -1;
  }
  Paint_NewImage(BlackImage, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT, 0, BLACK);
  Paint_SetScale(2);
  // 1.Select Image
  Paint_SelectImage(BlackImage);
  Paint_Clear(BLACK);
  Driver_Delay_ms(500);
  while (1)
  {
		#if USE_SPI_4W
    OLED_1in5_B_Display_Test();// OLED display test animation
    #endif
    printf("Paint_NewImage\r\n"); 
    Paint_DrawPoint(10, 10, WHITE, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(25, 10, WHITE, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(40, 10, WHITE, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawPoint(80, 10, WHITE, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(90, 10, WHITE, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(110, 10, WHITE, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    Paint_DrawLine(10, 10, 10, 25, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(20, 10, 20, 25, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(30, 10, 30, 25, WHITE, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(40, 10, 40, 25, WHITE, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    Paint_DrawCircle(30, 16, 14, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(22, 8, 38, 24, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(80, 16, 14, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(72, 8, 88, 24, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    Paint_DrawRectangle(22, 8, 38, 24, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawRectangle(72, 8, 88, 24, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(30, 16, 14, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(80, 16, 14, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(60, 30, 65, 40, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    Paint_DrawString_EN(0, 16, "waveshare", &Font16, WHITE,BLACK);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);	
    Paint_DrawNum(10, 30, 123.456789, &Font8, 4, WHITE,BLACK);
		Paint_DrawNum(10, 43, 987654, &Font12, 5,WHITE,BLACK);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    Paint_DrawString_CN(10, 40, "ÄãºÃ", &Font12CN,WHITE,BLACK);
    Paint_DrawString_EN(45, 45, "STM32", &Font16, WHITE,BLACK);
    OLED_1in5_B_Display(BlackImage);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
    // show the array image
    OLED_1in5_B_Display(gImage_1in5_b);
    Driver_Delay_ms(2000);
    Paint_Clear(BLACK);
  }
}

