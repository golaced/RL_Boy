/*****************************************************************************
* | File      	:   OLED_0in91_test.c
* | Author      :   Waveshare team
* | Function    :   0.91inch OLED Module test demo
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2020-08-17
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
#include "OLED_0in91.h"
#include "stm32f4xx_gpio.h"
#include "usart_fc.h"

int OLED_0in91_test(void)
{
	// 0.Create a new image cache
	UBYTE *BlackImage;
	UWORD Imagesize = ((OLED_0in91_WIDTH%8==0)? (OLED_0in91_WIDTH/8): (OLED_0in91_WIDTH/8+1)) * OLED_0in91_HEIGHT;
	if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
//			printf("Failed to apply for black memory...\r\n");
			return -1;
	}
//	printf("Paint_NewImage\r\n");
	Paint_NewImage(BlackImage, OLED_0in91_HEIGHT, OLED_0in91_WIDTH, 90, BLACK);	
//	printf("Drawing\r\n");
	//1.Select Image
	Paint_SelectImage(BlackImage);
	// Delay_ms(500);
	Paint_Clear(BLACK);
	
	Paint_DrawString_EN(0, 0, "Robot System On...", &Font12, WHITE, BLACK);
	Paint_DrawString_EN(0, 18, "IP:", &Font12, WHITE, BLACK);
	Paint_DrawString_EN(25, 18, "X.X.X.X", &Font12, WHITE, BLACK);
	//Paint_DrawNum(10, 18, 192.168, &Font12, 4, WHITE, BLACK);
	OLED_0in91_Display(BlackImage);
	//Delay_ms(2000);	
	//Paint_Clear(BLACK);		

	// Drawing on the image
	//	printf("Drawing:page 3\r\n");			
  //    Paint_DrawString_CN(0, 0,"ÄãºÃAb", &Font12CN, WHITE, WHITE);
	// Show image on page3
	//OLED_0in91_Display(BlackImage);
	//Delay_ms(2000);	
	//Paint_Clear(BLACK);	

	// Drawing on the image
	//	printf("Drawing:page 4\r\n");			
	//	OLED_0in91_Display(gImage_0in91);
	// Delay_ms(2000);	
	Paint_Clear(BLACK);		
}

void OLED_Show(void)
{
	// 0.Create a new image cache
	UBYTE *BlackImage;
	UWORD Imagesize = ((OLED_0in91_WIDTH%8==0)? (OLED_0in91_WIDTH/8): (OLED_0in91_WIDTH/8+1)) * OLED_0in91_HEIGHT;
 
	Paint_NewImage(BlackImage, OLED_0in91_HEIGHT, OLED_0in91_WIDTH, 90, BLACK);	

	Paint_SelectImage(BlackImage);
	Paint_Clear(BLACK);
	//--------------------------Line1
	const char ipPrefix_system[] = "192.168";
	char system_char[10];
	snprintf(system_char, sizeof(system_char), "S:%.0f ", robot.spi_link);
	char system_char1[10];
	snprintf(system_char1, sizeof(system_char1), "E:%.0f ", robot.extcan_link);
	char system_char2[10];
	snprintf(system_char2, sizeof(system_char2), "C1:%.0f ", robot.can1_link);
	char system_char3[10];
	snprintf(system_char3, sizeof(system_char3), "C2:%.0f ", robot.can2_link);	
	
	char fullIpStr_sys[25];
	snprintf(fullIpStr_sys, sizeof(fullIpStr_sys), "%s%s%s%s", system_char, system_char1, system_char2,system_char3);
	Paint_DrawString_EN(0, 0, fullIpStr_sys, &Font12, WHITE, BLACK);

	//--------------------------Line2
	const char ipPrefix_ip[] = "192.168";

	float ipThirdPart = robot.ip1;
	float ipFourthPart = robot.ip2;

	char ipThirdStr[10];
	char ipFourthStr[10];
	snprintf(ipThirdStr, sizeof(ipThirdStr), "%.0f", ipThirdPart);
	snprintf(ipFourthStr, sizeof(ipFourthStr), "%.0f", ipFourthPart);

	char fullIpStr[25];
	snprintf(fullIpStr, sizeof(fullIpStr), "%s.%s.%s", ipPrefix_ip, ipThirdStr, ipFourthStr);
		
	Paint_DrawString_EN(0, 18, "IP:", &Font12, WHITE, BLACK);
	Paint_DrawString_EN(25, 18, fullIpStr, &Font12, WHITE, BLACK);

	OLED_0in91_Display(BlackImage);

	Paint_Clear(BLACK);		
}
