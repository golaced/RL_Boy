/*****************************************************************************
* | File        :   OLED_0in96_rgb.c
* | Author      :   Waveshare team
* | Function    :   0.96inch OLED RGB Module Drive function
* | Info        :
*----------------
* | This version:   V1.0
* | Date        :   2023-5-17
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
#include "OLED_0in96_rgb.h"
#include "stdio.h"
#include "DEV_Config.h"
#include "Soft_IIC.h"
/*******************************************************************************
function:
            Hardware reset
*******************************************************************************/
static void OLED_Reset(void)
{
    OLED_RST_1;
    Driver_Delay_ms(200);
    OLED_RST_0;
    Driver_Delay_ms(200);
    OLED_RST_1;
    Driver_Delay_ms(200);
}
/*******************************************************************************
function:
            Write register address and data
*******************************************************************************/
static void OLED_WriteReg(uint8_t Reg)
{
	#if USE_SPI_4W
			OLED_DC_0;
			OLED_CS_0;
			SPI4W_Write_Byte(Reg);
			OLED_CS_1;
	#elif USE_IIC
			I2C_Write_Byte(Reg,IIC_CMD);
	#elif USE_IIC_SOFT
		iic_start();
		iic_write_byte(I2C_ADR<<1);
		iic_wait_for_ack();
		iic_write_byte(0x00);
		iic_wait_for_ack();
		iic_write_byte(Reg);
		iic_wait_for_ack();
		iic_stop();

	#endif
}

static void OLED_WriteData(uint8_t Data)
{   
	#if USE_SPI_4W
			OLED_DC_1;
			OLED_CS_0;
			SPI4W_Write_Byte(Data);
			OLED_CS_1;
	#elif USE_IIC
			I2C_Write_Byte(Data,IIC_RAM);
	#elif USE_IIC_SOFT
		iic_start();
		iic_write_byte(I2C_ADR<<1);
		iic_wait_for_ack();
		iic_write_byte(0x40);
		iic_wait_for_ack();
		iic_write_byte(Data);
		iic_wait_for_ack();
		iic_stop();

	#endif
	
}

/*******************************************************************************
function:
            Common register initialization
*******************************************************************************/
static void OLED_InitReg(void)
{
    OLED_WriteReg(0xfd);   // command lock
    OLED_WriteData(0x12);

    OLED_WriteReg(0xae);  // display off
    // OLED_WriteReg(0xa4);  # display off

    OLED_WriteReg(0xa0);  // set re-map & data format
    OLED_WriteData(0x60);     // Horizontal address increment
    OLED_WriteData(0x00);

    OLED_WriteReg(0xa1);  // set display start line
    OLED_WriteData(0x00);     // start 0 line

    OLED_WriteReg(0xa2);  // set display offset
    OLED_WriteData(0x00);

    OLED_WriteReg(0xB1);
    OLED_WriteData(0x84);

    OLED_WriteReg(0xB3);
    OLED_WriteData(0x20);

    OLED_WriteReg(0xB6);
    OLED_WriteData(0x01);

    OLED_WriteReg(0xBB);
    OLED_WriteData(0x00);

    OLED_WriteReg(0xBE);
    OLED_WriteData(0x07);

    OLED_WriteReg(0xC7);
    OLED_WriteData(0x0F);

    OLED_WriteReg(0xC1);
    OLED_WriteData(0x32);
    OLED_WriteData(0x29);
    OLED_WriteData(0x53);

    OLED_WriteReg(0xCA);
    OLED_WriteData(0x7F);
}


/********************************************************************************
function:
            initialization
********************************************************************************/
void OLED_0in96_rgb_Init()
{
    #if USE_IIC_SOFT||USE_IIC
    OLED_DC_1;
    #endif
    //Hardware reset
    OLED_Reset();
    //Set the initialization register
    OLED_InitReg();
    Driver_Delay_ms(200);

    //Turn on the OLED display
    OLED_WriteReg(0xaf);
    
    OLED_WriteReg(0x15);  // set column address
    OLED_WriteData(0x20);     // column address start 00
    OLED_WriteData(0x5f);     // column address end 127
    OLED_WriteReg(0x75);  // set row address
    OLED_WriteData(0x00);     // row address start 00
    OLED_WriteData(0x7f);     // row address end 95   
    OLED_WriteReg(0x5C); 
}


/********************************************************************************
function:
            Clear screen
********************************************************************************/
void OLED_0in96_rgb_Clear()
{    
    
    OLED_WriteReg(0x15);
    OLED_WriteData(0x20);
    OLED_WriteData(0x5f);
    OLED_WriteReg(0x75);
    OLED_WriteData(0x00);
    OLED_WriteData(0x7f);
    // fill!
    OLED_WriteReg(0x5C); 
    
    
    for (UWORD j = 0; j < OLED_0in96_rgb_HEIGHT; j++) {
        for (UWORD i = 0; i < OLED_0in96_rgb_WIDTH*2; i++) {
            OLED_WriteData(0x00);
        }
    }
}
void OLED_0in96_rgb_Clear_color(UWORD color)
{
    UWORD i;
    UBYTE temp1,temp2;
    temp1 = (color >> 8)  & 0xff;
    temp2 = color & 0xff;
    
    OLED_WriteReg(0x15);
    OLED_WriteData(0x20);
    OLED_WriteData(0x5f);
    OLED_WriteReg(0x75);
    OLED_WriteData(0x00);
    OLED_WriteData(0x7f);
    // fill!
    OLED_WriteReg(0x5C); 
    
    
    for(i=0; i<OLED_0in96_rgb_WIDTH*OLED_0in96_rgb_HEIGHT; i++){
        OLED_WriteData(temp1);
        OLED_WriteData(temp2);
    }
}


/********************************************************************************
function:	
    Update all memory to OLED
********************************************************************************/
void OLED_0in96_rgb_Display(const UBYTE *Image)
{     
    UWORD i, j, temp;
    
    OLED_WriteReg(0x15);
    OLED_WriteData(0x20);
    OLED_WriteData(0x5f);
    OLED_WriteReg(0x75);
    OLED_WriteData(0x00);
    OLED_WriteData(0x7f);
    // fill!
    OLED_WriteReg(0x5C); 
    
    for(i=0; i<OLED_0in96_rgb_WIDTH; i++)
    {
        for(j=0; j<OLED_0in96_rgb_HEIGHT*2; j++)
        {
            temp = Image[j + i*OLED_0in96_rgb_HEIGHT*2];
            OLED_WriteData(temp);
        }
    }
}



void OLED_0in96_rgb_Display_Part(const UBYTE *Image, UBYTE Xstart, UBYTE Ystart, UBYTE Xend, UBYTE Yend)
{     
    UWORD i, j, temp;
    
    OLED_WriteReg(0x15);
    OLED_WriteData(32+Xstart);
    OLED_WriteData(32+Xend-1);
    OLED_WriteReg(0x75);
    OLED_WriteData(Ystart);
    OLED_WriteData(Yend-1);
    // fill!
    OLED_WriteReg(0x5C); 
    
    for(i=0; i<Yend-Ystart; i++)
    {
        for(j=0; j<(Xend-Xstart)*2; j++)
        {
            temp = Image[j + i*(Xend-Xstart)*2];
            OLED_WriteData(temp);
        }
    }
}


