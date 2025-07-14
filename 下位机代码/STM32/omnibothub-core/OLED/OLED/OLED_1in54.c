/*****************************************************************************
* | File        :   OLED_1in54.c
* | Author      :   Waveshare team
* | Function    :   1.54inch OLED Drive function
* | Info        :
*----------------
* | This version:   V1.0
* | Date        :   2020-08-13
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
#include "OLED_1in54.h"
#include <stdio.h>


/*******************************************************************************
function:
            Hardware reset
*******************************************************************************/
static void OLED_1in54_Reset(void)
{
    OLED_RST_1;
    Driver_Delay_ms(100);
    OLED_RST_0;
    Driver_Delay_ms(100);
    OLED_RST_1;
    Driver_Delay_ms(100);
}

/*******************************************************************************
function:
            Write register address and data
*******************************************************************************/
static void OLED_1in54_WriteReg(uint8_t Reg)
{

#if USE_SPI_4W
    OLED_DC_0;
    OLED_CS_0;
    SPI4W_Write_Byte(Reg);
    OLED_CS_1;
#elif USE_IIC_SOFT
	iic_start();
	iic_write_byte(0x78);
	iic_wait_for_ack();
	iic_write_byte(0x00);
	iic_wait_for_ack();
	iic_write_byte(Reg);
	iic_wait_for_ack();
	iic_stop();
#endif
}

static void OLED_1in54_WriteData(uint8_t Data)
{
    
#if USE_SPI_4W
    OLED_DC_1;
    OLED_CS_0;
    SPI4W_Write_Byte(Data);
    OLED_CS_1;
#elif USE_IIC_SOFT	
	iic_start();
	iic_write_byte(0x78);
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
static void OLED_1in54_InitReg(void)
{
    OLED_1in54_WriteReg(0xAE);//--turn off oled panel

    OLED_1in54_WriteReg(0x00);//---set low column address
    OLED_1in54_WriteReg(0x10);//---set high column address

	  OLED_1in54_WriteReg(0x20);
    OLED_1in54_WriteReg(0x00);
        
    OLED_1in54_WriteReg(0xFF);
    
    OLED_1in54_WriteReg(0xA6);
    
    OLED_1in54_WriteReg(0xA8); 
    OLED_1in54_WriteReg(0x3F);
    
    OLED_1in54_WriteReg(0xD3);
    OLED_1in54_WriteReg(0x00);
    
    OLED_1in54_WriteReg(0xD5);
    OLED_1in54_WriteReg(0x80);
    
    OLED_1in54_WriteReg(0xD9);
    OLED_1in54_WriteReg(0x22);

    OLED_1in54_WriteReg(0xDA);
    OLED_1in54_WriteReg(0x12);
    
    OLED_1in54_WriteReg(0xDB);
    OLED_1in54_WriteReg(0x40);

}

/********************************************************************************
function:
            initialization
********************************************************************************/
void OLED_1IN54_Init()
{
    //Hardware reset
    OLED_1in54_Reset();

    //Set the initialization register
    OLED_1in54_InitReg();
    Driver_Delay_ms(200);

    //Turn on the OLED display
    OLED_1in54_WriteReg(0xaf);
}

/********************************************************************************
function:
			Clear screen
********************************************************************************/
void OLED_1IN54_Clear()
{
    // UWORD Width, Height;
    UWORD i, j;
    // Width = (OLED_1IN3_WIDTH % 8 == 0)? (OLED_1IN3_WIDTH / 8 ): (OLED_1IN3_WIDTH / 8 + 1);
    // Height = OLED_1IN3_HEIGHT; 
    for (i=0; i<8; i++) {
        /* set page address */
        OLED_1in54_WriteReg(0xB0 + i);
        /* set low column address */
        OLED_1in54_WriteReg(0x00);
        /* set high column address */
        OLED_1in54_WriteReg(0x10);
        for(j=0; j<128; j++) {
            /* write data */
            OLED_1in54_WriteData(0x00);  
        }
    }
}

/********************************************************************************
function:	Update memory to OLED
********************************************************************************/
void OLED_1IN54_Display(const UBYTE *Image)
{
    UWORD page, column, temp;

    for (page=0; page<8; page++) {
        /* set page address */
        OLED_1in54_WriteReg(0xB0 + page);
        /* set low column address */
        OLED_1in54_WriteReg(0x00);
        /* set high column address */
        OLED_1in54_WriteReg(0x10);

        /* write data */
        for(column=0; column<128; column++) {
            temp = Image[(7-page) + column*8];
            OLED_1in54_WriteData(temp);
        }       
    }
}

//float radians(float radian){

//return(radian * (180/3.14));

//}
