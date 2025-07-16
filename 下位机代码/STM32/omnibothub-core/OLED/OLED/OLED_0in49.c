/*****************************************************************************
* | File        :   OLED_0in49.c
* | Author      :   Waveshare team
* | Function    :   0.49inch OLED Module Drive function
* | Info        :
*----------------
* | This version:   V1.0
* | Date        :   2023-05-31
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
#include "OLED_0in49.h"
#include "stdio.h"
#include "DEV_Config.h"
#include "Soft_IIC.h"
/*******************************************************************************
function:
            Write register address and data
*******************************************************************************/
static void OLED_WriteReg(uint8_t Reg)
{
	#if USE_IIC
			I2C_Write_Byte(Reg, IIC_CMD);
	#elif USE_IIC_SOFT
			iic_start();
			iic_write_byte(I2C_ADR << 1);
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
	#if USE_IIC
			I2C_Write_Byte(Data, IIC_RAM);
	#elif USE_IIC_SOFT
			iic_start();
			iic_write_byte(I2C_ADR << 1);
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
    OLED_WriteReg(0xAE); /*display off*/

    OLED_WriteReg(0x00); /*set lower column address*/
    OLED_WriteReg(0x12); /*set higher column address*/

    OLED_WriteReg(0x00); /*set display start line*/

    OLED_WriteReg(0xB0); /*set page address*/

    OLED_WriteReg(0x81); /*contract control*/
    OLED_WriteReg(0x4f); /*128*/

    OLED_WriteReg(0xA0); /*set segment remap*/

    OLED_WriteReg(0xA6); /*normal / reverse*/

    OLED_WriteReg(0xA8); /*multiplex ratio*/
    OLED_WriteReg(0x1F); /*duty = 1/32*/

    OLED_WriteReg(0xC8); /*Com scan direction*/

    OLED_WriteReg(0xD3); /*set display offset*/
    OLED_WriteReg(0x00);

    OLED_WriteReg(0x20);
    OLED_WriteReg(0x01); /*set Vertical Addressing Mode*/

    OLED_WriteReg(0xD5); /*set osc division*/
    OLED_WriteReg(0x80);

    OLED_WriteReg(0xD9); /*set pre-charge period*/
    OLED_WriteReg(0xf1);

    OLED_WriteReg(0xDA); /*set COM pins*/
    OLED_WriteReg(0x12);

    OLED_WriteReg(0xdb); /*set vcomh*/
    OLED_WriteReg(0x40);

    OLED_WriteReg(0x8d); /*set charge pump enable*/
    OLED_WriteReg(0x14);

    OLED_WriteReg(0xAF); /*display ON*/
}

/********************************************************************************
function:
            initialization
********************************************************************************/
void OLED_0in49_Init()
{
    System_Init();
    // Hardware reset
    OLED_InitReg();
}

/********************************************************************************
function:
            Clear screen
********************************************************************************/
void OLED_0in49_Clear(void)
{
    UWORD i;
    OLED_WriteReg(0x22); // 设置页面地址
    OLED_WriteReg(0x00);
    OLED_WriteReg(0x03);
    OLED_WriteReg(0x21); // 设置列地址
    OLED_WriteReg(0x20);
    OLED_WriteReg(0x5f);
    for (i = 0; i < 256; i++)
    {
        OLED_WriteData(0x00);
    }
}

/********************************************************************************
function:
            reverse a byte data
********************************************************************************/
static UBYTE reverse(UBYTE temp)
{
    temp = ((temp & 0x55) << 1) | ((temp & 0xaa) >> 1);
    temp = ((temp & 0x33) << 2) | ((temp & 0xcc) >> 2);
    temp = ((temp & 0x0f) << 4) | ((temp & 0xf0) >> 4);
    return temp;
}

/********************************************************************************
function:
    Update all memory to OLED
********************************************************************************/
void OLED_0in49_Display(const UBYTE *Image)
{
    UBYTE temp;
    UWORD i;
    OLED_WriteReg(0x22);
    OLED_WriteReg(0x00);
    OLED_WriteReg(0x03);
    OLED_WriteReg(0x21);
    OLED_WriteReg(0x20);
    OLED_WriteReg(0x5f);
    for (i = 0; i < 256; i++)
    {
        temp = Image[i];
        temp = reverse(temp); // reverse the buffer
        OLED_WriteData(temp);
    }
}
