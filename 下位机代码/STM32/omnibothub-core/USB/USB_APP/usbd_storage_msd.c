include "usbd_msc_mem.h"
#include "usb_conf.h"
#include "w25qxx.h"			 
#include "sdio_sdcard.h"	 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//USB读卡器  驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/7/21
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	   
 
//最大支持的设备数,2个
#define STORAGE_LUN_NBR 	2

////////////////////////////自己定义的一个标记USB状态的寄存器///////////////////
//bit0:表示电脑正在向SD卡写入数据
//bit1:表示电脑正从SD卡读出数据
//bit2:SD卡写数据错误标志位
//bit3:SD卡读数据错误标志位
//bit4:1,表示电脑有轮询操作(表明连接还保持着)
vu8 USB_STATUS_REG=0;
////////////////////////////////////////////////////////////////////////////////


//USB Mass storage 标准查询数据(每个lun占36字节)
const int8_t  STORAGE_Inquirydata[] = { 
  
	/* LUN 0 */ 
	0x00,		
	0x80,		
	0x02,		
	0x02,
	(USBD_STD_INQUIRY_LENGTH - 4),
	0x00,
	0x00,	
	0x00,
    /* Vendor Identification */
    'A', 'L', 'I', 'E', 'N', 'T', 'E', 'K', ' ',//9字节
    /* Product Identification */
    'S', 'P', 'I', ' ', 'F', 'l', 'a', 's', 'h',//15字节
    ' ','D', 'i', 's', 'k', ' ',
    /* Product Revision Level */	
    '1', '.', '0', ' ',							//4字节		
	
	/* LUN 1 */
	0x00,
	0x80,		
	0x02,		
	0x02,
	(USBD_STD_INQUIRY_LENGTH - 4),
	0x00,
	0x00,	
	0x00,
	/* Vendor Identification */
	'A', 'L', 'I', 'E', 'N', 'T', 'E', 'K',' ',	//9字节
	/* Product Identification */				
	'S', 'D', ' ', 'F', 'l', 'a', 's', 'h', ' ',//15字节
	'D', 'i', 's', 'k', ' ', ' ',  
    /* Product Revision Level */
	'1', '.', '0' ,' ',                      	//4字节
}; 
int8_t STORAGE_Init (uint8_t lun);
int8_t STORAGE_GetCapacity (uint8_t lun,uint32_t *block_num,uint32_t *block_size);
int8_t  STORAGE_IsReady (uint8_t lun);
int8_t  STORAGE_IsWriteProtected (uint8_t lun);
int8_t STORAGE_Read (uint8_t lun,uint8_t *buf,uint32_t blk_addr,uint16_t blk_len);
int8_t STORAGE_Write (uint8_t lun,uint8_t *buf,uint32_t blk_addr,uint16_t blk_len);
int8_t STORAGE_GetMaxLun (void);

//USB Device 用户回调函数接口
USBD_STORAGE_cb_TypeDef USBD_MICRO_SDIO_fops =
{
	STORAGE_Init,
	STORAGE_GetCapacity,
	STORAGE_IsReady,
	STORAGE_IsWriteProtected,
	STORAGE_Read,
	STORAGE_Write,
	STORAGE_GetMaxLun,
	(int8_t *)STORAGE_Inquirydata,
};
USBD_STORAGE_cb_TypeDef  *USBD_STORAGE_fops=&USBD_MICRO_SDIO_fops;//指向USBD_MICRO_SDIO_fops结构体.

//初始化存储设备
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//返回值:0,成功;
//    其他,错误代码
int8_t STORAGE_Init (uint8_t lun)
{
	SD_Init();
	W25QXX_Init();
	return 0; 
} 

//获取存储设备的容量和块大小
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//block_num:块数量(扇区数)
//block_size:块大小(扇区大小)
//返回值:0,成功;
//    其他,错误代码
int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{     
	if(lun==1)
	{
		*block_size=512;  
		*block_num=SDCardInfo.CardCapacity/512;
	}else
	{
		*block_size=512;  
		*block_num=1024*1024*12/512;	//SPI FLASH的前面12M字节,文件系统用
	}	
	return 0; 
} 

//查看存储设备是否就绪
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//返回值:0,就绪;
//    其他,未就绪
int8_t  STORAGE_IsReady (uint8_t lun)
{ 
	USB_STATUS_REG|=0X10;//标记轮询
	return 0;
} 

//查看存储设备是否写保护
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//返回值:0,没有写保护;
//    其他,写保护(只读)
int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
	return  0;
} 

//从存储设备读取数据
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//buf:数据存储区首地址指针
//blk_addr:要读取的地址(扇区地址)
//blk_len:要读取的块数(扇区数) 
//返回值:0,成功;
//    其他,错误代码 
int8_t STORAGE_Read (uint8_t lun,uint8_t *buf,uint32_t blk_addr,uint16_t blk_len)
{
	int8_t res=0;
	USB_STATUS_REG|=0X02;//标记正在读数据
	if(lun==1)
	{
 		res=SD_ReadDisk(buf,blk_addr,blk_len);
		if(res)USB_STATUS_REG|=0X08;//SD卡读错误!
	}else
	{
		W25QXX_Read(buf,blk_addr*512,blk_len*512);
	} 
	return res;
}
//向存储设备写数据
//lun:逻辑单元编号,0,SD卡;1,SPI FLASH
//buf:数据存储区首地址指针
//blk_addr:要写入的地址(扇区地址)
//blk_len:要写入的块数(扇区数) 
//返回值:0,成功;
//    其他,错误代码 
int8_t STORAGE_Write (uint8_t lun,uint8_t *buf,uint32_t blk_addr,uint16_t blk_len) 
{
	int8_t res=0;
	USB_STATUS_REG|=0X01;//标记正在写数据
	if(lun==1)
	{
		res=SD_WriteDisk (buf,blk_addr,blk_len);
		if(res)USB_STATUS_REG|=0X04;//SD卡写错误!	 
	}else
	{
		W25QXX_Write(buf,blk_addr*512,blk_len*512);
	} 
	return res; 
}
//获取支持的最大逻辑单元个数
//返回值:支持的逻辑单元个数-1
int8_t STORAGE_GetMaxLun (void)
{
	return (STORAGE_LUN_NBR - 1);
}
























