#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H
#include "sys.h"
#include "usbd_cdc_core.h"
#include "usbd_conf.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//usb vcp驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/2/24
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define USB_USART_REC_LEN	 	200				//USB串口接收缓冲区最大字节数

extern u8  USB_USART_RX_BUF[USB_USART_REC_LEN]; //接收缓冲,最大USB_USART_REC_LEN个字节.末字节为换行符 
extern u16 USB_USART_RX_STA;   					//接收状态标记	
extern u8 uart_send_usb_done;
extern int uart_send_usb_cnt;

//USB虚拟串口相关配置参数
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;
 
void Anal_USB(u8 *data_buf,u8 num);
float floatFromData(unsigned char *data,int* anal_cnt);
int intFromData(unsigned char *data,int* anal_cnt);
char charFromData(unsigned char *data,int* anal_cnt);
uint16_t VCP_Init     (void);
uint16_t VCP_DeInit   (void);
uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
uint16_t VCP_DataTx   (uint8_t data);
uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);
void usb_printf(char* fmt,...); 

void use_vcp_test(float dt);
void use_bldc_test(float dt);
void use_uart_test(float dt);//发送
void use_vcp_record(float dt);//发送
void use_vcp_comm(float dt);//控制器通信
extern int  ocu_connect;
extern float ocu_loss_cnt;
#endif 
















