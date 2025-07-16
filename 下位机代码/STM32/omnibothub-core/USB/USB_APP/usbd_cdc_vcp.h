#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H
#include "sys.h"
#include "usbd_cdc_core.h"
#include "usbd_conf.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//usb vcp��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/2/24
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define USB_USART_REC_LEN	 	200				//USB���ڽ��ջ���������ֽ���

extern u8  USB_USART_RX_BUF[USB_USART_REC_LEN]; //���ջ���,���USB_USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USB_USART_RX_STA;   					//����״̬���	
extern u8 uart_send_usb_done;
extern int uart_send_usb_cnt;

//USB���⴮��������ò���
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
void use_uart_test(float dt);//����
void use_vcp_record(float dt);//����
void use_vcp_comm(float dt);//������ͨ��
extern int  ocu_connect;
extern float ocu_loss_cnt;
#endif 
















