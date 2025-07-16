#ifndef __SPI_H
#define __SPI_H
#include "include.h"

#define SPI_PI_IS_MASTER 0//SPI主发送
// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7
						  	    													  
void SPI3_Init(void);			 //初始化SPI1口
void SPI2_Init(void);			 //初始化SPI1口
u8 Spi_RW(u8 TxData);//SPI1总线读写一个字节

#define MPU9250 0
#define CS_ACCGRO 0 
#define NRF2401 1
#define MS5611  2
#define CS_FLASH 3
#define CS_LIS 4
void SPI_CS(u8 sel,u8 set);
void SPI_Receive(uint8_t *pData, uint16_t Size);
void SPI_SetSpeed(u8 SPI_BaudRatePrescaler);
#define SPI_CE_H()   GPIO_SetBits(GPIOC, GPIO_Pin_2) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define SPI_CSN_H()  SPI_CS(NRF2401,1)//GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L()  SPI_CS(NRF2401,0)//GPIO_ResetBits(GPIOB, GPIO_Pin_12) 
void master_send(void);
extern float test_spi_rx[2];
extern int spi_master_loss_pi;
extern int spi_master_connect_pi,spi_master_loss_pi_all;
#define SPI_BUF_SIZE 254  //偶数
#define SPIT_FLAG_TIMEOUT              ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT              ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))
extern unsigned char  spi_tx_buf[SPI_BUF_SIZE];
extern unsigned char  spi_rx_buf[SPI_BUF_SIZE];
extern int spi_tx_cnt;
extern int spi_rx_cnt;
void slave_rx(u8 *data_buf,u8 num);
extern char spi_flag_pi[2];
extern float spi_dt[10];
void slave_send(char sel);
extern int spi_rx_cnt_all,spi_master_connect_pi;
#endif

