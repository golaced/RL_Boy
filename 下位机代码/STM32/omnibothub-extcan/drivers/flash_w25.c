#include "flash_w25.h"    
#include "include.h"
#include "spi.h"
#include "flash.h"    
////////////////////////////////////////////////////////////////////////////////// 	
 
u16 W25QXX_TYPE=W25Q32;	//Ĭ����W25Q128

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector 
													 
//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{ 

	SPI_CS(CS_FLASH,1);
	W25QXX_TYPE=W25QXX_ReadID();	//��ȡFLASH ID.
  if(FLASH_USE_STM32||W25QXX_TYPE==W25Q128||W25QXX_TYPE==W25Q32||W25QXX_TYPE==0xC815||W25QXX_TYPE==0xEF15||W25QXX_TYPE!=0xFFFF)
		module.flash=1;
}  

//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	SPI_CS(CS_FLASH,0);                          //ʹ������   
	Spi_RW(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=Spi_RW(0Xff);             //��ȡһ���ֽ�  
	SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(u8 sr)   
{   
	SPI_CS(CS_FLASH,0);                          //ʹ������   
	Spi_RW(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
	Spi_RW(sr);               //д��һ���ֽ�  
	SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_WriteEnable);      //����дʹ��  
	SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_WriteDisable);     //����д��ָֹ��    
	SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	SPI_CS(CS_FLASH,0);		    
	Spi_RW(0x90);//���Ͷ�ȡID����	    
	Spi_RW(0x00); 	    
	Spi_RW(0x00); 	    
	Spi_RW(0x00); 	 			   
	Temp|=Spi_RW(0xFF)<<8;  
	Temp|=Spi_RW(0xFF);	 
	SPI_CS(CS_FLASH,1);			    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_ReadData);         //���Ͷ�ȡ����   
    Spi_RW((u8)((ReadAddr)>>16));  //����24bit��ַ    
    Spi_RW((u8)((ReadAddr)>>8));   
    Spi_RW((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=Spi_RW(0XFF);   //ѭ������  
    }
	SPI_CS(CS_FLASH,1);				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
	W25QXX_Write_Enable();                  //SET WEL 
	SPI_CS(CS_FLASH,0);                          //ʹ������   
	Spi_RW(W25X_PageProgram);      //����дҳ����   
	Spi_RW((u8)((WriteAddr)>>16)); //����24bit��ַ    
	Spi_RW((u8)((WriteAddr)>>8));   
	Spi_RW((u8)WriteAddr);   
	for(i=0;i<NumByteToWrite;i++)Spi_RW(pBuffer[i]);//ѭ��д��  
	SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
		W25QXX_Write_Enable();                  //SET WEL 
		W25QXX_Wait_Busy();   
		SPI_CS(CS_FLASH,0);                          //ʹ������   
		Spi_RW(W25X_ChipErase);        //����Ƭ��������  
		SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
		W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
// 	printf("fe:%x\r\n",Dst_Addr);	  
 	  Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_SectorErase);      //������������ָ�� 
    Spi_RW((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    Spi_RW((u8)((Dst_Addr)>>8));   
    Spi_RW((u8)Dst_Addr);  
	  SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				   //�ȴ��������
}  
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_PowerDown);        //���͵�������  
	  SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
    Delay_us(3);                               //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	SPI_CS(CS_FLASH,0);                          //ʹ������   
    Spi_RW(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	  SPI_CS(CS_FLASH,1);                          //ȡ��Ƭѡ     	      
    Delay_us(3);                               //�ȴ�TRES1
}   
