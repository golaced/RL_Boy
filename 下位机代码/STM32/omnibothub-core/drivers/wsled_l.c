#include "stm32f4xx.h"
#include "include.h"
#include "wsled.h"
 
RGB_Color  rgb_color;
HSV_Color  hsv_color;
_WS_LED wsled;
float led_k=1.0;
int base_led_delay[3]={3,35,280};
void delay(unsigned int us)  //300ns
{
	while(us--);
}

void init_gpio_ws(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;            //GPIO结构体设置

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;          //LED9 在PF9引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //初始化的复用功能（因为本引脚还要用到PWM输出功能）
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	wsled.light_percent[0]=0;
	wsled.light_percent[1]=0;
	
	for(int i=0;i<2;i++)
		for(int j=0;j<5;j++)
			wsled.rgb[i][j]=C_Black;
}

//--------------------------
void ws_set(u8 sel){
if(sel)
GPIO_SetBits(GPIOB,GPIO_Pin_0);
else
GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

void Reset(void)
{
	ws_set(0);
	delay_us(base_led_delay[2]);
}
void T0()
{
	ws_set(1);
	delay(base_led_delay[0]*led_k);
	ws_set(0);
	delay_us(base_led_delay[1]*led_k);
}

void T1()
{
	ws_set(1);
	delay(base_led_delay[1]*led_k);
	ws_set(0);
	delay(base_led_delay[0]*led_k);
}

void send_dat(uint8_t dat) 
{ 
   uint8_t i;
   for(i=8;i>0;i--) 
   {
     if(dat&0x80)
        T1();
     else
        T0();
     dat <<=1;
   }
}

 
void Continuous_Set_LED(uint8_t num,uint32_t GRB)
{

	while(num--)
	{
		send_dat((GRB>>16)&0xFF);
		send_dat((GRB>>8)&0xFF);
		send_dat((GRB>>0)&0xFF);
	}	Reset();
}

void All_Set_LED(uint8_t num,uint32_t GRB)
{
	Continuous_Set_LED(5,0x000000);
	Continuous_Set_LED(num,GRB);
}

void Chose_Set_LED(uint8_t num,uint32_t GRB)
{
	Continuous_Set_LED(5,0x000000);
	Continuous_Set_LED(num,GRB);
	Continuous_Set_LED(num-1,0x000000);
}

void Chose_Zoom_Set_LED(uint8_t num,uint32_t GRB[5])
{
	Continuous_Set_LED(5,0x000000);
	uint8_t i=0;
	for(i=0;i<num;i++)
	{
		Chose_Set_LED(i,GRB[i]);
	}
	Chose_Set_LED(num,GRB[i]);
}

void Continuous_Set_LED_G(uint8_t num,uint32_t GRB[5])
{
	int temp=num;
 
	while(temp--)
	{
		send_dat((GRB[temp]>>16)&0xFF);
		send_dat((GRB[temp]>>8)&0xFF);
		send_dat((GRB[temp]>>0)&0xFF);
	}
	Reset();
}

//----------------------LED Rgiht side---

void ws_set1(u8 sel){
if(sel)
GPIO_SetBits(GPIOB,GPIO_Pin_1);
else
GPIO_ResetBits(GPIOB,GPIO_Pin_1);
}

void Reset1(void)
{
	ws_set1(0);
	delay_us(base_led_delay[2]);
}
void T01()
{
	ws_set1(1);
	delay(base_led_delay[0]*led_k);
	ws_set1(0);
	delay(base_led_delay[1]*led_k);
}

void T11()
{
	ws_set1(1);
	delay(base_led_delay[1]*led_k);
	ws_set1(0);
	delay(base_led_delay[0]*led_k);
}

void send_dat1(uint8_t dat) 
{ 
   uint8_t i;
   for(i=8;i>0;i--) 
   {
     if(dat&0x80)
        T11();
     else
        T01();
     dat <<=1;
   }
}


void Continuous_Set_LED1(uint8_t num,uint32_t GRB)
{
 
	while(num--)
	{
		send_dat1((GRB>>16)&0xFF);
		send_dat1((GRB>>8)&0xFF);
		send_dat1((GRB>>0)&0xFF);
	}
	Reset1();
}

void All_Set_LED1(uint8_t num,uint32_t GRB)
{
	Continuous_Set_LED1(5,0x000000);
	Continuous_Set_LED1(num,GRB);
}

void Chose_Set_LED1(uint8_t num,uint32_t GRB)
{
	Continuous_Set_LED1(5,0x000000);
	Continuous_Set_LED1(num,GRB);
	Continuous_Set_LED1(num-1,0x000000);
}

void Chose_Zoom_Set_LED1(uint8_t num,uint32_t GRB[5])
{
	Continuous_Set_LED1(5,0x000000);
	uint8_t i=0;
	for(i=0;i<num;i++)
	{
		Chose_Set_LED1(i,GRB[i]);
	}
	Chose_Set_LED1(num,GRB[i]);
}

void Continuous_Set_LED_G1(uint8_t num,uint32_t GRB[5])
{
	int temp=num;
 
	while(temp--)
	{
		send_dat1((GRB[temp]>>16)&0xFF);
		send_dat1((GRB[temp]>>8)&0xFF);
		send_dat1((GRB[temp]>>0)&0xFF);
	}
	Reset1();
}

int test_ws(void)
{
	uint32_t GRB1[5]={C_Red,C_Red,C_Red,C_Red,C_Red};
	uint32_t GRB2[5]={C_Red,C_Red,C_Red,C_Red,C_Red};
	init_gpio_ws();

	Continuous_Set_LED_G(WS2812_LED_NUM,GRB1);
	Continuous_Set_LED_G1(WS2812_LED_NUM,GRB2);
	
	Continuous_Set_LED_G(WS2812_LED_NUM,GRB1);
	Continuous_Set_LED_G1(WS2812_LED_NUM,GRB2);
	//Chose_Zoom_Set_LED(5,C_Red);
	//Continuous_Set_LED(5,C_Red);
}


void wslled_loop(float dt)
{
	static float time[2]={0};
	static int flag[2]={1,1};
	uint32_t GRB1[5]={C_Red,C_Red,C_Red,C_Red,C_Red};
	uint32_t GRB2[5]={C_Red,C_Red,C_Red,C_Red,C_Red};
	RGB_Color temp;
	if(wsled.led_mode[0])
	{
		wsled.light_percent[0]=1;
		time[0]+=dt;
		if(time[0]>1){
			time[0]=0;
			flag[0]=!flag[0];
		}
		for(int i=0;i<5;i++)
			if(flag[0]){
				wsled.light_percent[0]=1;
				wsled.rgb[0][i]=C_Yellow;
			}
			else{
				wsled.light_percent[0]=0;
				wsled.rgb[0][i]=C_Indigo;
			}
	}else
	{
		wsled.light_percent[0]=0;
	}
	
	
	for(int i=0;i<5;i++){
		temp.G=(wsled.rgb[0][i]>>16);
		temp.R=(wsled.rgb[0][i]>>8);
		temp.B=(wsled.rgb[0][i]>>0);
		temp.l=1;
		__brightnessAdjust(wsled.light_percent[0],temp);
		
		GRB1[i]=(rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B);
	}
	
	if(wsled.led_mode[1])
	{
		wsled.light_percent[1]=1;
		time[1]+=dt;
		if(time[1]>1){
			time[1]=0;
			flag[1]=!flag[1];
		}
		for(int i=0;i<5;i++)
			if(flag[1]){
				wsled.light_percent[1]=1;
				wsled.rgb[1][i]=C_Yellow;
			}
			else{
				wsled.light_percent[1]=0;
				wsled.rgb[1][i]=C_Indigo;
			}
	}else
	{
		wsled.light_percent[1]=0;
	}
	for(int i=0;i<5;i++){
		temp.G=(wsled.rgb[1][i]>>16);
		temp.R=(wsled.rgb[1][i]>>8);
		temp.B=(wsled.rgb[1][i]>>0);
		temp.l=0;
		__brightnessAdjust(wsled.light_percent[1],temp);
		
		GRB2[i]=(rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B);
	} 
 

	wsled.led_lock=1;
	Continuous_Set_LED_G(WS2812_LED_NUM,GRB1);Continuous_Set_LED_G1(WS2812_LED_NUM,GRB2);
	wsled.led_lock=0;
}

//----------------------------------------------------------
//将三原色单独数据合并为24位数据
uint32_t ws281x_color(uint8_t red, uint8_t green, uint8_t blue)
{
  return green << 16 | red << 8 | blue;
}

/**
 * @Description  	WS2812 设置第n个灯珠的颜色
* @Param     n:第几个灯珠   red:0-255   green:0-255    blue:0-255 	   eg:yellow:255 255 0
 * @Return    	  
*/

//设定第n个灯珠的颜色
void ws281x_setPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t i;
  
//  if(n < WS2812_LED_NUM)
//  {
//    for(i = 0; i < 24; ++i)
//    {
//      ws2812_data_buffer[n][i] = (((ws281x_color(red,green,blue) << i) & 0X800000) ? SIG_1 : SIG_0);
//    }
//  }
//	//ws2812_Send_Data();
	delay_ms(10);
}


/**
 * @Description  	WS2812 设置灯珠颜色（固定的）
* @Param       		 n:第几个灯珠   color:哪种颜色（0-7）
 * @Return    	  
*/
void set_pixel_rgb(uint16_t n,u8 color)
{
	switch(color)
	{
		case Red: 
			ws281x_setPixelRGB(n,255,0,0);
			break;
		case Green: 
			ws281x_setPixelRGB(n,0,255,0);
			break;
		case Blue: 
			ws281x_setPixelRGB(n,0,0,255);
			break;
		case Yellow: 
			ws281x_setPixelRGB(n,255,255,0);
			break;
		case Purple: 
			ws281x_setPixelRGB(n,255,0,255);
			break;
		case Orange: 
			ws281x_setPixelRGB(n,255,125,0);
			break;
		case Indigo: 
			ws281x_setPixelRGB(n,0,255,255);
			break;
		case White:
			ws281x_setPixelRGB(n,255,255,255);
			break;
	
	}

}



//设置关闭第n个灯珠
void ws281x_ShutoffPixel(uint16_t n)
{
  uint8_t i;
  
//  if(n < WS2812_LED_NUM)
//  {
//    for(i = 0; i < 24; ++i)
//    {
//      ws2812_data_buffer[n][i] = SIG_0;
//    }
//  }
	//ws2812_Send_Data();
	delay_ms(10);
}



/**
 * @Description  	WS2812关闭所有灯光		1. 发送WS2812_LED_NUM * 24位的 0 码
																
 * @Param     	  {void}
 * @Return    	  {void}
*/
void ws2812_AllShutOff(void){
	uint16_t i;
  uint8_t j;
  
//  for(i = 0; i < WS2812_LED_NUM; i++)
//  {
//    for(j = 0; j < 24; j++)
//    {
//      ws2812_data_buffer[i][j] = SIG_0;
//    }
//  }
  //ws2812_Send_Data();
	delay_ms(10*WS2812_LED_NUM);
}


/**
 * @Description  	WS2812设置某一位的LED的颜色 但不发送
 * @Param     	  {uint16_t LED_index ,uint32_t GRB_color}
 * @Return    	  {void}
*/
void ws2812_Set_one_LED_Color(uint16_t LED_index ,uint32_t GRB_color){
  uint8_t i = 0;
	uint32_t cnt = 0x800000;
  if(LED_index < WS2812_LED_NUM){
//    for(i = 0; i < 24; ++i){
//			if(GRB_color & cnt){
//				ws2812_data_buffer[LED_index][i] = SIG_1;
//			}
//			else{
//				ws2812_data_buffer[LED_index][i] = SIG_0;
//			}
//			cnt >>= 1;
//    }
  }
}


/**
 * @Description  	WS2812 色环转化 0-255灰度值转换为GRB值
 * @Param     	  {uint8_t LED_gray}
 * @Return    	  {uint32_t}
*/
uint32_t ws2812_LED_Gray2GRB(uint8_t LED_gray){
	LED_gray = 0xFF - LED_gray;
	if(LED_gray < 85){
		return (((0xFF - 3 * LED_gray)<<8) | (3 * LED_gray));
	}
	if(LED_gray < 170){
		LED_gray = LED_gray - 85;
		return (((3 * LED_gray)<<16) | (0xFF - 3 * LED_gray));
	}
	LED_gray = LED_gray - 170;
	return (((0xFF - 3 * LED_gray)<<16) | ((3 * LED_gray)<<8));
}


/**
 * @Description  	WS2812 灰度值驱动渐变效果 沿色环转动
 * @Param     	  {uint16_t interval_time} 渐变间隔时间
 * @Return    	  {void}
*/
void ws2812_Roll_on_Color_Ring(uint16_t interval_time){
	uint8_t i = 0;
	uint16_t j = 0;
	for(i = 0;i <= 255;i++){
		for(j = 0;j < WS2812_LED_NUM;j++){
			ws2812_Set_one_LED_Color(j, ws2812_LED_Gray2GRB(i));
		}
		//ws2812_Send_Data();
		delay_ms(interval_time);
	}
}

/**
 * @Description  	WS2812 单色呼吸灯 暗->亮->暗
 * @Param     	  {uint16_t interval_time, uint32_t GRB_color} 渐变间隔时间
 * @Return    	  {void}
*/
void ws2812_All_LED_one_Color_breath(uint16_t interval_time, uint32_t GRB_color){
	uint8_t i = 0;
	uint16_t j = 0;
	rgb_color.G = GRB_color>>16;
	rgb_color.R = GRB_color>>8;
	rgb_color.B = GRB_color;
	for(i=1;i<=100;i++){
		__brightnessAdjust(i/100.0f, rgb_color);
		for(j=0;j<WS2812_LED_NUM;j++){
			ws2812_Set_one_LED_Color(j, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
		}
		//ws2812_Send_Data();
		delay_ms(interval_time);
	}
	for(i=100;i>=1;i--){
		__brightnessAdjust(i/100.0f, rgb_color);
		for(j=0;j<WS2812_LED_NUM;j++){
			ws2812_Set_one_LED_Color(j, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
		}
		//ws2812_Send_Data();
		delay_ms(interval_time);
	}
}

/**
 * @Description  	跑马灯效果
* @Param     interval_time:间隔时间
 * @Return    	NONE  
*/
void horse_race_lamp(uint16_t interval_time)
{
	u8 i,color;
	
	
  for(i = 0; i < WS2812_LED_NUM; i++)
  {
//		ws281x_setPixelRGB(i,255,255,0);
		color = rand()%7;
		set_pixel_rgb(i,color);//随机颜色
		ws281x_ShutoffPixel(i-1);
		delay_ms(interval_time);
  }
	ws281x_ShutoffPixel(WS2812_LED_NUM-1);
	delay_ms(interval_time);
}


/**
 * @Description  	流水灯效果
* @Param     interval_time:间隔时间  red:0-255 green:0-255 blue:0-255
 * @Return    	NONE  
*/
void Running_water_lamp( uint8_t red ,uint8_t green ,uint8_t blue, uint16_t interval_time )
{
	uint16_t i;
  
  for(i = 0; i < WS2812_LED_NUM; i++)
  {
		ws281x_setPixelRGB(i,red,green,blue);
		delay_ms(interval_time);
  }
	ws2812_AllShutOff();
	delay_ms(interval_time);
}
 



/**
 * @Description  	随机点亮RGB灯
* @Param     interval_time:间隔时间
 * @Return    	NONE  
*/
uint8_t tmp_flag[WS2812_LED_NUM];


void srand_lamp(uint16_t interval_time)
{
	static uint8_t tmp,i;
	uint8_t k,color;

	tmp = rand()%(WS2812_LED_NUM);
	color = rand()%7;
	if(i==0) //只做一次
	{
		memset(tmp_flag,50,WS2812_LED_NUM);
		tmp_flag[i] = tmp;
		set_pixel_rgb(tmp,color);
		delay_ms(interval_time);
		i++;
	
	}
	else if(i>=WS2812_LED_NUM)
	{
		return ;
	}
		
	for(k=0;k<i;k++)
	{
		if(tmp == tmp_flag[k])//相同就退出
		{
			return ;
		}
		
	}

	//遍历完成
	tmp_flag[i] = tmp;
	set_pixel_rgb(tmp,color);
	delay_ms(interval_time);
	i++;


}






/***********************************************************
										Private Function
************************************************************/
/**
 * @Description  	获得两数最大值
 * @Param     	  {float a,float b}
 * @Return    	  {float}
*/
float __getMaxValue(float a, float b){
	return a>=b?a:b;
}

/**
 * @Description  	获得两数最小值
 * @Param     	  {void}
 * @Return    	  {void}
*/
float __getMinValue(float a, float b){
	return a<=b?a:b;
}


/**
 * @Description  	RGB 转为 HSV
 * @Param     	  {RGB_Color RGB, HSV_Color *HSV}
 * @Return    	  {void}
*/
void __RGB_2_HSV(RGB_Color RGB, HSV_Color *HSV){
	float r,g,b,minRGB,maxRGB,deltaRGB;
	
	r = RGB.R/255.0f;
	g = RGB.G/255.0f;
	b = RGB.B/255.0f;
	maxRGB = __getMaxValue(r, __getMaxValue(g,b));
	minRGB = __getMinValue(r, __getMinValue(g,b));
	deltaRGB = maxRGB - minRGB;
	
	HSV->V = deltaRGB;
	if(maxRGB != 0.0f){
		HSV->S = deltaRGB / maxRGB;
	}
	else{
		HSV->S = 0.0f;
	}
	if(HSV->S <= 0.0f){
		HSV->H = 0.0f;
	}
	else{
		if(r == maxRGB){
			HSV->H = (g-b)/deltaRGB;
    }
    else{
			if(g == maxRGB){
        HSV->H = 2.0f + (b-r)/deltaRGB;
      }
      else{
				if (b == maxRGB){
					HSV->H = 4.0f + (r-g)/deltaRGB;
        }
      }
    }
    HSV->H = HSV->H * 60.0f;
    if (HSV->H < 0.0f){
			HSV->H += 360;
    }
    HSV->H /= 360;
  }
}


/**
 * @Description  	HSV 转为 RGB
 * @Param     	  {void}
 * @Return    	  {void}
*/
void __HSV_2_RGB(HSV_Color HSV, RGB_Color *RGB){
	float R,G,B,aa,bb,cc,f;
  int k;
  if (HSV.S <= 0.0f)
		R = G = B = HSV.V;
  else{
		if (HSV.H == 1.0f){
			HSV.H = 0.0f;
		}
    HSV.H *= 6.0f;
    k = (int)floor(HSV.H);
    f = HSV.H - k;
    aa = HSV.V * (1.0f - HSV.S);
    bb = HSV.V * (1.0f - HSV.S * f);
    cc = HSV.V * (1.0f -(HSV.S * (1.0f - f)));
    switch(k){
      case 0:
       R = HSV.V; 
       G = cc; 
       B =aa;
       break;
      case 1:
       R = bb; 
       G = HSV.V;
       B = aa;
       break;
      case 2:
       R =aa;
       G = HSV.V;
       B = cc;
       break;
      case 3:
       R = aa;
       G = bb;
       B = HSV.V;
       break;
      case 4:
       R = cc;
       G = aa;
       B = HSV.V;
       break;
      case 5:
       R = HSV.V;
       G = aa;
       B = bb;
       break;
    }
  }
  RGB->R = (unsigned char)(R * 255);
  RGB->G = (unsigned char)(G * 255);
  RGB->B = (unsigned char)(B * 255);
}


/**
 * @Description  	亮度调节
 * @Param     	  {void}
 * @Return    	  {void}
*/
void __brightnessAdjust(float percent, RGB_Color RGB){
	if(percent < 0.01f){
		percent = 0.01f;
	}
	if(percent > 1.0f){
		percent = 1.0f;
	}
	__RGB_2_HSV(RGB, &hsv_color);
	hsv_color.V = percent;
	__HSV_2_RGB(hsv_color, &rgb_color);
}
/************************************************************
														EOF
*************************************************************/

