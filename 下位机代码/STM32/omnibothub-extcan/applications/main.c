#include "include.h"
#include "init.h"
#include "scheduler.h"
//Program Size: Code=178136 RO-data=3328 RW-data=4036 ZI-data=49628  
//Program Size: Code=175640 RO-data=3328 RW-data=4048 ZI-data=49536   micro
u8 Init_Finish = 0;  
int main(void)
{

  Init_Finish = All_Init();		
	while(1)
	{  
		Duty_Loop() ; 
	}
}


