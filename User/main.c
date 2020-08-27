#include "bsp.h"			 /* 底层硬件驱动 */



/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参: 无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	uint16_t ucBright;	   	/* 背光亮度(0-255) */
	
	lv_init();
	bsp_Init();		/* 硬件初始化 */	
	lv_port_init();
	
	bsp_DelayMS(100); 
	ucBright = 200;
	
	LCD_SetPwmBackLight(ucBright);
	
	
//	lvgl_btn_test();	
//	lvgl_obj_test();
	lv_ex_arc_1();
//	lv_ex_arc_2();
	
	while (1)
	{
		bsp_DelayMS(5); 
		lv_task_handler();

	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
