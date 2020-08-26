#include "bsp.h"			 /* 底层硬件驱动 */

<<<<<<< HEAD
extern FT5X06_T g_tFT5X06;
=======
extern TOUCH_T g_tTP;
>>>>>>> 008184c... touch done

static void lv_port_init(void);

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
	uint8_t ucKeyCode;		/* 按键代码 */
	uint8_t fRefresh;		/* 刷屏请求标志,1表示需要刷新 */


	lv_init();
	bsp_Init();		/* 硬件初始化 */	
	lv_port_init();
	/* 界面整体显示完毕后，再打开背光，设置为缺省亮度 */
	bsp_DelayMS(100); 
<<<<<<< HEAD
	ucBright = 125;
=======
	ucBright = 200;
>>>>>>> 008184c... touch done
	LCD_SetPwmBackLight(ucBright);
	
	bsp_StartAutoTimer(0, 200); /* 启动1个200ms的自动重装的定时器，软件定时器0 */
	
	lvgl_obj_test();	
	/* 进入主程序循环体 */
	fRefresh = 1;	
	while (1)
	{
		bsp_Idle();
		bsp_DelayMS(5); 
		lv_task_handler();

	}
}
static void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
/*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
	int32_t x, y;
	for(y = area->y1; y <= area->y2; y++) 
	{
		for(x = area->x1; x <= area->x2; x++) 
		{
			LCDH7_PutPixel(x, y, RGB(color_p->ch.red, color_p->ch.green, color_p->ch.blue));
			color_p++;
		}
	}
	lv_disp_flush_ready(disp_drv);
}

static bool my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
	FT5X06_Scan();	/* 触摸屏 */
<<<<<<< HEAD
	data->point.x = g_tFT5X06.X[0];
	data->point.y = g_tFT5X06.Y[0];
	data->state = LV_INDEV_STATE_REL;
=======
	data->point.x = g_tTP.XBuf[0];
	data->point.y = g_tTP.YBuf[0];
	data->state = LV_INDEV_STATE_PR;
>>>>>>> 008184c... touch done
	return false; /*No buffering now so no more data read*/
}

static void lv_port_init(void)
{
	/*A static or global variable to store the buffers*/
	static lv_disp_buf_t disp_buf;
	/*Static or global buffer(s). The second buffer is optional*/
	static lv_color_t buf_1[LV_VER_RES_MAX * 10];
	static lv_color_t buf_2[LV_HOR_RES_MAX * 10];
	/*Initialize disp_buf` with the buffer(s) */
	lv_disp_buf_init(&disp_buf, buf_1, buf_2, LV_HOR_RES_MAX * 10);	
	
	lv_disp_drv_t disp_drv; /*A variable to hold the drivers. Can be local variable*/
	lv_disp_drv_init(&disp_drv); /*Basic initialization*/
	disp_drv.buffer = &disp_buf; /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb; /*Set a flush callback to draw to the display*/
	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv); /*Basic initialization*/
	indev_drv.type = LV_INDEV_TYPE_POINTER; /*See below.*/
	indev_drv.read_cb = my_input_read; /*See below.*/
	/*Register the driver in LVGL and save the created input device object*/
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
