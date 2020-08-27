#include "bsp.h"			 /* �ײ�Ӳ������ */



/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    ��: ��
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
	uint16_t ucBright;	   	/* ��������(0-255) */
	
	lv_init();
	bsp_Init();		/* Ӳ����ʼ�� */	
	lv_port_init();
	
	bsp_DelayMS(100); 
	ucBright = 200;
	
	LCD_SetPwmBackLight(ucBright);
	
	
//	lvgl_btn_test();	
//	lvgl_obj_test();
//	lv_ex_arc_1();
//	lv_ex_arc_2();
//	lv_ex_bar_1();
//	lv_ex_btnmatrix_1();
	lv_ex_calendar_1();
	while (1)
	{
		bsp_DelayMS(5); 
		lv_task_handler();

	}
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
