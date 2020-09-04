/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��(For STM32H7)
*	�ļ����� : bsp.h
*	��    �� : V1.0
*	˵    �� : ����Ӳ���ײ�������������ļ���ÿ��c�ļ����� #include "bsp.h" ���������е���������ģ�顣
*			   bsp = Borad surport packet �弶֧�ְ�
*	�޸ļ�¼ :
*		�汾��  ����         ����       ˵��
*		V1.0    2018-07-29  Eric2013   ��ʽ����
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H_

#define STM32_V7    


/* ����Ƿ����˿������ͺ� */
#if !defined (STM32_V7)
	#error "Please define the board model : STM32_V7"
#endif

/* ���� BSP �汾�� */
#define __STM32H7_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#define EXTI9_5_ISR_MOVE_OUT		/* bsp.h �ж�����У���ʾ�������Ƶ� stam32f4xx_it.c�� �����ظ����� */

#define ERROR_HANDLER()		Error_Handler(__FILE__, __LINE__);

/* Ĭ���ǹر�״̬ */
#define  Enable_EventRecorder  0

#if Enable_EventRecorder == 1
	#include "EventRecorder.h"
#endif

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* �������ȼ����� */
#define NVIC_PREEMPT_PRIORITY	4

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
#include "bsp_timer.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_dwt.h"
#include "lvgl.h"
#include "bsp_uart_fifo.h"

#include "bsp_fmc_sdram.h"
#include "bsp_fmc_io.h"

#include "bsp_i2c_gpio.h"
#include "bsp_i2c_eeprom_24xx.h"

#include "bsp_tft_h7.h"
#include "bsp_tft_429.h"

#include "bsp_ts_touch.h"
#include "bsp_ts_ft5x06.h"

#include "bsp_beep.h"
#include "bsp_tim_pwm.h"

#include "lvgl_port_register.h"
//#include "lvgl_obj_exp.h"
//#include "lvgl_arc_exp.h"
//#include "lvgl_bar_exp.h"
//#include "lvgl_btn_exp.h"
//#include "lvgl_btnmatrix_exp.h"
//#include "lvgl_calendar_exp.h"
//#include "lvgl_canvas_exp.h"
//#include "lvgl_checkbox_exp.h"
//#include "lvgl_chart_exp.h"
//#include "lvgl_ddlist_exp.h"
//#include "lvgl_objmask_exp.h"
//#include "lvgl_page_exp.h"
//#include "lvgl_spinbox_exp.h"
//#include "lvgl_table_exp.h"
//#include "lvgl_titeview_exp.h"
#include "lvgl_window_exp.h"

/* �ṩ������C�ļ����õĺ��� */
void bsp_Init(void);
void bsp_Idle(void);

void bsp_GetCpuID(uint32_t *_id);
void Error_Handler(char *file, uint32_t line);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
