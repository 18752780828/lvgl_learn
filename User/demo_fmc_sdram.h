/*
*********************************************************************************************************
*
*	ģ������ : SDRAM���ܲ����ļ�
*	�ļ����� : demo_fmc_sdram.h
*	��    �� : V1.0
*	˵    �� : �ⲿ32λ����SDRAM���ܲ��ԡ�
*              1��SDRAM�ͺ�IS42S32800G-6BLI, 32λ����, ����32MB, 6ns�ٶ�(166MHz)��
*              2. K1�����£�����32MBд�ٶ�;
*              3. K2�����£�����32MB���ٶ�;
*              4. K3�����£���ȡ1024�ֽڲ���ӡ;
*              5. ҡ��OK�����£�����SDRAM���е�Ԫ�Ƿ����쳣;
*              6. ����Cache
*                ��1��ʹ��MDK��IAR�ĸ����Ż��ȼ����ԣ��Ż�����Ӱ���С��
*                ��2��д�ٶ�376MB/S�����ٶ�182MB/S��
*              7. �ر�Cache
*                ��1��ʹ��MDK��IAR�ĸ����Ż��ȼ����ԣ��Ż�����Ӱ���С��
*                ��2��д�ٶ�307MB/S�����ٶ�116MB/S��
*              8. IAR������ߵȼ��Ż������ٶ���189MB/S����MDK��182MB/S�ߵ㡣
*              9. ����MDK����ʵ�鿪������ߵȼ��Ż���ʱ���Ż���
*              10. ��IAR����ʵ�鿪������ߵȼ��ٶ��Ż���
*	�޸ļ�¼ :
*		�汾��   ����         ����        ˵��
*		V1.0    2018-12-12   Eric2013     1. CMSIS�����汾 V5.4.0
*                                         2. HAL��汾 V1.3.0
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/	

#ifndef _DEMO_FMC_SDRAM_H
#define _DEMO_FMC_SDRAM_H

/* ���ⲿ���õĺ������� */
void DemoFmcSRAM(void);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/