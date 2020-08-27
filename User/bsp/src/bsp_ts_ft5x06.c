/*
*********************************************************************************************************
*
*	ģ������ : ft5x0x���ݴ���оƬ��������
*	�ļ����� : bsp_ts_ft5x06.c
*	��    �� : V1.0
*	˵    �� : FocalTech ft5x0x����оƬ��������4.3����480*272.
*	�޸ļ�¼ :
*		�汾��   ����        ����     ˵��
*		V1.0    2015-10-30  armfly   ��ʽ����
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

#if 1
	#define printf_dbg printf
#else
	#define printf_dbg(...)
#endif

FT5X06_T g_tFT5X06;

static void FT5X06_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen);

/*
*********************************************************************************************************
*	�� �� ��: FT5X06_InitHard
*	����˵��: ���ô���оƬ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void FT5X06_InitHard(void)
{	
#if 0	/* ��ӡȫ���ļĴ������� for debug */
	{
		uint8_t i;
		uint8_t reg_value;
		
		printf_dbg("[FTS] Touch Chip\r\n");
		
		bsp_DelayMS(10);
		
		for (i = 0; i < 255; i++)
		{
			FT5X06_ReadReg(i, &reg_value, 1);
			printf_dbg(" 0x%02X = 0x%02X\r\n", i, reg_value);
		}
	}
#endif	
	
#if 0
	{
		uint8_t reg_addr;
		uint8_t reg_value;
		
		//get some register information
		reg_addr = FT5X06_REG_FW_VER;
		FT5X06_ReadReg(reg_addr, &reg_value, 1);
		printf_dbg("[FTS] Firmware version = 0x%x\r\n", reg_value);
		
		reg_addr = FT5X06_REG_POINT_RATE;
		FT5X06_ReadReg(reg_addr, &reg_value, 1);
		printf_dbg("[FTS] report rate is %dHz.\r\n", reg_value * 10);
		
		reg_addr = FT5X06_REG_THGROUP;
		FT5X06_ReadReg(reg_addr, &reg_value, 1);
		printf_dbg("[FTS] touch threshold is %d.\r\n", reg_value * 4);
	}
#endif	
	
	g_tFT5X06.TimerCount = 0;
	g_tFT5X06.Enable = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: FT5X06_ReadReg
*	����˵��: ��1���������Ķ���Ĵ���
*	��    ��: _usRegAddr : �Ĵ�����ַ
*			  _pRegBuf : �Ĵ������ݻ�����
*			 _ucLen : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void FT5X06_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;

    i2c_Start();					/* ���߿�ʼ�ź� */

    i2c_SendByte(FT5X06_I2C_ADDR);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();

    //i2c_SendByte(_usRegAddr >> 8);	/* ��ַ��8λ */
	//i2c_WaitAck();

    i2c_SendByte(_usRegAddr);		/* ��ַ��8λ */
	i2c_WaitAck();

	i2c_Start();
    i2c_SendByte(FT5X06_I2C_ADDR + 0x01);	/* �����豸��ַ+���ź� */
	i2c_WaitAck();

	for (i = 0; i < _ucLen - 1; i++)
	{
	    _pRegBuf[i] = i2c_ReadByte();	/* ���Ĵ������� */
		i2c_Ack();
	}

	/* ���һ������ */
	 _pRegBuf[i] = i2c_ReadByte();		/* ���Ĵ������� */
	i2c_NAck();

    i2c_Stop();							/* ����ֹͣ�ź� */
}

/*
*********************************************************************************************************
*	�� �� ��: FT5X06_Timer1ms
*	����˵��: ÿ��1ms����1��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void FT5X06_Timer1ms(void)
{
	g_tFT5X06.TimerCount++;
}

/*
*********************************************************************************************************
*	�� �� ��: FT5X06_Scan
*	����˵��: ��ȡ�������ݡ���ȡȫ�������ݣ���Ҫ 720us���ҡ����������� bsp_Idle()��ִ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void FT5X06_Scan(void)
{
	uint8_t buf[CFG_POINT_READ_BUF];
	uint8_t i;
	static uint8_t s_tp_down = 0;
	uint16_t x, y;
	static uint16_t x_save, y_save;
	static uint8_t s_count = 0;	

	g_tTP.XBuf[0] = 0;
	g_tTP.YBuf[0] = 0;

	if (TOUCH_PenInt() == 1)
	{
//		 if (++s_tp_down > 2)
		{
			s_tp_down = 0;
			FT5X06_ReadReg(0, buf, CFG_POINT_READ_BUF);
			g_tTP.YBuf[0] = (int16_t)(buf[3] & 0x0F)<<8 | (int16_t)buf[4];
			g_tTP.XBuf[0] = (int16_t)(buf[5] & 0x0F)<<8 | (int16_t)buf[6];
		}
}
	else
	{
			s_tp_down = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: FT5X06_ReadID
*	����˵��: ��оƬID, ʶ����4.3��5.0��7.0�紥����
*			ʵ����:  4.3��id = 0x55    5.0��id = 0x0A  7.0��id = 0x06
*	��    ��: ��
*	�� �� ֵ: 1�ֽ�оƬID
*********************************************************************************************************
*/
uint8_t FT5X06_ReadID(void)
{
	uint8_t id;
	
	FT5X06_ReadReg(FTS_REG_CHIP_ID, &id, 1);
	
	g_tFT5X06.ChipID = id;		/* ����id */
	return id;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
