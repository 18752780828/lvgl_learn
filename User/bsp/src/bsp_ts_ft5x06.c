/*
*********************************************************************************************************
*
*	模块名称 : ft5x0x电容触摸芯片驱动程序
*	文件名称 : bsp_ts_ft5x06.c
*	版    本 : V1.0
*	说    明 : FocalTech ft5x0x触摸芯片驱动程序。4.3寸屏480*272.
*	修改记录 :
*		版本号   日期        作者     说明
*		V1.0    2015-10-30  armfly   正式发布
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
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
*	函 数 名: FT5X06_InitHard
*	功能说明: 配置触摸芯片
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void FT5X06_InitHard(void)
{	
#if 0	/* 打印全部的寄存器数据 for debug */
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
*	函 数 名: FT5X06_ReadReg
*	功能说明: 读1个或连续的多个寄存器
*	形    参: _usRegAddr : 寄存器地址
*			  _pRegBuf : 寄存器数据缓冲区
*			 _ucLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
static void FT5X06_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;

    i2c_Start();					/* 总线开始信号 */

    i2c_SendByte(FT5X06_I2C_ADDR);	/* 发送设备地址+写信号 */
	i2c_WaitAck();

    //i2c_SendByte(_usRegAddr >> 8);	/* 地址高8位 */
	//i2c_WaitAck();

    i2c_SendByte(_usRegAddr);		/* 地址低8位 */
	i2c_WaitAck();

	i2c_Start();
    i2c_SendByte(FT5X06_I2C_ADDR + 0x01);	/* 发送设备地址+读信号 */
	i2c_WaitAck();

	for (i = 0; i < _ucLen - 1; i++)
	{
	    _pRegBuf[i] = i2c_ReadByte();	/* 读寄存器数据 */
		i2c_Ack();
	}

	/* 最后一个数据 */
	 _pRegBuf[i] = i2c_ReadByte();		/* 读寄存器数据 */
	i2c_NAck();

    i2c_Stop();							/* 总线停止信号 */
}

/*
*********************************************************************************************************
*	函 数 名: FT5X06_Timer1ms
*	功能说明: 每隔1ms调用1次
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void FT5X06_Timer1ms(void)
{
	g_tFT5X06.TimerCount++;
}

/*
*********************************************************************************************************
*	函 数 名: FT5X06_Scan
*	功能说明: 读取触摸数据。读取全部的数据，需要 720us左右。放在主程序 bsp_Idle()中执行
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: FT5X06_ReadID
*	功能说明: 读芯片ID, 识别是4.3、5.0、7.0寸触摸。
*			实测结果:  4.3寸id = 0x55    5.0寸id = 0x0A  7.0寸id = 0x06
*	形    参: 无
*	返 回 值: 1字节芯片ID
*********************************************************************************************************
*/
uint8_t FT5X06_ReadID(void)
{
	uint8_t id;
	
	FT5X06_ReadReg(FTS_REG_CHIP_ID, &id, 1);
	
	g_tFT5X06.ChipID = id;		/* 保存id */
	return id;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
