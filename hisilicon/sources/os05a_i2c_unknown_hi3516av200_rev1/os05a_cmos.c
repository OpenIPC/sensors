/******************************************************************************

  Copyright (C), 2001-2013, Hisilicon Tech. Co., Ltd.

 ******************************************************************************
  File Name     : os05a_cmos.c
  Version       : Initial Draft
  Author        : Hisilicon BVT ISP group
  Created       : 2017/08/22
  Description   : Sony OS05A sensor driver
  History       :
  1.Date        : 2017/08/22
  Author        : 
  Modification  : Created file

******************************************************************************/

#if !defined(__OS05A_CMOS_H_)
#define __OS05A_CMOS_H_

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "hi_comm_sns.h"
#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_af.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* End of #ifdef __cplusplus */

#define OS05A_ID 05
#define HIGH_8BITS(x) ((x & 0xff00) >> 8)
#define LOW_8BITS(x)  (x & 0x00ff)

#ifndef MAX
#define MAX(a, b) (((a) < (b)) ?  (b) : (a))
#endif

#ifndef MIN
#define MIN(a, b) (((a) > (b)) ?  (b) : (a))
#endif

/****************************************************************************
 * extern function reference                                                *
 ****************************************************************************/

extern void os05a_init(ISP_DEV IspDev);
extern void os05a_exit(ISP_DEV IspDev);
extern void os05a_standby(ISP_DEV IspDev);
extern void os05a_restart(ISP_DEV IspDev);
extern int os05a_write_register(ISP_DEV IspDev, int addr, int data);
extern int os05a_read_register(ISP_DEV IspDev, int addr);

ISP_SNS_STATE_S             g_astOs05a[ISP_MAX_DEV_NUM] = {{0}};
static ISP_SNS_STATE_S     *g_apstSnsState[ISP_MAX_DEV_NUM] = {&g_astOs05a[0], &g_astOs05a[1]};
ISP_SNS_COMMBUS_U g_aunOs05aBusInfo[ISP_MAX_DEV_NUM] = {
    [0] = { .s8I2cDev = 0},
    [1] = { .s8I2cDev = 1}
};

static ISP_FSWDR_MODE_E genFSWDRMode[ISP_MAX_DEV_NUM] = {ISP_FSWDR_NORMAL_MODE,ISP_FSWDR_NORMAL_MODE};
static HI_U32 gu32MaxTimeGetCnt[ISP_MAX_DEV_NUM] = {0,0};

extern const unsigned int os05a_i2c_addr;
extern unsigned int os05a_addr_byte;
extern unsigned int os05a_data_byte;

HI_U32 g_au32InitExposure[ISP_MAX_DEV_NUM]  = {0};
HI_U32 g_au32LinesPer500ms[ISP_MAX_DEV_NUM] = {0};
HI_U16 g_au16InitWBGain[ISP_MAX_DEV_NUM][3] = {{0}};
HI_U16 g_au16SampleRgain[ISP_MAX_DEV_NUM] = {0};
HI_U16 g_au16SampleBgain[ISP_MAX_DEV_NUM] = {0};

#define OS05A_FULL_LINES_MAX  (0x9240)

#define VMAX_ADDR_H             (0x380E)
#define VMAX_ADDR_L             (0x380F)
#define AGAIN_H                  (0x3508)
#define AGAIN_L                  (0x3509)
#define DGAIN_H                  (0x350A)
#define DGAIN_L                  (0x350B)
#define SHORT_AGAIN_H            (0x350C)
#define SHORT_AGAIN_L            (0x350D)
#define SHORT_DGAIN_H            (0x350E)
#define SHORT_DGAIN_L            (0x350F)
#define EXPOSURE_TIME_LONG_H    (0x3501)
#define EXPOSURE_TIME_LONG_L    (0x3502)
#define EXPOSURE_TIME_SHORT_H   (0x3511)
#define EXPOSURE_TIME_SHORT_L   (0x3512)
#define LINE_LEN_PCK             (0x300C)

#define INCREASE_LINES          (5) /* make real fps less than stand fps because NVR require*/
#define OS05A_VMAX_5M30_12bit_LINEAE  (0x921+INCREASE_LINES)
#define OS05A_VMAX_5M30_10bit_WDR     (0x9c0+INCREASE_LINES)

#define Os05a_5M_2688x1944_12bit_linear30   (0)
#define Os05a_5M_2688x1944_10bit_wdr30      (1)

static HI_S32 cmos_get_ae_default(ISP_DEV IspDev,AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    HI_U32 u32Fll = 0;
    HI_FLOAT f32maxFps = 0;

    if (HI_NULL == pstAeSnsDft)
    {
        printf("null pointer when get ae default value!\n");
        return -1;
    }
    
    if(Os05a_5M_2688x1944_12bit_linear30 == g_apstSnsState[IspDev]->u8ImgMode)
    {
        u32Fll = OS05A_VMAX_5M30_12bit_LINEAE;
        f32maxFps = 30;
    }
    else if(Os05a_5M_2688x1944_10bit_wdr30 == g_apstSnsState[IspDev]->u8ImgMode)
    {
        u32Fll = OS05A_VMAX_5M30_10bit_WDR;
        f32maxFps = 30;
    }
    
	g_apstSnsState[IspDev]->u32FLStd = u32Fll;
    pstAeSnsDft->u32FullLinesStd = g_apstSnsState[IspDev]->u32FLStd;
    pstAeSnsDft->u32FlickerFreq = 0;
    
    pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;
    
	if(g_au32InitExposure[IspDev] == 0)
	{
		pstAeSnsDft->u32InitExposure = 130000;
	}
	else
	{
		pstAeSnsDft->u32InitExposure = g_au32InitExposure[IspDev];
	}
    
    if(g_au32LinesPer500ms[IspDev] == 0)
	{
		pstAeSnsDft->u32LinesPer500ms = ((HI_U64)(u32Fll * f32maxFps)) >> 1;
	}
	else
	{
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[IspDev];
	}
    
    pstAeSnsDft->u32MaxIntTime = g_apstSnsState[IspDev]->u32FLStd - 12;
    pstAeSnsDft->stIntTimeAccu.f32Offset = 0;
    
    pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
    pstAeSnsDft->stAgainAccu.f32Accuracy = 0.0625;
    
    pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stDgainAccu.f32Accuracy = 0.0009765625;
    
    pstAeSnsDft->u32MaxAgain = 15872;
    pstAeSnsDft->u32MinAgain = 1024;
    pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
    pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;
    
    pstAeSnsDft->u32MaxDgain = 16383;  
    pstAeSnsDft->u32MinDgain = 1024;
    pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
    pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;    
    
    pstAeSnsDft->u32ISPDgainShift = 8;
    pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
    pstAeSnsDft->u32MaxISPDgainTarget = 6 << pstAeSnsDft->u32ISPDgainShift;
    
    pstAeSnsDft->enMaxIrisFNO = ISP_IRIS_F_NO_1_0;
    pstAeSnsDft->enMinIrisFNO = ISP_IRIS_F_NO_32_0;
    
    pstAeSnsDft->bAERouteExValid = HI_FALSE;
    pstAeSnsDft->stAERouteAttr.u32TotalNum = 0;
    pstAeSnsDft->stAERouteAttrEx.u32TotalNum = 0;
    
    switch(g_apstSnsState[IspDev]->enWDRMode)
    {
        default:
        case WDR_MODE_NONE:
            /* linear mode */
            pstAeSnsDft->au8HistThresh[0] = 0xd;
            pstAeSnsDft->au8HistThresh[1] = 0x28;
            pstAeSnsDft->au8HistThresh[2] = 0x60;
            pstAeSnsDft->au8HistThresh[3] = 0x80;
            
            pstAeSnsDft->u8AeCompensation = 0x38;
            
            pstAeSnsDft->u32MaxIntTime = g_apstSnsState[IspDev]->u32FLStd - 8;
            pstAeSnsDft->u32MinIntTime = 2;
            pstAeSnsDft->u32MaxIntTimeTarget = 65535;
            pstAeSnsDft->u32MinIntTimeTarget = 2;
            
            break;
		
        case WDR_MODE_2To1_LINE:
            /* FS WDR mode */
            pstAeSnsDft->au8HistThresh[0] = 0xc;
            pstAeSnsDft->au8HistThresh[1] = 0x18;
            pstAeSnsDft->au8HistThresh[2] = 0x60;
            pstAeSnsDft->au8HistThresh[3] = 0x80;
            
            pstAeSnsDft->u8AeCompensation = 0x38;
            
            pstAeSnsDft->u32MaxIntTime = g_apstSnsState[IspDev]->u32FLStd - 8;
            pstAeSnsDft->u32MinIntTime = 2;
            pstAeSnsDft->u32MaxIntTimeTarget = 65535;
            pstAeSnsDft->u32MinIntTimeTarget = 2;

            break;
        }
    
    return 0;
}

/* the function of sensor set fps */
static HI_VOID cmos_fps_set(ISP_DEV IspDev,HI_FLOAT f32Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    HI_FLOAT f32maxFps;
    HI_U32 u32Lines;
    
    switch (g_apstSnsState[IspDev]->u8ImgMode)
    {
        case Os05a_5M_2688x1944_12bit_linear30:
            if ((f32Fps <= 30) && (f32Fps >= 1.87))
            {
                f32maxFps = 30;
                u32Lines = OS05A_VMAX_5M30_12bit_LINEAE * f32maxFps / f32Fps;
            }
            else
            {
                printf("Not support Fps: %f\n", f32Fps);
                return;
            }
            break;
            
        case Os05a_5M_2688x1944_10bit_wdr30:
            if ((f32Fps <= 30) && (f32Fps >= 2.0))
            {
                f32maxFps = 30;
                u32Lines = OS05A_VMAX_5M30_10bit_WDR * f32maxFps / f32Fps;
            }
            else
            {
                printf("Not support Fps: %f\n", f32Fps);
                return;
            }
            break;
            
        default:
            printf("Not support this Mode\n");
            return;
            break;
    }
    
    g_apstSnsState[IspDev]->u32FLStd = u32Lines;
    g_apstSnsState[IspDev]->u32FLStd = (g_apstSnsState[IspDev]->u32FLStd > OS05A_FULL_LINES_MAX) ? OS05A_FULL_LINES_MAX : g_apstSnsState[IspDev]->u32FLStd;
    
    pstAeSnsDft->f32Fps = f32Fps;
    pstAeSnsDft->u32LinesPer500ms = g_apstSnsState[IspDev]->u32FLStd * f32Fps / 2;
	
    pstAeSnsDft->u32MaxIntTime = g_apstSnsState[IspDev]->u32FLStd - 8;
    pstAeSnsDft->u32FullLinesStd = g_apstSnsState[IspDev]->u32FLStd;
    g_apstSnsState[IspDev]->au32FL[0]= g_apstSnsState[IspDev]->u32FLStd;
    pstAeSnsDft->u32FullLines = g_apstSnsState[IspDev]->au32FL[0];
    
    if (WDR_MODE_NONE == g_apstSnsState[IspDev]->enWDRMode)
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u32Data = LOW_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u32Data = HIGH_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
    }
    else
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u32Data = LOW_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u32Data = HIGH_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
    }
    
    return;
}

static HI_VOID cmos_slow_framerate_set(ISP_DEV IspDev,HI_U32 u32FullLines,
    AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    g_apstSnsState[IspDev]->au32FL[0] = (u32FullLines > OS05A_FULL_LINES_MAX) ? OS05A_FULL_LINES_MAX : u32FullLines;
	pstAeSnsDft->u32FullLines = g_apstSnsState[IspDev]->au32FL[0];
    pstAeSnsDft->u32MaxIntTime = g_apstSnsState[IspDev]->au32FL[0] - 8;
    
    if (WDR_MODE_NONE == g_apstSnsState[IspDev]->enWDRMode)
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u32Data = LOW_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u32Data = HIGH_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
    }
    else
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u32Data = LOW_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u32Data = HIGH_8BITS(g_apstSnsState[IspDev]->au32FL[0]);
    }
    
    return;
}

/* while isp notify ae to update sensor regs, ae call these funcs.*/
static HI_VOID cmos_inttime_update(ISP_DEV IspDev,HI_U32 u32IntTime)
{
    static HI_BOOL bFirst = HI_TRUE;
    
    if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
    {
        if(bFirst)/* short exposure */
        {
            g_apstSnsState[IspDev]->au32WDRIntTime[0] = u32IntTime;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[8].u32Data = LOW_8BITS(u32IntTime);
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[9].u32Data = HIGH_8BITS(u32IntTime);
            bFirst = HI_FALSE;
        }
        else/* long exposure */
        {
            g_apstSnsState[IspDev]->au32WDRIntTime[1] = u32IntTime;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[2].u32Data = LOW_8BITS(u32IntTime);
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[3].u32Data = HIGH_8BITS(u32IntTime);
            bFirst = HI_TRUE;
        }
    }
    else
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[2].u32Data =  LOW_8BITS(u32IntTime);
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[3].u32Data = HIGH_8BITS(u32IntTime);
    }
    
    return;
}

#define AGAIN_NODE_NUM 64
static HI_U32 au32Again_table[AGAIN_NODE_NUM] =
{    
    1024,1088,1152,1216,1280,1344,1408,1472,1536,1600,1664,1728,1792,1856,1920,1984,
    2048,2176,2304,2432,2560,2688,2816,2944,3072,3200,3328,3456,3584,3712,3840,3968,
    4096,4352,4608,4864,5120,5376,5632,5888,6144,6400,6656,6912,7168,7424,7680,7936,
    8192,8704,9216,9728,10240,10752,11264,11776,12288,12800,13312,13824,14336,14848,
    15360,15872
};

static HI_VOID cmos_again_calc_table(ISP_DEV IspDev,HI_U32 *pu32AgainLin, HI_U32 *pu32AgainDb)
{
    int i;
    
    if((HI_NULL == pu32AgainLin) ||(HI_NULL == pu32AgainDb))
    {
        printf("null pointer when get ae sensor gain info  value!\n");
        return;
    }
    
    if (*pu32AgainLin >= au32Again_table[AGAIN_NODE_NUM -1])
    {
         *pu32AgainLin = au32Again_table[AGAIN_NODE_NUM -1];
         *pu32AgainDb = AGAIN_NODE_NUM -1;
         return ;
    }
    
    for (i = 1; i < AGAIN_NODE_NUM; i++)
    {
        if (*pu32AgainLin < au32Again_table[i])
        {
            *pu32AgainLin = au32Again_table[i - 1];
            *pu32AgainDb = i - 1;
            break;
        }
    }
    return;
}

static HI_VOID cmos_gains_update(ISP_DEV IspDev,HI_U32 u32Again, HI_U32 u32Dgain)
{
    HI_U32 AGain_Reg = au32Again_table[u32Again] >> 3;
    
    g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[4].u32Data = LOW_8BITS(AGain_Reg);
    g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[5].u32Data = ((AGain_Reg & 0x3F00) >> 8);
    g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[6].u32Data = LOW_8BITS(u32Dgain);
    g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[7].u32Data = ((u32Dgain & 0x3F00) >> 8);
    
    if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[10].u32Data = g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[4].u32Data;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[11].u32Data = g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[5].u32Data;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[12].u32Data = g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[6].u32Data;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[13].u32Data = g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[7].u32Data;
    }
    
    return;
}

/* Only used in WDR_MODE_2To1_LINE and WDR_MODE_2To1_FRAME mode */
static HI_VOID cmos_get_inttime_max(ISP_DEV IspDev,HI_U16 u16ManRatioEnable, HI_U32 *au32Ratio, HI_U32 *au32IntTimeMax, HI_U32 *au32IntTimeMin, HI_U32 *pu32LFMaxIntTime)
{
    HI_U32 i = 0;
    HI_U32 u32IntTimeMaxTmp = 0;
    HI_U32 u32IntTimeMaxTmp1 = 0;
    HI_U32 u32RatioTmp = 0x40;
    HI_U32 u32ShortTimeMinLimit = 0;
    
    u32ShortTimeMinLimit =  2;
    
    if((HI_NULL == au32Ratio) || (HI_NULL == au32IntTimeMax) || (HI_NULL == au32IntTimeMin))
    {
        printf("null pointer when get ae sensor ExpRatio/IntTimeMax/IntTimeMin value!\n");
        return;
    }
    
    if ((WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode))
    {
        if(ISP_FSWDR_LONG_FRAME_MODE == genFSWDRMode[IspDev])  /* long frame mode enable */
        {
			if(gu32MaxTimeGetCnt[IspDev] > 15)
			{
				u32IntTimeMaxTmp = g_apstSnsState[IspDev]->au32FL[0] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0];
				u32IntTimeMaxTmp1 = g_apstSnsState[IspDev]->au32FL[0] - 50;
			}
			else
			{
				u32IntTimeMaxTmp = ((g_apstSnsState[IspDev]->au32FL[1] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0]) << 6) / DIV_0_TO_1(au32Ratio[0]);
				u32IntTimeMaxTmp1 = (g_apstSnsState[IspDev]->au32FL[0] - 50) * 0x40 / (au32Ratio[0] + 0x40);
			}
            
            u32IntTimeMaxTmp = (u32IntTimeMaxTmp < u32IntTimeMaxTmp1) ? u32IntTimeMaxTmp : u32IntTimeMaxTmp1;
            gu32MaxTimeGetCnt[IspDev]++;
        }		
		else if(ISP_FSWDR_AUTO_LONG_FRAME_MODE == genFSWDRMode[IspDev])
        {
			if(g_apstSnsState[IspDev]->au32WDRIntTime[0] == u32ShortTimeMinLimit && au32Ratio[0] == 0x40)
			{
				if(gu32MaxTimeGetCnt[IspDev] > 15)
				{
					u32IntTimeMaxTmp = g_apstSnsState[IspDev]->au32FL[0] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0];
					u32IntTimeMaxTmp1 = g_apstSnsState[IspDev]->au32FL[0] - 50;
				}
				else
				{
					u32IntTimeMaxTmp = ((g_apstSnsState[IspDev]->au32FL[1] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0]) << 6) / DIV_0_TO_1(au32Ratio[0]);
            		u32IntTimeMaxTmp1 = (g_apstSnsState[IspDev]->au32FL[0] - 50) * 0x40 / (au32Ratio[0] + 0x40);
				}
				
            	u32IntTimeMaxTmp = (u32IntTimeMaxTmp < u32IntTimeMaxTmp1) ? u32IntTimeMaxTmp : u32IntTimeMaxTmp1;
				*pu32LFMaxIntTime = u32IntTimeMaxTmp;
				gu32MaxTimeGetCnt[IspDev]++;
			}
			else
			{
				gu32MaxTimeGetCnt[IspDev] = 0;
				u32IntTimeMaxTmp = ((g_apstSnsState[IspDev]->au32FL[1] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0]) << 6) / DIV_0_TO_1(au32Ratio[0]);
            	u32IntTimeMaxTmp1 = (g_apstSnsState[IspDev]->au32FL[0] - 50) * 0x40 / (au32Ratio[0] + 0x40);
                u32IntTimeMaxTmp = (u32IntTimeMaxTmp <  u32IntTimeMaxTmp1) ? u32IntTimeMaxTmp :  u32IntTimeMaxTmp1;
				*pu32LFMaxIntTime = u32IntTimeMaxTmp1;
			}
        }
        else
        {
            u32IntTimeMaxTmp = ((g_apstSnsState[IspDev]->au32FL[1] - 50 - g_apstSnsState[IspDev]->au32WDRIntTime[0]) << 6) / DIV_0_TO_1(au32Ratio[0]);
            u32IntTimeMaxTmp1 = (g_apstSnsState[IspDev]->au32FL[0] - 50) * 0x40 / (au32Ratio[0] + 0x40);
            u32IntTimeMaxTmp = (u32IntTimeMaxTmp <  u32IntTimeMaxTmp1) ? u32IntTimeMaxTmp :  u32IntTimeMaxTmp1;
        }
    }
    
    if(u32IntTimeMaxTmp >= u32ShortTimeMinLimit)
    {
        if (IS_LINE_WDR_MODE(g_apstSnsState[IspDev]->enWDRMode))
        {
            au32IntTimeMax[0] = u32IntTimeMaxTmp;
            au32IntTimeMax[1] = au32IntTimeMax[0] * au32Ratio[0] >> 6;
            au32IntTimeMax[2] = au32IntTimeMax[1] * au32Ratio[1] >> 6;
            au32IntTimeMax[3] = au32IntTimeMax[2] * au32Ratio[2] >> 6;
            au32IntTimeMin[0] = u32ShortTimeMinLimit;
            au32IntTimeMin[1] = au32IntTimeMin[0] * au32Ratio[0] >> 6;
            au32IntTimeMin[2] = au32IntTimeMin[1] * au32Ratio[1] >> 6;
            au32IntTimeMin[3] = au32IntTimeMin[2] * au32Ratio[2] >> 6;
        }
        else
        {
        }
    }
    else
    {
        if(1 == u16ManRatioEnable)
        {
            printf("Manaul ExpRatio is too large!\n");
            return;
        }
        else
        {
            u32IntTimeMaxTmp = u32ShortTimeMinLimit;
            
            if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
            {
                for(i = 0x40; i <= 0xFFF; i++)
                {
                    if((u32IntTimeMaxTmp + (u32IntTimeMaxTmp*i >> 6)) > (g_apstSnsState[IspDev]->au32FL[0] - 50))
                    {
                        u32RatioTmp = i - 1;
                        break;
                    }
                }
                au32IntTimeMax[0] = u32IntTimeMaxTmp;
                au32IntTimeMax[1] = au32IntTimeMax[0] * u32RatioTmp >> 6;
            }
            else
            {
            }
            au32IntTimeMin[0] = au32IntTimeMax[0];
            au32IntTimeMin[1] = au32IntTimeMax[1];
            au32IntTimeMin[2] = au32IntTimeMax[2];
            au32IntTimeMin[3] = au32IntTimeMax[3];
        }
    }
    return;
}

/* Only used in FSWDR mode */
static HI_VOID cmos_ae_fswdr_attr_set(ISP_DEV IspDev,AE_FSWDR_ATTR_S *pstAeFSWDRAttr)
{
    genFSWDRMode[IspDev] = pstAeFSWDRAttr->enFSWDRMode;
    gu32MaxTimeGetCnt[IspDev] = 0;
}

HI_S32 cmos_init_ae_exp_function(AE_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(AE_SENSOR_EXP_FUNC_S));
    
    pstExpFuncs->pfn_cmos_get_ae_default    = cmos_get_ae_default;
    pstExpFuncs->pfn_cmos_fps_set           = cmos_fps_set;
    pstExpFuncs->pfn_cmos_slow_framerate_set= cmos_slow_framerate_set;
    pstExpFuncs->pfn_cmos_inttime_update    = cmos_inttime_update;
    pstExpFuncs->pfn_cmos_gains_update      = cmos_gains_update;
    pstExpFuncs->pfn_cmos_again_calc_table  = cmos_again_calc_table;
    pstExpFuncs->pfn_cmos_dgain_calc_table  = NULL;
    pstExpFuncs->pfn_cmos_get_inttime_max   = cmos_get_inttime_max;
    pstExpFuncs->pfn_cmos_ae_fswdr_attr_set = cmos_ae_fswdr_attr_set;
    
    return 0;
}

static AWB_CCM_S g_stAwbCcm =
{
   4850,
   {
        0x01ba, 0x8091, 0x8029,
        0x8062, 0x01be, 0x805c,
        0x8003, 0x80d3, 0x01d6
   },
   
   3100,
   {
        0x0192, 0x8074, 0x801e,
        0x8078, 0x01b9, 0x8041,
        0x000b, 0x80d0, 0x01c5
   },
   
   2450,
   {     
        0x01ac, 0x8066, 0x8046,
        0x8094, 0x01ce, 0x803a,
        0x800c, 0x8187, 0x0293
   }
};

static AWB_CCM_S g_stwdrAwbCcm =
{
   4850,
   {
        0x01ba, 0x8091, 0x8029,
        0x8062, 0x01be, 0x805c,
        0x8003, 0x80d3, 0x01d6
   },
   
   3100,
   {
        0x0192, 0x8074, 0x801e,
        0x8078, 0x01b9, 0x8041,
        0x000b, 0x80d0, 0x01c5
   },
   
   2450,
   {     
        0x01ac, 0x8066, 0x8046,
        0x8094, 0x01ce, 0x803a,
        0x800c, 0x8187, 0x0293
   }
};

static AWB_AGC_TABLE_S g_stAwbAgcTable =
{
    /* bvalid */
    1,
    
    /* saturation */
    {0x80,0x80,0x7e,0x72,0x68,0x60,0x58,0x50,0x48,0x40,0x38,0x38,0x38,0x38,0x38,0x38}
};

static AWB_AGC_TABLE_S g_stAwbAgcTableFSWDR =
{
    /* bvalid */
    1,
    
    /* saturation */
    {0x80,0x80,0x7e,0x72,0x68,0x60,0x58,0x50,0x48,0x40,0x38,0x38,0x38,0x38,0x38,0x38}
};

/* Rgain and Bgain of the golden sample */
#define GOLDEN_RGAIN 0
#define GOLDEN_BGAIN 0

static HI_S32 cmos_get_awb_default(ISP_DEV IspDev,AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
    if (HI_NULL == pstAwbSnsDft)
    {
        printf("null pointer when get awb default value!\n");
        return -1;
    }
    
    memset(pstAwbSnsDft, 0, sizeof(AWB_SENSOR_DEFAULT_S));
    
    pstAwbSnsDft->u16WbRefTemp = 4850;
    
    pstAwbSnsDft->au16GainOffset[0] = 0x1BD;
    pstAwbSnsDft->au16GainOffset[1] = 0x100;
    pstAwbSnsDft->au16GainOffset[2] = 0x100;
    pstAwbSnsDft->au16GainOffset[3] = 0x220;
    
    pstAwbSnsDft->as32WbPara[0] = -1;
    pstAwbSnsDft->as32WbPara[1] = 253;
    pstAwbSnsDft->as32WbPara[2] = -4;
    pstAwbSnsDft->as32WbPara[3] = 178887;
    pstAwbSnsDft->as32WbPara[4] = 128;
    pstAwbSnsDft->as32WbPara[5] = -106450;
    
    switch (g_apstSnsState[IspDev]->enWDRMode)
    {
        default:
        case WDR_MODE_NONE:
            memcpy(&pstAwbSnsDft->stCcm, &g_stAwbCcm, sizeof(AWB_CCM_S));
            memcpy(&pstAwbSnsDft->stAgcTbl, &g_stAwbAgcTable, sizeof(AWB_AGC_TABLE_S));
        break;
        case WDR_MODE_2To1_LINE:
            memcpy(&pstAwbSnsDft->stCcm, &g_stwdrAwbCcm, sizeof(AWB_CCM_S));
            memcpy(&pstAwbSnsDft->stAgcTbl, &g_stAwbAgcTableFSWDR, sizeof(AWB_AGC_TABLE_S));
        break;
    }
    
    pstAwbSnsDft->u16SampleRgain = g_au16SampleRgain[IspDev];
    pstAwbSnsDft->u16SampleBgain = g_au16SampleBgain[IspDev];
    pstAwbSnsDft->u16InitRgain = g_au16InitWBGain[IspDev][0];
    pstAwbSnsDft->u16InitGgain = g_au16InitWBGain[IspDev][1];
    pstAwbSnsDft->u16InitBgain = g_au16InitWBGain[IspDev][2];
    
    pstAwbSnsDft->u8AWBRunInterval = 2;
    
    return 0;
}

static HI_S32 cmos_init_awb_exp_function(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(AWB_SENSOR_EXP_FUNC_S));
    pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;
    return 0;
}

static ISP_CMOS_AGC_TABLE_S g_stIspAgcTable =
{
    /* bvalid */
    1,
    
    /* snr_thresh */
    {0x08,0x0c,0x10,0x14,0x18,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},
               
    /* demosaic_np_offset */
    {0x0,0xa,0x12,0x1a,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},
    
    /* ge_strength */
    {0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37}
};

static ISP_CMOS_BAYER_SHARPEN_S g_stIspBayerSharpen =
{
    /* bvalid */
    1,
    
    /* ShpAlgSel = 1 is Demosaic SharpenEx, else Demosaic sharpen. */
    0,
    
    /* sharpen_alt_d to Sharpen */
    {0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x40,0x30,0x20,0x10},
    
    /* sharpen_alt_ud to Sharpen */
    {0x3a,0x36,0x32,0x30,0x2c,0x30,0x30,0x30,0x28,0x24,0x24,0x20,0x20,0x20,0x10,0x10},
    
    /* demosaic_lum_thresh to Sharpen */
    {0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x40,0x30,0x20,0x20,0x20,0x20,0x20,0x20},
    
    /* SharpenHF to SharpenEx */
    {0x30,0x30,0x30,0x30,0x30,0x30,0x2c,0x28,0x20,0x18,0x14,0x10,0x10,0x10,0x10,0x10},
    
    /* SharpenMF to SharpenEx */
    {0x30,0x30,0x30,0x30,0x28,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x10},
    
    /* SharpenLF to SharpenEx */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18},
    
    /* SadAmplifier to SharpenEx */
    {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10}
};

static ISP_CMOS_BAYER_SHARPEN_S g_stIspBayerSharpenFSWDR =
{
    /* bvalid */
    1,
    
    /* ShpAlgSel = 1 is Demosaic SharpenEx, else Demosaic sharpen. */ 
    0,
    
    /* sharpen_alt_d to Sharpen */
    {0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x48,0x40,0x30,0x20,0x10},
    
    /* sharpen_alt_ud to Sharpen */
    {0x48,0x46,0x42,0x40,0x40,0x40,0x40,0x40,0x40,0x3c,0x38,0x32,0x28,0x20,0x10,0x10},
    
    /* demosaic_lum_thresh to Sharpen */
    {0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x40,0x30,0x20,0x20,0x20,0x20,0x20,0x20},
    
    /* SharpenHF to SharpenEx */
    {0x30,0x30,0x30,0x30,0x30,0x30,0x2c,0x28,0x20,0x18,0x14,0x10,0x10,0x10,0x10,0x10},
    
    /* SharpenMF to SharpenEx */
    {0x30,0x30,0x30,0x30,0x28,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x10},
    
    /* SharpenLF to SharpenEx */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18},
    
    /* SadAmplifier to SharpenEx */
    {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10}
};

static ISP_CMOS_YUV_SHARPEN_S g_stIspYuvSharpen =
{
    /* bvalid */
    1,
    
    /* 100, 200,  400,   800,  1600,  3200,  6400,  12800,  25600,  51200, 102400, 204800, 409600, 819200, 1638400, 3276800 */
    /* bEnLowLumaShoot */ 
    {0,     0,     0,     0,     0,     0,      0,      0,      0,     1,     1,      1,      1,      1,      1,     1},

    /* TextureSt */
    {50,   48,    46,    44,    48,    56,     56,     56,     48,    36,    20,      8,      4,      4,      4,     4},
    
    /* EdgeSt */
    {64,   64,    64,    64,    64,    64,     56,     56,     56,    48,    48,     48,     48,     32,     20,    20},
    
    /* OverShoot */
    {64,   64,    64,    64,    72,    72,     72,     72,     72,    64,    64,     56,     50,     40,     30,    30},
    
    /* UnderShoot */
    {64,   64,    64,    64,    72,    72,     72,     72,     72,    64,    64,     56,     50,     40,     30,    30},
    
    /* TextureThd */
    {80,   80,    80,    80,    80,    80,     90,     90,     90,   100,   100,    100,    100,    100,    110,   110},
    
    /* EdgeThd */
    {0,     0,     5,    10,    10,    10,     16,     20,     30,    40,    50,     50,     60,     60,     60,    60},
    
    /* JagCtrl */
    {63,   38,    20,    18,    16,    12,     10,      8,      8,     4,     4,      4,      2,      2,      2,     2},
    
    /* SaltCtrl */
    {50,   50,    50,    90,    90,    90,     90,     90,     90,    90,    90,     90,     90,     50,     50,    50},
    
    /* PepperCtrl */
    {0,     0,     0,    20,    60,    60,     60,     80,    120,    160,   180,   180,    180,    180,    180,   180},
    
    /* DetailCtrl */
    {150, 150,   135,   135,   130,   130,    130,    130,    120,    120,   120,   120,    100,     50,     50,    50}, 
    
    /* LumaThd */
    {
        {20,    20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20}, /* LumaThd0 */
        {40,    40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40}, /* LumaThd1 */
        {65,    65,     65,     65,     65,     65,     65,     65,     65,     65,     65,     65,     65,     65,     65,     65}, /* LumaThd2 */
        {90,    90,     90,     90,     90,     90,     90,     90,     90,     90,     90,     90,     90,     90,     90,     90}  /* LumaThd3 */
    },
    
     /* LumaWgt */
    {
        {160,   160,    160,    150,    140,    130,    120,    110,    100,    100,     90,     90,     80,     80,     80,     80},
        {200,   200,    200,    180,    170,    160,    150,    150,    150,    150,    120,    120,    120,    120,    120,    120},
        {240,   240,    240,    200,    200,    190,    180,    180,    180,    180,    160,    160,    160,    160,    160,    160},
        {255,   255,    255,    255,    255,    255,    255,    255,    255,    255,    255,    255,    255,    255,    255,    255},
    }
};

static ISP_CMOS_YUV_SHARPEN_S g_stIspYuvSharpenFSWDR =
{
    /* bvalid */
    1,
    
    /* 100, 200,  400,   800,  1600,  3200,   6400,   12800,  25600, 51200, 102400, 204800, 409600, 819200, 1638400, 3276800 */
    /* bEnLowLumaShoot */ 
    {0,     0,     0,     0,     0,     0,      0,      0,      0,     1,     1,      1,      1,      1,      1,     1},
    
    /* TextureSt */
    {50,   48,    46,    44,    48,    56,     56,     56,     48,    36,    20,      8,      4,      4,      4,     4},
    
    /* EdgeSt */
    {64,   64,    64,    64,    64,    64,     56,     56,     56,    48,    48,     48,     48,     32,     20,    20},
    
    /* OverShoot */
    {32,   36,    40,    48,    48,    48,     44,     40,     40,    32,    32,     28,     25,     20,     20,    20},
    
    /* UnderShoot */
    {32,   36,    40,    48,    48,    48,     44,     40,     40,    32,    32,     28,     25,     20,     20,    20},
    
    /* TextureThd */
    {20,   24,    28,    32,    36,    40,     48,     56,     64,   128,   156,    156,    156,    160,    160,   160},
    
    /* EdgeThd */
    {0,     0,     0,    10,    10,    10,     16,     32,     64,   128,   156,    156,    156,    160,    160,   160},
    
    /* JagCtrl */
    {24,   22,    20,    18,    16,    12,     10,      8,      8,     4,     4,      4,      2,      2,      2,     2},
    
    /* SaltCtrl */
    {50,   50,    50,    90,    90,    90,     90,     90,     90,    90,    90,     90,     90,     50,     50,    50},
    
    /* PepperCtrl */
    {0,     0,     0,    20,    60,    60,     60,     80,    120,   160,   180,    180,    180,    180,    180,   180},
    
    /* DetailCtrl */
    {140,  140,  135,   135,   130,   130,    130,    130,    120,   120,   120,    120,    100,     50,     50,    50}, 
    
    /* LumaThd */
    {
        {20,    20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20,     20}, /* LumaThd0 */
        {40,    40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40,     40}, /* LumaThd1 */
        {65,    65,     65,     65,     65,     65,     70,     75,     80,     80,     80,     80,     80,     80,     80,     80}, /* LumaThd2 */
        {90,    90,     90,     90,     90,     90,     100,   110,    120,    120,    120,    120,    120,    120,    120,    120}  /* LumaThd3 */  
    },
    
    /* LumaWgt */
    {
        {160,   160,    160,    150,    120,     80,     40,     20,     10,    10,     10,     10,     10,     10,     10,     10},
        {200,   200,    200,    180,    160,    120,     80,     60,     40,    40,     40,     40,     40,     40,     40,     40},
        {240,   240,    240,    200,    200,    180,    160,    140,    128,   128,    128,    128,    128,    128,    128,    128},
        {255,   255,    255,    255,    255,    255,    255,    255,    255,   255,    255,    255,    255,    255,    255,    255},
    }
};

static ISP_CMOS_AGC_TABLE_S g_stIspAgcTableFSWDR =
{
    /* bvalid */
    1,
    
    /* snr_thresh */
    {0x8,0xC,0x10,0x14,0x18,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},
    
    /* demosaic_np_offset */
    {0x0,0xa,0x12,0x1a,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},
    
    /* ge_strength */
    {0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37},
};

static ISP_CMOS_NOISE_TABLE_S g_stIspNoiseTable =
{
    /* bvalid */
    1,
    
    /* nosie_profile_weight_lut */
    {
        0,0,0,0,0,0,0,8,14,18,21,24,26,27,28,30,31,32,33,34,34,36,36,37,38,38,39,39,
        40,40,41,41,42,42,42,43,43,44,44,44,45,45,45,46,46,46,46,47,47,47,48,48,48,48,49,49,49,
        49,49,50,50,50,50,51,51,51,51,51,51,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,
        54,55,55,55,55,55,55,55,55,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,58,58,58,
        58,58,58,58,58,58,59,59,59,59,59,59,59
    },
    
    /* demosaic_weight_lut */
    {
        0,8,14,18,21,24,26,27,28,30,31,32,33,34,34,36,36,37,38,38,39,39,40,40,41,41,42,42,42,
        43,43,44,44,44,45,45,45,46,46,46,46,47,47,47,48,48,48,48,49,49,49,49,49,50,50,50,50,
        51,51,51,51,51,51,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,
        55,55,55,55,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,
        58,58,59,59,59,59,59,59,59,59,59,59,59,59,59
    }
};

static ISP_CMOS_NOISE_TABLE_S g_stIspNoiseTableFSWDR =
{
    /* bvalid */
    1,
    
    /* nosie_profile_weight_lut */
    {
        0,0,0,0,0,0,0,8,14,18,21,24,26,27,28,30,31,32,33,34,34,36,36,37,38,38,39,39,
        40,40,41,41,42,42,42,43,43,44,44,44,45,45,45,46,46,46,46,47,47,47,48,48,48,48,49,49,49,
        49,49,50,50,50,50,51,51,51,51,51,51,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,
        54,55,55,55,55,55,55,55,55,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,58,58,58,
        58,58,58,58,58,58,59,59,59,59,59,59,59
    },
    
    /* demosaic_weight_lut */
    {
        0,2,15,20,24,27,30,32,33,35,36,37,38,39,40,41,42,42,43,44,44,45,45,46,
        46,47,47,48,48,49,49,49,50,50,50,51,51,51,52,52,52,53,53,53,53,54,54,54,54,55,55,
        55,55,56,56,56,56,56,57,57,57,57,57,58,58,58,58,58,59,59,59,59,59,59,59,60,60,60,
        60,60,60,61,61,61,61,61,61,61,62,62,62,62,62,62,62,62,63,63,63,63,63,63,63,63,63,
        64,64,64,64,64,64,64,64,64,65,65,65,65,65,65,65,65,65,65,65,65,65,65
    }
};

static ISP_CMOS_DEMOSAIC_S g_stIspDemosaic =
{
    /* bvalid */
    1,
    
    /*vh_slope*/
    0xac,
    
    /*aa_slope*/
    0xaa,
    
    /*va_slope*/
    0xa8,
    
    /*uu_slope*/
    0x55,
    
    /*sat_slope*/
    0x5d,
    
    /*ac_slope*/
    0xa0,
    
    /*fc_slope*/
    0x80,
    
    /*vh_thresh*/
    0x00,
    
    /*aa_thresh*/
    0x00,
    
    /*va_thresh*/
    0x00,
    
    /*uu_thresh*/
    0x08,
    
    /*sat_thresh*/
    0x00,
    
    /*ac_thresh*/
    0x1b3
};

static ISP_CMOS_DEMOSAIC_S g_stIspDemosaicFSWDR =
{
    /* bvalid */
    1,
    
    /*vh_slope*/
    0xac,
    
    /*aa_slope*/
    0xaa,
    
    /*va_slope*/
    0xa8,
    
    /*uu_slope*/
    0x55,
    
    /*sat_slope*/
    0x5d,
    
    /*ac_slope*/
    0xa0,
    
    /*fc_slope*/
    0x80,
    
    /*vh_thresh*/
    0x00,
    
    /*aa_thresh*/
    0x00,
    
    /*va_thresh*/
    0x00,
    
    /*uu_thresh*/
    0x00,
    
    /*sat_thresh*/
    0x00,
    
    /*ac_thresh*/
    0x1b3
};

static ISP_CMOS_GAMMA_S g_stIspGamma =
{
    /* bvalid */
    1,
    {
        0, 180, 320, 426, 516, 590, 660, 730, 786, 844, 896, 946, 994, 1040, 1090, 1130, 1170, 1210, 1248,
        1296, 1336, 1372, 1416, 1452, 1486, 1516, 1546, 1580, 1616, 1652, 1678, 1714, 1742, 1776, 1798, 1830,
        1862, 1886, 1912, 1940, 1968, 1992, 2010, 2038, 2062, 2090, 2114, 2134, 2158, 2178, 2202, 2222, 2246,
        2266, 2282, 2300, 2324, 2344, 2360, 2372, 2390, 2406, 2422, 2438, 2458, 2478, 2494, 2510, 2526, 2546,
        2562, 2582, 2598, 2614, 2630, 2648, 2660, 2670, 2682, 2698, 2710, 2724, 2736, 2752, 2764, 2780, 2792,
        2808, 2820, 2836, 2848, 2864, 2876, 2888, 2896, 2908, 2920, 2928, 2940, 2948, 2960, 2972, 2984, 2992,
        3004, 3014, 3028, 3036, 3048, 3056, 3068, 3080, 3088, 3100, 3110, 3120, 3128, 3140, 3148, 3160, 3168,
        3174, 3182, 3190, 3202, 3210, 3218, 3228, 3240, 3256, 3266, 3276, 3288, 3300, 3306, 3318, 3326, 3334,
        3342, 3350, 3360, 3370, 3378, 3386, 3394, 3398, 3406, 3414, 3422, 3426, 3436, 3444, 3454, 3466, 3476,
        3486, 3498, 3502, 3510, 3518, 3526, 3530, 3538, 3546, 3554, 3558, 3564, 3570, 3574, 3582, 3590, 3598,
        3604, 3610, 3618, 3628, 3634, 3640, 3644, 3652, 3656, 3664, 3670, 3678, 3688, 3696, 3700, 3708, 3712,
        3716, 3722, 3730, 3736, 3740, 3748, 3752, 3756, 3760, 3766, 3774, 3778, 3786, 3790, 3800, 3808, 3812,
        3816, 3824, 3830, 3832, 3842, 3846, 3850, 3854, 3858, 3862, 3864, 3870, 3874, 3878, 3882, 3888, 3894,
        3900, 3908, 3912, 3918, 3924, 3928, 3934, 3940, 3946, 3952, 3958, 3966, 3974, 3978, 3982, 3986, 3990,
        3994, 4002, 4006, 4010, 4018, 4022, 4032, 4038, 4046, 4050, 4056, 4062, 4072, 4076, 4084, 4090, 4095
    }
};

static ISP_CMOS_GAMMA_S g_stIspGammaFSWDR =
{
    /* bvalid */
    1,
    
    {
        0,1,2,4,8,12,17,23,30,38,47,57,68,79,92,105,120,133,147,161,176,192,209,226,243,260,278,296,
        317,340,365,390,416,440,466,491,517,538,561,584,607,631,656,680,705,730,756,784,812,835,
        858,882,908,934,958,982,1008,1036,1064,1092,1119,1143,1167,1192,1218,1243,1269,1294,1320,
        1346,1372,1398,1424,1450,1476,1502,1528,1554,1580,1607,1633,1658,1684,1710,1735,1761,1786,
        1811,1836,1860,1884,1908,1932,1956,1979,2002,2024,2046,2068,2090,2112,2133,2154,2175,2196,
        2217,2237,2258,2278,2298,2318,2337,2357,2376,2395,2414,2433,2451,2469,2488,2505,2523,2541,
        2558,2575,2592,2609,2626,2642,2658,2674,2690,2705,2720,2735,2750,2765,2779,2793,2807,2821,
        2835,2848,2861,2874,2887,2900,2913,2925,2937,2950,2962,2974,2986,2998,3009,3021,3033,3044,
        3056,3067,3078,3088,3099,3109,3119,3129,3139,3148,3158,3168,3177,3187,3197,3207,3217,3227,
        3238,3248,3259,3270,3281,3292,3303,3313,3324,3335,3346,3357,3368,3379,3389,3400,3410,3421,
        3431,3441,3451,3461,3471,3481,3491,3501,3511,3521,3531,3541,3552,3562,3572,3583,3593,3604,
        3615,3625,3636,3646,3657,3668,3679,3689,3700,3711,3721,3732,3743,3753,3764,3774,3784,3795,
        3805,3816,3826,3837,3847,3858,3869,3880,3891,3902,3913,3925,3937,3949,3961,3973,3985,3997,
        4009,4022,4034,4046,4058,4071,4083,4095
    }
};

//recommend Lens Shading Table
static ISP_CMOS_MESH_SHADING_S g_stIspMeshShading_4M =
{
    1,
    {5000,5000,5000,5000},
    0,
    1,
    {
        {
			128, 123, 117, 113, 109, 105, 102, 98, 96, 94, 91, 89, 87, 85, 84, 82, 81, 80, 78, 77, 76, 75, 74, 74, 72, 72, 71, 71, 71, 70, 70, 69, 69, 69, 69, 69, 70, 71, 71, 71, 71, 72, 72, 73, 74, 75, 75, 76, 78, 79, 80, 81, 83, 85, 86, 88, 91, 93, 95, 98, 101, 104, 108, 112, 
			127, 122, 117, 113, 108, 104, 101, 98, 95, 93, 91, 89, 87, 85, 84, 82, 81, 80, 78, 77, 76, 75, 74, 73, 72, 72, 71, 71, 70, 70, 70, 69, 69, 69, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 74, 74, 75, 76, 77, 79, 80, 81, 83, 85, 86, 88, 90, 92, 95, 98, 101, 104, 107, 111, 
			126, 121, 116, 112, 107, 103, 100, 97, 95, 93, 91, 89, 87, 85, 84, 82, 80, 79, 78, 77, 75, 74, 74, 73, 72, 71, 71, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 74, 75, 76, 77, 78, 80, 81, 82, 84, 86, 88, 90, 91, 94, 97, 100, 103, 106, 110, 
			124, 121, 116, 110, 106, 103, 100, 97, 94, 92, 90, 88, 86, 85, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 70, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 71, 71, 72, 73, 74, 74, 75, 77, 78, 79, 81, 82, 83, 85, 87, 89, 91, 93, 96, 99, 102, 106, 109, 
			123, 120, 114, 109, 105, 102, 99, 96, 94, 91, 89, 87, 86, 84, 82, 81, 79, 78, 77, 75, 75, 74, 72, 72, 71, 71, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 70, 70, 71, 72, 72, 73, 74, 75, 76, 77, 79, 80, 81, 83, 85, 87, 88, 91, 93, 96, 98, 101, 105, 109, 
			121, 118, 113, 108, 105, 102, 98, 96, 93, 91, 89, 87, 85, 83, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 70, 70, 71, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 88, 91, 93, 95, 98, 101, 104, 107, 
			120, 116, 111, 107, 104, 101, 98, 95, 92, 90, 88, 86, 84, 82, 81, 80, 78, 77, 76, 74, 74, 72, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 69, 69, 70, 71, 71, 72, 73, 74, 75, 76, 77, 78, 80, 81, 83, 85, 87, 90, 92, 94, 97, 100, 103, 106, 
			118, 114, 110, 107, 103, 100, 97, 94, 91, 89, 87, 85, 83, 82, 81, 79, 78, 76, 75, 74, 73, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 69, 69, 70, 71, 71, 72, 73, 74, 75, 76, 78, 79, 81, 83, 84, 86, 89, 91, 93, 96, 99, 102, 105, 
			117, 114, 110, 106, 102, 99, 96, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 71, 69, 69, 68, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 69, 69, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 88, 90, 92, 95, 98, 101, 104, 
			116, 113, 109, 105, 101, 97, 95, 92, 90, 88, 86, 84, 82, 81, 79, 77, 77, 75, 74, 73, 72, 71, 70, 69, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 68, 69, 70, 71, 71, 72, 73, 74, 76, 77, 78, 80, 82, 83, 85, 87, 89, 92, 95, 97, 100, 103, 
			115, 112, 108, 104, 100, 97, 95, 92, 90, 87, 85, 84, 82, 80, 78, 77, 76, 75, 74, 72, 71, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 84, 87, 89, 91, 94, 97, 99, 102, 
			114, 111, 107, 103, 100, 97, 94, 91, 89, 87, 85, 83, 81, 80, 78, 77, 76, 74, 73, 72, 71, 70, 69, 69, 68, 68, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 70, 71, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 88, 91, 93, 96, 99, 102, 
			113, 110, 107, 103, 99, 96, 94, 91, 89, 87, 85, 83, 81, 79, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 80, 82, 83, 85, 87, 90, 93, 96, 98, 101, 
			113, 109, 105, 102, 98, 95, 93, 91, 88, 86, 84, 82, 80, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 79, 81, 83, 85, 87, 89, 92, 95, 98, 100, 
			113, 109, 105, 101, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 67, 68, 69, 69, 70, 71, 72, 73, 75, 77, 78, 79, 81, 82, 84, 86, 89, 92, 94, 97, 99, 
			112, 108, 104, 101, 97, 94, 92, 90, 88, 86, 84, 82, 80, 78, 76, 75, 74, 72, 71, 71, 69, 69, 68, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 82, 84, 86, 88, 91, 94, 97, 99, 
			111, 108, 104, 100, 97, 94, 92, 90, 88, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 80, 82, 84, 86, 88, 90, 93, 96, 98, 
			111, 107, 103, 99, 96, 94, 91, 89, 88, 85, 82, 81, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 72, 73, 74, 75, 77, 78, 80, 82, 83, 85, 87, 90, 92, 95, 98, 
			109, 106, 102, 99, 96, 94, 91, 89, 87, 84, 82, 80, 79, 77, 75, 74, 73, 72, 71, 70, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 81, 83, 85, 87, 90, 92, 95, 98, 
			109, 106, 102, 98, 95, 93, 91, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 87, 89, 92, 95, 98, 
			109, 105, 102, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 89, 92, 95, 97, 
			108, 105, 101, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 70, 72, 73, 74, 76, 77, 79, 81, 82, 84, 86, 89, 92, 94, 97, 
			107, 104, 101, 98, 95, 92, 90, 87, 85, 83, 81, 79, 77, 76, 75, 73, 72, 71, 69, 69, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 72, 73, 74, 75, 77, 78, 80, 82, 84, 86, 89, 91, 94, 96, 
			107, 104, 100, 98, 95, 92, 89, 87, 85, 83, 81, 79, 77, 75, 74, 73, 72, 71, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 74, 75, 76, 78, 80, 82, 84, 86, 89, 91, 93, 96, 
			107, 104, 100, 98, 95, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 73, 75, 75, 76, 78, 80, 82, 83, 86, 88, 91, 93, 96, 
			107, 104, 101, 97, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 71, 69, 68, 67, 67, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 73, 75, 75, 76, 78, 79, 81, 83, 86, 88, 91, 93, 96, 
			108, 106, 102, 97, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 71, 69, 68, 67, 66, 66, 66, 66, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 73, 74, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 95, 
			108, 107, 101, 96, 94, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 71, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 95, 
			109, 108, 102, 98, 94, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 72, 73, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 95, 
			108, 106, 101, 99, 96, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 64, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 95, 
			107, 103, 100, 100, 97, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 64, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 95, 
			106, 103, 100, 100, 96, 91, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 69, 69, 67, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 64, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 78, 79, 81, 83, 85, 87, 90, 93, 95, 
			107, 103, 100, 98, 95, 91, 89, 87, 85, 83, 81, 80, 78, 76, 74, 73, 72, 71, 69, 69, 67, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 88, 90, 93, 95, 
			107, 103, 101, 98, 95, 91, 89, 87, 85, 83, 81, 79, 78, 76, 75, 73, 72, 71, 69, 69, 67, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 88, 91, 93, 95, 
			107, 104, 102, 99, 95, 92, 90, 87, 85, 83, 81, 79, 78, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 83, 85, 88, 91, 93, 96, 
			107, 104, 102, 99, 96, 92, 90, 87, 85, 83, 81, 80, 78, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 83, 85, 88, 91, 93, 96, 
			108, 105, 102, 99, 95, 92, 90, 87, 85, 83, 81, 80, 78, 77, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 84, 86, 88, 91, 94, 96, 
			108, 105, 101, 98, 95, 92, 90, 88, 86, 83, 81, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 69, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 88, 91, 94, 96, 
			108, 105, 102, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 69, 70, 71, 72, 74, 75, 77, 79, 80, 82, 84, 86, 88, 91, 93, 96, 
			108, 105, 102, 98, 95, 93, 91, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 77, 79, 80, 82, 84, 87, 89, 91, 94, 96, 
			109, 106, 102, 99, 96, 93, 91, 89, 86, 84, 82, 81, 79, 77, 76, 74, 73, 72, 71, 69, 69, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 69, 70, 72, 73, 74, 75, 77, 79, 81, 83, 85, 87, 89, 92, 95, 97, 
			110, 106, 102, 99, 96, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 89, 92, 95, 98, 
			110, 107, 103, 100, 96, 94, 92, 89, 87, 85, 83, 81, 79, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 96, 98, 
			110, 108, 104, 100, 97, 94, 91, 89, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 71, 69, 68, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 81, 83, 85, 88, 90, 93, 96, 98, 
			111, 108, 104, 101, 97, 95, 92, 90, 88, 86, 83, 81, 80, 78, 77, 75, 74, 73, 71, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 72, 73, 74, 75, 77, 78, 80, 82, 83, 86, 88, 91, 93, 96, 98, 
			111, 108, 104, 101, 98, 95, 92, 90, 88, 86, 84, 82, 80, 79, 77, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 77, 78, 80, 82, 84, 86, 88, 91, 94, 96, 99, 
			111, 108, 105, 102, 98, 96, 93, 90, 88, 87, 84, 82, 81, 79, 77, 76, 75, 74, 72, 71, 70, 69, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 80, 82, 84, 87, 89, 91, 94, 97, 99, 
			112, 109, 106, 102, 99, 96, 94, 91, 89, 87, 85, 83, 81, 79, 78, 77, 75, 74, 73, 71, 70, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 71, 72, 74, 75, 76, 77, 79, 81, 83, 84, 87, 89, 92, 94, 97, 100, 
			114, 110, 107, 103, 100, 97, 94, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 69, 68, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 75, 77, 79, 80, 81, 83, 85, 87, 89, 92, 95, 98, 100, 
			114, 111, 107, 103, 100, 97, 95, 92, 90, 88, 86, 83, 81, 80, 79, 77, 76, 75, 73, 72, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 69, 70, 71, 71, 72, 73, 75, 76, 79, 81, 81, 82, 83, 86, 88, 90, 93, 96, 99, 101, 
			115, 112, 108, 104, 100, 98, 95, 93, 90, 88, 86, 84, 82, 81, 79, 78, 77, 75, 74, 72, 72, 71, 70, 69, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 69, 69, 70, 71, 71, 72, 74, 75, 76, 80, 82, 81, 82, 84, 86, 89, 91, 93, 96, 99, 102, 
			116, 113, 108, 104, 101, 98, 95, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 75, 73, 72, 71, 71, 69, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 68, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 80, 82, 81, 83, 85, 87, 89, 91, 94, 97, 100, 103, 
			117, 113, 109, 105, 102, 99, 96, 94, 91, 89, 87, 85, 84, 82, 80, 78, 77, 76, 75, 74, 72, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 66, 67, 67, 67, 67, 68, 68, 69, 69, 70, 71, 71, 72, 73, 75, 76, 77, 79, 80, 81, 83, 85, 87, 89, 92, 95, 97, 100, 103, 
			118, 114, 110, 106, 102, 99, 97, 94, 92, 90, 88, 86, 84, 82, 81, 79, 78, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 68, 68, 67, 67, 67, 67, 66, 67, 67, 67, 67, 67, 68, 68, 69, 69, 70, 71, 71, 72, 73, 74, 75, 76, 78, 79, 80, 82, 84, 86, 88, 90, 93, 95, 98, 101, 104, 
			119, 115, 111, 107, 103, 100, 97, 95, 92, 90, 88, 86, 84, 83, 81, 80, 78, 77, 75, 75, 74, 72, 72, 71, 70, 70, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 71, 72, 72, 73, 75, 76, 77, 78, 80, 81, 83, 85, 87, 89, 91, 93, 96, 99, 102, 104, 
			120, 116, 112, 108, 104, 101, 98, 95, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 69, 68, 68, 68, 68, 67, 67, 68, 68, 68, 68, 69, 69, 70, 70, 71, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 83, 85, 87, 89, 91, 94, 97, 99, 102, 105, 
			121, 117, 113, 109, 105, 102, 99, 96, 94, 92, 90, 88, 86, 84, 82, 81, 79, 78, 77, 76, 74, 73, 72, 72, 71, 71, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 70, 70, 71, 71, 72, 73, 74, 74, 75, 77, 78, 79, 81, 82, 84, 86, 88, 90, 92, 94, 97, 100, 103, 106, 
			122, 118, 114, 110, 106, 103, 100, 97, 95, 93, 90, 88, 86, 85, 83, 81, 80, 79, 77, 76, 75, 74, 73, 72, 71, 71, 71, 70, 70, 69, 69, 69, 69, 68, 68, 68, 68, 69, 69, 70, 70, 71, 71, 72, 73, 74, 74, 75, 76, 78, 79, 80, 81, 83, 85, 86, 88, 91, 93, 95, 98, 101, 104, 108, 
			124, 120, 115, 111, 107, 103, 100, 98, 95, 93, 91, 89, 87, 85, 83, 82, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 71, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 86, 87, 89, 92, 94, 96, 99, 102, 105, 109, 
			126, 121, 116, 112, 108, 104, 101, 99, 96, 94, 92, 90, 88, 86, 84, 82, 81, 80, 78, 77, 76, 75, 74, 74, 73, 72, 71, 71, 70, 70, 70, 70, 69, 69, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 74, 75, 75, 76, 78, 79, 80, 81, 83, 85, 86, 88, 90, 92, 94, 97, 100, 103, 106, 110, 
			128, 123, 118, 113, 109, 106, 103, 100, 97, 95, 93, 90, 88, 87, 85, 83, 81, 80, 79, 78, 77, 76, 75, 74, 74, 72, 72, 71, 71, 71, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 72, 72, 73, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 85, 87, 89, 91, 93, 95, 98, 101, 104, 107, 111, 
			129, 124, 119, 114, 110, 107, 104, 101, 98, 96, 93, 91, 89, 88, 86, 84, 82, 81, 80, 79, 77, 77, 75, 74, 74, 73, 72, 72, 72, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 72, 72, 73, 74, 74, 75, 76, 77, 78, 79, 80, 82, 83, 85, 86, 88, 90, 92, 94, 96, 99, 102, 105, 108, 112, 
			131, 126, 120, 115, 111, 108, 105, 102, 99, 96, 94, 92, 90, 88, 87, 85, 83, 82, 80, 79, 78, 77, 76, 75, 74, 74, 73, 72, 72, 72, 72, 71, 71, 71, 71, 71, 71, 71, 72, 72, 73, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 86, 87, 88, 90, 93, 95, 97, 100, 103, 106, 110, 113, 
			133, 127, 122, 116, 112, 109, 106, 103, 100, 97, 95, 92, 90, 89, 87, 85, 84, 82, 80, 80, 79, 77, 77, 75, 75, 74, 74, 73, 72, 72, 72, 71, 71, 71, 72, 72, 72, 72, 72, 72, 73, 74, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 86, 88, 89, 91, 93, 95, 98, 101, 104, 107, 111, 114, 
        },
        
        {
			129, 124, 119, 115, 110, 106, 103, 100, 97, 94, 92, 90, 88, 87, 85, 83, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 72, 72, 71, 71, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 71, 72, 73, 73, 74, 75, 76, 77, 78, 79, 80, 82, 83, 85, 87, 88, 91, 93, 96, 99, 102, 105, 108, 112, 
			128, 123, 118, 114, 109, 105, 102, 99, 96, 94, 92, 90, 88, 86, 84, 83, 81, 79, 78, 77, 76, 75, 74, 74, 73, 72, 72, 71, 71, 70, 70, 70, 70, 70, 69, 70, 70, 70, 70, 71, 71, 72, 72, 73, 74, 74, 75, 76, 77, 78, 80, 81, 83, 84, 86, 88, 90, 92, 95, 98, 101, 104, 107, 111, 
			128, 122, 117, 112, 108, 104, 101, 98, 95, 93, 91, 89, 87, 85, 84, 82, 80, 79, 78, 76, 75, 75, 74, 73, 72, 71, 71, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 85, 87, 89, 91, 94, 97, 100, 103, 106, 110, 
			126, 121, 116, 111, 106, 103, 100, 97, 95, 92, 90, 88, 86, 85, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 72, 71, 71, 70, 69, 69, 69, 69, 68, 68, 68, 69, 69, 69, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 76, 77, 78, 80, 82, 83, 85, 87, 89, 91, 93, 96, 99, 102, 105, 109, 
			124, 119, 114, 110, 106, 103, 99, 96, 94, 92, 90, 88, 86, 84, 82, 81, 79, 78, 76, 75, 75, 74, 73, 72, 71, 70, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 70, 70, 71, 72, 73, 74, 75, 75, 76, 78, 80, 81, 82, 84, 86, 88, 91, 93, 95, 98, 101, 104, 108, 
			122, 117, 113, 109, 105, 101, 98, 96, 93, 91, 89, 87, 85, 83, 82, 80, 78, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 69, 69, 68, 68, 68, 68, 67, 67, 68, 68, 68, 68, 69, 69, 70, 70, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 82, 83, 85, 87, 90, 92, 94, 97, 100, 103, 107, 
			121, 116, 112, 108, 104, 101, 98, 95, 92, 90, 88, 86, 84, 82, 81, 79, 78, 77, 75, 74, 73, 72, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 69, 69, 70, 71, 71, 72, 73, 75, 76, 77, 78, 79, 81, 83, 85, 87, 89, 92, 94, 96, 99, 102, 106, 
			120, 115, 111, 107, 103, 101, 97, 94, 92, 90, 87, 85, 83, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 70, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 82, 84, 86, 88, 90, 93, 96, 99, 102, 105, 
			119, 114, 110, 106, 102, 99, 96, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 75, 73, 72, 71, 70, 70, 69, 68, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 68, 69, 70, 71, 71, 72, 73, 74, 75, 77, 78, 80, 81, 83, 85, 87, 89, 92, 95, 98, 101, 104, 
			117, 113, 109, 105, 101, 98, 95, 93, 90, 88, 86, 84, 82, 80, 79, 78, 76, 75, 74, 73, 72, 71, 70, 69, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 89, 91, 94, 97, 100, 103, 
			117, 112, 108, 105, 101, 97, 94, 92, 90, 88, 85, 83, 82, 80, 78, 77, 76, 75, 74, 72, 71, 70, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 68, 68, 68, 69, 70, 70, 71, 72, 73, 74, 76, 77, 79, 80, 82, 84, 86, 88, 90, 93, 96, 100, 102, 
			115, 112, 107, 103, 100, 97, 94, 91, 89, 87, 85, 83, 81, 80, 78, 76, 76, 75, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 71, 72, 73, 74, 75, 77, 78, 80, 82, 83, 85, 87, 90, 93, 96, 99, 101, 
			114, 111, 107, 103, 99, 96, 94, 91, 89, 87, 85, 83, 81, 79, 78, 76, 75, 74, 73, 71, 70, 70, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 70, 70, 71, 73, 74, 75, 77, 78, 79, 81, 83, 85, 87, 89, 92, 95, 98, 101, 
			114, 110, 106, 102, 98, 95, 93, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 73, 75, 76, 77, 79, 81, 82, 84, 87, 89, 91, 94, 97, 100, 
			114, 109, 105, 102, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 80, 82, 84, 86, 88, 91, 94, 97, 100, 
			113, 109, 105, 101, 97, 95, 92, 90, 88, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 70, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 69, 70, 71, 73, 74, 75, 77, 78, 79, 81, 83, 85, 88, 90, 93, 96, 99, 
			112, 108, 104, 100, 97, 94, 91, 89, 88, 85, 83, 81, 79, 78, 76, 75, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 77, 78, 79, 81, 83, 85, 88, 90, 93, 96, 99, 
			111, 107, 103, 100, 96, 94, 91, 89, 87, 84, 82, 80, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 87, 90, 92, 95, 98, 
			110, 107, 103, 99, 96, 94, 91, 89, 87, 84, 82, 80, 79, 77, 76, 74, 73, 72, 71, 69, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 80, 82, 84, 87, 89, 92, 95, 97, 
			109, 106, 103, 99, 96, 93, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 73, 74, 76, 77, 79, 80, 82, 84, 86, 89, 91, 94, 97, 
			109, 106, 102, 99, 95, 92, 90, 88, 85, 83, 82, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 77, 78, 80, 82, 84, 86, 89, 91, 94, 97, 
			109, 106, 102, 98, 95, 92, 90, 88, 85, 83, 81, 79, 78, 76, 75, 74, 72, 71, 70, 68, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 81, 84, 86, 88, 91, 94, 96, 
			108, 105, 101, 98, 95, 92, 90, 87, 85, 83, 81, 79, 78, 76, 75, 73, 72, 71, 69, 68, 67, 67, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 78, 79, 81, 83, 86, 88, 91, 94, 96, 
			108, 105, 101, 98, 95, 92, 89, 87, 85, 83, 81, 79, 78, 76, 74, 73, 72, 70, 69, 68, 67, 67, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 86, 88, 91, 93, 96, 
			108, 104, 100, 97, 94, 92, 89, 87, 85, 83, 81, 79, 78, 76, 74, 73, 72, 70, 69, 68, 67, 67, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 76, 77, 79, 81, 83, 85, 88, 91, 93, 96, 
			108, 105, 101, 97, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 70, 69, 68, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 74, 75, 77, 79, 81, 83, 85, 88, 90, 93, 95, 
			108, 106, 101, 97, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 70, 69, 68, 67, 67, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 67, 68, 68, 69, 70, 72, 74, 74, 75, 77, 79, 80, 83, 85, 88, 90, 93, 95, 
			109, 108, 101, 97, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 70, 69, 68, 67, 67, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 73, 74, 75, 77, 79, 80, 83, 85, 87, 90, 93, 95, 
			109, 108, 102, 98, 95, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 77, 79, 80, 82, 85, 87, 90, 93, 95, 
			109, 107, 102, 100, 96, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 71, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 77, 79, 80, 82, 85, 87, 90, 93, 95, 
			108, 105, 101, 101, 97, 92, 89, 87, 85, 83, 81, 79, 78, 76, 74, 73, 72, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 77, 78, 80, 82, 84, 87, 90, 93, 95, 
			107, 104, 101, 101, 97, 92, 89, 87, 85, 83, 81, 79, 78, 76, 74, 73, 72, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 77, 78, 80, 82, 85, 87, 90, 93, 95, 
			107, 104, 101, 99, 95, 92, 90, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 70, 69, 68, 68, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 74, 75, 77, 78, 80, 82, 85, 87, 90, 93, 95, 
			108, 104, 101, 98, 95, 92, 90, 87, 85, 83, 81, 79, 78, 76, 75, 73, 72, 70, 70, 68, 68, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 73, 74, 75, 77, 78, 80, 82, 85, 87, 90, 93, 95, 
			108, 105, 102, 99, 95, 92, 90, 87, 85, 83, 81, 79, 78, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 73, 74, 76, 77, 79, 80, 82, 85, 87, 90, 93, 95, 
			109, 105, 102, 99, 96, 92, 90, 87, 85, 83, 81, 80, 78, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 76, 77, 79, 80, 83, 85, 87, 90, 93, 95, 
			109, 105, 102, 99, 95, 92, 90, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 76, 77, 79, 81, 83, 85, 88, 91, 93, 95, 
			109, 105, 101, 98, 95, 92, 90, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 67, 68, 68, 69, 70, 72, 73, 74, 76, 77, 79, 81, 83, 85, 88, 91, 93, 96, 
			109, 105, 101, 98, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 84, 86, 88, 91, 93, 96, 
			109, 106, 102, 99, 96, 93, 91, 88, 86, 84, 82, 80, 79, 77, 75, 74, 73, 71, 70, 69, 68, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 81, 84, 86, 89, 91, 94, 96, 
			109, 106, 103, 99, 96, 94, 92, 89, 86, 84, 82, 80, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 96, 
			110, 106, 103, 100, 96, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 67, 68, 69, 69, 70, 71, 73, 74, 75, 77, 78, 80, 82, 84, 86, 89, 91, 94, 97, 
			111, 107, 103, 100, 97, 94, 92, 89, 87, 85, 83, 81, 79, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 69, 70, 71, 73, 74, 76, 77, 79, 80, 82, 84, 87, 89, 92, 95, 98, 
			111, 107, 103, 100, 97, 94, 92, 89, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 90, 92, 95, 98, 
			112, 108, 104, 101, 97, 95, 92, 90, 88, 86, 84, 82, 80, 78, 76, 75, 74, 73, 72, 70, 70, 69, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 87, 90, 92, 95, 98, 
			112, 109, 105, 101, 98, 95, 93, 90, 88, 86, 84, 82, 80, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 81, 83, 85, 88, 91, 93, 96, 98, 
			113, 109, 105, 102, 98, 95, 93, 90, 88, 86, 84, 82, 81, 79, 77, 76, 75, 73, 72, 71, 70, 70, 69, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 78, 80, 82, 84, 86, 88, 91, 93, 96, 99, 
			113, 110, 106, 102, 99, 96, 94, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 71, 70, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 77, 78, 80, 82, 84, 87, 89, 91, 94, 97, 100, 
			114, 111, 107, 103, 100, 97, 94, 92, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 67, 68, 69, 69, 70, 71, 72, 74, 75, 77, 78, 79, 81, 82, 85, 87, 89, 92, 95, 98, 100, 
			115, 111, 107, 104, 100, 97, 95, 92, 90, 88, 86, 84, 82, 80, 79, 77, 76, 75, 74, 72, 71, 70, 70, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 71, 72, 73, 74, 75, 78, 80, 80, 81, 83, 85, 87, 90, 92, 95, 98, 101, 
			116, 112, 108, 104, 101, 98, 95, 92, 90, 88, 86, 84, 82, 81, 79, 78, 76, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 70, 71, 72, 73, 74, 76, 80, 82, 81, 82, 83, 86, 88, 90, 93, 96, 99, 102, 
			117, 113, 109, 105, 101, 98, 96, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 70, 70, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 75, 76, 80, 82, 81, 82, 84, 86, 89, 91, 94, 96, 99, 103, 
			118, 114, 109, 105, 102, 99, 96, 94, 92, 90, 88, 86, 84, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 70, 70, 71, 72, 73, 74, 76, 77, 79, 80, 81, 83, 85, 87, 89, 92, 94, 97, 100, 103, 
			119, 115, 110, 106, 103, 100, 97, 94, 92, 90, 88, 86, 84, 82, 81, 79, 78, 76, 75, 74, 73, 72, 71, 70, 70, 69, 69, 68, 68, 67, 67, 67, 67, 66, 67, 67, 67, 67, 67, 67, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 82, 83, 85, 87, 90, 92, 95, 98, 101, 104, 
			120, 116, 111, 107, 103, 100, 98, 95, 93, 90, 88, 86, 85, 83, 81, 80, 78, 77, 76, 74, 74, 73, 72, 71, 70, 70, 69, 69, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 74, 76, 77, 78, 79, 81, 82, 84, 86, 88, 91, 93, 96, 99, 102, 105, 
			121, 116, 112, 108, 104, 101, 98, 96, 94, 91, 89, 87, 85, 83, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 71, 70, 70, 69, 69, 68, 68, 68, 68, 67, 67, 67, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 81, 83, 85, 87, 89, 91, 94, 96, 99, 103, 106, 
			122, 117, 113, 109, 106, 102, 99, 96, 94, 92, 90, 88, 86, 84, 83, 81, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 70, 69, 69, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 85, 87, 89, 92, 95, 97, 100, 104, 107, 
			123, 119, 114, 110, 106, 103, 100, 97, 95, 93, 91, 88, 86, 85, 83, 81, 80, 79, 77, 76, 75, 74, 73, 72, 72, 71, 71, 70, 70, 69, 69, 69, 68, 68, 68, 68, 69, 69, 69, 69, 70, 70, 71, 72, 73, 74, 74, 75, 76, 77, 79, 80, 81, 83, 84, 86, 88, 90, 93, 95, 98, 101, 105, 108, 
			125, 120, 116, 111, 107, 104, 101, 98, 96, 94, 92, 89, 87, 86, 84, 82, 81, 79, 78, 77, 76, 75, 74, 73, 72, 72, 71, 71, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 80, 82, 84, 85, 87, 89, 91, 94, 96, 99, 102, 106, 109, 
			127, 122, 117, 113, 109, 105, 102, 99, 97, 94, 92, 90, 88, 86, 85, 83, 81, 80, 79, 78, 77, 75, 74, 74, 73, 72, 72, 71, 71, 70, 70, 70, 70, 69, 69, 70, 70, 70, 70, 71, 71, 71, 72, 73, 73, 74, 75, 76, 77, 79, 80, 81, 83, 84, 86, 88, 90, 92, 95, 97, 100, 103, 107, 110, 
			129, 124, 118, 114, 110, 106, 103, 100, 98, 95, 93, 91, 89, 87, 85, 84, 82, 80, 79, 78, 77, 76, 75, 74, 73, 73, 72, 72, 71, 71, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 72, 73, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 85, 87, 89, 91, 93, 95, 98, 101, 104, 108, 111, 
			131, 125, 120, 115, 111, 107, 104, 102, 99, 96, 94, 92, 90, 88, 86, 84, 83, 81, 80, 79, 78, 77, 76, 75, 74, 74, 73, 72, 72, 71, 71, 71, 71, 71, 71, 71, 71, 71, 72, 72, 72, 73, 73, 74, 75, 76, 77, 78, 79, 80, 81, 83, 84, 86, 87, 89, 92, 94, 96, 99, 102, 105, 109, 113, 
			132, 127, 121, 116, 112, 108, 106, 103, 100, 97, 95, 93, 91, 89, 87, 85, 84, 82, 81, 80, 78, 78, 76, 76, 75, 74, 74, 73, 72, 72, 72, 72, 71, 71, 71, 71, 71, 72, 72, 73, 73, 74, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 87, 88, 90, 93, 95, 97, 100, 103, 106, 110, 114, 
			134, 129, 123, 117, 113, 110, 106, 104, 101, 98, 96, 93, 91, 89, 87, 86, 84, 82, 81, 80, 79, 78, 77, 76, 75, 74, 74, 73, 73, 72, 72, 72, 72, 72, 72, 72, 72, 72, 73, 73, 73, 74, 74, 75, 76, 77, 78, 79, 80, 81, 83, 84, 85, 87, 89, 91, 93, 95, 98, 101, 104, 107, 111, 115, 
        },
        
        {
     		126, 121, 116, 112, 108, 104, 101, 98, 95, 93, 91, 89, 87, 85, 84, 82, 81, 80, 78, 77, 76, 75, 74, 74, 73, 72, 72, 72, 71, 70, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 72, 72, 72, 73, 74, 75, 75, 76, 77, 78, 80, 81, 82, 84, 85, 87, 89, 91, 93, 96, 99, 102, 106, 110, 
			124, 119, 114, 110, 107, 103, 100, 97, 94, 92, 90, 88, 86, 84, 83, 81, 80, 79, 78, 76, 75, 75, 74, 73, 72, 72, 71, 71, 70, 70, 69, 69, 69, 69, 69, 69, 70, 70, 70, 70, 71, 71, 72, 72, 73, 74, 74, 75, 77, 78, 79, 81, 82, 84, 85, 87, 89, 91, 93, 96, 99, 102, 105, 109, 
			123, 119, 114, 109, 105, 102, 98, 96, 93, 91, 89, 87, 85, 84, 82, 81, 80, 78, 77, 76, 75, 74, 73, 72, 72, 71, 71, 70, 70, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 74, 75, 76, 77, 79, 80, 81, 83, 85, 87, 88, 90, 92, 95, 98, 101, 105, 108, 
			122, 118, 113, 108, 104, 101, 98, 95, 93, 90, 88, 87, 85, 83, 81, 80, 79, 77, 76, 75, 74, 74, 73, 72, 71, 70, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 70, 70, 70, 71, 72, 73, 74, 74, 75, 76, 78, 80, 81, 82, 84, 85, 87, 90, 92, 94, 97, 100, 103, 108, 
			120, 116, 111, 107, 103, 100, 97, 94, 92, 90, 88, 86, 84, 83, 81, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 72, 73, 74, 75, 76, 77, 79, 80, 81, 83, 85, 87, 89, 92, 93, 96, 99, 102, 106, 
			119, 114, 110, 106, 102, 99, 96, 93, 91, 89, 87, 85, 84, 82, 80, 79, 78, 76, 75, 74, 73, 73, 72, 71, 70, 70, 69, 68, 68, 68, 68, 67, 67, 67, 67, 68, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 72, 73, 74, 75, 77, 78, 79, 81, 83, 85, 86, 89, 91, 92, 95, 98, 102, 105, 
			117, 113, 109, 105, 101, 99, 96, 93, 90, 88, 87, 85, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 82, 84, 85, 88, 90, 92, 94, 97, 101, 104, 
			116, 112, 108, 104, 101, 98, 95, 92, 90, 88, 86, 84, 82, 81, 79, 78, 77, 75, 74, 73, 72, 72, 70, 70, 69, 68, 68, 68, 67, 67, 67, 67, 66, 66, 66, 67, 67, 67, 67, 67, 68, 68, 69, 69, 70, 71, 72, 72, 73, 74, 75, 77, 78, 79, 81, 83, 85, 87, 89, 91, 94, 97, 100, 103, 
			115, 111, 107, 103, 100, 97, 94, 92, 89, 87, 85, 84, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 68, 69, 70, 71, 71, 72, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 88, 90, 93, 96, 99, 102, 
			114, 110, 106, 103, 99, 96, 93, 90, 88, 87, 85, 83, 81, 79, 78, 77, 75, 75, 73, 72, 71, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 68, 69, 70, 70, 71, 72, 73, 74, 76, 77, 79, 81, 82, 83, 85, 87, 89, 92, 95, 98, 101, 
			113, 109, 105, 102, 98, 95, 92, 90, 88, 86, 84, 82, 80, 79, 78, 77, 75, 74, 73, 72, 71, 70, 69, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 69, 70, 70, 71, 72, 73, 74, 75, 77, 78, 79, 81, 83, 85, 87, 89, 92, 95, 98, 101, 
			112, 108, 104, 101, 97, 94, 92, 90, 88, 86, 84, 82, 80, 79, 77, 76, 75, 74, 73, 72, 70, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 65, 66, 66, 65, 66, 66, 66, 66, 66, 66, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 79, 81, 82, 85, 87, 89, 91, 94, 97, 100, 
			112, 107, 103, 100, 97, 94, 92, 89, 87, 85, 83, 82, 80, 78, 77, 75, 75, 73, 72, 71, 70, 69, 68, 68, 67, 67, 66, 66, 66, 66, 65, 65, 66, 66, 65, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 71, 72, 73, 75, 76, 77, 78, 80, 82, 84, 86, 88, 91, 94, 97, 99, 
			111, 107, 103, 100, 97, 93, 91, 89, 87, 85, 83, 81, 80, 78, 76, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78, 79, 81, 83, 85, 87, 90, 93, 96, 99, 
			110, 107, 103, 99, 96, 93, 91, 89, 87, 85, 83, 81, 79, 77, 76, 75, 73, 72, 72, 70, 69, 68, 68, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 81, 83, 85, 87, 90, 93, 96, 98, 
			110, 106, 102, 98, 95, 93, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 70, 71, 73, 74, 75, 76, 78, 79, 81, 82, 84, 86, 89, 92, 95, 98, 
			109, 106, 101, 97, 94, 92, 90, 88, 86, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 80, 82, 84, 86, 89, 92, 94, 97, 
			108, 105, 101, 97, 94, 92, 89, 87, 86, 83, 81, 79, 78, 77, 75, 73, 72, 71, 70, 70, 68, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 80, 82, 84, 86, 88, 91, 94, 97, 
			108, 104, 100, 97, 94, 92, 89, 87, 85, 83, 81, 79, 78, 76, 75, 74, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 76, 77, 78, 80, 81, 83, 85, 88, 90, 93, 96, 
			107, 103, 100, 97, 94, 91, 89, 87, 85, 83, 81, 79, 77, 76, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 75, 77, 78, 79, 81, 83, 85, 87, 90, 93, 95, 
			106, 103, 100, 97, 93, 91, 89, 86, 84, 82, 80, 79, 77, 76, 75, 73, 72, 71, 70, 68, 68, 67, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 79, 81, 83, 85, 87, 90, 93, 95, 
			106, 103, 99, 96, 93, 91, 89, 86, 84, 82, 80, 79, 77, 75, 74, 73, 72, 70, 70, 68, 68, 67, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 89, 92, 95, 
			106, 102, 99, 96, 93, 91, 88, 86, 84, 82, 80, 78, 77, 75, 74, 73, 72, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 67, 68, 69, 71, 72, 73, 74, 76, 77, 79, 81, 82, 84, 87, 89, 91, 94, 
			105, 102, 99, 95, 93, 91, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 71, 72, 73, 74, 75, 77, 79, 81, 82, 84, 87, 89, 92, 94, 
			105, 102, 98, 95, 93, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 72, 74, 74, 75, 77, 78, 80, 82, 84, 87, 89, 92, 94, 
			106, 103, 99, 95, 92, 90, 88, 86, 83, 81, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 72, 74, 74, 75, 76, 78, 80, 82, 84, 86, 89, 92, 94, 
			105, 104, 99, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 72, 73, 74, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 
			106, 105, 99, 95, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 
			106, 105, 100, 96, 93, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 93, 
			106, 104, 100, 98, 94, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 
			106, 102, 99, 99, 95, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 
			105, 101, 98, 98, 95, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 91, 93, 
			105, 102, 98, 96, 93, 90, 88, 86, 84, 82, 80, 78, 77, 75, 74, 72, 71, 70, 69, 68, 68, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 92, 94, 
			105, 102, 99, 96, 93, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 71, 70, 69, 68, 68, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 92, 94, 
			105, 102, 99, 98, 94, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 72, 70, 69, 68, 68, 67, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 78, 80, 82, 84, 86, 89, 92, 94, 
			106, 102, 99, 98, 94, 90, 88, 86, 84, 83, 80, 78, 77, 75, 74, 73, 72, 70, 69, 68, 68, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 76, 78, 80, 82, 84, 86, 89, 92, 94, 
			106, 102, 99, 97, 93, 91, 88, 86, 84, 83, 81, 79, 77, 75, 74, 73, 72, 71, 70, 68, 68, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 75, 77, 78, 80, 82, 84, 87, 89, 92, 94, 
			106, 103, 99, 96, 93, 91, 88, 86, 84, 83, 81, 79, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 75, 77, 78, 80, 82, 84, 87, 90, 92, 95, 
			106, 103, 99, 96, 93, 91, 89, 86, 84, 83, 81, 79, 78, 76, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 76, 77, 79, 81, 82, 84, 87, 90, 92, 95, 
			106, 103, 99, 96, 94, 91, 89, 86, 84, 83, 81, 79, 78, 76, 75, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 90, 93, 95, 
			107, 104, 100, 97, 94, 92, 90, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 72, 70, 70, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 68, 68, 69, 71, 72, 73, 74, 76, 77, 79, 81, 83, 85, 87, 90, 93, 95, 
			107, 104, 101, 97, 94, 92, 90, 88, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 70, 70, 69, 68, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 79, 81, 83, 86, 88, 90, 93, 96, 
			107, 104, 101, 98, 94, 92, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 72, 70, 70, 69, 68, 67, 67, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 67, 67, 68, 69, 70, 71, 72, 74, 75, 76, 78, 79, 82, 84, 86, 88, 91, 94, 97, 
			108, 105, 101, 98, 95, 93, 90, 88, 86, 84, 82, 80, 78, 77, 76, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 68, 69, 70, 71, 73, 74, 75, 76, 78, 80, 82, 84, 86, 88, 91, 94, 97, 
			109, 106, 102, 98, 96, 93, 90, 88, 86, 84, 83, 81, 79, 77, 76, 75, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 68, 69, 69, 70, 72, 73, 74, 75, 76, 78, 80, 82, 84, 86, 89, 91, 94, 97, 
			110, 106, 102, 99, 96, 93, 91, 89, 87, 85, 83, 81, 79, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 79, 80, 82, 84, 86, 89, 92, 95, 97, 
			110, 107, 103, 99, 96, 94, 92, 89, 87, 85, 83, 81, 80, 78, 77, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 73, 75, 76, 77, 79, 80, 83, 85, 87, 89, 92, 95, 98, 
			110, 107, 103, 100, 97, 94, 92, 89, 87, 85, 83, 82, 80, 78, 77, 76, 75, 73, 72, 72, 70, 70, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 65, 65, 65, 66, 66, 65, 66, 66, 66, 66, 67, 67, 68, 69, 69, 70, 71, 73, 74, 75, 76, 77, 79, 81, 83, 85, 88, 90, 93, 96, 98, 
			111, 108, 104, 100, 97, 95, 92, 90, 88, 86, 84, 82, 80, 79, 78, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 68, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 79, 82, 84, 86, 88, 90, 93, 96, 99, 
			112, 108, 104, 101, 98, 95, 93, 90, 88, 86, 84, 83, 81, 79, 78, 77, 75, 74, 73, 72, 71, 70, 70, 69, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 68, 69, 70, 70, 71, 72, 73, 74, 77, 79, 79, 80, 82, 84, 86, 88, 90, 93, 97, 99, 
			113, 109, 105, 102, 98, 96, 93, 91, 89, 87, 85, 83, 81, 80, 78, 77, 76, 75, 73, 72, 72, 70, 70, 69, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 70, 71, 72, 73, 74, 75, 79, 81, 80, 81, 82, 84, 86, 89, 91, 94, 97, 100, 
			114, 109, 105, 102, 99, 96, 93, 91, 89, 87, 85, 83, 82, 80, 79, 77, 76, 75, 74, 73, 72, 71, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 70, 71, 71, 72, 73, 74, 75, 79, 81, 80, 81, 83, 85, 87, 89, 92, 95, 98, 100, 
			114, 110, 106, 103, 99, 97, 94, 92, 90, 88, 86, 84, 82, 81, 79, 78, 77, 76, 74, 73, 72, 72, 71, 70, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 80, 81, 83, 85, 87, 90, 92, 95, 98, 101, 
			115, 112, 108, 104, 100, 97, 95, 92, 90, 88, 87, 85, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 74, 75, 76, 78, 79, 80, 82, 84, 86, 88, 90, 93, 96, 99, 102, 
			116, 112, 108, 104, 101, 98, 96, 93, 90, 88, 87, 85, 83, 81, 80, 79, 77, 76, 75, 74, 73, 72, 72, 71, 70, 69, 69, 68, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 80, 81, 82, 84, 86, 89, 91, 94, 97, 100, 103, 
			117, 113, 109, 105, 102, 99, 96, 93, 91, 89, 87, 85, 84, 82, 80, 79, 78, 77, 76, 74, 73, 73, 72, 71, 71, 70, 70, 69, 68, 68, 68, 68, 68, 68, 68, 67, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 73, 74, 75, 76, 78, 79, 81, 82, 83, 85, 87, 89, 91, 94, 97, 100, 104, 
			118, 114, 110, 106, 103, 100, 97, 94, 92, 90, 88, 86, 84, 83, 81, 80, 78, 77, 76, 75, 74, 73, 72, 72, 71, 70, 70, 70, 69, 69, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 70, 70, 71, 72, 73, 73, 74, 75, 76, 77, 79, 80, 81, 82, 84, 86, 88, 90, 92, 95, 98, 101, 105, 
			120, 115, 111, 107, 103, 100, 98, 95, 93, 91, 89, 87, 85, 83, 82, 80, 79, 78, 77, 75, 74, 74, 73, 72, 72, 71, 70, 70, 69, 69, 69, 69, 69, 68, 68, 68, 69, 69, 69, 69, 70, 70, 70, 71, 73, 74, 74, 75, 76, 77, 78, 79, 81, 82, 83, 85, 87, 89, 91, 93, 96, 98, 102, 105, 
			121, 117, 112, 108, 104, 101, 99, 96, 94, 92, 90, 88, 86, 84, 82, 81, 80, 78, 77, 76, 75, 74, 73, 73, 72, 72, 71, 70, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 71, 71, 72, 73, 74, 74, 75, 76, 77, 78, 80, 81, 83, 84, 86, 87, 89, 91, 94, 96, 99, 103, 106, 
			123, 118, 113, 109, 105, 102, 99, 97, 94, 92, 90, 88, 87, 85, 83, 81, 80, 79, 78, 77, 76, 75, 74, 73, 73, 72, 72, 71, 71, 70, 70, 70, 70, 69, 69, 69, 70, 70, 70, 70, 71, 71, 72, 73, 73, 74, 75, 76, 77, 78, 79, 81, 82, 84, 85, 86, 88, 90, 92, 95, 97, 100, 103, 107, 
			124, 120, 115, 111, 107, 103, 100, 98, 95, 93, 91, 89, 87, 86, 84, 82, 81, 80, 78, 77, 76, 75, 74, 74, 73, 72, 72, 72, 71, 71, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 72, 72, 73, 73, 74, 75, 76, 77, 78, 79, 80, 81, 83, 85, 86, 87, 89, 91, 94, 96, 98, 101, 104, 108, 
			126, 121, 116, 112, 108, 105, 101, 99, 96, 94, 92, 90, 88, 86, 84, 83, 81, 80, 79, 78, 77, 76, 75, 75, 74, 73, 73, 72, 72, 71, 71, 71, 71, 71, 71, 71, 71, 71, 72, 72, 73, 73, 74, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 87, 88, 90, 92, 94, 96, 99, 102, 106, 109, 
			128, 123, 117, 113, 109, 106, 103, 101, 98, 95, 93, 91, 89, 87, 85, 84, 82, 81, 80, 78, 78, 77, 76, 75, 75, 74, 73, 73, 72, 72, 72, 72, 72, 72, 71, 72, 72, 72, 72, 73, 73, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 86, 88, 90, 92, 93, 95, 98, 100, 103, 107, 110, 
			131, 125, 119, 114, 110, 107, 104, 101, 99, 96, 94, 92, 89, 87, 86, 84, 83, 81, 81, 79, 78, 77, 76, 75, 75, 74, 74, 73, 73, 73, 72, 72, 72, 72, 72, 72, 72, 72, 73, 73, 74, 74, 75, 76, 76, 77, 78, 79, 80, 81, 83, 84, 85, 87, 89, 90, 92, 94, 97, 99, 101, 104, 108, 112, 
       },
    },
    4096
};

static HI_U32 cmos_get_isp_default(ISP_DEV IspDev,ISP_CMOS_DEFAULT_S *pstDef)
{
    if (HI_NULL == pstDef)
    {
        printf("null pointer when get isp default value!\n");
        return -1;
    }
    
    memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));
    
    switch (g_apstSnsState[IspDev]->enWDRMode)
    {
        default:
        case WDR_MODE_NONE:
            pstDef->stDrc.bEnable               = HI_FALSE;
            pstDef->stDrc.u32BlackLevel         = 0x00;
            pstDef->stDrc.u32WhiteLevel         = 0xB0000; 
            pstDef->stDrc.u32SlopeMax           = 0x60;
            pstDef->stDrc.u32SlopeMin           = 0xD0;
            pstDef->stDrc.u32VarianceSpace      = 0x06;
            pstDef->stDrc.u32VarianceIntensity  = 0x08;
            pstDef->stDrc.u32Asymmetry          = 0x06;
            pstDef->stDrc.u32BrightEnhance      = 0xC8;
            pstDef->stDrc.bFilterMux            = 0x0;
            pstDef->stDrc.u32Svariance          = 0xC;
            pstDef->stDrc.u32BrightPr           = 0x94;
            pstDef->stDrc.u32Contrast           = 0x80;
            pstDef->stDrc.u32DarkEnhance        = 0x6000;
            
            memcpy(&pstDef->stAgcTbl, &g_stIspAgcTable, sizeof(ISP_CMOS_AGC_TABLE_S));
            memcpy(&pstDef->stNoiseTbl, &g_stIspNoiseTable, sizeof(ISP_CMOS_NOISE_TABLE_S));
            memcpy(&pstDef->stDemosaic, &g_stIspDemosaic, sizeof(ISP_CMOS_DEMOSAIC_S));
            memcpy(&pstDef->stBayerSharpen, &g_stIspBayerSharpen, sizeof(ISP_CMOS_BAYER_SHARPEN_S));
            memcpy(&pstDef->stYuvSharpen, &g_stIspYuvSharpen, sizeof(ISP_CMOS_YUV_SHARPEN_S));
            memcpy(&pstDef->stGamma, &g_stIspGamma, sizeof(ISP_CMOS_GAMMA_S));
			memcpy(&pstDef->stMeshShading, &g_stIspMeshShading_4M, sizeof(ISP_CMOS_MESH_SHADING_S));
            break;
            
        case WDR_MODE_2To1_LINE:
            pstDef->stDrc.bEnable               = HI_TRUE;
            pstDef->stDrc.u32BlackLevel         = 0x00;
            pstDef->stDrc.u32WhiteLevel         = 0xB0000; 
            pstDef->stDrc.u32SlopeMax           = 0x30;
            pstDef->stDrc.u32SlopeMin           = 0xD0;
            pstDef->stDrc.u32VarianceSpace      = 0x06;
            pstDef->stDrc.u32VarianceIntensity  = 0x08;
            pstDef->stDrc.u32Asymmetry          = 0x1C;
            pstDef->stDrc.u32BrightEnhance      = 0xF0;
            pstDef->stDrc.bFilterMux            = 0x0;
            pstDef->stDrc.u32Svariance          = 0xC;
            pstDef->stDrc.u32BrightPr           = 0x94;
            pstDef->stDrc.u32Contrast           = 0x80;
            pstDef->stDrc.u32DarkEnhance        = 0x6000;
            
            pstDef->stWDRAttr.au32ExpRatio[0]   = 0x40;
            pstDef->stWDRAttr.au32ExpRatio[1]   = 0x40;
            pstDef->stWDRAttr.au32ExpRatio[2]   = 0x40;
            
            memcpy(&pstDef->stAgcTbl, &g_stIspAgcTableFSWDR, sizeof(ISP_CMOS_AGC_TABLE_S));
            memcpy(&pstDef->stNoiseTbl, &g_stIspNoiseTableFSWDR, sizeof(ISP_CMOS_NOISE_TABLE_S));
            memcpy(&pstDef->stDemosaic, &g_stIspDemosaicFSWDR, sizeof(ISP_CMOS_DEMOSAIC_S));
            memcpy(&pstDef->stBayerSharpen, &g_stIspBayerSharpenFSWDR, sizeof(ISP_CMOS_BAYER_SHARPEN_S));
            memcpy(&pstDef->stYuvSharpen, &g_stIspYuvSharpenFSWDR, sizeof(ISP_CMOS_YUV_SHARPEN_S));
            memcpy(&pstDef->stGamma, &g_stIspGammaFSWDR, sizeof(ISP_CMOS_GAMMA_S));
			memcpy(&pstDef->stMeshShading, &g_stIspMeshShading_4M, sizeof(ISP_CMOS_MESH_SHADING_S));
            break;
    }
    
    pstDef->stSensorMaxResolution.u32MaxWidth  = 2592;
    pstDef->stSensorMaxResolution.u32MaxHeight = 1944;
    pstDef->stSensorMode.u32SensorID = OS05A_ID;
    pstDef->stSensorMode.u8SensorMode = g_apstSnsState[IspDev]->u8ImgMode;
    
    return 0;
}

static HI_U32 cmos_get_isp_black_level(ISP_DEV IspDev,ISP_CMOS_BLACK_LEVEL_S *pstBlackLevel)
{
    HI_S32  i;
    
    if (HI_NULL == pstBlackLevel)
    {
        printf("null pointer when get isp black level value!\n");
        return -1;
    }
    
    /* Don't need to update black level when iso change */
    pstBlackLevel->bUpdate = HI_FALSE;
    if(WDR_MODE_NONE == g_apstSnsState[IspDev]->enWDRMode)
    {
        for (i=0; i<4; i++)
        {
            pstBlackLevel->au16BlackLevel[i] = 64; /*12bit,0x40*/
        }
    }
    else if(WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
    {
        for (i=0; i<4; i++)
        {
            pstBlackLevel->au16BlackLevel[i] = 256; /*10bit,0x10s0*/
        }
    }
    
    return 0;
}

static HI_VOID cmos_set_pixel_detect(ISP_DEV IspDev,HI_BOOL bEnable)
{
    HI_U32 u32FullLines_5Fps, u32MaxIntTime_5Fps;
    
    if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
    {
        return;
    }
    else
    {
        if (Os05a_5M_2688x1944_12bit_linear30 == g_apstSnsState[IspDev]->u8ImgMode)
        {
            u32FullLines_5Fps = (OS05A_VMAX_5M30_12bit_LINEAE * 30) / 5;
        }
        else
        {
            return;
        }
    }
    
    u32MaxIntTime_5Fps = u32FullLines_5Fps - 4;
    
    if (bEnable) /* setup for ISP pixel calibration mode */
    {
        os05a_write_register(IspDev, VMAX_ADDR_L, LOW_8BITS(u32FullLines_5Fps)); 
        os05a_write_register(IspDev, VMAX_ADDR_H, HIGH_8BITS(u32FullLines_5Fps));
        
        os05a_write_register(IspDev, EXPOSURE_TIME_LONG_H, HIGH_8BITS(u32MaxIntTime_5Fps));    /* shutter */
        os05a_write_register(IspDev, EXPOSURE_TIME_LONG_L, LOW_8BITS(u32MaxIntTime_5Fps));
        
        os05a_write_register(IspDev, AGAIN_H, 0x00); //gain
        os05a_write_register(IspDev, AGAIN_L, 0x90);
    }
    else /* setup for ISP 'normal mode' */
    {
        os05a_write_register (IspDev, VMAX_ADDR_L, LOW_8BITS(g_apstSnsState[IspDev]->u32FLStd)); 
        os05a_write_register (IspDev, VMAX_ADDR_H, HIGH_8BITS(g_apstSnsState[IspDev]->u32FLStd));
        g_apstSnsState[IspDev]->bSyncInit = HI_FALSE ;
    }
    return;
}

static HI_VOID cmos_set_wdr_mode(ISP_DEV IspDev,HI_U8 u8Mode)
{
    g_apstSnsState[IspDev]->bSyncInit = HI_FALSE;
    
    switch(u8Mode)
    {
        case WDR_MODE_NONE:
            g_apstSnsState[IspDev]->enWDRMode = WDR_MODE_NONE;
            g_apstSnsState[IspDev]->u32FLStd = OS05A_VMAX_5M30_12bit_LINEAE;
            g_apstSnsState[IspDev]->u8ImgMode = Os05a_5M_2688x1944_12bit_linear30;
            printf("linear mode\n");
        break;
        
        case WDR_MODE_2To1_LINE:
            g_apstSnsState[IspDev]->enWDRMode = WDR_MODE_2To1_LINE;
            g_apstSnsState[IspDev]->u32FLStd = OS05A_VMAX_5M30_10bit_WDR;
            g_apstSnsState[IspDev]->u8ImgMode = Os05a_5M_2688x1944_10bit_wdr30;
            printf("2to1 line WDR mode\n");
        break;
        
        default:
            printf("NOT support this mode!\n");
            return;
        break;
    }
    
    memset(g_apstSnsState[IspDev]->au32WDRIntTime, 0, sizeof(g_apstSnsState[IspDev]->au32WDRIntTime));
    
    return;
}

static HI_S32 cmos_set_image_mode(ISP_DEV IspDev,ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
    HI_U8 u8SensorImageMode = g_apstSnsState[IspDev]->u8ImgMode;
    g_apstSnsState[IspDev]->bSyncInit = HI_FALSE;
    
    if (HI_NULL == pstSensorImageMode )
    {
        printf("null pointer when set image mode\n");
        return -1;
    }
    
    if((pstSensorImageMode->u16Width <= 2688)&&(pstSensorImageMode->u16Height <= 1944))
    {
        if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
        {
            if (pstSensorImageMode->f32Fps <= 30)
            {
                u8SensorImageMode = Os05a_5M_2688x1944_10bit_wdr30;
            }
            else
            {
                printf("Not support! Width:%d, Height:%d, Fps:%f\n",
                    pstSensorImageMode->u16Width,
                    pstSensorImageMode->u16Height,
                    pstSensorImageMode->f32Fps);
                
                return -1;
            }
        }
        else if(WDR_MODE_NONE == g_apstSnsState[IspDev]->enWDRMode)
        {
            if (pstSensorImageMode->f32Fps <= 30)
            {
                u8SensorImageMode = Os05a_5M_2688x1944_12bit_linear30;
            }
            else
            {
                printf("Not support! Width:%d, Height:%d, Fps:%f\n",
                    pstSensorImageMode->u16Width,
                    pstSensorImageMode->u16Height,
                    pstSensorImageMode->f32Fps);
                
                return -1;
            }
        }
    }
    else
    {
        printf("Not support! Width:%d, Height:%d, Fps:%f\n",
            pstSensorImageMode->u16Width,
            pstSensorImageMode->u16Height,
            pstSensorImageMode->f32Fps);
        return -1;
    }
    
    /* Sensor first init */
    if (HI_FALSE == g_apstSnsState[IspDev]->bInit)
    {
        g_apstSnsState[IspDev]->u8ImgMode = u8SensorImageMode;
        return 0;
    }
    
    /* Switch SensorImageMode */
    if (u8SensorImageMode == g_apstSnsState[IspDev]->u8ImgMode)
    {
        /* Don't need to switch SensorImageMode */
        return -1;
    }
    
    g_apstSnsState[IspDev]->u8ImgMode = u8SensorImageMode;
    memset(g_apstSnsState[IspDev]->au32WDRIntTime, 0, sizeof(g_apstSnsState[IspDev]->au32WDRIntTime));
    
    return 0;
}

HI_U32 cmos_get_sns_regs_info(ISP_DEV IspDev,ISP_SNS_REGS_INFO_S *pstSnsRegsInfo)
{
    HI_S32 i;
    
    if (HI_NULL == pstSnsRegsInfo)
    {
        printf("null pointer when get sns reg info!\n");
        return -1;
    }
    
    if ((HI_FALSE == g_apstSnsState[IspDev]->bSyncInit) || (HI_FALSE == pstSnsRegsInfo->bConfig))
    {
        g_apstSnsState[IspDev]->astRegsInfo[0].enSnsType = ISP_SNS_I2C_TYPE;
        g_apstSnsState[IspDev]->astRegsInfo[0].unComBus.s8I2cDev = g_aunOs05aBusInfo[IspDev].s8I2cDev;
        g_apstSnsState[IspDev]->astRegsInfo[0].u8Cfg2ValidDelayMax = 2;
        g_apstSnsState[IspDev]->astRegsInfo[0].u32RegNum = 8;
        
        if ((WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode) || (WDR_MODE_2To1_FRAME == g_apstSnsState[IspDev]->enWDRMode) || (WDR_MODE_2To1_FRAME_FULL_RATE == g_apstSnsState[IspDev]->enWDRMode))
        {
            g_apstSnsState[IspDev]->astRegsInfo[0].u32RegNum += 6; 
        }
        
        for (i=0; i<g_apstSnsState[IspDev]->astRegsInfo[0].u32RegNum; i++)
        {
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].bUpdate = HI_TRUE;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].u8DevAddr = os05a_i2c_addr;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].u32AddrByteNum = os05a_addr_byte;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].u32DataByteNum = os05a_data_byte;
        }
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u8DelayFrmNum = 0; 
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u32RegAddr = VMAX_ADDR_L;        // Vmax
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u32RegAddr = VMAX_ADDR_H;        // Vmax
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[2].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[2].u32RegAddr = EXPOSURE_TIME_LONG_L;        // Long shutter
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[3].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[3].u32RegAddr = EXPOSURE_TIME_LONG_H;        // Long shutter
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[4].u8DelayFrmNum = 0; 
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[4].u32RegAddr = AGAIN_L;        // Again
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[5].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[5].u32RegAddr = AGAIN_H;        // Again
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[6].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[6].u32RegAddr = DGAIN_L;     // Dgain
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[7].u8DelayFrmNum = 0;
        g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[7].u32RegAddr = DGAIN_H;     // Dgain

        if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode)
        {
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[8].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[8].u32RegAddr = EXPOSURE_TIME_SHORT_L;     // Short shutter
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[9].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[9].u32RegAddr = EXPOSURE_TIME_SHORT_H;     // Short shutter
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[10].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[10].u32RegAddr = SHORT_AGAIN_L;     // Short Again
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[11].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[11].u32RegAddr = SHORT_AGAIN_H;     // Short Again
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[12].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[12].u32RegAddr = SHORT_DGAIN_L;     // Short Dgain
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[13].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[13].u32RegAddr = SHORT_DGAIN_H;     // Short Dgain
        }
        g_apstSnsState[IspDev]->bSyncInit = HI_TRUE;
    }
    else
    {
        for (i=0; i<g_apstSnsState[IspDev]->astRegsInfo[0].u32RegNum; i++)
        {
            if (g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].u32Data == g_apstSnsState[IspDev]->astRegsInfo[1].astI2cData[i].u32Data)
            {
                g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].bUpdate = HI_FALSE;
            }
            else
            {
                g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[i].bUpdate = HI_TRUE;
            }
        }
    }

    if (WDR_MODE_2To1_LINE == g_apstSnsState[IspDev]->enWDRMode )
    {
        if(g_apstSnsState[IspDev]->au32FL[1] > g_apstSnsState[IspDev]->au32FL[0])
        {
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u8DelayFrmNum = 2;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u8DelayFrmNum = 2;
        }
        else if(g_apstSnsState[IspDev]->au32FL[1] < g_apstSnsState[IspDev]->au32FL[0])
        {
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[0].u8DelayFrmNum = 0;
            g_apstSnsState[IspDev]->astRegsInfo[0].astI2cData[1].u8DelayFrmNum = 0;
        }
        else
        {
            //do nothing
        }
    }
    
    pstSnsRegsInfo->bConfig = HI_FALSE;
    memcpy(pstSnsRegsInfo, &g_apstSnsState[IspDev]->astRegsInfo[0], sizeof(ISP_SNS_REGS_INFO_S));
    memcpy(&g_apstSnsState[IspDev]->astRegsInfo[1], &g_apstSnsState[IspDev]->astRegsInfo[0], sizeof(ISP_SNS_REGS_INFO_S));
    g_apstSnsState[IspDev]->au32FL[1] = g_apstSnsState[IspDev]->au32FL[0];
    return 0;
}

static HI_VOID sensor_global_init(ISP_DEV IspDev)
{
    g_apstSnsState[IspDev]->bInit = HI_FALSE; 
    g_apstSnsState[IspDev]->bSyncInit = HI_FALSE;
    g_apstSnsState[IspDev]->u8ImgMode = Os05a_5M_2688x1944_12bit_linear30;
    g_apstSnsState[IspDev]->enWDRMode = WDR_MODE_NONE;
    g_apstSnsState[IspDev]->u32FLStd = OS05A_VMAX_5M30_12bit_LINEAE;
    g_apstSnsState[IspDev]->au32FL[0] = OS05A_VMAX_5M30_12bit_LINEAE;
    g_apstSnsState[IspDev]->au32FL[1] = OS05A_VMAX_5M30_12bit_LINEAE;
    
    memset(&g_apstSnsState[IspDev]->astRegsInfo[0], 0, sizeof(ISP_SNS_REGS_INFO_S));
    memset(&g_apstSnsState[IspDev]->astRegsInfo[1], 0, sizeof(ISP_SNS_REGS_INFO_S));
}

HI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
    memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));
    
    pstSensorExpFunc->pfn_cmos_sensor_init = os05a_init;
    pstSensorExpFunc->pfn_cmos_sensor_exit = os05a_exit;
    pstSensorExpFunc->pfn_cmos_sensor_global_init = sensor_global_init;
    pstSensorExpFunc->pfn_cmos_set_image_mode = cmos_set_image_mode;
    pstSensorExpFunc->pfn_cmos_set_wdr_mode = cmos_set_wdr_mode;
    
    pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
    pstSensorExpFunc->pfn_cmos_get_isp_black_level = cmos_get_isp_black_level;
    pstSensorExpFunc->pfn_cmos_set_pixel_detect = cmos_set_pixel_detect;
    pstSensorExpFunc->pfn_cmos_get_sns_reg_info = cmos_get_sns_regs_info;
    
    return 0;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/
static int os05a_set_bus_info(ISP_DEV IspDev, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
    g_aunOs05aBusInfo[IspDev].s8I2cDev = unSNSBusInfo.s8I2cDev;
    
    return 0;
}

static int sensor_register_callback(ISP_DEV IspDev, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
    HI_S32 s32Ret;
    ISP_SENSOR_REGISTER_S stIspRegister;
    AE_SENSOR_REGISTER_S  stAeRegister;
    AWB_SENSOR_REGISTER_S stAwbRegister;
    
    cmos_init_sensor_exp_function(&stIspRegister.stSnsExp);
    s32Ret = HI_MPI_ISP_SensorRegCallBack(IspDev, OS05A_ID, &stIspRegister);
    if (s32Ret)
    {
        printf("sensor register callback function failed!\n");
        return s32Ret;
    }
    
    cmos_init_ae_exp_function(&stAeRegister.stSnsExp);
    s32Ret = HI_MPI_AE_SensorRegCallBack(IspDev, pstAeLib, OS05A_ID, &stAeRegister);
    if (s32Ret)
    {
        printf("sensor register callback function to ae lib failed!\n");
        return s32Ret;
    }
    
    cmos_init_awb_exp_function(&stAwbRegister.stSnsExp);
    s32Ret = HI_MPI_AWB_SensorRegCallBack(IspDev, pstAwbLib, OS05A_ID, &stAwbRegister);
    if (s32Ret)
    {
        printf("sensor register callback function to awb lib failed!\n");
        return s32Ret;
    }
    
    return 0;
}

static int sensor_unregister_callback(ISP_DEV IspDev, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
    HI_S32 s32Ret;
    
    s32Ret = HI_MPI_ISP_SensorUnRegCallBack(IspDev, OS05A_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function failed!\n");
        return s32Ret;
    }
    
    s32Ret = HI_MPI_AE_SensorUnRegCallBack(IspDev, pstAeLib, OS05A_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function to ae lib failed!\n");
        return s32Ret;
    }
    
    s32Ret = HI_MPI_AWB_SensorUnRegCallBack(IspDev, pstAwbLib, OS05A_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function to awb lib failed!\n");
        return s32Ret;
    }
    
    return 0;
}

static int sensor_set_init(ISP_DEV IspDev, ISP_INIT_ATTR_S *pstInitAttr)
{
    g_au32InitExposure[IspDev] = pstInitAttr->u32Exposure;
    g_au32LinesPer500ms[IspDev] = pstInitAttr->u32LinesPer500ms;
    g_au16InitWBGain[IspDev][0] = pstInitAttr->u16WBRgain;
    g_au16InitWBGain[IspDev][1] = pstInitAttr->u16WBGgain;
    g_au16InitWBGain[IspDev][2] = pstInitAttr->u16WBBgain;
    g_au16SampleRgain[IspDev] = pstInitAttr->u16SampleRgain;
    g_au16SampleBgain[IspDev] = pstInitAttr->u16SampleBgain;
    
    return 0;
}

ISP_SNS_OBJ_S stSnsOs05aObj =
{
    .pfnRegisterCallback    = sensor_register_callback,
    .pfnUnRegisterCallback  = sensor_unregister_callback,
    .pfnStandby             = os05a_standby,
    .pfnRestart             = os05a_restart,
    .pfnWriteReg            = os05a_write_register,
    .pfnReadReg             = os05a_read_register,
    .pfnSetBusInfo          = os05a_set_bus_info,
    .pfnSetInit             = sensor_set_init
};

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
