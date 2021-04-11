/******************************************************************************

  Copyright (C), 2001-2013, Hisilicon Tech. Co., Ltd.

 ******************************************************************************
  File Name     : imx274_sensor_ctl.c
  Version       : Initial Draft
  Author        : Hisilicon BVT ISP group
  Created       : 2014/05/22
  Description   : Sony IMX274 sensor driver
  History       :
  1.Date        : 2014/05/22
  Author        : 
  Modification  : Created file

******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"
#include "mpi_sys.h"

#include "hi_i2c.h"
#include "imx274_slave_priv.h"
#include "mpi_isp.h"

extern const IMX274_VIDEO_MODE_TBL_S g_astImx274ModeTbl[];
extern ISP_SLAVE_SNS_SYNC_S gstImx274Sync[];

static int g_fd[ISP_MAX_DEV_NUM] = {-1,-1};

extern ISP_SNS_STATE_S             g_astImx274[ISP_MAX_DEV_NUM];
extern ISP_SNS_COMMBUS_U     g_aunImx274BusInfo[];

const unsigned char imx274_i2c_addr     =    0x34;        /* I2C Address of IMX274 */
const unsigned int  imx274_addr_byte    =    2;
const unsigned int  imx274_data_byte    =    1;

static const HI_U16 gs_au16SensorCfgSeq[][IMX274_MODE_BUTT+1] = {
    /*  M0     M0_dol     ADDR   */         
    { 0x12,  0x12,  0x3000 },   
    {   -1,    -1,  0xFFFF },
    { 0xf0,  0xf0,  0x3120 }, 
    { 0x00,  0x00,  0x3121 }, 
    { 0x02,  0x02,  0x3122 }, 
    //{ 0x01,  0x03,  0x3123 }, 
    { 0x9c,  0x9c,  0x3129 }, 
    { 0x02,  0x02,  0x312a }, 
    { 0x02,  0x02,  0x312d }, 
    //{ 0x01,  0x01,  0x3ac4 }, 
    { 0x00,  0x00,  0x310b }, 
    
    { 0x00,  0x00,  0x304c }, 
    { 0x03,  0x03,  0x304d }, 
    { 0x1a,  0x1a,  0x331c }, 
    { 0x00,  0x00,  0x331d }, 
    { 0x02,  0x02,  0x3502 }, 
    { 0x0e,  0x0e,  0x3529 }, 
    { 0x0e,  0x0e,  0x352a }, 
    { 0x0e,  0x0e,  0x352b }, 
    { 0x0e,  0x0e,  0x3538 }, 
    { 0x0e,  0x0e,  0x3539 }, 
    { 0x00,  0x00,  0x3553 }, 
    { 0x05,  0x05,  0x357d }, 
    { 0x05,  0x05,  0x357f }, 
    { 0x04,  0x04,  0x3581 }, 
    { 0x76,  0x76,  0x3583 }, 
    { 0x01,  0x01,  0x3587 }, 
    { 0x0e,  0x0e,  0x35bb }, 
    { 0x0e,  0x0e,  0x35bc }, 
    { 0x0e,  0x0e,  0x35bd }, 
    { 0x0e,  0x0e,  0x35be }, 
    { 0x0e,  0x0e,  0x35bf }, 
    { 0x00,  0x00,  0x366e }, 
    { 0x00,  0x00,  0x366f }, 
    { 0x00,  0x00,  0x3670 }, 
    { 0x00,  0x00,  0x3671 }, 
    
    { 0x32,  0x32,  0x3304 }, 
    { 0x00,  0x00,  0x3305 }, 
    { 0x32,  0x32,  0x3306 }, 
    { 0x00,  0x00,  0x3307 }, 
    { 0x32,  0x32,  0x3590 }, 
    { 0x00,  0x00,  0x3591 }, 
    { 0x32,  0x32,  0x3686 }, 
    { 0x00,  0x00,  0x3687 }, 

    { 0x00,  0x05,  0x3004 }, 
    { 0x07,  0x01,  0x3005 }, 
    { 0x00,  0x00,  0x3006 }, 
    { 0x02,  0x02,  0x3007 }, 
    { 0x10,  0x08,  0x3a41 }, 

    { 0x00,  0x00,  0x300e },
    { 0x00,  0x00,  0x300f },
    { 0x10,  0x31,  0x3019 },
    { 0x00,  0x00,  0x301a },
    { 0x08,  0x40,  0x3032 },
    { 0x00,  0x00,  0x3033 },
    { 0x00,  0x00,  0x3037 },
    { 0x00,  0x00,  0x3038 },
    { 0x00,  0x00,  0x3039 },
    { 0x00,  0x00,  0x303a },
    { 0x00,  0x00,  0x303b },
    { 0x30,  0x31,  0x3041 },
    { 0x08,  0x07,  0x3042 },
    { 0x01,  0x01,  0x3043 },
      
    { 0x07,  0x05,  0x306b },
    { 0x00,  0x00,  0x30dd },
    { 0x00,  0x00,  0x30de },
    { 0x00,  0x00,  0x30df },
    { 0x00,  0x00,  0x30e0 },
    { 0x00,  0x00,  0x30e1 },
    { 0x00,  0x01,  0x30e2 }, 
    { 0x00,  0x01,  0x30e9 },
    { 0x01,  0x01,  0x30ee }, 
    
    { 0x08,  0x20,  0x30f6 }, 
    { 0x02,  0x04,  0x30f7 }, //524
    { 0x0c,  0xe3,  0x30f8 }, 
    { 0x12,  0x08,  0x30f9 }, 
    { 0x00,  0x00,  0x30fa }, //4620
    
    { 0xaa,  0x86,  0x3130 }, 
    { 0x08,  0x08,  0x3131 }, 
    { 0x9a,  0x7e,  0x3132 }, 
    { 0x08,  0x08,  0x3133 }, 
 
    { 0xff,  0x0a,  0x3342 }, 
    { 0x01,  0x00,  0x3343 }, 
    { 0xff,  0x16,  0x3344 }, 
    { 0x01,  0x00,  0x3345 },
    { 0x01,  0x01,  0x33a6 }, 
    { 0x0f,  0x0e,  0x3528 }, 
    { 0x00,  0x1f,  0x3554 }, 
    { 0x00,  0x01,  0x3555 }, 
    { 0x00,  0x01,  0x3556 }, 
    { 0x00,  0x01,  0x3557 }, 
    { 0x00,  0x01,  0x3558 }, 
    { 0x1f,  0x00,  0x3559 }, 
    { 0x1f,  0x00,  0x355a }, 
    { 0x0f,  0x0e,  0x35ba }, 
    { 0x00,  0x1b,  0x366a }, 
    { 0x00,  0x1a,  0x366b }, 
    { 0x00,  0x19,  0x366c }, 
    { 0x00,  0x17,  0x366d }, 

    {   -1,    -1,  0xFFFF }, 	

    { 0x00,  0x00,  0x3000 },
    { 0x02,  0x02,  0x303e },
    {   -1,    -1,  0xFFFF },

    { 0x00,  0x00,  0x30f4 }, 
    //{ 0xa2,  0xa2,  0x3018 },     
}; 

 

int imx274_i2c_init(ISP_DEV IspDev)
{
    char acDevFile[16] = {0};
    HI_U8 u8DevNum;
    int ret;

    if(g_fd[IspDev] >= 0)
    {
        return 0;
    }

    u8DevNum = g_aunImx274BusInfo[IspDev].s8I2cDev;
    snprintf_s(acDevFile, sizeof(acDevFile), sizeof(acDevFile)-1, "/dev/i2c-%d", u8DevNum);

    g_fd[IspDev] = open(acDevFile, O_RDWR);
    if(g_fd[IspDev] < 0)
    {
        printf("Open /dev/i2c-%d error!\n", IspDev);
        return -1;
    }

    ret = ioctl(g_fd[IspDev], I2C_SLAVE_FORCE, (imx274_i2c_addr>>1));
    if (ret < 0)
    {
        printf("CMD_SET_DEV error!\n");
        return ret;
    }
    return 0;
}

int imx274_i2c_exit(ISP_DEV IspDev)
{
    if (g_fd[IspDev] >= 0)
    {
        close(g_fd[IspDev]);
        g_fd[IspDev] = -1;
        return 0;
    }
    return -1;
}

int imx274_write_register(ISP_DEV IspDev,int addr, int data)
{
    if (0 > g_fd[IspDev])
    {
        return 0;
    }

    int idx = 0;
    int ret;
    char buf[8];

    if (imx274_addr_byte == 2)
    {
        buf[idx] = (addr >> 8) & 0xff;
        idx++;
        buf[idx] = addr & 0xff;
        idx++;
    }
    else
    {
        //buf[idx] = addr & 0xff;
        //idx++;
    }

    if (imx274_data_byte == 2)
    {
        //buf[idx] = (data >> 8) & 0xff;
        //idx++;
        //buf[idx] = data & 0xff;
        //idx++;
    }
    else
    {
        buf[idx] = data & 0xff;
        idx++;
    }

    ret = write(g_fd[IspDev], buf, imx274_addr_byte + imx274_data_byte);
    if(ret < 0)
    {
        printf("I2C_WRITE error!\n");
        return -1;
    }

    return 0;
}

int imx274_read_register(ISP_DEV IspDev,unsigned int addr)
{
    int idx = 0;
    int ret;
    char buf[8];
	
    imx274_i2c_init(IspDev);

    buf[idx++] = addr & 0xFF;
    if (imx274_addr_byte == 2)
    {
        ret = ioctl(g_fd[IspDev], I2C_16BIT_REG, 1); // 16 bit addr
        buf[idx++] = addr >> 8;
    }
    else
    {
        ret = ioctl(g_fd[IspDev], I2C_16BIT_REG, 0);
    }

    if (ret < 0)
    {
        printf("CMD_SET_REG_WIDTH error!\n");
        return -1;
    }

    if (imx274_data_byte == 2)
    {
        ret = ioctl(g_fd[IspDev], I2C_16BIT_DATA, 1);
    }
    else
    {
        ret = ioctl(g_fd[IspDev], I2C_16BIT_DATA, 0); // 8 bit data
    }

    if (ret)
    {
        printf("hi_i2c read faild!\n");
        return -1;
    }

    ret = read(g_fd[IspDev], buf, idx);
    if(ret < 0)
    {
        printf("I2C_READ error!\n");
        return -1;
    }

    if (imx274_data_byte == 2) {
        ret = buf[0] | (buf[1] << 8);
    } else
        ret = buf[0];
	
    return ret;
}


static void delay_ms(int ms) { 
    hi_usleep(ms*1000);
}

void imx274_prog(ISP_DEV IspDev,int* rom) 
{
    int i = 0;
    while (1) {
        int lookup = rom[i++];
        int addr = (lookup >> 16) & 0xFFFF;
        int data = lookup & 0xFFFF;
        if (addr == 0xFFFE) {
            delay_ms(data);
        } else if (addr == 0xFFFF) {
            return;
        } else {
            imx274_write_register(IspDev,addr, data);
        }
    }
}

void imx274_standby(ISP_DEV IspDev)
{
    // TODO:
    return;
}

void imx274_restart(ISP_DEV IspDev)
{
    // TODO:
    return;
}


void imx274_init(ISP_DEV IspDev)
{
    HI_U8            u8ImgMode;
    u8ImgMode   = g_astImx274[IspDev].u8ImgMode;
    
    HI_U16 u16RegData;
    HI_U16 u16RegAddr;
    HI_U32 i;
    HI_U32 u32SeqEntries;

   /* 2. sensor i2c init */
    imx274_i2c_init(IspDev);

    /* When sensor first init, config all registers */
    u32SeqEntries = sizeof(gs_au16SensorCfgSeq) / sizeof(gs_au16SensorCfgSeq[0]);
	
    for ( i = 0 ; i < u32SeqEntries; i++ )
    {
        u16RegAddr = gs_au16SensorCfgSeq[i][IMX274_MODE_BUTT];
        u16RegData = gs_au16SensorCfgSeq[i][u8ImgMode];
        if (0xFFFF == u16RegAddr)
        {
            delay_ms(50);
        }
        else
        {
            imx274_write_register(IspDev,u16RegAddr, u16RegData);
        }
    }

    for (i=0; i<g_astImx274[IspDev].astRegsInfo[0].u32RegNum; i++)
    {
        imx274_write_register(IspDev, g_astImx274[IspDev].astRegsInfo[0].astSspData[i].u32RegAddr, g_astImx274[IspDev].astRegsInfo[0].astSspData[i].u32Data);
    }

    printf("IMX274 %s init succuss!\n", g_astImx274ModeTbl[u8ImgMode].pszModeName);

    g_astImx274[IspDev].bInit = HI_TRUE;

    return ;
}

void imx274_exit(ISP_DEV IspDev)
{
    imx274_i2c_exit(IspDev);

    return;
}


