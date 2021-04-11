/* SigmaStar trade secret */
/* Copyright (c) [2019~2020] SigmaStar Technology.
All rights reserved.

Unless otherwise stipulated in writing, any and all information contained
herein regardless in any format shall remain the sole proprietary of
SigmaStar and be kept in strict confidence
(SigmaStar Confidential Information) by the recipient.
Any unauthorized act including without limitation unauthorized disclosure,
copying, use, reproduction, sale, distribution, modification, disassembling,
reverse engineering and compiling of the contents of SigmaStar Confidential
Information is unlawful and strictly prohibited. SigmaStar hereby reserves the
rights to any and all damages, losses, costs and expenses resulting therefrom.
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <drv_sensor.h>
#include <drv_sensor_common.h>
#include <sensor_i2c_api.h>

#ifdef __cplusplus
}
#endif

SENSOR_DRV_ENTRY_IMPL_BEGIN(JXF37_HDR);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE CAM_OS_ARRAY_SIZE
#endif

//#define _DEBUG_
// c11 extern int usleep(u32 usec);
// int usleep(u32 usec);

#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
// MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (2)
//#define SENSOR_MIPI_HDR_MODE (1) //0: Non-HDR mode. 1:Sony DOL mode
// MIPI config end.
//============================================

#define R_GAIN_REG 1
#define G_GAIN_REG 2
#define B_GAIN_REG 3

//#undef SENSOR_DBG
#define SENSOR_DBG 0

///////////////////////////////////////////////////////////////
//          @@@                                                                                       //
//       @   @@      ==  S t a r t * H e r e ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                           //
//         @@@@                                                                                  //
//                                                                                                     //
//      Start Step 1 --  show preview on LCM                                         //
//                                                                                                    ��//
//  Fill these #define value and table with correct settings                        //
//      camera can work and show preview on LCM                                 //
//                                                                                                       //
///////////////////////////////////////////////////////////////

#define SENSOR_ISP_TYPE ISP_EXT // ISP_EXT, ISP_SOC
#define F_number 22 // CFG, demo module
//#define SENSOR_DATAFMT      CUS_DATAFMT_BAYER        //CUS_DATAFMT_YUV, CUS_DATAFMT_BAYER
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_MIPI // CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_FOOTER_EDGE // PACKET_FOOTER_EDGE
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000 // CFG
#define SENSOR_BAYERID CUS_BAYER_GB // CUS_BAYER_BG            //CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAX_GAIN 15872 // 15.5*1024               // max sensor again, a-gain
//#define SENSOR_YCORDER      CUS_SEN_YCODR_YC       //CUS_SEN_YCODR_YC, CUS_SEN_YCODR_CY
#define lane_number 2
#define vc0_hs_mode 3 // 0: packet header edge  1: line end edge 2: line start edge 3: packet footer edge
#define long_packet_type_enable 0x00 // UD1~UD8 (user define)

#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
//#define Preview_line_period 30000                  ////HTS/PCLK=4455 pixels/148.5MHZ=30usec @MCLK=36MHz
//#define vts_30fps 1125//1346,1616                 //for 29.1fps @ MCLK=36MHz
#define Preview_line_period 29630 // 29630                  //(36M/37.125M)*30fps=29.091fps(34.375msec), hts=34.375/1125=30556,
#define Preview_line_period_hdr 24691 // 27777
//#define Line_per_second     32727
#define vts_30fps 1125 // 1266//1150//1090                              //for 29.091fps @ MCLK=36MHz

#define Prv_Max_line_number 2200 // maximum exposure line munber of sensor when preview
#define Preview_WIDTH 1920 // resolution Width when preview
#define Preview_HEIGHT 1080 // resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 5 // slowest preview FPS
#define Preview_CROP_START_X 0 // CROP_START_X
#define Preview_CROP_START_Y 0 // CROP_START_Y

#define SENSOR_I2C_ADDR 0x80 // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A8D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL CUS_CLK_POL_NEG // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG
#define SENSOR_PCLK_POL CUS_CLK_POL_POS // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG
#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_POS // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS

#define JXF37_HDR
#ifdef JXF37_HDR
#define SENSOR_CHANNEL_MODE_HDR CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR
#define vts_15fps_hdr 2700 // 2400//1266//1150//1090
#define Preview_MAX_FPS_HDR 15 // fastest preview FPS
#define Preview_MIN_FPS_HDR 5 // slowest preview FPS
#endif

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_OFFSET
// static int blk_flag = 1;
// static int  drv_Fnumber = 22;
static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int DoOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit);
static int DoOrien_hdr(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit);
CUS_MCLK_FREQ UseParaMclk(void);

CUS_CAMSENSOR_CAP sensor_cap = {
    .length = sizeof(CUS_CAMSENSOR_CAP),
    .version = 0x0001,
};

typedef struct {
    struct {
        u16 pre_div0;
        u16 div124;
        u16 div_cnt7b;
        u16 sdiv0;
        u16 mipi_div0;
        u16 r_divp;
        u16 sdiv1;
        u16 r_seld5;
        u16 r_sclk_dac;
        u16 sys_sel;
        u16 pdac_sel;
        u16 adac_sel;
        u16 pre_div_sp;
        u16 r_div_sp;
        u16 div_cnt5b;
        u16 sdiv_sp;
        u16 div12_sp;
        u16 mipi_lane_sel;
        u16 div_dac;
    } clk_tree;
    struct {
        bool bVideoMode;
        u16 res_idx;
        //        bool binning;
        //        bool scaling;
        CUS_CAMSENSOR_ORIT orit;
    } res;
    struct {
        u32 sclk;
        u32 hts;
        u32 vts;
        u32 ho;
        u32 xinc;
        u32 line_freq;
        u32 us_per_line;
        u32 final_us;
        u32 final_gain;
        u32 a_gain; // cch123
        u32 back_pv_us;
        u32 fps;
        u32 line;
    } expo;

    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool dirty; // same as lef
    bool orient_dirty;
    I2C_ARRAY tGain_reg[1];
    I2C_ARRAY tExpo_reg[2];
    I2C_ARRAY tVts_reg[2];
    I2C_ARRAY tExpo_reg_hdr_sef[1];
    I2C_ARRAY tBsc_reg[3];
    I2C_ARRAY tBsc_reg_hdr_sef[3];
} jxf37_params;
// set sensor ID address and data,

/* typedef struct {
    unsigned int total_gain;
    unsigned short reg_val;
} Gain_ARRAY;
 */
static I2C_ARRAY Sensor_id_table[] = {
    { 0xf0, 0x20 }, // {address of ID, ID },
    { 0xf1, 0x53 },
};

typedef struct {

    u32 gain;
    unsigned short again_reg_val_0;
    unsigned short again_reg_val_1;
    unsigned short again_reg_val_2;
    unsigned short again_reg_val_3;

} Gain_ARRAY;

static I2C_ARRAY Sensor_init_table[] = {
    { 0x12, 0x60 },
    { 0x48, 0x8A },
    { 0x48, 0x0A },
    { 0x0E, 0x11 },
    { 0x0F, 0x14 },
    { 0x10, 0x30 }, // 0x30 27M //0x36 24M
    { 0x11, 0x80 },
    { 0x0D, 0xF0 },
    { 0x5F, 0x42 }, // 0x42 27M //0x41 24M
    { 0x60, 0x2B }, // 0x2B 27M //0x20 24M
    { 0x58, 0x12 },
    { 0x57, 0x60 },
    { 0x9D, 0x00 },
    { 0x20, 0x80 }, // 00
    { 0x21, 0x07 }, // 05
    { 0x22, 0x65 },
    { 0x23, 0x04 },
    { 0x24, 0xC4 }, // c0
    { 0x25, 0x40 }, // 38
    { 0x26, 0x43 },
    { 0x27, 0x18 }, // 97
    { 0x28, 0x11 }, // 1D
    { 0x29, 0x07 }, // 04
    { 0x2A, 0x0C }, // 8a
    { 0x2B, 0x17 }, // 14
    { 0x2C, 0x00 },
    { 0x2D, 0x00 },
    { 0x2E, 0x14 }, // 16
    { 0x2F, 0x44 },
    { 0x41, 0xC7 }, // c5
    { 0x42, 0x3B },
    { 0x47, 0x42 }, // 62
    { 0x76, 0x6A }, // 60
    { 0x77, 0x09 },
    { 0x1D, 0x00 },
    { 0x1E, 0x04 },
    { 0x6C, 0x40 },
    { 0x6E, 0x2C },
#if 1
    { 0x70, 0xD0 }, // old DC//520
    { 0x71, 0xD3 },
    { 0x72, 0xD4 },
    { 0x73, 0x58 },
    { 0x74, 0x02 },
    { 0x78, 0xAE }, // old 96//520
#else
    { 0x70, 0x6C },
    { 0x71, 0x6D },
    { 0x72, 0x6A },
    { 0x73, 0x36 },
    { 0x74, 0x02 },
    { 0x78, 0x9E },
#endif
    { 0x89, 0x01 },
    { 0x6B, 0x20 },

    { 0x86, 0x40 }, // add
    { 0x31, 0x0C }, // add
    { 0x32, 0x38 }, // add
    { 0x33, 0x6C }, // add
    { 0x34, 0x88 }, // add
    { 0x35, 0x88 }, // add
    { 0x3A, 0xAF }, // add

    //    {0x31,0x08},
    //    {0x32,0x27},
    //    {0x33,0x60},
    //    {0x34,0x5E},
    //    {0x35,0x5E},
    //    {0x3A,0xAF},
    /// add
    { 0x3B, 0x00 },
    { 0x3C, 0x57 },
    { 0x3D, 0x78 },
    { 0x3E, 0xFF },
    { 0x3F, 0xF8 },
    { 0x40, 0xFF },
    ////
    { 0x56, 0xB2 },
    { 0x59, 0xE8 }, // 9E
    { 0x5A, 0x04 },
    { 0x85, 0x70 }, // 4D
    { 0x8A, 0x04 },
    { 0x91, 0x13 },
    { 0x9B, 0x03 },
    ////add
    { 0x9C, 0xE1 },
    ////
    { 0xA9, 0x78 },
    { 0x5B, 0xB0 },
    { 0x5C, 0x71 },

    { 0x5D, 0x46 }, // 8C
    { 0x5E, 0x14 }, // 1F
    ///////////////
    { 0x62, 0x01 },
    { 0x63, 0x0F },
    { 0x64, 0xC0 },
    { 0x65, 0x02 }, // 06

    { 0x67, 0x65 }, // 55
    { 0x66, 0x04 },

    { 0x68, 0x00 },
    { 0x69, 0x7C },

    { 0x6A, 0x12 }, // 18

    { 0x7A, 0x80 },
    { 0x82, 0x20 },
    { 0x8F, 0x91 },
    { 0xAE, 0x30 },
    { 0x13, 0x81 },
    { 0x96, 0x04 },
    { 0x4A, 0x05 },
    { 0x7E, 0xCD },
    { 0x50, 0x02 },
    { 0x49, 0x10 }, // old40

    { 0xAF, 0x12 },
    { 0x80, 0x01 }, /// for mixer
    { 0x7B, 0x4A },
    { 0x7C, 0x08 },
    { 0x7F, 0x57 },
    { 0x90, 0x00 },
    { 0x8C, 0xFF },
    { 0x8D, 0xC7 },
    { 0x8E, 0x00 },
    { 0x8B, 0x01 },
    { 0x0C, 0x00 },
    ////add
    { 0x81, 0x74 },
    ////
    { 0x19, 0x28 }, // old 0x20 520
    { 0x46, 0x00 },
    { 0x12, 0x20 },
    { 0x48, 0x8A },
    { 0x48, 0x0A },
    /* init reg done */
};

#ifdef JXF37_HDR
static I2C_ARRAY Sensor_init_table_hdr[] = {
    //  3200x2700 @ 129.6Mhz, 15fps, line-time : 24.691 us
    { 0x12, 0x68 },
    { 0x48, 0x8A },
    { 0x48, 0x0A },
    { 0x0E, 0x11 },
    { 0x0F, 0x14 },
    { 0x10, 0x30 },
    { 0x11, 0x80 },
    { 0x0D, 0xF0 },
    { 0x5F, 0x42 },
    { 0x60, 0x2B },
    { 0x58, 0x12 },
    { 0x57, 0x60 },
    { 0x9D, 0x00 },
    { 0x20, 0x40 },
    { 0x21, 0x06 },
    { 0x22, 0x8C },
    { 0x23, 0x0A },
    { 0x24, 0xC4 },
    { 0x25, 0x40 },
    { 0x26, 0x43 },
    { 0x27, 0x0E },
    { 0x28, 0x21 },
    { 0x29, 0x06 },
    { 0x2A, 0x00 },
    { 0x2B, 0x16 },
    { 0x2C, 0x00 },
    { 0x2D, 0x00 },
    { 0x2E, 0x16 },
    { 0x2F, 0x44 },
    { 0x41, 0xC9 },
    { 0x42, 0x3B },
    { 0x47, 0x42 },
    { 0x76, 0x6A },
    { 0x77, 0x09 },
    { 0x80, 0x01 }, // 0x41},
    { 0xAF, 0x22 },
    { 0xAB, 0x00 },
    { 0x46, 0x04 },
    { 0x1D, 0x00 },
    { 0x1E, 0x04 },
    { 0x6C, 0x40 },
    { 0x6E, 0x2C },
    { 0x70, 0xD0 },
    { 0x71, 0xD3 },
    { 0x72, 0xD4 },
    { 0x73, 0x58 },
    { 0x74, 0x02 },
    { 0x78, 0xAE }, //
    { 0x89, 0x81 },
    { 0x6B, 0x20 },
    { 0x86, 0x40 },
    { 0x31, 0x0C },
    { 0x32, 0x38 },
    { 0x33, 0x6C },
    { 0x34, 0x80 }, // 0x68
    { 0x35, 0x68 },
    { 0x3A, 0xAF },
    { 0x3B, 0x00 },
    { 0x3C, 0x57 },
    { 0x3D, 0x78 },
    { 0x3E, 0xFF },
    { 0x3F, 0xD8 },
    { 0x40, 0xFF },
    { 0x56, 0xB2 },
    { 0x59, 0xC8 },
    { 0x5A, 0x04 },
    { 0x85, 0x60 },
    { 0x8A, 0x04 },
    { 0x91, 0x13 },
    { 0x9B, 0x43 },
    { 0x9C, 0xE1 },
    { 0xA9, 0x78 },
    { 0x5B, 0xB0 },
    { 0x5C, 0x71 },
    { 0x5D, 0xF6 },
    { 0x5E, 0x14 },
    { 0x62, 0x01 },
    { 0x63, 0x0F },
    { 0x64, 0xC0 },
    { 0x65, 0x02 },
    { 0x67, 0x65 },
    { 0x66, 0x04 },
    { 0x68, 0x00 },
    { 0x69, 0x7C },
    { 0x6A, 0x12 },
    { 0x7A, 0x80 },
    { 0x82, 0x21 },
    { 0x8F, 0x91 },
    { 0xAE, 0x30 },
    { 0x13, 0x81 },
    { 0x96, 0x04 },
    { 0x4A, 0x05 },
    { 0x7E, 0xCD },
    { 0x50, 0x02 },
    { 0x49, 0x10 }, // 0x10},//old:40
    { 0xAF, 0x12 },
    { 0x7B, 0x4A },
    { 0x7C, 0x08 },
    { 0x7F, 0x57 },
    { 0x90, 0x00 },
    { 0x8C, 0xFF },
    { 0x8D, 0xC7 },
    { 0x8E, 0x00 },
    { 0x8B, 0x01 },
    { 0x0C, 0x00 },
    { 0x81, 0x74 },
    { 0x19, 0x28 }, // 0x20},
    { 0x07, 0x03 },
    { 0x1B, 0x4F },
    { 0x06, 0x6F },
    { 0x03, 0xFF },
    { 0x04, 0xFF },
    { 0x12, 0x28 },
    { 0x48, 0x8A },
    { 0x48, 0x0A },
    { 0x05, 0x10 }

};
#endif

I2C_ARRAY TriggerStartTbl[] = {
    //{0x30f4,0x00},//Master mode start
};

I2C_ARRAY PatternTbl[] = {
    // pattern mode
};

I2C_ARRAY Current_Mirror_Flip_Tbl[] = {
    { 0x17, 0x80 }, // bit[1:0]
};

/////////////////////////////////////////////////////////////////
//       @@@@@@                                                                                    //
//                 @@                                                                                    //
//             @@@                                                                                      //
//       @       @@                                                                                    //
//         @@@@                                                                                        //
//                                                                                                          //
//      Step 3 --  complete camera features                                              //
//                                                                                                         //
//                                                                                                         //
//  camera set EV, MWB, orientation, contrast, sharpness                          //
//   , saturation, and Denoise can work correctly.                                     //
//                                                                                                          //
/////////////////////////////////////////////////////////////////

#ifdef SENSOR_OFFSET
static I2C_ARRAY mirr_flip_table[] = {
    { 0x12, 0x20 }, // mirror bit[5]
    { 0x27, 0x18 },
    { 0x28, 0x11 },
    { 0x80, 0x41 }, // crop for bayer mirror
    { 0x46, 0x04 },
};

static I2C_ARRAY mirr_flip_table_hdr[] = {
    { 0x12, 0x20 }, // mirror bit[5]
    { 0x27, 0x0E },
    { 0x28, 0x21 },
    { 0x80, 0x41 }, // crop for bayer mirror
    { 0x46, 0x04 },
};
#else
static I2C_ARRAY mirr_flip_table[] = {
    { 0x12, 0x20 }, // mirror bit[5]
    //    {0x27, 0x18},
    //    {0x28, 0x11},
    { 0x80, 0x41 }, // crop for bayer mirror
    { 0x46, 0x04 },
};

static I2C_ARRAY mirr_flip_table_hdr[] = {
    { 0x12, 0x20 }, // mirror bit[5]
    //    {0x27, 0x4A},
    //    {0x28, 0x21},
    { 0x80, 0x41 }, // crop for bayer mirror
    { 0x46, 0x04 },
};
#endif

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

static int g_sensor_ae_min_gain = 1024; // 1280;

const I2C_ARRAY gain_reg[] = {
    { 0x00, 0x00 },
};

const I2C_ARRAY expo_reg[] = {
    { 0x02, 0x00 }, // long expo[15:8] texp=expo[15:0]*Tline
    { 0x01, 0xff }, // long expo[7:0]
};

const I2C_ARRAY vts_reg[] = {
    { 0x23, 0x04 },
    { 0x22, 0x65 }, // frame H 1125
};

I2C_ARRAY temperature_reg[] = {
    { 0x0c, 0x30 },
    { 0x0d, 0x30 },
    { 0x0e, 0x30 },
    { 0x0f, 0x30 },
};

const I2C_ARRAY expo_reg_sef[] = {
    { 0x05, 0x10 }, // long expo[15:8] texp=expo[15:0]*Tline
};

// from jxf37 application note
const I2C_ARRAY bsc_reg[] = {
    { 0x2F, 0x44 },
    { 0x0C, 0x00 },
    { 0x82, 0x20 },
};

const I2C_ARRAY bsc_reg_hdr[] = {
    { 0x2F, 0x44 },
    { 0x0C, 0x00 },
    { 0x82, 0x21 },
};

/////////// function definition ///////////////////
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME jxf37

//#define SensorReg_Read(_reg,_data)     (handle->i2c_bus->i2c_rx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
//#define SensorReg_Write(_reg,_data)    (handle->i2c_bus->i2c_tx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
//#define SensorRegArrayW(_reg,_len)  (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))
//#define SensorRegArrayR(_reg,_len)  (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))
#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

static int cus_camsensor_release_handle(ms_cus_sensor* handle);

/////////////////// sensor hardware dependent //////////////
#if 0
static int ISP_config_io(ms_cus_sensor *handle) {
    ISensorIfAPI *sensor_if = handle->sensor_if_api;//&handle->sensor_if_api;

    SENSOR_DMSG("[%s]", __FUNCTION__);

    sensor_if->HsyncPol(handle, handle->HSYNC_POLARITY);
    sensor_if->VsyncPol(handle, handle->VSYNC_POLARITY);
    sensor_if->ClkPol(handle, handle->PCLK_POLARITY);
    sensor_if->BayerFmt(handle, handle->bayer_id);
    sensor_if->DataBus(handle, handle->sif_bus);

    sensor_if->DataPrecision(handle, handle->data_prec);
    sensor_if->FmtConv(handle,  handle->data_mode);
    return SUCCESS;
}
#endif

// static u32 timeGetTimeU(void)
//{
//     CamOsTimespec_t tRes;
//     CamOsGetMonotonicTime(&tRes);
//     return (tRes.nSec * 1000000)+(tRes.nNanoSec/1000);
// }
// static u32 TStart = 0;

static int pCus_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api; //&handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    // TStart = timeGetTimeU();
    /*PAD and CSI*/
    //    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    //    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);   ///???
    //    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);  ///???
    //    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);    //=========   ????

    //  if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
    //      sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, CUS_HDR_MODE_DCG);
    //    }
    /*Power ON*/
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    CamOsMsSleep(5);

    /*Reset PIN*/
    SENSOR_EMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    CamOsMsSleep(2);
    //    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_EMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    CamOsMsSleep(2);

    /*MCLK ON*/
    sensor_if->MCLK(idx, 1, handle->mclk);

    /*Reset PIN*/
    sensor_if->Reset(idx, handle->reset_POLARITY);
    CamOsMsSleep(20);
    //    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    CamOsMsSleep(2);

    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    CamOsMsSleep(2);
    // CamOsPrintf("pCus_poweron = %d us \n",timeGetTimeU()-TStart);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M); ///???
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1); ///???
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0); //=========   ????

    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, CUS_HDR_MODE_DCG);
    }

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api; //&handle->sensor_if_api;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    // handle->i2c_bus->i2c_close(handle->i2c_bus);
    // CamOsMsSleep(20);
    // Set_csi_if(0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);
    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 0);
    }
    CamOsMsSleep(50);
    return SUCCESS;
}

/////////////////// image function /////////////////////////
// Get and check sensor ID
// if i2c error or sensor id does not match then return FAIL
static int pCus_GetSensorID(ms_cus_sensor* handle, u32* id)
{
    int i, n;
    int table_length = ARRAY_SIZE(Sensor_id_table);
    I2C_ARRAY id_from_sensor[ARRAY_SIZE(Sensor_id_table)];

    SensorReg_Write(0xfe, 0x00);

    for (n = 0; n < table_length; ++n) {
        id_from_sensor[n].reg = Sensor_id_table[n].reg;
        id_from_sensor[n].data = 0;
    }

    *id = 0;
    if (table_length > 8)
        table_length = 8;

    for (n = 0; n < 4; ++n) // retry , until I2C success
    {
        if (n > 2)
            return FAIL;

        if (SensorRegArrayR((I2C_ARRAY*)id_from_sensor, table_length) == SUCCESS) // read sensor ID from I2C
            break;
        else
            continue;
    }

    // convert sensor id to u32 format
    for (i = 0; i < table_length; ++i) {
        if (id_from_sensor[i].data != Sensor_id_table[i].data)
            return FAIL;
        //*id = id_from_sensor[i].data;
        *id = ((*id) + id_from_sensor[i].data) << 8;
    }

    *id >>= 8;
    SENSOR_EMSG("[%s]jxf37 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    // SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);

    return SUCCESS;
}

static int jxf37_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{

    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);

    return SUCCESS;
}
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
// static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init(ms_cus_sensor* handle)
{
    int i, cnt = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    // ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
    // sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);
    // TStart = timeGetTimeU();

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table); i++) {
        if (Sensor_init_table[i].reg == 0xffff) {
            msleep(Sensor_init_table[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table[i].reg, Sensor_init_table[i].data) != SUCCESS) {
                cnt++;
                // SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if (cnt >= 10) {
                    // SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                msleep(10);
            }
        }
    }

    // CamOsPrintf("pCus_init = %d us \n",timeGetTimeU()-TStart);
    DoOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;

    CamOsMsSleep(20);
    SensorReg_Write(0x74, 0x03);
    CamOsMsSleep(20);
    SensorReg_Write(0x74, 0x02);
    return SUCCESS;
}
/*
int pCus_release(ms_cus_sensor *handle)
{
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    sensor_if->PCLK(NULL,CUS_PCLK_OFF);
    return SUCCESS;
}
*/

static int pCus_GetVideoResNum(ms_cus_sensor* handle, u32* ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}

static int pCus_GetVideoRes(ms_cus_sensor* handle, u32 res_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int pCus_GetCurVideoRes(ms_cus_sensor* handle, u32* cur_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int pCus_SetVideoRes(ms_cus_sensor* handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0:
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init;
        break;

    default:
        break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;

    sen_data = mirr_flip_table[0].data & 0x30;
    SENSOR_DMSG("\n\n[%s]:mirror:%x\r\n\n\n\n", __FUNCTION__, sen_data);
    switch (sen_data) {
    case 0x00:
        *orit = CUS_ORIT_M0F0;
        break;
    case 0x20:
        *orit = CUS_ORIT_M1F0;
        break;
    case 0x10:
        *orit = CUS_ORIT_M0F1;
        break;
    case 0x30:
        *orit = CUS_ORIT_M1F1;
        break;
    }

    return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    handle->orient = orit;
    params->orient_dirty = true;

    return SUCCESS;
}

static int DoOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    // jxf37_params *params = (jxf37_params *)handle->private_data;
    int table_length = ARRAY_SIZE(mirr_flip_table);
    int i;

    switch (orit) {
    case CUS_ORIT_M0F0:
#ifdef SENSOR_OFFSET
        mirr_flip_table[0].data = 0x00;
        mirr_flip_table[1].data = 0x18;
        mirr_flip_table[2].data = 0x11;
        mirr_flip_table[3].data = 0x41;
// mirr_flip_table[4].data = 0x00;
#else
        mirr_flip_table[0].data = 0x00;
        mirr_flip_table[1].data = 0x41;
        mirr_flip_table[2].data = 0x00;
#endif
        // handle->bayer_id=CUS_BAYER_BG;//>???
        handle->orient = CUS_ORIT_M0F0;

        break;
    case CUS_ORIT_M1F0:
#ifdef SENSOR_OFFSET
        mirr_flip_table[0].data = 0x20;
        mirr_flip_table[1].data = 0x18;
        mirr_flip_table[2].data = 0X11;
        mirr_flip_table[3].data = 0x01;
#else
        mirr_flip_table[0].data = 0x20;
        handle->orient = CUS_ORIT_M1F0;
#endif
        break;
        // ttttt
    case CUS_ORIT_M0F1:
#ifdef SENSOR_OFFSET
        mirr_flip_table[0].data = 0x10;
        mirr_flip_table[1].data = 0x18;
        mirr_flip_table[2].data = 0x12;
        mirr_flip_table[3].data = 0x41;
#else
        mirr_flip_table[0].data = 0x10;
#endif
        handle->orient = CUS_ORIT_M0F1;

        break;
    case CUS_ORIT_M1F1:
#ifdef SENSOR_OFFSET
        mirr_flip_table[0].data = 0x30;
        mirr_flip_table[1].data = 0x18;
        mirr_flip_table[2].data = 0x12;
        mirr_flip_table[3].data = 0x01;
#else
        mirr_flip_table[0].data = 0x30;
#endif
        handle->orient = CUS_ORIT_M1F1;

        break;
    }
    for (i = 0; i < table_length; i++) {
        SensorReg_Write(mirr_flip_table[i].reg, mirr_flip_table[i].data);
    }

    SENSOR_DMSG("pCus_SetOrien:%x,%x\r\n", orit, mirr_flip_table[0].data);
    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor* handle)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    // SENSOR_DMSG("[%s]", __FUNCTION__);

    return params->expo.fps;
}
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    int vts = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("\n\n[%s][%d]", __FUNCTION__, fps);

    // return SUCCESS;

    if (fps >= 5 && fps <= 30) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * 30) / fps;
    } else if (fps >= 5000 && fps <= 30000) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * 30000) / fps;
    } else {
        // params->expo.vts=vts_30fps;
        // params->expo.fps=30;
        // SENSOR_DMSG("[%s] FPS %d out of range.\n",__FUNCTION__,fps);
        return FAIL;
    }

    if ((params->expo.line) > (params->expo.vts) - 4) {
        vts = params->expo.line + 4;
    } else
        vts = params->expo.vts;
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;

    params->dirty = true;
    return SUCCESS;
}

#if 0
static int pCus_GetSensorCap(ms_cus_sensor *handle, CUS_CAMSENSOR_CAP *cap) {
    if (cap)
        memcpy(cap, &sensor_cap, sizeof(CUS_CAMSENSOR_CAP));
    else     return FAIL;
    return SUCCESS;
}
#endif

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    //   return SUCCESS;

    switch (status) {
    case CUS_FRAME_INACTIVE:
        if (params->orient_dirty) {
            DoOrien(handle, handle->orient);
            handle->sensor_if_api->SetSkipFrame(handle->snr_pad_group, params->expo.fps, 3);
            params->orient_dirty = false;
        }
        break;
    case CUS_FRAME_ACTIVE:
        if (params->dirty) {

            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));

            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tBsc_reg, ARRAY_SIZE(bsc_reg));
            SENSOR_DMSG("[%s] gain:%x, exp:%x%x\n", "Linear:", params->tGain_reg[0], params->tExpo_reg[0], params->tExpo_reg[1]);
            params->dirty = false;
        }

        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    u32 lines = 0;

    lines = (u32)(params->tExpo_reg[0].data & 0xff);
    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 8;

    *us = (lines * Preview_line_period) / 1000;
    // SENSOR_DMSG("====================================================\n");
    // SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);
    // SENSOR_DMSG("====================================================\n");

    SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);

    return SUCCESS;
}

static int pCus_GetAEUSecs_hdr(ms_cus_sensor* handle, u32* us)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    u32 lines = 0;

    lines = (u32)(params->tExpo_reg[0].data & 0xff);
    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 8;

    *us = (lines * Preview_line_period_hdr) / 1000;
    // SENSOR_DMSG("====================================================\n");
    // SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);
    // SENSOR_DMSG("====================================================\n");

    SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);

    return SUCCESS;
}

static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("pCus_SetAEUSecs[%s] val=[%d]\n", __FUNCTION__, us);

    lines = (1000 * us) / Preview_line_period; // Preview_line_period in ns
    if (lines < 1) {
        lines = 1;
    }

    if (lines > params->expo.vts - 4) {
        vts = lines + 4;
    } else {
        vts = params->expo.vts;
    }

    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->tExpo_reg[0].data = (lines >> 8) & 0xff;
    params->tExpo_reg[1].data = (lines >> 0) & 0xff;
    params->dirty = true;
    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__, us, lines, params->expo.vts);
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;

    u32 temp_gain = 0, temp_gain1 = 0, temp_gain2 = 0;

    temp_gain = ((params->tGain_reg[0].data & 0x70) >> 4);

    if (temp_gain == 0)
        temp_gain1 = 1;
    else if (temp_gain == 1)
        temp_gain1 = 2;
    else if (temp_gain == 2)
        temp_gain1 = 4;
    else if (temp_gain == 3)
        temp_gain1 = 8;

    temp_gain2 = 1024 + (1024 * (params->tGain_reg[0].data & 0x0f)) / 16;
    *gain = temp_gain1 * temp_gain2;
    SENSOR_DMSG("[%s] gain/reg [%ld]\n", __FUNCTION__, *gain);

    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    u32 gain2_4, gain4_8, gain8_16;
    SENSOR_DMSG("[%s] pCus_SetAEGain:%ld\n", __FUNCTION__, gain);
    params->expo.final_gain = gain;

    if (gain < 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAX_GAIN) {
        gain = SENSOR_MAX_GAIN;
    }

    if (gain < 2048) {
        params->tGain_reg[0].data = (((gain - 1024) >> 6)) & 0x000f; //<X2
        params->tBsc_reg[0].data = 0x64;
        params->tBsc_reg[1].data = 0x40;
        params->tBsc_reg[2].data = 0x22;
        params->tBsc_reg_hdr_sef[0].data = 0x64;
        params->tBsc_reg_hdr_sef[1].data = 0x40;
        params->tBsc_reg_hdr_sef[2].data = 0x23;

    } else {
        params->tBsc_reg[0].data = 0x44;
        params->tBsc_reg[1].data = 0x00;
        params->tBsc_reg[2].data = 0x20;
        params->tBsc_reg_hdr_sef[0].data = 0x44;
        params->tBsc_reg_hdr_sef[1].data = 0x00;
        params->tBsc_reg_hdr_sef[2].data = 0x21;
        if ((gain >= 2048) && (gain < 4096)) // X2~X4
        {
            gain2_4 = gain - 2048;
            params->tGain_reg[0].data = ((gain2_4 >> 7) & 0x000f) | 0x10;
        } else if ((gain >= 4096) && (gain < 8192)) // X4~X8
        {
            gain4_8 = gain - 4096;
            params->tGain_reg[0].data = ((gain4_8 >> 8) & 0x000f) | 0x20;
        } else if ((gain >= 8192) && (gain <= 15872)) // X8~X15.5
        {
            gain8_16 = gain - 8192;
            params->tGain_reg[0].data = ((gain8_16 >> 9) & 0x000f) | 0x30;
        }
    }

    // SENSOR_EMSG("[%s] pCus_SetAEGain:%ld,%x\n", __FUNCTION__, params->expo.final_gain , params->tGain_reg[0].data);
    params->dirty = true;
    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    SENSOR_DMSG("\n\n[%s]\n", __FUNCTION__);
    *min = 1; // 30
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    SENSOR_DMSG("\n\n[%s]\n", __FUNCTION__);
    *min = handle->sat_mingain;
    *max = SENSOR_MAX_GAIN;
    return SUCCESS;
}

static int jxf37_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    SENSOR_DMSG("\n\n[%s]\n", __FUNCTION__);
    info->max = 1000000000 / Preview_MIN_FPS; /// 12;
    info->min = Preview_line_period * 1; // 2
    info->step = Preview_line_period * 1; // 2
    return SUCCESS;
}

static int jxf37_GetShutterInfo_hdr(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    SENSOR_DMSG("\n\n[%s]\n", __FUNCTION__);
    info->max = 1000000000 / Preview_MIN_FPS; /// 12;
    info->min = Preview_line_period_hdr * 1; // 2
    info->step = Preview_line_period_hdr * 1; // 2
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    //  u32 i, j;
    return SUCCESS;
}

#ifdef JXF37_HDR
static int DoOrien_hdr(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    // jxf37_params *params = (jxf37_params *)handle->private_data;
    int table_length = ARRAY_SIZE(mirr_flip_table_hdr);
    int i;
    SENSOR_EMSG("\n\n[%s][%d]\n", __FUNCTION__, orit);
    // return SUCCESS;

    switch (orit) {
    case CUS_ORIT_M0F0:
#ifdef SENSOR_OFFSET
        mirr_flip_table_hdr[0].data = 0x08;
        mirr_flip_table_hdr[1].data = 0x0E;
        mirr_flip_table_hdr[2].data = 0x21;
        mirr_flip_table_hdr[3].data = 0x41;
#else
        mirr_flip_table_hdr[0].data = 0x08;
#endif
        handle->orient = CUS_ORIT_M0F0;

        break;
    case CUS_ORIT_M1F0: // OK
#ifdef SENSOR_OFFSET
        mirr_flip_table_hdr[0].data = 0x28;
        mirr_flip_table_hdr[1].data = 0x0E;
        mirr_flip_table_hdr[2].data = 0x21;
        mirr_flip_table_hdr[3].data = 0x01;
#else
        mirr_flip_table_hdr[0].data = 0x28;
#endif
        handle->orient = CUS_ORIT_M1F0;

        break;
    case CUS_ORIT_M0F1:
#ifdef SENSOR_OFFSET
        mirr_flip_table_hdr[0].data = 0x18;
        mirr_flip_table_hdr[1].data = 0x0E;
        mirr_flip_table_hdr[2].data = 0x2F;
        mirr_flip_table_hdr[3].data = 0x41;
#else
        mirr_flip_table_hdr[0].data = 0x18;
#endif

        handle->orient = CUS_ORIT_M0F1;

        break;
    case CUS_ORIT_M1F1: // ok
#ifdef SENSOR_OFFSET
        mirr_flip_table_hdr[0].data = 0x38;
        mirr_flip_table_hdr[1].data = 0x0E;
        mirr_flip_table_hdr[2].data = 0x2F;
        mirr_flip_table_hdr[3].data = 0x01;
#else
        mirr_flip_table_hdr[0].data = 0x38;
#endif
        handle->orient = CUS_ORIT_M1F1;

        break;
    }
    for (i = 0; i < table_length; i++) {
        SensorReg_Write(mirr_flip_table_hdr[i].reg, mirr_flip_table_hdr[i].data);
    }
    // handle->sensor_if_api->SetSkipFrame(handle->snr_pad_group, params->expo.fps, 3);
    SENSOR_DMSG("pCus_SetOrien:%x,%x\r\n", orit, mirr_flip_table_hdr[0].data);
    return SUCCESS;
}

static int pCus_AEStatusNotify_hdr_sef(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:
        if (params->orient_dirty) {
            DoOrien_hdr(handle, handle->orient);
            handle->sensor_if_api->SetSkipFrame(handle->snr_pad_group, params->expo.fps, 3);
            params->orient_dirty = false;
        }
        break;
    case CUS_FRAME_ACTIVE:
        if (params->dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg_hdr_sef, ARRAY_SIZE(expo_reg_sef));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));

            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tBsc_reg_hdr_sef, ARRAY_SIZE(bsc_reg_hdr));
            // SENSOR_DMSG("[%s] gain:%x, l_exp:%x,%x, s_exp:%x\n", "HDR:", params->tGain_reg[0], params->tExpo_reg[0], params->tExpo_reg[1], params->tExpo_reg_hdr_sef[0]);
            params->dirty = false;
        }

        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_SetAEUSecs_hdr_lef(ms_cus_sensor* handle, u32 us)
{
#if 1
    u32 lines = 0, vts = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("pCus_SetAEUSecs_hdr_lef[%s] val=[%d]\n", __FUNCTION__, us);

    lines = (1000 * us) / Preview_line_period_hdr; // Preview_line_period_hdr in ns

    if (lines < 1) {
        lines = 1;
    }

    if (lines % 2 == 0) // only accept odd lines
        lines--;

    if (lines > params->expo.vts - 4) {
        vts = lines + 4;
    } else {
        vts = params->expo.vts;
    }

    /*     if(vts != params->expo.vts)
        {
            params->expo.fps = (vts_30fps*30+(vts>>1))/vts;
        } */

    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->tExpo_reg[0].data = (lines >> 8) & 0xff;
    params->tExpo_reg[1].data = (lines >> 0) & 0xff;
    params->dirty = true;
    // SENSOR_EMSG("[%s] us %ld, lines %ld, vts %ld,%ld\n", __FUNCTION__, us, lines, params->expo.vts,vts);
#endif
    return SUCCESS;
}

static int pCus_SetAEUSecs_hdr_sef(ms_cus_sensor* handle, u32 us)
{
#if 1
    u32 lines = 0, vts = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_DMSG("pCus_SetAEUSecs_hdr_sef[%s] val=[%d]\n", __FUNCTION__, us);

    lines = (1000 * us) / Preview_line_period_hdr; // Preview_line_period_hdr in ns
    if (lines < 1) {
        lines = 1;
    }
#if 1 // do not cal vts in sef
    if (lines > params->expo.vts - 4) {
        vts = lines + 4;
    } else {
        vts = params->expo.vts;
    }

    if (vts != params->expo.vts) {
        params->expo.fps = (vts_30fps * 30 + (vts >> 1)) / vts;
    }
#endif
    if (lines > 511)
        lines = 511;

    lines = (lines - 1) / 2;

    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->tExpo_reg_hdr_sef[0].data = lines & 0xff; //(lines>>8) & 0xff;
    params->dirty = true; // tttt
    // SENSOR_EMSG("[%s] us %ld, lines %ld, vts %ld,%ld\n", __FUNCTION__, us, lines, params->expo.vts,vts);
#endif
    return SUCCESS;
}

static int pCus_AEStatusNotify_hdr_lef(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    // jxf37_params *params = (jxf37_params *)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:
        break;
    case CUS_FRAME_ACTIVE:

        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_init_hdr(ms_cus_sensor* handle)
{
    int i, cnt = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_EMSG("\n\[%s]\n\n", __FUNCTION__);
    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_hdr); i++) {
        if (Sensor_init_table_hdr[i].reg == 0xffff) {
            msleep(Sensor_init_table_hdr[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_hdr[i].reg, Sensor_init_table_hdr[i].data) != SUCCESS) {
                cnt++;
                // SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if (cnt >= 10) {
                    // SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                msleep(10);
            }
        }
    }
    DoOrien_hdr(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;

    CamOsMsSleep(20);
    SensorReg_Write(0x74, 0x03);
    CamOsMsSleep(20);
    SensorReg_Write(0x74, 0x02);

    return SUCCESS;
}

static int pCus_SetVideoRes_hdr(ms_cus_sensor* handle, u32 res_idx)
{
    SENSOR_DMSG("\n\npCus_SetVideoRes_HDR_DOL\n");
    return SUCCESS;
}

static int pCus_SetFPS_hdr_lef(ms_cus_sensor* handle, u32 fps)
{
    int vts = 0;
    jxf37_params* params = (jxf37_params*)handle->private_data;
    SENSOR_EMSG("\n\n[%s][%d]", __FUNCTION__, fps);

    if (fps >= 5 && fps <= 15) {
        params->expo.fps = fps;
        params->expo.vts = (vts_15fps_hdr * 15) / fps;
    } else if (fps >= 5000 && fps <= 15000) {
        params->expo.fps = fps;
        params->expo.vts = (vts_15fps_hdr * 15000) / fps;
    } else {
        fps = 15;
        params->expo.fps = fps;
        params->expo.vts = (vts_15fps_hdr * 15) / fps;
        // return FAIL;
    }

    if ((params->expo.line) > (params->expo.vts) - 4) {
        vts = params->expo.line + 4;
    } else
        vts = params->expo.vts;
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;

    params->dirty = true;
    return SUCCESS;
}

static u32 pCus_TryAEGain(ms_cus_sensor* handle, u32 gain)
{
    jxf37_params* params = (jxf37_params*)handle->private_data;
#if 1
    u32 gain1_2, gain2_4, gain4_8, gain8_16;

    if (gain < 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAX_GAIN) {
        gain = SENSOR_MAX_GAIN;
    }

    if (gain < 2048) {
        gain1_2 = gain - 1024;
        params->expo.a_gain = 64 * (16 + (gain1_2 >> 6));
    } else {
        if ((gain >= 2048) && (gain < 4096)) // X2~X4
        {
            gain2_4 = gain - 2048;
            params->expo.a_gain = 64 * 2 * (16 + (gain2_4 >> 7));
        } else if ((gain >= 4096) && (gain < 8192)) // X4~X8
        {
            gain4_8 = gain - 4096;
            params->expo.a_gain = 64 * 4 * (16 + (gain4_8 >> 8));
        } else if ((gain >= 8192) && (gain <= 15872)) // X8~X15.5
        {
            gain8_16 = gain - 8192;
            params->expo.a_gain = 64 * 8 * (16 + (gain8_16 >> 9));
        }
    }
    return params->expo.a_gain; // 4096;//cch123
//  SENSOR_EMSG("[%s] for test:%d,%d\n", __FUNCTION__, gain, params->expo.a_gain);
#else // workaround
    u32 i;
    u32 nGainTable[] = { 1, 2, 4, 8 };
    u32 nMaxId = sizeof(nGainTable) / sizeof(nGainTable[0]);
    params->expo.a_gain = 0;

    for (i = 1; i < nMaxId; ++i) {
        if ((nGainTable[i] * 1024) > gain) {
            params->expo.a_gain = (nGainTable[i - 1] * 1024);
            break;
        }
    }

    if (params->expo.a_gain == 0)
        params->expo.a_gain = (nGainTable[nMaxId - 1] * 1024);

    // pr_info("SG Target = %d, Actual = %d\n", gain, params->expo.a_gain);

    return params->expo.a_gain;
#endif
}

#endif

static int cus_camsensor_init_handle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    jxf37_params* params;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    // private data allocation & init
    // handle->private_data = CamOsMemCalloc(1, sizeof(jxf37_params));
    if (handle->private_data == NULL) {
        SENSOR_EMSG("[%s] Private data is empty!\n", __FUNCTION__);
        return FAIL;
    }
    params = (jxf37_params*)handle->private_data;

    /////copy para////
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tBsc_reg, bsc_reg, sizeof(bsc_reg));
    memcpy(params->tExpo_reg_hdr_sef, expo_reg_sef, sizeof(expo_reg_sef));
    memcpy(params->tBsc_reg_hdr_sef, bsc_reg_hdr, sizeof(bsc_reg_hdr));

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "JXF37_MIPI");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    // SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type = SENSOR_ISP_TYPE; // ISP_SOC;
    // handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus = SENSOR_IFBUS_TYPE; // CUS_SENIF_BUS_PARL;
    handle->data_prec = SENSOR_DATAPREC; // CUS_DATAPRECISION_8;
    handle->data_mode = SENSOR_DATAMODE;
    handle->bayer_id = SENSOR_BAYERID; // CUS_BAYER_GB;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->orient = SENSOR_ORIT; // CUS_ORIT_M1F1;
    // handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; // don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_NONE;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 0; // Short frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0;
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT;
    handle->video_res_supported.res[0].max_fps = Preview_MAX_FPS;
    handle->video_res_supported.res[0].min_fps = Preview_MIN_FPS;
    handle->video_res_supported.res[0].crop_start_x = 0; // cch123
    handle->video_res_supported.res[0].crop_start_y = 0;
    handle->video_res_supported.res[0].nOutputWidth = 1920;
    handle->video_res_supported.res[0].nOutputHeight = 1080;
    sprintf(handle->video_res_supported.res[0].strResDesc, "1920x1080@30fps");

    // i2c

    handle->i2c_cfg.mode = SENSOR_I2C_LEGACY; //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt = SENSOR_I2C_FMT; // CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address = SENSOR_I2C_ADDR; // 0x5a;
    handle->i2c_cfg.speed = SENSOR_I2C_SPEED; // 320000;

    // mclk
    handle->mclk = Preview_MCLK_SPEED;

    // polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY = SENSOR_PWDN_POL; // CUS_CLK_POL_NEG;
    handle->reset_POLARITY = SENSOR_RST_POL; // CUS_CLK_POL_NEG;
    handle->VSYNC_POLARITY = SENSOR_VSYNC_POL; // CUS_CLK_POL_POS;
    handle->HSYNC_POLARITY = SENSOR_HSYNC_POL; // CUS_CLK_POL_POS;
    handle->PCLK_POLARITY = SENSOR_PCLK_POL; // CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2;
    handle->ae_shutter_delay = 2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 1;

    /// calibration
    handle->sat_mingain = g_sensor_ae_min_gain;

    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = pCus_init;

    handle->pCus_sensor_poweron = pCus_poweron;
    handle->pCus_sensor_poweroff = pCus_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID = pCus_GetSensorID;

    handle->pCus_sensor_GetVideoResNum = pCus_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes = pCus_GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes = pCus_GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes = pCus_SetVideoRes;

    handle->pCus_sensor_GetOrien = pCus_GetOrien;
    handle->pCus_sensor_SetOrien = pCus_SetOrien;
    handle->pCus_sensor_GetFPS = pCus_GetFPS;
    handle->pCus_sensor_SetFPS = pCus_SetFPS;
    // handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = jxf37_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    // handle->pCus_sensor_GetAETrigger_mode      = pCus_GetAETrigger_mode;
    // handle->pCus_sensor_SetAETrigger_mode      = pCus_SetAETrigger_mode;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs;
    handle->pCus_sensor_GetAEGain = pCus_GetAEGain;
    handle->pCus_sensor_TryAEGain = pCus_TryAEGain; // cch123

    handle->pCus_sensor_SetAEGain = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = pCus_GetAEMinMaxUSecs;

    // sensor calibration
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = jxf37_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    //    params->expo.line = 100;

    params->dirty = false;
    params->orient_dirty = false;
    return SUCCESS;
}

static int cus_camsensor_init_handle_hdr_sef(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    jxf37_params* params = NULL;

    cus_camsensor_init_handle(drv_handle);
    params = (jxf37_params*)handle->private_data;

    // SENSOR_DMSG("[%s] HDR SEF INIT!\n", __FUNCTION__);
    sprintf(handle->model_id, "JXF37_MIPI_HDR_SEF");

    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tBsc_reg, bsc_reg, sizeof(bsc_reg));
    memcpy(params->tExpo_reg_hdr_sef, expo_reg_sef, sizeof(expo_reg_sef));
    memcpy(params->tBsc_reg_hdr_sef, bsc_reg_hdr, sizeof(bsc_reg_hdr));

    handle->bayer_id = SENSOR_BAYERID;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->data_prec = SENSOR_DATAPREC;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM; // hdr_lane_num;
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_DCG;

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0; // default resolution index is 0.
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT; // TBD. Workaround for Sony DOL HDR mode
    handle->video_res_supported.res[0].max_fps = Preview_MAX_FPS_HDR;
    handle->video_res_supported.res[0].min_fps = Preview_MIN_FPS_HDR;
    handle->video_res_supported.res[0].crop_start_x = Preview_CROP_START_X;
    handle->video_res_supported.res[0].crop_start_y = Preview_CROP_START_Y;
    handle->video_res_supported.res[0].nOutputWidth = 1920; // 0x79C;
    handle->video_res_supported.res[0].nOutputHeight = 1080; // 0x449;
    sprintf(handle->video_res_supported.res[0].strResDesc, "1920x1080@15fps_HDR");

    handle->pCus_sensor_SetVideoRes = pCus_SetVideoRes_hdr;

    handle->pCus_sensor_init = pCus_init_hdr;

    handle->pCus_sensor_SetFPS = pCus_SetFPS_hdr_lef;

    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify_hdr_sef;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs_hdr;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs_hdr_sef;
    handle->pCus_sensor_SetAEGain = pCus_SetAEGain;
    handle->pCus_sensor_GetShutterInfo = jxf37_GetShutterInfo_hdr;
    handle->pCus_sensor_GetAEMinMaxUSecs = pCus_GetAEMinMaxUSecs;
    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    params->expo.vts = vts_15fps_hdr;
    params->expo.fps = 15;

    // HDR    params->expo.expo_lines = 673;

#if 1

    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 1; // Short frame

    handle->ae_gain_delay = 2; // SENSOR_GAIN_DELAY_FRAME_COUNT;
    handle->ae_shutter_delay = 2; // SENSOR_SHUTTER_DELAY_FRAME_COUNT_HDR_DOL;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2; // 2;//ccc
    params->dirty = false;
    params->orient_dirty = false;
#endif
    return SUCCESS;
}

static int cus_camsensor_init_handle_hdr_lef(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    jxf37_params* params;

    cus_camsensor_init_handle(drv_handle);
    // SENSOR_DMSG("[%s] HDR LEF INIT!\n", __FUNCTION__);
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    // private data allocation & init
    if (handle->private_data == NULL) {
        SENSOR_EMSG("[%s] Private data is empty!\n", __FUNCTION__);
        return FAIL;
    }
    params = (jxf37_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tBsc_reg, bsc_reg, sizeof(bsc_reg));
    memcpy(params->tExpo_reg_hdr_sef, expo_reg_sef, sizeof(expo_reg_sef));
    memcpy(params->tBsc_reg_hdr_sef, bsc_reg_hdr, sizeof(bsc_reg_hdr));

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "JXF37_MIPI_HDR_LEF");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    // SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type = SENSOR_ISP_TYPE; // ISP_SOC;
    // handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus = SENSOR_IFBUS_TYPE; // CUS_SENIF_BUS_PARL;
    handle->data_prec = SENSOR_DATAPREC; // CUS_DATAPRECISION_8;
    handle->data_mode = SENSOR_DATAMODE;
    handle->bayer_id = SENSOR_BAYERID; // CUS_BAYER_GB;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->orient = SENSOR_ORIT; // CUS_ORIT_M1F1;
    // handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; // don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_DCG;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 0; // Long frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0; // default resolution index is 0.
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT; // TBD. Workaround for Sony DOL HDR mode
    handle->video_res_supported.res[0].max_fps = Preview_MAX_FPS_HDR;
    handle->video_res_supported.res[0].min_fps = Preview_MIN_FPS_HDR;
    handle->video_res_supported.res[0].crop_start_x = Preview_CROP_START_X;
    handle->video_res_supported.res[0].crop_start_y = Preview_CROP_START_Y;
    handle->video_res_supported.res[0].nOutputWidth = 1920;
    handle->video_res_supported.res[0].nOutputHeight = 1080;
    sprintf(handle->video_res_supported.res[0].strResDesc, "1920x1080@15fps_HDR");

    // i2c
    handle->i2c_cfg.mode = SENSOR_I2C_LEGACY; //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt = SENSOR_I2C_FMT; // CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address = SENSOR_I2C_ADDR; // 0x5a;
    handle->i2c_cfg.speed = SENSOR_I2C_SPEED; // 320000;

    // mclk
    handle->mclk = Preview_MCLK_SPEED; // ParaMclk(SENSOR_DRV_PARAM_MCLK());

    // polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY = SENSOR_PWDN_POL; // CUS_CLK_POL_NEG;
    handle->reset_POLARITY = SENSOR_RST_POL; // CUS_CLK_POL_NEG;
    handle->VSYNC_POLARITY = SENSOR_VSYNC_POL; // CUS_CLK_POL_POS;
    handle->HSYNC_POLARITY = SENSOR_HSYNC_POL; // CUS_CLK_POL_POS;
    handle->PCLK_POLARITY = SENSOR_PCLK_POL; // CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2; // 2;//SENSOR_GAIN_DELAY_FRAME_COUNT;
    handle->ae_shutter_delay = 2; // SENSOR_SHUTTER_DELAY_FRAME_COUNT_HDR_DOL;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2; // ccc2;

    /// calibration
    handle->sat_mingain = g_sensor_ae_min_gain; // g_sensor_ae_min_gain;
    // handle->dgain_remainder = 0;

    // LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = pCus_init_hdr;
    // handle->pCus_sensor_powerupseq  = pCus_powerupseq   ;
    handle->pCus_sensor_poweron = pCus_poweron;
    handle->pCus_sensor_poweroff = pCus_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID = pCus_GetSensorID;

    handle->pCus_sensor_GetVideoResNum = NULL;
    handle->pCus_sensor_GetVideoRes = NULL;
    handle->pCus_sensor_GetCurVideoRes = NULL;
    handle->pCus_sensor_SetVideoRes = NULL;

    handle->pCus_sensor_GetShutterInfo = jxf37_GetShutterInfo_hdr;
    handle->pCus_sensor_GetOrien = pCus_GetOrien;
    handle->pCus_sensor_SetOrien = pCus_SetOrien;
    handle->pCus_sensor_GetFPS = pCus_GetFPS;
    handle->pCus_sensor_SetFPS = pCus_SetFPS_hdr_lef;
    // handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap_hdr_dol_lef;
    handle->pCus_sensor_SetPatternMode = jxf37_SetPatternMode; // imx307_SetPatternMode_hdr_dol_lef;
    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = pCus_GetAEMinMaxUSecs;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify_hdr_lef;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs_hdr_lef;
    handle->pCus_sensor_TryAEGain = pCus_TryAEGain; // cch123

    //  handle->pCus_sensor_SetAEGain     = pCus_SetAEGain_hdr_sef;
    params->expo.vts = vts_15fps_hdr;
    params->expo.fps = 15;

    params->dirty = false;
    params->orient_dirty = false;

    //  params->expo.expo_lines = 673;
    //  if (CUS_CMU_CLK_36MHZ == handle->mclk)
    //      params->expo.fps = 29;
    //  else
    //      params->expo.fps = 30;

    //  params->dirty = false;

    // handle->channel_num = SENSOR_CHANNEL_NUM + 1;
    //  handle->channel_mode = SENSOR_CHANNEL_MODE_HDR;

    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    // ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    // sensor_if-> PCLK(NULL,CUS_PCLK_OFF);
    // sensor_if->SetCSI_Clk(handle,CUS_CSI_CLK_DISABLE);
#if 0
    if (handle && handle->private_data) {
        SENSOR_EMSG("[%s] release handle, handle %x, private data %x",
                __FUNCTION__,
                (int)handle,
                (int)handle->private_data);
        CamOsMemRelease(handle->private_data);
        handle->private_data = NULL;
    }
#endif
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(JXF37_HDR,
    cus_camsensor_init_handle,
    cus_camsensor_init_handle_hdr_sef,
    cus_camsensor_init_handle_hdr_lef,
    jxf37_params);
