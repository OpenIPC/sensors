/* Copyright (c) 2018-2019 Sigmastar Technology Corp.
 All rights reserved.

 Unless otherwise stipulated in writing, any and all information contained
herein regardless in any format shall remain the sole proprietary of
Sigmastar Technology Corp. and be kept in strict confidence
(Sigmastar Confidential Information) by the recipient.
Any unauthorized act including without limitation unauthorized disclosure,
copying, use, reproduction, sale, distribution, modification, disassembling,
reverse engineering and compiling of the contents of Sigmastar Confidential
Information is unlawful and strictly prohibited. Sigmastar hereby reserves the
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

SENSOR_DRV_ENTRY_IMPL_BEGIN(gc2083);

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
//  @@@                                                                 //
//  @   @@      ==  S t a r t * H e r e ==                              //
//  @@      ==  S t a r t * H e r e  ==                                 //
//  @@      ==  S t a r t * H e r e  ==                                 //
//  @@@@                                                                //
//                                                                      //
//      Start Step 1 --  show preview on LCM                            //
//                                                                      //
//  Fill these #define value and table with correct settings            //
//      camera can work and show preview on LCM                         //
//                                                                      //
///////////////////////////////////////////////////////////////

#define SENSOR_ISP_TYPE ISP_EXT // ISP_EXT, ISP_SOC
#define F_number 22 // CFG, demo module
//#define SENSOR_DATAFMT      CUS_DATAFMT_BAYER     //CUS_DATAFMT_YUV, CUS_DATAFMT_BAYER
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_MIPI // CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000 // CFG
#define SENSOR_BAYERID CUS_BAYER_RG // CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAX_GAIN 128 // max sensor again, a-gain
//#define SENSOR_YCORDER      CUS_SEN_YCODR_YC      //CUS_SEN_YCODR_YC, CUS_SEN_YCODR_CY
#define lane_number 2
#define vc0_hs_mode 3 // 0: packet header edge  1: line end edge 2: line start edge 3: packet footer edge
#define long_packet_type_enable 0x00 // UD1~UD8 (user define)

#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
//#define Preview_line_period 30000                 ////HTS/PCLK=4455 pixels/148.5MHZ=30usec @MCLK=36MHz
//#define vts_30fps 1125//1346,1616                 //for 29.1fps @ MCLK=36MHz
#define Preview_line_period 29616 // 30580            //(36M/37.125M)*30fps=29.091fps(34.375msec), hts=34.375/1125=30556,
//#define Line_per_second     32727
#define vts_30fps 1125 // 1266//1150//1090                              //for 29.091fps @ MCLK=36MHz
#define Prv_Max_line_number 2200 // maximum exposure line munber of sensor when preview
#define Preview_WIDTH 1920 // resolution Width when preview
#define Preview_HEIGHT 1080 // resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 5 // slowest preview FPS
#define Preview_CROP_START_X 0 // CROP_START_X
#define Preview_CROP_START_Y 0 // CROP_START_Y

#define SENSOR_I2C_ADDR 0x6E // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 300000// 240000 //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A16D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL CUS_CLK_POL_POS // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_NEG // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL CUS_CLK_POL_POS // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG

// int blk_flag=1;
// int times = 0;
// static int  drv_Fnumber = 22;
static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
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
        u32 back_pv_us;
        u32 fps;
        u32 preview_fps;
        u32 line;
    } expo;

    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool dirty;
    bool orient_dirty;
} gc2083_params;
// set sensor ID address and data,

/* typedef struct {
    unsigned int total_gain;
    unsigned short reg_val;
} Gain_ARRAY;
 */
static I2C_ARRAY Sensor_id_table[] = {
    { 0x03f0, 0x20 }, // {address of ID, ID },
    { 0x03f1, 0x83 },
};

typedef struct {
    u32 gain;
    unsigned short again_reg_val_0;
    unsigned short again_reg_val_1;
    unsigned short again_reg_val_2;
    unsigned short again_reg_val_3;
    unsigned short again_reg_val_4;
    unsigned short again_reg_val_5;
    unsigned short again_reg_val_6;
    unsigned short again_reg_val_7;
    unsigned short again_reg_val_8;
    unsigned short again_reg_val_9;
    unsigned short again_reg_val_10;
    unsigned short again_reg_val_11;
    unsigned short again_reg_val_12;
} Gain_ARRAY;

static I2C_ARRAY Sensor_init_table[] = {
    // rowtime=29.61616us
    // fps=30.01364
    // mipi_rate=594Mbps/lane
    // bayer mode:RGGB

    /****system****/
    { 0x03fe, 0xf0 },
    { 0x03fe, 0xf0 },
    { 0x03fe, 0xf0 },
    { 0x03fe, 0x00 },
    { 0x03f2, 0x00 },
    { 0x03f3, 0x00 },
    { 0x03f4, 0x36 },
    { 0x03f5, 0xc0 },
    { 0x03f6, 0x24 },
    { 0x03f7, 0x01 },
    { 0x03f8, 0x2c },
    { 0x03f9, 0x43 },
    { 0x03fc, 0x8e },
    { 0x0381, 0x07 },
    { 0x00d7, 0x29 },

    /****CISCTL & ANALOG****/
    { 0x0d6d, 0x18 },
    { 0x00d5, 0x03 },
    { 0x0082, 0x01 },
    { 0x0db3, 0xd4 },
    { 0x0db0, 0x0d },
    { 0x0db5, 0x96 },

    { 0x0d03, 0x02 },
    { 0x0d04, 0x02 },
    { 0x0d05, 0x05 },
    { 0x0d06, 0xba },
    { 0x0d07, 0x00 },
    { 0x0d08, 0x11 },
    { 0x0d09, 0x00 },
    { 0x0d0a, 0x02 },
    { 0x000b, 0x00 },
    { 0x000c, 0x02 },
    { 0x0d0d, 0x04 },
    { 0x0d0e, 0x40 },
    { 0x000f, 0x07 },
    { 0x0010, 0x90 },
    { 0x0017, 0x0c },
    { 0x0d73, 0x92 },
    { 0x0076, 0x00 },
    { 0x0d76, 0x00 },
    { 0x0d41, 0x04 },
    { 0x0d42, 0x65 },
    { 0x0d7a, 0x10 },
    { 0x0d19, 0x31 },
    { 0x0d25, 0xcb },
    { 0x0d20, 0x60 },
    { 0x0d27, 0x03 },
    { 0x0d29, 0x60 },
    { 0x0d43, 0x10 },
    { 0x0d49, 0x10 },
    { 0x0d55, 0x18 },
    { 0x0dc2, 0x44 },
    { 0x0058, 0x3c },
    { 0x00d8, 0x68 },
    { 0x00d9, 0x14 },
    { 0x00da, 0xc1 },
    { 0x0050, 0x18 },
    { 0x0db6, 0x3d },
    { 0x00d2, 0xbc },
    { 0x0d66, 0x42 },
    { 0x008c, 0x07 },
    { 0x008d, 0xff },

    /*gain*/
    { 0x007a, 0x50 }, // global gain
    { 0x00d0, 0x00 },
    { 0x0dc1, 0x00 },

    /*isp*/
    { 0x0102, 0xa9 }, // 89
    { 0x0158, 0x00 },
    { 0x0107, 0xa6 },
    { 0x0108, 0xa9 },
    { 0x0109, 0xa8 },
    { 0x010a, 0xa7 },
    { 0x010b, 0xff },
    { 0x010c, 0xff },

    { 0x0428, 0x86 }, // 84
    { 0x0429, 0x86 }, // 84
    { 0x042a, 0x86 }, // 84
    { 0x042b, 0x68 }, // 84
    { 0x042c, 0x68 }, // 84
    { 0x042d, 0x68 }, // 84
    { 0x042e, 0x68 }, // 83
    { 0x042f, 0x68 }, // 82

    { 0x0430, 0x4f }, // 82
    { 0x0431, 0x68 }, // 82
    { 0x0432, 0x67 }, // 82
    { 0x0433, 0x66 }, // 82
    { 0x0434, 0x66 }, // 82
    { 0x0435, 0x66 }, // 82
    { 0x0436, 0x66 }, // 64
    { 0x0437, 0x66 }, // 68

    { 0x0438, 0x62 },
    { 0x0439, 0x62 },
    { 0x043a, 0x62 },
    { 0x043b, 0x62 },
    { 0x043c, 0x62 },
    { 0x043d, 0x62 },
    { 0x043e, 0x62 },
    { 0x043f, 0x62 },

    /*dark sun*/
    { 0x0077, 0x01 }, // 01 settle_en
    { 0x0078, 0x65 }, // settle_exp_th
    { 0x0079, 0x04 }, // settle_exp_th
    { 0x0067, 0xa0 }, // settle_ref_th
    { 0x0054, 0xff }, // settle_sig_th
    { 0x0055, 0x02 }, // settle_sig_th
    { 0x0056, 0x00 }, // settle_colgain_th
    { 0x0057, 0x04 }, // settle_colgain_th
    { 0x005a, 0xff }, // settle_data_value
    { 0x005b, 0x07 }, // settle_data_value

    /*blk*/
    { 0x0026, 0x01 },
    { 0x0152, 0x02 },
    { 0x0153, 0x50 },
    { 0x0155, 0x93 },
    { 0x0410, 0x16 },
    { 0x0411, 0x16 },
    { 0x0412, 0x16 },
    { 0x0413, 0x16 },
    { 0x0414, 0x6f },
    { 0x0415, 0x6f },
    { 0x0416, 0x6f },
    { 0x0417, 0x6f },
    { 0x04e0, 0x18 },

    /*window*/
    { 0x0192, 0x04 },
    { 0x0194, 0x04 },
    { 0x0195, 0x04 },
    { 0x0196, 0x38 },
    { 0x0197, 0x07 },
    { 0x0198, 0x80 },

    /****DVP & MIPI****/
    { 0x0201, 0x27 }, // 20//27[6:5]clkctr [2]phy-lane1_en [1]phy-lane0_en [0]phy_clk_en
    { 0x0202, 0x53 }, // 56//[7:6]data1ctr [5:4]data0ctr [3:0]mipi_diff
    { 0x0203, 0xce }, // b2//b6[7]clklane_p2s_sel [6:5]data0hs_ph [4]data0_delay1s [3]clkdelay1s [2]mipi_en [1:0]clkhs_ph
    { 0x0204, 0x40 },
    { 0x0212, 0x07 },
    { 0x0213, 0x80 }, // LWC
    { 0x0215, 0x12 }, //[1:0]clk_lane_mode
    { 0x0229, 0x05 },
    { 0x0237, 0x03 },
    { 0x023e, 0x99 }, // 91//40//91[7]lane_ena [6]DVPBUF_ena [5]ULPEna [4]MIPI_ena [3]mipi_set_auto_disable [2]RAW8_mode [1]ine_sync_mode [0]double_lane_enff  03fe  00
};

I2C_ARRAY TriggerStartTbl[] = {
    //  {0x30f4,0x00},//Master mode start
};

I2C_ARRAY PatternTbl[] = {
    //  pattern mode
};

I2C_ARRAY Current_Mirror_Flip_Tbl[] = {
    { 0x0015, 0x00 }, // bit[1:0]
    { 0x0d15, 0x00 }, // bit[1:0]
};

/////////////////////////////////////////////////////////////////
//  @@@@@@                                                           //
//  @@                                                               //
//  @@@                                                              //
//  @       @@                                                       //
//  @@@@                                                             //
//                                                                   //
//  Step 3 --  complete camera features                              //
//                                                                   //
//                                                                   //
//  camera set EV, MWB, orientation, contrast, sharpness             //
//   , saturation, and Denoise can work correctly.                   //
//                                                                   //
/////////////////////////////////////////////////////////////////

static I2C_ARRAY mirr_flip_table[] = {
    { 0x0015, 0x00 }, // bit[1:0]
    { 0x0d15, 0x00 }, // bit[1:0]
    //  {0x0192, 0x02},    // bit[1:0]
    //	{0x0194, 0x02},    // bit[1:0]

    { 0x0015, 0x01 }, // bit[1:0]
    { 0x0d15, 0x01 }, // bit[1:0]
    //  {0x0192, 0x02},    // bit[1:0]
    //	{0x0194, 0x04},    // bit[1:0]

    { 0x0015, 0x02 }, // bit[1:0]
    { 0x0d15, 0x02 }, // bit[1:0]
    //  {0x0192, 0x03},    // bit[1:0]
    //	{0x0194, 0x02},    // bit[1:0]

    { 0x0015, 0x03 }, // bit[1:0]
    { 0x0d15, 0x03 }, // bit[1:0]
    //  {0x0192, 0x03},    // bit[1:0]
    //	{0x0194, 0x04},    // bit[1:0]

};

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

static int g_sensor_ae_min_gain = 1024; // 1280;

static I2C_ARRAY expo_reg[] = {
    { 0x0d04, 0xd0 },
    { 0x0d03, 0x05 },
};

static I2C_ARRAY vts_reg[] = {
    { 0x0d41, 0x04 },
    { 0x0d42, 0x65 },

};

static I2C_ARRAY gain_reg[] = {
    { 0x00d0, 0x00 },
    { 0x031d, 0x2e },
    { 0x0dc1, 0x00 },
    { 0x031d, 0x28 },
    { 0x00b8, 0x01 },
    { 0x00b9, 0x00 },
    { 0x0155, 0x03 },

    { 0x0410, 0x16 },
    { 0x0411, 0x16 },
    { 0x0412, 0x16 },
    { 0x0413, 0x16 },
    { 0x0414, 0x6f },
    { 0x0415, 0x6f },
    { 0x0416, 0x6f },
    { 0x0417, 0x6f },

    { 0x00b1, 0x01 },
    { 0x00b2, 0x00 },

};

static Gain_ARRAY gain_table[] = {
    // 0x00d0 0x0dc1 0x00b8 0x00b9 0x0155 0x0410 0x0411 0x0412 0x0413 0x0414 0x0415 0x0416 0x0417
    { 1024, 0x00, 0x00, 0x01, 0x00, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 1216, 0x10, 0x00, 0x01, 0x0c, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 1440, 0x01, 0x00, 0x01, 0x1a, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 1712, 0x11, 0x00, 0x01, 0x2b, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 2048, 0x02, 0x00, 0x02, 0x00, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 2432, 0x12, 0x00, 0x02, 0x18, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 2864, 0x03, 0x00, 0x02, 0x33, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 3408, 0x13, 0x00, 0x03, 0x15, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 4096, 0x04, 0x00, 0x04, 0x00, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 4880, 0x14, 0x00, 0x04, 0xe0, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 5728, 0x05, 0x00, 0x05, 0x26, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 6832, 0x15, 0x00, 0x06, 0x2b, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 8192, 0x44, 0x00, 0x08, 0x00, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 9760, 0x54, 0x00, 0x09, 0x22, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 11472, 0x45, 0x00, 0x0b, 0x0d, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 13664, 0x55, 0x00, 0x0d, 0x16, 0x03, 0x11, 0x11, 0x11, 0x11, 0x6f, 0x6f, 0x6f, 0x6f },
    { 16384, 0x04, 0x01, 0x10, 0x00, 0x19, 0x16, 0x16, 0x16, 0x16, 0x6f, 0x6f, 0x6f, 0x6f },
    { 19520, 0x14, 0x01, 0x13, 0x04, 0x19, 0x16, 0x16, 0x16, 0x16, 0x6f, 0x6f, 0x6f, 0x6f },
    { 22944, 0x24, 0x01, 0x16, 0x1a, 0x19, 0x16, 0x16, 0x16, 0x16, 0x6f, 0x6f, 0x6f, 0x6f },
    { 27312, 0x34, 0x01, 0x1a, 0x2b, 0x19, 0x16, 0x16, 0x16, 0x16, 0x6f, 0x6f, 0x6f, 0x6f },
    { 32768, 0x44, 0x01, 0x20, 0x00, 0x36, 0x18, 0x18, 0x18, 0x18, 0x6f, 0x6f, 0x6f, 0x6f },
    { 39024, 0x54, 0x01, 0x26, 0x07, 0x36, 0x18, 0x18, 0x18, 0x18, 0x6f, 0x6f, 0x6f, 0x6f },
    { 45872, 0x64, 0x01, 0x2c, 0x33, 0x36, 0x18, 0x18, 0x18, 0x18, 0x6f, 0x6f, 0x6f, 0x6f },
    { 54640, 0x74, 0x01, 0x35, 0x17, 0x36, 0x18, 0x18, 0x18, 0x18, 0x6f, 0x6f, 0x6f, 0x6f },
    { 65536, 0x84, 0x01, 0x35, 0x17, 0x64, 0x16, 0x16, 0x16, 0x16, 0x72, 0x72, 0x72, 0x72 },
    { 78048, 0x94, 0x01, 0x35, 0x17, 0x64, 0x16, 0x16, 0x16, 0x16, 0x72, 0x72, 0x72, 0x72 },
    { 91744, 0x85, 0x01, 0x35, 0x17, 0x64, 0x16, 0x16, 0x16, 0x16, 0x72, 0x72, 0x72, 0x72 },
    { 109280, 0x95, 0x01, 0x35, 0x17, 0x64, 0x16, 0x16, 0x16, 0x16, 0x72, 0x72, 0x72, 0x72 },
    { 128448, 0xa5, 0x01, 0x35, 0x17, 0x64, 0x16, 0x16, 0x16, 0x16, 0x72, 0x72, 0x72, 0x72 },
};

#if 0
static CUS_INT_TASK_ORDER def_order = {
    .RunLength = 9,
    .Orders = {
        CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
    },
};
#endif

/////////// function definition ///////////////////
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME gc2083

#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

static int cus_camsensor_release_handle(ms_cus_sensor* handle);

/////////////////// sensor hardware dependent //////////////
#if 0
static int ISP_config_io(ms_cus_sensor *handle) {
    ISensorIfAPI *sensor_if = handle->sensor_if_api;

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
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    // TStart = timeGetTimeU();
    /*PAD and CSI*/
    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M); ///???
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1); ///???
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0); //=========   ????

    /*Power ON*/
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);

    /*Reset PIN*/
    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(1000);

    /*MCLK ON*/
    sensor_if->MCLK(idx, 1, handle->mclk);
    SENSOR_USLEEP(5000);
    // CamOsPrintf("pCus_poweron = %d us \n",timeGetTimeU()-TStart);
    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    // handle->i2c_bus->i2c_close(handle->i2c_bus);
    SENSOR_USLEEP(1000);
    // Set_csi_if (0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);

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

    for (n = 0; n < table_length; ++n) {
        id_from_sensor[n].reg = Sensor_id_table[n].reg;
        id_from_sensor[n].data = 0;
    }

    *id = 0;
    if (table_length > 8)
        table_length = 8;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

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
        *id = ((*id) + id_from_sensor[i].data) << 8;
    }

    *id >>= 8;
    SENSOR_DMSG("[%s]gc2083 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    // SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    return SUCCESS;
}

static int gc2083_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{
    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);
    return SUCCESS;
}

static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init(ms_cus_sensor* handle)
{
    int i, cnt = 0;
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

    /*
        for (i=0;i< ARRAY_SIZE(PatternTbl);i++)
        {
            if (SensorReg_Write(PatternTbl[i].reg,PatternTbl[i].data) != SUCCESS)
            {
                //MSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                return FAIL;
            }
        }
    */

    // pCus_SetAEGain(handle,1024); //Set sensor gain = 1x
    // pCus_SetAEUSecs(handle, 30000);
    // pCus_AEStatusNotify(handle,CUS_FRAME_ACTIVE);

    // CamOsPrintf("pCus_init = %d us \n",timeGetTimeU()-TStart);

    return SUCCESS;
}
/*
int pCus_release(ms_cus_sensor *handle)
{
    ISensorIfAPI *sensor_if = handle->sensor_if_api;
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
    short HFlip, VFlip;
    SensorReg_Read(0x0015, &HFlip);
    SensorReg_Read(0x0d15, &VFlip);

    if (((HFlip & 0x01) == 0) && ((VFlip & 0x02) == 0)) //&&((VFlip&0x80)==0))
        *orit = CUS_ORIT_M0F0;
    else if (((HFlip & 0x01) == 1) && ((VFlip & 0x01) == 1)) //&&((VFlip&0x80)==0))
        *orit = CUS_ORIT_M1F0;
    else if (((HFlip & 0x02) == 2) && ((VFlip & 0x02) == 2)) //&&((VFlip&0x80)!=0))
        *orit = CUS_ORIT_M0F1;
    else if (((HFlip & 0x03) == 3) && ((VFlip & 0x03) == 3)) //&&((VFlip&0x80)!=0))
        *orit = CUS_ORIT_M1F1;

    // SENSOR_DMSG("mirror:%x\r\n", HFlip&0x80);
    // SENSOR_DMSG("Flip:%x\r\n", VFlip&0x80);

    return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    // gc2083_params *params = (gc2083_params *)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    switch (orit) {
    case CUS_ORIT_M0F0:
        SensorReg_Write(mirr_flip_table[0].reg, mirr_flip_table[0].data);
        SensorReg_Write(mirr_flip_table[1].reg, mirr_flip_table[1].data);
        //        SensorReg_Write(mirr_flip_table[2].reg,mirr_flip_table[2].data);
        //		SensorReg_Write(mirr_flip_table[3].reg,mirr_flip_table[3].data);
        Current_Mirror_Flip_Tbl[0].reg = mirr_flip_table[0].reg;
        Current_Mirror_Flip_Tbl[0].data = mirr_flip_table[0].data;
        Current_Mirror_Flip_Tbl[1].reg = mirr_flip_table[1].reg;
        Current_Mirror_Flip_Tbl[1].data = mirr_flip_table[1].data;
        //      handle->bayer_id=  CUS_BAYER_BG;
        break;
    case CUS_ORIT_M1F0:
        SensorReg_Write(mirr_flip_table[2].reg, mirr_flip_table[2].data);
        SensorReg_Write(mirr_flip_table[3].reg, mirr_flip_table[3].data);
        //		SensorReg_Write(mirr_flip_table[6].reg,mirr_flip_table[6].data);
        //        SensorReg_Write(mirr_flip_table[7].reg,mirr_flip_table[7].data);
        Current_Mirror_Flip_Tbl[0].reg = mirr_flip_table[2].reg;
        Current_Mirror_Flip_Tbl[0].data = mirr_flip_table[2].data;
        Current_Mirror_Flip_Tbl[1].reg = mirr_flip_table[3].reg;
        Current_Mirror_Flip_Tbl[1].data = mirr_flip_table[3].data;
        //      handle->bayer_id= CUS_BAYER_BG;
        break;
    case CUS_ORIT_M0F1:
        SensorReg_Write(mirr_flip_table[4].reg, mirr_flip_table[4].data);
        SensorReg_Write(mirr_flip_table[5].reg, mirr_flip_table[5].data);
        //        SensorReg_Write(mirr_flip_table[10].reg,mirr_flip_table[10].data);
        //       SensorReg_Write(mirr_flip_table[11].reg,mirr_flip_table[11].data);
        Current_Mirror_Flip_Tbl[0].reg = mirr_flip_table[4].reg;
        Current_Mirror_Flip_Tbl[0].data = mirr_flip_table[4].data;
        Current_Mirror_Flip_Tbl[1].reg = mirr_flip_table[5].reg;
        Current_Mirror_Flip_Tbl[1].data = mirr_flip_table[5].data;
        //     handle->bayer_id= CUS_BAYER_GR;
        break;
    case CUS_ORIT_M1F1:

        SensorReg_Write(mirr_flip_table[6].reg, mirr_flip_table[6].data);
        SensorReg_Write(mirr_flip_table[7].reg, mirr_flip_table[7].data);
        //        SensorReg_Write(mirr_flip_table[14].reg,mirr_flip_table[14].data);
        //		SensorReg_Write(mirr_flip_table[15].reg,mirr_flip_table[15].data);
        Current_Mirror_Flip_Tbl[0].reg = mirr_flip_table[6].reg;
        Current_Mirror_Flip_Tbl[0].data = mirr_flip_table[6].data;
        Current_Mirror_Flip_Tbl[1].reg = mirr_flip_table[7].reg;
        Current_Mirror_Flip_Tbl[1].data = mirr_flip_table[7].data;
        //     handle->bayer_id= CUS_BAYER_GR;
        break;
    default:

        SensorReg_Write(mirr_flip_table[0].reg, mirr_flip_table[0].data);
        SensorReg_Write(mirr_flip_table[1].reg, mirr_flip_table[1].data);
        //        SensorReg_Write(mirr_flip_table[2].reg,mirr_flip_table[2].data);
        //		SensorReg_Write(mirr_flip_table[3].reg,mirr_flip_table[3].data);
        Current_Mirror_Flip_Tbl[0].reg = mirr_flip_table[0].reg;
        Current_Mirror_Flip_Tbl[0].data = mirr_flip_table[0].data;
        Current_Mirror_Flip_Tbl[1].reg = mirr_flip_table[1].reg;
        Current_Mirror_Flip_Tbl[1].data = mirr_flip_table[1].data;
        //      handle->bayer_id= CUS_BAYER_BG;
        break;
    }
    //  SensorReg_Write(0xef,0x01);
    //  SensorReg_Write(0x09,1);

    //  params->orient_dirty = true;
    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor* handle)
{
    gc2083_params* params = (gc2083_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = vts_reg[0].data << 8 | vts_reg[1].data;

    if (params->expo.fps >= 5000)
        params->expo.preview_fps = (vts_30fps * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    int vts = 0;
    gc2083_params* params = (gc2083_params*)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

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
    vts_reg[0].data = (vts >> 8) & 0x003f;
    vts_reg[1].data = (vts >> 0) & 0x00ff;

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
    gc2083_params* params = (gc2083_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:
        break;
    case CUS_FRAME_ACTIVE:
        if (params->dirty) {
            SensorRegArrayW((I2C_ARRAY*)gain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)expo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)vts_reg, ARRAY_SIZE(vts_reg));
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
    //  gc2083_params *params = (gc2083_params *)handle->private_data;
    u32 lines = 0;

    lines = (u32)(expo_reg[0].data & 0xff);
    lines |= (u32)(expo_reg[1].data & 0x3f) << 8;

    *us = (lines * Preview_line_period) / 1000;

    SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);

    return SUCCESS;
}

static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    gc2083_params* params = (gc2083_params*)handle->private_data;

    lines = (1000 * us) / Preview_line_period;

    if (lines < 1)
        lines = 1;
    if (lines > params->expo.vts - 4) {
        vts = lines + 4;
    } else
        vts = params->expo.vts;

    params->expo.line = lines;
    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);

    expo_reg[0].data = (lines)&0x00ff;
    expo_reg[1].data = (lines >> 8) & 0x003f;
    vts_reg[0].data = (vts >> 8) & 0x003f;
    vts_reg[1].data = (vts >> 0) & 0x00ff;

    params->dirty = true;
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    //  gc2083_params *params = (gc2083_params *)handle->private_data;

    u32 Again_0 = 1, Dgain_0 = 1;
    u8 i;
    u32 Fine_again = 1024;

    for (i = 0; i < sizeof(gain_table) / sizeof(Gain_ARRAY); i++) {
        if ((gain_table[i].again_reg_val_0 == gain_reg[0].data)) {
            Fine_again = gain_table[i].gain;
            break;
        }
    }

    Dgain_0 = (gain_reg[12].data & 0xf) * 64 + (gain_reg[13].data >> 2);
    Again_0 = Fine_again;
    *gain = (u32)((Again_0 * Dgain_0) / 64);

    return SUCCESS;
}

static u8 total = sizeof(gain_table) / sizeof(Gain_ARRAY);
// static u8 temperature_flag=0;
static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain)
{

    gc2083_params* params = (gc2083_params*)handle->private_data;
    u32 dgain = 1;
    u8 dgain_0 = 1, dgain_1 = 0;
    u8 i = 0, tmp = 0;
    params->expo.final_gain = gain;

#if 0	
    short temperature_value;
    SensorReg_Read(0x040c,&temperature_value);

       if(temperature_value >= 0x1a)
       {
		  
		  
		//	total= sizeof(gain_table)/sizeof(Gain_ARRAY);
		//	temperature_flag=0;
		//	SensorReg_Write(0x0171,0x40);
		//	SensorReg_Write(0x0172,0x40);   
		if(gain<(24*1024))
		{
			temperature_flag=1;
			SensorReg_Write(0x0171,0x3c);
			SensorReg_Write(0x0172,0x3d);

		}
		else if(((24*1024) <= gain ) &&( gain< (64*1024)))
        {	
			temperature_flag=2;
			SensorReg_Write(0x0171,0x39);
			SensorReg_Write(0x0172,0x3a);
        }
		else if(((64*1024) <= gain ) &&( gain < (80*1024)))
        {	
			temperature_flag=3;
			SensorReg_Write(0x0171,0x37);
			SensorReg_Write(0x0172,0x39);
        }
		else if(((80*1024)<= gain ) &&(gain < (105*1024)))
        {	
			temperature_flag=4;
			SensorReg_Write(0x0171,0x35);
			SensorReg_Write(0x0172,0x36);
        }
		else if(((105*1024) <= gain) &&(gain < (120*1024)))
        {	
			temperature_flag=5;
			SensorReg_Write(0x0171,0x30);
			SensorReg_Write(0x0172,0x32);
        }
		else if(gain>= (120*1024))
        {	
			temperature_flag=6;
			SensorReg_Write(0x0171,0x2a);
			SensorReg_Write(0x0172,0x30);
        }
		if(temperature_value>0x28)

		{			
		total=16; 
		}
		else total=23;
       }
	   
		
       if(temperature_value < 0x0e)
       {
            total= sizeof(gain_table)/sizeof(Gain_ARRAY);
			temperature_flag=0;
			SensorReg_Write(0x0171,0x40);
			SensorReg_Write(0x0172,0x40); 
		
       }
#endif

    if (gain < 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAX_GAIN * 1024) {
        gain = SENSOR_MAX_GAIN * 1024;
    }

    for (i = 0; i < total - 1; i++) {
        if ((gain >= gain_table[i].gain) && (gain < gain_table[i + 1].gain)) {
            tmp = i;
            break;
        } else {
            tmp = total - 1;
        }
    }
    //	printk("temperature_value = %x, temperature_flag = %d \n",temperature_value, temperature_flag);
    dgain = (gain * 64) / (gain_table[tmp].gain);
    if (dgain > 16 * 64)
        dgain = 16 * 64 - 1;
    dgain_0 = (dgain) >> 6;
    dgain_1 = (dgain & 0x3f) << 2;
    gain_reg[0].data = gain_table[tmp].again_reg_val_0;
    gain_reg[1].data = 0x2e;
    gain_reg[2].data = gain_table[tmp].again_reg_val_1;
    gain_reg[3].data = 0x28;

    gain_reg[4].data = gain_table[tmp].again_reg_val_2;
    gain_reg[5].data = gain_table[tmp].again_reg_val_3;
    gain_reg[6].data = gain_table[tmp].again_reg_val_4;
    gain_reg[7].data = gain_table[tmp].again_reg_val_5;
    gain_reg[8].data = gain_table[tmp].again_reg_val_6;
    gain_reg[9].data = gain_table[tmp].again_reg_val_7;
    gain_reg[10].data = gain_table[tmp].again_reg_val_8;
    gain_reg[11].data = gain_table[tmp].again_reg_val_9;
    gain_reg[12].data = gain_table[tmp].again_reg_val_10;
    gain_reg[13].data = gain_table[tmp].again_reg_val_11;
    gain_reg[14].data = gain_table[tmp].again_reg_val_12;

    gain_reg[15].data = dgain_0;
    gain_reg[16].data = dgain_1;

    SENSOR_DMSG("[%s] set gain/regH/regL=%d/0x%x/0x%x\n", __FUNCTION__, gain, gain_reg[2].data, gain_reg[3].data);

    params->dirty = true;
    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1; // 30
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = handle->sat_mingain;
    *max = SENSOR_MAX_GAIN * 1024;
    return SUCCESS;
}

static int gc2083_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period * 1; // 2
    info->step = Preview_line_period * 1; // 2
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    //  u32 i, j;
    /*
        for (i=0,j=0;i< num ;i++,j+=2){
            gain_gap_compensate[i].gain=pArray[i].gain;
            gain_gap_compensate[i].offset=pArray[i].offset;
        }
        //SENSOR_DMSG("[%s]%d, %d, %d, %d\n", __FUNCTION__, num, pArray[0].gain, pArray[1].gain, pArray[num-1].offset);
    */
    return SUCCESS;
}

static int cus_camsensor_init_handle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    gc2083_params* params;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    // private data allocation & init
    handle->private_data = CamOsMemCalloc(1, sizeof(gc2083_params));
    params = (gc2083_params*)handle->private_data;

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    strcpy(handle->model_id, "gc2083_MIPI");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    // SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type = SENSOR_ISP_TYPE; // ISP_SOC;
    //  handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus = SENSOR_IFBUS_TYPE; // CUS_SENIF_BUS_PARL;
    handle->data_prec = SENSOR_DATAPREC; // CUS_DATAPRECISION_8;
    handle->data_mode = SENSOR_DATAMODE;
    handle->bayer_id = SENSOR_BAYERID; // CUS_BAYER_GB;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->orient = SENSOR_ORIT; // CUS_ORIT_M1F1;
    //  handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
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
    handle->video_res_supported.res[0].crop_start_x = 0;
    handle->video_res_supported.res[0].crop_start_y = 0;
    handle->video_res_supported.res[0].nOutputWidth = 1920;
    handle->video_res_supported.res[0].nOutputHeight = 1080;
    SENSOR_DMSG(handle->video_res_supported.res[0].strResDesc, "1920x1080@30fps");

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
    //  handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = gc2083_SetPatternMode;
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

    handle->pCus_sensor_SetAEGain = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = pCus_GetAEMinMaxUSecs;

    // sensor calibration
    //  handle->pCus_sensor_SetAEGain_cal   = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = gc2083_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.line = 100;
    params->dirty = false;
    params->orient_dirty = false;
    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    // ISensorIfAPI *sensor_if = handle->sensor_if_api;
    // sensor_if->PCLK(NULL,CUS_PCLK_OFF);
    // sensor_if->SetCSI_Clk(handle,CUS_CSI_CLK_DISABLE);
    if (handle && handle->private_data) {
        SENSOR_DMSG("[%s] release handle, handle %x, private data %x",
            __FUNCTION__,
            (int)handle,
            (int)handle->private_data);
        CamOsMemRelease(handle->private_data);
        handle->private_data = NULL;
    }
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END(gc2083,
    cus_camsensor_init_handle,
    NULL,
    NULL);
