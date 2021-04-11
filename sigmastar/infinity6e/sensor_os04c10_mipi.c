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

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(OS04c10_HDR);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE CAM_OS_ARRAY_SIZE
#endif

#define SENSOR_PAD_GROUP_SET CUS_SENSOR_PAD_GROUP_A
#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
// MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (4)
#define SENSOR_MIPI_LANE_NUM_HDR (4)
#define SENSOR_MIPI_HDR_MODE (2) // 0: Non-HDR mode. 1:Sony DOL mode
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
//                                                                                                    �@//
//  Fill these #define value and table with correct settings                        //
//      camera can work and show preview on LCM                                 //
//                                                                                                       //
///////////////////////////////////////////////////////////////

#define SENSOR_ISP_TYPE ISP_EXT // ISP_EXT, ISP_SOC
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_MIPI // CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_MIPI_HSYNC_MODE_HDR PACKET_FOOTER_EDGE
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAPREC_HDR CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000 // CFG
#define SENSOR_BAYERID CUS_BAYER_BG // CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_BAYERID_HDR CUS_BAYER_BG // CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,

#define SENSOR_MAX_GAIN 246 * 1024 //(15.5*15.9)                  // max sensor again, a-gain * conversion-gain*d-gain
#define SENSOR_MIN_GAIN 1024 //(15.5*15.9)                  // max sensor again, a-gain * conversion-gain*d-gain

#define Preview_MCLK_SPEED CUS_CMU_CLK_24MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_MCLK_SPEED_HDR CUS_CMU_CLK_24MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_line_period 21177 // 17814                           // MCLK=21.6 HTS/PCLK=3080 pixels/97.2MHZ=31.687us                              // 3126 for 25fps
#define Preview_line_period_DCG 21177 // 19817//17814                     // MCLK=21.6 HTS/PCLK=3080 pixels/97.2MHZ=31.687us                              // 3126 for 25fps
#define vts_30fps 1574 // 1770                               // VTS for 20fps
#define vts_30fps_DCG 1574 // 1682//3000                     // VTS for 20fps

#define Preview_WIDTH 2560 // 2688                    //resolution Width when preview
#define Preview_HEIGHT 1440 // 1520                    //resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 5 // slowest preview FPS

#define SENSOR_I2C_ADDR 0x6c // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A16D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL CUS_CLK_POL_NEG // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_POS // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL CUS_CLK_POL_POS // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG

#if defined(SENSOR_MODULE_VERSION)
#define TO_STR_NATIVE(e) #e
#define TO_STR_PROXY(m, e) m(e)
#define MACRO_TO_STRING(e) TO_STR_PROXY(TO_STR_NATIVE, e)
static char* sensor_module_version = MACRO_TO_STRING(SENSOR_MODULE_VERSION);
module_param(sensor_module_version, charp, S_IRUGO);
#endif
static int cus_camsensor_release_handle(ms_cus_sensor* handle);
static int OS04c10_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int OS04c10_SetAEUSecs(ms_cus_sensor* handle, u32 us);
static int OS04c10_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit);
static int OS04c10_SetAEUSecs_HDR_lef(ms_cus_sensor* handle, u32 us);
static int OS04c10_SetAEUSecs_HDR_sef(ms_cus_sensor* handle, u32 us);
static long int framecount = 0;
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
        float sclk;
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
        u32 lines;
        u32 max_short;
    } expo;
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool mirror_dirty;
    bool dirty;
    bool mwb_dirty;
    I2C_ARRAY tVts_reg[2];
    I2C_ARRAY tGain_reg[4];
    I2C_ARRAY tExpo_reg[4];
    I2C_ARRAY tMirror_reg[2];
    I2C_ARRAY tGain_vc1_reg[4];
    I2C_ARRAY tExpo_vc0_reg[2];
    I2C_ARRAY tExpo_vc1_reg[2];
    I2C_ARRAY tMWB_HDR_reg[4];
    CUS_CAMSENSOR_ORIT cur_orien;
} os04c10_params;

// set sensor ID address and data,
const static I2C_ARRAY Sensor_id_table[] = {
    { 0x300a, 0x53 }, // {address of ID, ID },
    { 0x300b, 0x08 }, // {address of ID, ID },
    //{0x300c, 0x41},      // {address of ID, ID },
    // max 8 sets in this table
};

////////////////////////////////////
// Image Info                     //
////////////////////////////////////
static struct { // LINEAR
    // Modify it based on number of support resolution
    enum { LINEAR_RES_1 = 0,
        LINEAR_RES_END } mode;
    // Sensor Output Image info
    struct _senout {
        s32 width, height, min_fps, max_fps;
    } senout;
    // VIF Get Image Info
    struct _sensif {
        s32 crop_start_X, crop_start_y, preview_w, preview_h;
    } senif;
    // Show resolution string
    struct _senstr {
        const char* strResDesc;
    } senstr;
} os04c10_mipi_linear[] = {
    { LINEAR_RES_1, { 2568, 1448, 3, 30 }, { 4, 4, 2560, 1440 }, { "2560x1440@30fps" } },
};

static struct { // HDR
    // Modify it based on number of support resolution
    enum { HDR_RES_1 = 0,
        HDR_RES_END } mode;
    // Sensor Output Image info
    struct _hsenout {
        s32 width, height, min_fps, max_fps;
    } senout;
    // VIF Get Image Info
    struct _hsensif {
        s32 crop_start_X, crop_start_y, preview_w, preview_h;
    } senif;
    // Show resolution string
    struct _hsenstr {
        const char* strResDesc;
    } senstr;
} os04c10_mipi_hdr[] = {
    { HDR_RES_1, { 2568, 1448, 3, 30 }, { 4, 4, 2560, 1440 }, { "2560x1440@30fps_HDR" } }, // Modify it
};

const static I2C_ARRAY Sensor_init_table_DCG_3p6m30[] = {
    { 0x3018, 0xf8 },
    { 0x0100, 0x01 },
    { 0x0103, 0x01 },
    { 0x0301, 0x84 },
    { 0x0303, 0x01 },
    { 0x0305, 0x5b },
    { 0x0306, 0x01 },
    { 0x0307, 0x17 },
    { 0x0323, 0x04 },
    { 0x0324, 0x01 },
    { 0x0325, 0x62 },
    { 0x3016, 0x72 },
    { 0x3021, 0x03 },
    { 0x3106, 0x21 },
    { 0x3107, 0xa1 },

    // new add 0922 for hdr low expo MWB
    { 0x320d, 0x11 }, // Bit[1:0]:1, return to group 1, Bit[4]:auto switch enable
    { 0x3209, 0x01 }, // Bit[7:0]:2, stay 2 frames in group 0
    { 0x320a, 0x01 }, // Bit[7:0]:3, stay 3 frames in group 1
    { 0x320b, 0x00 }, // Bit[7:0]:0, do not stay in group 2
    { 0x320c, 0x00 }, // Bit[7:0]:0, do not stay in group 3

    { 0x3500, 0x00 },
    { 0x3501, 0x04 },
    { 0x3502, 0x08 },
    { 0x3503, 0x88 },
    { 0x3508, 0x00 },
    { 0x3509, 0x80 },
    { 0x350a, 0x04 },
    { 0x350b, 0x00 },
    { 0x350c, 0x00 },
    { 0x350d, 0x80 },
    { 0x350e, 0x04 },
    { 0x350f, 0x00 },
    { 0x3510, 0x00 },
    { 0x3511, 0x00 },
    { 0x3512, 0x20 },
    { 0x3624, 0x00 }, // 20210421 sandy avoid vertical dot line
    { 0x3625, 0x4c },
    { 0x3660, 0x04 },
    { 0x3666, 0xa5 },
    { 0x3667, 0xa5 },
    { 0x366a, 0x54 },
    { 0x3673, 0x0d },
    { 0x3672, 0x0d },
    { 0x3671, 0x0d },
    { 0x3670, 0x0d },
    { 0x3685, 0x00 },
    { 0x3694, 0x0d },
    { 0x3693, 0x0d },
    { 0x3692, 0x0d },
    { 0x3691, 0x0d },
    { 0x3696, 0x4c },
    { 0x3697, 0x4c },
    { 0x3698, 0x42 },
    { 0x3699, 0x18 },
    { 0x369a, 0x18 },
    { 0x369b, 0x14 },
    { 0x369c, 0x14 },
    { 0x369d, 0xa6 },
    { 0x369e, 0x53 },
    { 0x369f, 0x2a },
    { 0x36a0, 0x15 },
    { 0x36a1, 0x53 },
    { 0x36a2, 0x66 },
    { 0x370a, 0x00 },
    { 0x370e, 0x0c },
    { 0x3710, 0x00 },
    { 0x3713, 0x00 },
    { 0x3725, 0x02 },
    { 0x372a, 0x03 },
    { 0x3738, 0xce },
    { 0x3739, 0x10 },
    { 0x3748, 0x00 },
    { 0x374a, 0x00 },
    { 0x374c, 0x00 },
    { 0x374e, 0x00 },
    { 0x3756, 0x00 },
    { 0x3757, 0x00 },
    { 0x3767, 0x00 },
    { 0x3771, 0x00 },
    { 0x377b, 0x28 },
    { 0x377c, 0x00 },
    { 0x377d, 0x0c },
    { 0x3781, 0x03 },
    { 0x3782, 0x00 },
    { 0x3789, 0x14 },
    { 0x3795, 0x02 },
    { 0x379c, 0x0c },
    { 0x379d, 0x04 },
    { 0x37b8, 0x04 },
    { 0x37ba, 0x03 },
    { 0x37bb, 0x00 },
    { 0x37bc, 0x04 },
    { 0x37be, 0x08 },
    { 0x37c4, 0x11 },
    { 0x37c5, 0x80 },
    { 0x37c6, 0x14 },
    { 0x37c7, 0x08 },
    { 0x37da, 0x11 },
    { 0x3829, 0x03 },
    { 0x3881, 0x25 },
    { 0x3888, 0x04 },
    { 0x388b, 0x00 },
    { 0x3c80, 0x10 },
    { 0x3c86, 0x00 },
    { 0x3c9f, 0x01 },
    { 0x3d85, 0x1b },
    { 0x3d8c, 0x71 },
    { 0x3d8d, 0xe2 },
    { 0x3f00, 0x0b },
    { 0x400a, 0x01 },
    { 0x400b, 0x50 },
    { 0x400e, 0x08 },
    { 0x4040, 0x00 },
    { 0x4041, 0x07 },
    { 0x4043, 0x7e },
    { 0x4045, 0x7e },
    { 0x4047, 0x7e },
    { 0x4049, 0x7e },
    { 0x4090, 0x14 },
    { 0x40ba, 0x01 },
    { 0x4301, 0x00 },
    { 0x4303, 0x00 },
    { 0x4502, 0x04 },
    { 0x4503, 0x00 },
    { 0x4504, 0x06 },
    { 0x4506, 0x00 },
    { 0x4507, 0x47 },
    { 0x4800, 0x64 },
    { 0x4803, 0x00 },
    { 0x480c, 0x32 },
    { 0x480e, 0x04 },
    { 0x4813, 0xe4 },
    { 0x4819, 0x70 },
    { 0x481f, 0x30 },
    { 0x4823, 0x3f },
    { 0x4825, 0x30 },
    { 0x4833, 0x10 },
    { 0x484b, 0x27 },
    { 0x488b, 0x00 },
    { 0x4d00, 0x04 },
    { 0x4d01, 0xad },
    { 0x4d02, 0xbc },
    { 0x4d03, 0xa1 },
    { 0x4d04, 0x1f },
    { 0x4d05, 0x4c },
    { 0x4d0b, 0x01 },
    { 0x4e00, 0x2a },
    { 0x4e0d, 0x00 },
    { 0x5001, 0x09 },
    { 0x5004, 0x00 },
    { 0x5080, 0x04 },
    { 0x5036, 0x80 },
    { 0x5180, 0x70 },
    { 0x5181, 0x10 },
    { 0x520a, 0x03 },
    { 0x520b, 0x06 },
    { 0x520c, 0x0c },
    { 0x580b, 0x0f },
    { 0x580d, 0x00 },
    { 0x580f, 0x00 },
    { 0x5820, 0x00 },
    { 0x5821, 0x00 },
    { 0x301c, 0xf8 },
    { 0x301e, 0xb4 },
    { 0x301f, 0xf0 },
    { 0x3022, 0x01 },
    { 0x3109, 0xe7 },
    { 0x3600, 0x00 },
    { 0x3610, 0x65 },
    { 0x3611, 0x85 },
    { 0x3613, 0x3b },
    { 0x3615, 0x60 },
    { 0x3621, 0x90 },
    { 0x3620, 0x0c },
    { 0x3629, 0x00 },
    { 0x3661, 0x04 },
    { 0x3662, 0x10 },
    { 0x3664, 0x70 },
    { 0x3665, 0x00 },
    { 0x3681, 0xa6 },
    { 0x3682, 0x53 },
    { 0x3683, 0x2a },
    { 0x3684, 0x15 },
    { 0x3700, 0x2a },
    { 0x3701, 0x12 },
    { 0x3703, 0x28 },
    { 0x3704, 0x0e },
    { 0x3706, 0x4a },
    { 0x3709, 0x4a },
    { 0x370b, 0xa2 },
    { 0x370c, 0x01 },
    { 0x370f, 0x04 },
    { 0x3714, 0x24 },
    { 0x3716, 0x24 },
    { 0x3719, 0x11 },
    { 0x371a, 0x1e },
    { 0x3720, 0x00 },
    { 0x3724, 0x13 },
    { 0x373f, 0xb0 },
    { 0x3741, 0x4a },
    { 0x3743, 0x4a },
    { 0x3745, 0x4a },
    { 0x3747, 0x4a },
    { 0x3749, 0xa2 },
    { 0x374b, 0xa2 },
    { 0x374d, 0xa2 },
    { 0x374f, 0xa2 },
    { 0x3755, 0x10 },
    { 0x376c, 0x00 },
    { 0x378d, 0x30 },
    { 0x3790, 0x4a },
    { 0x3791, 0xa2 },
    { 0x3798, 0x40 }, // HCG: 0x40; LCG: 0xc0
    { 0x379e, 0x00 },
    { 0x379f, 0x00 },
    { 0x37a1, 0x10 },
    { 0x37a2, 0x1e },
    { 0x37a8, 0x10 },
    { 0x37a9, 0x1e },
    { 0x37ac, 0x00 },
    { 0x37b9, 0x01 },
    { 0x37bd, 0x01 },
    { 0x37bf, 0x08 },
    { 0x37c0, 0x11 },
    { 0x37c2, 0x04 },
    { 0x37cd, 0x19 },
    { 0x37d8, 0x02 },
    { 0x37d9, 0x08 },
    { 0x37e5, 0x02 },
    { 0x3800, 0x00 },
    { 0x3801, 0x00 },
    { 0x3802, 0x00 },
    { 0x3803, 0x00 },
    { 0x3804, 0x0a },
    { 0x3805, 0x8f },
    { 0x3806, 0x05 },
    { 0x3807, 0xff },
    { 0x3808, 0x0a },
    { 0x3809, 0x08 }, // 80
    { 0x380a, 0x05 },
    { 0x380b, 0xa8 }, // f0
    { 0x380c, 0x04 },
    { 0x380d, 0x2e },
    { 0x380e, 0x06 },
    { 0x380f, 0x26 },
    { 0x3811, 0x08 },
    { 0x3813, 0x08 },
    { 0x3814, 0x01 },
    { 0x3815, 0x01 },
    { 0x3816, 0x01 },
    { 0x3817, 0x01 },
    { 0x3820, 0x88 },
    { 0x3821, 0x04 },
    { 0x3880, 0x25 },
    { 0x3882, 0x20 },
    { 0x3c91, 0x0b },
    { 0x3c94, 0x45 },
    { 0x4000, 0xf3 },
    { 0x4001, 0x60 },
    { 0x4003, 0x40 },
    { 0x4008, 0x02 },
    { 0x4009, 0x0d },
    { 0x4300, 0xff },
    { 0x4302, 0x0f },
    { 0x4305, 0x83 },
    { 0x4505, 0x84 },
    { 0x4809, 0x0e },
    { 0x480a, 0x04 },
    { 0x4837, 0x15 },
    { 0x4c00, 0x08 },
    { 0x4c01, 0x08 },
    { 0x4c04, 0x00 },
    { 0x4c05, 0x00 },
    { 0x5000, 0xe9 },
#if 0
    {0x3208, 0x00},
    {0x3501, 0x01},
    {0x3502, 0xf0},
    {0x3511, 0x01},
    {0x3512, 0xf0},
    {0x3508, 0x02},
    {0x3509, 0x00},
    {0x350c, 0x02},// ;S_Gain
    {0x350d, 0x00},
    {0x3698, 0x00},
    {0x3699, 0x80},
    {0x369a, 0x80},
    {0x369b, 0x1f},
    {0x369c, 0x1f},
    {0x369d, 0x80},
    {0x369e, 0x40},
    {0x369f, 0x21},
    {0x36a0, 0x12},
    {0x36a1, 0xdd},
    {0x370e, 0x00},
    {0x3713, 0x04},
    {0x379c, 0x00},
    {0x379d, 0x00},
    {0x37be, 0x26},
    {0x37c7, 0xa8},
    {0x3881, 0x00},
    {0x3681, 0x80},
    {0x3682, 0x40},
    {0x3683, 0x21},
    {0x3684, 0x12},
    {0x370f, 0x00},
    {0x379f, 0x04},
    {0x37ac, 0xa0},
    {0x37bf, 0x26},
    {0x3880, 0x00},
    {0x3208, 0x10},
    {0x320d, 0x00},
    {0x3208, 0xa0},
#endif
    { 0x0100, 0x01 },

};

const static I2C_ARRAY Sensor_3p6m30_init_table[] = {
    // 4lane
    { 0x0103, 0x01 },
    { 0x0301, 0x84 },
    { 0x0303, 0x01 },
    { 0x0305, 0x5b },
    { 0x0306, 0x01 },
    { 0x0307, 0x17 },
    { 0x0323, 0x04 },
    { 0x0324, 0x01 },
    { 0x0325, 0x62 },
    { 0x3012, 0x06 },
    { 0x3013, 0x02 },
    { 0x3016, 0x72 },
    { 0x3021, 0x03 },
    { 0x3106, 0x21 },
    { 0x3107, 0xa1 },
    { 0x3500, 0x00 },
    { 0x3501, 0x03 },
    { 0x3502, 0x40 },
    { 0x3503, 0x88 },
    { 0x3508, 0x00 },
    { 0x3509, 0x80 },
    { 0x350a, 0x04 },
    { 0x350b, 0x00 },
    { 0x350c, 0x00 },
    { 0x350d, 0x80 },
    { 0x350e, 0x04 },
    { 0x350f, 0x00 },
    { 0x3510, 0x00 },
    { 0x3511, 0x00 },
    { 0x3512, 0x20 },
    { 0x3624, 0x00 }, // 20210421 sandy avoid vertical dot line
    { 0x3625, 0x4c },
    { 0x3660, 0x00 },
    { 0x3666, 0xa5 },
    { 0x3667, 0xa5 },
    { 0x366a, 0x64 },
    { 0x3673, 0x0d },
    { 0x3672, 0x0d },
    { 0x3671, 0x0d },
    { 0x3670, 0x0d },
    { 0x3685, 0x00 },
    { 0x3694, 0x0d },
    { 0x3693, 0x0d },
    { 0x3692, 0x0d },
    { 0x3691, 0x0d },
    { 0x3696, 0x4c },
    { 0x3697, 0x4c },
    { 0x3698, 0x40 },
    { 0x3699, 0x80 },
    { 0x369a, 0x18 },
    { 0x369b, 0x1f },
    { 0x369c, 0x14 },
    { 0x369d, 0x80 },
    { 0x369e, 0x40 },
    { 0x369f, 0x21 },
    { 0x36a0, 0x12 },
    { 0x36a1, 0x5d },
    { 0x36a2, 0x66 },
    { 0x370a, 0x00 },
    { 0x370e, 0x0c },
    { 0x3710, 0x00 },
    { 0x3713, 0x00 },
    { 0x3725, 0x02 },
    { 0x372a, 0x03 },
    { 0x3738, 0xce },
    { 0x3739, 0x10 },
    { 0x3748, 0x00 },
    { 0x374a, 0x00 },
    { 0x374c, 0x00 },
    { 0x374e, 0x00 },
    { 0x3756, 0x00 },
    { 0x3757, 0x0e },
    { 0x3767, 0x00 },
    { 0x3771, 0x00 },
    { 0x377b, 0x20 },
    { 0x377c, 0x00 },
    { 0x377d, 0x0c },
    { 0x3781, 0x03 },
    { 0x3782, 0x00 },
    { 0x3789, 0x14 },
    { 0x3795, 0x02 },
    { 0x379c, 0x00 },
    { 0x379d, 0x00 },
    { 0x37b8, 0x04 },
    { 0x37ba, 0x03 },
    { 0x37bb, 0x00 },
    { 0x37bc, 0x04 },
    { 0x37be, 0x08 },
    { 0x37c4, 0x11 },
    { 0x37c5, 0x80 },
    { 0x37c6, 0x14 },
    { 0x37c7, 0x08 },
    { 0x37da, 0x11 },
    { 0x381f, 0x08 },
    { 0x3829, 0x03 },
    { 0x3832, 0x00 },
    { 0x3881, 0x00 },
    { 0x3888, 0x04 },
    { 0x388b, 0x00 },
    { 0x3c80, 0x10 },
    { 0x3c86, 0x00 },
    { 0x3c9f, 0x01 },
    { 0x3d85, 0x1b },
    { 0x3d8c, 0x71 },
    { 0x3d8d, 0xe2 },
    { 0x3f00, 0x0b },
    { 0x3f06, 0x04 },
    { 0x400a, 0x01 },
    { 0x400b, 0x50 },
    { 0x400e, 0x08 },
    { 0x4040, 0x00 },
    { 0x4041, 0x07 },
    { 0x4043, 0x7e },
    { 0x4045, 0x7e },
    { 0x4047, 0x7e },
    { 0x4049, 0x7e },
    { 0x4090, 0x14 },
    { 0x40b0, 0x00 },
    { 0x40b1, 0x00 },
    { 0x40b2, 0x00 },
    { 0x40b3, 0x00 },
    { 0x40b4, 0x00 },
    { 0x40b5, 0x00 },
    { 0x40b7, 0x00 },
    { 0x40b8, 0x00 },
    { 0x40b9, 0x00 },
    { 0x40ba, 0x00 },
    { 0x4301, 0x00 },
    { 0x4303, 0x00 },
    { 0x4502, 0x04 },
    { 0x4503, 0x00 },
    { 0x4504, 0x06 },
    { 0x4506, 0x00 },
    { 0x4507, 0x64 },
    { 0x4803, 0x00 },
    { 0x480c, 0x32 },
    { 0x480e, 0x00 },
    { 0x4813, 0x00 },
    { 0x4819, 0x70 },
    { 0x481f, 0x30 },
    { 0x4823, 0x3f },
    { 0x4825, 0x30 },
    { 0x4833, 0x10 },
    { 0x484b, 0x07 },
    { 0x488b, 0x00 },
    { 0x4d00, 0x04 },
    { 0x4d01, 0xad },
    { 0x4d02, 0xbc },
    { 0x4d03, 0xa1 },
    { 0x4d04, 0x1f },
    { 0x4d05, 0x4c },
    { 0x4d0b, 0x01 },
    { 0x4e00, 0x2a },
    { 0x4e0d, 0x00 },
    { 0x5001, 0x09 },
    { 0x5004, 0x00 },
    { 0x5080, 0x04 },
    { 0x5036, 0x00 },
    { 0x5180, 0x70 },
    { 0x5181, 0x10 },
    { 0x520a, 0x03 },
    { 0x520b, 0x06 },
    { 0x520c, 0x0c },
    { 0x580b, 0x0f },
    { 0x580d, 0x00 },
    { 0x580f, 0x00 },
    { 0x5820, 0x00 },
    { 0x5821, 0x00 },
    { 0x301c, 0xf8 },
    { 0x301e, 0xb4 },
    { 0x301f, 0xd0 },
    { 0x3022, 0x01 },
    { 0x3109, 0xe7 },
    { 0x3600, 0x00 },
    { 0x3610, 0x65 },
    { 0x3611, 0x85 },
    { 0x3613, 0x3a },
    { 0x3615, 0x60 },
    { 0x3621, 0x90 },
    { 0x3620, 0x0c },
    { 0x3629, 0x00 },
    { 0x3661, 0x04 },
    { 0x3662, 0x10 },
    { 0x3664, 0x70 },
    { 0x3665, 0x00 },
    { 0x3681, 0xa6 },
    { 0x3682, 0x53 },
    { 0x3683, 0x2a },
    { 0x3684, 0x15 },
    { 0x3700, 0x2a },
    { 0x3701, 0x12 },
    { 0x3703, 0x28 },
    { 0x3704, 0x0e },
    { 0x3706, 0x4a },
    { 0x3709, 0x4a },
    { 0x370b, 0xa2 },
    { 0x370c, 0x01 },
    { 0x370f, 0x04 },
    { 0x3714, 0x24 },
    { 0x3716, 0x24 },
    { 0x3719, 0x11 },
    { 0x371a, 0x1e },
    { 0x3720, 0x00 },
    { 0x3724, 0x13 },
    { 0x373f, 0xb0 },
    { 0x3741, 0x4a },
    { 0x3743, 0x4a },
    { 0x3745, 0x4a },
    { 0x3747, 0x4a },
    { 0x3749, 0xa2 },
    { 0x374b, 0xa2 },
    { 0x374d, 0xa2 },
    { 0x374f, 0xa2 },
    { 0x3755, 0x10 },
    { 0x376c, 0x00 },
    { 0x378d, 0x30 },
    { 0x3790, 0x4a },
    { 0x3791, 0xa2 },
    { 0x3798, 0x40 }, // HCG: 0x40; LCG: 0xc0
    { 0x379e, 0x00 },
    { 0x379f, 0x04 },
    { 0x37a1, 0x10 },
    { 0x37a2, 0x1e },
    { 0x37a8, 0x10 },
    { 0x37a9, 0x1e },
    { 0x37ac, 0xa0 },
    { 0x37b9, 0x01 },
    { 0x37bd, 0x01 },
    { 0x37bf, 0x26 },
    { 0x37c0, 0x11 },
    { 0x37c2, 0x04 },
    { 0x37cd, 0x19 },
    { 0x37d8, 0x02 },
    { 0x37d9, 0x08 },
    { 0x37e5, 0x02 },
    { 0x3800, 0x00 },
    { 0x3801, 0x00 },
    { 0x3802, 0x00 },
    { 0x3803, 0x00 },
    { 0x3804, 0x0a },
    { 0x3805, 0x8f },
    { 0x3806, 0x05 },
    { 0x3807, 0xff },
    { 0x3808, 0x0a },
    { 0x3809, 0x08 },
    { 0x380a, 0x05 },
    { 0x380b, 0xa8 },
    { 0x380c, 0x08 },
    { 0x380d, 0x5c },
    { 0x380e, 0x06 },
    { 0x380f, 0x26 },
    { 0x3811, 0x08 },
    { 0x3813, 0x08 },
    { 0x3814, 0x01 },
    { 0x3815, 0x01 },
    { 0x3816, 0x01 },
    { 0x3817, 0x01 },
    { 0x3820, 0x88 },
    { 0x3821, 0x00 },
    { 0x3880, 0x25 },
    { 0x3882, 0x20 },
    { 0x3c91, 0x0b },
    { 0x3c94, 0x45 },
    { 0x4000, 0xf3 },
    { 0x4001, 0x60 },
    { 0x4003, 0x40 },
    { 0x4008, 0x02 },
    { 0x4009, 0x0d },
    { 0x4300, 0xff },
    { 0x4302, 0x0f },
    { 0x4305, 0x83 },
    { 0x4505, 0x84 },
    { 0x4800, 0x64 },
    { 0x4809, 0x1e },
    { 0x480a, 0x04 },
    { 0x4837, 0x15 },
    { 0x4c00, 0x08 },
    { 0x4c01, 0x08 },
    { 0x4c04, 0x00 },
    { 0x4c05, 0x00 },
    { 0x5000, 0xe9 },
    { 0x0100, 0x01 },

    /*//2lane
    {0x0100, 0x00},
    {0x0103, 0x01},
    {0x0301, 0x84},
    {0x0303, 0x01},
    {0x0305, 0x3c},
    {0x0306, 0x00},
    {0x0307, 0x17},
    {0x0323, 0x04},
    {0x0324, 0x01},
    {0x0325, 0x62},
    {0x3012, 0x06},
    {0x3013, 0x02},
    {0x3016, 0x32},
    {0x3021, 0x03},
    {0x3106, 0x25},
    {0x3107, 0xa1},
    {0x3500, 0x00},
    {0x3501, 0x03},
    {0x3502, 0x40},
    {0x3503, 0x88},
    {0x3508, 0x00},
    {0x3509, 0x80},
    {0x350a, 0x04},
    {0x350b, 0x00},
    {0x350c, 0x00},
    {0x350d, 0x80},
    {0x350e, 0x04},
    {0x350f, 0x00},
    {0x3510, 0x00},
    {0x3511, 0x00},
    {0x3512, 0x20},
    {0x3624, 0x02},
    {0x3625, 0x4c},
    {0x3660, 0x00},
    {0x3666, 0xa5},
    {0x3667, 0xa5},
    {0x366a, 0x64},
    {0x3673, 0x0d},
    {0x3672, 0x0d},
    {0x3671, 0x0d},
    {0x3670, 0x0d},
    {0x3685, 0x00},
    {0x3694, 0x0d},
    {0x3693, 0x0d},
    {0x3692, 0x0d},
    {0x3691, 0x0d},
    {0x3696, 0x4c},
    {0x3697, 0x4c},
    {0x3698, 0x40},
    {0x3699, 0x80},
    {0x369a, 0x18},
    {0x369b, 0x1f},
    {0x369c, 0x14},
    {0x369d, 0x80},
    {0x369e, 0x40},
    {0x369f, 0x21},
    {0x36a0, 0x12},
    {0x36a1, 0x5d},
    {0x36a2, 0x66},
    {0x370a, 0x00},
    {0x370e, 0x0c},
    {0x3710, 0x00},
    {0x3713, 0x00},
    {0x3725, 0x02},
    {0x372a, 0x03},
    {0x3738, 0xce},
    {0x3739, 0x10},
    {0x3748, 0x00},
    {0x374a, 0x00},
    {0x374c, 0x00},
    {0x374e, 0x00},
    {0x3756, 0x00},
    {0x3757, 0x0e},
    {0x3767, 0x00},
    {0x3771, 0x00},
    {0x377b, 0x20},
    {0x377c, 0x00},
    {0x377d, 0x0c},
    {0x3781, 0x03},
    {0x3782, 0x00},
    {0x3789, 0x14},
    {0x3795, 0x02},
    {0x379c, 0x00},
    {0x379d, 0x00},
    {0x37b8, 0x04},
    {0x37ba, 0x03},
    {0x37bb, 0x00},
    {0x37bc, 0x04},
    {0x37be, 0x08},
    {0x37c4, 0x11},
    {0x37c5, 0x80},
    {0x37c6, 0x14},
    {0x37c7, 0x08},
    {0x37da, 0x11},
    {0x381f, 0x08},
    {0x3829, 0x03},
    {0x3832, 0x00},
    {0x3881, 0x00},
    {0x3888, 0x04},
    {0x388b, 0x00},
    {0x3c80, 0x10},
    {0x3c86, 0x00},
    {0x3c9f, 0x01},
    {0x3d85, 0x1b},
    {0x3d8c, 0x71},
    {0x3d8d, 0xe2},
    {0x3f00, 0x0b},
    {0x3f06, 0x04},
    {0x400a, 0x01},
    {0x400b, 0x50},
    {0x400e, 0x08},
    {0x4040, 0x00},
    {0x4041, 0x07},
    {0x4043, 0x7e},
    {0x4045, 0x7e},
    {0x4047, 0x7e},
    {0x4049, 0x7e},
    {0x4090, 0x14},
    {0x40b0, 0x00},
    {0x40b1, 0x00},
    {0x40b2, 0x00},
    {0x40b3, 0x00},
    {0x40b4, 0x00},
    {0x40b5, 0x00},
    {0x40b7, 0x00},
    {0x40b8, 0x00},
    {0x40b9, 0x00},
    {0x40ba, 0x00},
    {0x4301, 0x00},
    {0x4303, 0x00},
    {0x4502, 0x04},
    {0x4503, 0x00},
    {0x4504, 0x06},
    {0x4506, 0x00},
    {0x4507, 0x64},
    {0x4800, 0x64},
    {0x4803, 0x10},
    {0x480c, 0x32},
    {0x480e, 0x00},
    {0x4813, 0x00},
    {0x4819, 0x70},
    {0x481f, 0x30},
    {0x4823, 0x3c},
    {0x4825, 0x32},
    {0x4833, 0x10},
    {0x484b, 0x07},
    {0x488b, 0x00},
    {0x4d00, 0x04},
    {0x4d01, 0xad},
    {0x4d02, 0xbc},
    {0x4d03, 0xa1},
    {0x4d04, 0x1f},
    {0x4d05, 0x4c},
    {0x4d0b, 0x01},
    {0x4e00, 0x2a},
    {0x4e0d, 0x00},
    {0x5001, 0x09},
    {0x5004, 0x00},
    {0x5080, 0x04},
    {0x5036, 0x00},
    {0x5180, 0x70},
    {0x5181, 0x10},
    {0x520a, 0x03},
    {0x520b, 0x06},
    {0x520c, 0x0c},
    {0x580b, 0x0f},
    {0x580d, 0x00},
    {0x580f, 0x00},
    {0x5820, 0x00},
    {0x5821, 0x00},
    {0x301c, 0xf0},
    {0x301e, 0xb4},
    {0x301f, 0xd0},
    {0x3022, 0x01},
    {0x3109, 0xe7},
    {0x3600, 0x00},
    {0x3610, 0x65},
    {0x3611, 0x85},
    {0x3613, 0x3a},
    {0x3615, 0x60},
    {0x3621, 0x90},
    {0x3620, 0x0c},
    {0x3629, 0x00},
    {0x3661, 0x04},
    {0x3662, 0x10},
    {0x3664, 0x70},
    {0x3665, 0x00},
    {0x3681, 0xa6},
    {0x3682, 0x53},
    {0x3683, 0x2a},
    {0x3684, 0x15},
    {0x3700, 0x2a},
    {0x3701, 0x12},
    {0x3703, 0x28},
    {0x3704, 0x0e},
    {0x3706, 0x4a},
    {0x3709, 0x4a},
    {0x370b, 0xa2},
    {0x370c, 0x01},
    {0x370f, 0x04},
    {0x3714, 0x24},
    {0x3716, 0x24},
    {0x3719, 0x11},
    {0x371a, 0x1e},
    {0x3720, 0x00},
    {0x3724, 0x13},
    {0x373f, 0xb0},
    {0x3741, 0x4a},
    {0x3743, 0x4a},
    {0x3745, 0x4a},
    {0x3747, 0x4a},
    {0x3749, 0xa2},
    {0x374b, 0xa2},
    {0x374d, 0xa2},
    {0x374f, 0xa2},
    {0x3755, 0x10},
    {0x376c, 0x00},
    {0x378d, 0x30},
    {0x3790, 0x4a},
    {0x3791, 0xa2},
    {0x3798, 0x40},//HCG: 0x40; LCG: 0xc0
    {0x379e, 0x00},
    {0x379f, 0x04},
    {0x37a1, 0x10},
    {0x37a2, 0x1e},
    {0x37a8, 0x10},
    {0x37a9, 0x1e},
    {0x37ac, 0xa0},
    {0x37b9, 0x01},
    {0x37bd, 0x01},
    {0x37bf, 0x26},
    {0x37c0, 0x11},
    {0x37c2, 0x04},
    {0x37cd, 0x19},
    {0x37d8, 0x02},
    {0x37d9, 0x08},
    {0x37e5, 0x02},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x00},
    {0x3804, 0x0a},
    {0x3805, 0x8f},
    {0x3806, 0x05},
    {0x3807, 0xff},
    {0x3808, 0x0a},
    {0x3809, 0x08},
    {0x380a, 0x05},
    {0x380b, 0xa8},
    {0x380c, 0x08},
    {0x380d, 0x5c},
    {0x380e, 0x06},
    {0x380f, 0x26},
    {0x3811, 0x08},
    {0x3813, 0x08},
    {0x3814, 0x01},
    {0x3815, 0x01},
    {0x3816, 0x01},
    {0x3817, 0x01},
    {0x3820, 0x88},
    {0x3821, 0x00},
    {0x3880, 0x25},
    {0x3882, 0x20},
    {0x3c91, 0x0b},
    {0x3c94, 0x45},
    {0x4000, 0xf3},
    {0x4001, 0x60},
    {0x4003, 0x40},
    {0x4008, 0x02},
    {0x4009, 0x0d},
    {0x4300, 0xff},
    {0x4302, 0x0f},
    {0x4305, 0x83},
    {0x4505, 0x84},
    {0x4809, 0x1e},
    {0x480a, 0x04},
    {0x4837, 0x08},
    {0x4c00, 0x08},
    {0x4c01, 0x00},
    {0x4c04, 0x00},
    {0x4c05, 0x00},
    {0x5000, 0xe9},
    {0x0100, 0x01},*/
};

I2C_ARRAY TriggerStartTbl[] = {
    { 0x0100, 0x01 }, // normal mode
};

const I2C_ARRAY PatternTbl[] = {
    { 0x5081, 0x00 }, // colorbar pattern , bit 7 to enable
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

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

const I2C_ARRAY mirror_reg[] = {
    { 0x3820, 0x88 }, //[4]Flip [0]mirror
    { 0x3716, 0x24 }, //[4]Flip [0]mirror
};

const I2C_ARRAY gain_reg[] = {
    { 0x3508, 0x00 }, // long a-gain[13:8]
    { 0x3509, 0x80 }, // long a-gain[7:0]
    { 0x350A, 0x00 }, // d-gain[13:8]
    { 0x350B, 0x00 }, // d-gain[7:0]
};

static CUS_GAIN_GAP_ARRAY gain_gap_compensate[16] = { // compensate  gain gap
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 },
    { 0, 0 }
};

const I2C_ARRAY expo_reg[] = {
    { 0x3208, 0x00 }, // Group 0 hold start
    { 0x3500, 0x00 }, // long exp[19,16]
    { 0x3501, 0x02 }, // long exp[15,8]
    { 0x3502, 0x00 }, // long exp[7,0]
};

const I2C_ARRAY vts_reg[] = {
    { 0x380e, 0x06 },
    { 0x380f, 0x26 },
    //{0x3208, 0x10},//Group 0 hold end
    //{0x3208, 0xa0},// Group delay launch
};

const static I2C_ARRAY gain_vc1_reg[] = {
    { 0x350C, 0x00 }, // short
    { 0x350D, 0x80 },
    { 0x350E, 0x80 }, // Dgain
    { 0x350F, 0x80 },
};

const I2C_ARRAY expo_vc0_reg[] = {
    { 0x3501, 0x00 }, // long
    { 0x3502, 0x40 },
};

const I2C_ARRAY expo_vc1_reg[] = {
    { 0x3511, 0x00 }, // short
    { 0x3512, 0x20 },
};

const I2C_ARRAY MWB_HDR_reg[] = {
    { 0x5102, 0x04 }, // default
    { 0x5103, 0x00 },
    { 0x5142, 0x04 }, // default
    { 0x5143, 0x00 },
};

I2C_ARRAY expo_vc0_reg_temp[] = {
    { 0x3501, 0x02 },
    { 0x3502, 0x00 },
};

I2C_ARRAY expo_vc1_reg_temp[] = {
    { 0x3511, 0x00 },
    { 0x3512, 0x20 },
};

#if 0
CUS_INT_TASK_ORDER def_order = {
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
#define SENSOR_NAME os04c10

#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

/////////////////// sensor hardware dependent //////////////

static int OS04c10_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api;

    SENSOR_DMSG("[%s] ", __FUNCTION__);

    // Sensor power on sequence
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_UDELAY(5000);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_UDELAY(1000);
    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);

    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 2);
    }
    sensor_if->MCLK(idx, 1, handle->mclk);
    SENSOR_UDELAY(20);
    ///////////////////

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_UDELAY(20);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(6000);

    // sensor_if->Set3ATaskOrder(handle, def_order);
    //  pure power on
    // ISP_config_io(handle);
    // sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    // SENSOR_USLEEP(5000);
    // handle->i2c_bus->i2c_open(handle->i2c_bus,&handle->i2c_cfg);

    return SUCCESS;
}

static int OS04c10_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_UDELAY(30);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_UDELAY(30);
    sensor_if->MCLK(idx, 0, handle->mclk);

    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 0);
    }

    SENSOR_USLEEP(2000); // mantis:1690203
    return SUCCESS;
}

/////////////////// image function /////////////////////////
// Get and check sensor ID
// if i2c error or sensor id does not match then return FAIL
static int OS04c10_GetSensorID(ms_cus_sensor* handle, u32* id)
{
    int i, n;
    // u16 sen_data1,sen_data2;
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

        *id >>= 8;
        SENSOR_DMSG("[%s]OS04c10 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
        // printf("OS04c10 Read sensor id, get 0x%x Success\n", (int)*id);
    }
    return SUCCESS;
}

static int OS04c10_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{

    return SUCCESS;
}

static int OS04c10_SetFPS(ms_cus_sensor* handle, u32 fps);
static int OS04c10_SetAEGain_cal(ms_cus_sensor* handle, u32 gain);
static int OS04c10_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);

static int OS04c10_init_DCG_3p6m30(ms_cus_sensor* handle)
{
    int i;
    os04c10_params* params = (os04c10_params*)handle->private_data;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_DCG_3p6m30); i++) {
        if (Sensor_init_table_DCG_3p6m30[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_DCG_3p6m30[i].data);
        }
        if (SensorReg_Write(Sensor_init_table_DCG_3p6m30[i].reg, Sensor_init_table_DCG_3p6m30[i].data) != SUCCESS) {
            SENSOR_DMSG("[%s] I2C write fail\n", __FUNCTION__);
            return FAIL;
        }
        // SensorReg_Read(Sensor_init_table_DCG_3p6m30[i].reg, &sen_data);
        // printf("[%s] i=0x%x,sen_data=0x%x\n", __FUNCTION__,i,sen_data);
    }

    for (i = 0; i < ARRAY_SIZE(mirror_reg); i++) {
        if (SensorReg_Write(mirror_reg[i].reg, mirror_reg[i].data) != SUCCESS) {
            return FAIL;
        }
    }

    params->tVts_reg[0].data = ((params->expo.vts >> 8) & 0x00ff);
    params->tVts_reg[1].data = ((params->expo.vts >> 0) & 0x00ff);

    return SUCCESS;
}

static int OS04c10_3p6m30_init(ms_cus_sensor* handle)
{
    int i;
    os04c10_params* params = (os04c10_params*)handle->private_data;

    for (i = 0; i < ARRAY_SIZE(Sensor_3p6m30_init_table); i++) {
        if (Sensor_3p6m30_init_table[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_3p6m30_init_table[i].data);
        }
        if (SensorReg_Write(Sensor_3p6m30_init_table[i].reg, Sensor_3p6m30_init_table[i].data) != SUCCESS) {
            SENSOR_DMSG("[%s] I2C write fail\n", __FUNCTION__);
            return FAIL;
        }
        // SensorReg_Read(Sensor_init_table[i].reg, &sen_data);
        // printf("[%s] i=0x%x,sen_data=0x%x\n", __FUNCTION__,i,sen_data);
    }

    for (i = 0; i < ARRAY_SIZE(PatternTbl); i++) {
        if (SensorReg_Write(PatternTbl[i].reg, PatternTbl[i].data) != SUCCESS) {
            return FAIL;
        }
    }

    for (i = 0; i < ARRAY_SIZE(TriggerStartTbl); i++) {
        if (SensorReg_Write(TriggerStartTbl[i].reg, TriggerStartTbl[i].data) != SUCCESS) {
            return FAIL;
        }
    }

    for (i = 0; i < ARRAY_SIZE(mirror_reg); i++) {
        if (SensorReg_Write(mirror_reg[i].reg, mirror_reg[i].data) != SUCCESS) {
            return FAIL;
        }
    }

    params->tVts_reg[0].data = ((params->expo.vts >> 8) & 0x00ff);
    params->tVts_reg[1].data = ((params->expo.vts >> 0) & 0x00ff);

    return SUCCESS;
}

static int OS04c10_GetVideoResNum(ms_cus_sensor* handle, u32* ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}

static int OS04c10_GetVideoRes(ms_cus_sensor* handle, u32 res_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int OS04c10_GetCurVideoRes(ms_cus_sensor* handle, u32* cur_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int OS04c10_SetVideoRes(ms_cus_sensor* handle, u32 res_idx)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0: // 3840x2160@30fps
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = OS04c10_3p6m30_init;
        params->expo.vts = vts_30fps;
        params->expo.fps = 30;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int OS04c10_SetVideoRes_HDR_DCG(ms_cus_sensor* handle, u32 res_idx)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0: // 3840x2160@30fps
        handle->video_res_supported.ulcur_res = 0;
        if (handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num == 1) {
            handle->pCus_sensor_init = OS04c10_init_DCG_3p6m30;
        }
        params->expo.vts = vts_30fps_DCG;
        params->expo.fps = 20;
        params->expo.max_short = 176;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int OS04c10_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    return params->cur_orien;
}

static int OS04c10_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    switch (orit) {
    case CUS_ORIT_M0F0:
        params->tMirror_reg[0].data = 0x88;
        params->tMirror_reg[1].data = 0x24;
        params->mirror_dirty = true;
        break;
    case CUS_ORIT_M1F0:
        params->tMirror_reg[0].data = 0x80;
        params->tMirror_reg[1].data = 0x24;
        params->mirror_dirty = true;
        break;
    case CUS_ORIT_M0F1:
        params->tMirror_reg[0].data = 0xb8;
        params->tMirror_reg[1].data = 0x04;
        params->mirror_dirty = true;
        break;
    case CUS_ORIT_M1F1:
        params->tMirror_reg[0].data = 0xb0;
        params->tMirror_reg[1].data = 0x04;
        params->mirror_dirty = true;
        break;
    default:
        break;
    }

    // SensorReg_Write(0x0100,0x01);
    params->cur_orien = orit;
    return SUCCESS;
}

static int OS04c10_GetFPS(ms_cus_sensor* handle)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int OS04c10_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    u32 vts = 0;
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * max_fps) / fps;
    } else if (fps >= (min_fps * 1000) && fps <= (max_fps * 1000)) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * (max_fps * 1000)) / fps;
    } else {
        SENSOR_DMSG("[%s] FPS %d out of range.\n", __FUNCTION__, fps);
        return FAIL;
    }

    if ((params->expo.lines) > (params->expo.vts - 8))
        vts = params->expo.lines + 8;
    else
        vts = params->expo.vts;
    params->tVts_reg[0].data = (vts >> 8) & 0x007f;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->dirty = true;
    return SUCCESS;
}

static int OS04c10_GetFPS_HDR_lef(ms_cus_sensor* handle)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps_DCG * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps_DCG * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int OS04c10_SetFPS_HDR_lef(ms_cus_sensor* handle, u32 fps)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_DCG * max_fps) / fps;
    } else if (fps >= (min_fps * 1000) && fps <= (max_fps * 1000)) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_DCG * (max_fps * 1000)) / fps;
    } else {
        SENSOR_DMSG("[%s] FPS %d out of range.\n", __FUNCTION__, fps);
        return FAIL;
    }
    // pr_info("[%s] %d  %d \n\n", __FUNCTION__, params->expo.vts, fps);
    params->expo.max_short = (((params->expo.vts) / 17 - 1) >> 1) << 1;
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x007f;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    params->dirty = true;
    return SUCCESS;
}

static int OS04c10_GetFPS_HDR_sef(ms_cus_sensor* handle)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps_DCG * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps_DCG * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int OS04c10_SetFPS_HDR_sef(ms_cus_sensor* handle, u32 fps)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_DCG * max_fps) / fps;
    } else if (fps >= (min_fps * 1000) && fps <= (max_fps * 1000)) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_DCG * (max_fps * 1000)) / fps;
    } else {
        SENSOR_DMSG("[%s] FPS %d out of range.\n", __FUNCTION__, fps);
        return FAIL;
    }

    // pr_info("[%s] %d  %d \n\n", __FUNCTION__, params->expo.vts, fps);
    params->expo.max_short = (((params->expo.vts) / 17 - 1) >> 1) << 1;
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x007f;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    params->dirty = true;
    return SUCCESS;
}

static int OS04c10_GetSensorCap(ms_cus_sensor* handle, CUS_CAMSENSOR_CAP* cap)
{
    if (cap)
        memcpy(cap, &sensor_cap, sizeof(CUS_CAMSENSOR_CAP));
    else
        return FAIL;

    return SUCCESS;
}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int OS04c10_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:

        break;
    case CUS_FRAME_ACTIVE:
        if (params->mirror_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            params->mirror_dirty = false;
        }
        if (params->dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            SensorReg_Write(0x3208, 0x10);
            SensorReg_Write(0x3208, 0xa0);
            params->dirty = false;
        }

        break;
    default:
        break;
    }
    return SUCCESS;
}

static int OS04c10_AEStatusNotify_HDR_lef(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:

        break;
    case CUS_FRAME_ACTIVE:

        if (params->dirty) {
            // SensorReg_Write(0x3208, 0x00);
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_vc0_reg, ARRAY_SIZE(expo_vc0_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_vc1_reg, ARRAY_SIZE(expo_vc1_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_vc1_reg, ARRAY_SIZE(gain_vc1_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            params->dirty = false;
            if (params->mwb_dirty) {
                SensorReg_Write(0x3208, 0x01); // group hold
                SensorRegArrayW((I2C_ARRAY*)params->tMWB_HDR_reg, ARRAY_SIZE(MWB_HDR_reg));
                SensorReg_Write(0x3208, 0x11); // group hold
                SensorReg_Write(0x3208, 0xa0); // group hold
                params->mwb_dirty = false;
            }
        }
        break;
    default:
        break;
    }
    return SUCCESS;
}

static int OS04c10_AEStatusNotify_HDR_sef(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:
        /*skip bad frame while do mirror/flip, should skip first, then do mirror */
        if (framecount == 1) {
            SensorReg_Write(0x3208, 0x00);
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            SensorReg_Write(0x3208, 0x10);
            SensorReg_Write(0x3209, 0x01);
            SensorReg_Write(0x3208, 0xe0);
            framecount = 0;
        }
        if (params->mirror_dirty) {
            handle->sensor_if_api->SetSkipFrame(handle->snr_pad_group, params->expo.fps, 3);
            ++framecount;
            params->mirror_dirty = false;
        }

        break;
    case CUS_FRAME_ACTIVE:

        break;
    default:
        break;
    }
    return SUCCESS;
}

static int OS04c10_GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    int rc = SUCCESS;
    u32 lines = 0;
    // rc = SensorRegArrayR((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
    os04c10_params* params = (os04c10_params*)handle->private_data;

    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 16;
    lines |= (u32)(params->tExpo_reg[2].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_reg[3].data & 0xff) << 0;

    *us = (lines * Preview_line_period);

    return rc;
}

static int OS04c10_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    os04c10_params* params = (os04c10_params*)handle->private_data;

    lines = (1000 * us) / Preview_line_period;
    if (lines < 2)
        lines = 2;
    if (lines > params->expo.vts - 8)
        vts = lines + 8;
    else
        vts = params->expo.vts;
    params->expo.lines = lines;

    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);
    // lines <<= 4;
    params->tExpo_reg[1].data = (lines >> 16) & 0x000f;
    params->tExpo_reg[2].data = (lines >> 8) & 0x00ff;
    params->tExpo_reg[3].data = (lines >> 0) & 0x00ff;

    params->tVts_reg[0].data = (vts >> 8) & 0x007f;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;

    params->dirty = true;

    return SUCCESS;
}

static int OS04c10_GetAEUSecs_HDR_lef(ms_cus_sensor* handle, u32* us)
{
    int rc = SUCCESS;
    u32 lines = 0;
    // rc = SensorRegArrayR((I2C_ARRAY*)params->tExpo_vc0_reg, ARRAY_SIZE(expo_vc0_reg));
    os04c10_params* params = (os04c10_params*)handle->private_data;

    lines |= (u32)(params->tExpo_vc0_reg[0].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_vc0_reg[1].data & 0xff) << 0;

    *us = (lines * Preview_line_period_DCG);

    return rc;
}

static int OS04c10_SetAEUSecs_HDR_lef(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    os04c10_params* params = (os04c10_params*)handle->private_data;
    memcpy(expo_vc0_reg_temp, params->tExpo_vc0_reg, sizeof(expo_vc0_reg));

    lines = (1000 * us) / Preview_line_period_DCG;
    if (lines < 2)
        lines = 2;
    if (lines > ((params->expo.vts) - (params->expo.max_short) - 4))
        lines = (params->expo.vts) - (params->expo.max_short) - 4;
    else
        vts = params->expo.vts;

    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);
    // lines <<= 4;
    params->tExpo_vc0_reg[0].data = (lines >> 8) & 0x00ff;
    params->tExpo_vc0_reg[1].data = (lines >> 0) & 0x00ff;

    // pr_info("[%s] shutter %d  0x3e01 0x%x  0x3e02 0x%x\n", __FUNCTION__, us, params->tExpo_vc0_reg[0].data,params->tExpo_vc0_reg[1].data);
    params->dirty = true;

    return SUCCESS;
}

static int OS04c10_GetAEUSecs_HDR_sef(ms_cus_sensor* handle, u32* us)
{
    int rc = SUCCESS;
    u32 lines = 0;
    // rc = SensorRegArrayR((I2C_ARRAY*)params->tExpo_vc1_reg, ARRAY_SIZE(expo_vc1_reg));
    os04c10_params* params = (os04c10_params*)handle->private_data;

    lines |= (u32)(params->tExpo_vc1_reg[0].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_vc1_reg[1].data & 0xff) << 0;

    *us = (lines * Preview_line_period_DCG);

    return rc;
}

static int OS04c10_SetAEUSecs_HDR_sef(ms_cus_sensor* handle, u32 us)
{
    int i = 0;
    u32 lines = 0, vts = 0;
    u32 longexpo = 0, shortexpo = 0, MWB_L_G = 1024, MWB_S_G = 1024;
    os04c10_params* params = (os04c10_params*)handle->private_data;
    I2C_ARRAY MWB_HDR_reg_temp[] = {
        { 0x5102, 0x04 }, // default
        { 0x5103, 0x00 },
        { 0x5142, 0x04 }, // default
        { 0x5143, 0x00 },
    };
    memcpy(MWB_HDR_reg_temp, params->tMWB_HDR_reg, sizeof(MWB_HDR_reg));
    memcpy(expo_vc1_reg_temp, params->tExpo_vc1_reg, sizeof(expo_vc1_reg));

    longexpo = ((expo_vc0_reg_temp[0].data & 0xff) << 8) | ((expo_vc0_reg_temp[1].data & 0xff) << 0);
    shortexpo = ((expo_vc1_reg_temp[0].data & 0xff) << 8) | ((expo_vc1_reg_temp[1].data & 0xff) << 0);
    MWB_L_G = ((4 * longexpo + 3) << 10) / (4 * longexpo + 1);
    MWB_S_G = ((4 * shortexpo + 3) << 10) / (4 * shortexpo + 1);

    lines = (1000 * us) / Preview_line_period_DCG;
    if (lines < 2)
        lines = 2;
    if (lines > ((params->expo.max_short) - 4))
        lines = (params->expo.max_short) - 4;
    else
        vts = params->expo.vts;

    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);
    // lines <<= 4;
    params->tExpo_vc1_reg[0].data = (lines >> 8) & 0x00ff;
    params->tExpo_vc1_reg[1].data = (lines >> 0) & 0x00ff;
    params->tMWB_HDR_reg[1].data = (MWB_L_G >> 0) & 0x00ff;
    params->tMWB_HDR_reg[3].data = ((MWB_S_G << 10) / MWB_L_G) & 0x00ff;

    for (i = 0; i < sizeof(MWB_HDR_reg) / sizeof(I2C_ARRAY); i++) {
        if (params->tMWB_HDR_reg[i].data != MWB_HDR_reg_temp[i].data) {
            params->mwb_dirty = true;
            break;
        }
    }

    params->dirty = true;
    return SUCCESS;
}

// Gain: 1x = 1024
static int OS04c10_GetAEGain(ms_cus_sensor* handle, u32* gain)
{

    // SENSOR_DMSG("[%s] get gain/reg0/reg1 (1024=1X)= %d/0x%x/0x%x\n", __FUNCTION__, *gain,params->tGain_reg[0].data,params->tGain_reg[1].data);
    return SUCCESS;
}

#define MAX_A_GAIN 15872 //(15.5*1024)
static int OS04c10_SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    CUS_GAIN_GAP_ARRAY* Sensor_Gain_Linearity;
    u32 i, input_gain = 0;
    u16 gain16 = 0;

    if (gain < 1024)
        gain = 1024;
    else if (gain >= SENSOR_MAX_GAIN)
        gain = SENSOR_MAX_GAIN;

    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    input_gain = gain;
    if (gain < 1024)
        gain = 1024;
    else if (gain >= MAX_A_GAIN)
        gain = MAX_A_GAIN;

    Sensor_Gain_Linearity = gain_gap_compensate;

    for (i = 0; i < sizeof(gain_gap_compensate) / sizeof(CUS_GAIN_GAP_ARRAY); i++) {

        if (Sensor_Gain_Linearity[i].gain == 0)
            break;
        if ((gain > Sensor_Gain_Linearity[i].gain) && (gain < (Sensor_Gain_Linearity[i].gain + Sensor_Gain_Linearity[i].offset))) {
            gain = Sensor_Gain_Linearity[i].gain;
            break;
        }
    }

    /* A Gain */
    if (gain < 1024) {
        gain = 1024;
    } else if ((gain >= 1024) && (gain < 2048)) {
        gain = (gain >> 6) << 6;
    } else if ((gain >= 2048) && (gain < 4096)) {
        gain = (gain >> 7) << 7;
    } else if ((gain >= 4096) && (gain < 8192)) {
        gain = (gain >> 8) << 8;
    } else if ((gain >= 8192) && (gain < MAX_A_GAIN)) {
        gain = (gain >> 9) << 9;
    } else {
        gain = MAX_A_GAIN;
    }

    gain16 = (u16)(gain >> 3);
    params->tGain_reg[0].data = (gain16 >> 8) & 0x3f; // high bit
    params->tGain_reg[1].data = gain16 & 0xff; // low byte

    if (input_gain > MAX_A_GAIN) {
        params->tGain_reg[2].data = (u16)((input_gain * 4) / MAX_A_GAIN) & 0x3F;
        params->tGain_reg[3].data = (u16)((input_gain * 1024) / MAX_A_GAIN) & 0xFF;
    } else {
        u16 tmp_dgain = ((input_gain * 1024) / gain);
        params->tGain_reg[2].data = (u16)((tmp_dgain >> 8) & 0x3F);
        params->tGain_reg[3].data = (u16)(tmp_dgain & 0xFF);
    }

    // pr_info("[%s] gain %d  0x3508 0x%x  0x3509 0x%x\n", __FUNCTION__, input_gain, params->tGain_reg[0].data, params->tGain_reg[1].data);
    params->dirty = true;
    return SUCCESS;
}

static int OS04c10_SetAEGain_HDR_sef(ms_cus_sensor* handle, u32 gain)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    CUS_GAIN_GAP_ARRAY* Sensor_Gain_Linearity;
    u32 i, input_gain = 0;
    u16 gain16 = 0;

    if (gain < 1024)
        gain = 1024;
    else if (gain >= SENSOR_MAX_GAIN)
        gain = SENSOR_MAX_GAIN;

    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    // gain = 4*gain;
    input_gain = gain;
    if (gain < 1024)
        gain = 1024;
    else if (gain >= MAX_A_GAIN)
        gain = MAX_A_GAIN;

    Sensor_Gain_Linearity = gain_gap_compensate;

    for (i = 0; i < sizeof(gain_gap_compensate) / sizeof(CUS_GAIN_GAP_ARRAY); i++) {

        if (Sensor_Gain_Linearity[i].gain == 0)
            break;
        if ((gain > Sensor_Gain_Linearity[i].gain) && (gain < (Sensor_Gain_Linearity[i].gain + Sensor_Gain_Linearity[i].offset))) {
            gain = Sensor_Gain_Linearity[i].gain;
            break;
        }
    }

    /* A Gain */
    if (gain < 1024) {
        gain = 1024;
    } else if ((gain >= 1024) && (gain < 2048)) {
        gain = (gain >> 6) << 6;
    } else if ((gain >= 2048) && (gain < 4096)) {
        gain = (gain >> 7) << 7;
    } else if ((gain >= 4096) && (gain < 8192)) {
        gain = (gain >> 8) << 8;
    } else if ((gain >= 8192) && (gain < MAX_A_GAIN)) {
        gain = (gain >> 9) << 9;
    } else {
        gain = MAX_A_GAIN;
    }

    gain16 = (u16)(gain >> 3);
    params->tGain_vc1_reg[0].data = (gain16 >> 8) & 0x3f; // high bit
    params->tGain_vc1_reg[1].data = gain16 & 0xff; // low byte

    if (input_gain > MAX_A_GAIN) {
        params->tGain_vc1_reg[2].data = (u16)((input_gain * 4) / MAX_A_GAIN) & 0x3F;
        params->tGain_vc1_reg[3].data = (u16)((input_gain * 1024) / MAX_A_GAIN) & 0xFF;
    } else {
        u16 tmp_dgain = ((input_gain * 1024) / gain);
        params->tGain_vc1_reg[2].data = (u16)((tmp_dgain >> 8) & 0x3F);
        params->tGain_vc1_reg[3].data = (u16)(tmp_dgain & 0xFF);
    }

    // pr_info("[%s] gain %d  0x350c 0x%x  0x350d 0x%x\n", __FUNCTION__, input_gain, params->tGain_vc1_reg[0].data, params->tGain_vc1_reg[1].data);

    params->dirty = true;
    // pr_info("[%s] set input gain/gain/AregH/AregL/DregH/DregL=%d/%d/0x%x/0x%x/0x%x/0x%x\n", __FUNCTION__, input_gain,gain,params->tGain_reg[0].data,params->tGain_reg[1].data,params->tGain_reg[2].data,params->tGain_reg[3].data);

    return SUCCESS;
}

static int OS04c10_SetAEGain_cal(ms_cus_sensor* handle, u32 gain)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    // CUS_GAIN_GAP_ARRAY* Sensor_Gain_Linearity;
    u32 input_gain = 0;
    u16 gain16;

    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    input_gain = gain;

    if (gain < 1024)
        gain = 1024;
    else if (gain >= MAX_A_GAIN)
        gain = MAX_A_GAIN;

    gain16 = (u16)(gain >> 3);
    params->tGain_reg[0].data = (gain16 >> 8) & 0x3f; // high bit
    params->tGain_reg[1].data = gain16 & 0xff; // low byte

    if (input_gain > MAX_A_GAIN) {
        params->tGain_reg[2].data = (u16)((input_gain * 4) / MAX_A_GAIN) & 0x3F;
        params->tGain_reg[3].data = (u16)((input_gain * 1024) / MAX_A_GAIN) & 0xFF;
    } else {
        params->tGain_reg[2].data = 0x04;
        params->tGain_reg[3].data = 0;
    }

    SENSOR_DMSG("[%s] set input gain/gain/regH/regL=%d/%d/0x%x/0x%x\n", __FUNCTION__, input_gain, gain, params->tGain_reg[0].data, params->tGain_reg[1].data);
    return SUCCESS;
}

static int OS04c10_SetAEGain_cal_lef(ms_cus_sensor* handle, u32 gain)
{
    return SUCCESS;
}

static int OS04c10_setCaliData_gain_linearity_hdr_lef(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    return SUCCESS;
}

static int OS04c10_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1;
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int OS04c10_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{

    *min = SENSOR_MIN_GAIN; // 1024*1.52;
    *max = SENSOR_MAX_GAIN;
    return SUCCESS;
}

static int OS04c10_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    u32 i, j;

    for (i = 0, j = 0; i < num; i++, j += 2) {
        gain_gap_compensate[i].gain = pArray[i].gain;
        gain_gap_compensate[i].offset = pArray[i].offset;
    }

    return SUCCESS;
}

static int OS04c10_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period * 2;
    info->step = Preview_line_period;
    return SUCCESS;
}

static int OS04c10_GetShutterInfo_HDR_SEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    os04c10_params* params = (os04c10_params*)handle->private_data;
    info->max = Preview_line_period_DCG * params->expo.max_short;
    info->min = Preview_line_period_DCG * 2;
    info->step = Preview_line_period_DCG;
    return SUCCESS;
}

static int OS04c10_GetShutterInfo_HDR_LEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period_DCG * 2;
    info->step = Preview_line_period_DCG;
    return SUCCESS;
}

static int OS04c10_SetPatternMode_hdr_lef(ms_cus_sensor* handle, u32 mode)
{
    return SUCCESS;
}

static int OS04c10_poweron_hdr_lef(ms_cus_sensor* handle, u32 idx)
{
    return SUCCESS;
}

static int OS04c10_poweroff_hdr_lef(ms_cus_sensor* handle, u32 idx)
{
    return SUCCESS;
}

static int OS04c10_GetSensorID_hdr_lef(ms_cus_sensor* handle, u32* id)
{
    *id = 0;
    return SUCCESS;
}

static int OS04c10_init_hdr_lef(ms_cus_sensor* handle)
{
    return SUCCESS;
}

#define CMDID_I2C_READ (0x01)
#define CMDID_I2C_WRITE (0x02)

static int pCus_sensor_CustDefineFunction(ms_cus_sensor* handle, u32 cmd_id, void* param)
{

    if (param == NULL || handle == NULL) {
        SENSOR_EMSG("param/handle data NULL \n");
        return FAIL;
    }

    switch (cmd_id) {
    case CMDID_I2C_READ: {
        I2C_ARRAY* reg = (I2C_ARRAY*)param;
        SensorReg_Read(reg->reg, &reg->data);
        SENSOR_EMSG("reg %x, read data %x \n", reg->reg, reg->data);
        break;
    }
    case CMDID_I2C_WRITE: {
        I2C_ARRAY* reg = (I2C_ARRAY*)param;
        SENSOR_EMSG("reg %x, write data %x \n", reg->reg, reg->data);
        SensorReg_Write(reg->reg, reg->data);
        break;
    }
    default:
        SENSOR_EMSG("cmd id %d err \n", cmd_id);
        break;
    }

    return SUCCESS;
}

int cus_camsensor_init_handle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    os04c10_params* params;
    int res;

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
    params = (os04c10_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    memcpy(params->tGain_vc1_reg, gain_vc1_reg, sizeof(gain_vc1_reg));
    memcpy(params->tExpo_vc0_reg, expo_vc0_reg, sizeof(expo_vc0_reg));
    memcpy(params->tExpo_vc1_reg, expo_vc1_reg, sizeof(expo_vc1_reg));
    memcpy(params->tMWB_HDR_reg, MWB_HDR_reg, sizeof(MWB_HDR_reg));

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "OS04c10_MIPI");

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
    handle->video_res_supported.ulcur_res = 0;
    for (res = 0; res < LINEAR_RES_END; res++) {
        handle->video_res_supported.num_res = res + 1;
        handle->video_res_supported.res[res].width = os04c10_mipi_linear[res].senif.preview_w;
        handle->video_res_supported.res[res].height = os04c10_mipi_linear[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = os04c10_mipi_linear[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = os04c10_mipi_linear[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = os04c10_mipi_linear[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = os04c10_mipi_linear[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = os04c10_mipi_linear[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = os04c10_mipi_linear[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, os04c10_mipi_linear[res].senstr.strResDesc);
    }

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
    // handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    // handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    handle->PCLK_POLARITY = SENSOR_PCLK_POL; // CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    // Mirror / Flip
    params->cur_orien = SENSOR_ORIT;

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2; // 0;//1;
    handle->ae_shutter_delay = 2; // 1;//2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2;

    /// calibration
    handle->sat_mingain = SENSOR_MIN_GAIN;

    // LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = OS04c10_3p6m30_init;
    handle->pCus_sensor_poweron = OS04c10_poweron;
    handle->pCus_sensor_poweroff = OS04c10_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID = OS04c10_GetSensorID;
    handle->pCus_sensor_GetVideoResNum = OS04c10_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes = OS04c10_GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes = OS04c10_GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes = OS04c10_SetVideoRes;
    handle->pCus_sensor_GetOrien = OS04c10_GetOrien;
    handle->pCus_sensor_SetOrien = OS04c10_SetOrien;
    handle->pCus_sensor_GetFPS = OS04c10_GetFPS;
    handle->pCus_sensor_SetFPS = OS04c10_SetFPS;
    // handle->pCus_sensor_GetSensorCap    = OS04c10_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = OS04c10_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = OS04c10_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs = OS04c10_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = OS04c10_SetAEUSecs;
    handle->pCus_sensor_GetAEGain = OS04c10_GetAEGain;
    handle->pCus_sensor_SetAEGain = OS04c10_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = OS04c10_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = OS04c10_GetAEMinMaxUSecs;

    handle->pCus_sensor_GetShutterInfo = OS04c10_GetShutterInfo;
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    // sensor calibration
    // handle->pCus_sensor_setCaliData_mingain=OS04c10_setCaliData_mingain;
    handle->pCus_sensor_SetAEGain_cal = OS04c10_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = OS04c10_setCaliData_gain_linearity;

    params->expo.vts = vts_30fps;
    params->expo.fps = 20;
    params->expo.lines = 1000;
    params->mirror_dirty = false;
    params->dirty = false;
    params->mwb_dirty = false;

    return SUCCESS;
}

int cus_camsensor_init_handle_hdr_dcg_sef(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    os04c10_params* params = NULL;
    int res;

    cus_camsensor_init_handle(drv_handle);
    params = (os04c10_params*)handle->private_data;

    sprintf(handle->model_id, "OS04c10_MIPI_HDR_SEF");

    handle->data_prec = SENSOR_DATAPREC_HDR; // CUS_DATAPRECISION_8;
    handle->bayer_id = SENSOR_BAYERID_HDR; // CUS_BAYER_GB;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->orient = SENSOR_ORIT; // CUS_ORIT_M1F1;
    // handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM_HDR;
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE_HDR;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_DCG;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 1; // Short frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////
    handle->video_res_supported.ulcur_res = 0;
    for (res = 0; res < HDR_RES_END; res++) {
        handle->video_res_supported.num_res = res + 1;
        handle->video_res_supported.res[res].width = os04c10_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = os04c10_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = os04c10_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = os04c10_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = os04c10_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = os04c10_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = os04c10_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = os04c10_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, os04c10_mipi_hdr[res].senstr.strResDesc);
    }

    // mclk
    handle->mclk = Preview_MCLK_SPEED_HDR;

    // Mirror / Flip
    params->cur_orien = SENSOR_ORIT;

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2;
    handle->ae_shutter_delay = 2;
    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2;

    /// calibration
    handle->sat_mingain = SENSOR_MIN_GAIN;

    // LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_init = OS04c10_init_DCG_3p6m30;

    // Normal
    handle->pCus_sensor_SetVideoRes = OS04c10_SetVideoRes_HDR_DCG;
    handle->pCus_sensor_GetFPS = OS04c10_GetFPS_HDR_sef;
    handle->pCus_sensor_SetFPS = OS04c10_SetFPS_HDR_sef;

    handle->pCus_sensor_AEStatusNotify = OS04c10_AEStatusNotify_HDR_sef;
    handle->pCus_sensor_GetAEUSecs = OS04c10_GetAEUSecs_HDR_sef;
    handle->pCus_sensor_SetAEUSecs = OS04c10_SetAEUSecs_HDR_sef;
    handle->pCus_sensor_GetAEGain = OS04c10_GetAEGain;
    handle->pCus_sensor_SetAEGain = OS04c10_SetAEGain_HDR_sef;

    handle->pCus_sensor_GetShutterInfo = OS04c10_GetShutterInfo_HDR_SEF;

    params->expo.vts = vts_30fps_DCG;
    params->expo.fps = 20;
    params->expo.max_short = 90;

    return SUCCESS;
}

int cus_camsensor_init_handle_hdr_dcg_lef(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    os04c10_params* params;
    int res;

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
    params = (os04c10_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    memcpy(params->tGain_vc1_reg, gain_vc1_reg, sizeof(gain_vc1_reg));
    memcpy(params->tExpo_vc0_reg, expo_vc0_reg, sizeof(expo_vc0_reg));
    memcpy(params->tExpo_vc1_reg, expo_vc1_reg, sizeof(expo_vc1_reg));
    memcpy(params->tMWB_HDR_reg, MWB_HDR_reg, sizeof(MWB_HDR_reg));

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "OS04c10_MIPI_HDR_LEF");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    // SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type = SENSOR_ISP_TYPE; // ISP_SOC;
    // handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus = SENSOR_IFBUS_TYPE; // CUS_SENIF_BUS_PARL;
    handle->data_prec = SENSOR_DATAPREC_HDR; // CUS_DATAPRECISION_8;
    handle->data_mode = SENSOR_DATAMODE;
    handle->bayer_id = SENSOR_BAYERID_HDR; // CUS_BAYER_GB;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->orient = SENSOR_ORIT; // CUS_ORIT_M1F1;
    // handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM_HDR;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; // don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE_HDR;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_DCG;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 0; // Long frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////
    handle->video_res_supported.ulcur_res = 0;
    for (res = 0; res < HDR_RES_END; res++) {
        handle->video_res_supported.num_res = res + 1;
        handle->video_res_supported.res[res].width = os04c10_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = os04c10_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = os04c10_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = os04c10_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = os04c10_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = os04c10_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = os04c10_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = os04c10_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, os04c10_mipi_hdr[res].senstr.strResDesc);
    }

    // i2c
    handle->i2c_cfg.mode = SENSOR_I2C_LEGACY; //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt = SENSOR_I2C_FMT; // CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address = SENSOR_I2C_ADDR; // 0x5a;
    handle->i2c_cfg.speed = SENSOR_I2C_SPEED; // 320000;

    // mclk
    handle->mclk = Preview_MCLK_SPEED_HDR;

    // polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY = SENSOR_PWDN_POL; // CUS_CLK_POL_NEG;
    handle->reset_POLARITY = SENSOR_RST_POL; // CUS_CLK_POL_NEG;
    // handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    // handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    // handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    // Mirror / Flip
    params->cur_orien = SENSOR_ORIT;

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2; // 0;//1;
    handle->ae_shutter_delay = 2; // 1;//2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2;

    /// calibration
    handle->sat_mingain = SENSOR_MIN_GAIN;

    // LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = OS04c10_init_hdr_lef;
    handle->pCus_sensor_poweron = OS04c10_poweron_hdr_lef;
    handle->pCus_sensor_poweroff = OS04c10_poweroff_hdr_lef;

    // Normal
    handle->pCus_sensor_GetSensorID = OS04c10_GetSensorID_hdr_lef;
    // handle->pCus_sensor_GetVideoResNum = OS04c10_GetVideoResNum;
    // handle->pCus_sensor_GetVideoRes       = OS04c10_GetVideoRes;
    // handle->pCus_sensor_GetCurVideoRes  = OS04c10_GetCurVideoRes;
    // handle->pCus_sensor_SetVideoRes       = OS04c10_SetVideoRes_HDR_DCG_lef;
    handle->pCus_sensor_GetOrien = OS04c10_GetOrien;
    handle->pCus_sensor_SetOrien = OS04c10_SetOrien;
    handle->pCus_sensor_GetFPS = OS04c10_GetFPS_HDR_lef;
    handle->pCus_sensor_SetFPS = OS04c10_SetFPS_HDR_lef;
    handle->pCus_sensor_GetSensorCap = OS04c10_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = OS04c10_SetPatternMode_hdr_lef;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = OS04c10_AEStatusNotify_HDR_lef;
    handle->pCus_sensor_GetAEUSecs = OS04c10_GetAEUSecs_HDR_lef;
    handle->pCus_sensor_SetAEUSecs = OS04c10_SetAEUSecs_HDR_lef;
    handle->pCus_sensor_GetAEGain = OS04c10_GetAEGain;
    handle->pCus_sensor_SetAEGain = OS04c10_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = OS04c10_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = OS04c10_GetAEMinMaxUSecs;

    handle->pCus_sensor_GetShutterInfo = OS04c10_GetShutterInfo_HDR_LEF;

    // sensor calibration
    // handle->pCus_sensor_setCaliData_mingain=OS04c10_setCaliData_mingain;
    handle->pCus_sensor_SetAEGain_cal = OS04c10_SetAEGain_cal_lef;
    handle->pCus_sensor_setCaliData_gain_linearity = OS04c10_setCaliData_gain_linearity_hdr_lef;
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    params->expo.vts = vts_30fps_DCG;
    params->expo.fps = 20;
    params->expo.lines = 1000;
    params->mirror_dirty = false;
    params->dirty = false;
    params->mwb_dirty = false;

    return SUCCESS;
}

int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(OS04c10_HDR,
    cus_camsensor_init_handle,
    cus_camsensor_init_handle_hdr_dcg_sef,
    cus_camsensor_init_handle_hdr_dcg_lef,
    os04c10_params);
