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

#include <drv_sensor_common.h>
#include <sensor_i2c_api.h>
#include <drv_sensor.h>

#ifdef __cplusplus
}
#endif

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(sc501ai);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE CAM_OS_ARRAY_SIZE
#endif

#define SENSOR_PAD_GROUP_SET CUS_SENSOR_PAD_GROUP_A
#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
// MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (4)
#define SENSOR_MIPI_LANE_NUM_HDR (4)
#define SENSOR_MIPI_HDR_MODE (2) // 0: Non-HDR mode. 1:Sony DOL mode
// MIPI config end.
//============================================

//#undef SENSOR_DBG
#define SENSOR_DBG 0

#define SENSOR_ISP_TYPE ISP_EXT // ISP_EXT, ISP_SOC
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_MIPI // CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_MIPI_HSYNC_MODE_HDR PACKET_FOOTER_EDGE
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAPREC_HDR CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000
#define SENSOR_BAYERID CUS_BAYER_BG // CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_BAYERID_HDR CUS_BAYER_BG // CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAXGAIN (24067 * 3175) / 100000 // max sensor gain, a-gain*conversion-gain*d-gain
#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_MCLK_SPEED_HDR CUS_CMU_CLK_27MHZ

u32 Preview_line_period = 20202;
#define vts_30fps 1650
#define Preview_line_period_HDR 10101
#define vts_30fps_HDR 3300
#define Preview_WIDTH 2880 // resolution Width when preview
#define Preview_HEIGHT 1620 // resolution Height when preview
#define Preview_MAX_FPS 30 // 25                     //fastest preview FPS
#define Preview_MAX_FPS_HDR 30 // 25                     //fastest preview FPS
#define Preview_MIN_FPS 5 // slowest preview FPS
#define Preview_CROP_START_X 0 // CROP_START_X
#define Preview_CROP_START_Y 0 // CROP_START_Y

#define SENSOR_I2C_ADDR 0x60 // I2C slave address
#define SENSOR_I2C_SPEED 240000 // I2C speed,60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A16D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL CUS_CLK_POL_NEG // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.
#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_POS // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL CUS_CLK_POL_POS // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG

////////////////////////////////////
// Image Info                     //
////////////////////////////////////
static struct { // LINEAR
    // Modify it based on number of support resolution
    enum { LINEAR_RES_1 = 0,
        LINEAR_RES_2,
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
} sc501ai_mipi_linear[] = {
    { LINEAR_RES_1, { 2880, 1620, 3, 30 }, { 0, 0, 2880, 1620 }, { "2880x1620@30fps" } },
    { LINEAR_RES_2, { 2880, 1620, 3, 60 }, { 480, 270, 1920, 1080 }, { "1920x1080@60fps" } },
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
} sc501ai_mipi_hdr[] = {
    { HDR_RES_1, { 2880, 1620, 3, 30 }, { 0, 0, 2880, 1620 }, { "2880x1620@30fps_HDR" } }, // Modify it
};

#if defined(SENSOR_MODULE_VERSION)
#define TO_STR_NATIVE(e) #e
#define TO_STR_PROXY(m, e) m(e)
#define MACRO_TO_STRING(e) TO_STR_PROXY(TO_STR_NATIVE, e)
static char* sensor_module_version = MACRO_TO_STRING(SENSOR_MODULE_VERSION);
module_param(sensor_module_version, charp, S_IRUGO);
#endif
static int cus_camsensor_release_handle(ms_cus_sensor* handle);
static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit);
//#define ABS(a)   ((a)>(0) ? (a) : (-(a)))
static int g_sensor_ae_min_gain = 1024;
#define ENABLE_NR 1

CUS_MCLK_FREQ UseParaMclk(void);

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
        u32 max_short_exp;
        u32 line;
    } expo;
    struct {
        bool bVideoMode;
        u16 res_idx;
        CUS_CAMSENSOR_ORIT orit;
    } res;
    I2C_ARRAY tVts_reg[2];
    I2C_ARRAY tGain_reg[4];
    I2C_ARRAY tGain_reg_HDR_SEF[4];
    I2C_ARRAY tExpo_reg[3];
    I2C_ARRAY tExpo_reg_HDR_SEF[2];
    I2C_ARRAY tMirror_reg[1];
#if ENABLE_NR
    I2C_ARRAY tTemperature_reg[1];
#endif
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool orient_dirty;
    bool reg_dirty;
    bool temperature_dirty;
} sc501ai_params;

// set sensor ID address and data,
const static I2C_ARRAY Sensor_id_table[] = {
    { 0x3107, 0xce },
    { 0x3108, 0x1f },
};

const static I2C_ARRAY Sensor_init_table_5M30fps[] = {
    // cleaned_0x01_SC501AI_27MInput_MIPI_4lane_396Mbps_10bit_2880x1620_30fps
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301f, 0x01 },
    { 0x3253, 0x0a },
    { 0x3301, 0x0a },
    { 0x3302, 0x18 },
    { 0x3303, 0x10 },
    { 0x3304, 0x60 },
    { 0x3306, 0x60 },
    { 0x3308, 0x10 },
    { 0x3309, 0x70 },
    { 0x330a, 0x00 },
    { 0x330b, 0xf0 },
    { 0x330d, 0x18 },
    { 0x330e, 0x20 },
    { 0x330f, 0x02 },
    { 0x3310, 0x02 },
    { 0x331c, 0x04 },
    { 0x331e, 0x51 },
    { 0x331f, 0x61 },
    { 0x3320, 0x09 },
    { 0x3333, 0x10 },
    { 0x334c, 0x08 },
    { 0x3356, 0x09 },
    { 0x3364, 0x17 },
    { 0x336d, 0x03 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x0a },
    { 0x3394, 0x20 },
    { 0x3395, 0x20 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x0a },
    { 0x339a, 0x20 },
    { 0x339b, 0x20 },
    { 0x339c, 0x20 },
    { 0x33ac, 0x10 },
    { 0x33ae, 0x10 },
    { 0x33af, 0x19 },
    { 0x360f, 0x01 },
    { 0x3622, 0x03 },
    { 0x363a, 0x1f },
    { 0x363c, 0x40 },
    { 0x3651, 0x7d },
    { 0x3670, 0x0a },
    { 0x3671, 0x07 },
    { 0x3672, 0x17 },
    { 0x3673, 0x1e },
    { 0x3674, 0x82 },
    { 0x3675, 0x64 },
    { 0x3676, 0x66 },
    { 0x367a, 0x48 },
    { 0x367b, 0x78 },
    { 0x367c, 0x58 },
    { 0x367d, 0x78 },
    { 0x3690, 0x34 },
    { 0x3691, 0x34 },
    { 0x3692, 0x54 },
    { 0x369c, 0x48 },
    { 0x369d, 0x78 },
    { 0x36ec, 0x1a },
    { 0x3904, 0x04 },
    { 0x3908, 0x41 },
    { 0x391d, 0x04 },
    { 0x39c2, 0x30 },
    { 0x3e01, 0xcd },
    { 0x3e02, 0xc0 },
    { 0x3e16, 0x00 },
    { 0x3e17, 0x80 },
    { 0x4500, 0x88 },
    { 0x4509, 0x20 },
    { 0x5799, 0x00 },
    { 0x59e0, 0x60 },
    { 0x59e1, 0x08 },
    { 0x59e2, 0x3f },
    { 0x59e3, 0x18 },
    { 0x59e4, 0x18 },
    { 0x59e5, 0x3f },
    { 0x59e7, 0x02 },
    { 0x59e8, 0x38 },
    { 0x59e9, 0x20 },
    { 0x59ea, 0x0c },
    { 0x59ec, 0x08 },
    { 0x59ed, 0x02 },
    { 0x59ee, 0xa0 },
    { 0x59ef, 0x08 },
    { 0x59f4, 0x18 },
    { 0x59f5, 0x10 },
    { 0x59f6, 0x0c },
    { 0x59f9, 0x02 },
    { 0x59fa, 0x18 },
    { 0x59fb, 0x10 },
    { 0x59fc, 0x0c },
    { 0x59ff, 0x02 },
    { 0x36e9, 0x1c },
    { 0x36f9, 0x24 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

const static I2C_ARRAY Sensor_init_table_5M60fps[] = {
    // cleaned_0x05_SC501AI_MIPI_27Minput_4lane_792Mbps_10bit_2880x1620_60fps_continue_mode   update 20220425
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301f, 0x05 },
    { 0x3106, 0x01 },
    { 0x3250, 0x40 },
    { 0x3253, 0x0a },
    { 0x3301, 0x0b },
    { 0x3302, 0x20 },
    { 0x3303, 0x10 },
    { 0x3304, 0x70 },
    { 0x3306, 0x50 },
    { 0x3308, 0x18 },
    { 0x3309, 0x80 },
    { 0x330a, 0x00 },
    { 0x330b, 0xe8 },
    { 0x330d, 0x30 },
    { 0x330e, 0x30 },
    { 0x330f, 0x02 },
    { 0x3310, 0x02 },
    { 0x331c, 0x08 },
    { 0x331e, 0x61 },
    { 0x331f, 0x71 },
    { 0x3320, 0x11 },
    { 0x3333, 0x10 },
    { 0x334c, 0x10 },
    { 0x3356, 0x11 },
    { 0x3364, 0x17 },
    { 0x336d, 0x03 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x0a },
    { 0x3394, 0x0a },
    { 0x3395, 0x12 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x0a },
    { 0x339a, 0x0a },
    { 0x339b, 0x0a },
    { 0x339c, 0x12 },
    { 0x33ac, 0x10 },
    { 0x33ae, 0x20 },
    { 0x33af, 0x21 },
    { 0x360f, 0x01 },
    { 0x3621, 0xe8 },
    { 0x3622, 0x06 },
    { 0x3630, 0x82 },
    { 0x3633, 0x33 },
    { 0x3634, 0x64 },
    { 0x3637, 0x50 },
    { 0x363a, 0x1f },
    { 0x363c, 0x40 },
    { 0x3651, 0x7d },
    { 0x3670, 0x0a },
    { 0x3671, 0x06 },
    { 0x3672, 0x16 },
    { 0x3673, 0x17 },
    { 0x3674, 0x82 },
    { 0x3675, 0x62 },
    { 0x3676, 0x44 },
    { 0x367a, 0x48 },
    { 0x367b, 0x78 },
    { 0x367c, 0x48 },
    { 0x367d, 0x58 },
    { 0x3690, 0x34 },
    { 0x3691, 0x34 },
    { 0x3692, 0x54 },
    { 0x369c, 0x48 },
    { 0x369d, 0x78 },
    { 0x36ea, 0x35 },
    { 0x36eb, 0x04 },
    { 0x36ec, 0x0a },
    { 0x36ed, 0x14 },
    { 0x36fa, 0x35 },
    { 0x36fb, 0x04 },
    { 0x36fc, 0x00 },
    { 0x36fd, 0x16 },
    { 0x3904, 0x04 },
    { 0x3908, 0x41 },
    { 0x391f, 0x10 },
    { 0x39c2, 0x30 },
    { 0x3e01, 0xcd },
    { 0x3e02, 0xc0 },
    { 0x4500, 0x88 },
    { 0x4509, 0x20 },
    { 0x4800, 0x04 },
    { 0x4837, 0x15 },
    { 0x36e9, 0x44 },
    { 0x36f9, 0x44 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

const static I2C_ARRAY Sensor_init_table_HDR_DOL_4lane[] = {
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301f, 0x23 },
    { 0x3106, 0x01 },
    { 0x320e, 0x0c },
    { 0x320f, 0xe4 },
    { 0x3220, 0x53 },
    { 0x3250, 0x3f },
    { 0x3253, 0x0a },
    { 0x3301, 0x0b },
    { 0x3302, 0x20 },
    { 0x3303, 0x10 },
    { 0x3304, 0x70 },
    { 0x3306, 0x50 },
    { 0x3308, 0x18 },
    { 0x3309, 0x80 },
    { 0x330a, 0x00 },
    { 0x330b, 0xe8 },
    { 0x330d, 0x30 },
    { 0x330e, 0x30 },
    { 0x330f, 0x02 },
    { 0x3310, 0x02 },
    { 0x331c, 0x08 },
    { 0x331e, 0x61 },
    { 0x331f, 0x71 },
    { 0x3320, 0x11 },
    { 0x3333, 0x10 },
    { 0x334c, 0x10 },
    { 0x3356, 0x11 },
    { 0x3364, 0x17 },
    { 0x336d, 0x03 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x0a },
    { 0x3394, 0x0a },
    { 0x3395, 0x12 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x0a },
    { 0x339a, 0x0a },
    { 0x339b, 0x0a },
    { 0x339c, 0x12 },
    { 0x33ac, 0x10 },
    { 0x33ae, 0x20 },
    { 0x33af, 0x21 },
    { 0x360f, 0x01 },
    { 0x3621, 0xe8 },
    { 0x3622, 0x06 },
    { 0x3630, 0x82 },
    { 0x3633, 0x33 },
    { 0x3634, 0x64 },
    { 0x3637, 0x50 },
    { 0x363a, 0x1f },
    { 0x363c, 0x40 },
    { 0x3651, 0x7d },
    { 0x3670, 0x0a },
    { 0x3671, 0x06 },
    { 0x3672, 0x16 },
    { 0x3673, 0x17 },
    { 0x3674, 0x82 },
    { 0x3675, 0x62 },
    { 0x3676, 0x44 },
    { 0x367a, 0x48 },
    { 0x367b, 0x78 },
    { 0x367c, 0x48 },
    { 0x367d, 0x58 },
    { 0x3690, 0x34 },
    { 0x3691, 0x34 },
    { 0x3692, 0x54 },
    { 0x369c, 0x48 },
    { 0x369d, 0x78 },
    { 0x36ea, 0x2d },
    { 0x36eb, 0x04 },
    { 0x36ec, 0x0a },
    { 0x36ed, 0x14 },
    { 0x36fa, 0x35 },
    { 0x36fb, 0x04 },
    { 0x36fc, 0x00 },
    { 0x36fd, 0x16 },
    { 0x3904, 0x04 },
    { 0x3908, 0x41 },
    { 0x391f, 0x10 },
    { 0x39c2, 0x30 },
    { 0x3e00, 0x01 },
    { 0x3e01, 0x82 },
    { 0x3e02, 0x00 },
    { 0x3e04, 0x18 },
    { 0x3e05, 0x20 },
    { 0x3e23, 0x00 },
    { 0x3e24, 0xc6 },
    { 0x4500, 0x88 },
    { 0x4509, 0x20 },
    { 0x4800, 0x24 }, // bit5; 0:continous-mode; 1:non
    { 0x4837, 0x14 },
    { 0x4853, 0xfd },
    { 0x36e9, 0x30 },
    { 0x36f9, 0x44 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

const static I2C_ARRAY mirror_reg[] = {
    { 0x3221, 0x00 }, // mirror[2:1], flip[6:5]
};

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

const static I2C_ARRAY gain_reg[] = {
    { 0x3e06, 0x00 },
    { 0x3e07, 0x80 },
    { 0x3e08, 0x00 | 0x03 },
    { 0x3e09, 0x40 }, // low bit, 0x40 - 0x7f, step 1/64
};
const static I2C_ARRAY gain_reg_HDR_SEF[] = {
    { 0x3e10, 0x00 },
    { 0x3e11, 0x00 },
    { 0x3e12, 0x00 | 0x03 },
    { 0x3e13, 0x40 }, // low bit, 0x40 - 0x7f, step 1/64
};

const I2C_ARRAY expo_reg[] = {
    { 0x3e00, 0x00 }, // expo [20:17]
    { 0x3e01, 0x30 }, // expo[16:8]
    { 0x3e02, 0x00 }, // expo[7:0], [3:0] fraction of line
};

const I2C_ARRAY expo_reg_HDR_SEF[] = {
    { 0x3e04, 0x00 }, // expo[16:8]
    { 0x3e05, 0x80 }, // expo[7:0], [3:0] fraction of line
};

const I2C_ARRAY vts_reg[] = {
    { 0x320e, 0x05 },
    { 0x320f, 0xdc },
};

#if ENABLE_NR
const I2C_ARRAY temperature_reg[] = {
    { 0x5799, 0x00 },
};
#endif

I2C_ARRAY PatternTbl[] = {
    { 0x4501, 0xc8 }, // testpattern , bit 3 to enable
};

CUS_INT_TASK_ORDER def_order = {
    .RunLength = 9,
    .Orders = {
        CUS_INT_TASK_AE | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_AE | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_AE | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_AWB | CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
        CUS_INT_TASK_VDOS | CUS_INT_TASK_AF,
    },
};
/*
/////////// function definition ///////////////////
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) LOGD(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) printf(args)
#elif SENSOR_DBG == 0
#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME sc501ai
*/
#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

/////////////////// sensor hardware dependent //////////////
static int pCus_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY); ////pwd low
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    // Sensor power on sequence
    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);
    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 2);
    }
    sensor_if->MCLK(idx, 1, handle->mclk);

    SENSOR_USLEEP(2000);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(1000);

    SENSOR_DMSG("[%s] pwd high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(2000);

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    // sc501ai_params *params = (sc501ai_params *)handle->private_data;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    // handle->i2c_bus->i2c_close(handle->i2c_bus);
    SENSOR_USLEEP(1000);
    // Set_csi_if(0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 0);
    }
    sensor_if->MCLK(idx, 0, handle->mclk);

    handle->orient = CUS_ORIT_M0F0;

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

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    for (n = 0; n < table_length; ++n) {
        id_from_sensor[n].reg = Sensor_id_table[n].reg;
        id_from_sensor[n].data = 0;
    }

    *id = 0;
    if (table_length > 8)
        table_length = 8;
    for (n = 0; n < 4; ++n) { // retry , until I2C success
        if (n > 2)
            return FAIL;
        if (SensorRegArrayR((I2C_ARRAY*)id_from_sensor, table_length) == SUCCESS) // read sensor ID from I2C
            break;
        else
            SENSOR_USLEEP(1000);
    }

    for (i = 0; i < table_length; ++i) {
        if (id_from_sensor[i].data != Sensor_id_table[i].data)
            return FAIL;
        *id = ((*id) + id_from_sensor[i].data) << 8;
    }
    *id >>= 8;
    SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);

    return SUCCESS;
}

static int sc501ai_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{
    int i;
    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);

    switch (mode) {
    case 1:
        PatternTbl[0].data = 0xc8; // enable
        break;
    case 0:
        PatternTbl[0].data = 0xc0; // disable
        break;
    default:
        PatternTbl[0].data = 0xc0; // disable
        break;
    }
    for (i = 0; i < ARRAY_SIZE(PatternTbl); i++) {
        if (SensorReg_Write(PatternTbl[i].reg, PatternTbl[i].data) != SUCCESS)
            return FAIL;
    }

    return SUCCESS;
}
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init_linear_5M30fps(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    // SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i, cnt;
    u16 reg3109, reg3109_1, reg3040, reg3040_l;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_5M30fps); i++) {
        if (Sensor_init_table_5M30fps[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_5M30fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_5M30fps[i].reg, Sensor_init_table_5M30fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n", cnt);
                if (cnt >= 10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    // SC501AI add logic  2020/08/24 by 06547    unuse
#if 1
    SensorReg_Read(0x3109, &reg3109);
    SensorReg_Read(0x3109, &reg3109_1);
    if (reg3109_1 == reg3109) {
        if (1 == reg3109) {
            SensorReg_Write(0x336d, 0x23);
        } else if (2 == reg3109) {
            SensorReg_Write(0x336d, 0x03);
        }
    }

    SensorReg_Read(0x3040, &reg3040);
    SensorReg_Read(0x3040, &reg3040_l);
    if (reg3040 == reg3040_l) {
        if (0 == reg3040) {
            SensorReg_Write(0x363c, 0x42);
        } else if (1 == reg3040) {
            SensorReg_Write(0x363c, 0x40);
        }
    }
#endif

    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

static int pCus_init_linear_60fps(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    int i, cnt;
    u16 reg3109, reg3109_1, reg3040, reg3040_l;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_5M60fps); i++) {
        if (Sensor_init_table_5M60fps[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_5M60fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_5M60fps[i].reg, Sensor_init_table_5M60fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n", cnt);
                if (cnt >= 10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    // SC501AI add logic  2020/08/24 by 06547    unuse
#if 1
    SensorReg_Read(0x3109, &reg3109);
    SensorReg_Read(0x3109, &reg3109_1);
    if (reg3109_1 == reg3109) {
        if (1 == reg3109) {
            SensorReg_Write(0x336d, 0x23);
        } else if (2 == reg3109) {
            SensorReg_Write(0x336d, 0x03);
        }
    }

    SensorReg_Read(0x3040, &reg3040);
    SensorReg_Read(0x3040, &reg3040_l);
    if (reg3040 == reg3040_l) {
        if (0 == reg3040) {
            SensorReg_Write(0x363c, 0x42);
        } else if (1 == reg3040) {
            SensorReg_Write(0x363c, 0x40);
        }
    }
#endif

    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

static int pCus_init_mipi4lane_HDR_DOL(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    int i, cnt = 0;
    u16 reg3109, reg3109_1, reg3040, reg3040_l;
    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_HDR_DOL_4lane); i++) {
        if (Sensor_init_table_HDR_DOL_4lane[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_HDR_DOL_4lane[i].data);
        } else {
            cnt = 0;
            SENSOR_DMSG("reg =  %x, data = %x\n", Sensor_init_table_HDR_DOL_4lane[i].reg, Sensor_init_table_HDR_DOL_4lane[i].data);
            while (SensorReg_Write(Sensor_init_table_HDR_DOL_4lane[i].reg, Sensor_init_table_HDR_DOL_4lane[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table_HDR_DOL_4lane -> Retry %d...\n", cnt);
                if (cnt >= 10) {
                    return FAIL;
                }
                // usleep(10*1000);
            }
        }
    }

    // SC501AI add logic  2020/08/24 by 06547    unuse
#if 1
    SensorReg_Read(0x3109, &reg3109);
    SensorReg_Read(0x3109, &reg3109_1);
    if (reg3109_1 == reg3109) {
        if (1 == reg3109) {
            SensorReg_Write(0x336d, 0x23);
        } else if (2 == reg3109) {
            SensorReg_Write(0x336d, 0x03);
        }
    }

    SensorReg_Read(0x3040, &reg3040);
    SensorReg_Read(0x3040, &reg3040_l);
    if (reg3040 == reg3040_l) {
        if (0 == reg3040) {
            SensorReg_Write(0x363c, 0x42);
        } else if (1 == reg3040) {
            SensorReg_Write(0x363c, 0x40);
        }
    }
#endif
    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;

    return SUCCESS;
}

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
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0: //"2880x1620@30fps"
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init_linear_5M30fps;
        params->expo.vts = vts_30fps;
        Preview_line_period = 20202;
        params->expo.fps = 30;
        break;
    case 1: //"MAX: 2880x1620@60fps"
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init_linear_60fps;
        params->expo.vts = vts_30fps;
        Preview_line_period = 10101;
        params->expo.fps = 30;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_SetVideoRes_HDR(ms_cus_sensor* handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;

    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0:
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init_mipi4lane_HDR_DOL;
        params->expo.vts = vts_30fps_HDR;
        params->expo.fps = 15;
        params->expo.max_short_exp = 198;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    sen_data = params->tMirror_reg[0].data;
    SENSOR_DMSG("[%s] mirror:%x\r\n", __FUNCTION__, sen_data & 0x66);
    switch (sen_data & 0x66) {
    case 0x00:
        *orit = CUS_ORIT_M0F0;
        break;
    case 0x06:
        *orit = CUS_ORIT_M1F0;
        break;
    case 0x60:
        *orit = CUS_ORIT_M0F1;
        break;
    case 0x66:
        *orit = CUS_ORIT_M1F1;
        break;
    }

    return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{

    sc501ai_params* params = (sc501ai_params*)handle->private_data;

    switch (orit) {
    case CUS_ORIT_M0F0:
        params->tMirror_reg[0].data = 0;
        params->orient_dirty = true;
        break;
    case CUS_ORIT_M1F0:
        params->tMirror_reg[0].data = 6;
        params->orient_dirty = true;
        break;
    case CUS_ORIT_M0F1:
        params->tMirror_reg[0].data = 0x60;
        params->orient_dirty = true;
        break;
    case CUS_ORIT_M1F1:
        params->tMirror_reg[0].data = 0x66;
        params->orient_dirty = true;
        break;
    }
    SENSOR_DMSG("pCus_SetOrien:%x\r\n", orit);

    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    u32 vts = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * max_fps) / fps;
    } else if ((fps >= (min_fps * 1000)) && (fps <= (max_fps * 1000))) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps * (max_fps * 1000)) / fps;
    } else {
        SENSOR_DMSG("[%s] FPS %d out of range.\n", __FUNCTION__, fps);
        return FAIL;
    }

    if (params->expo.line > 2 * (params->expo.vts) - 10) {
        vts = (params->expo.line + 11) / 2;
    } else {
        vts = params->expo.vts;
    }
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;
}

static int pCus_GetFPS_HDR_SEF(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps_HDR * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps_HDR * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS_HDR_SEF(ms_cus_sensor* handle, u32 fps)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * max_fps) / fps;
        params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        return SUCCESS;
    } else if ((fps >= (min_fps * 1000)) && (fps <= (max_fps * 1000))) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * (max_fps * 1000)) / fps;
        params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        // pr_info("[%s]  vts_reg_sef : %x , %x\n\n", __FUNCTION__,vts_reg[0].data,vts_reg[1].data);
        return SUCCESS;
    } else {
        return FAIL;
    }
}

static int pCus_SetFPS_HDR_LEF(ms_cus_sensor* handle, u32 fps)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * max_fps) / fps;
        params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        return SUCCESS;
    } else if ((fps >= (min_fps * 1000)) && (fps <= (max_fps * 1000))) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * (max_fps * 1000)) / fps;
        params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        // pr_info("[%s]  vts_reg_sef : %x , %x\n\n", __FUNCTION__,vts_reg[0].data,vts_reg[1].data);
        return SUCCESS;
    } else {
        return FAIL;
    }
}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:
        break;
    case CUS_FRAME_ACTIVE:
        if (params->orient_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            params->orient_dirty = false;
        }
        if (params->reg_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            params->reg_dirty = false;
        }
        if (params->temperature_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tTemperature_reg, ARRAY_SIZE(temperature_reg));
            params->temperature_dirty = false;
        }
        break;
    default:
        break;
    }
    return SUCCESS;
}

static int pCus_AEStatusNotify_HDR_LEF(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    // sc501ai_params *params = (sc501ai_params *)handle->private_data;
    // ISensorIfAPI *sensor_if = handle->sensor_if_api;
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

static int pCus_AEStatusNotify_HDR_SEF(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    // ISensorIfAPI *sensor_if = handle->sensor_if_api;
    switch (status) {
    case CUS_FRAME_INACTIVE:

        break;
    case CUS_FRAME_ACTIVE:
        if (params->orient_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            params->orient_dirty = false;
        }
        if (params->reg_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg_HDR_SEF, ARRAY_SIZE(expo_reg_HDR_SEF));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg_HDR_SEF, ARRAY_SIZE(gain_reg_HDR_SEF));
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            params->reg_dirty = false;
        }
        if (params->temperature_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tTemperature_reg, ARRAY_SIZE(temperature_reg));
            params->temperature_dirty = false;
        }
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_SetAEUSecs_HDR_LEF(ms_cus_sensor* handle, u32 us)
{
    int i;
    u32 half_lines = 0, dou_lines = 0, vts = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x00 }, // expo[16:8]
        { 0x3e02, 0x10 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, params->tExpo_reg, sizeof(expo_reg));
    dou_lines = (1000 * us) / (Preview_line_period_HDR * 2); // Preview_line_period in ns
    half_lines = 4 * dou_lines;
    if (half_lines < 8)
        half_lines = 8;
    if (half_lines > 2 * (params->expo.vts - params->expo.max_short_exp) - 18) {
        half_lines = 2 * (params->expo.vts - params->expo.max_short_exp) - 18;
    } else
        vts = params->expo.vts;

    half_lines = half_lines << 4;

    params->tExpo_reg[0].data = (half_lines >> 16) & 0x0f;
    params->tExpo_reg[1].data = (half_lines >> 8) & 0xff;
    params->tExpo_reg[2].data = (half_lines >> 0) & 0xf0;

    for (i = 0; i < ARRAY_SIZE(expo_reg); i++) {
        if (params->tExpo_reg[i].data != expo_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
    return SUCCESS;
}

static int pCus_GetAEUSecs_HDR_SEF(ms_cus_sensor* handle, u32* us)
{

    u32 lines = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    lines |= (u32)(params->tExpo_reg_HDR_SEF[0].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_reg_HDR_SEF[1].data & 0xff) << 0;

    *us = (lines * Preview_line_period_HDR) / 1000;

    SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);
    return SUCCESS;
}

static int pCus_SetAEUSecs_HDR_SEF(ms_cus_sensor* handle, u32 us)
{
    int i;
    u32 half_lines = 0, dou_lines = 0, vts = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        { 0x3e04, 0x21 }, // expo[7:0]
        { 0x3e05, 0x00 }, // expo[7:4]
    };
    memcpy(expo_reg_temp, params->tExpo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));

    dou_lines = (1000 * us) / (Preview_line_period_HDR * 2); // Preview_line_period in ns
    half_lines = 4 * dou_lines;
    if (half_lines < 8)
        half_lines = 8;
    if (half_lines > 2 * (params->expo.max_short_exp) - 14) {
        half_lines = 2 * (params->expo.max_short_exp) - 14;
    } else
        vts = params->expo.vts;

    half_lines = half_lines << 4;

    params->tExpo_reg_HDR_SEF[0].data = (half_lines >> 8) & 0xff;
    params->tExpo_reg_HDR_SEF[1].data = (half_lines >> 0) & 0xf0;
    for (i = 0; i < ARRAY_SIZE(expo_reg_HDR_SEF); i++) {
        if (params->tExpo_reg_HDR_SEF[i].data != expo_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
    return SUCCESS;
}

static int pCus_GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    int rc = 0;
    u32 lines = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    lines |= (u32)(params->tExpo_reg[0].data & 0x0f) << 16;
    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_reg[2].data & 0xf0) << 0;
    lines >>= 4;
    *us = (lines * Preview_line_period) / 1000 / 2; // return us

    SENSOR_DMSG("[%s] sensor expo lines/us %d, %dus\n", __FUNCTION__, lines, *us);
    return rc;
}

static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    int i;
    u32 half_lines = 0, vts = 0;
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x00 }, // expo[16:8]
        { 0x3e02, 0x10 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, params->tExpo_reg, sizeof(expo_reg));

    half_lines = (1000 * us * 2) / Preview_line_period; // Preview_line_period in ns
    if (half_lines <= 3)
        half_lines = 3;
    if (half_lines > 2 * (params->expo.vts) - 10) {
        vts = (half_lines + 11) / 2;
    } else
        vts = params->expo.vts;
    params->expo.line = half_lines;

    SENSOR_DMSG("[%s] us %d, half_lines %d, vts %d\n", __FUNCTION__, us, half_lines, params->expo.vts);

    half_lines = half_lines << 4;
    params->tExpo_reg[0].data = (half_lines >> 16) & 0x0f;
    params->tExpo_reg[1].data = (half_lines >> 8) & 0xff;
    params->tExpo_reg[2].data = (half_lines >> 0) & 0xf0;
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;

    for (i = 0; i < ARRAY_SIZE(expo_reg); i++) {
        if (params->tExpo_reg[i].data != expo_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    int rc = 0;

    return rc;
}

static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain)
{

    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u8 i = 0, Coarse_gain = 1, DIG_gain = 1;
    u32 Dcg_gainx100 = 1, ANA_Fine_gainx64 = 1, DIG_Fine_gainx1000 = 1;
    u8 Dcg_gain_reg = 0, Coarse_gain_reg = 0, DIG_gain_reg = 0, ANA_Fine_gain_reg = 0x40, DIG_Fine_gain_reg = 0x80;

    I2C_ARRAY gain_reg_temp[] = {
        { 0x3e06, 0x00 },
        { 0x3e07, 0x80 },
        { 0x3e08, 0x00 | 0x03 },
        { 0x3e09, 0x40 },
    };
#if ENABLE_NR
    I2C_ARRAY temperature_reg_temp[] = {
        { 0x5799, 0x00 },
    };
#endif
    memcpy(gain_reg_temp, params->tGain_reg, sizeof(gain_reg_temp));
    memcpy(temperature_reg_temp, params->tTemperature_reg, sizeof(temperature_reg_temp));

    if (gain <= 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAXGAIN * 1024) {
        gain = SENSOR_MAXGAIN * 1024;
    }

    if (gain < 1552) // start again  1.516 * 1024
    {
        Dcg_gainx100 = 1000;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 0;
        Coarse_gain_reg = 0x03;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 3104) // 3.032 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x23;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 6209) // 6.064 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x27;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 12419) // 12.128 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 4;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x2f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 24644) // 24.067 * 1024 // end again
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    }
#if 1
    else if (gain <= 24644 * 2) // start dgain
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 1;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 4) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 2;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x1;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 8) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 4;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x3;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 16) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 8;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x7;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 782466) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 16;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0xF;
        ANA_Fine_gain_reg = 0x7f;
    }
#endif

    if (gain == 1552) {
        ANA_Fine_gain_reg = 0x40;
    } else if (gain <= 24644) {
        ANA_Fine_gain_reg = abs(1000 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
    } else {
        DIG_Fine_gain_reg = abs(1000 * gain / (Dcg_gainx100 * DIG_gain) / ANA_Fine_gainx64); //*8000/coarse_gain = *1000
    }
    params->tGain_reg[3].data = ANA_Fine_gain_reg;
    params->tGain_reg[2].data = Coarse_gain_reg;
    params->tGain_reg[1].data = DIG_Fine_gain_reg;
    params->tGain_reg[0].data = DIG_gain_reg & 0xF;

    for (i = 0; i < ARRAY_SIZE(params->tGain_reg); i++) {
        if (params->tGain_reg[i].data != gain_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
#if 1
    if (gain >= 30 * 1024) {
        params->tTemperature_reg[0].data = 0x07;
    } else if (gain <= 20 * 1024) {
        params->tTemperature_reg[0].data = 0x00;
    }
    for (i = 0; i < ARRAY_SIZE(temperature_reg_temp); i++) {
        if (params->tTemperature_reg[i].data != temperature_reg_temp[i].data) {
            params->temperature_dirty = true;
            break;
        }
    }
#endif

    return SUCCESS;
}

static int pCus_SetAEGain_HDR_SEF(ms_cus_sensor* handle, u32 gain)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u8 i = 0, Coarse_gain = 1, DIG_gain = 1;
    u32 Dcg_gainx100 = 1, ANA_Fine_gainx64 = 1, DIG_Fine_gainx1000 = 1;
    u8 Dcg_gain_reg = 0, Coarse_gain_reg = 0, DIG_gain_reg = 0, ANA_Fine_gain_reg = 0x40, DIG_Fine_gain_reg = 0x80;

    I2C_ARRAY gain_reg_temp[] = {
        { 0x3e10, 0x00 },
        { 0x3e11, 0x80 },
        { 0x3e12, 0x00 | 0x03 },
        { 0x3e13, 0x40 },
    };
    memcpy(gain_reg_temp, params->tGain_reg_HDR_SEF, sizeof(gain_reg_temp));

    if (gain <= 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAXGAIN * 1024) {
        gain = SENSOR_MAXGAIN * 1024;
    }

    if (gain < 1552) // start again  1.516 * 1024
    {
        Dcg_gainx100 = 1000;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 0;
        Coarse_gain_reg = 0x03;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 3104) // 3.032 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x23;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 6209) // 6.064 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x27;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 12419) // 12.128 * 1024
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 4;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x2f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain <= 24644) // 24.067 * 1024 // end again
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 1;
        DIG_Fine_gainx1000 = 1000;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    }
#if 1
    else if (gain <= 24644 * 2) // start dgain
    {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 1;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 4) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 2;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x1;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 8) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 4;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x3;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 24644 * 16) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 8;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x7;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= 782466) {
        Dcg_gainx100 = 1516;
        Coarse_gain = 8;
        DIG_gain = 16;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0xF;
        ANA_Fine_gain_reg = 0x7f;
    }
#endif

    if (gain == 1552) // || gain == 1552)
    {
        ANA_Fine_gain_reg = 0x40;
    } else if (gain <= 24644) {
        ANA_Fine_gain_reg = abs(1000 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
    } else {
        DIG_Fine_gain_reg = abs(1000 * gain / (Dcg_gainx100 * DIG_gain) / ANA_Fine_gainx64);
    }

    params->tGain_reg_HDR_SEF[3].data = ANA_Fine_gain_reg;
    params->tGain_reg_HDR_SEF[2].data = Coarse_gain_reg;
    params->tGain_reg_HDR_SEF[1].data = DIG_Fine_gain_reg;
    params->tGain_reg_HDR_SEF[0].data = DIG_gain_reg & 0xF;

    for (i = 0; i < ARRAY_SIZE(gain_reg_HDR_SEF); i++) {
        if (params->tGain_reg_HDR_SEF[i].data != gain_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
#if 1
    if (gain >= 16 * 1024) {
        params->tTemperature_reg[0].data = 0x07;
    } else if (gain <= 10 * 1024) {
        params->tTemperature_reg[0].data = 0x00;
    }
    for (i = 0; i < ARRAY_SIZE(temperature_reg); i++) {
        if (params->tTemperature_reg[i].data != temperature_reg[i].data) {
            params->temperature_dirty = true;
            break;
        }
    }
#endif

    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 30;
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1024;
    *max = SENSOR_MAXGAIN * 1024;
    return SUCCESS;
}

static int sc501ai_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = (Preview_line_period * 3) / 2;
    info->step = Preview_line_period / 2;
    return SUCCESS;
}

static int sc501ai_GetShutterInfo_HDR_LEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS; /// 12;
    info->min = Preview_line_period_HDR * 4; // 2
    info->step = Preview_line_period_HDR * 2; // 2
    return SUCCESS;
}

static int sc501ai_GetShutterInfo_HDR_SEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    info->max = (Preview_line_period_HDR * params->expo.max_short_exp);
    info->min = Preview_line_period_HDR * 4; // 2
    info->step = Preview_line_period_HDR * 2; // 2
    return SUCCESS;
}

static int pCus_poweron_HDR_LEF(ms_cus_sensor* handle, u32 idx)
{
    return SUCCESS;
}

static int pCus_poweroff_HDR_LEF(ms_cus_sensor* handle, u32 idx)
{
    return SUCCESS;
}
static int pCus_GetSensorID_HDR_LEF(ms_cus_sensor* handle, u32* id)
{
    *id = 0;
    return SUCCESS;
}
static int pCus_init_HDR_LEF(ms_cus_sensor* handle)
{
    return SUCCESS;
}

static int pCus_GetFPS_HDR_LEF(ms_cus_sensor* handle)
{
    sc501ai_params* params = (sc501ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps_HDR * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps_HDR * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_setCaliData_gain_linearity_HDR_LEF(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    return SUCCESS;
}
static int pCus_SetAEGain_cal_HDR_LEF(ms_cus_sensor* handle, u32 gain)
{
    return SUCCESS;
}
static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
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
    sc501ai_params* params;
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
    params = (sc501ai_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tGain_reg_HDR_SEF, gain_reg_HDR_SEF, sizeof(gain_reg_HDR_SEF));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tExpo_reg_HDR_SEF, expo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
#if ENABLE_NR
    memcpy(params->tTemperature_reg, temperature_reg, sizeof(temperature_reg));
#endif
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "sc501ai_MIPI");

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
        handle->video_res_supported.res[res].width = sc501ai_mipi_linear[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc501ai_mipi_linear[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc501ai_mipi_linear[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc501ai_mipi_linear[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc501ai_mipi_linear[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc501ai_mipi_linear[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc501ai_mipi_linear[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc501ai_mipi_linear[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc501ai_mipi_linear[res].senstr.strResDesc);
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
    handle->pCus_sensor_init = pCus_init_linear_5M30fps;

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
    handle->pCus_sensor_SetPatternMode = sc501ai_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs;
    handle->pCus_sensor_GetAEGain = pCus_GetAEGain;

    handle->pCus_sensor_SetAEGain = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = pCus_GetAEMinMaxUSecs;
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    // sensor calibration
    handle->pCus_sensor_SetAEGain_cal = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = sc501ai_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.line = 1000;
    params->reg_dirty = false;
    params->orient_dirty = false;
    params->temperature_dirty = false;

    return SUCCESS;
}

int cus_camsensor_init_handle_HDR_SEF(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc501ai_params* params = NULL;
    int res;

    cus_camsensor_init_handle(drv_handle);
    params = (sc501ai_params*)handle->private_data;

    sprintf(handle->model_id, "sc501ai_MIPI_HDR_SEF");

    handle->bayer_id = SENSOR_BAYERID_HDR;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->data_prec = SENSOR_DATAPREC_HDR;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM_HDR;
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE_HDR;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_DCG;

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////
    handle->video_res_supported.ulcur_res = 0;
    for (res = 0; res < HDR_RES_END; res++) {
        handle->video_res_supported.num_res = res + 1;
        handle->video_res_supported.res[res].width = sc501ai_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc501ai_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc501ai_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc501ai_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc501ai_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc501ai_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc501ai_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc501ai_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc501ai_mipi_hdr[res].senstr.strResDesc);
    }

    handle->pCus_sensor_SetVideoRes = pCus_SetVideoRes_HDR;
    handle->mclk = Preview_MCLK_SPEED_HDR;

    handle->pCus_sensor_init = pCus_init_mipi4lane_HDR_DOL;
    handle->pCus_sensor_poweron = pCus_poweron; // Need to check
    handle->pCus_sensor_poweroff = pCus_poweroff; // Need to check

    handle->pCus_sensor_GetFPS = pCus_GetFPS_HDR_SEF;
    handle->pCus_sensor_SetFPS = pCus_SetFPS_HDR_SEF;
    handle->pCus_sensor_GetOrien = pCus_GetOrien;
    handle->pCus_sensor_SetOrien = pCus_SetOrien;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify_HDR_SEF;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs_HDR_SEF;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs_HDR_SEF;
    handle->pCus_sensor_GetAEGain = pCus_GetAEGain;
    handle->pCus_sensor_SetAEGain = pCus_SetAEGain_HDR_SEF;
    handle->pCus_sensor_GetShutterInfo = sc501ai_GetShutterInfo_HDR_SEF;

    params->expo.vts = vts_30fps_HDR;
    params->expo.line = 1000;
    params->expo.fps = 15;
    params->expo.max_short_exp = 198;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 1; // Short frame

    handle->ae_gain_delay = 2;
    handle->ae_shutter_delay = 2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2;

    return SUCCESS;
}

int cus_camsensor_init_handle_HDR_LEF(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc501ai_params* params;
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
    params = (sc501ai_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tGain_reg_HDR_SEF, gain_reg_HDR_SEF, sizeof(gain_reg_HDR_SEF));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tExpo_reg_HDR_SEF, expo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
#if ENABLE_NR
    memcpy(params->tTemperature_reg, temperature_reg, sizeof(temperature_reg));
#endif
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "sc501ai_MIPI_HDR_LEF");

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
        handle->video_res_supported.res[res].width = sc501ai_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc501ai_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc501ai_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc501ai_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc501ai_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc501ai_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc501ai_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc501ai_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc501ai_mipi_hdr[res].senstr.strResDesc);
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

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay = 2;
    handle->ae_shutter_delay = 2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 2;

    /// calibration
    handle->sat_mingain = g_sensor_ae_min_gain;

    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = pCus_init_HDR_LEF;

    handle->pCus_sensor_poweron = pCus_poweron_HDR_LEF;
    handle->pCus_sensor_poweroff = pCus_poweroff_HDR_LEF;

    // Normal
    handle->pCus_sensor_GetSensorID = pCus_GetSensorID_HDR_LEF;

    handle->pCus_sensor_GetVideoResNum = NULL;
    handle->pCus_sensor_GetVideoRes = NULL;
    handle->pCus_sensor_GetCurVideoRes = NULL;
    handle->pCus_sensor_SetVideoRes = NULL;

    handle->pCus_sensor_GetOrien = pCus_GetOrien;
    handle->pCus_sensor_SetOrien = pCus_SetOrien;
    handle->pCus_sensor_GetFPS = pCus_GetFPS_HDR_LEF;
    handle->pCus_sensor_SetFPS = pCus_SetFPS_HDR_LEF;
    handle->pCus_sensor_SetPatternMode = sc501ai_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify_HDR_LEF;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs_HDR_LEF;
    handle->pCus_sensor_GetAEGain = pCus_GetAEGain;

    handle->pCus_sensor_SetAEGain = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;

    // sensor calibration
    handle->pCus_sensor_SetAEGain_cal = pCus_SetAEGain_cal_HDR_LEF;
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity_HDR_LEF;
    handle->pCus_sensor_GetShutterInfo = sc501ai_GetShutterInfo_HDR_LEF;
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    params->expo.vts = vts_30fps_HDR;
    params->expo.fps = 15;
    params->expo.max_short_exp = 198;
    params->reg_dirty = false;
    params->orient_dirty = false;
    params->temperature_dirty = false;
    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(sc501ai,
    cus_camsensor_init_handle,
    cus_camsensor_init_handle_HDR_SEF,
    cus_camsensor_init_handle_HDR_LEF,
    sc501ai_params);
