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

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(SC450AI);

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

#define R_GAIN_REG 1
#define G_GAIN_REG 2
#define B_GAIN_REG 3

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
#define SENSOR_MAXGAIN (60523 * 15875) / 1000000 // max sensor gain, a-gain*conversion-gain*d-gain
#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_MCLK_SPEED_HDR CUS_CMU_CLK_27MHZ

u32 Preview_line_period;
u32 vts_30fps;
#define Preview_line_period_HDR 10684
#define vts_30fps_HDR 3120
#define Preview_WIDTH 2688 // resolution Width when preview
#define Preview_HEIGHT 1520 // resolution Height when preview
#define Preview_MAX_FPS 30 // 25                //fastest preview FPS
#define Preview_MAX_FPS_HDR 30 // 25                //fastest preview FPS
#define Preview_MIN_FPS 3 // slowest preview FPS
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
} sc450ai_mipi_linear[] = {
    { LINEAR_RES_1, { 2688, 1520, 3, 30 }, { 0, 0, 2688, 1520 }, { "2688x1520@30fps" } },
    { LINEAR_RES_2, { 2688, 1520, 3, 60 }, { 0, 0, 2688, 1520 }, { "2688x1520@60fps" } },
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
} sc450ai_mipi_hdr[] = {
    { HDR_RES_1, { 2688, 1520, 3, 30 }, { 0, 0, 2688, 1520 }, { "2688x1520@30fps_HDR" } }, // Modify it
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
static int g_sensor_ae_min_gain = 1024;

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
    I2C_ARRAY tVts_reg[9];
    I2C_ARRAY tVts_reg_HDR[9];
    I2C_ARRAY tGain_reg[4];
    I2C_ARRAY tGain_reg_HDR_SEF[4];
    I2C_ARRAY tExpo_reg[3];
    I2C_ARRAY tExpo_reg_HDR_SEF[2];
    I2C_ARRAY tMax_short_exp_reg[2];
    I2C_ARRAY tMirror_reg[1];
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool orient_dirty;
    bool reg_dirty;
    bool vts_reg_dirty;
} sc450ai_params;

// set sensor ID address and data,
const static I2C_ARRAY Sensor_id_table[] = {
    { 0x3107, 0xbd },
    { 0x3108, 0x2f },
};

const static I2C_ARRAY Sensor_init_table_4M30fps[] = {
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301c, 0x78 },
    { 0x301f, 0x01 },
    { 0x302e, 0x00 },
    { 0x3208, 0x0a },
    { 0x3209, 0x80 },
    { 0x320a, 0x05 },
    { 0x320b, 0xf0 },
    { 0x320c, 0x02 },
    { 0x320d, 0xee },
    { 0x320e, 0x06 },
    { 0x320f, 0x18 },
    { 0x3214, 0x11 },
    { 0x3215, 0x11 },
    { 0x3220, 0x00 },
    { 0x3223, 0xc0 },
    { 0x325f, 0x44 },
    { 0x3274, 0x09 },
    { 0x3280, 0x01 },
    { 0x3301, 0x07 },
    { 0x3306, 0x22 },
    { 0x330b, 0x5a },
    { 0x330e, 0x18 },
    { 0x3315, 0x00 },
    { 0x335d, 0x60 },
    { 0x3364, 0x56 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x07 },
    { 0x3394, 0x10 },
    { 0x3395, 0x18 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x10 },
    { 0x339a, 0x13 },
    { 0x339b, 0x15 },
    { 0x339c, 0x18 },
    { 0x33af, 0x18 },
    { 0x360f, 0x13 },
    { 0x3621, 0xec },
    { 0x3622, 0x00 },
    { 0x3625, 0x0b },
    { 0x3627, 0xa0 },
    { 0x3630, 0x90 },
    { 0x3633, 0x56 },
    { 0x3637, 0x1d },
    { 0x3638, 0x12 },
    { 0x363e, 0x08 },
    { 0x3670, 0x4a },
    { 0x3671, 0xe0 },
    { 0x3672, 0xe0 },
    { 0x3673, 0xe1 },
    { 0x3674, 0xb0 },
    { 0x3675, 0xc0 },
    { 0x3676, 0x87 },
    { 0x367a, 0x48 },
    { 0x367b, 0x58 },
    { 0x367c, 0x40 },
    { 0x367d, 0x48 },
    { 0x3690, 0x34 },
    { 0x3691, 0x43 },
    { 0x3692, 0x54 },
    { 0x3699, 0x03 },
    { 0x369a, 0x07 },
    { 0x369b, 0x0f },
    { 0x369c, 0x40 },
    { 0x369d, 0x48 },
    { 0x36a2, 0x08 },
    { 0x36a3, 0x58 },
    { 0x36b0, 0x54 },
    { 0x36b1, 0x74 },
    { 0x36b2, 0x34 },
    { 0x36b3, 0x48 },
    { 0x36b4, 0x78 },
    { 0x36b7, 0xa0 },
    { 0x36b8, 0xa0 },
    { 0x36b9, 0x20 },
    { 0x36bd, 0x40 },
    { 0x36be, 0x48 },
    { 0x36d0, 0x20 },
    { 0x36e0, 0x08 },
    { 0x36e1, 0x08 },
    { 0x36e2, 0x12 },
    { 0x36e3, 0x48 },
    { 0x36e4, 0x78 },
    { 0x36ec, 0x53 },
    { 0x36fc, 0x00 },
    { 0x3907, 0x00 },
    { 0x3908, 0x41 },
    { 0x391e, 0xf1 },
    { 0x3e01, 0xc2 },
    { 0x3e02, 0x60 },
    { 0x3e03, 0x0b },
    { 0x3e08, 0x03 },
    { 0x3e1b, 0x2a },
    { 0x4509, 0x20 },
    { 0x4837, 0x2c },
    { 0x5001, 0x44 },
    { 0x5799, 0x06 },
    { 0x59e0, 0xfe },
    { 0x59e1, 0x40 },
    { 0x59e2, 0x3f },
    { 0x59e3, 0x38 },
    { 0x59e4, 0x30 },
    { 0x59e5, 0x3f },
    { 0x59e6, 0x38 },
    { 0x59e7, 0x30 },
    { 0x59e8, 0x3f },
    { 0x59e9, 0x3c },
    { 0x59ea, 0x38 },
    { 0x59eb, 0x3f },
    { 0x59ec, 0x3c },
    { 0x59ed, 0x38 },
    { 0x59ee, 0xfe },
    { 0x59ef, 0x40 },
    { 0x59f4, 0x3f },
    { 0x59f5, 0x38 },
    { 0x59f6, 0x30 },
    { 0x59f7, 0x3f },
    { 0x59f8, 0x38 },
    { 0x59f9, 0x30 },
    { 0x59fa, 0x3f },
    { 0x59fb, 0x3c },
    { 0x59fc, 0x38 },
    { 0x59fd, 0x3f },
    { 0x59fe, 0x3c },
    { 0x59ff, 0x38 },
    { 0x36e9, 0x44 },
    { 0x36f9, 0x20 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

const static I2C_ARRAY Sensor_init_table_4M60fps[] = {
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301c, 0x78 },
    { 0x301f, 0x02 },
    { 0x302e, 0x00 },
    { 0x3208, 0x0a },
    { 0x3209, 0x80 },
    { 0x320a, 0x05 },
    { 0x320b, 0xf0 },
    { 0x320c, 0x03 },
    { 0x320d, 0xa8 },
    { 0x320e, 0x06 },
    { 0x320f, 0x18 },
    { 0x3214, 0x11 },
    { 0x3215, 0x11 },
    { 0x3220, 0x00 },
    { 0x3223, 0xc0 },
    { 0x325f, 0x44 },
    { 0x3274, 0x09 },
    { 0x3280, 0x01 },
    { 0x3301, 0x09 },
    { 0x3306, 0x24 },
    { 0x3309, 0x60 },
    { 0x330b, 0x60 },
    { 0x3315, 0x00 },
    { 0x331f, 0x59 },
    { 0x335d, 0x60 },
    { 0x3364, 0x56 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x0b },
    { 0x3394, 0x10 },
    { 0x3395, 0x18 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x10 },
    { 0x339a, 0x13 },
    { 0x339b, 0x15 },
    { 0x339c, 0x18 },
    { 0x33af, 0x18 },
    { 0x360f, 0x13 },
    { 0x3621, 0xec },
    { 0x3627, 0xa0 },
    { 0x3630, 0x90 },
    { 0x3633, 0x56 },
    { 0x3637, 0x1d },
    { 0x3638, 0x0a },
    { 0x363e, 0x08 },
    { 0x3670, 0x4a },
    { 0x3671, 0xe0 },
    { 0x3672, 0xe0 },
    { 0x3673, 0xe1 },
    { 0x3674, 0xb0 },
    { 0x3675, 0xb0 },
    { 0x3676, 0x87 },
    { 0x367a, 0x48 },
    { 0x367b, 0x58 },
    { 0x367c, 0x40 },
    { 0x367d, 0x48 },
    { 0x3690, 0x34 },
    { 0x3691, 0x43 },
    { 0x3692, 0x44 },
    { 0x3699, 0x03 },
    { 0x369a, 0x07 },
    { 0x369b, 0x0f },
    { 0x369c, 0x40 },
    { 0x369d, 0x48 },
    { 0x36a2, 0x08 },
    { 0x36a3, 0x58 },
    { 0x36b0, 0x54 },
    { 0x36b1, 0x74 },
    { 0x36b2, 0x34 },
    { 0x36b3, 0x48 },
    { 0x36b4, 0x78 },
    { 0x36b7, 0xa0 },
    { 0x36b8, 0xa0 },
    { 0x36b9, 0x20 },
    { 0x36bd, 0x40 },
    { 0x36be, 0x48 },
    { 0x36d0, 0x20 },
    { 0x36e0, 0x08 },
    { 0x36e1, 0x08 },
    { 0x36e2, 0x12 },
    { 0x36e3, 0x48 },
    { 0x36e4, 0x78 },
    { 0x36e9, 0x44 },
    { 0x36f9, 0x20 },
    { 0x36fa, 0x0d },
    { 0x36fb, 0xa4 },
    { 0x36fc, 0x00 },
    { 0x36fd, 0x24 },
    { 0x3907, 0x00 },
    { 0x3908, 0x41 },
    { 0x391e, 0x01 },
    { 0x3e01, 0xc2 },
    { 0x3e02, 0x60 },
    { 0x3e03, 0x0b },
    { 0x3e08, 0x03 },
    { 0x3e1b, 0x2a },
    { 0x440e, 0x02 },
    { 0x4509, 0x20 },
    { 0x4837, 0x16 },
    { 0x5000, 0x0e },
    { 0x5001, 0x44 },
    { 0x5799, 0x06 },
    { 0x59e0, 0xfe },
    { 0x59e1, 0x40 },
    { 0x59e2, 0x3f },
    { 0x59e3, 0x38 },
    { 0x59e4, 0x30 },
    { 0x59e5, 0x3f },
    { 0x59e6, 0x38 },
    { 0x59e7, 0x30 },
    { 0x59e8, 0x3f },
    { 0x59e9, 0x3c },
    { 0x59ea, 0x38 },
    { 0x59eb, 0x3f },
    { 0x59ec, 0x3c },
    { 0x59ed, 0x38 },
    { 0x59ee, 0xfe },
    { 0x59ef, 0x40 },
    { 0x59f4, 0x3f },
    { 0x59f5, 0x38 },
    { 0x59f6, 0x30 },
    { 0x59f7, 0x3f },
    { 0x59f8, 0x38 },
    { 0x59f9, 0x30 },
    { 0x59fa, 0x3f },
    { 0x59fb, 0x3c },
    { 0x59fc, 0x38 },
    { 0x59fd, 0x3f },
    { 0x59fe, 0x3c },
    { 0x59ff, 0x38 },
    { 0x36e9, 0x44 },
    { 0x36f9, 0x20 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

const static I2C_ARRAY Sensor_init_table_HDR_DOL_4lane[] = {
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301c, 0x78 },
    { 0x301f, 0x03 },
    { 0x302e, 0x00 },
    { 0x3208, 0x0a },
    { 0x3209, 0x80 },
    { 0x320a, 0x05 },
    { 0x320b, 0xf0 },
    { 0x320c, 0x03 },
    { 0x320d, 0xa8 },
    { 0x320e, 0x0c },
    { 0x320f, 0x30 },
    { 0x3214, 0x11 },
    { 0x3215, 0x11 },
    { 0x3220, 0x00 },
    { 0x3223, 0xc0 },
    { 0x3250, 0xff },
    { 0x325f, 0x44 },
    { 0x3274, 0x09 },
    { 0x3280, 0x01 },
    { 0x3281, 0x01 },
    { 0x3301, 0x09 },
    { 0x3306, 0x24 },
    { 0x3309, 0x60 },
    { 0x330b, 0x60 },
    { 0x3314, 0x94 },
    { 0x3315, 0x00 },
    { 0x331f, 0x59 },
    { 0x335d, 0x60 },
    { 0x3364, 0x56 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x0b },
    { 0x3394, 0x10 },
    { 0x3395, 0x18 },
    { 0x3396, 0x08 },
    { 0x3397, 0x18 },
    { 0x3398, 0x38 },
    { 0x3399, 0x10 },
    { 0x339a, 0x13 },
    { 0x339b, 0x15 },
    { 0x339c, 0x18 },
    { 0x33af, 0x18 },
    { 0x3400, 0x04 },
    { 0x3410, 0x04 },
    { 0x360f, 0x13 },
    { 0x3621, 0xec },
    { 0x3627, 0xa0 },
    { 0x3630, 0x90 },
    { 0x3633, 0x56 },
    { 0x3637, 0x1d },
    { 0x3638, 0x0a },
    { 0x363e, 0x08 },
    { 0x3670, 0x4a },
    { 0x3671, 0xe0 },
    { 0x3672, 0xe0 },
    { 0x3673, 0xe1 },
    { 0x3674, 0xb0 },
    { 0x3675, 0xb0 },
    { 0x3676, 0x87 },
    { 0x367a, 0x48 },
    { 0x367b, 0x58 },
    { 0x367c, 0x40 },
    { 0x367d, 0x48 },
    { 0x3690, 0x34 },
    { 0x3691, 0x43 },
    { 0x3692, 0x44 },
    { 0x3699, 0x03 },
    { 0x369a, 0x07 },
    { 0x369b, 0x0f },
    { 0x369c, 0x40 },
    { 0x369d, 0x48 },
    { 0x36a2, 0x08 },
    { 0x36a3, 0x58 },
    { 0x36b0, 0x54 },
    { 0x36b1, 0x74 },
    { 0x36b2, 0x34 },
    { 0x36b3, 0x48 },
    { 0x36b4, 0x78 },
    { 0x36b7, 0xa0 },
    { 0x36b8, 0xa0 },
    { 0x36b9, 0x20 },
    { 0x36bd, 0x40 },
    { 0x36be, 0x48 },
    { 0x36d0, 0x20 },
    { 0x36e0, 0x08 },
    { 0x36e1, 0x08 },
    { 0x36e2, 0x12 },
    { 0x36e3, 0x48 },
    { 0x36e4, 0x78 },
    { 0x36fa, 0x0d },
    { 0x36fb, 0xa4 },
    { 0x36fc, 0x00 },
    { 0x36fd, 0x24 },
    { 0x3907, 0x00 },
    { 0x3908, 0x41 },
    { 0x391e, 0x01 },
    { 0x3e00, 0x01 },
    { 0x3e01, 0x6c },
    { 0x3e02, 0x00 },
    { 0x3e03, 0x0b },
    { 0x3e04, 0x16 },
    { 0x3e05, 0xc0 },
    { 0x3e06, 0x00 },
    { 0x3e07, 0x80 },
    { 0x3e08, 0x03 },
    { 0x3e09, 0x40 },
    { 0x3e10, 0x00 },
    { 0x3e11, 0x80 },
    { 0x3e12, 0x03 },
    { 0x3e13, 0x40 },
    { 0x3e1b, 0x2a },
    { 0x3e22, 0x00 },
    { 0x3e23, 0x00 },
    { 0x3e24, 0xbc },
    { 0x4509, 0x20 },
    { 0x4837, 0x16 },
    { 0x4853, 0xf8 },
    { 0x5001, 0x44 },
    { 0x5011, 0x80 },
    { 0x5799, 0x06 },
    { 0x59e0, 0xfe },
    { 0x59e1, 0x40 },
    { 0x59e2, 0x3f },
    { 0x59e3, 0x38 },
    { 0x59e4, 0x30 },
    { 0x59e5, 0x3f },
    { 0x59e6, 0x38 },
    { 0x59e7, 0x30 },
    { 0x59e8, 0x3f },
    { 0x59e9, 0x3c },
    { 0x59ea, 0x38 },
    { 0x59eb, 0x3f },
    { 0x59ec, 0x3c },
    { 0x59ed, 0x38 },
    { 0x59ee, 0xfe },
    { 0x59ef, 0x40 },
    { 0x59f4, 0x3f },
    { 0x59f5, 0x38 },
    { 0x59f6, 0x30 },
    { 0x59f7, 0x3f },
    { 0x59f8, 0x38 },
    { 0x59f9, 0x30 },
    { 0x59fa, 0x3f },
    { 0x59fb, 0x3c },
    { 0x59fc, 0x38 },
    { 0x59fd, 0x3f },
    { 0x59fe, 0x3c },
    { 0x59ff, 0x38 },
    { 0x36e9, 0x44 },
    { 0x36f9, 0x20 },
    { 0x0100, 0x01 },
    { 0xffff, 0x0a },
};

I2C_ARRAY mirror_reg[] = {
    { 0x3221, 0x00 }, // mirror[2:1], flip[6:5]
};

const static I2C_ARRAY gain_reg[] = {
    { 0x3e06, 0x00 },
    { 0x3e07, 0x80 },
    { 0x3e08, 0x00 | 0x03 },
    { 0x3e09, 0x40 }, // low bit, 0x40 - 0x7f, step 1/64
};

const static I2C_ARRAY gain_reg_HDR_SEF[] = {
    { 0x3e10, 0x00 },
    { 0x3e11, 0x80 },
    { 0x3e12, 0x00 | 0x03 },
    { 0x3e13, 0x40 }, // low bit, 0x40 - 0x7f, step 1/64
};

const static I2C_ARRAY expo_reg[] = {
    { 0x3e00, 0x00 }, // expo [20:17]
    { 0x3e01, 0x30 }, // expo[16:8]
    { 0x3e02, 0x00 }, // expo[7:0], [3:0] fraction of line
};

const static I2C_ARRAY expo_reg_HDR_SEF[] = {
    { 0x3e04, 0x16 }, // expo[16:8]
    { 0x3e05, 0xc0 }, // expo[7:0], [3:0] fraction of line
};

const static I2C_ARRAY max_short_exp_reg[] = {
    { 0x3e23, 0x00 }, // expo[16:8]
    { 0x3e24, 0xbc }, // expo[7:0], [3:0] fraction of line
};

const static I2C_ARRAY vts_reg[] = {
    { 0x320e, 0x05 },
    { 0x320f, 0xdc },
    { 0x3800, 0x00 }, // group hold
    { 0x3368, 0x05 },
    { 0x3369, 0xdc },
    { 0x336a, 0x0b },
    { 0x336b, 0xb8 },
    { 0x3800, 0x10 }, //
    { 0x3800, 0x40 }, //
};

const static I2C_ARRAY vts_reg_HDR[] = {
    { 0x320e, 0x0c },
    { 0x320f, 0x30 },
    { 0x3800, 0x00 }, // group hold
    { 0x3368, 0x0b },
    { 0x3369, 0x04 },
    { 0x336a, 0x16 },
    { 0x336b, 0x08 },
    { 0x3800, 0x10 }, //
    { 0x3800, 0x40 }, //
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
#define SENSOR_NAME sc450ai
*/
#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

// int cus_camsensor_release_handle(ms_cus_sensor *handle);

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
    // sc450ai_params *params = (sc450ai_params *)handle->private_data;
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

static int sc450ai_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{

    return SUCCESS;
}

static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init_linear_4M30fps(ms_cus_sensor* handle)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    // SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i, cnt;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_4M30fps); i++) {
        if (Sensor_init_table_4M30fps[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_4M30fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_4M30fps[i].reg, Sensor_init_table_4M30fps[i].data) != SUCCESS) {
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

    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

static int pCus_init_linear_4M60fps(ms_cus_sensor* handle)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    // SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i, cnt;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_4M60fps); i++) {
        if (Sensor_init_table_4M60fps[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_4M60fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_4M60fps[i].reg, Sensor_init_table_4M60fps[i].data) != SUCCESS) {
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

    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

static int pCus_init_mipi4lane_HDR_DOL(ms_cus_sensor* handle)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    int i, cnt = 0;

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
                    // printf("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, handle->orient);
    params->tVts_reg_HDR[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg_HDR[1].data = (params->expo.vts >> 0) & 0x00ff;

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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0: //"2688x1520@30fps"
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init_linear_4M30fps;
        vts_30fps = 1560; // 1500
        params->expo.vts = vts_30fps;
        params->expo.fps = 30;
        Preview_line_period = 21367;
        break;
    case 1: //"2688x1520@60fps"
        handle->video_res_supported.ulcur_res = 1;
        handle->pCus_sensor_init = pCus_init_linear_4M60fps;
        vts_30fps = 1560; // 1500
        params->expo.vts = vts_30fps;
        params->expo.fps = 30; // maxfps 60, default 30
        Preview_line_period = 10683;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_SetVideoRes_HDR(ms_cus_sensor* handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    sc450ai_params* params = (sc450ai_params*)handle->private_data;

    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0:
        handle->video_res_supported.ulcur_res = 0;
        if (handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num == 1) {
            handle->pCus_sensor_init = pCus_init_mipi4lane_HDR_DOL;
        }
        params->expo.vts = vts_30fps_HDR;
        params->expo.fps = 30;
        params->expo.max_short_exp = 188;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;

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
    default:
        break;
    }

    handle->orient = orit;
    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor* handle)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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

    if (params->expo.line > 2 * (params->expo.vts) - 8) {
        vts = (params->expo.line + 8) / 2;
    } else {
        vts = params->expo.vts;
    }
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->tVts_reg[3].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[4].data = (vts >> 0) & 0x00ff;
    params->tVts_reg[5].data = ((2 * vts) >> 8) & 0x00ff;
    params->tVts_reg[6].data = ((2 * vts) >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;
}

static int pCus_GetFPS_HDR(ms_cus_sensor* handle)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[0].data << 8) | (params->tVts_reg[1].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps_HDR * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps_HDR * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS_HDR(ms_cus_sensor* handle, u32 fps)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;

    if (fps >= min_fps && fps <= max_fps) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * max_fps) / fps;
    } else if ((fps >= (min_fps * 1000)) && (fps <= (max_fps * 1000))) {
        params->expo.fps = fps;
        params->expo.vts = (vts_30fps_HDR * (max_fps * 1000)) / fps;
    } else {
        return FAIL;
    }

    params->expo.max_short_exp = (((params->expo.vts) / 17 - 1) >> 1) << 1;
    params->tMax_short_exp_reg[0].data = (params->expo.max_short_exp >> 8) & 0x00ff;
    params->tMax_short_exp_reg[1].data = (params->expo.max_short_exp >> 0) & 0x00ff;
    params->tVts_reg_HDR[0].data = (params->expo.vts >> 8) & 0x00ff;
    params->tVts_reg_HDR[1].data = (params->expo.vts >> 0) & 0x00ff;
    params->tVts_reg_HDR[3].data = ((params->expo.vts - params->expo.max_short_exp) >> 8) & 0x00ff;
    params->tVts_reg_HDR[4].data = ((params->expo.vts - params->expo.max_short_exp) >> 0) & 0x00ff;
    params->tVts_reg_HDR[5].data = (2 * (params->expo.vts - params->expo.max_short_exp) >> 8) & 0x00ff;
    params->tVts_reg_HDR[6].data = (2 * (params->expo.vts - params->expo.max_short_exp) >> 0) & 0x00ff;
    params->vts_reg_dirty = true;
    return SUCCESS;
}

static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
        break;
    default:
        break;
    }
    return SUCCESS;
}

static int pCus_AEStatusNotify_HDR_LEF(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    // sc450ai_params *params = (sc450ai_params *)handle->private_data;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
            // SensorRegArrayW((I2C_ARRAY*)params->tVts_reg_HDR, ARRAY_SIZE(vts_reg_HDR));
            // SensorRegArrayW((I2C_ARRAY*)params->tMax_short_exp_reg, ARRAY_SIZE(max_short_exp_reg));
            params->reg_dirty = false;
        }
        if (params->vts_reg_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg_HDR, ARRAY_SIZE(vts_reg_HDR));
            SensorRegArrayW((I2C_ARRAY*)params->tMax_short_exp_reg, ARRAY_SIZE(max_short_exp_reg));
            params->vts_reg_dirty = false;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x00 }, // expo[16:8]
        { 0x3e02, 0x10 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, params->tExpo_reg, sizeof(expo_reg));
    dou_lines = (1000 * us) / (Preview_line_period_HDR * 2); // Preview_line_period in ns
    half_lines = 4 * dou_lines;
    if (half_lines < 4)
        half_lines = 4;
    if (half_lines > 2 * (params->expo.vts - params->expo.max_short_exp) - 6) {
        half_lines = 2 * (params->expo.vts - params->expo.max_short_exp) - 6;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        { 0x3e04, 0x02 }, // expo[7:0]
        { 0x3e05, 0x00 }, // expo[7:4]
    };
    memcpy(expo_reg_temp, params->tExpo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));

    dou_lines = (1000 * us) / (Preview_line_period_HDR * 2); // Preview_line_period in ns
    half_lines = 4 * dou_lines;
    if (half_lines < 4)
        half_lines = 4;
    if (half_lines > 2 * (params->expo.max_short_exp) - 2) {
        half_lines = 2 * (params->expo.max_short_exp) - 2;
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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
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
    u32 half_lines = 0, lines = 0, vts = 0;
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x00 }, // expo[16:8]
        { 0x3e02, 0x10 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, params->tExpo_reg, sizeof(expo_reg));

    lines = (1000 * us) / Preview_line_period; // Preview_line_period in ns
    half_lines = 2 * lines;
    if (half_lines <= 2)
        half_lines = 2;
    if (half_lines > 2 * (params->expo.vts) - 8) {
        vts = (half_lines + 8) / 2;
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
    params->tVts_reg[3].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[4].data = (vts >> 0) & 0x00ff;
    params->tVts_reg[5].data = ((2 * vts) >> 8) & 0x00ff;
    params->tVts_reg[6].data = ((2 * vts) >> 0) & 0x00ff;

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
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    u8 i = 0, Coarse_gain = 1, DIG_gain = 1;
    u32 Dcg_gainx1024 = 1, ANA_Fine_gainx64 = 1, DIG_Fine_gainx128 = 1;
    u8 Dcg_gain_reg = 0, Coarse_gain_reg = 0, DIG_gain_reg = 0, ANA_Fine_gain_reg = 0x20, DIG_Fine_gain_reg = 0x80;

    I2C_ARRAY gain_reg_temp[] = {
        { 0x3e06, 0x00 },
        { 0x3e07, 0x80 },
        { 0x3e08, 0x00 | 0x03 },
        { 0x3e09, 0x40 }, // low bit, 0x40 - 0x7f, step 1/64
    };
    memcpy(gain_reg_temp, params->tGain_reg, sizeof(gain_reg_temp));

    if (gain <= 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAXGAIN * 1024) {
        gain = SENSOR_MAXGAIN * 1024;
    }

    if (gain < 2 * 1024) {
        Dcg_gainx1024 = 1024;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 0;
        Coarse_gain_reg = 0x03;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 3904) // 3.813 * 1024
    {
        Dcg_gainx1024 = 1024;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x07;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 7808) // 7.625 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x23;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 15616) // 15.25 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x27;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 31232) // 30.5 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 4;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x2f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 61976) // 60.523 * 1024 // end again
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 61976 * 2) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 1;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 4) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 2;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x1;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 8) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 4;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x3;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 16) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 8;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x7;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= SENSOR_MAXGAIN * 1024) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 16;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0xf;
        ANA_Fine_gain_reg = 0x7f;
    }

    if (gain < 61976) {
        ANA_Fine_gain_reg = abs(64 * gain / (Dcg_gainx1024 * Coarse_gain));
        DIG_Fine_gain_reg = abs(8192 * gain / (Dcg_gainx1024 * Coarse_gain * ANA_Fine_gain_reg));
    } else {
        DIG_Fine_gain_reg = abs(8192 * gain / (Dcg_gainx1024 * Coarse_gain * DIG_gain) / ANA_Fine_gainx64);
    }

    params->tGain_reg[3].data = ANA_Fine_gain_reg;
    params->tGain_reg[2].data = Coarse_gain_reg;
    params->tGain_reg[1].data = DIG_Fine_gain_reg;
    params->tGain_reg[0].data = DIG_gain_reg & 0xF;

    for (i = 0; i < ARRAY_SIZE(gain_reg); i++) {
        if (params->tGain_reg[i].data != gain_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }

    return SUCCESS;
}

static int pCus_SetAEGain_HDR_SEF(ms_cus_sensor* handle, u32 gain)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    u8 i = 0, Coarse_gain = 1, DIG_gain = 1;
    u32 Dcg_gainx1024 = 1, ANA_Fine_gainx64 = 1, DIG_Fine_gainx128 = 1;
    u8 Dcg_gain_reg = 0, Coarse_gain_reg = 0, DIG_gain_reg = 0, ANA_Fine_gain_reg = 0x20, DIG_Fine_gain_reg = 0x80;

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

    if (gain < 2 * 1024) {
        Dcg_gainx1024 = 1024;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 0;
        Coarse_gain_reg = 0x03;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 3904) // 3.813 * 1024
    {
        Dcg_gainx1024 = 1024;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x07;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 7808) // 7.625 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 1;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x23;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 15616) // 15.25 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 2;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x27;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 31232) // 30.5 * 1024
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 4;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x2f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 61976) // 60.523 * 1024 // end again
    {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 1;
        DIG_Fine_gainx128 = 128;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        DIG_Fine_gain_reg = 0x80;
    } else if (gain < 61976 * 2) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 1;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x0;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 4) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 2;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x1;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 8) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 4;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x3;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain < 61976 * 16) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 8;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0x7;
        ANA_Fine_gain_reg = 0x7f;
    } else if (gain <= SENSOR_MAXGAIN * 1024) {
        Dcg_gainx1024 = 3904;
        Coarse_gain = 8;
        DIG_gain = 16;
        ANA_Fine_gainx64 = 127;
        Dcg_gain_reg = 1;
        Coarse_gain_reg = 0x3f;
        DIG_gain_reg = 0xf;
        ANA_Fine_gain_reg = 0x7f;
    }

    if (gain < 61976) {
        ANA_Fine_gain_reg = abs(64 * gain / (Dcg_gainx1024 * Coarse_gain));
        DIG_Fine_gain_reg = abs(8192 * gain / (Dcg_gainx1024 * Coarse_gain * ANA_Fine_gain_reg));
    } else {
        DIG_Fine_gain_reg = abs(8192 * gain / (Dcg_gainx1024 * Coarse_gain * DIG_gain) / ANA_Fine_gainx64);
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

static int sc450ai_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period;
    info->step = Preview_line_period;
    return SUCCESS;
}

static int sc450ai_GetShutterInfo_HDR_LEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS; /// 12;
    info->min = Preview_line_period_HDR * 2; // 2
    info->step = Preview_line_period_HDR * 2; // 2
    return SUCCESS;
}

static int sc450ai_GetShutterInfo_HDR_SEF(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    sc450ai_params* params = (sc450ai_params*)handle->private_data;
    info->max = (Preview_line_period_HDR * params->expo.max_short_exp);
    info->min = Preview_line_period_HDR * 2; // 2
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
#if 0
static int pCus_GetVideoRes_HDR_LEF( ms_cus_sensor *handle, u32 res_idx, cus_camsensor_res **res )
{
    *res = &handle->video_res_supported.res[res_idx];
    return SUCCESS;
}

static int pCus_SetVideoRes_HDR_LEF( ms_cus_sensor *handle, u32 res )
{
    handle->video_res_supported.ulcur_res = 0; //TBD
    return SUCCESS;
}
#endif

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
    sc450ai_params* params;
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
    params = (sc450ai_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tVts_reg_HDR, vts_reg_HDR, sizeof(vts_reg_HDR));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tGain_reg_HDR_SEF, gain_reg_HDR_SEF, sizeof(gain_reg_HDR_SEF));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tExpo_reg_HDR_SEF, expo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));
    memcpy(params->tMax_short_exp_reg, max_short_exp_reg, sizeof(max_short_exp_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    SENSOR_DMSG(handle->model_id, "sc450ai_MIPI");

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
        handle->video_res_supported.res[res].width = sc450ai_mipi_linear[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc450ai_mipi_linear[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc450ai_mipi_linear[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc450ai_mipi_linear[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc450ai_mipi_linear[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc450ai_mipi_linear[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc450ai_mipi_linear[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc450ai_mipi_linear[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc450ai_mipi_linear[res].senstr.strResDesc);
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
    // handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
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
    handle->pCus_sensor_init = pCus_init_linear_4M30fps;

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
    handle->pCus_sensor_SetPatternMode = sc450ai_SetPatternMode;
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
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    // sensor calibration
    handle->pCus_sensor_SetAEGain_cal = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = sc450ai_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.line = 1000;
    params->reg_dirty = false;
    params->orient_dirty = false;

    return SUCCESS;
}

int cus_camsensor_init_handle_HDR_SEF(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc450ai_params* params = NULL;
    int res;

    cus_camsensor_init_handle(drv_handle);
    params = (sc450ai_params*)handle->private_data;

    sprintf(handle->model_id, "sc450ai_MIPI_HDR_SEF");

    handle->bayer_id = SENSOR_BAYERID_HDR;
    handle->RGBIR_id = SENSOR_RGBIRID;
    handle->data_prec = SENSOR_DATAPREC_HDR;
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
        handle->video_res_supported.res[res].width = sc450ai_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc450ai_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc450ai_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc450ai_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc450ai_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc450ai_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc450ai_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc450ai_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc450ai_mipi_hdr[res].senstr.strResDesc);
    }

    handle->pCus_sensor_SetVideoRes = pCus_SetVideoRes_HDR;
    handle->mclk = Preview_MCLK_SPEED_HDR;

    handle->pCus_sensor_init = pCus_init_mipi4lane_HDR_DOL;

    handle->pCus_sensor_GetFPS = pCus_GetFPS_HDR;
    handle->pCus_sensor_SetFPS = pCus_SetFPS_HDR;
    handle->pCus_sensor_GetOrien = pCus_GetOrien;
    handle->pCus_sensor_SetOrien = pCus_SetOrien;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify_HDR_SEF;
    handle->pCus_sensor_GetAEUSecs = pCus_GetAEUSecs_HDR_SEF;
    handle->pCus_sensor_SetAEUSecs = pCus_SetAEUSecs_HDR_SEF;
    handle->pCus_sensor_GetAEGain = pCus_GetAEGain;
    handle->pCus_sensor_SetAEGain = pCus_SetAEGain_HDR_SEF;
    handle->pCus_sensor_GetShutterInfo = sc450ai_GetShutterInfo_HDR_SEF;

    params->expo.vts = vts_30fps_HDR;
    params->expo.line = 1000;
    params->expo.fps = 30;
    params->expo.max_short_exp = 188;

    handle->ae_gain_delay = 2;
    handle->ae_shutter_delay = 2;

    handle->ae_gain_ctrl_num = 2;
    handle->ae_shutter_ctrl_num = 2;

    return SUCCESS;
}

int cus_camsensor_init_handle_HDR_LEF(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc450ai_params* params;
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
    params = (sc450ai_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tGain_reg_HDR_SEF, gain_reg_HDR_SEF, sizeof(gain_reg_HDR_SEF));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tExpo_reg_HDR_SEF, expo_reg_HDR_SEF, sizeof(expo_reg_HDR_SEF));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "sc450ai_MIPI_HDR_LEF");

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
        handle->video_res_supported.res[res].width = sc450ai_mipi_hdr[res].senif.preview_w;
        handle->video_res_supported.res[res].height = sc450ai_mipi_hdr[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = sc450ai_mipi_hdr[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = sc450ai_mipi_hdr[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = sc450ai_mipi_hdr[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = sc450ai_mipi_hdr[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = sc450ai_mipi_hdr[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = sc450ai_mipi_hdr[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, sc450ai_mipi_hdr[res].senstr.strResDesc);
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

    handle->ae_gain_ctrl_num = 2;
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
    handle->pCus_sensor_GetFPS = pCus_GetFPS_HDR;
    handle->pCus_sensor_SetFPS = pCus_SetFPS_HDR;
    // handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = sc450ai_SetPatternMode;
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
    handle->pCus_sensor_GetShutterInfo = sc450ai_GetShutterInfo_HDR_LEF;
    handle->pCus_sensor_CustDefineFunction = pCus_sensor_CustDefineFunction;

    params->expo.vts = vts_30fps_HDR;
    params->expo.fps = 30;
    params->expo.max_short_exp = 188;
    params->reg_dirty = false;
    params->vts_reg_dirty = false;
    params->orient_dirty = false;

    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    return SUCCESS;
}
SENSOR_DRV_ENTRY_IMPL_END_EX(SC450AI,
    cus_camsensor_init_handle,
    cus_camsensor_init_handle_HDR_SEF,
    cus_camsensor_init_handle_HDR_LEF,
    sc450ai_params);
