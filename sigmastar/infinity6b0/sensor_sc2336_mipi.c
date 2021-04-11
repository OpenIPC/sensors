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

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(SC2336);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE CAM_OS_ARRAY_SIZE
#endif

#define SENSOR_PAD_GROUP_SET CUS_SENSOR_PAD_GROUP_A
#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL

//============================================
// MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (2)
//#define SENSOR_MIPI_HDR_MODE (1) //0: Non-HDR mode. 1:Sony DOL mode
// MIPI config end.
//============================================

//#undef SENSOR_DBG
#define SENSOR_DBG 0

#define SENSOR_ISP_TYPE ISP_EXT // ISP_EXT, ISP_SOC
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_MIPI // CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000 // CFG
#define SENSOR_MAXGAIN (32 * 4)
#define SENSOR_BAYERID CUS_BAYER_BG // CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,

#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M

u32 Preview_line_period = 29630;
u32 vts_30fps = 1125;

#define Preview_WIDTH 1920 // resolution Width when preview
#define Preview_HEIGHT 1080 // resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 3 // slowest preview FPS
#define Preview_CROP_START_X 0 // CROP_START_X
#define Preview_CROP_START_Y 0 // CROP_START_Y

#define SENSOR_I2C_ADDR 0x60 // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A16D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL CUS_CLK_POL_NEG // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_NEG // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL CUS_CLK_POL_POS // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG

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
        u32 preview_fps;
        u32 fps;
        u32 line;
    } expo;
    struct {
        bool bVideoMode;
        u16 res_idx;
        CUS_CAMSENSOR_ORIT orit;
    } res;
    I2C_ARRAY tVts_reg[2];
    I2C_ARRAY tGain_reg[3];
    I2C_ARRAY tExpo_reg[3];
    I2C_ARRAY tMirror_reg[1];
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool orient_dirty;
    bool reg_dirty;
    CUS_CAMSENSOR_ORIT cur_orien;
} sc2336_params;
// set sensor ID address and data,
const I2C_ARRAY Sensor_id_table[] = {
    { 0x3107, 0xCB },
    { 0x3108, 0x3a },
};

const I2C_ARRAY Sensor_init_table_2M30fps[] = {
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x37f9, 0x80 },
    { 0x301f, 0x01 },
    { 0x3106, 0x05 },
    { 0x3248, 0x04 },
    { 0x3249, 0x0b },
    { 0x3253, 0x08 },
    { 0x3301, 0x09 },
    { 0x3302, 0xff },
    { 0x3303, 0x10 },
    { 0x3306, 0x60 },
    { 0x3307, 0x02 },
    { 0x330a, 0x01 },
    { 0x330b, 0x10 },
    { 0x330c, 0x16 },
    { 0x330d, 0x90 },
    { 0x3318, 0x02 },
    { 0x3321, 0x0a },
    { 0x3327, 0x0e },
    { 0x332b, 0x12 },
    { 0x3333, 0x10 },
    { 0x3334, 0x40 },
    { 0x335e, 0x06 },
    { 0x335f, 0x0a },
    { 0x3364, 0x1f },
    { 0x337c, 0x02 },
    { 0x337d, 0x0e },
    { 0x3390, 0x09 },
    { 0x3391, 0x0f },
    { 0x3392, 0x1f },
    { 0x3393, 0x20 },
    { 0x3394, 0x20 },
    { 0x3395, 0xff },
    { 0x33a2, 0x04 },
    { 0x33b1, 0x80 },
    { 0x33b2, 0x68 },
    { 0x33b3, 0x42 },
    { 0x33f9, 0x70 },
    { 0x33fb, 0xd0 },
    { 0x33fc, 0x0f },
    { 0x33fd, 0x1f },
    { 0x349f, 0x03 },
    { 0x34a6, 0x0f },
    { 0x34a7, 0x1f },
    { 0x34a8, 0x42 },
    { 0x34a9, 0x06 },
    { 0x34aa, 0x01 },
    { 0x34ab, 0x23 },
    { 0x34ac, 0x01 },
    { 0x34ad, 0x84 },
    { 0x3630, 0xf4 },
    { 0x3633, 0x22 },
    { 0x3639, 0xf4 },
    { 0x3670, 0x09 },
    { 0x3674, 0xf8 },
    { 0x3675, 0xfb },
    { 0x3676, 0xed },
    { 0x367c, 0x09 },
    { 0x367d, 0x0f },
    { 0x3690, 0x22 },
    { 0x3691, 0x32 },
    { 0x3692, 0x33 },
    { 0x3698, 0x89 },
    { 0x3699, 0x95 },
    { 0x369a, 0xbb },
    { 0x369b, 0xbb },
    { 0x369c, 0x09 },
    { 0x369d, 0x0f },
    { 0x36a2, 0x09 },
    { 0x36a3, 0x0f },
    { 0x36a4, 0x1f },
    { 0x36d0, 0x01 },
    { 0x3722, 0xe1 },
    { 0x3724, 0x41 },
    { 0x3725, 0xc1 },
    { 0x3728, 0x20 },
    { 0x3900, 0x0d },
    { 0x3905, 0x98 },
    { 0x391b, 0x81 },
    { 0x391c, 0x10 },
    { 0x3933, 0x81 },
    { 0x3934, 0xc9 },
    { 0x3940, 0x6d },
    { 0x3942, 0x01 },
    { 0x3943, 0xcb },
    { 0x3952, 0x02 },
    { 0x3953, 0x0f },
    { 0x3e01, 0x46 },
    { 0x3e08, 0x1f },
    { 0x3e1b, 0x14 },
    { 0x440e, 0x02 },
    { 0x4509, 0x38 },
    { 0x5799, 0x06 },
    { 0x5ae0, 0xfe },
    { 0x5ae1, 0x40 },
    { 0x5ae2, 0x30 },
    { 0x5ae3, 0x28 },
    { 0x5ae4, 0x20 },
    { 0x5ae5, 0x30 },
    { 0x5ae6, 0x28 },
    { 0x5ae7, 0x20 },
    { 0x5ae8, 0x3c },
    { 0x5ae9, 0x30 },
    { 0x5aea, 0x28 },
    { 0x5aeb, 0x3c },
    { 0x5aec, 0x30 },
    { 0x5aed, 0x28 },
    { 0x5aee, 0xfe },
    { 0x5aef, 0x40 },
    { 0x5af4, 0x30 },
    { 0x5af5, 0x28 },
    { 0x5af6, 0x20 },
    { 0x5af7, 0x30 },
    { 0x5af8, 0x28 },
    { 0x5af9, 0x20 },
    { 0x5afa, 0x3c },
    { 0x5afb, 0x30 },
    { 0x5afc, 0x28 },
    { 0x5afd, 0x3c },
    { 0x5afe, 0x30 },
    { 0x5aff, 0x28 },
    { 0x36e9, 0x20 },
    { 0x37f9, 0x27 },
    { 0x0100, 0x01 },

    { 0xffff, 0x0a }, /////delay 10ms
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
    { 0x3e09, 0x00 }, // low bit
};

const static I2C_ARRAY expo_reg[] = {
    { 0x3e00, 0x00 }, // expo [20:17]
    { 0x3e01, 0x46 }, // expo[16:8]
    { 0x3e02, 0x00 }, // expo[7:0], [3:0] fraction of line
};

const static I2C_ARRAY vts_reg[] = {
    { 0x320e, 0x04 },
    { 0x320f, 0x65 }
};

I2C_ARRAY temperature_reg_1[] = {
    { 0x3306, 0x68 },
    { 0x330b, 0x18 },
};

/////////// function definition ///////////////////
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) LOGD(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME sc2336

#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

/////////////////// sensor hardware dependent //////////////
static int pCus_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);
    // ISP_config_io(handle);
    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);

    // Sensor power on sequence
    sensor_if->MCLK(idx, 1, handle->mclk);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    CamOsMsSleep(1);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    CamOsMsSleep(1);

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    sc2336_params* params = (sc2336_params*)handle->private_data;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    // handle->i2c_bus->i2c_close(handle->i2c_bus);
    CamOsMsSleep(1);
    // Set_csi_if(0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);

    sensor_if->MCLK(idx, 0, handle->mclk);

    params->cur_orien = CUS_ORIT_M0F0;

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
        //*id = id_from_sensor[i].data;
        *id = ((*id) + id_from_sensor[i].data) << 8;
    }

    *id >>= 8;
    SENSOR_DMSG("[%s]sc2336 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    // SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);

    return SUCCESS;
}

static int sc2336_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{

    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);

    return SUCCESS;
}
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init_mipi2lane_linear_2M30fps(ms_cus_sensor* handle)
{
    sc2336_params* params = (sc2336_params*)handle->private_data;
    // SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i, cnt;
    // ISensorIfAPI *sensor_if = (handle->sensor_if_api);
    // sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table_2M30fps); i++) {
        if (Sensor_init_table_2M30fps[i].reg == 0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_2M30fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_2M30fps[i].reg, Sensor_init_table_2M30fps[i].data) != SUCCESS) {
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

    pCus_SetOrien(handle, params->cur_orien);
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
    sc2336_params* params = (sc2336_params*)handle->private_data;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0: //"1920x1080@30fps"
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = pCus_init_mipi2lane_linear_2M30fps;
        vts_30fps = 1125;
        params->expo.vts = vts_30fps;
        params->expo.fps = 30;
        Preview_line_period = 29629;
        break;

    default:
        break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;
    sc2336_params* params = (sc2336_params*)handle->private_data;
    sen_data = params->tMirror_reg[0].data;
    SENSOR_DMSG("mirror:%x\r\n", sen_data);
    switch (sen_data) {
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
    sc2336_params* params = (sc2336_params*)handle->private_data;

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
    sc2336_params* params = (sc2336_params*)handle->private_data;
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
    sc2336_params* params = (sc2336_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;
    //  pr_info("[%s]  max_min_fps : %d ,%d\n\n", __FUNCTION__,max_fps,min_fps);
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

    if (params->expo.line > (params->expo.vts) - 5) {
        vts = (params->expo.line + 5);
    } else {
        vts = params->expo.vts;
    }
    params->tVts_reg[0].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;
}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{

    sc2336_params* params = (sc2336_params*)handle->private_data;

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

static int pCus_GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    int rc = 0;
    u32 lines = 0;
    sc2336_params* params = (sc2336_params*)handle->private_data;
    lines |= (u32)(params->tExpo_reg[0].data & 0x0f) << 16;
    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 8;
    lines |= (u32)(params->tExpo_reg[2].data & 0xf0) << 0;
    lines >>= 4;
    *us = (lines * Preview_line_period) / 1000; // return us

    SENSOR_DMSG("[%s] sensor expo lines/us %d, %dus\n", __FUNCTION__, lines, *us);
    return rc;
}

static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    int i;
    u32 expo_lines = 0, vts = 0;
    sc2336_params* params = (sc2336_params*)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x46 }, // expo[16:8]
        { 0x3e02, 0x00 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, params->tExpo_reg, sizeof(expo_reg));

    expo_lines = (1000 * us) / Preview_line_period; // Preview_line_period in ns
    if (expo_lines <= 1)
        expo_lines = 1;
    if (expo_lines > params->expo.vts - 5) {
        vts = (expo_lines + 5);
    } else
        vts = params->expo.vts;

    params->expo.line = expo_lines;
    SENSOR_DMSG("[%s] us %ld, expo_lines %ld, vts %ld\n", __FUNCTION__, us, expo_lines, params->expo.vts);

    expo_lines = expo_lines << 4;
    params->tExpo_reg[0].data = (expo_lines >> 16) & 0x0f;
    params->tExpo_reg[1].data = (expo_lines >> 8) & 0xff;
    params->tExpo_reg[2].data = (expo_lines >> 0) & 0xf0;
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
    sc2336_params* params = (sc2336_params*)handle->private_data;
    u8 i = 0;
    u64 gain_reg_0x3e09, gain_reg_0x3e06, gain_reg_0x3e07;

    I2C_ARRAY gain_reg_temp[] = {
        { 0x3e06, 0x00 },
        { 0x3e07, 0x00 | 0x80 },
        { 0x3e09, 0x00 },
    };
    memcpy(gain_reg_temp, gain_reg, sizeof(gain_reg));

    if (gain <= 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAXGAIN * 1024) {
        gain = SENSOR_MAXGAIN * 1024;
    }

    if (gain < 2 * 1024) {
        gain_reg_0x3e09 = 0x00;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 1024;
    } else if (gain < 4 * 1024) {
        gain_reg_0x3e09 = 0x08;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 2048;
    } else if (gain < 8 * 1024) {
        gain_reg_0x3e09 = 0x09;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 4096;
    } else if (gain < 16 * 1024) {
        gain_reg_0x3e09 = 0x0b;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 8192;
    } else if (gain < 32 * 1024) {
        gain_reg_0x3e09 = 0x0f;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 16384;
    } else if (gain < 64 * 1024) // open dgain begin  max digital gain 4X
    {
        gain_reg_0x3e09 = 0x1f;
        gain_reg_0x3e06 = 0x00;
        gain_reg_0x3e07 = gain * 128 / 32768;
    } else if (gain < 128 * 1024) {
        gain_reg_0x3e09 = 0x1f;
        gain_reg_0x3e06 = 0x01;
        gain_reg_0x3e07 = gain * 128 / 65536;
    } else if (gain == SENSOR_MAXGAIN * 1024) {
        gain_reg_0x3e09 = 0x1f;
        gain_reg_0x3e06 = 0x03;
        gain_reg_0x3e07 = 0x80;
    }

    params->tGain_reg[2].data = gain_reg_0x3e09; // 0x3e09
    params->tGain_reg[1].data = gain_reg_0x3e07; // 0x3e07
    params->tGain_reg[0].data = gain_reg_0x3e06; // 0x3e06

    for (i = 0; i < ARRAY_SIZE(gain_reg); i++) {
        if (params->tGain_reg[i].data != gain_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }
    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1;
    *max = 1000000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1024; //
    *max = SENSOR_MAXGAIN * 1024;
    return SUCCESS;
}

static int sc2336_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period;
    info->step = Preview_line_period;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{

    return SUCCESS;
}

int cus_camsensor_init_handle_linear(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc2336_params* params;
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
    params = (sc2336_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "sc2336_MIPI");

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
    handle->video_res_supported.ulcur_res = 0; // default resolution index is 0.
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT;
    handle->video_res_supported.res[0].max_fps = Preview_MAX_FPS;
    handle->video_res_supported.res[0].min_fps = Preview_MIN_FPS;
    handle->video_res_supported.res[0].crop_start_x = 0;
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
    handle->pCus_sensor_init = pCus_init_mipi2lane_linear_2M30fps;

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
    handle->pCus_sensor_SetPatternMode = sc2336_SetPatternMode;
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
    handle->pCus_sensor_SetAEGain_cal = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = sc2336_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.line = 1000;
    params->reg_dirty = false;
    params->orient_dirty = false;
    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{

    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(SC2336,
    cus_camsensor_init_handle_linear,
    NULL,
    NULL,
    sc2336_params);
