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

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(OS02G10);

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
#define SENSOR_DATAPREC CUS_DATAPRECISION_10 // CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000 // CFG
#define SENSOR_BAYERID CUS_BAYER_BG // CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAX_GAIN 15872 //(15.5)        // max sensor again, a-gain * conversion-gain*d-gain
#define MAX_A_GAIN 15872
#define Preview_MCLK_SPEED CUS_CMU_CLK_24MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_line_period 30057 // 1s/(vts*fps) ns
#define vts_30fps 1109 // VTS for 30fps

#define Preview_WIDTH 1920 // resolution Width when preview
#define Preview_HEIGHT 1080 // resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 3 // slowest preview FPS

#define SENSOR_I2C_ADDR 0x78 // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE // usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT I2C_FMT_A8D8 // CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

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
static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int g_sensor_ae_min_gain = 1024;

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
        u32 lines;
    } expo;
    I2C_ARRAY tVts_reg[3];
    I2C_ARRAY tGain_reg[5];
    I2C_ARRAY tExpo_reg[2];
    I2C_ARRAY tMirror_reg[6];
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool reg_dirty;
    bool orien_dirty;
} OS02G10_params;
// set sensor ID address and data

const I2C_ARRAY Sensor_id_table[] = {
    { 0xfd, 0x00 },
    { 0x02, 0x56 }, // {address of ID, ID },
    { 0x03, 0x02 }, // {address of ID, ID },
    { 0x04, 0x47 }, // {address of ID, ID },
    { 0x05, 0x00 }, // {address of ID, ID },
};

const I2C_ARRAY Sensor_init_table[] = // 1920*1080_30fps_27MCLK_756MMIPI_2840*1109*30
    {
        { 0xfd, 0x00 },
        { 0x36, 0x01 },
        { 0xfd, 0x00 },
        { 0x36, 0x00 },
        { 0xfd, 0x00 },
        { 0x20, 0x00 },
        { 0xff, 0x05 },
        { 0xfd, 0x00 },
        { 0xfd, 0x00 },
        { 0x30, 0x0a },
        { 0x35, 0x04 },
        { 0x38, 0x11 },
        { 0x41, 0x06 },
        { 0x44, 0x20 },
        { 0xfd, 0x01 },
        { 0x03, 0x04 },
        { 0x04, 0x4c },
        { 0x06, 0x00 },
        { 0x24, 0x30 },
        { 0x01, 0x01 },
        { 0x19, 0x50 },
        { 0x1a, 0x0c },
        { 0x1b, 0x0d },
        { 0x1c, 0x00 },
        { 0x1d, 0x75 },
        { 0x1e, 0x52 },
        { 0x22, 0x14 },
        { 0x25, 0x44 },
        { 0x26, 0x0f },
        { 0x3c, 0xca },
        { 0x3d, 0x4a },
        { 0x40, 0x0f },
        { 0x43, 0x38 },
        { 0x46, 0x01 },
        { 0x47, 0x00 },
        { 0x49, 0x32 },
        { 0x50, 0x01 },
        { 0x51, 0x28 },
        { 0x52, 0x20 },
        { 0x53, 0x03 },
        { 0x57, 0x16 },
        { 0x59, 0x01 },
        { 0x5a, 0x01 },
        { 0x5d, 0x04 },
        { 0x6a, 0x04 },
        { 0x6b, 0x03 },
        { 0x6e, 0x28 },
        { 0x71, 0xc2 },
        { 0x72, 0x04 },
        { 0x73, 0x38 },
        { 0x74, 0x04 },
        { 0x79, 0x00 },
        { 0x7a, 0xb2 },
        { 0x7b, 0x10 },
        { 0x8f, 0x80 },
        { 0x91, 0x38 },
        { 0x92, 0x02 },
        { 0x9d, 0x03 },
        { 0x9e, 0x55 },
        { 0xb8, 0x70 },
        { 0xb9, 0x70 },
        { 0xba, 0x70 },
        { 0xbb, 0x70 },
        { 0xbc, 0x00 },
        { 0xc4, 0x6d },
        { 0xc5, 0x6d },
        { 0xc6, 0x6d },
        { 0xc7, 0x6d },
        { 0xcc, 0x11 },
        { 0xcd, 0xe0 },
        { 0xd0, 0x1b },
        { 0xd2, 0x76 },
        { 0xd3, 0x68 },
        { 0xd4, 0x68 },
        { 0xd5, 0x73 },
        { 0xd6, 0x73 },
        { 0xe8, 0x55 },
        { 0xf0, 0x40 },
        { 0xf1, 0x40 },
        { 0xf2, 0x40 },
        { 0xf3, 0x40 },
        { 0xfa, 0x1c },
        { 0xfb, 0x33 },
        { 0xfc, 0x80 },
        { 0xfe, 0x80 },
        { 0xfd, 0x03 },
        { 0x03, 0x67 },
        { 0x00, 0x59 },
        { 0x04, 0x11 },
        { 0x05, 0x04 },
        { 0x06, 0x0c },
        { 0x07, 0x08 },
        { 0x08, 0x08 },
        { 0x09, 0x4f },
        { 0x0b, 0x08 },
        { 0x0d, 0x26 },
        { 0x0f, 0x00 },
        { 0xfd, 0x02 },
        { 0x34, 0xfe },
        { 0x5e, 0x22 },
        { 0xa1, 0x06 },
        { 0xa3, 0x38 },
        { 0xa5, 0x02 },
        { 0xa7, 0x80 },
        { 0xfd, 0x01 },
        { 0xa1, 0x05 },
        { 0xb1, 0x01 },
        { 0xfd, 0x01 },
        { 0xb1, 0x03 },
    };

const I2C_ARRAY TriggerStartTbl[] = {

};

const I2C_ARRAY PatternTbl[] = {
    //{0xb2,0x40}, //colorbar pattern , bit 0 to enable
};

const I2C_ARRAY mirror_reg[] = {
    { 0xfd, 0x01 },
    { 0x3f, 0x00 }, // P1 M0F0 [1]:F [0]:M
    { 0xfd, 0x02 }, //
    { 0x5e, 0x22 }, // mem down en + enable auto BR first
    { 0xfd, 0x01 },
    { 0x01, 0x01 },
};

/////////////////////////////////////////////////////////////////
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

const static I2C_ARRAY gain_reg[] = {
    { 0xfd, 0x01 },
    { 0x38, 0x10 }, // again msb
    { 0x24, 0x20 }, // again 1x:0x10, 15.5x:0xf8
    { 0x39, 0x40 }, // dgain[7:0] 1x:0x40 32x:0x7ff
    { 0x37, 0x00 }, // dgain[10:8]
};

const I2C_ARRAY expo_reg[] = {
    { 0x03, 0x01 },
    { 0x04, 0x00 },
};

const I2C_ARRAY vts_reg[] = {
    { 0x0d, 0x10 }, // 0x10 enable
    { 0x0E, 0x07 }, // MSB
    { 0x0F, 0xc1 }, // LSB
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

/////////// function definition ///////////////////
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME OS02G10

#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

int cus_camsensor_release_handle(ms_cus_sensor* handle);

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
static int pCus_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);

    // Sensor power on sequence
    sensor_if->MCLK(idx, 1, handle->mclk);
    SENSOR_USLEEP(1000);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x3C00, 0);
    SENSOR_USLEEP(5000);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_USLEEP(5000);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(5000);
    // handle->i2c_bus->i2c_open(handle->i2c_bus,&handle->i2c_cfg);

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;

    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(5000);
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

    for (n = 0; n < 5; ++n) { // retry , until I2C success
        if (n > 3)
            return FAIL;

        if (SensorRegArrayR((I2C_ARRAY*)id_from_sensor, table_length) == SUCCESS) // read sensor ID from I2C
            break;
        else
            SENSOR_MSLEEP_(1);
    }

    for (i = 0; i < table_length; ++i) {
        if (id_from_sensor[i].data != Sensor_id_table[i].data) {
            SENSOR_DMSG("[%s]Read OS02G10 id: 0x%x 0x%x\n", __FUNCTION__, id_from_sensor[0].data, id_from_sensor[1].data);
            return FAIL;
        }
        *id = id_from_sensor[i].data;
    }
    SENSOR_DMSG("[%s]Read OS02G10 id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    return SUCCESS;
}

static int OS02G10_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{

    return SUCCESS;
}
static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init(ms_cus_sensor* handle)
{
    int i, cnt = 0;
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    for (i = 0; i < ARRAY_SIZE(Sensor_init_table); i++) {
        if (Sensor_init_table[i].reg == 0xff) {
            SENSOR_MSLEEP_(Sensor_init_table[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table[i].reg, Sensor_init_table[i].data) != SUCCESS && Sensor_init_table[i].reg != 0x20) {
                cnt++;
                // SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if (cnt >= 10) {
                    // SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP_(10);
            }
        }
    }

    for (i = 0; i < ARRAY_SIZE(mirror_reg); i++) {
        if (SensorReg_Write(params->tMirror_reg[i].reg, params->tMirror_reg[i].data) != SUCCESS) {
            return FAIL;
        }
    }

    pCus_SetAEUSecs(handle, 25000);
    pCus_AEStatusNotify(handle, CUS_FRAME_ACTIVE);
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
    OS02G10_params* params = (OS02G10_params*)handle->private_data;

    sen_data = params->tMirror_reg[1].data & 0x03;
    SENSOR_DMSG("\n\n[%s]:mirror:%x\r\n\n\n\n", __FUNCTION__, sen_data);
    switch (sen_data) {
    case 0x00:
        *orit = CUS_ORIT_M0F0;
        break;
    case 0x01:
        *orit = CUS_ORIT_M1F0;
        break;
    case 0x02:
        *orit = CUS_ORIT_M0F1;
        break;
    case 0x03:
        *orit = CUS_ORIT_M1F1;
        break;
    }
    return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    OS02G10_params* params = (OS02G10_params*)handle->private_data;

    switch (orit) {
    case CUS_ORIT_M0F0:
        params->tMirror_reg[1].data = 0x00;
        params->tMirror_reg[3].data = 0x22;
        params->orien_dirty = true;
        break;
    case CUS_ORIT_M1F0:
        params->tMirror_reg[1].data = 0x01;
        params->tMirror_reg[3].data = 0x32;
        params->orien_dirty = true;
        break;
    case CUS_ORIT_M0F1:
        params->tMirror_reg[1].data = 0x02;
        params->tMirror_reg[3].data = 0x32;
        params->orien_dirty = true;
        break;
    case CUS_ORIT_M1F1:
        params->tMirror_reg[1].data = 0x03;
        params->tMirror_reg[3].data = 0x32;
        params->orien_dirty = true;
        break;
    }

    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor* handle)
{
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[1].data << 8) | (params->tVts_reg[2].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    u32 vts = 0;
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
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

    if ((params->expo.lines) > (params->expo.vts - 8))
        vts = params->expo.lines + 8;
    else
        vts = params->expo.vts;
    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
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
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    // ISensorIfAPI *sensor_if = handle->sensor_if_api;

    switch (status) {
    case CUS_FRAME_INACTIVE:
        if (params->orien_dirty) {
            handle->sensor_if_api->SetSkipFrame(handle->snr_pad_group, params->expo.fps, 3);
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            params->orien_dirty = false;
        }
        break;
    case CUS_FRAME_ACTIVE:
        if (params->reg_dirty) {
            SensorReg_Write(0xfd, 0x01); // page 1
            SensorReg_Write(0x01, 0x00); // frame sync disable
            SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
            SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
            SensorReg_Write(0x01, 0x01); // frame sync enable
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
    u32 lines = 0;
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    lines = (u32)(params->tExpo_reg[1].data & 0xff);
    lines |= (u32)(params->tExpo_reg[0].data & 0xff) << 8;

    *us = (lines * Preview_line_period) / 1000;
    return SUCCESS;
}

static int pCus_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    OS02G10_params* params = (OS02G10_params*)handle->private_data;

    lines = (u32)((1000 * us + (Preview_line_period >> 1)) / Preview_line_period);
    if (lines < 4)
        lines = 4;
    if (lines > params->expo.vts - 8)
        vts = lines + 8;
    else
        vts = params->expo.vts;

    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);

    // lines <<= 4;
    params->tExpo_reg[0].data = (u16)((lines >> 8) & 0x00ff);
    params->tExpo_reg[1].data = (u16)((lines >> 0) & 0x00ff);

    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;

    params->reg_dirty = true;
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    int again;
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    again = params->tGain_reg[2].data;

    *gain = again << 6;
    return SUCCESS;
}

static int pCus_SetAEGain_cal(ms_cus_sensor* handle, u32 gain)
{

    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    u32 input_gain = 0;
    u16 gain16;

    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    input_gain = gain;
    if (gain < 1024)
        gain = 1024;
    else if (gain >= MAX_A_GAIN)
        gain = MAX_A_GAIN;

    gain16 = (u16)(gain >> 6);
    params->tGain_reg[1].data = (gain16 >> 8) & 0x01; // high bit
    params->tGain_reg[2].data = gain16 & 0xff; // low byte

    SENSOR_DMSG("[%s] set gain/regH/regL=%d/0x%x/0x%x\n", __FUNCTION__, gain, params->tGain_reg[0].data, params->tGain_reg[1].data);
    // params->reg_dirty = true;
    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    OS02G10_params* params = (OS02G10_params*)handle->private_data;
    u32 input_gain = 0;
    u16 gain16;
    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    input_gain = gain;
    if (gain < 1024)
        gain = 1024;
    else if (gain >= MAX_A_GAIN)
        gain = MAX_A_GAIN;

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
    }

    gain16 = (u16)(gain >> 6);
    params->tGain_reg[1].data = (gain16 >> 8) & 0x01; // high bit
    params->tGain_reg[2].data = gain16 & 0xff; // low byte

    params->reg_dirty = true;
    // pr_info("[%s] set input gain/gain/AregH/AregL/DregH/DregL=%d/%d/0x%x/0x%x/0x%x/0x%x\n", __FUNCTION__, input_gain,gain,gain_reg[0].data,gain_reg[1].data,gain_reg[2].data,gain_reg[3].data);
    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1;
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1024;
    *max = SENSOR_MAX_GAIN;
    return SUCCESS;
}

static int OS02G10_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period * 4;
    info->step = Preview_line_period;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{

    return SUCCESS;
}

int cus_camsensor_init_handle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    OS02G10_params* params;
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
    params = (OS02G10_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));
    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "OS02G10_MIPI");

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
    handle->video_res_supported.res[0].crop_start_x = 0;
    handle->video_res_supported.res[0].crop_start_y = 0;
    handle->video_res_supported.res[0].nOutputWidth = 1920; // 2592;
    handle->video_res_supported.res[0].nOutputHeight = 1080; // 1944;
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
    handle->pCus_sensor_SetPatternMode = OS02G10_SetPatternMode;
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
    handle->pCus_sensor_GetShutterInfo = OS02G10_GetShutterInfo;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.lines = 1000;
    params->reg_dirty = false;
    params->orien_dirty = false;
    return SUCCESS;
}

int cus_camsensor_release_handle(ms_cus_sensor* handle)
{
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(OS02G10,
    cus_camsensor_init_handle,
    NULL,
    NULL,
    OS02G10_params);
