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

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(OS03B10);

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

#define SENSOR_MAX_GAIN 128 //(15.5*15.9)                  // max sensor again, a-gain * conversion-gain*d-gain

#define Preview_MCLK_SPEED CUS_CMU_CLK_24MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_line_period 24691 // 16801//17814                           // MCLK=21.6 HTS/PCLK=3080 pixels/97.2MHZ=31.687us                              // 3126 for 25fps
#define vts_30fps 1350 // VTS for 20fps

#define Preview_WIDTH 2304 // 2688                    //resolution Width when preview
#define Preview_HEIGHT 1296 // 1520                    //resolution Height when preview
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
static int cus_camsensor_release_handle(ms_cus_sensor* handle);
static int OS03B10_SetAEGain(ms_cus_sensor* handle, u32 gain);
static int OS03B10_SetAEUSecs(ms_cus_sensor* handle, u32 us);

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
        u32 fps;
        u32 preview_fps;
        u32 lines;
        u32 final_gain;
    } expo;
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    u32 gain;
    bool reg_dirty;
    bool ori_dirty;
    I2C_ARRAY tVts_reg[3];
    I2C_ARRAY tGain_reg[4];
    I2C_ARRAY tExpo_reg[3];
    I2C_ARRAY tMirror_reg[5];
} OS03B10_params;

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
} os03b10_mipi_linear[] = {
    { LINEAR_RES_1, { 2304, 1296, 3, 30 }, { 0, 0, 2304, 1296 }, { "2304x1296@30fps" } },
};

// set sensor ID address and data,
const static I2C_ARRAY Sensor_id_table[] = {
    // P0
    { 0xfd, 0x00 },
    { 0x02, 0x53 }, // {address of ID, ID },
    { 0x03, 0x03 }, // {address of ID, ID },
    { 0x04, 0x42 }, // {address of ID, ID },
    { 0x05, 0x10 }, // {address of ID, ID },
};

const static I2C_ARRAY Sensor_init_table[] = {
    { 0xfd, 0x00 }, //
    { 0x20, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0x36, 0x01 }, //;pwd pll
    { 0xfd, 0x00 }, //
    { 0x2e, 0x2d }, //
    { 0x2f, 0x01 }, //
    { 0x41, 0x12 }, //
    { 0x36, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0xff, 0x05 }, //
    { 0xfd, 0x00 }, //
    { 0xfd, 0x00 }, //
    { 0x44, 0x40 }, //
    { 0x38, 0x21 }, //;clk gating
    { 0x45, 0x04 }, //;[2] mipi vb psv en
    { 0xfd, 0x01 }, //
    { 0x03, 0x00 }, //
    { 0x04, 0x04 }, //
    { 0x06, 0x19 }, //
    { 0x24, 0xff }, //; rpc
    { 0x01, 0x01 }, //
    { 0x18, 0x2f }, //; sampling enable
    { 0x1a, 0x06 }, //; sampling enable
    { 0x19, 0x58 }, //
    { 0x1b, 0x3c }, //; pwd_col enable
    { 0x2e, 0x03 }, //;sa_clk delay
    { 0x2f, 0x02 }, //; [1]dac code vb psv
    { 0x30, 0x52 }, //
    { 0x3c, 0xca }, //; vb psv enable
    { 0xfd, 0x03 }, //
    { 0x01, 0x0e }, //; pd2 enable
    { 0xfd, 0x01 }, // ;timing
    { 0x51, 0x0e }, //
    { 0x52, 0x0b }, //
    { 0x57, 0x0b }, //
    { 0x5a, 0xd3 }, //
    { 0x66, 0xd0 }, //; pwd col time
    { 0x6e, 0x26 }, //
    { 0x71, 0x80 }, //
    { 0x73, 0x2b }, //
    { 0xb8, 0x1c }, //
    { 0xd0, 0x20 }, //
    { 0xd2, 0x8e }, //
    { 0xd3, 0x1a }, //
    { 0xfd, 0x01 }, // ;vref related
    { 0xbd, 0x00 }, //
    { 0xd7, 0xbe }, // ;vlow 1x 0.71 ,vlow 2x 1.18
    { 0xd8, 0xef }, // ;vlow 3x 1.18 ,vlow 4x 1.3
    { 0xe8, 0x09 }, //;;;vbl
    { 0xe9, 0x05 }, //
    { 0xea, 0x08 }, //
    { 0xeb, 0x06 }, //
    { 0xfd, 0x03 }, //
    { 0x00, 0x5c }, //; adc range 0.801V
    { 0x03, 0xcd }, //
    { 0x06, 0x07 }, //; ncp -1.3
    { 0x07, 0x78 }, //
    { 0x08, 0x36 }, //; pd for dac_clk enbale
    { 0x09, 0x28 }, //; pldo=avdd;vbl row sel up
    { 0x0a, 0x0c }, //
    { 0x0b, 0x06 }, //; rst_num2
    { 0x0f, 0x13 }, //; pcp off/pd3 enable
    { 0xfd, 0x01 }, //
    { 0x1d, 0x08 }, //
    { 0x1e, 0x18 }, //
    { 0x1f, 0x30 }, //
    { 0x20, 0x5c }, //
    { 0xbc, 0x00 }, //; fixed disable
    { 0xfd, 0x03 }, //
    { 0x02, 0x00 }, //; bit7=0 psnc on,=1 psnc off
    { 0x05, 0x18 }, //
    { 0xfd, 0x02 }, //
    { 0x5e, 0x22 }, //; digital window enable
    { 0x34, 0x80 }, //; dpc on
    { 0xfd, 0x01 }, //;
    { 0xf0, 0x40 }, //;offset
    { 0xf1, 0x40 }, //;
    { 0xf2, 0x40 }, //;
    { 0xf3, 0x40 }, //;
    { 0xfa, 0x5c }, //
    { 0xfb, 0x6b }, //;blc
    { 0xf6, 0x00 }, //
    { 0xf7, 0xc0 }, //
    { 0xfc, 0x00 }, //
    { 0xfe, 0xc0 }, //
    { 0xff, 0x88 }, //
    { 0xc4, 0x70 }, // ;blc_exp_*=1
    { 0xc5, 0x70 }, //
    { 0xc6, 0x70 }, //
    { 0xc7, 0x70 }, //
    { 0xfd, 0x01 }, //
    { 0xce, 0x7c }, //
    { 0x8f, 0x00 }, //;mipi size
    { 0x91, 0x10 }, //
    { 0x92, 0x19 }, //
    { 0x94, 0x44 }, //
    { 0x95, 0x44 }, //
    { 0x98, 0x55 }, //
    { 0x9d, 0x03 }, //
    { 0x9e, 0x5f }, //
    { 0xa4, 0x13 }, //
    { 0xa5, 0xff }, //
    { 0xa6, 0xff }, //
    { 0xb1, 0x03 }, //;mipi en
    { 0x01, 0x02 }, //;update
    { 0x14, 0x03 }, //;row count start
};

const static I2C_ARRAY mirror_reg[] = {
    { 0xfd, 0x01 }, // 0
    { 0x3f, 0x00 }, // Mirror/Flip
    { 0x01, 0x01 }, // 2
    { 0xfd, 0x02 }, // 3
    { 0x5e, 0x22 }, // mem down en + enable auto BR first
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

const static I2C_ARRAY gain_reg[] = {
    { 0xfd, 0x01 },
    { 0x24, 0x20 }, // long a-gain[7:0] 1x~15.5x:0x10~0xF8
    { 0x39, 0x40 }, // d-gain[7:0] step 1/64, 1x:0x40
    { 0x37, 0x00 }, // d-gain[2:0]
};
static int g_sensor_ae_min_gain = 1024;

const static I2C_ARRAY expo_reg[] = {
    { 0xfd, 0x01 }, //
    { 0x03, 0x00 }, // long exp[15,8]
    { 0x04, 0x9a }, // long exp[7,0]
};

const static I2C_ARRAY vts_reg[] = {
    { 0x0d, 0x10 }, // 0x10 enable
    { 0x0E, 0x07 }, // MSB
    { 0x0F, 0xc1 }, // LSB
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
#define SENSOR_NAME OS03B10

#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg), _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg), (_reg), (_len)))

/////////////////// sensor hardware dependent //////////////
static int OS03B10_poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    // Sensor power on sequence
    sensor_if->MCLK(idx, 1, handle->mclk);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x3C00, 0);

    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(5000);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    SENSOR_USLEEP(5000);

    return SUCCESS;
}

static int OS03B10_poweroff(ms_cus_sensor* handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI* sensor_if = handle->sensor_if_api;

    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(500);
    return SUCCESS;
}

/////////////////// image function /////////////////////////
// Get and check sensor ID
// if i2c error or sensor id does not match then return FAIL
static int OS03B10_GetSensorID(ms_cus_sensor* handle, u32* id)
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
        SENSOR_DMSG("[%s]OS03B10 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
        // printf("OS03B10 Read sensor id, get 0x%x Success\n", (int)*id);
    }
    return SUCCESS;
}

static int OS03B10_SetPatternMode(ms_cus_sensor* handle, u32 mode)
{
    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);
    return SUCCESS;
}

static int OS03B10_init(ms_cus_sensor* handle)
{
    // SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i, cnt;

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table); i++) {

        if (Sensor_init_table[i].reg == 0xff) {
            SENSOR_MSLEEP(Sensor_init_table[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table[i].reg, Sensor_init_table[i].data) != SUCCESS) {
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
    return SUCCESS;
}

static int OS03B10_GetVideoResNum(ms_cus_sensor* handle, u32* ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}

static int OS03B10_GetVideoRes(ms_cus_sensor* handle, u32 res_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }
    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int OS03B10_GetCurVideoRes(ms_cus_sensor* handle, u32* cur_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int OS03B10_SetVideoRes(ms_cus_sensor* handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0:
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = OS03B10_init;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int OS03B10_GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;
    OS03B10_params* params = (OS03B10_params*)handle->private_data;

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

static int OS03B10_SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
    switch (orit) {
    case CUS_ORIT_M0F0:
        params->tMirror_reg[1].data = 0x00;
        params->tMirror_reg[4].data = 0x22;
        params->ori_dirty = true;
        break;
    case CUS_ORIT_M1F0:
        params->tMirror_reg[1].data = 0x01;
        params->tMirror_reg[4].data = 0x32;
        params->ori_dirty = true;
        break;
    case CUS_ORIT_M0F1:
        params->tMirror_reg[1].data = 0x02;
        params->tMirror_reg[4].data = 0x32;
        params->ori_dirty = true;
        break;
    case CUS_ORIT_M1F1:
        params->tMirror_reg[1].data = 0x03;
        params->tMirror_reg[4].data = 0x32;
        params->ori_dirty = true;
        break;
    }
    return SUCCESS;
}

static int OS03B10_GetFPS(ms_cus_sensor* handle)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 tVts = (params->tVts_reg[1].data << 8) | (params->tVts_reg[2].data << 0);

    if (params->expo.fps >= 1000)
        params->expo.preview_fps = (vts_30fps * max_fps * 1000) / tVts;
    else
        params->expo.preview_fps = (vts_30fps * max_fps) / tVts;

    return params->expo.preview_fps;
}

static int OS03B10_SetFPS(ms_cus_sensor* handle, u32 fps)
{
    u32 vts = 0;
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
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

    if ((params->expo.lines) > (params->expo.vts - 9))
        vts = params->expo.lines + 9;
    else
        vts = params->expo.vts;
    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;
}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int OS03B10_AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:

        break;
    case CUS_FRAME_ACTIVE:
        if (params->ori_dirty) {
            SensorRegArrayW((I2C_ARRAY*)params->tMirror_reg, ARRAY_SIZE(mirror_reg));
            params->ori_dirty = false;
        }
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

static int OS03B10_GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
    int rc = SUCCESS;
    u32 lines = 0;

    lines = (u32)(params->tExpo_reg[2].data & 0xff);
    lines |= (u32)(params->tExpo_reg[1].data & 0xff) << 8;
    *us = (lines * Preview_line_period) / 1000;
    return rc;
}

static int OS03B10_SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    u32 lines = 0, vts = 0;
    OS03B10_params* params = (OS03B10_params*)handle->private_data;

    lines = (u32)((1000 * us + (Preview_line_period >> 1)) / Preview_line_period);
    if (lines < 4)
        lines = 4;
    if (lines > params->expo.vts - 9)
        vts = lines + 9;
    else
        vts = params->expo.vts;

    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
        us,
        lines,
        params->expo.vts);
    // lines <<= 4;
    params->tExpo_reg[1].data = (lines >> 8) & 0x00ff;
    params->tExpo_reg[2].data = (lines >> 0) & 0x00ff;

    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;

    params->reg_dirty = true;
    return SUCCESS;
}

// Gain: 1x = 1024
static int OS03B10_GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;

    *gain = params->expo.final_gain;
    return SUCCESS;
}

#define MAX_A_GAIN 15872
static int OS03B10_SetAEGain_cal(ms_cus_sensor* handle, u32 gain)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
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

    SENSOR_DMSG("[%s] set input gain/gain/regH/regL=%d/%d/0x%x/0x%x\n", __FUNCTION__, input_gain, gain, gain_reg[0].data, gain_reg[1].data);
    return SUCCESS;
}

static int OS03B10_SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    OS03B10_params* params = (OS03B10_params*)handle->private_data;
    u32 dgain = 0, again;
    gain = (gain * handle->sat_mingain + 512) >> 10; // need to add min sat gain

    if (gain < 1024)
        gain = 1024;
    else if (gain >= SENSOR_MAX_GAIN * 1024)
        gain = SENSOR_MAX_GAIN * 1024;

    params->expo.final_gain = gain;
    /* A Gain */
    if (gain <= 1024) {
        again = 1024;
    } else if (gain < 2048) {
        again = (gain >> 6) << 6;
    } else if (gain < 4096) {
        again = (gain >> 7) << 7;
    } else if (gain < 8192) {
        again = (gain >> 8) << 8;
    } else if (gain < MAX_A_GAIN) {
        again = (gain >> 9) << 9;
    } else {
        again = MAX_A_GAIN;
    }
    dgain = gain * 64 / again;

    again = again >> 6;
    params->tGain_reg[1].data = again & 0xff; // again
    params->tGain_reg[2].data = dgain & 0xff; // low byte
    params->tGain_reg[3].data = (dgain >> 8) & 0x07; // high byte
    return SUCCESS;
}

static int OS03B10_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{

    return SUCCESS;
}

static int OS03B10_GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1;
    *max = 1000000 / Preview_MIN_FPS;
    return SUCCESS;
}

static int OS03B10_GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{

    *min = 1024; // 1024*1.52;
    *max = SENSOR_MAX_GAIN * 1024;
    return SUCCESS;
}

static int OS03B10_GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period * 4;
    info->step = Preview_line_period;
    return SUCCESS;
}

int cus_camsensor_init_handle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    OS03B10_params* params;
    int res;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    // private data allocation & init
    handle->private_data = CamOsMemCalloc(1, sizeof(OS03B10_params));
    params = (OS03B10_params*)handle->private_data;
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tMirror_reg, mirror_reg, sizeof(mirror_reg));

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "OS03B10_MIPI");

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
        handle->video_res_supported.res[res].width = os03b10_mipi_linear[res].senif.preview_w;
        handle->video_res_supported.res[res].height = os03b10_mipi_linear[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = os03b10_mipi_linear[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = os03b10_mipi_linear[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = os03b10_mipi_linear[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = os03b10_mipi_linear[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = os03b10_mipi_linear[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = os03b10_mipi_linear[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, os03b10_mipi_linear[res].senstr.strResDesc);
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
    handle->ae_gain_delay = 2; // 0;//1;
    handle->ae_shutter_delay = 2; // 1;//2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 1;

    /// calibration
    handle->sat_mingain = g_sensor_ae_min_gain;

    // LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release = cus_camsensor_release_handle;
    handle->pCus_sensor_init = OS03B10_init;
    handle->pCus_sensor_poweron = OS03B10_poweron;
    handle->pCus_sensor_poweroff = OS03B10_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID = OS03B10_GetSensorID;
    handle->pCus_sensor_GetVideoResNum = OS03B10_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes = OS03B10_GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes = OS03B10_GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes = OS03B10_SetVideoRes;
    handle->pCus_sensor_GetOrien = OS03B10_GetOrien;
    handle->pCus_sensor_SetOrien = OS03B10_SetOrien;
    handle->pCus_sensor_GetFPS = OS03B10_GetFPS;
    handle->pCus_sensor_SetFPS = OS03B10_SetFPS;
    // handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = OS03B10_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = OS03B10_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs = OS03B10_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = OS03B10_SetAEUSecs;
    handle->pCus_sensor_GetAEGain = OS03B10_GetAEGain;
    handle->pCus_sensor_SetAEGain = OS03B10_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = OS03B10_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = OS03B10_GetAEMinMaxUSecs;

    handle->pCus_sensor_GetShutterInfo = OS03B10_GetShutterInfo;

    // sensor calibration
    // handle->pCus_sensor_setCaliData_mingain=OS03B10_setCaliData_mingain;
    handle->pCus_sensor_SetAEGain_cal = OS03B10_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity = OS03B10_setCaliData_gain_linearity;

    params->expo.vts = vts_30fps;
    params->expo.fps = 25;
    params->expo.lines = 1000;
    params->gain = 1024;
    params->reg_dirty = false;
    params->ori_dirty = false;
    return SUCCESS;
}

static int cus_camsensor_release_handle(ms_cus_sensor* handle)
{

    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(OS03B10,
    cus_camsensor_init_handle,
    NULL,
    NULL,
    OS03B10_params);
