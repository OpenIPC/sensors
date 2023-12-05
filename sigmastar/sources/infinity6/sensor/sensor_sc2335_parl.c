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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <cam_os_wrapper.h>
#include <drv_ms_cus_sensor.h>
#include <drv_sensor.h>
#include <linux/delay.h>
#ifdef __cplusplus
}
#endif

//#define _DEBUG_

#define SENSOR_ISP_TYPE ISP_EXT
#define SENSOR_IFBUS_TYPE CUS_SENIF_BUS_PARL // CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_DATAPREC CUS_DATAPRECISION_12 // CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE CUS_SEN_10TO12_9000

#define SENSOR_BAYERID CUS_BAYER_BG // CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID CUS_RGBIR_NONE
#define SENSOR_ORIT CUS_ORIT_M0F0 // CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAXGAIN (15875 * 31) / 1000 //  (15.75*31.5)   max sensor gain, a-gain*conversion-gain*d-gain

#define Preview_MCLK_SPEED CUS_CMU_CLK_27MHZ // CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_line_period 29630 // 2200*1125*30
#define Prv_Max_line_number 1080 // maximum exposure line munber of sensor when preview
#define vts_30fps 1125 // for 30 fps
#define Preview_WIDTH 1920 // resolution Width when preview
#define Preview_HEIGHT 1080 // resolution Height when preview
#define Preview_MAX_FPS 30 // fastest preview FPS
#define Preview_MIN_FPS 3 // slowest preview FPS

#define SENSOR_I2C_LEGACY I2C_NORMAL_MODE
#define SENSOR_I2C_FMT I2C_FMT_A16D8
#define SENSOR_I2C_ADDR 0x60 // I2C slave address
#define SENSOR_I2C_SPEED 200000 // 200KHz

#define SENSOR_PWDN_POL CUS_CLK_POL_NEG // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL CUS_CLK_POL_NEG // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL CUS_CLK_POL_NEG // CUS_CLK_POL_NEG //CUS_CLK_POL_POS        // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL CUS_CLK_POL_POS // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL CUS_CLK_POL_NEG // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG
#define ENABLE_NR 1

CUS_CAMSENSOR_CAP sensor_cap = {
    .length = sizeof(CUS_CAMSENSOR_CAP),
    .version = 0x0001,
};

typedef struct {
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
        u32 line;
    } expo;

    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool reg_mf;
    bool reg_dirty; // sensor setting need to update through I2C
    bool nr_dirty;
    bool temperature_reg_1_dirty;
    CUS_CAMSENSOR_ORIT cur_orien;
    // pthread_mutex_t rw_lock;
} sc2335_params;

I2C_ARRAY Sensor_id_table[] = {
    { 0x3107, 0xCB },
    { 0x3108, 0x14 },
};

I2C_ARRAY Sensor_init_table[] = {
    // printf("===SC2239_mipi_lane2_bit10_1080P_30fps_20200309\n");
    { 0x0103, 0x01 },
    { 0xffff, 0x50 },
    { 0x0100, 0x00 },
    { 0x36e9, 0x80 },
    { 0x36f9, 0x80 },
    { 0x301f, 0x02 },
    { 0x3207, 0x3f },
    { 0x3249, 0x0f },
    { 0x3253, 0x08 },
    { 0x3271, 0x00 },
    { 0x3273, 0x03 },
    { 0x3301, 0x06 },
    { 0x3302, 0x09 },
    { 0x3304, 0x28 },
    { 0x3306, 0x30 },
    { 0x330b, 0x94 },
    { 0x330c, 0x08 },
    { 0x330d, 0x18 },
    { 0x330e, 0x14 },
    { 0x330f, 0x05 },
    { 0x3310, 0x06 },
    { 0x3314, 0x96 },
    { 0x3316, 0x00 },
    { 0x331e, 0x21 },
    { 0x332b, 0x08 },
    { 0x3333, 0x10 },
    { 0x3338, 0x80 },
    { 0x333a, 0x04 },
    { 0x334c, 0x04 },
    { 0x335f, 0x04 },
    { 0x3364, 0x17 },
    { 0x3366, 0x62 },
    { 0x337c, 0x05 },
    { 0x337d, 0x09 },
    { 0x337e, 0x00 },
    { 0x3390, 0x08 },
    { 0x3391, 0x18 },
    { 0x3392, 0x38 },
    { 0x3393, 0x09 },
    { 0x3394, 0x20 },
    { 0x3395, 0x20 },
    { 0x33a2, 0x07 },
    { 0x33ac, 0x04 },
    { 0x33ae, 0x14 },
    { 0x3614, 0x00 },
    { 0x3622, 0x16 },
    { 0x3630, 0x68 },
    { 0x3631, 0x84 },
    { 0x3637, 0x20 },
    { 0x363a, 0x1f },
    { 0x363c, 0x0e },
    { 0x3670, 0x0e },
    { 0x3674, 0xa1 },
    { 0x3675, 0x9c },
    { 0x3676, 0x9e },
    { 0x3677, 0x84 },
    { 0x3678, 0x85 },
    { 0x3679, 0x87 },
    { 0x367c, 0x18 },
    { 0x367d, 0x38 },
    { 0x367e, 0x08 },
    { 0x367f, 0x18 },
    { 0x3690, 0x32 },
    { 0x3691, 0x32 },
    { 0x3692, 0x44 },
    { 0x369c, 0x08 },
    { 0x369d, 0x38 },
    { 0x3908, 0x82 },
    { 0x391f, 0x18 },
    { 0x3e01, 0x8c },
    { 0x3e02, 0x00 },
    { 0x3f00, 0x0d },
    { 0x3f04, 0x02 },
    { 0x3f05, 0x0e },
    { 0x3f09, 0x48 },
    { 0x4505, 0x0a },
    { 0x4509, 0x20 },
    { 0x481d, 0x0a },
    { 0x4827, 0x03 },
    { 0x5787, 0x10 },
    { 0x5788, 0x06 },
    { 0x578a, 0x10 },
    { 0x578b, 0x06 },
    { 0x5790, 0x10 },
    { 0x5791, 0x10 },
    { 0x5792, 0x00 },
    { 0x5793, 0x10 },
    { 0x5794, 0x10 },
    { 0x5795, 0x00 },
    { 0x5799, 0x00 },
    { 0x57c7, 0x10 },
    { 0x57c8, 0x06 },
    { 0x57ca, 0x10 },
    { 0x57cb, 0x06 },
    { 0x57d1, 0x10 },
    { 0x57d4, 0x10 },
    { 0x57d9, 0x00 },
    { 0x36e9, 0x20 }, // pll
    { 0x36f9, 0x27 },
    { 0x0100, 0x01 },
    { 0xffff, 0x50 },
};

I2C_ARRAY mirror_reg[] = {
    { 0x3221, 0x00 }, // mirror[2:1], flip[6:5]
};

I2C_ARRAY gain_reg[] = {
    { 0x3e06, 0x00 },
    { 0x3e07, 0x80 }, // 128
    { 0x3e08, 0x03 },
    { 0x3e09, 0x40 }, // 64
};

I2C_ARRAY expo_reg[] = {
    // max expo line vts-4!
    { 0x3e00, 0x00 }, // expo [20:17]
    { 0x3e01, 0x02 }, // expo[15:8]
    { 0x3e02, 0x00 }, // expo[7:0], [3:0] fraction of line
};

I2C_ARRAY vts_reg[] = {
    { 0x320e, 0x04 },
    { 0x320f, 0x65 },
};

#if ENABLE_NR
I2C_ARRAY nr_reg[] = {
    { 0x363c, 0x0e },
};
#endif

I2C_ARRAY temperature_reg_1[] = {
    { 0x5799, 0x07 },
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
#define SensorReg_Read(_reg, _data) (handle->i2c_bus->i2c_rx(handle->i2c_bus, handle->i2c_cfg, _reg, _data))
#define SensorReg_Write(_reg, _data) (handle->i2c_bus->i2c_tx(handle->i2c_bus, handle->i2c_cfg, _reg, _data))
#define SensorRegArrayW(_reg, _len) (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, handle->i2c_cfg, (_reg), (_len)))
#define SensorRegArrayR(_reg, _len) (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, handle->i2c_cfg, (_reg), (_len)))

/////////////////// sensor hardware dependent //////////////
#if 0
static int ISP_config_io(ms_cus_sensor *handle) {

  ISensorIfAPI *sensor_if = &handle->sensor_if_api;

  //SENSOR_DMSG("[%s]", __FUNCTION__);

  sensor_if->HsyncPol(handle, handle->HSYNC_POLARITY);
  sensor_if->VsyncPol(handle, handle->VSYNC_POLARITY);
  sensor_if->ClkPol(handle, handle->PCLK_POLARITY);
  sensor_if->BayerFmt(handle, handle->bayer_id);
  sensor_if->RGBIRFmt(handle, handle->RGBIR_id);
  sensor_if->DataBus(handle, handle->sif_bus);

  sensor_if->DataPrecision(handle, handle->data_prec);
  sensor_if->FmtConv(handle,  handle->data_mode);
  return SUCCESS;
}
#endif

static int sc2335poweron(ms_cus_sensor* handle, u32 idx)
{
    ISensorIfAPI* sensor_if = &handle->sensor_if_api;

    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    CamOsMsSleep(1);
    // Sensor power on sequence
    sensor_if->MCLK(idx, 1, handle->mclk);
    sensor_if->SetIOPad(idx, handle->sif_bus, 0);
    CamOsMsSleep(5);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    CamOsMsSleep(2);
    SENSOR_DMSG("[%s] pwd high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->reset_POLARITY);
    CamOsMsSleep(5);

    return SUCCESS;
}

static int sc2335poweroff(ms_cus_sensor* handle, u32 idx)
{

    ISensorIfAPI* sensor_if = &handle->sensor_if_api;
    sc2335_params* params = (sc2335_params*)handle->private_data;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    CamOsMsSleep(10);
    sensor_if->MCLK(idx, 0, handle->mclk);
    params->cur_orien = CUS_ORIT_M0F0;
    return SUCCESS;
}

/////////////////// image function /////////////////////////
// Get and check sensor ID
// if i2c error or sensor id does not match then return FAIL
static int sc2335GetSensorID(ms_cus_sensor* handle, u32* id)
{
    int i, n;
    int table_length = ARRAY_SIZE(Sensor_id_table);
    I2C_ARRAY id_from_sensor[ARRAY_SIZE(Sensor_id_table)];

    pr_info("\n[%s]\n", __FUNCTION__);
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
        {
            // convert sensor id to u32 format
            for (i = 0; i < table_length; ++i) {
                *id = id_from_sensor[i].data;
                SENSOR_DMSG("[%s] %s read id[%d], get 0x%x\n", __FUNCTION__, handle->model_id, i, (int)*id);

                if (id_from_sensor[i].data != Sensor_id_table[i].data) {
                    SENSOR_EMSG("[%s] %s,get 0x%x, error id!\n", __FUNCTION__, handle->model_id, Sensor_id_table[i].data);
                    return FAIL;
                }
            }
            break;
        } else
            CamOsMsSleep(1);
    }
    pr_info("[%s] %s get sensor id:0x%x, SUCCESS!\n", __FUNCTION__, handle->model_id, (int)*id);
    return SUCCESS;
}

static int sc2335SetPatternMode(ms_cus_sensor* handle, u32 mode)
{
    return SUCCESS;
}

static u16 reg3040;

static int sc2335Init(ms_cus_sensor* handle)
{
    int i, cnt = 0;
    // ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
    pr_info("[%s:%d]\n", __FUNCTION__, __LINE__);

    // logic
    SensorReg_Read(0x3040, &reg3040);

    for (i = 0; i < ARRAY_SIZE(Sensor_init_table); i++) {
        if ((Sensor_init_table[i].reg == 0x3640) || (Sensor_init_table[i].reg == 0x3641)) {
            SENSOR_EMSG("[%s] reg:0x%x, value:0x%x\n", __FUNCTION__, Sensor_init_table[i].reg, Sensor_init_table[i].data);
        }

        // pr_info("[%s] addr:0x%x, data:0x%x\n", __FUNCTION__, Sensor_init_table[i].reg, Sensor_init_table[i].data);
        if (Sensor_init_table[i].reg == 0xffff) {
            CamOsMsSleep(Sensor_init_table[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table[i].reg, Sensor_init_table[i].data) != SUCCESS) {
                cnt++;
                pr_info("[%s] set Sensor_init_table retry : %d...\n", __FUNCTION__, cnt);
                if (cnt >= 10) {
                    pr_info("[%s:%d] retry FAIL!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                CamOsMsSleep(10);
            }
        }
    }

    if (reg3040 == 0x40) {
        SensorReg_Write(0x363c, 0x0e);
    } else {
        SensorReg_Write(0x363c, 0x0f);
    }

    /*reset mirror/flip to 0/0*/
    mirror_reg[0].data = 0;
    CamOsMsSleep(50);

    return SUCCESS;
}

static int sc2335GetVideoResNum(ms_cus_sensor* handle, u32* ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}
static int sc2335GetVideoRes(ms_cus_sensor* handle, u32 res_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int sc2335GetCurVideoRes(ms_cus_sensor* handle, u32* cur_idx, cus_camsensor_res** res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int sc2335SetVideoRes(ms_cus_sensor* handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
    case 0:
        handle->video_res_supported.ulcur_res = 0;
        handle->pCus_sensor_init = sc2335Init;
        break;
    default:
        break;
    }

    return SUCCESS;
}

static int sc2335GetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* orit)
{
    char sen_data;

    sen_data = mirror_reg[0].data;
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

static int sc2335SetOrien(ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT orit)
{
    sc2335_params* params = (sc2335_params*)handle->private_data;

    switch (orit) {
    case CUS_ORIT_M0F0:
        mirror_reg[0].data = 0x00;
        params->cur_orien = CUS_ORIT_M0F0;
        params->reg_mf = true;
        break;
    case CUS_ORIT_M1F0:
        mirror_reg[0].data = 0x06;
        params->cur_orien = CUS_ORIT_M1F0;
        params->reg_mf = true;
        break;
    case CUS_ORIT_M0F1:
        mirror_reg[0].data = 0x60;
        params->cur_orien = CUS_ORIT_M0F1;
        params->reg_mf = true;
        break;
    case CUS_ORIT_M1F1:
        mirror_reg[0].data = 0x66;
        params->cur_orien = CUS_ORIT_M1F1;
        params->reg_mf = true;
        break;
    default:
        break;
    }
    return SUCCESS;
}

static int sc2335GetFPS(ms_cus_sensor* handle)
{
    sc2335_params* params = (sc2335_params*)handle->private_data;
    return params->expo.fps;
}

static int sc2335SetFPS(ms_cus_sensor* handle, u32 fps)
{
    u32 vts = 0;
    sc2335_params* params = (sc2335_params*)handle->private_data;
    SENSOR_DMSG("\n\n ****************  [%s], fps=%d  **************** \n", __FUNCTION__, fps);
    if (fps >= 3 && fps <= 30) {
        params->expo.fps = fps;
        params->expo.vts = ((vts_30fps * 30 + (fps >> 1)) / fps) - 1;
    } else if (fps >= 3000 && fps <= 30000) {
        params->expo.fps = fps;
        params->expo.vts = ((vts_30fps * 30000 + (fps >> 1)) / fps) - 1;
    } else {
        SENSOR_DMSG("[%s] FPS %d out of range.\n", __FUNCTION__, fps);
        return FAIL;
    }

    if (params->expo.line > 2 * (params->expo.vts) - 10) {
        vts = (params->expo.line + 11) / 2;
    } else {
        vts = params->expo.vts;
    }
    vts_reg[0].data = (vts >> 8) & 0x00ff;
    vts_reg[1].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;
}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
// AE status notification
static int sc2335AEStatusNotify(ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    sc2335_params* params = (sc2335_params*)handle->private_data;
    switch (status) {
    case CUS_FRAME_INACTIVE:

        break;
    case CUS_FRAME_ACTIVE:
        if (params->reg_mf) {
            SensorRegArrayW((I2C_ARRAY*)mirror_reg, sizeof(mirror_reg) / sizeof(I2C_ARRAY));
            params->reg_mf = false;
        }
        if (params->reg_dirty) {
            SensorRegArrayW((I2C_ARRAY*)expo_reg, sizeof(expo_reg) / sizeof(I2C_ARRAY));
            SensorRegArrayW((I2C_ARRAY*)gain_reg, sizeof(gain_reg) / sizeof(I2C_ARRAY));
            SensorRegArrayW((I2C_ARRAY*)vts_reg, sizeof(vts_reg) / sizeof(I2C_ARRAY));
            params->reg_dirty = false;
        }
#if ENABLE_NR
        if (params->nr_dirty || params->temperature_reg_1_dirty) {
            SensorReg_Write(0x3812, 0x00);
            if (params->nr_dirty) {
                SensorRegArrayW((I2C_ARRAY*)nr_reg, sizeof(nr_reg) / sizeof(I2C_ARRAY));
                params->nr_dirty = false;
            }
            if (params->temperature_reg_1_dirty) {
                SensorRegArrayW((I2C_ARRAY*)temperature_reg_1, sizeof(temperature_reg_1) / sizeof(I2C_ARRAY));
                params->temperature_reg_1_dirty = false;
            }
            SensorReg_Write(0x3812, 0x30);
        }
#endif

        break;
    default:
        break;
    }
    return SUCCESS;
}

static int sc2335GetAEUSecs(ms_cus_sensor* handle, u32* us)
{
    int rc = 0;
    u32 lines = 0;
    lines |= (u32)(expo_reg[0].data & 0x000f) << 16;
    lines |= (u32)(expo_reg[1].data & 0x00ff) << 8;
    lines |= (u32)(expo_reg[2].data & 0x00f0) << 0;
    lines >>= 4;
    *us = (lines * Preview_line_period) / 1000 / 2; // return us

    SENSOR_DMSG("[%s] sensor expo lines/us %d, %dus\n", __FUNCTION__, lines, *us);
    return rc;
}

static int sc2335SetAEUSecs(ms_cus_sensor* handle, u32 us)
{
    int i;
    u32 half_lines = 0, vts = 0;
    sc2335_params* params = (sc2335_params*)handle->private_data;

    I2C_ARRAY expo_reg_temp[] = {
        // max expo line vts-4!
        { 0x3e00, 0x00 }, // expo [20:17]
        { 0x3e01, 0x00 }, // expo[16:8]
        { 0x3e02, 0x10 }, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, expo_reg, sizeof(expo_reg));

    half_lines = (1000 * us * 2) / Preview_line_period; // Preview_line_period in ns
    if (half_lines < 2)
        half_lines = 2;
    if (half_lines > 2 * (params->expo.vts) - 10) {
        vts = (half_lines + 11) / 2;
    } else
        vts = params->expo.vts;
    params->expo.line = half_lines;
    SENSOR_DMSG("[%s] us %ld, half_lines %ld, vts %ld\n", __FUNCTION__, us, half_lines, params->expo.vts);

    half_lines = half_lines << 4;
    //  printf("===================================================================\n");
    //  printf("us = %d  half_lines = %x params->expo.vts = %x\n",us, half_lines, params->expo.vts);
    //  printf("===================================================================\n");
    expo_reg[0].data = (half_lines >> 16) & 0x0f;
    expo_reg[1].data = (half_lines >> 8) & 0xff;
    expo_reg[2].data = (half_lines >> 0) & 0xf0;
    vts_reg[0].data = (vts >> 8) & 0x00ff;
    vts_reg[1].data = (vts >> 0) & 0x00ff;

    for (i = 0; i < sizeof(expo_reg) / sizeof(I2C_ARRAY); i++) {
        if (expo_reg[i].data != expo_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }

    return SUCCESS;
}

// Gain: 1x = 1024
volatile static u32 ori_gain = 1024;
static int sc2335GetAEGain(ms_cus_sensor* handle, u32* gain)
{
    int rc = 0;

    return rc;
}

static int sc2335SetAEGain(ms_cus_sensor* handle, u32 gain)
{
    sc2335_params* params = (sc2335_params*)handle->private_data;
    u8 i = 0, Dgain = 1, Ana_gain = 1;
    u64 Fine_againx64 = 64, Fine_dgainx128 = 128;
    u8 Dgain_reg = 0, Ana_gain_reg = 0, Fine_again_reg = 0x40, Fine_dgain_reg = 0x80;

    I2C_ARRAY gain_reg_temp[] = {
        { 0x3e06, 0x00 },
        { 0x3e07, 0x80 },
        { 0x3e08, 0x03 },
        { 0x3e09, 0x40 },
    };
    I2C_ARRAY nr_reg_temp[] = {
        { 0x363c, 0x0e },
    };
    I2C_ARRAY temperature_reg_1_temp[] = {
        { 0x5799, 0x07 },
    };

    memcpy(gain_reg_temp, gain_reg, sizeof(gain_reg));
    memcpy(nr_reg_temp, nr_reg, sizeof(nr_reg));
    memcpy(temperature_reg_1_temp, temperature_reg_1, sizeof(temperature_reg_1));

    if (gain < 1024) {
        gain = 1024;
    } else if (gain >= SENSOR_MAXGAIN * 1024) {
        gain = SENSOR_MAXGAIN * 1024;
    }

    if (gain < 2 * 1024) {
        Dgain = 1;
        Fine_dgainx128 = 128;
        Ana_gain = 1;
        Dgain_reg = 0x00;
        Fine_dgain_reg = 0x80;
        Ana_gain_reg = 0x03;
    } else if (gain < 4 * 1024) {
        Dgain = 1;
        Fine_dgainx128 = 128;
        Ana_gain = 2;
        Dgain_reg = 0x00;
        Fine_dgain_reg = 0x80;
        Ana_gain_reg = 0x07;
    } else if (gain < 8 * 1024) {
        Dgain = 1;
        Fine_dgainx128 = 128;
        Ana_gain = 4;
        Dgain_reg = 0x00;
        Fine_dgain_reg = 0x80;
        Ana_gain_reg = 0x0f;
    } else if (gain <= 16256) {
        Dgain = 1;
        Fine_dgainx128 = 128;
        Ana_gain = 8;
        Dgain_reg = 0x00;
        Fine_dgain_reg = 0x80;
        Ana_gain_reg = 0x1f;
    } else if (gain < 32512) {
        Dgain = 1;
        Fine_againx64 = 127;
        Ana_gain = 8;
        Dgain_reg = 0x00;
        Fine_again_reg = 0x7f;
        Ana_gain_reg = 0x1f;
    } else if (gain < 65024) {
        Dgain = 2;
        Fine_againx64 = 127;
        Ana_gain = 8;
        Dgain_reg = 0x01;
        Fine_again_reg = 0x7f;
        Ana_gain_reg = 0x1f;
    } else if (gain < 127 * 1024) {
        Dgain = 4;
        Fine_againx64 = 127;
        Ana_gain = 8;
        Dgain_reg = 0x03;
        Fine_again_reg = 0x7f;
        Ana_gain_reg = 0x1f;
    } else if (gain < 254 * 1024) {
        Dgain = 8;
        Fine_againx64 = 127;
        Ana_gain = 8;
        Dgain_reg = 0x07;
        Fine_again_reg = 0x7f;
        Ana_gain_reg = 0x1f;
    } else if (gain <= SENSOR_MAXGAIN * 1024) {
        Dgain = 16;
        Fine_againx64 = 127;
        Ana_gain = 8;
        Dgain_reg = 0x0f;
        Fine_again_reg = 0x7f;
        Ana_gain_reg = 0x1f;
    }

    if (gain <= 16256) {
        Fine_againx64 = abs(8 * gain / (Dgain * Ana_gain * Fine_dgainx128));
        // Fine_dgainx128 = abs(8 * gain/ (Dgain * Ana_gain * Fine_againx64));
        Fine_again_reg = Fine_againx64;
        // Fine_dgain_reg = Fine_dgainx128;
    } else {
        Fine_dgainx128 = abs(8 * gain / (Dgain * Ana_gain * Fine_againx64));
        Fine_dgain_reg = Fine_dgainx128;
    }

    gain_reg[3].data = Fine_again_reg;
    gain_reg[2].data = Ana_gain_reg;
    gain_reg[1].data = Fine_dgain_reg;
    gain_reg[0].data = Dgain_reg & 0x0f;

    for (i = 0; i < sizeof(gain_reg) / sizeof(I2C_ARRAY); i++) {
        if (gain_reg[i].data != gain_reg_temp[i].data) {
            params->reg_dirty = true;
            break;
        }
    }

#if ENABLE_NR
    if (reg3040 == 0x40) {
        if (gain_reg_temp[2].data < 0x07) { // delay 1 frame group, group & exp & gain active at the same time
            nr_reg[0].data = 0x0e;
        } else {
            nr_reg[0].data = 0x07;
        }
    } else if (reg3040 == 0x41) {
        if (gain_reg_temp[2].data < 0x07) {
            nr_reg[0].data = 0x0f;
        } else {
            nr_reg[0].data = 0x07;
        }
    } else {
        nr_reg[0].data = 0x07;
    }

    /* Each frame refresh to avoid the red flash when linear and wide dynamic switching */
    for (i = 0; i < sizeof(nr_reg) / sizeof(I2C_ARRAY); i++) {
        if (nr_reg[i].data != nr_reg_temp[i].data) {
            params->nr_dirty = true;
            break;
        }
    }
#endif

    if (gain >= 16256)
        temperature_reg_1[0].data = 0x07;
    else if (gain <= 10240)
        temperature_reg_1[0].data = 0x00;

    for (i = 0; i < sizeof(temperature_reg_1) / sizeof(I2C_ARRAY); i++) {
        if (temperature_reg_1[i].data != temperature_reg_1_temp[i].data) {
            params->temperature_reg_1_dirty = true;
            break;
        }
    }

    return SUCCESS;
}

static int sc2335GetAEMinMaxUSecs(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 30;
    *max = 1000000 / Preview_MIN_FPS;

    return SUCCESS;
}

static int sc2335GetAEMinMaxGain(ms_cus_sensor* handle, u32* min, u32* max)
{
    *min = 1024;
    *max = SENSOR_MAXGAIN * 1024;

    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    // u32 i, j;

    SENSOR_DMSG("[%s]%d, %d, %d, %d\n", __FUNCTION__, num, pArray[0].gain, pArray[1].gain, pArray[num - 1].offset);

    return SUCCESS;
}

static int sc2335GetShutterInfo(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info)
{
    info->max = 1000000000 / Preview_MIN_FPS;
    info->min = Preview_line_period;
    info->step = Preview_line_period / 2;
    return SUCCESS;
}

int sc2335ReleaseHandle(ms_cus_sensor* handle)
{
    SENSOR_DMSG("[%s]\n", __FUNCTION__);
    if (handle && handle->private_data) {
        SENSOR_IMSG("[%s] release handle 0x%x, private data %x\n", __FUNCTION__, (int)handle, (int)handle->private_data);
        CamOsMemRelease(handle->private_data);
        handle->private_data = NULL;
    }
    return SUCCESS;
}

int sc2335InitHandle(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor* handle = drv_handle;
    sc2335_params* params = 0;
    if (!handle) {
        SENSOR_EMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }

    SENSOR_DMSG("[%s]\n", __FUNCTION__);
    // private data allocation & init
    handle->private_data = CamOsMemCalloc(1, sizeof(sc2335_params));
    params = (sc2335_params*)handle->private_data;

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id, "sc2335_PARL");

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
    sprintf(handle->video_res_supported.res[0].strResDesc, "1920x1080@30fps");

    // i2c
    handle->i2c_cfg.mode = SENSOR_I2C_LEGACY; //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt = SENSOR_I2C_FMT; // CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address = SENSOR_I2C_ADDR; // 0x5a;
    handle->i2c_cfg.speed = SENSOR_I2C_SPEED; // 200000;

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
    handle->sat_mingain = 1024;
    handle->pCus_sensor_release = sc2335ReleaseHandle;
    handle->pCus_sensor_init = sc2335Init;
    handle->pCus_sensor_poweron = sc2335poweron;
    handle->pCus_sensor_poweroff = sc2335poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID = sc2335GetSensorID;

    handle->pCus_sensor_GetVideoResNum = sc2335GetVideoResNum;
    handle->pCus_sensor_GetVideoRes = sc2335GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes = sc2335GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes = sc2335SetVideoRes;

    handle->pCus_sensor_GetOrien = sc2335GetOrien;
    handle->pCus_sensor_SetOrien = sc2335SetOrien;
    handle->pCus_sensor_GetFPS = sc2335GetFPS;
    handle->pCus_sensor_SetFPS = sc2335SetFPS;
    handle->pCus_sensor_SetPatternMode = sc2335SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    handle->pCus_sensor_AEStatusNotify = sc2335AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs = sc2335GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs = sc2335SetAEUSecs;
    handle->pCus_sensor_GetAEGain = sc2335GetAEGain;
    handle->pCus_sensor_SetAEGain = sc2335SetAEGain;
    handle->pCus_sensor_SetAEGain_cal = sc2335SetAEGain;
    handle->pCus_sensor_GetShutterInfo = sc2335GetShutterInfo;

    handle->pCus_sensor_GetAEMinMaxGain = sc2335GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs = sc2335GetAEMinMaxUSecs;

    handle->pCus_sensor_setCaliData_gain_linearity = pCus_setCaliData_gain_linearity;
    params->expo.vts = vts_30fps;
    params->expo.fps = 30;
    params->expo.line = 1000;
    params->reg_dirty = false;
    params->reg_mf = false;
    params->nr_dirty = false;

    // handle->channel_num = SENSOR_CHANNEL_NUM;
    // handle->channel_mode = SENSOR_CHANNEL_MODE;
    return SUCCESS;
}

/******************** Linux kernel library *******************/
int chmap = 0;
module_param(chmap, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(chmap, "VIF channel mapping");

static int __init sigmastar_sc2335_init_driver(void)
{
    int nCamID = 0;
    for (nCamID = 0; nCamID < 4; ++nCamID) {
        if ((chmap >> nCamID) & 0x1) // if bitmap bit is 1, register sensor
        {
            DrvRegisterSensorDriver(nCamID, sc2335InitHandle);
            pr_info("Connect %s linear to vif channel-%d\n", __FUNCTION__, nCamID);
        }
    }
    return 0;
}
static void __exit sigmastar_sc2335_exit_driver(void)
{
    int nCamID = 0;
    for (nCamID = 0; nCamID < 4; ++nCamID) {
        if ((chmap >> nCamID) & 0x1) // if bitmap bit is 1, register sensor
        {
            DrvSensorRelease(nCamID);
            pr_info("Release sensor with CamID %d\n", nCamID);
        }
    }
    pr_info("sensordrv exit");
}

subsys_initcall(sigmastar_sc2335_init_driver);
module_exit(sigmastar_sc2335_exit_driver);

MODULE_DESCRIPTION("Sensor_sc2335");
MODULE_AUTHOR("SigmaStar");
MODULE_LICENSE("GPL");
