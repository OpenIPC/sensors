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

/*! @file drv_ms_cus_sensor.h
      @brief This file contains Infinity ISP sensor driver interface.
*/

/** @defgroup group1 ISP Sensor Driver Interface
 * @{
 */

#ifndef DRV_MS_CUS_SENSOR_H_
#define DRV_MS_CUS_SENSOR_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <sensor_i2c_api.h>

#define I2C_RETRYTIME (5)

#ifndef SUCCESS
#define FAIL (-1)
#define SUCCESS 0
#endif

#ifdef __cplusplus
#define EXPORT_CUS extern "C"
#else
#define EXPORT_CUS
#endif

#define CUS_CAMSENSOR_HANDLE_MAJ_VER 0x0002
#define CUS_CAMSENSOR_HANDLE_MIN_VER 0x0002

#define CUS_CAMSENSORIF_MAJ_VER 0x0002
#define CUS_CAMSENSORIF_MIN_VER 0x0001

#define CUS_CAMSENSOR_I2C_MAJ_VER 0x0001
#define CUS_CAMSENSOR_I2C_MIN_VER 0x0001

#define CUS_MSTART_CAMSENSOR_CAP_VERSION 0x0001

//#define usleep(usec)  CamOsMsSleep(usec*1000);
//#define usleep(usec)  udelay(usec)
//#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#define SENSOR_DRIVER_MODE_NOT_SUUPORT (0xFFFF)

struct __ms_cus_sensor; /**< Sensor driver handle */
// struct __ISensorAPI; /**< Sensor to ISP control interface */

/*! @brief Resolution descriptor*/
typedef struct _cus_camsensor_res {
    u32 width; /**< Image crop width */
    u32 height; /**< Image crop height */
    u32 max_fps; /**< Max fps in this resolution */
    u32 min_fps; /**< Min fps in this resolution*/
    u32 crop_start_x;
    u32 crop_start_y;
    u32 nOutputWidth; /**< Sensor actual output width */
    u32 nOutputHeight; /**< Sensor actual output height */
    char strResDesc[32];
    u32 nMinFrameLengthLine; // in Line
    u32 nRowTime;
} __attribute__((packed, aligned(4))) cus_camsensor_res;

/*! @brief Resolution list*/
typedef struct _cus_camsensor_res_list {
    u32 num_res; /**< number of sensor resolution in list */
    u32 ulcur_res; /**< current sensor resolution*/
    cus_camsensor_res res[31]; /**< resolution list */
} __attribute__((packed, aligned(4))) cus_camsensor_res_list;

/*! @brief Select sensor type */
typedef enum {
    ISP_SOC, /**< Not support */
    ISP_EXT /**< sensor without built-in ISP */
} CUS_CAMSENSOR_ISPTYPE;

/*! @brief Sensor bayer raw pixel order */
typedef enum {
    CUS_BAYER_RG = 0, /**< bayer data start with R channel */
    CUS_BAYER_GR, /**<  bayer data start with Gr channel */
    CUS_BAYER_BG, /**<  bayer data start with B channel */
    CUS_BAYER_GB /**<  bayer data start with Gb channel */
} CUS_SEN_BAYER;

typedef enum {
    CUS_RGBIR_NONE = 0, /** modify RGBIR pixel order enumeration */
    CUS_RGBIR_R0 = 1,
    CUS_RGBIR_G0 = 2,
    CUS_RGBIR_B0 = 3,
    CUS_RGBIR_G1 = 4,
    CUS_RGBIR_G2 = 5,
    CUS_RGBIR_I0 = 6,
    CUS_RGBIR_G3 = 7,
    CUS_RGBIR_I1 = 8
} CUS_SEN_RGBIR;

/*! @brief Set sensor image mirror and flip.*/
typedef enum {
    CUS_ORIT_M0F0, /**< mirror, flip unchanged */
    CUS_ORIT_M1F0, /**< mirror changed, flip unchanged */
    CUS_ORIT_M0F1, /**< mirror unchanged, flip changed */
    CUS_ORIT_M1F1, /**< mirror and flip changed */
} CUS_CAMSENSOR_ORIT;

typedef enum {
    CUS_SNR_ANADEC_STATUS_NO_READY = 0,
    CUS_SNR_ANADEC_STATUS_DISCNT,
    CUS_SNR_ANADEC_STATUS_CONNECT,
    CUS_SNR_ANADEC_STATUS_NUM
} CUS_SNR_Anadec_Status_e;

typedef enum {
    CUS_SNR_ANADEC_TRANSFERMODE_CVBS = 0,
    CUS_SNR_ANADEC_TRANSFERMODE_CVI,
    CUS_SNR_ANADEC_TRANSFERMODE_TVI,
    CUS_SNR_ANADEC_TRANSFERMODE_AHD,
    CUS_SNR_ANADEC_TRANSFERMODE_NUM,
} CUS_SNR_Anadec_TransferMode_e;

typedef enum {
    CUS_SNR_ANADEC_FORMAT_NTSC = 0,
    CUS_SNR_ANADEC_FORMAT_PAL,
    CUS_SNR_ANADEC_FORMAT_NUM,
} CUS_SNR_Anadec_Format_e;

/*! @brief Get input source type.*/
typedef enum {
    CUS_SNR_ANADEC_SRC_NO_READY, /**< input no ready */
    CUS_SNR_ANADEC_SRC_DISCNT, /**< input disconnect */
    CUS_SNR_ANADEC_SRC_PAL, /**< input type is PAL */
    CUS_SNR_ANADEC_SRC_NTSC, /**< input type is NTSC */
    CUS_SNR_ANADEC_SRC_HD_25P, /**< input source type is HD */
    CUS_SNR_ANADEC_SRC_HD_30P, /**< input source type is HD */
    CUS_SNR_ANADEC_SRC_HD_50P, /**< input source type is HD */
    CUS_SNR_ANADEC_SRC_HD_60P, /**< input source type is HD */
    CUS_SNR_ANADEC_SRC_FHD_25P, /**< input source type is FHD */
    CUS_SNR_ANADEC_SRC_FHD_30P, /**< input source type is FHD */
} CUS_SNR_ANADEC_SRC_TYPE;

/*! @brief Get input source type.*/
typedef struct CUS_SNR_Anadec_SrcAttr_s {
    CUS_SNR_Anadec_Status_e eStatus;
    CUS_SNR_Anadec_TransferMode_e eTransferMode;
    CUS_SNR_Anadec_Format_e eFormat;
    u32 width;
    u32 height;
    u32 u32Fps;
} CUS_SNR_Anadec_SrcAttr_t;

/*! @brief ISP AE event notifycation*/
typedef enum {
    CUS_FRAME_INACTIVE = 0, /**< Frame end */
    CUS_FRAME_ACTIVE = 1, /**< Frame start */
} CUS_CAMSENSOR_AE_STATUS_NOTIFY;

/*! @brief Sensor bayer raw (8/10 bits) to 12bits mode control */
typedef enum {
    CUS_SEN_8TO12_7074, /**< Do not use*/
    CUS_SEN_8TO12_7000, /**< Sensor bayer raw is 8 bits*/
    CUS_SEN_8TO12_114118, /**< Do not use*/
    CUS_SEN_8TO12_11400, /**< Do not use*/
    CUS_SEN_10TO12_9098, /**< Do not use*/
    CUS_SEN_10TO12_9000, /**< Sensor bayer raw is 10 bits*/
    CUS_SEN_10TO12_1121110, /**< Do not use*/
    CUS_SEN_10TO12_11200 /**< Do not use*/
} CUS_SEN_FMT_CONV_MODE;

/*! @brief Sensor input raw data precision */
typedef enum {
    CUS_DATAPRECISION_8 = 0, /**< raw data precision is 8bits */
    CUS_DATAPRECISION_10 = 1, /**< raw data precision is 10bits */
    CUS_DATAPRECISION_16 = 2, /**< raw data precision is 16bits */
    CUS_DATAPRECISION_12 = 3, /**< raw data precision is 12bits */
    CUS_DATAPRECISION_14 = 4, /**< raw data precision is 14bits */
} CUS_DATAPRECISION;

/*! @brief Select sensor data intarface */
typedef enum {
    CUS_SENIF_BUS_PARL = 0, /**< sensor data bus is parallel bus */
    CUS_SENIF_BUS_MIPI = 1, /**<  sensor data bus is mipi */
    CUS_SENIF_BUS_BT601 = 2,
    CUS_SENIF_BUS_BT656 = 3,
    CUS_SENIF_BUS_BT1120 = 4,
    CUS_SENIF_BUS_LVDS = 5, /**<  sensor data bus is lvds */
    CUS_SENIF_BUS_MAX
} CUS_SENIF_BUS;

typedef enum {
    CUS_SEN_INPUT_FORMAT_YUV422,
    CUS_SEN_INPUT_FORMAT_RGB,
} CUS_SEN_INPUT_FORMAT;

/*! @brief Select pin polarity */
typedef enum {
    CUS_CLK_POL_POS = 0, /**< High active */
    CUS_CLK_POL_NEG /**< Low active */
} CUS_CLK_POL;

typedef enum {
    CUS_SENSOR_YUV_ORDER_CY = 0,
    CUS_SENSOR_YUV_ORDER_YC = 1,
} CUS_SENSOR_YUV_ORDER;

/*! @brief Sensor master clock select */
typedef enum {
    CUS_CMU_CLK_27MHZ,
    CUS_CMU_CLK_21P6MHZ,
    CUS_CMU_CLK_12MHZ,
    CUS_CMU_CLK_5P4MHZ,
    CUS_CMU_CLK_36MHZ,
    CUS_CMU_CLK_54MHZ,
    CUS_CMU_CLK_43P2MHZ,
    CUS_CMU_CLK_61P7MHZ,
    CUS_CMU_CLK_72MHZ,
    CUS_CMU_CLK_48MHZ,
    CUS_CMU_CLK_24MHZ,
    CUS_CMU_CLK_37P125MHZ,
    CUS_CMU_CLK_LPLL_DIV1,
    CUS_CMU_CLK_LPLL_DIV2,
    CUS_CMU_CLK_LPLL_DIV4,
    CUS_CMU_CLK_LPLL_DIV8,
} CUS_MCLK_FREQ; // Depends on chip.

// Depends on chip definition.
typedef enum {
    CUS_SR0_PAR_DISABLE,
    CUS_SR0_PAR_MODE_1,
    CUS_SR0_PAR_MODE_2,
} CUS_SR0_PAR_MODE;

// Depends on chip definition.
typedef enum {
    CUS_SR0_BT656_DISABLE,
    CUS_SR0_BT656_MODE_1,
    CUS_SR0_BT656_MODE_2,
    CUS_SR0_BT656_MODE_3,
    CUS_SR0_BT656_MODE_4,
} CUS_SR0_BT656_MODE;

// Depends on chip definition.
typedef enum {
    CUS_SR1_BT656_DISABLE,
    CUS_SR1_BT656_MODE_1,
} CUS_SR1_BT656_MODE;

// Depends on chip definition.
typedef enum {
    CUS_SR0_BT601_DISABLE,
    CUS_SR0_BT601_MODE_1,
    CUS_SR0_BT601_MODE_2,
    CUS_SR0_BT601_MODE_3,
    CUS_SR0_BT601_MODE_4,
} CUS_SR0_BT601_MODE;

// Depends on chip definition.
typedef enum {
    CUS_SR0_MIPI_DISABLE,
    CUS_SR0_MIPI_MODE_1,
    CUS_SR0_MIPI_MODE_2,
} CUS_SR0_MIPI_MODE;

// Depends on chip definition.
typedef enum {
    CUS_SR1_MIPI_DISABLE,
    CUS_SR1_MIPI_MODE_1,
    CUS_SR1_MIPI_MODE_2,
    CUS_SR1_MIPI_MODE_3,
    CUS_SR1_MIPI_MODE_4,
} CUS_SR1_MIPI_MODE;

// Depends on chip definition.
typedef enum {
    CUS_VIF_BT656_EAV_DETECT = 0,
    CUS_VIF_BT656_SAV_DETECT = 1,
} CUS_VIF_BT656_CHANNEL_SELECT;

// Depends on chip definition.
typedef enum {
    CUS_VIF_BT656_VSYNC_DELAY_1LINE = 0,
    CUS_VIF_BT656_VSYNC_DELAY_2LINE = 1,
    CUS_VIF_BT656_VSYNC_DELAY_0LINE = 2,
    CUS_VIF_BT656_VSYNC_DELAY_AUTO = 3,
} CUS_VIF_BT656_VSYNC_DELAY;

typedef enum {
    CUS_BT656_CLK_EDGE_SINGLE_UP,
    CUS_BT656_CLK_EDGE_SINGLE_DOWN,
    CUS_BT656_CLK_EDGE_DOUBLE,
    CUS_BT656_CLK_EDGE_MAX
} CUS_BT656_ClkEdge_e;

typedef enum {
    CUS_SENSOR_FUNC_DISABLE = 0,
    CUS_SENSOR_FUNC_ENABLE = 1,
} CUS_SENSOR_FUNC;

typedef enum {
    CUS_SENSOR_PAD_GROUP_A = 0,
    CUS_SENSOR_PAD_GROUP_B = 1,
    CUS_SENSOR_PAD_GROUP_C = 2,
    CUS_SENSOR_PAD_GROUP_D = 3,
} CUS_SENSOR_PAD_GROUP;

typedef enum {
    CUS_SENSOR_MASTER_MODE = 0,
    CUS_SENSOR_SLAVE_MODE = 1,
} CUS_SENSOR_MODE;

typedef enum {
    CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL = 0,
    CUS_SENSOR_CHANNEL_MODE_REALTIME_HDR = 1,
    CUS_SENSOR_CHANNEL_MODE_RAW_STORE = 2,
    CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR = 3,
} CUS_SENSOR_CHANNEL_MODE;

typedef struct {
    unsigned int gain;
    unsigned int offset;
} CUS_GAIN_GAP_ARRAY;

//////////////////////////////////////
// sensor functions
//////////////////////////////////////

typedef struct {
    u32 length; // header length
    u32 version; // version
} CUS_CAMSENSOR_CAP;

/////////////////// ISP for SENSOR API ///////////////////
typedef enum {
    CUS_INT_TASK_AE = (1 << 0),
    CUS_INT_TASK_AWB = (1 << 1),
    CUS_INT_TASK_AF = (1 << 2),
    CUS_INT_TASK_VS = (1 << 3),
    CUS_INT_TASK_VDOS = (1 << 4),
} CUS_INT_TASK_TYPE;

#define MAX_RUN_ORDER 16
typedef struct {
    u8 RunLength;
    u8 Orders[MAX_RUN_ORDER];
    u8 CurTaskType;
} CUS_INT_TASK_ORDER;

/////////////////// Shutter Info ///////////////////////
/*! @brief Report shutter information */
typedef struct {
    u32 length; /**< struct size */
    u32 max; /**< maximun shutter in us*/
    u32 min; /**< minimum shutter in us*/
    u32 step; /**< shutter in step us*/
} CUS_SHUTTER_INFO;

////////////////// CSI CLOCK ////////////////////////
/*! @brief Select MIPI clock*/
typedef enum {
    CUS_CSI_CLK_DISABLE = -1, /**< Disable MIPI clock*/
    CUS_CSI_CLK_108M = 0, /**< MIPI pixel rate 864Mbps (1-lane)*/
    CUS_CSI_CLK_86M = 1, /**< MIPI pixel rate 344Mbps(1-lane)*/
    CUS_CSI_CLK_144M = 2, /**< MIPI pixel rate 1152Mbps(1-lane)*/
    CUS_CSI_CLK_172M = 3,
    CUS_CSI_CLK_216M = 4,
    CUS_CSI_CLK_288M = 5,
    CUS_CSI_CLK_40M = 6,
    CUS_CSI_CLK_24M = 7,
    CUS_CSI_CLK_12M = 8,
    CUS_CSI_CLK_123M = 9,
    CUS_CSI_CLK_320M = 10,
    CUS_CSI_CLK_384M = 11,
    CUS_CSI_CLK_432M = 12
} CUS_CSI_CLK;

///////////////// SENSOR PIN CONFIG/////////////////
/*! @brief Sensor pin assignment*/
typedef enum {
    CUS_SR_PAD_CFG_1 = 1, /**< Pin config for MIPI 1/2 lanes*/
    CUS_SR_PAD_CFG_MIPI = 1, /**< Pin config for  MIPI 1/2 lanes*/
    CUS_SR_PAD_CFG_2 = 2, /**< Reserved */
    CUS_SR_PAD_CFG_3 = 3, /**< Reserved */
    CUS_SR_PAD_CFG_10BITS = 4, /**< Pin config for parallel interface 10 bits */
    CUS_SR_PAD_CFG_4 = 4, /**< Pin config for parallel interface 12 bits */
    CUS_SR_PAD_CFG_12BITS = 5, /**< Pin config for parallel interface 12 bits */
    CUS_SR_PAD_CFG_5 = 5, /**< Pin config for parallel interface 12 bits */
    CUS_SR_PAD_CFG_6 = 6 /**< Pin config for parallel interface 12 bits */
} CUS_SR_PAD_CFG;

/*! @brief virtual channel0 hsync. mode */
typedef enum {
    PACKET_HEADER_EDGE1 = 0, /**< packet header edge */
    PACKET_HEADER_EDGE2 = 1, /**< line end edge */
    PACKET_HEADER_EDGE3 = 2, /**< line start edge */
    PACKET_FOOTER_EDGE = 3, /**< packet footer edge */
} CUS_CSI_VC_HS_MODE;

typedef enum {
    CUS_HDR_MODE_NONE = 0,
    CUS_HDR_MODE_SONY_DOL = 1,
    CUS_HDR_MODE_DCG = 2,
    CUS_HDR_MODE_EMBEDDED_RAW8 = 3,
    CUS_HDR_MODE_EMBEDDED_RAW10 = 4,
    CUS_HDR_MODE_EMBEDDED_RAW12 = 5,
    CUS_HDR_MODE_EMBEDDED_RAW14 = 6,
    CUS_HDR_MODE_EMBEDDED_RAW16 = 7,
    CUS_HDR_MODE_LI = 8,
    CUS_HDR_MODE_MULTI_VC = 9,
} CUS_HDR_MODE;

typedef enum {
    // Index 0
    SENSOR_DRIVER_MODE_VGA_30P_RESOLUTION, // 640*360 30P
    SENSOR_DRIVER_MODE_VGA_50P_RESOLUTION, // 640*360 50P
    SENSOR_DRIVER_MODE_VGA_60P_RESOLUTION, // 640*360 60P
    SENSOR_DRIVER_MODE_VGA_100P_RESOLUTION, // 640*360 100P
    SENSOR_DRIVER_MODE_VGA_120P_RESOLUTION, // 640*360 120P

    // Index 5
    SENSOR_DRIVER_MODE_HD_24P_RESOLUTION, // 1280*720 24P
    SENSOR_DRIVER_MODE_HD_30P_RESOLUTION, // 1280*720 30P
    SENSOR_DRIVER_MODE_HD_50P_RESOLUTION, // 1280*720 50P
    SENSOR_DRIVER_MODE_HD_60P_RESOLUTION, // 1280*720 60P
    SENSOR_DRIVER_MODE_HD_100P_RESOLUTION, // 1280*720 100P

    // Index 10
    SENSOR_DRIVER_MODE_HD_120P_RESOLUTION, // 1280*720 120P
    SENSOR_DRIVER_MODE_1600x900_30P_RESOLUTION, // 1600*900 30P
    SENSOR_DRIVER_MODE_FULL_HD_15P_RESOLUTION, // 1920*1080 15P
    SENSOR_DRIVER_MODE_FULL_HD_24P_RESOLUTION, // 1920*1080 24P
    SENSOR_DRIVER_MODE_FULL_HD_25P_RESOLUTION, // 1920*1080 25P

    // Index 15
    SENSOR_DRIVER_MODE_FULL_HD_30P_RESOLUTION, // 1920*1080 30P
    SENSOR_DRIVER_MODE_FULL_HD_50P_RESOLUTION, // 1920*1080 50P
    SENSOR_DRIVER_MODE_FULL_HD_60P_RESOLUTION, // 1920*1080 60P
    SENSOR_DRIVER_MODE_SUPER_HD_30P_RESOLUTION, // 2304*1296 30P
    SENSOR_DRIVER_MODE_SUPER_HD_25P_RESOLUTION, // 2304*1296 25P

    // Index 20
    SENSOR_DRIVER_MODE_SUPER_HD_24P_RESOLUTION, // 2304*1296 24P
    SENSOR_DRIVER_MODE_1440_30P_RESOLUTION, // 2560*1440 30P
    SENSOR_DRIVER_MODE_2D7K_15P_RESOLUTION, // 2704*1524 15P
    SENSOR_DRIVER_MODE_2D7K_30P_RESOLUTION, // 2704*1524 30P
    SENSOR_DRIVER_MODE_4K2K_15P_RESOLUTION, // 3840*2160 15P

    // Index 25
    SENSOR_DRIVER_MODE_4K2K_30P_RESOLUTION, // 3840*2160 30P
    SENSOR_DRIVER_MODE_4TO3_VGA_30P_RESOLUTION, // 640*480   30P
    SENSOR_DRIVER_MODE_4TO3_1D2M_30P_RESOLUTION, // 1280*960  30P
    SENSOR_DRIVER_MODE_4TO3_1D5M_30P_RESOLUTION, // 1440*1080 30P
    SENSOR_DRIVER_MODE_4TO3_3M_15P_RESOLUTION, // 2048*1536 15P

    // Index 30
    SENSOR_DRIVER_MODE_4TO3_3M_30P_RESOLUTION, // 2048*1536 30P
    SENSOR_DRIVER_MODE_4TO3_5M_15P_RESOLUTION, // 2560*1920 15P
    SENSOR_DRIVER_MODE_4TO3_5M_30P_RESOLUTION, // 2560*1920 30P
    SENSOR_DRIVER_MODE_4TO3_8M_15P_RESOLUTION, // 3264*2448 15P
    SENSOR_DRIVER_MODE_4TO3_8M_30P_RESOLUTION, // 3264*2448 30P

    // Index 35
    SENSOR_DRIVER_MODE_4TO3_10M_15P_RESOLUTION, // 3648*2736 15P
    SENSOR_DRIVER_MODE_4TO3_10M_30P_RESOLUTION, // 3648*2736 30P
    SENSOR_DRIVER_MODE_4TO3_12M_15P_RESOLUTION, // 4032*3024 15P
    SENSOR_DRIVER_MODE_4TO3_12M_30P_RESOLUTION, // 4032*3024 30P
    SENSOR_DRIVER_MODE_4TO3_14M_15P_RESOLUTION, // 4352*3264 15P

    // Index 40
    SENSOR_DRIVER_MODE_4TO3_14M_30P_RESOLUTION, // 4352*3264 30P
    SENSOR_DRIVER_MODE_4K2K_24P_RESOLUTION,
    SENSOR_DRIVER_MODE_PAL_25P_RESOLUTION,
    SENSOR_DRIVER_MODE_NTSC_30P_RESOLUTION,

    // For Camera Preview
    SENSOR_DRIVER_MODE_BEST_CAMERA_PREVIEW_RESOLUTION,
    SENSOR_DRIVER_MODE_BEST_CAMERA_CAPTURE_16TO9_RESOLUTION,
    SENSOR_DRIVER_MODE_BEST_CAMERA_CAPTURE_4TO3_RESOLUTION,
    SENSOR_DRIVER_MODE_FULL_HD_30P_RESOLUTION_HDR,
} CUS_SNR_RESOLUTION;

typedef enum {
    CUS_IRLED_EVT_FRM_LINE_CNT = 0,
    CUS_IRLED_EVT_FRMST,
    CUS_IRLED_EVT_FRMEND,
    CUS_IRLED_EVT_MAX
} CUS_IRLED_EVT_TYPE;

typedef struct
{
    u8 bOutputInvert;
    CUS_IRLED_EVT_TYPE StartEvtType;
    CUS_IRLED_EVT_TYPE EndEvtType;
    u16 nStartLine;
    u16 nEndLine;
    u8 nStartFrm;
    u8 nEndFrm;
} CUS_IRLED_EVT_CFG;

typedef struct {
    u32 nStartDelay;
    u32 nEndDelay;
} CUS_IRLED_DELAY_CFG;

//========================== SENSOR LVDS CONTROL ========================
/**@brief LVDS interface control id */
typedef enum {
    CUS_LVDS_CTRL_ID_INIT, /** <Open LVDS port, Parameter type CUS_LVDS_INIT_PARAM_t*/
    CUS_LVDS_CTRL_ID_DEINIT, /** <Close LVDS port */
    CUS_LVDS_CTRL_ID_SET_CLK, /** <Set maximum mipi data rate (amount of all lans), Parameter type CUS_LVDS_NUM_VC_PARAM_t*/
    CUS_LVDS_CTRL_ID_SET_NUM_VC, /** <Set number of LVDS virtual channel, Parameter type CUS_LVDS_NUM_VC_PARAM_t*/
    CUS_LVDS_CTRL_ID_SET_LANE, /** <Set number of LVDS lanes, Parameter type CUS_LVDS_LANE_PARAM_t*/
    CUS_LVDS_CTRL_ID_SET_BPP, /** <Set LVDS bit per pixel, Parameter type CUS_LVDS_BPP_PARAM_t */
    CUS_LVDS_CTRL_ID_SET_SYNC_CODE, /** <Set LVDS sync code, The parameter type is CUS_LVDS_SYNC_CODE_PARAM_t */

    CUS_LVDS_CTRL_ID_SET_L0_2LANE_SEL, /** <Set LVDS layer0 4lane mapping, The parameter type is CUS_LVDS_L0_2LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_L1_2LANE_SEL, /** <Set LVDS layer1 4lane mapping, The parameter type is CUS_LVDS_L1_2LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_2LANE_PN_SWAP, /** <Set LVDS layer1 4lane pn swap, The parameter type is CUS_LVDS_2LANE_PN_SWAP_t */

    CUS_LVDS_CTRL_ID_SET_L0_4LANE_SEL, /** <Set LVDS layer0 4lane mapping, The parameter type is CUS_LVDS_L0_4LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_L1_4LANE_SEL, /** <Set LVDS layer1 4lane mapping, The parameter type is CUS_LVDS_L1_4LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_4LANE_PN_SWAP, /** <Set LVDS layer1 4lane pn swap, The parameter type is CUS_LVDS_4LANE_PN_SWAP_t */

    CUS_LVDS_CTRL_ID_SET_L0_8LANE_SEL, /** <Set LVDS layer0 4lane mapping, The parameter type is CUS_LVDS_L0_8LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_L1_8LANE_SEL, /** <Set LVDS layer1 4lane mapping, The parameter type is CUS_LVDS_L1_8LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_8LANE_PN_SWAP, /** <Set LVDS layer1 4lane pn swap, The parameter type is CUS_LVDS_8LANE_PN_SWAP_t */

    CUS_LVDS_CTRL_ID_SET_L0_16LANE_SEL, /** <Set LVDS layer0 4lane mapping, The parameter type is CUS_LVDS_L0_16LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_L1_16LANE_SEL, /** <Set LVDS layer1 4lane mapping, The parameter type is CUS_LVDS_L1_16LANE_SEL_t */
    CUS_LVDS_CTRL_ID_SET_16LANE_PN_SWAP, /** <Set LVDS layer1 4lane pn swap, The parameter type is CUS_LVDS_16LANE_PN_SWAP_t */

    CUS_LVDS_CTRL_ID_MAX,
} CUS_LVDS_CTRL_ID_e;

/** @brief Open LVDS port */
typedef struct
{
    u16 uNumLane; /**< num_lan Number of lanes. */
} CUS_LVDS_INIT_PARAM_t;

/** @brief Set maximum LVDS data rate (amount of all lans) */
typedef struct
{
    u16 uNumLane; /**< num_lan Number of lanes. */
    CUS_CSI_CLK eClk; /**< LVDS Max data rate */
} CUS_LVDS_CLK_PARAM_t;

/** @brief Set number of LVDS virtual channel */
typedef struct
{
    u16 uNumVc; /**< Number of virtual channel. */
} CUS_LVDS_NUM_VC_PARAM_t;

/** @brief Set number of LVDS lanes */
typedef struct
{
    u16 uNumLane; /**< Set number of LVDS lanes */
    u8 bEn; /**< Clock ON/OFF control. */
} CUS_LVDS_LANE_PARAM_t;

/** @brief Set LVDS bit per pixel */
typedef struct
{
    CUS_DATAPRECISION uBpp; /**< pixel precision , select 8/10/12 bits */
} CUS_LVDS_BPP_PARAM_t;

/** @brief Customize LVDS sync code, This function can only support to customize 4th byte of sync code */
typedef struct
{
    u8 uVc; /**< Select a virtual channel to configure. range 0~2 */
    u16 uValidSav; /**< Valid line SAV 4th code */
    u16 uValidEav; /**< Valid line EAV 4th code */
    u16 uInvalidSav; /**< Invalid line SAV 4th code */
    u16 uInvalidEav; /**< Invalid line EAV 4th code */
    u16 uMask; /**< Sync code mask, set corresponding bit 1 to indicate the don't care bit in eav and sav */
} CUS_LVDS_SYNC_CODE_PARAM_t, CUS_LVDS_MODE0_SYNC_CODE_PARAM_t;

/** @brief Customize LVDS sync code, This function can only support to customize 4th byte of sync code */
typedef struct
{
    u8 uVc; /**< Select a virtual channel to configure. range 0~2 */
    u16 uSol; /**< Start of line 4th code */
    u16 uEol; /**< End of line code */
    u16 uSof; /**< Start of frame code */
    u16 uEof; /**< End of frame 4th code */
    u16 uMask; /**< Sync code mask, set corresponding bit 1 to indicate the don't care bit in eav and sav */
} CUS_LVDS_MODE1_SYNC_CODE_PARAM_t;

/** @brief Customize LVDS layer 0 lane swap*/
typedef struct
{
    u16 uL0LaneSel[3]; /**< Lane mapping for layer 0 */
} CUS_LVDS_L0_2LANE_SEL_t;

/** @brief Customize LVDS layer 1 lane swap*/
typedef struct
{
    u16 uL1LaneSel[2]; /**< Lane mapping for layer 1 */
} CUS_LVDS_L1_2LANE_SEL_t;

/** @brief Customize LVDS pn swap*/
typedef struct
{
    u8 uPnSwap[3]; /**< PN swap table */
} CUS_LVDS_2LANE_PN_SWAP_t;

/** @brief Customize LVDS layer 0 lane swap*/
typedef struct
{
    u16 uL0LaneSel[5]; /**< Lane mapping for layer 0 */
} CUS_LVDS_L0_4LANE_SEL_t;

/** @brief Customize LVDS layer 1 lane swap*/
typedef struct
{
    u16 uL1LaneSel[4]; /**< Lane mapping for layer 1 */
} CUS_LVDS_L1_4LANE_SEL_t;

/** @brief Customize LVDS pn swap*/
typedef struct
{
    u8 uPnSwap[5]; /**< PN swap table */
} CUS_LVDS_4LANE_PN_SWAP_t;

/** @brief Customize LVDS layer 0 lane swap*/
typedef struct
{
    u16 uL0LaneSel[10]; /**< Lane mapping for layer 0 */
} CUS_LVDS_L0_8LANE_SEL_t;

/** @brief Customize LVDS layer 1 lane swap*/
typedef struct
{
    u16 uL1LaneSel[8]; /**< Lane mapping for layer 1 */
} CUS_LVDS_L1_8LANE_SEL_t;

/** @brief Customize LVDS pn swap*/
typedef struct
{
    u8 uPnSwap[10]; /**< PN swap table */
} CUS_LVDS_8LANE_PN_SWAP_t;

/** @brief Customize LVDS layer 0 lane swap*/
typedef struct
{
    u16 uL0LaneSel[20]; /**< Lane mapping for layer 0 */
} CUS_LVDS_L0_16LANE_SEL_t;

/** @brief Customize LVDS layer 1 lane swap*/
typedef struct
{
    u16 uL1LaneSel[16]; /**< Lane mapping for layer 1 */
} CUS_LVDS_L1_16LANE_SEL_t;

/** @brief Customize LVDS pn swap*/
typedef struct
{
    u8 uPnSwap[20]; /**< PN swap table */
} CUS_LVDS_16LANE_PN_SWAP_t;

typedef struct
{
    union {
        CUS_LVDS_INIT_PARAM_t tInit;
        CUS_LVDS_CLK_PARAM_t tClk;
        CUS_LVDS_NUM_VC_PARAM_t tNumVc;
        CUS_LVDS_LANE_PARAM_t tLane;
        CUS_LVDS_BPP_PARAM_t tBpp;
        CUS_LVDS_SYNC_CODE_PARAM_t tSyncCode;
        CUS_LVDS_L0_2LANE_SEL_t tL0_2LaneSel;
        CUS_LVDS_L1_2LANE_SEL_t tL1_2LaneSel;
        CUS_LVDS_2LANE_PN_SWAP_t t2LanePnSwap;
        CUS_LVDS_L0_4LANE_SEL_t tL0_4LaneSel;
        CUS_LVDS_L1_4LANE_SEL_t tL1_4LaneSel;
        CUS_LVDS_4LANE_PN_SWAP_t t4LanePnSwap;
    };
} CUS_LVDS_PARAM_t;

//========================== SENSOR SLAVE MODE ==========================
typedef enum {
    CUS_SLAVE_MODE_CTRL_ID_INIT,
    CUS_SLAVE_MODE_CTRL_ID_DEINIT,
    CUS_SLAVE_MODE_CTRL_ID_ENABLE_XVS,
    CUS_SLAVE_MODE_CTRL_ID_ENABLE_XHS
} CUS_SLAVE_MODE_CTRL_ID_e;

/**@brief Sensor slave mode control
For XHS, Clock source is uSrcClk from DrvVifSsmInit()
For XVS, Clock source is XHS
*/
typedef struct
{
    u32 uLoSt; /** < Low start timing:
               For XHS, Low start timing = uLoSt * source clock period , range 0 to 65536
               For XVS, Fix to 0 */
    u32 uLoEnd; /** < Low end timing:
                For XHS, Low start timing = uLoEnd*source clock period , range 1 to 65535
                For XVS, Low start timing = uLoEnd*XHS period , range 1 to 65535 */
    u32 uPeriod; /** < Period: uPeriod * source clock period , range 1 to 65535 */
    u32 uParam; /** < [0]:Polarity [1]:XVS leads XHS one cycle, this option is only for XVS */
} CUS_SSM_TIMING_CFG;

/** @brief Configure sensor slave mode timing */
typedef struct
{
    CUS_MCLK_FREQ eSrcSck; /** < Hsync clock source. */
    CUS_SSM_TIMING_CFG tXhs; /** < Hsync parameter */
    CUS_SSM_TIMING_CFG tXvs; /** < Vsync parameter */
} CUS_SLAVE_MODE_INIT_t;

/** @brief Enable Vsync */
typedef struct
{
    u8 bEn; /** < Enable Vsync */
} CUS_SLAVE_MODE_ENABLE_XVS_t;

/** @brief Enable Hsync */
typedef struct
{
    u8 bEn; /** < Enable Hsync */
} CUS_SLAVE_MODE_ENABLE_XHS_t;

typedef struct
{
    union {
        CUS_SLAVE_MODE_INIT_t tInit;
        CUS_SLAVE_MODE_ENABLE_XVS_t tEnableXvs;
        CUS_SLAVE_MODE_ENABLE_XHS_t tEnableXhs;
    };
} CUS_SLAVE_MODE_PARAM_t;

typedef struct
{
    CUS_MCLK_FREQ eSrcSck; /** < Hsync clock source. */
    CUS_CLK_POL ePolHsync; /** < Hsync initial polarity */
    CUS_CLK_POL ePolVsync; /** < Vsync initial polarity */
    u32 uHsyncStartT; /** <Hsync start timing = uHsyncStartT * source clock period , range 0 to 65536 */
    u32 uHsyncEndT; /** <Hsync end timing = uHsyncEndT * source clock period , range 1 to 65535 */
    u32 uHsyncPeriod; /** <Hsync Period: uHsyncPeriod * source clock period , range 1 to 65535 */
    u32 uVsyncStartT; /** <Vsync start timing = uVsyncStartT * XHS period , range 1 to 65535 */
    u32 uVsyncPeriod; /** <Vsync Period: uVsyncPeriod * XHS period , range 1 to 65535 */
    u32 uXTrigPeriod; /** <XTrigger peripd = uXTrigPeriod * source clock period , range 0 to (2^32)-1 */
} CUS_SLAVE_MODE_ATTR_t;

#ifdef __cplusplus
extern "C"
#endif

    /**@brief ISP sensor interface control API */
    typedef struct __ISensorIfAPI // isp sensor interface API
{
    version_info version;
    /**@brief Reserved */
    void* pdata;

    /** @brief Set sensor power down pin.
    @param[in] idx Sensor pad ID.
    @param[in] pol pin polarity.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*PowerOff)(u32 idx, CUS_CLK_POL pol);

    /** @brief Set sensor power reset pin.
    @param[in] idx Sensor pad ID.
    @param[in] pol pin polarity.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*Reset)(u32 idx, CUS_CLK_POL pol);

    /** @brief Configure sensor master clock.
    @param[in] idx Sensor pad ID.
    @param[in] bONOFF Clock ON/OFF control.
    @param[in] mclk Clock frequency Hz.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*MCLK)(u32 idx, u8 bONOFF, CUS_MCLK_FREQ mclk);

    /** @brief Query sensor master clock.
    @param[in] idx Sensor pad ID.
    @param[in] mclk Query if clock frequency Hz is available.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*QueryMCLK)(u32 idx, CUS_MCLK_FREQ mclk);

    /** @brief Query MIPI lane number.
    @param[in] idx Sensor pad ID.
    @param[in] lane_num Query max lane number.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*QueryLaneNum)(u32 idx, u8* max_lane);
#if 0
    /** @brief Select pixel clock source
    @remarks Parallel interface only
    @param[in] handle Handle to sensor driver.
    @param[in] pclk_source Clock source.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*PCLK)(CUS_PCLK_SOURCE pclk_source);        //senect sensor

    /** @brief Configure HSYNC pin polarity.
    @remarks Parallel interface only
    @param[in] handle Handle to sensor driver.
    @param[in] pol pin polarity.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*HsyncPol)(CUS_CLK_POL pol);

    /** @brief [parallel interface only] Configure VSYNC pin polarity.
    @param[in] handle Handle to sensor driver.
    @param[in] pol pin polarity.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*VsyncPol)(CUS_CLK_POL pol);

    /** @brief Configure PCLK pin polarity.
    @param[in] handle Handle to sensor driver.
    @param[in] pol pin polarity.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*ClkPol)(CUS_CLK_POL pol);

    /** @brief Configure sensor bayer raw pixel order
    @param[in] handle Handle to sensor driver.
    @param[in] bayer_id Select pixel order
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*BayerFmt)(CUS_SEN_BAYER bayer_id);

    /** @brief Configure sensor RGBIR (8 or 10bits) convert to 12bit bits isp input, This config will be ignore for 12/16 bits sensor bayer raw input.
    @param[in] handle Handle to sensor driver.
    @param[in] mode Mode select
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*FmtConv)(CUS_SEN_FMT_CONV_MODE mode);

    /** @brief Select sensor image data interface
    @param[in] handle Handle to sensor driver.
    @param[in] bus Sensor interface
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*DataBus)(CUS_SENIF_BUS bus);

    /** @brief Configure sensor bayer raw pixel bits
    @param[in] handle Handle to sensor driver.
    @param[in] prec Select pixel bits
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*DataPrecision)(CUS_DATAPRECISION prec);

    /** @brief Wait isp frame start event
    @param[in] handle Handle to sensor driver.
    @param[in] ms Max timeout in ms
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*WaitVStart)(int ms);

    /** @brief Wait isp frame end event
    @param[in] handle Handle to sensor driver.
    @param[in] ms Max timeout in ms
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*WaitVEnd)(int ms);

    /** @brief Configure 3A task order table
    @param[in] handle Handle to sensor driver.
    @param[in] tasks Task order table
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*Set3ATaskOrder)(CUS_INT_TASK_ORDER tasks);
#endif
    /** @brief Select sensor IO pin assignment
    @param[in] idx Sensor pad ID.
    @param[in] ulSnrType Interface type.
    @param[in] ulSnrPadCfg Pin config.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetIOPad)(u32 idx, CUS_SENIF_BUS ulSnrType, u32 ulSnrPadCfg);

    // FOR CSI

    /** @brief Set maximum mipi data rate (amount of all lans)
    @remarks MIPI interface only.
    @param[in] idx Sensor pad ID.
    @param[in] clk Max data rate.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_Clk)(u32 idx, CUS_CSI_CLK clk);

    /** @brief Set number of MIPI lanes
    @remarks MIPI interface only.
    @param[in] idx Sensor pad ID.
    @param[in] num_lan Number of lanes.
    @param[in] bon_off Clock ON/OFF control.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_Lane)(u32 idx, u16 num_lan, u8 bon_off);

    /** @brief Enable long packet type
    @remarks MIPI interface only
    @param[in] idx Sensor pad ID.
    @param[in] ctl_cfg0_15 Control flag bit[0:15]
    @param[in] ctl_cfg16_31 Control flag bit[16:31]
    @param[in] ctl_cfg32_47 Control flag bit[32:47]
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_LongPacketType)(u32 idx, u16 ctl_cfg0_15, u16 ctl_cfg16_31, u16 ctl_cfg32_47);

    /** @brief Virtual channel0 hsync mode
    @remarks MIPI interface only
    @param[in] idx Sensor pad ID.
    @param[in] mode HSYNC mode.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_VC0HSmode)(CUS_CSI_VC_HS_MODE mode);

    /** @brief Configure MIPI capture start timing
    @remarks MIPI interface only
    @param[in] idx Sensor pad ID.
    @param[in] rx_clk_skip_ns
    @param[in] rx_data_skip_ns
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_clk_data_skip)(u32 idx, u8 rx_clk_skip_ns, u8 rx_data_skip_ns);

    /** @brief Configure MIPI hdr mode
    @remarks MIPI interface only
    @param[in] idx Sensor pad ID.
    @param[in] hdr_mode HDR mode.
    @param[in] bon_off Clock ON/OFF control.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_hdr_mode)(u32 idx, CUS_HDR_MODE hdr_mode, u8 bon_off);

    /** @brief Change yuv output order
    @remarks yuv422 8bit output mode change
    @param[in] idx Sensor pad ID.
    @param[in] swap on/off
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_yuv_order_swap)(u32 idx, u8 swap);

    /** @brief MIPI Pattern Gen Enable For HAPS
    @remarks MIPI Pattern Gen Enable
    @param[in] idx Sensor pad ID.
    @param[in] width MIPI Pattern output image width
    @param[in] height MIPI Pattern output image height
    @param[in] bon_off MIPI Pattern Gen ON/OFF control.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetCSI_PatternGen)(u32 idx, u16 num_lane, u8 bon_off);

#if 0
    /** @brief Configure sensor RGBIR raw pixel order
    @param[in] handle Handle to sensor driver.
    @param[in] bayer_id Select pixel order
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*RGBIRFmt)(CUS_SEN_RGBIR RGBIR_id);
#endif

    /** @brief Skip vif output frame
    @remarks Skip Frame interface only
    @param[in] idx Sensor pad ID.
    @param[in] fps Sensor current fps.
    @param[in] cnt Skip counter
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetSkipFrame)(u32 idx, u32 fps, u32 cnt);

    /** @brief IRLed set trigger type
    @remarks set IRLed trigger para
    @param[in] idx IrLed ID
    @param[in] event config
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*IrLed_SetTriggerType)(u32 idx, CUS_IRLED_EVT_CFG* EventCfg);
    /** @brief IRLed set trigger delay
    @remarks set IRLed delay paras
    @param[in] idx IrLed ID
    @param[in] delay config
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*IrLed_SetIRLedDelay)(u32 idx, CUS_IRLED_DELAY_CFG* DelayConfig);
    /** @brief Frame mode ID
    @remarks Get IR period
    @param[in] idx Sensor pad ID.
    @param[out] frame_id Frame ID.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*GetFrameID)(u32 idx, u64* frame_id);

    /** @brief Set maximum mipi data rate (amount of all lans)
    @remarks MIPI interface only.
    @param[in] idx Sensor pad ID.
    @param[in] num_lan Number of lanes.
    @param[in] lvds Max data rate.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_Clk)(u32 idx, CUS_CSI_CLK clk);

    /** @brief Set number of LVDS lanes
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @param[in] bon_off Clock ON/OFF control.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_Lane)(u32 idx, u16 num_lan, u8 bon_off);

    /** @brief Set number of LVDS virtual channel
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @param[in] num_vc Number of virtual channel.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_NumVC)(u32 idx, u8 num_vc);

    /** @brief Customize LVDS sync code, This function can only support to customize 4th byte of sync code
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @param[in] type select a virtual channel to modify. range 0~2
    @param[in] sav_valid sav 4th code
    @param[in] eav_valid eav 4th code
    @param[in] sav_invalid sav 4th code
    @param[in] eav_invalid eav 4th code
    @param[in] mask sync code mask, set corresponding bit 1 to indicate the don't care bit in eav and sav
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_SyncCode)(u32 idx, u8 vc, u16 sav_valid, u16 eav_valid, u16 sav_invalid, u16 eav_invalid, u16 mask);

    /** @brief Set LVDS bit per pixel
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @param[in] bits pixel precision , select 8/10/12 bits
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_BitPerPixel)(u32 idx, u8 bpp);

    /** @brief Open LVDS port
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @param[in] num_lan Number of lanes.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_Init)(u32 idx, u16 num_lane);

    /** @brief Close LVDS port
    @remarks LVDS interface only.
    @param[in] idx Sensor pad ID.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*SetLVDS_Deinit)(u32 idx);

    /** @brief Configure sensor slave mode timing
    @remarks Sensor slave mode
    @param[in] idx Sensor pad ID.
    @param[in] uSrcClk Hsync clock source.
    @param[in] pXvs Vsync timing
    @param[in] pXhs Hsync timing
    @retval SUCCESS or FAIL if error occurs.
    */
    s32 (*SetSSM_Init)(u32 idx, u32 uSrcClk, CUS_SSM_TIMING_CFG* pXvs, CUS_SSM_TIMING_CFG* pXhs);

    /** @brief Release sensor slave mode
    @remarks Sensor slave mode
    @param[in] idx Sensor pad ID.
    @retval SUCCESS or FAIL if error occurs.
    */
    s32 (*SetSSM_Deinit)(u32 idx);

    /** @brief Release sensor slave mode
    @remarks Sensor slave mode
    @param[in] idx Sensor pad ID.
    @param[in] bEn Enable Vsync signal
    @retval SUCCESS or FAIL if error occurs.
    */
    s32 (*SetSSM_EnableXvs)(u32 idx, u8 bEn);

    /** @brief LVDS mode control
    @remarks Sensor LVDS interface control
    @param[in] idx Sensor pad ID.
    @param[in] eCtl LVDS API control ID
    @param[in] pParam LVDS API parameter
    @retval SUCCESS or FAIL if error occurs.
    */
    s32 (*LvdsCtrl)(u32 idx, CUS_LVDS_CTRL_ID_e eCtl, void* pParam);

    /** @brief Slave mode control
    @remarks Sensor SlaveMode interface control
    @param[in] idx Sensor pad ID.
    @param[in] eCtl Slave mode API control ID
    @param[in] pParam Slave mode API parameter
    @retval SUCCESS or FAIL if error occurs.
    */
    s32 (*SlaveModeCtrl)(u32 idx, CUS_SLAVE_MODE_CTRL_ID_e eCtl, void* pParam);

} ISensorIfAPI;

typedef union {
    // Parallel sensor
    struct {
        u32 parallel_reserved; // No additional attribute.
    } attr_parallel;

    // MIPI sensor
    struct {
        u32 mipi_lane_num;
        u32 mipi_data_format; // 0: YUV 422 format. 1: RGB pattern.
        u32 mipi_yuv_order; // YUYV or UYVY
        u32 mipi_hsync_mode;
        u32 mipi_sampling_delay; /** < MIPI start sampling delay */ /*bit 0~7: clk_skip_ns. bit 8~15: data_skip_ns*/
        CUS_HDR_MODE mipi_hdr_mode;
        u32 mipi_hdr_virtual_channel_num;
    } attr_mipi;

    // BT656 sensor
    struct {
        u32 bt656_total_ch;
        u32 bt656_cur_ch;
        u32 bt656_ch_det_en;
        CUS_VIF_BT656_CHANNEL_SELECT bt656_ch_det_sel;
        u32 bt656_bit_swap;
        u32 bt656_8bit_mode;
        CUS_VIF_BT656_VSYNC_DELAY bt656_vsync_delay;
        u32 bt656_hsync_inv;
        u32 bt656_vsync_inv;
        u32 bt656_clamp_en;
        CUS_BT656_ClkEdge_e bt656_clkedge;
    } attr_bt656;

    // LVDS sensor
    struct {
        u32 lvds_lane_num;
        u32 lvds_data_format; // 0: YUV 422 format. 1: RGB pattern.
        u32 lvds_yuv_order; // YUYV or UYVY
        u32 lvds_hsync_mode;
        u32 lvds_sampling_delay; /** < LVDS start sampling delay */ /*bit 0~7: clk_skip_ns. bit 8~15: data_skip_ns*/
        CUS_HDR_MODE lvds_hdr_mode;
        u32 lvds_hdr_virtual_channel_num; /** < LVDS virtual channel number*/
        union {
            CUS_LVDS_MODE0_SYNC_CODE_PARAM_t sync_code_mode0[4]; /** < LVDS 4th sync code for VC0~3*/
            CUS_LVDS_MODE1_SYNC_CODE_PARAM_t sync_code_mode1[4]; /** < LVDS 4th sync code for VC0~3*/
        };
    } attr_lvds;
} InterfaceAttr_u;

///////////////////////////////////////////////////////

/** @brief Sensor driver interface \n
The function here are implemented by sensor driver.
*/
typedef struct __ms_cus_sensor {
    // short     lib_idx;      /**< Reserved , Do not use */
    version_info version;
    char model_id[32]; /**< Please fill the sensor modle id string here then libcamera user can read model_id by using cameraGetSensorModelID() .*/
    void* sen_handle; /**< Reserved , Do not use */
    void* private_data; /**< sensor driver dependent variables should store in private_data and free when release */
    void* slave_mode_set; /**< Reserved , For sensor vsync/hsync control*/

    // i2c
    app_i2c_cfg i2c_cfg; /**< Sensor i2c setting */
    i2c_handle_t* i2c_bus; /**< Handle to sensor i2c API. */

    // sensor if api
    ISensorIfAPI* sensor_if_api; /**< sensor interface API */

    // ae parameters
    int ae_gain_delay; /**< How many frame delay from writing AE gain to take effect*/
    int ae_shutter_delay; /**< How many frame delay from writing AE shutter to take effect*/
    int ae_gain_ctrl_num;
    int ae_shutter_ctrl_num;

    // polarity
    CUS_CLK_POL reset_POLARITY; /** < Sensor REST pin polarity */
    CUS_CLK_POL pwdn_POLARITY; /** < Sensor Power Down pin polarity */
    CUS_CLK_POL VSYNC_POLARITY; /** < Sensor VSYNC polarity */
    CUS_CLK_POL HSYNC_POLARITY; /** < Sensor HSYNC pin polarity */
    CUS_CLK_POL PCLK_POLARITY; /** < Sensor Plxel Clock pin polarity */

    // sensor data enum list*/
    CUS_CAMSENSOR_ISPTYPE isp_type; /** < Always use ISP_EXT */
    CUS_SEN_BAYER bayer_id; /** < Sensor bayer raw pixel order */
    CUS_SEN_RGBIR RGBIR_id; /** < Sensor bayer raw pixel order */
    CUS_SENIF_BUS sif_bus; /** < Select sensor interface */
    CUS_DATAPRECISION data_prec; /** < Raw data bits */
    CUS_SEN_FMT_CONV_MODE data_mode; /** < Data convert mode*/
    CUS_CAMSENSOR_ORIT orient; /** < Sensor mirror/flip */
    CUS_SENSOR_PAD_GROUP snr_pad_group;
    CUS_SENSOR_MODE snr_pad_mode; /** < support master/slave mode sensor */

    cus_camsensor_res_list video_res_supported; /** < Resolution list */
    InterfaceAttr_u interface_attr;
    CUS_SLAVE_MODE_ATTR_t slave_mode_attr; /** < Sensor slave mode Hsync Vsync timing configuration*/

    // sensor calibration
    u32 mclk; /** < Sensor master clock frequency */
    u32 sat_mingain; /**< Minimum AE gain */
    u32 ir_only_fmt;
    u32 nPixelSize; // in nano meter
    ////////////////////////////////////////////////
    // system functions
    ////////////////////////////////////////////////

    /** @brief Sensor power on sequence, I2C must be ready after calling this function
    @param[in] handle Handle to sensor driver.
    @remark Following configuration need to set up at this stage \n
    @ref __ISensorIfAPI::Reset Reset sensor \n
    @ref __ISensorIfAPI::PowerOff Sensor power down pin \n
    @ref __ISensorIfAPI::MCLK Sensor master clock \n
    @ref __ISensorIfAPI::SetIOPad ISP sensor IO \n
    @ref __ISensorIfAPI::SetCSI_Clk  [MIPI sensor only] MIPI clock\n
    @ref __ISensorIfAPI::HsyncPol Hsync polarity\n
    @ref __ISensorIfAPI::VsyncPol Vsync polarity\n
    @ref __ISensorIfAPI::ClkPol [Parallel sensor only]  Pixel clock polarity\n
    @ref __ISensorIfAPI::BayerFmt Raw data format\n
    @ref __ISensorIfAPI::DataPrecision Raw data pixel bits\n
    @ref __ISensorIfAPI::FmtConv Raw data to ISP pixel convert\n
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*pCus_sensor_poweron)(struct __ms_cus_sensor* handle, u32 idx);

    /** @brief Sensor power off
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    */
    int (*pCus_sensor_poweroff)(struct __ms_cus_sensor* handle, u32 idx);

    /** @brief Sensor initialization
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    @remark Fill sensor initial table here, Sensor beginning to output raw images after calling this function .
    */
    int (*pCus_sensor_init)(struct __ms_cus_sensor* handle);

    /** @brief Sensor Post initialization
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    @remark Fill sensor initial table here, Sensor beginning to output raw images after calling this function .
    */
    int (*pCus_sensor_post_init)(struct __ms_cus_sensor* handle, u32 idx);

    /** @brief Release resources those allocated in cus_camsensor_init_handle()
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    @remark Release resource allocated in \ref cus_camsensor_init_handle
    */
    int (*pCus_sensor_release)(struct __ms_cus_sensor* handle);

    /** @brief Enter sensor suspend mode
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    @remark ISP call this function before enter power saving mode
    */
    int (*pCus_sensor_suspend)(struct __ms_cus_sensor* handle);

    /** @brief Sensor wakeup
    @param[in] handle Handle to sensor driver.
    @retval SUCCESS or FAIL if error occurs.
    @remark ISP call this function after exit power saving mode
    */
    int (*pCus_sensor_resume)(struct __ms_cus_sensor* handle);

    /** @brief Enable sensor pattern mode if sensor hardward supported
    @param[in] handle Handle to sensor driver.
    @param[in] mode Pattern select, if 0 disable pattern mode.
    @retval SUCCESS or FAIL if error occurs.
    @remark This function is optional
    */
    int (*pCus_sensor_SetPatternMode)(struct __ms_cus_sensor* handle, u32 mode);

    // Normal

    /** @brief Check sensor ID and report to ISP sensor match or not
    @param[in] handle Handle to sensor driver.
    @param[out] id Receive 4 bytes customer defined sensor ID.
    @retval Return SUCCESS if sensor matched or Retuen FAIL if sensor mismatch.
    @remark Read sensor ID through I2C
    */
    int (*pCus_sensor_GetSensorID)(struct __ms_cus_sensor* handle, u32* id);

    /** @brief Get resolution list
   @param[in] handle Handle to sensor driver.
   @param[out] id Receive supported resolution list
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetVideoRes)(struct __ms_cus_sensor* handle, u32 res_idx, cus_camsensor_res** res);

    /** @brief Get resolution list
   @param[in] handle Handle to sensor driver.
   @param[out] id Receive supported resolution list
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetCurVideoRes)(struct __ms_cus_sensor* handle, u32* cur_idx, cus_camsensor_res** res);

    /** @brief Select a sensor output resolution sensor list
   @param[in] handle Handle to sensor driver.
   @param[in] res_id Resolution id
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_SetVideoRes)(struct __ms_cus_sensor* handle, u32 res_id);

    /** @brief Get sensor current mirror flip setting
   @param[in] handle Handle to sensor driver.
   @param[out] ori Receive Mirror/Flip setting.
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetOrien)(struct __ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT* ori);

    /** @brief Select a sensor mirror flip
   @param[in] handle Handle to sensor driver.
   @param[in] ori Mirror/Flip configuration.
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_SetOrien)(struct __ms_cus_sensor* handle, CUS_CAMSENSOR_ORIT ori);

    /** @brief Get sensor capability
   @param[in] handle Handle to sensor driver.
   @param[out] cap Receive sensor capability
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetSensorCap)(struct __ms_cus_sensor* handle, CUS_CAMSENSOR_CAP* cap);

    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds

    /** @brief AE/Frame status change notification
   @param[in] handle Handle to sensor driver.
   @param[in] status Current status
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_AEStatusNotify)(struct __ms_cus_sensor* handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);

    /** @brief Get sensor shutter setting in us
   @param[in] handle Handle to sensor driver.
   @param[out] us Receive current shutter setting
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetAEUSecs)(struct __ms_cus_sensor* handle, u32* us);

    /** @brief Set sensor shutter in us
   @param[in] handle Handle to sensor driver.
   @param[in] us Shutter setting in us
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_SetAEUSecs)(struct __ms_cus_sensor* handle, u32 us);

    // Gain: 1x = 1024
    /** @brief Get sensor current AE gain
    @param[in] handle Handle to sensor driver.
    @param[out] gain Receive current AE gain
    @retval Return SUCCESS or FAIL if error occurs.
    @remark gain: 1x = 1024
    */
    int (*pCus_sensor_GetAEGain)(struct __ms_cus_sensor* handle, u32* gain);

    /** @brief Set sensor AE gain
    @param[in] handle Handle to sensor driver.
    @param[in] gain AE gain
    @retval Return SUCCESS or FAIL if error occurs.
    @remark gain: 1x = 1024
    */
    int (*pCus_sensor_SetAEGain)(struct __ms_cus_sensor* handle, u32 gain);

    /** @brief Get supported shutter range
    @param[in] handle Handle to sensor driver.
    @param[out] min Receive minimum shutter which sensor can supported
    @param[out] min Receive maxiimum shutter which sensor can supported
    @retval Return SUCCESS or FAIL if error occurs.
    @remark gain: 1x = 1024
    */
    int (*pCus_sensor_GetAEMinMaxUSecs)(struct __ms_cus_sensor* handle, u32* min, u32* max);

    /** @brief Get supported AE gain range
    @param[in] handle Handle to sensor driver.
    @param[out] min Receive minimum gain which sensor can supported
    @param[out] min Receive maxiimum gain which sensor can supported
    @retval Return SUCCESS or FAIL if error occurs.
    @remark gain: 1x = 1024
    */
    int (*pCus_sensor_GetAEMinMaxGain)(struct __ms_cus_sensor* handle, u32* min, u32* max);

    // frame rate control
    /** @brief Get current fps
    @param[in] handle Handle to sensor driver.
    @retval Return current frame rate per second
    */
    int (*pCus_sensor_GetFPS)(struct __ms_cus_sensor* handle);

    /** @brief Set sensor output fps
    @param[in] handle Handle to sensor driver.
    @param[in] fps
    @retval Return SUCCESS or FAIL if fps is out of range.
    */
    int (*pCus_sensor_SetFPS)(struct __ms_cus_sensor* handle, u32 fps);

    //[OPTIONAL] sensor calibration
    /** @brief Optional function */
    int (*pCus_sensor_SetAEGain_cal)(struct __ms_cus_sensor* handle, u32);

    /** @brief Optional function */
    int (*pCus_sensor_setCaliData_gain_linearity)(struct __ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num);

    // Get shutter information
    /** @brief Get shutter information
    @param[in] handle Handle to sensor driver.
    @param[out] info return shutter information.
    @retval Return current frame rate per second
    */
    int (*pCus_sensor_GetShutterInfo)(struct __ms_cus_sensor* handle, CUS_SHUTTER_INFO* info);

    /** @brief Get resolution list number
   @param[in] handle Handle to sensor driver.
   @param[out] ulres_num resolution list number
   @retval Return SUCCESS or FAIL if error occurs.
   */
    int (*pCus_sensor_GetVideoResNum)(struct __ms_cus_sensor* handle, u32* ulres_num);

    // Get shutter information
    /** @brief Sensor vendor command
    @param[in] handle Handle to sensor driver.
    @param[in] reserved
    @param[in] param Command input
    @param[out] out Command output
    @retval Return SUCCESS or FAIL if error occurs.
    */
    int (*pCus_sensor_CustDefineFunction)(struct __ms_cus_sensor* handle, u32 cmd_id, void* param);

    // Get Source Type
    /** @brief Get Source Type
    @param[in] handle Handle to sensor driver.
    @param[in] plane id.
    @param[out] psrc_type info
    @retval Return SUCCESS or FAIL if error occurs.
    */
    int (*pCus_sensor_GetSrcType)(struct __ms_cus_sensor* handle, u32 plane_id, CUS_SNR_ANADEC_SRC_TYPE* psrc_type);
    // old api for I5 MHAL_Sensorif build pass
    int (*pCus_sensor_GetSrcAttr)(struct __ms_cus_sensor* handle, u32 plane_id, CUS_SNR_Anadec_SrcAttr_t* psrc_type);

    int (*pCus_sensor_SetSrcAttr)(struct __ms_cus_sensor* handle, u32 plane_id, CUS_SNR_Anadec_SrcAttr_t* psrc_type);

    // Gain: 1x = 1024
    /** @brief Try AE gain value
    @param[in] handle Handle to sensor driver.
    @param[in] Target AE gain value to try
    @retval Return actual sensor gain will apply on the HW.
    @remark gain: 1x = 1024
    */
    u32 (*pCus_sensor_TryAEGain)(struct __ms_cus_sensor* handle, u32 gain);

    // Return actual shutter
    /** @brief Try AE shutter value
    @param[in] handle Handle to sensor driver.
    @param[in] Target AE shutter value to try , in us
    @retval Return actual sensor gain will apply on the HW.
    @remark gain: 1x = 1024
    */
    u32 (*pCus_sensor_TryAEShutter)(struct __ms_cus_sensor* handle, u32 us);

    int (*pCus_sensor_SwitchgSensorId)(struct __ms_cus_sensor* handle, u32 sensorId);

    int (*pCus_sensor_GetCurSwtichSensorId)(struct __ms_cus_sensor* handle, u32* pu32sensorId);
} ms_cus_sensor;

/** @brief Sensor driver entry. ISP call this function before start using sensor driver. \n
ISP pass \ref ms_cus_sensor struct to obtain the driver information and function entries. \n
And all allocated resources here should be released at \ref __ms_cus_sensor::pCus_sensor_release.
Every sensor driver must implement this api.
@param[in] drv_handle A uninitialized \ref ms_cus_sensor struct from ISP, Sensor driver fill the driver information and function entries to drv_handle.
@retval SUCCESS or FAIL if error occurs.
*/

typedef int (*SensorInitHandle)(ms_cus_sensor* handle);

#ifdef __cplusplus
}
#endif

#endif /* DRV_MS_CUS_SENSOR_H_ */
/** @} */ // end of ISP Sensor Driver Interface
