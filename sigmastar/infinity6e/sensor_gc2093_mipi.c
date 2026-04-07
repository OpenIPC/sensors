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
extern "C"
{
#endif

#include <drv_sensor_common.h>
#include <sensor_i2c_api.h>
#include <drv_sensor.h>

#ifdef __cplusplus
}
#endif

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(gc2093_MIPI);

#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
//MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (2)
//#define SENSOR_MIPI_HDR_MODE (1) //0: Non-HDR mode. 1:Sony DOL mode
//MIPI config end.
//============================================

#define R_GAIN_REG 1
#define G_GAIN_REG 2
#define B_GAIN_REG 3


//#undef SENSOR_DBG
//#define SENSOR_DBG 0

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

#define SENSOR_ISP_TYPE     ISP_EXT                   //ISP_EXT, ISP_SOC
#define F_number  22                                  // CFG, demo module
//#define SENSOR_DATAFMT      CUS_DATAFMT_BAYER        //CUS_DATAFMT_YUV, CUS_DATAFMT_BAYER
#define SENSOR_IFBUS_TYPE   CUS_SENIF_BUS_MIPI      //CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_MIPI_HSYNC_MODE_HDR_DOL PACKET_FOOTER_EDGE
#define SENSOR_DATAPREC     CUS_DATAPRECISION_10    //CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE     CUS_SEN_10TO12_9000     //CFG
//#define SENSOR_MAXGAIN      (15875*315)/10000   /////sensor again 15.875 dgain=31.5
#define SENSOR_MAXGAIN      (77660)/100
#define SENSOR_BAYERID      CUS_BAYER_BG            //CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID      CUS_RGBIR_NONE
#define SENSOR_ORIT         CUS_ORIT_M0F0           //CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAX_GAIN     80                 // max sensor again, a-gain
//#define SENSOR_YCORDER      CUS_SEN_YCODR_YC       //CUS_SEN_YCODR_YC, CUS_SEN_YCODR_CY
#define lane_number 2
#define vc0_hs_mode 3   //0: packet header edge  1: line end edge 2: line start edge 3: packet footer edge
#define long_packet_type_enable 0x00 //UD1~UD8 (user define)

#define Preview_MCLK_SPEED  CUS_CMU_CLK_27MHZ        //CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
#define Preview_MCLK_SPEED_HDR_DOL  CUS_CMU_CLK_27MHZ

u32 Preview_line_period;
u32 vts_30fps;
#define Preview_WIDTH       1920                    //resolution Width when preview
#define Preview_HEIGHT      1080                    //resolution Height when preview
#define Preview_MAX_FPS     120                     //fastest preview FPS
#define Preview_MIN_FPS     3                      //slowest preview FPS
#define Preview_CROP_START_X     0                      //CROP_START_X
#define Preview_CROP_START_Y     0                      //CROP_START_Y

#define SENSOR_I2C_ADDR    0x6e                   //I2C slave address
#define SENSOR_I2C_SPEED   200000 //300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY  I2C_NORMAL_MODE     //usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT     I2C_FMT_A16D8        //CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL     CUS_CLK_POL_POS        // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL      CUS_CLK_POL_NEG        // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL    CUS_CLK_POL_NEG        // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL    CUS_CLK_POL_NEG        // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL     CUS_CLK_POL_POS        // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG
//static int  drv_Fnumber = 22;

static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps);
static int pCus_SetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT orit);
static int g_sensor_ae_min_gain = 1024;

CUS_MCLK_FREQ UseParaMclk(void);

CUS_CAMSENSOR_CAP sensor_cap = {
    .length = sizeof(CUS_CAMSENSOR_CAP),
    .version = 0x0001,
};

static struct {
    // Modify it based on number of support resolution
    enum { 
		LINEAR_RES_1 = 0,
//      LINEAR_RES_2,
		LINEAR_RES_3,
		LINEAR_RES_4,
//		LINEAR_RES_5,
		LINEAR_RES_6,
		LINEAR_RES_7,
		LINEAR_RES_8,
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
} gc2093_mipi_linear[] = {
	{ LINEAR_RES_1, { 1920, 1080, 3, 60  }, { 0, 0, 1920, 1080 }, { "1920x1080@60fps" } },
//	{ LINEAR_RES_2, { 1920, 1080, 3, 70  }, { 0, 0, 1920, 1080 }, { "1920x1080@70fps" } },
	{ LINEAR_RES_3, { 1920, 1080, 3, 80  }, { 0, 0, 1920, 1080 }, { "1920x1080@80fps" } },
    { LINEAR_RES_4, { 1920, 1080, 3, 90  }, { 0, 0, 1920, 1080 }, { "1920x1080@90fps" } },
//	{ LINEAR_RES_5, { 1920, 960,  3, 90  }, { 0, 0, 1920,  960 }, { "1920x960@90fps"  } },
	{ LINEAR_RES_6, { 1920, 864,  3, 100 }, { 0, 0, 1920,  864 }, { "1920x864@100fps" } },
    { LINEAR_RES_7, { 1920, 768,  3, 120 }, { 0, 0, 1920,  768 }, { "1920x768@120fps" } },
	{ LINEAR_RES_8, { 1280, 720,  3, 120 }, { 0, 0, 1280,  720 }, { "1280x720@120fps" } },
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
        u32 half_lines;
        u32 half_line;
        u32 fps;
        u32 preview_fps;
        u32 max_short_exp;
        u32 line;
    } expo;
    struct {
        bool bVideoMode;
        u16 res_idx;
        //        bool binning;
        //        bool scaling;
        CUS_CAMSENSOR_ORIT  orit;
    } res;
    /* I2C_ARRAY tVts_reg[2];
    I2C_ARRAY tGain_reg[6];
    I2C_ARRAY tGain_reg_HDR_DOL_SEF[5];
    I2C_ARRAY tExpo_reg[2];
    I2C_ARRAY tExpo_reg_HDR_DOL_SEF[2];
//    I2C_ARRAY tMax_short_exp_reg[2];
    I2C_ARRAY tMirror_table[1]; */
    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool reg_mf;
    bool reg_dirty;
    bool ori_dirty;
    CUS_CAMSENSOR_ORIT cur_orien;
} gc2093_params;

typedef struct {
    u64 gain;
    u8 fine_gain_reg;
} FINE_GAIN;

typedef struct {

      u32 gain;
      unsigned short again_reg_val_0;
      unsigned short again_reg_val_1;
      unsigned short again_reg_val_2;
      unsigned short again_reg_val_3;

} Gain_ARRAY;

const I2C_ARRAY Sensor_id_table[] =
{
    {0x03f0, 0x20},
    {0x03f1, 0x93},
};

// LINEAR_RES_1, 1920x1080, 60fps, vts=1127, pclk=88.875MHz
I2C_ARRAY Sensor_init_table_1920x1080_60fps[] =
{	
    /****system****/
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0x00},
    {0x03f2, 0x00},
    {0x03f3, 0x00},
    {0x03f4, 0x36},
    {0x03f5, 0xc0},
    {0x03f6, 0x0B},
    {0x03f7, 0x01},
    {0x03f8, 0x4f},
    {0x03f9, 0x40},
    {0x03fc, 0x8e},
	
    /****CISCTL & ANALOG****/
    {0x0087, 0x18},
    {0x00ee, 0x30},
    {0x00d0, 0xbf},
    {0x01a0, 0x00},
    {0x01a4, 0x40},
    {0x01a5, 0x40},
    {0x01a6, 0x40},
    {0x01af, 0x09},
    {0x0001, 0x00},
    {0x0002, 0x02},
    {0x0003, 0x00},
    {0x0004, 0x64},
    {0x0005, 0x02},
    {0x0006, 0x91},
    {0x0007, 0x00},
    {0x0008, 0x6e},
    {0x0009, 0x00},
    {0x000a, 0x02},
    {0x000b, 0x00},
    {0x000c, 0x04},
    {0x000d, 0x04},
    {0x000e, 0x40},
    {0x000f, 0x07},
    {0x0010, 0x8c},
    {0x0013, 0x15},
    {0x0019, 0x0c},
    {0x0041, 0x04},
    {0x0042, 0x67},
    {0x0053, 0x60},
    {0x008d, 0x92},
    {0x0090, 0x00},
    {0x00c7, 0xe1},
    {0x001b, 0x73},
    {0x0028, 0x0d},
    {0x0029, 0x24},
    {0x002b, 0x04},
    {0x002e, 0x23},
    {0x0037, 0x03},
    {0x0043, 0x04},
    {0x0044, 0x28},
    {0x004a, 0x01},
    {0x004b, 0x20},
    {0x0055, 0x28},
    {0x0066, 0x3f},
    {0x0068, 0x3f},
    {0x006b, 0x44},
    {0x0077, 0x00},
    {0x0078, 0x20},
    {0x007c, 0xa1},
    {0x00ce, 0x7c},
    {0x00d3, 0xd4},
    {0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02}, 
	{0x0194, 0x03}, 
	{0x0195, 0x04},
	{0x0196, 0x38}, 
	{0x0197, 0x07},
	{0x0198, 0x80}, 

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_2, 1920x1080, 70fps, vts=1125, pclk=103.500MHz
I2C_ARRAY Sensor_init_table_1920x1080_70fps[] =
{	
    /****system****/
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0x00},
    {0x03f2, 0x00},
    {0x03f3, 0x00},
    {0x03f4, 0x36},
    {0x03f5, 0xc0},
    {0x03f6, 0x0B},
    {0x03f7, 0x01},
    {0x03f8, 0x5c},
    {0x03f9, 0x40},
    {0x03fc, 0x8e},
	
    /****CISCTL & ANALOG****/
    {0x0087, 0x18},
    {0x00ee, 0x30},
    {0x00d0, 0xbf},
    {0x01a0, 0x00},
    {0x01a4, 0x40},
    {0x01a5, 0x40},
    {0x01a6, 0x40},
    {0x01af, 0x09},
    {0x0001, 0x00},
    {0x0002, 0x02},
    {0x0003, 0x00},
    {0x0004, 0x64},
    {0x0005, 0x02},
    {0x0006, 0x91},
    {0x0007, 0x00},
    {0x0008, 0x6e},
    {0x0009, 0x00},
    {0x000a, 0x02},
    {0x000b, 0x00},
    {0x000c, 0x04},
    {0x000d, 0x04},
    {0x000e, 0x40},
    {0x000f, 0x07},
    {0x0010, 0x8c},
    {0x0013, 0x15},
    {0x0019, 0x0c},
    {0x0041, 0x04},
    {0x0042, 0x65},
    {0x0053, 0x60},
    {0x008d, 0x92},
    {0x0090, 0x00},
    {0x00c7, 0xe1},
    {0x001b, 0x73},
    {0x0028, 0x0d},
    {0x0029, 0x24},
    {0x002b, 0x04},
    {0x002e, 0x23},
    {0x0037, 0x03},
    {0x0043, 0x04},
    {0x0044, 0x28},
    {0x004a, 0x01},
    {0x004b, 0x20},
    {0x0055, 0x28},
    {0x0066, 0x3f},
    {0x0068, 0x3f},
    {0x006b, 0x44},
    {0x0077, 0x00},
    {0x0078, 0x20},
    {0x007c, 0xa1},
    {0x00ce, 0x7c},
    {0x00d3, 0xd4},
    {0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02}, 
	{0x0194, 0x03}, 
	{0x0195, 0x04},
	{0x0196, 0x38}, 
	{0x0197, 0x07},
	{0x0198, 0x80}, 

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_3, 1920x1080, 80fps, vts=1125, pclk=118.125MHz
I2C_ARRAY Sensor_init_table_1920x1080_80fps[] =
{	
    /****system****/
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0x00},
    {0x03f2, 0x00},
    {0x03f3, 0x00},
    {0x03f4, 0x36},
    {0x03f5, 0xc0},
    {0x03f6, 0x0B},
    {0x03f7, 0x01},
    {0x03f8, 0x69},
    {0x03f9, 0x40},
    {0x03fc, 0x8e},
	
    /****CISCTL & ANALOG****/
    {0x0087, 0x18},
    {0x00ee, 0x30},
    {0x00d0, 0xbf},
    {0x01a0, 0x00},
    {0x01a4, 0x40},
    {0x01a5, 0x40},
    {0x01a6, 0x40},
    {0x01af, 0x09},
    {0x0001, 0x00},
    {0x0002, 0x02},
    {0x0003, 0x00},
    {0x0004, 0x64},
    {0x0005, 0x02},
    {0x0006, 0x91},
    {0x0007, 0x00},
    {0x0008, 0x6e},
    {0x0009, 0x00},
    {0x000a, 0x02},
    {0x000b, 0x00},
    {0x000c, 0x04},
    {0x000d, 0x04},
    {0x000e, 0x40},
    {0x000f, 0x07},
    {0x0010, 0x8c},
    {0x0013, 0x15},
    {0x0019, 0x0c},
    {0x0041, 0x04},
    {0x0042, 0x65},
    {0x0053, 0x60},
    {0x008d, 0x92},
    {0x0090, 0x00},
    {0x00c7, 0xe1},
    {0x001b, 0x73},
    {0x0028, 0x0d},
    {0x0029, 0x24},
    {0x002b, 0x04},
    {0x002e, 0x23},
    {0x0037, 0x03},
    {0x0043, 0x04},
    {0x0044, 0x28},
    {0x004a, 0x01},
    {0x004b, 0x20},
    {0x0055, 0x28},
    {0x0066, 0x3f},
    {0x0068, 0x3f},
    {0x006b, 0x44},
    {0x0077, 0x00},
    {0x0078, 0x20},
    {0x007c, 0xa1},
    {0x00ce, 0x7c},
    {0x00d3, 0xd4},
    {0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02}, 
	{0x0194, 0x03}, 
	{0x0195, 0x04},
	{0x0196, 0x38}, 
	{0x0197, 0x07},
	{0x0198, 0x80}, 

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_4, 1920x1080, 90fps, vts=1122, pclk=132.750MHz
I2C_ARRAY Sensor_init_table_1920x1080_90fps[] =
{	
    /****system****/
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0xf0},
    {0x03fe, 0x00},
    {0x03f2, 0x00},
    {0x03f3, 0x00},
    {0x03f4, 0x36},
    {0x03f5, 0xc0},
    {0x03f6, 0x0B},
    {0x03f7, 0x01},
    {0x03f8, 0x76},
    {0x03f9, 0x40},
    {0x03fc, 0x8e},
	
    /****CISCTL & ANALOG****/
    {0x0087, 0x18},
    {0x00ee, 0x30},
    {0x00d0, 0xbf},
    {0x01a0, 0x00},
    {0x01a4, 0x40},
    {0x01a5, 0x40},
    {0x01a6, 0x40},
    {0x01af, 0x09},
    {0x0001, 0x00},
    {0x0002, 0x02},
    {0x0003, 0x00},
    {0x0004, 0x64},
    {0x0005, 0x02},
    {0x0006, 0x91},
    {0x0007, 0x00},
    {0x0008, 0x6e},
    {0x0009, 0x00},
    {0x000a, 0x02},
    {0x000b, 0x00},
    {0x000c, 0x04},
    {0x000d, 0x04},
    {0x000e, 0x40},
    {0x000f, 0x07},
    {0x0010, 0x8c},
    {0x0013, 0x15},
    {0x0019, 0x0c},
    {0x0041, 0x04},
    {0x0042, 0x62},
    {0x0053, 0x60},
    {0x008d, 0x92},
    {0x0090, 0x00},
    {0x00c7, 0xe1},
    {0x001b, 0x73},
    {0x0028, 0x0d},
    {0x0029, 0x24},
    {0x002b, 0x04},
    {0x002e, 0x23},
    {0x0037, 0x03},
    {0x0043, 0x04},
    {0x0044, 0x28},
    {0x004a, 0x01},
    {0x004b, 0x20},
    {0x0055, 0x28},
    {0x0066, 0x3f},
    {0x0068, 0x3f},
    {0x006b, 0x44},
    {0x0077, 0x00},
    {0x0078, 0x20},
    {0x007c, 0xa1},
    {0x00ce, 0x7c},
    {0x00d3, 0xd4},
    {0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02}, 
	{0x0194, 0x03}, 
	{0x0195, 0x04},
	{0x0196, 0x38}, 
	{0x0197, 0x07},
	{0x0198, 0x80}, 

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_5, 1920x960(18:9), 90fps, vts=998, pclk=118.125MHz
I2C_ARRAY Sensor_init_table_1920x960_90fps[] =
{
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x69},
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x00},
	{0x0004, 0x64},
	{0x0005, 0x02},
	{0x0006, 0x91},
	{0x0007, 0x00},
	{0x0008, 0x5d},
	{0x0009, 0x00},
	{0x000a, 0x3e},//158//62
	{0x000b, 0x00},
	{0x000c, 0x04},//4
	{0x000d, 0x03},
	{0x000e, 0xc4},//772=768+4//964
	{0x000f, 0x07},
	{0x0010, 0x8c},//1932
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x03},
	{0x0042, 0xe6},//
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x03},
	{0x0196, 0xc0},//768//960
	{0x0197, 0x07},
	{0x0198, 0x80},//1920

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_6, 1920x864(20:9), 100fps, vts=899, pclk=118.125MHz
I2C_ARRAY Sensor_init_table_1920x864_100fps[] =
{
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x69},
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x00},
	{0x0004, 0x64},
	{0x0005, 0x02},
	{0x0006, 0x91},
	{0x0007, 0x00},
	{0x0008, 0x5d},
	{0x0009, 0x00},
	{0x000a, 0x6e},
	{0x000b, 0x00},
	{0x000c, 0x04},
	{0x000d, 0x03},
	{0x000e, 0x64},
	{0x000f, 0x07},
	{0x0010, 0x8c},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x03},
	{0x0042, 0xe6},
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x03},
	{0x0196, 0x60},//768//960//864
	{0x0197, 0x07},
	{0x0198, 0x80},//1920

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_7, 1920x768(2.5:1), 120fps, vts=799, pclk=126.000MHz
I2C_ARRAY Sensor_init_table_1920x768_120fps[] =
{
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x70},
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x00},
	{0x0004, 0x64},
	{0x0005, 0x02},
	{0x0006, 0x91},
	{0x0007, 0x00},
	{0x0008, 0x5d},
	{0x0009, 0x00},
	{0x000a, 0x9e},
	{0x000b, 0x00},
	{0x000c, 0x04},
	{0x000d, 0x03},
	{0x000e, 0x04},
	{0x000f, 0x07},
	{0x0010, 0x8c},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x03},
	{0x0042, 0x1f},
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x03},
	{0x0196, 0x00},
	{0x0197, 0x07},
	{0x0198, 0x80},

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};

// LINEAR_RES_8, 1280x720, 120fps, vts=750, pclk=119.250MHz
I2C_ARRAY Sensor_init_table_1280x720_120fps[] =
{	
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x6a}, 
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x00},
	{0x0004, 0x64},
	{0x0005, 0x02},
	{0x0006, 0x91},
	{0x0007, 0x00},
	{0x0008, 0x5d},
	{0x0009, 0x00},
	{0x000a, 0xb6},
	{0x000b, 0x02},
	{0x000c, 0x88},
	{0x000d, 0x02},
	{0x000e, 0xd4},
	{0x000f, 0x05},
	{0x0010, 0x08},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x02},	
	{0x0042, 0xee},
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	
	 /*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},

	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},

	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	{0x04e0, 0x18},

	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x00},
	{0x0122, 0x10},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},

	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x02},
	{0x0196, 0xd0},
	{0x0197, 0x05},
	{0x0198, 0x00},

    //DVP & MIPI
    {0x0199, 0x00},
    {0x019a, 0x06},
    {0x007b, 0x2a},
    {0x0023, 0x2d},
    {0x0201, 0x27},
    {0x0202, 0x56},
    {0x0203, 0xb6},
    {0x0212, 0x80},
    {0x0213, 0x07},
    {0x0215, 0x10},
    {0x003e, 0x91},
};


static  I2C_ARRAY mirror_reg[] =
{
    {0x0017, 0x82}, 
}; 

typedef struct 
{
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

static I2C_ARRAY gain_reg[] = {
    //{0x00b4, 0x00},     
	{0x00b3, 0x00},
	{0x00b8, 0x01},
	{0x00b9, 0x00},
	{0x0078, 0x20},
	{0x00b1, 0x01},
	{0x00b2, 0x00},
};

static I2C_ARRAY expo_reg[] = {  // max expo line vts*2-6!
    {0x0003, 0x00}, // expo[15:8]
    {0x0004, 0x02}, // expo[7:0] 
};

static I2C_ARRAY vts_reg[] = {
    {0x0041, 0x04},
    {0x0042, 0x65}, 
};

static Gain_ARRAY gain_table[]= {
	{1024, 0x00,0x01,0x00,0x20},
	{1184, 0x10,0x01,0x0c,0x20},
	{1424, 0x20,0x01,0x1b,0x20},
	{1664, 0x30,0x01,0x2c,0x20},
	{2016, 0x40,0x01,0x3f,0x2c},
	{2336, 0x50,0x02,0x16,0x2c},
	{2864, 0x60,0x02,0x35,0x2c},
	{3344, 0x70,0x03,0x16,0x2c},
	{4064, 0x80,0x04,0x02,0x28},
	{4720, 0x90,0x04,0x31,0x28},
	{5424, 0xa0,0x05,0x32,0x28},
	{6304, 0xb0,0x06,0x35,0x28},
	{7664, 0xc0,0x08,0x04,0x24},
	{9056, 0x5a,0x09,0x19,0x24},
	{10912, 0x83,0x0b,0x0f,0x24},
	{12688, 0x93,0x0d,0x12,0x24},
	{15408, 0x84,0x10,0x00,0x20},
	{17904, 0x94,0x12,0x3a,0x20},
	{23440, 0x5d,0x1a,0x02,0x20},
	{26016, 0x9b,0x1b,0x20,0x20},
	{30832, 0x8c,0x20,0x0f,0x14},
	{35824, 0x9c,0x26,0x07,0x14},
	{45296, 0xb6,0x36,0x21,0x14},
	{54544, 0xad,0x37,0x3a,0x14},
	{63376, 0xbd,0x3d,0x02,0x08},
	//{63904, 0xbd,0x3f,0x3f,0x08},
	//{77248, 0x85,0x3f,0x3f,0x08},
	//{74848, 0x95,0x3f,0x3f,0x04},
	//{110000, 0xce,0x3f,0x3f,0x04},
}; 

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
#define SENSOR_NAME gc2093
*/
#define SensorReg_Read(_reg,_data)     (handle->i2c_bus->i2c_rx(handle->i2c_bus, &(handle->i2c_cfg),_reg,_data))
#define SensorReg_Write(_reg,_data)    (handle->i2c_bus->i2c_tx(handle->i2c_bus, &(handle->i2c_cfg),_reg,_data))
#define SensorRegArrayW(_reg,_len)  (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, &(handle->i2c_cfg),(_reg),(_len)))
#define SensorRegArrayR(_reg,_len)  (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, &(handle->i2c_cfg),(_reg),(_len)))

int cus_camsensor_release_handle(ms_cus_sensor *handle);

/////////////////// sensor hardware dependent //////////////
static int pCus_poweron(ms_cus_sensor *handle, u32 idx)
{
    ISensorIfAPI *sensor_if = handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

//	SENSOR_USLEEP(1000);
//	Sensor power on sequence
// 	sensor_if->MCLK(idx, 1, handle->mclk);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 2);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);

	/*
    if (handle->interface_attr.attr_mipi.mipi_hdr_mode == CUS_HDR_MODE_DCG) {
        sensor_if->SetCSI_hdr_mode(idx, handle->interface_attr.attr_mipi.mipi_hdr_mode, 2);
    } */

    //ISP_config_io(handle);
    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    SENSOR_USLEEP(1000);
	
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    SENSOR_USLEEP(1000);

    //sensor_if->Set3ATaskOrder(handle, def_order);
    // pure power on
    //ISP_config_io(handle);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
	SENSOR_USLEEP(1000);
	SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
	SENSOR_USLEEP(1000);
	
	sensor_if->MCLK(idx, 1, handle->mclk);
    SENSOR_USLEEP(5000);
    //handle->i2c_bus->i2c_open(handle->i2c_bus,&handle->i2c_cfg);

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor *handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI *sensor_if = handle->sensor_if_api;
//    gc2093_params *params = (gc2093_params *)handle->private_data;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
	sensor_if->Reset(idx, handle->reset_POLARITY);
    //handle->i2c_bus->i2c_close(handle->i2c_bus);
    SENSOR_USLEEP(1000);
    //Set_csi_if(0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);

    return SUCCESS;
}

/////////////////// image function /////////////////////////
//Get and check sensor ID
//if i2c error or sensor id does not match then return FAIL
static int pCus_GetSensorID(ms_cus_sensor *handle, u32 *id)
{
    int i,n;
    int table_length= ARRAY_SIZE(Sensor_id_table);
    I2C_ARRAY id_from_sensor[ARRAY_SIZE(Sensor_id_table)];

  SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    for(n=0;n<table_length;++n)
    {
      id_from_sensor[n].reg = Sensor_id_table[n].reg;
      id_from_sensor[n].data = 0;
    }

    *id =0;
    if(table_length>8) table_length=8;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    for(n=0;n<4;++n) //retry , until I2C success
    {
      if(n>2) return FAIL;

      if( SensorRegArrayR((I2C_ARRAY*)id_from_sensor,table_length) == SUCCESS) //read sensor ID from I2C
          break;
      else
          continue;
    }

    //convert sensor id to u32 format
    for(i=0;i<table_length;++i)
    {
      if( id_from_sensor[i].data != Sensor_id_table[i].data )
        return FAIL;
      //*id = id_from_sensor[i].data;
      *id = ((*id)+ id_from_sensor[i].data)<<8;
    }

    *id >>= 8;
    SENSOR_DMSG("[%s]gc2093 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    //SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);

    return SUCCESS;
}

static int gc2093_SetPatternMode(ms_cus_sensor *handle,u32 mode)
{

    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);

    return SUCCESS;
}

static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);

// LINEAR_RES_1
static int pCus_init_mipi2lane_linear_1920x1080_60fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x1080_60fps);i++) {
        if (Sensor_init_table_1920x1080_60fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x1080_60fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x1080_60fps[i].reg, 
								Sensor_init_table_1920x1080_60fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

// LINEAR_RES_2
/*
static int pCus_init_mipi2lane_linear_1920x1080_70fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x1080_70fps);i++) {
        if (Sensor_init_table_1920x1080_70fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x1080_70fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x1080_70fps[i].reg, 
								Sensor_init_table_1920x1080_70fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}
*/

// LINEAR_RES_3
static int pCus_init_mipi2lane_linear_1920x1080_80fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x1080_80fps);i++) {
        if (Sensor_init_table_1920x1080_80fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x1080_80fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x1080_80fps[i].reg, 
								Sensor_init_table_1920x1080_80fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

// LINEAR_RES_4
static int pCus_init_mipi2lane_linear_1920x1080_90fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x1080_90fps);i++) {
        if (Sensor_init_table_1920x1080_90fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x1080_90fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x1080_90fps[i].reg, 
								Sensor_init_table_1920x1080_90fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

// LINEAR_RES_5
/*
static int pCus_init_mipi2lane_linear_1920x960_90fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x960_90fps);i++) {
        if (Sensor_init_table_1920x960_90fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x960_90fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x960_90fps[i].reg, 
								Sensor_init_table_1920x960_90fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}
*/

// LINEAR_RES_6
static int pCus_init_mipi2lane_linear_1920x864_100fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x864_100fps);i++) {
        if (Sensor_init_table_1920x864_100fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x864_100fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x864_100fps[i].reg, 
								Sensor_init_table_1920x864_100fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

// LINEAR_RES_7
static int pCus_init_mipi2lane_linear_1920x768_120fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1920x768_120fps);i++) {
        if (Sensor_init_table_1920x768_120fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1920x768_120fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1920x768_120fps[i].reg, 
								Sensor_init_table_1920x768_120fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

// LINEAR_RES_8
static int pCus_init_mipi2lane_linear_1280x720_120fps(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
    int i,cnt;
    //ISensorIfAPI *sensor_if = handle->sensor_if_api;

    for (i=0;i< ARRAY_SIZE(Sensor_init_table_1280x720_120fps);i++) {
        if (Sensor_init_table_1280x720_120fps[i].reg==0xffff) {
            SENSOR_MSLEEP(Sensor_init_table_1280x720_120fps[i].data);
        } else {
            cnt = 0;
            while (SensorReg_Write(Sensor_init_table_1280x720_120fps[i].reg, 
								Sensor_init_table_1280x720_120fps[i].data) != SUCCESS) {
                cnt++;
                SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
                if(cnt>=10) {
                    SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                SENSOR_MSLEEP(10);
            }
        }
    }

    pCus_SetOrien(handle, params->cur_orien);
	vts_reg[0].data = (params->expo.vts >> 8) & 0x003f;
    vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
    return SUCCESS;
}

static int pCus_GetVideoResNum( ms_cus_sensor *handle, u32 *ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}

static int pCus_GetVideoRes(ms_cus_sensor *handle, u32 res_idx, cus_camsensor_res **res)
{
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int pCus_GetCurVideoRes(ms_cus_sensor *handle, u32 *cur_idx, cus_camsensor_res **res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int pCus_SetVideoRes(ms_cus_sensor *handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    gc2093_params *params = (gc2093_params *)handle->private_data;
	
    if (res_idx >= num_res) {
        return FAIL;
    }
	
	handle->video_res_supported.ulcur_res = res_idx;
	
    switch (res_idx) {
        case LINEAR_RES_1: //"1920x1080@60fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_1;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x1080_60fps;
            vts_30fps=1127;
            params->expo.vts = vts_30fps;
            params->expo.fps = 60;
            Preview_line_period  = 14789;
            break;
/*		case LINEAR_RES_2: //"1920x1080@70fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_2;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x1080_70fps;
            vts_30fps=1125;
            params->expo.vts = vts_30fps;
            params->expo.fps = 70;
            Preview_line_period  = 12698;
            break;*/
		case LINEAR_RES_3: //"1920x1080@80fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_3;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x1080_80fps;
            vts_30fps=1125;
            params->expo.vts = vts_30fps;
            params->expo.fps = 80;
            Preview_line_period  = 11111;
            break;
		case LINEAR_RES_4: //"1920x1080@90fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_4;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x1080_90fps;
            vts_30fps=1122;
            params->expo.vts = vts_30fps;
            params->expo.fps = 90;
            Preview_line_period  = 9902;
            break;
/*		case LINEAR_RES_5: //"1920x960@90fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_5;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x960_90fps;
            vts_30fps=998;
            params->expo.vts = vts_30fps;
            params->expo.fps = 90;
            Preview_line_period  = 11133;
            break;*/
		case LINEAR_RES_6: //"1920x864@100fps" 
            handle->video_res_supported.ulcur_res = LINEAR_RES_6;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x864_100fps;
            vts_30fps=899;
            params->expo.vts = vts_30fps;
            params->expo.fps = 100;
            Preview_line_period  = 11123;
            break;
		case LINEAR_RES_7: //"1920x768@120fps"
            handle->video_res_supported.ulcur_res = LINEAR_RES_7;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1920x768_120fps;
            vts_30fps=799;
            params->expo.vts = vts_30fps;
            params->expo.fps = 120;
            Preview_line_period  = 10430;
            break;
		case LINEAR_RES_8: //"1280x720@120fps"
            handle->video_res_supported.ulcur_res = LINEAR_RES_8;
            handle->pCus_sensor_init = pCus_init_mipi2lane_linear_1280x720_120fps;
            vts_30fps=750;
            params->expo.vts = vts_30fps;
            params->expo.fps = 120;
            Preview_line_period  = 11111;
            break;
        default:
            break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT *orit) 
{
     char sen_data;
//    gc2093_params *params = (gc2093_params *)handle->private_data;
    //sen_data = params->tMirror_table[0].data;
	sen_data = mirror_reg[0].data;
    SENSOR_DMSG("mirror:%x\r\n", sen_data);
    switch(sen_data & 0x0f) {
      case 0x03:
        *orit = CUS_ORIT_M0F0;
        break;
      case 0x02:
        *orit = CUS_ORIT_M1F0;
        break;
      case 0x01:
        *orit = CUS_ORIT_M0F1;
        break;
      case 0x00:
        *orit = CUS_ORIT_M1F1;
        break;
      } 
      return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT orit)
{
	gc2093_params *params = (gc2093_params *)handle->private_data;
	switch(orit) {
    case CUS_ORIT_M0F0:
      if (mirror_reg[0].data != 0x02) {
          mirror_reg[0].data = 0x83;
        // params->orient_dirty = true;
		  params->reg_dirty = true;
      }
      break;
    case CUS_ORIT_M1F0:
      if (mirror_reg[0].data != 0x03) {
          mirror_reg[0].data = 0x82;
        //  params->orient_dirty = true;
		  params->reg_dirty = true;
      }
      break;
    case CUS_ORIT_M0F1:
      if (mirror_reg[0].data != 0x00) {
          mirror_reg[0].data = 0x80;
        //  params->orient_dirty = true;
		  params->reg_dirty = true;
      }
      break;
    case CUS_ORIT_M1F1:
      if (mirror_reg[0].data!=0x01) {
          mirror_reg[0].data = 0x81;
        //  params->orient_dirty = true;
		  params->reg_dirty = true;
      }
      break;
    }
    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor *handle)
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
	
	params->expo.preview_fps = vts_30fps * Preview_MAX_FPS * 1000/ params->expo.vts;

    return params->expo.preview_fps;
}

static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps)
{
    u32 vts=0;
    gc2093_params *params = (gc2093_params *)handle->private_data;
    u32 max_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].max_fps;
    u32 min_fps = handle->video_res_supported.res[handle->video_res_supported.ulcur_res].min_fps;
  //  pr_info("[%s]  max_min_fps : %d ,%d\n\n", __FUNCTION__,max_fps,min_fps);
    if(fps>=min_fps && fps <= max_fps){
        params->expo.fps = fps;
        params->expo.vts=  (vts_30fps*max_fps)/fps;
    }else if((fps >= (min_fps*1000)) && (fps <= (max_fps*1000))){
        params->expo.fps = fps;
        params->expo.vts=  (vts_30fps*(max_fps*1000))/fps;
    }else{
        SENSOR_DMSG("[%s] FPS %d out of range.\n",__FUNCTION__,fps);
        return FAIL;
    }

     if ((params->expo.line) > (params->expo.vts)-4) {
        vts = params->expo.line + 4;
    }else
        vts = params->expo.vts;
    /* params->tVts_reg[0].data = (vts >> 8) & 0x003f;
    params->tVts_reg[1].data = (vts >> 0) & 0x00ff; */
	vts_reg[0].data = (vts >> 8) & 0x003f;
    vts_reg[1].data = (vts >> 0) & 0x00ff;
    params->reg_dirty = true;
    return SUCCESS;

}

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
//AE status notification
static int pCus_AEStatusNotify(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
   gc2093_params *params = (gc2093_params *)handle->private_data;

   switch(status)
   {
       case CUS_FRAME_INACTIVE:
       break;
       case CUS_FRAME_ACTIVE:
       if(params->reg_dirty)
       {
 //           SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, sizeof(expo_reg)/sizeof(I2C_ARRAY));
 //           SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, sizeof(gain_reg)/sizeof(I2C_ARRAY));
  //          SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, sizeof(vts_reg)/sizeof(I2C_ARRAY));
				SensorRegArrayW((I2C_ARRAY*)mirror_reg, sizeof(mirror_reg)/sizeof(I2C_ARRAY));
				SensorRegArrayW((I2C_ARRAY*)vts_reg, sizeof(vts_reg)/sizeof(I2C_ARRAY));
				SensorRegArrayW((I2C_ARRAY*)expo_reg, sizeof(expo_reg)/sizeof(I2C_ARRAY));
				SensorRegArrayW((I2C_ARRAY*)gain_reg, sizeof(gain_reg)/sizeof(I2C_ARRAY));
			params->reg_dirty = false;
       }
       break;
       default :
       break;
   }
   return SUCCESS;
}

static int pCus_GetAEUSecs(ms_cus_sensor *handle, u32 *us) 
{
	int rc=0;
	u32 lines = 0;

	lines  = (u32)(expo_reg[1].data&0xff);
	lines |= (u32)(expo_reg[0].data&0x3f)<<8;
    *us = (lines*Preview_line_period)/1000;
	SENSOR_DMSG("[%s] sensor expo lines/us %d, %dus\n", __FUNCTION__, lines, *us);
	
	return rc;
}

static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us) 
{
    u32 lines = 0, vts = 0, i = 0;
    gc2093_params *params = (gc2093_params *)handle->private_data;
	I2C_ARRAY expo_reg_temp[] = { 
		{0x0003, 0x00}, // expo[15:8]
		{0x0004, 0x02}, // expo[7:0] 
    };

	memcpy(expo_reg_temp, expo_reg, sizeof(expo_reg));
	lines = (1000*us)/Preview_line_period;

    if (lines < 1) 
		lines=1;
    if (lines > params->expo.vts - 36) {
		lines = params->expo.vts - 36;
        vts = lines + 4;
    } else
        vts = params->expo.vts;

    params->expo.line = lines;
    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", 
		__FUNCTION__, us, lines, params->expo.vts);
	
    expo_reg[1].data = (lines) & 0x00ff;
    expo_reg[0].data = (lines>>8) & 0x003f;
    vts_reg[0].data = (vts >> 8) & 0x003f;
    vts_reg[1].data = (vts >> 0) & 0x00ff;
	for (i = 0; i < sizeof(expo_reg)/sizeof(I2C_ARRAY); i++) {
		if (expo_reg[i].data != expo_reg_temp[i].data) {
			params->reg_dirty = true;
			break;
		}
	}
	
    return SUCCESS;
}

static int pCus_GetAEGain(ms_cus_sensor *handle, u32* gain) 
{
    int rc = 0;

	u32 Again_0 = 1, Dgain_0 = 1;
    u8 i;
    u32  Fine_again = 1024;

   
    for(i = 0;i < sizeof(gain_table)/sizeof(Gain_ARRAY);i++ )
        {
            if(( gain_table[i].again_reg_val_0 == gain_reg[0].data) && (gain_table[i].again_reg_val_1 == gain_reg[1].data))
            {
                Fine_again = gain_table[i].gain;
                break;
            }
        }

     Dgain_0 = (gain_reg[4].data & 0xf) * 64 + (gain_reg[5].data >> 2);
     Again_0 = Fine_again;
     *gain =(u32)((Again_0 * Dgain_0)/64);
    return rc;
}

static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain) 
{
    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain) 
{
    gc2093_params *params = (gc2093_params *)handle->private_data;
	u32 dgain = 1;
    u8 dgain_0 = 1, dgain_1 = 0;
    u8 i = 0,tmp = 0;
	
	I2C_ARRAY gain_reg_temp[] = {
		{0x00b3, 0x00},
		{0x00b8, 0x01},
		{0x00b9, 0x00},
		{0x0078, 0x20},
		{0x00b1, 0x01},
		{0x00b2, 0x00},
    };
    memcpy(gain_reg_temp, gain_reg, sizeof(gain_reg));

    params->expo.final_gain = gain;

    if (gain < 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAX_GAIN  * 1024) {
        gain = SENSOR_MAX_GAIN  * 1024;
    }

	for(i = 0;i < sizeof(gain_table)/sizeof(Gain_ARRAY);i++ ) {
		if((gain >= gain_table[i].gain) && (gain < gain_table[i + 1].gain)) {
			tmp = i;
			break;
		} else
			tmp = sizeof(gain_table)/sizeof(Gain_ARRAY) - 1;
	} 
		
    dgain =(gain*64)/(gain_table[tmp].gain);
    dgain_0 = (dgain)>>6;
    dgain_1 =(dgain & 0x3f)<<2;
    //gain_reg[0].data  = gain_table[tmp].again_reg_val_0;
    gain_reg[0].data  = gain_table[tmp].again_reg_val_0;
    gain_reg[1].data  = gain_table[tmp].again_reg_val_1;
    gain_reg[2].data  = gain_table[tmp].again_reg_val_2;
	gain_reg[3].data  = gain_table[tmp].again_reg_val_3;

    gain_reg[4].data  = dgain_0;
    gain_reg[5].data  = dgain_1;
	
    SENSOR_DMSG("[%s] set gain/regH/regL=%d/0x%x/0x%x\n", __FUNCTION__, gain,gain_reg[0].data,gain_reg[1].data);

	for (i = 0; i < sizeof(gain_reg)/sizeof(I2C_ARRAY); i++) {
		if (gain_reg[i].data != gain_reg_temp[i].data) {
			params->reg_dirty = true;
			break;
		}
    }
	
    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor *handle, u32 *min, u32 *max) 
{
	*min = 30;
	*max = 1000000/Preview_MIN_FPS;
	return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor *handle, u32 *min, u32 *max) 
{
	*min = 1024;
	*max = SENSOR_MAXGAIN*1024;
	return SUCCESS;
}

static int gc2093_GetShutterInfo(struct __ms_cus_sensor* handle,CUS_SHUTTER_INFO *info)
{
    info->max = 1000000000/Preview_MIN_FPS;
    info->min = (Preview_line_period * 1);
    info->step = Preview_line_period;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num) 
{

    return SUCCESS;
}

int cus_camsensor_init_handle_linear(ms_cus_sensor* drv_handle) 
{
	ms_cus_sensor *handle = drv_handle;
    gc2093_params *params;
	int res;
	
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    //private data allocation & init
    if (handle->private_data == NULL) {
        SENSOR_EMSG("[%s] Private data is empty!\n", __FUNCTION__);
        return FAIL;
    }
    params = (gc2093_params *)handle->private_data;
    /* memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tGain_reg_HDR_DOL_SEF, gain_reg_HDR_DOL_SEF, sizeof(gain_reg_HDR_DOL_SEF));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tExpo_reg_HDR_DOL_SEF, expo_reg_HDR_DOL_SEF, sizeof(expo_reg_HDR_DOL_SEF));
 //   memcpy(params->tMax_short_exp_reg, max_short_exp_reg, sizeof(max_short_exp_reg));
    memcpy(params->tMirror_table, mirror_table, sizeof(mirror_table)); */

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    SENSOR_DMSG(handle->model_id,"gc2093_MIPI");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    //SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type    = SENSOR_ISP_TYPE;  //ISP_SOC;
    //handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus     = SENSOR_IFBUS_TYPE;//CUS_SENIF_BUS_PARL;
    handle->data_prec   = SENSOR_DATAPREC;  //CUS_DATAPRECISION_8;
    handle->data_mode   = SENSOR_DATAMODE;
    handle->bayer_id    = SENSOR_BAYERID;   //CUS_BAYER_GB;
    handle->RGBIR_id    = SENSOR_RGBIRID;
    handle->orient      = SENSOR_ORIT;      //CUS_ORIT_M1F1;
    //handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; //don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_NONE;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 0; //Short frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    
    handle->video_res_supported.ulcur_res = 0; //default resolution index is 0.
	for (res = 0; res < LINEAR_RES_END; res++) {
        handle->video_res_supported.num_res = res + 1;
        handle->video_res_supported.res[res].width = gc2093_mipi_linear[res].senif.preview_w;
        handle->video_res_supported.res[res].height = gc2093_mipi_linear[res].senif.preview_h;
        handle->video_res_supported.res[res].max_fps = gc2093_mipi_linear[res].senout.max_fps;
        handle->video_res_supported.res[res].min_fps = gc2093_mipi_linear[res].senout.min_fps;
        handle->video_res_supported.res[res].crop_start_x = gc2093_mipi_linear[res].senif.crop_start_X;
        handle->video_res_supported.res[res].crop_start_y = gc2093_mipi_linear[res].senif.crop_start_y;
        handle->video_res_supported.res[res].nOutputWidth = gc2093_mipi_linear[res].senout.width;
        handle->video_res_supported.res[res].nOutputHeight = gc2093_mipi_linear[res].senout.height;
        sprintf(handle->video_res_supported.res[res].strResDesc, gc2093_mipi_linear[res].senstr.strResDesc);
    }

    // i2c
    handle->i2c_cfg.mode                = SENSOR_I2C_LEGACY;    //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt                 = SENSOR_I2C_FMT;       //CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address             = SENSOR_I2C_ADDR;      //0x5a;
    handle->i2c_cfg.speed               = SENSOR_I2C_SPEED;     //320000;

    // mclk
    handle->mclk                        = Preview_MCLK_SPEED;

    //polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY               = SENSOR_PWDN_POL;  //CUS_CLK_POL_NEG;
    handle->reset_POLARITY              = SENSOR_RST_POL;   //CUS_CLK_POL_NEG;
    handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay       = 2;
    handle->ae_shutter_delay    = 2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 1;

    ///calibration
    handle->sat_mingain=g_sensor_ae_min_gain;


    handle->pCus_sensor_release     = cus_camsensor_release_handle;
    handle->pCus_sensor_init        = pCus_init_mipi2lane_linear_1920x1080_60fps;
    handle->pCus_sensor_poweron     = pCus_poweron ;
    handle->pCus_sensor_poweroff    = pCus_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID       = pCus_GetSensorID   ;

    handle->pCus_sensor_GetVideoResNum = pCus_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes       = pCus_GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes  = pCus_GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes       = pCus_SetVideoRes;

    handle->pCus_sensor_GetOrien          = pCus_GetOrien      ;
    handle->pCus_sensor_SetOrien          = pCus_SetOrien      ;
    handle->pCus_sensor_GetFPS          = pCus_GetFPS      ;
    handle->pCus_sensor_SetFPS          = pCus_SetFPS      ;
    //handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = gc2093_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    //handle->pCus_sensor_GetAETrigger_mode      = pCus_GetAETrigger_mode;
    //handle->pCus_sensor_SetAETrigger_mode      = pCus_SetAETrigger_mode;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs      = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs      = pCus_SetAEUSecs;
    handle->pCus_sensor_GetAEGain       = pCus_GetAEGain;

    handle->pCus_sensor_SetAEGain       = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs= pCus_GetAEMinMaxUSecs;

     //sensor calibration
    handle->pCus_sensor_SetAEGain_cal   = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity=pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = gc2093_GetShutterInfo;
    params->expo.vts=vts_30fps;
    //params->expo.fps = 60;
    //params->expo.line= 1000;
    params->reg_dirty = false;

    return SUCCESS;
}

int cus_camsensor_release_handle(ms_cus_sensor *handle) 
{
    return SUCCESS;
}

SENSOR_DRV_ENTRY_IMPL_END_EX(gc2093_MIPI, cus_camsensor_init_handle_linear, NULL, NULL, gc2093_params);