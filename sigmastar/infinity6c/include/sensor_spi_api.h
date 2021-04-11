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

/*! @file isp_spi_api.h
      @brief This file contains Infinity ISP SPI basic API.
*/

/** @defgroup group1 ISP SPI API
 * @{
 */
#ifndef ISP_SPI_API_H
#define ISP_SPI_API_H

#define SENSOR_SPI_SUCCESS (0)
#define SENSOR_SPI_FAIL (-1)
#define SENSOR_SPI_NOT_SUPPORT (-2)

/**@brief handle version info */
// typedef struct _version_info{
//     u16          major;
//     u16          minor;
// }__attribute__((packed, aligned(1))) version_info;

/*! @brief SPI API handle.*/
struct __spi_handle_t;

/*! @brief SPI batch read/write data.*/
typedef struct _SPI_ARRAY {
    u16 reg; /**< Register address.*/
    u16 data; /**< Data.*/
} SPI_ARRAY;

typedef enum {
    SPI_FMT_A8D8, /**< 8 bits Address, 8 bits Data */
    SPI_FMT_A16D8, /**< 16 bits Address 8 bits Data */
    SPI_FMT_A8D16, /**< 8 bits Address 16 bits Data */
    SPI_FMT_A16D16, /**< 16 bits Address 16 bits Data */
    SPI_FMT_END, /**< Reserved */
} ISP_SPI_FMT;

/*! @brief ISP_SPI_MODE Internal use for SPI API*/
typedef enum {
    SPI_SINPLEX,
    SPI_HALF_DUPLEX,
    SPI_DUPLEX,
} ISP_SPI_MODE;

/*! @brief app_spic_cfg SPI setting for sensor and bus.*/
typedef struct _app_spi_cfg {
    ISP_SPI_FMT fmt;
    u8 u8bis_per_word;
    u8 u8Mode;
    u32 Speed; // nSpeed 46.875K~72M
    u16 RESERVED;
} __attribute__((packed, aligned(1))) app_spi_cfg;

/*! @brief The interface of SPI APIs export to user*/
typedef struct _spi_handle_t {
    // int version;
    version_info version;
    void* pdata;

    u32 nSensorID;
    /** @brief Open isp spi port. This function must be called before using isp SPI APIs.
    Call spi_close to close isp spi port and allocated resource.
    @param[in] handle Handle to isp spi api.
    @param[in] cfg SPI initial configuration.
    @retval SENSOR_SPI_SUCCESS or SENSOR_SPI_FAIL if error occurs.
    */
    int (*spi_open)(struct _spi_handle_t* handle, app_spi_cfg* cfg);

    /** @brief Close isp SPI port. Call this functon to release resource which allocated form spi_open.
    @param[in] handle Handle to isp spi api.
    @param[in] cfg SPI configuration mode and spped are necessary in this stage.
    @retval SENSOR_SPI_SUCCESS or SENSOR_SPI_FAIL if error occurs.
    */
    int (*spi_close)(struct _spi_handle_t* handle);

    /** @brief Write single data to device.
    @param[in] handle Handle to isp spi api.
    @param[in] cfg SPI config, fmd and address are necessary in this stage.
    @param[in] reg Device register address address width depend on cfg->fmt.
    @param[in] data Data to write, data width depend on cfg->fmt.
    @retval SENSOR_SPI_SUCCESS or SENSOR_SPI_FAIL if error occurs.
    */
    int (*spi_tx)(struct _spi_handle_t* handle, app_spi_cfg* cfg, u16 reg, u16 data);

    /** @brief Read single data from device.
    @param[in] handle Handle to isp spi api.
    @param[in] cfg SPI config, fmd and address are necessary in this stage.
    @param[in] reg Device register address address width depend on cfg->fmt.
    @param[out] data Data buffer for read, data width depend on cfg->fmt.
    @retval SENSOR_SPI_SUCCESS or SENSOR_SPI_FAIL if error occurs.
    */
    int (*spi_rx)(struct _spi_handle_t* handle, app_spi_cfg* cfg, u16 reg, volatile u16* data);
} spi_handle_t;

/** @} */ // end of ISP SPI API

#define SPI_CPHA 0x01
#define SPI_CPOL 0x02

#define SPI_MODE_0 (0 | 0)
#define SPI_MODE_1 (0 | SPI_CPHA)
#define SPI_MODE_2 (SPI_CPOL | 0)
#define SPI_MODE_3 (SPI_CPOL | SPI_CPHA)

#define SPI_CS_HIGH 0x04
#define SPI_LSB_FIRST 0x08
#define SPI_3WIRE 0x10
#define SPI_LOOP 0x20
#define SPI_NO_CS 0x40
#define SPI_READY 0x80
#define SPI_TX_DUAL 0x100
#define SPI_TX_QUAD 0x200
#define SPI_RX_DUAL 0x400
#define SPI_RX_QUAD 0x800

#endif
