/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 131362 $
 * $Date: 2023-12-25 17:18:11 +0800 (週一, 25 十二月 2023) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef _LINUX_NVT_TOUCH_H
#define _LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/slab.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "nt519xx_mem_map.h"
#include "nvt_flash_info.h"

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943

//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING

//---bus transfer length---
#define BUS_TRANSFER_LENGTH 256

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define CHANGE_I2C_DEVICE_ADDR (0)
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62
#define I2C_DP_Address 0x6F // Both DP Primary and Secondary IC

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info.---
#define TOUCH_MAX_WIDTH 1920
#define TOUCH_MAX_HEIGHT 1080
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000

/* Each data length in event buffer (unit: byte) */
#define ASIL_FLAG_LEN     (1) // 1st byte of event buffer
#define POINT_DATA_LEN    (7) // data length of every finger point info
#define BUTTON_STATUS_LEN (2) // button status length
#define TOUCH_EVENT_LEN   (ASIL_FLAG_LEN + (TOUCH_MAX_FINGER_NUM * POINT_DATA_LEN) + BUTTON_STATUS_LEN) // event buffer length

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

/* Probe drive all the time even if trim version check failed */
#define DRIVER_FORCE_PROBE 0
#define DEFAULT_MMAP (NT51920_2chip_memory_map)
#define DEFAULT_HW_INFO (NT51920_2chip_hw_info)

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 0
#define NVT_SAVE_TEST_DATA_IN_FILE 1
#define MT_PROTOCOL_B 1
#define NVT_TOUCH_EVENT_CRC8 1
#if NVT_TOUCH_EVENT_CRC8
#define ASIL_INFO_LEN       (6)
#define CRC_VALUE_LEN       (1) // CRC value length
#define TOUCH_EVENT_CRC_LEN (TOUCH_EVENT_LEN + ASIL_INFO_LEN + CRC_VALUE_LEN) // event buffer length with crc
#endif
#define WAKEUP_GESTURE 0
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

#define BOOT_UPDATE_FIRMWARE 0
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"

#if BOOT_UPDATE_FIRMWARE
#define BOOT_UPDATE_INIT_CODE 0
#define BOOT_UPDATE_INIT_CODE_NAME "novatek_dp_fw.bin"

#define BOOT_UPDATE_OSD 0
#define BOOT_UPDATE_OSD_NAME "novatek_dp_osd.bin"
#endif

//---ASIL interrupt error hanfler---
#define NVT_TOUCH_ASIL 0
#if NVT_TOUCH_ASIL
#define NVT_ASIL_NAME "NVT-ASIL"
#define ABD_TRIGGER_TYPE IRQ_TYPE_EDGE_FALLING
#endif

//---OSD image trigger---
#define NVT_OSD_TRIGGER 0

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 0
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */
#define NVT_TOUCH_FRAME_COUNTER 0

#define DUMMY_BYTES     (1) // 1st byte (low byte of address) of bus buffer
#define NVT_XBUF_LEN	(1025)

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	struct notifier_block drm_notif;
#else
	struct notifier_block fb_notif;
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint8_t fw_ver;
#if ((BOOT_UPDATE_FIRMWARE) && (BOOT_UPDATE_INIT_CODE))
	uint8_t dp_fw_ver;
#endif
#if NVT_OSD_TRIGGER
	uint8_t dp_osd_en;
#endif /* NVT_OSD_TRIGGER */
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
#if NVT_TOUCH_ASIL
	uint32_t abd_trigger_type;
#endif
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
#if NVT_TOUCH_ASIL
	int32_t abd_irq;
	int32_t abd_gpio;
	uint32_t abd_flags;
#endif
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	CASCADE_TYPE cascade_type;
	CASCADE_CHIP_NUMBER cascade_num;
	uint8_t evtbuf_fmt;
	uint8_t cust_func; /* Customized Function */
	uint8_t combuf_report;
	uint16_t nvt_pid;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
#if DRIVER_FORCE_PROBE
	bool force_probe;
#endif
	uint8_t flash_mid;
	uint16_t flash_did; /* 2 bytes did read by 9Fh cmd */
	const flash_info_t *match_finfo;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data {
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef struct gcm_transfer {
	uint8_t flash_cmd;
	uint32_t flash_addr;
	uint16_t flash_checksum;
	uint8_t flash_addr_len;
	uint8_t pem_byte_len; /* performance enhanced mode / contineous read mode byte length*/
	uint8_t dummy_byte_len;
	uint8_t *tx_buf;
	uint16_t tx_len;
	uint8_t *rx_buf;
	uint16_t rx_len;
} gcm_xfer_t;

typedef enum {
	RESET_STATE_INIT = 0xA0, // IC reset
	RESET_STATE_REK,         // ReK baseline
	RESET_STATE_REK_FINISH,  // baseline is ready
	RESET_STATE_NORMAL_RUN,  // normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FRAME_COUNTER                 = 0x70,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_CASCADE_CHIP_NUMBERS          = 0x99,
    EVENT_MAP_PROJECTID                     = 0x9A,
} I2C_EVENT_MAP;

typedef enum {
  NVTWRITE = 0,
  NVTREAD  = 1
} NVT_I2C_RW;

typedef enum {
  MDATA_RAW_AF_DIFF    = 0x01, /* 2D diff (after FW algo) data */
  MDATA_RAW_IIR        = 0x02, /* 2D raw data */
  MDATA_RAW_BL         = 0x03, /* 2D baseline data */
} NVT_MDATA_TYPE;

#if NVT_TOUCH_MP
typedef enum {
	FREQ_F1    = 0x01,
	FREQ_F2    = 0x02,
	FREQ_F3    = 0x03,
	FREQ_F4    = 0x04,
} FW_FREQUENCY;

typedef enum {
	WITH_NF    = 0x01,
	WITHOUT_NF = 0x02,
} FW_NORMALIZE;
#endif

//---extern structures---
extern struct nvt_ts_data *ts;

//---extern functions---
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern void nvt_bootloader_reset(void);
extern void nvt_sw_reset_idle(void);
extern int32_t nvt_wait_bootloader_done(void);
extern int32_t nvt_clear_crc_err_flag(void);
extern void nvt_read_fw_history(uint32_t fw_history_addr);
extern int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_get_fw_info(void);
extern int32_t nvt_check_cascade_chip_numbers(void);
extern int32_t nvt_clear_fw_status(void);
extern int32_t nvt_check_fw_status(void);
extern int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr);
extern int32_t nvt_write_addr(uint32_t addr, uint8_t data);
extern int32_t nvt_read_reg(nvt_ts_reg_t reg, uint8_t *val);
extern int32_t nvt_write_reg(nvt_ts_reg_t reg, uint8_t val);

#if BOOT_UPDATE_FIRMWARE
extern int32_t Check_CheckSum_GCM(const struct firmware *fw_entry);
extern int32_t nvt_check_flash_end_flag_gcm(void);
extern int32_t Update_Firmware_GCM(const struct firmware *fw_entry);
#endif

#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
extern void nvt_frame_counter_enable(uint8_t enable);
#endif

extern void nvt_stop_crc_reboot(void);

#endif /* _LINUX_NVT_TOUCH_H */
