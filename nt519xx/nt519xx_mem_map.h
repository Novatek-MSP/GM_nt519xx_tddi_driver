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

#define CHIP_VER_TRIM_ADDR 0xFF004 // 0x3F004
#define SWRST_SIF_ADDR     0xFF0FE // 0x3F0FE
#define BLD_SPE_PUPS_ADDR  0xFF135 // 0x3F135

typedef enum {
	BIT0 = 0x01,
	BIT1 = 0x02,
	BIT2 = 0x04,
	BIT3 = 0x08,
	BIT4 = 0x10,
	BIT5 = 0x20,
	BIT6 = 0x40,
	BIT7 = 0x80
} BIT_MASK;

typedef enum {
	CASCADE_NONE          = 0x00, // none cascade cases
	CASCADE_ENB_CASC      = 0x01, // identity by ENB_CASC[0]
	CASCADE_SDLOC         = 0x02, // identity by SDLOC[2:0]
	CASCADE_NT51925_SDLOC = 0x03, // identity by SDLOC[2:0]
	CASCADE_IC_NUM        = 0x04  // identity by IC_NUM[3:2]
} CASCADE_TYPE;

typedef enum {
	ENB_CASC_2CHIP = 0x00,
	ENB_CASC_1CHIP = 0x01
} CASCADE_TYPE_ENB_CASC;

/*
 * Cascade type: SDLOC
 * (IC_PRIMARY ID: 1xx, Others' IDs: 0xx)
 *
 * 1 CHIP: 100
 * 2 CHIP: 110 (001)
 * 3 CHIP: (000) 111 (001)
 * 4 CHIP: (000) 111 (011 001)
 * 5 CHIP: (000 010) 111 (011 001)
 */
typedef enum {
	SDLOC_1CHIP           = 0x04, // IC_P 100
	SDLOC_2CHIP           = 0x06, // IC_P 110
	SDLOC_3CHIP_AND_ABOVE = 0x07  // IC_P 111
} CASCADE_TYPE_SDLOC;

/*
 * NT51925 Cascade type: SDLOC
 * (IC_PRIMARY ID: xx1, Others' IDs: xx0)
 *
 * 1 CHIP: 011
 * 2 CHIP: 001 (000)
 * 3 CHIP: (110) 101 (100)
 */
typedef enum {
	NT51925_SDLOC_2CHIP           = 0x01, // IC_P 001
	NT51925_SDLOC_1CHIP           = 0x03, // IC_P 011
	NT51925_SDLOC_3CHIP_AND_ABOVE = 0x05  // IC_P 101
} CASCADE_TYPE_NT51925_SDLOC;

/*
 * Cascade type: IC_NUM
 */
typedef enum {
	IC_NUM_3CHIP           = 0x00,
	IC_NUM_1CHIP           = 0x01,
	IC_NUM_2CHIP           = 0x02,
	IC_NUM_3CHIP_AND_ABOVE = 0x03
} CASCADE_TYPE_IC_NUM;

typedef enum {
	CASCADE_CHECK_ERR = 0,
	NONE_CASCADE_CASE = 1,
	CASCADE_1CHIP     = NONE_CASCADE_CASE,
	CASCADE_2CHIP     = 2,
	CASCADE_3CHIP     = 3,
	CASCADE_4CHIP     = 4,
	CASCADE_5CHIP     = 5,
	CASCADE_CHIP_MAX  = CASCADE_5CHIP
} CASCADE_CHIP_NUMBER;

typedef struct nvt_ts_reg {
	uint32_t addr;   /* byte in which address */
	uint8_t bitmask; /* in which bits of that byte */
} nvt_ts_reg_t;

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
	uint32_t Q_WR_CMD_ADDR;
	/* FW multi-task */
	uint32_t COMMON_BUF_ADDR;
	/* FW History */
	uint32_t MMAP_HISTORY_EVENT0;
	uint32_t MMAP_HISTORY_EVENT1;
	/* Cascade */
	nvt_ts_reg_t ENB_CASC_REG;
	nvt_ts_reg_t SDLOC_REG;
	nvt_ts_reg_t IC_NUM_REG;
	/* BLD CRC */
	nvt_ts_reg_t CRC_DONE_REG;
	nvt_ts_reg_t TP_FLASH_GRANT_REG;
	nvt_ts_reg_t WP_TP_REG;
	/* GCM FW update */
	uint32_t GCM_CODE_ADDR;
	uint32_t FLASH_CMD_ADDR;
	uint32_t FLASH_CMD_ISSUE_ADDR;
	uint32_t FLASH_CKSUM_STATUS_ADDR;
	uint32_t GCM_FLAG_ADDR;
	nvt_ts_reg_t PP4IO_EN_REG;
	nvt_ts_reg_t BLD_RD_ADDR_SEL_REG;
	nvt_ts_reg_t BLD_RD_IO_SEL_REG;
};

struct nvt_ts_hw_info {
	CASCADE_TYPE cascade_type;
	CASCADE_CHIP_NUMBER cascade_num;
	/* FW Config */
	uint8_t default_x_num;
	uint8_t default_y_num;
	uint16_t default_abs_x_max;
	uint16_t default_abs_y_max;
	uint8_t default_max_touch_num;
	uint8_t default_max_button_num;
};

static const struct nvt_ts_mem_map NT51900_memory_map = {
	.EVENT_BUF_ADDR            = 0x2A800,
	.RAW_PIPE0_ADDR            = 0x2C620, // 0x2BE50
	.RAW_PIPE1_ADDR            = 0x2C620,
	.BASELINE_ADDR             = 0x2CDF0,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x2D8E0,
	.DIFF_PIPE1_ADDR           = 0x2E0B0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x24000,
	.RW_FLASH_DATA_ADDR        = 0x24002,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x356E4,
	.MMAP_HISTORY_EVENT1       = 0x35724,
	/* Cascade */
	.ENB_CASC_REG              = {.addr = 0x3F02C, .bitmask = BIT0},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0x3F133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0x3F134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0x3F13B, .bitmask = BIT0},
	.GCM_CODE_ADDR             = 0x3F780,
	.FLASH_CMD_ADDR            = 0x3F783,
	.FLASH_CMD_ISSUE_ADDR      = 0x3F78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0x3F78F,
	.GCM_FLAG_ADDR             = 0x3F793,
};

static const struct nvt_ts_mem_map NT51920_1chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x30500,
	.RAW_PIPE0_ADDR            = 0x32E40, // 0x31E80
	.RAW_PIPE1_ADDR            = 0x32E40,
	.BASELINE_ADDR             = 0x33E00,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x34DC0,
	.DIFF_PIPE1_ADDR           = 0x35D80,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x24000,
	.RW_FLASH_DATA_ADDR        = 0x24002,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x3B6BC,
	.MMAP_HISTORY_EVENT1       = 0x3B6FC,
	/* Cascade */
	.ENB_CASC_REG              = {.addr = 0x3F02C, .bitmask = BIT0},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0x3F133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0x3F134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0x3F13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0x3F780,
	.FLASH_CMD_ADDR            = 0x3F783,
	.FLASH_CMD_ISSUE_ADDR      = 0x3F78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0x3F78F,
	.GCM_FLAG_ADDR             = 0x3F793,
};

static const struct nvt_ts_mem_map NT51920_2chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x30500,
	.RAW_PIPE0_ADDR            = 0x32E40, // 0x31E80
	.RAW_PIPE1_ADDR            = 0x32E40,
	.BASELINE_ADDR             = 0x33E00,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x34DC0,
	.DIFF_PIPE1_ADDR           = 0x35D80,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x24000,
	.RW_FLASH_DATA_ADDR        = 0x24002,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x3B6BC,
	.MMAP_HISTORY_EVENT1       = 0x3B6FC,
	/* Cascade */
	.ENB_CASC_REG              = {.addr = 0x3F02C, .bitmask = BIT0},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0x3F133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0x3F134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0x3F13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0x3F780,
	.FLASH_CMD_ADDR            = 0x3F783,
	.FLASH_CMD_ISSUE_ADDR      = 0x3F78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0x3F78F,
	.GCM_FLAG_ADDR             = 0x3F793,
};

static const struct nvt_ts_mem_map NT51922_1chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x2F000,
	.RAW_PIPE0_ADDR            = 0x301F8,
	.RAW_PIPE1_ADDR            = 0x301F8,
	.BASELINE_ADDR             = 0x30828,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x30E58,
	.DIFF_PIPE1_ADDR           = 0x31488,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x24000,
	.RW_FLASH_DATA_ADDR        = 0x24002,
	/* Cascade */
	.ENB_CASC_REG              = {.addr = 0x3F02C, .bitmask = BIT0},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0x3F133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0x3F134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0x3F13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0x3F780,
	.FLASH_CMD_ADDR            = 0x3F783,
	.FLASH_CMD_ISSUE_ADDR      = 0x3F78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0x3F78F,
	.GCM_FLAG_ADDR             = 0x3F793,
};

static const struct nvt_ts_mem_map NT51922_2chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x2F000,
	.RAW_PIPE0_ADDR            = 0x308A0,
	.RAW_PIPE1_ADDR            = 0x308A0,
	.BASELINE_ADDR             = 0x317F0,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x33180,
	.DIFF_PIPE1_ADDR           = 0x340D0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x24000,
	.RW_FLASH_DATA_ADDR        = 0x24002,
	/* Cascade */
	.ENB_CASC_REG              = {.addr = 0x3F02C, .bitmask = BIT0},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0x3F133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0x3F134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0x3F13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0x3F780,
	.FLASH_CMD_ADDR            = 0x3F783,
	.FLASH_CMD_ISSUE_ADDR      = 0x3F78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0x3F78F,
	.GCM_FLAG_ADDR             = 0x3F793,
};

static const struct nvt_ts_mem_map NT51923_1chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x94000,
	.RAW_PIPE0_ADDR            = 0x84130,
	.RAW_PIPE1_ADDR            = 0x84130,
	.BASELINE_ADDR             = 0x85810,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x86EF0,
	.DIFF_PIPE1_ADDR           = 0x885D0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x941CC,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9ACA0,
	.MMAP_HISTORY_EVENT1       = 0x9ACE0,
	/* Cascade */
	.SDLOC_REG                 = {.addr = 0xFF02C, .bitmask = (BIT2|BIT1|BIT0)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51923_2chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x94000,
	.RAW_PIPE0_ADDR            = 0x84130,
	.RAW_PIPE1_ADDR            = 0x84130,
	.BASELINE_ADDR             = 0x85810,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x86EF0,
	.DIFF_PIPE1_ADDR           = 0x885D0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x941CC,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9ACA0,
	.MMAP_HISTORY_EVENT1       = 0x9ACE0,
	/* Cascade */
	.SDLOC_REG                 = {.addr = 0xFF02C, .bitmask = (BIT2|BIT1|BIT0)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51923_3chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x94000,
	.RAW_PIPE0_ADDR            = 0x84130,
	.RAW_PIPE1_ADDR            = 0x84130,
	.BASELINE_ADDR             = 0x85810,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x86EF0,
	.DIFF_PIPE1_ADDR           = 0x885D0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x941CC,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9ACA0,
	.MMAP_HISTORY_EVENT1       = 0x9ACE0,
	/* Cascade */
	.SDLOC_REG                 = {.addr = 0xFF02C, .bitmask = (BIT2|BIT1|BIT0)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51925_1chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x91000,
	.RAW_PIPE0_ADDR            = 0x92208,
	.RAW_PIPE1_ADDR            = 0x92208,
	.BASELINE_ADDR             = 0x92838,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x92E68,
	.DIFF_PIPE1_ADDR           = 0x93498,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* Cascade */
	.SDLOC_REG                 = {.addr = 0xFF02C, .bitmask = (BIT2|BIT1|BIT0)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51925_2chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x91000,
	.RAW_PIPE0_ADDR            = 0x928C0,
	.RAW_PIPE1_ADDR            = 0x928C0,
	.BASELINE_ADDR             = 0x93810,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x951E0,
	.DIFF_PIPE1_ADDR           = 0x96130,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* Cascade */
	.SDLOC_REG                 = {.addr = 0xFF02C, .bitmask = (BIT2|BIT1|BIT0)},
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51926_1chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x96A00,
	.RAW_PIPE0_ADDR            = 0x82E20, // 0x81EE0
	.RAW_PIPE1_ADDR            = 0x82E20,
	.BASELINE_ADDR             = 0x83D60,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x84CA0,
	.DIFF_PIPE1_ADDR           = 0x85BE0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x97B9C,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9BCA0,
	.MMAP_HISTORY_EVENT1       = 0x9BCE0,
	/* Cascade */
	.IC_NUM_REG                = {.addr = 0xFF02C, .bitmask = (BIT3|BIT2)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51926_2chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x96A00,
	.RAW_PIPE0_ADDR            = 0x82E20, // 0x81EE0
	.RAW_PIPE1_ADDR            = 0x82E20,
	.BASELINE_ADDR             = 0x83D60,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x84CA0,
	.DIFF_PIPE1_ADDR           = 0x85BE0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x97B9C,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9BCA0,
	.MMAP_HISTORY_EVENT1       = 0x9BCE0,
	/* Cascade */
	.IC_NUM_REG                = {.addr = 0xFF02C, .bitmask = (BIT3|BIT2)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

static const struct nvt_ts_mem_map NT51926_3chip_memory_map = {
	.EVENT_BUF_ADDR            = 0x96A00,
	.RAW_PIPE0_ADDR            = 0x82E20,
	.RAW_PIPE1_ADDR            = 0x82E20,
	.BASELINE_ADDR             = 0x83D60,
	.BASELINE_BTN_ADDR         = 0,
	.DIFF_PIPE0_ADDR           = 0x84CA0,
	.DIFF_PIPE1_ADDR           = 0x85BE0,
	.RAW_BTN_PIPE0_ADDR        = 0,
	.RAW_BTN_PIPE1_ADDR        = 0,
	.DIFF_BTN_PIPE0_ADDR       = 0,
	.DIFF_BTN_PIPE1_ADDR       = 0,
	.READ_FLASH_CHECKSUM_ADDR  = 0x84000,
	.RW_FLASH_DATA_ADDR        = 0x84002,
	/* FW multi-task */
	.COMMON_BUF_ADDR           = 0x97B9C,
	/* FW History */
	.MMAP_HISTORY_EVENT0       = 0x9BCA0,
	.MMAP_HISTORY_EVENT1       = 0x9BCE0,
	/* Cascade */
	.IC_NUM_REG                = {.addr = 0xFF02C, .bitmask = (BIT3|BIT2)},
	/* BLD CRC */
	.CRC_DONE_REG              = {.addr = 0xFF133, .bitmask = BIT2},
	.TP_FLASH_GRANT_REG        = {.addr = 0xFF134, .bitmask = BIT3},
	.WP_TP_REG                 = {.addr = 0xFF13B, .bitmask = BIT0},
	/* GCM FW update */
	.GCM_CODE_ADDR             = 0xFF780,
	.FLASH_CMD_ADDR            = 0xFF783,
	.FLASH_CMD_ISSUE_ADDR      = 0xFF78E,
	.FLASH_CKSUM_STATUS_ADDR   = 0xFF78F,
	.GCM_FLAG_ADDR             = 0xFF793,
};

// Refer to Common_FW/ap/ap_fwconfig.c
// ST_PUB_FW_CONFIG
static struct nvt_ts_hw_info NT51900_hw_info = {
	.cascade_type                 = CASCADE_NONE,
	.cascade_num                  = NONE_CASCADE_CASE,

	.default_x_num                = 40,
	.default_y_num                = 24,
	.default_abs_x_max            = 1280,
	.default_abs_y_max            = 720,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51920_1chip_hw_info = {
	.cascade_type                 = CASCADE_ENB_CASC,
	.cascade_num                  = CASCADE_1CHIP,

	.default_x_num                = 48,
	.default_y_num                = 20,
	.default_abs_x_max            = 960,
	.default_abs_y_max            = 540,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51920_2chip_hw_info = {
	.cascade_type                 = CASCADE_ENB_CASC,
	.cascade_num                  = CASCADE_2CHIP,

	.default_x_num                = 56,
	.default_y_num                = 21,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1080,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51922_1chip_hw_info = {
	.cascade_type                 = CASCADE_ENB_CASC,
	.cascade_num                  = CASCADE_1CHIP,

	.default_x_num                = 36,
	.default_y_num                = 22,
	.default_abs_x_max            = 1440,
	.default_abs_y_max            = 900,
	.default_max_button_num	= 0,
};

static struct nvt_ts_hw_info NT51922_2chip_hw_info = {
	.cascade_type                 = CASCADE_ENB_CASC,
	.cascade_num                  = CASCADE_2CHIP,

	.default_x_num                = 56,
	.default_y_num                = 34,
	.default_abs_x_max            = 1920,
	.default_abs_y_max            = 1200,
	.default_max_button_num	= 0,
};

static struct nvt_ts_hw_info NT51923_1chip_hw_info = {
	.cascade_type                 = CASCADE_SDLOC,
	.cascade_num                  = CASCADE_1CHIP,

	.default_x_num                = 24,
	.default_y_num                = 60,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1080,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51923_2chip_hw_info = {
	.cascade_type                 = CASCADE_SDLOC,
	.cascade_num                  = CASCADE_2CHIP,

	.default_x_num                = 48,
	.default_y_num                = 60,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1080,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51923_3chip_hw_info = {
	.cascade_type                 = CASCADE_SDLOC,
	.cascade_num                  = CASCADE_3CHIP,

	.default_x_num                = 72,
	.default_y_num                = 40,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1620,
	.default_max_button_num       = 0,
};

static struct nvt_ts_hw_info NT51925_1chip_hw_info = {
	.cascade_type			= CASCADE_NT51925_SDLOC,
	.cascade_num			= CASCADE_1CHIP,

	.default_x_num			= 36,
	.default_y_num			= 22,
	.default_abs_x_max		= 1440,
	.default_abs_y_max		= 900,
	.default_max_button_num	= 0,
};

static struct nvt_ts_hw_info NT51925_2chip_hw_info = {
	.cascade_type			= CASCADE_NT51925_SDLOC,
	.cascade_num			= CASCADE_2CHIP,

	.default_x_num			= 56,
	.default_y_num			= 34,
	.default_abs_x_max		= 1920,
	.default_abs_y_max		= 1200,
	.default_max_button_num	= 0,
};

static struct nvt_ts_hw_info NT51926_1chip_hw_info = {
	.cascade_type                 = CASCADE_IC_NUM,
	.cascade_num                  = CASCADE_1CHIP,
	.default_x_num                = 16,
	.default_y_num                = 60,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1080,
	.default_max_button_num       = 0,
};
static struct nvt_ts_hw_info NT51926_2chip_hw_info = {
	.cascade_type                 = CASCADE_IC_NUM,
	.cascade_num                  = CASCADE_2CHIP,
	.default_x_num                = 32,
	.default_y_num                = 60,
	.default_abs_x_max            = 1768,
	.default_abs_y_max            = 828,
	.default_max_button_num       = 0,
};
static struct nvt_ts_hw_info NT51926_3chip_hw_info = {
	.cascade_type                 = CASCADE_IC_NUM,
	.cascade_num                  = CASCADE_3CHIP,
	.default_x_num                = 48,
	.default_y_num                = 60,
	.default_abs_x_max            = 2880,
	.default_abs_y_max            = 1080,
	.default_max_button_num       = 0,
};
#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	const struct nvt_ts_hw_info *hwinfo;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{.id = {0x00, 0x00, 0x00, 0x00, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51900_memory_map,  .hwinfo = &NT51900_hw_info},
	{.id = {0x00, 0x00, 0x01, 0x20, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51920_1chip_memory_map,  .hwinfo = &NT51920_1chip_hw_info},
	{.id = {0x00, 0x00, 0x01, 0x20, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51920_2chip_memory_map,  .hwinfo = &NT51920_2chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x22, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51922_1chip_memory_map,  .hwinfo = &NT51922_1chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x22, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51922_2chip_memory_map,  .hwinfo = &NT51922_2chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x23, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51923_1chip_memory_map,  .hwinfo = &NT51923_1chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x23, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51923_2chip_memory_map,  .hwinfo = &NT51923_2chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x23, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51923_3chip_memory_map,  .hwinfo = &NT51923_3chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x25, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51925_1chip_memory_map,  .hwinfo = &NT51925_1chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x25, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51925_2chip_memory_map,  .hwinfo = &NT51925_2chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x26, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51926_1chip_memory_map,  .hwinfo = &NT51926_1chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x26, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51926_2chip_memory_map,  .hwinfo = &NT51926_2chip_hw_info},
	{.id = {0x00, 0x00, 0x00, 0x26, 0x19, 0x05}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT51926_3chip_memory_map,  .hwinfo = &NT51926_3chip_hw_info},
};
