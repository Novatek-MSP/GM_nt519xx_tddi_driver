/*
 * Copyright (C) 2021 Novatek, Inc.
 *
 * $Revision: 106205 $
 * $Date: 2022-09-27 18:16:34 +0800 (週二, 27 九月 2022) $
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

#define SIZE_4KB 4096
#define SIZE_8KB 8192
#define SIZE_64KB 65536
#define SIZE_192KB 196608
#define SIZE_256KB 262144
#define SIZE_1792KB 1835008

/* flash manufacturer idenfication */
typedef enum {
	FLASH_MFR_UNKNOWN		= 0x00,
	FLASH_MFR_ESMT			= 0x1C,
	FLASH_MFR_MACRONIX		= 0xC2,	/* Macronix */
	FLASH_MFR_GIGADEVICE	= 0xC8,	/* GigaDevice */
	FLASH_MFR_WINBOND		= 0xEF,	/* Winbond */
	FLASH_MFR_MAX			= 0xFF
} FLASH_MFR_t;

#define FLASH_DID_ALL 0xFFFF	/* all device id of same manufacture */

typedef enum {
	QEB_POS_UNKNOWN = 0,
	QEB_POS_SR_1B,		/* QE bit in SR 1st byte */
	QEB_POS_OTHER,		/* QE bit not in SR 1st byte */
	QEB_POS_MAX
} QEB_POS_t;

typedef struct flash_qeb_info {
	QEB_POS_t qeb_pos;		/* QE bit position type, ex. in SR 1st/2nd byte, etc. */
	uint8_t qeb_order;		/* in which bit of that byte, start from bit 0 */
} FLASH_QEB_INFO_t;

typedef enum {
	FLASH_READ_METHOD_UNKNOWN = 0,
	SISO_0x03,
	SISO_0x0B,
	SIQO_0x6B,
	QIQO_0xEB,
	FLASH_READ_METHOD_MAX
} FLASH_READ_METHOD_t;

typedef enum {
	FLASH_PROG_METHOD_UNKNOWN = 0,
	SPP_0x02,						/* SingalPageProgram_0x02 */
	QPP_0x32,						/* QuadPageProgram_0x32 */
	QPP_0x38,						/* QuadPageProgram_0x38 */
	FLASH_PROG_METHOD_MAX
} FLASH_PROG_METHOD_t;

typedef enum {
	FLASH_WRSR_METHOD_UNKNOWN = 0,
	WRSR_01H1BYTE,					/* 01H (S7-S0) */
	WRSR_01H2BYTE,					/* 01H (S7-S0) (S15-S8) */
	FLASH_WRSR_METHOD_MAX
} FLASH_WRSR_METHOD_t;

/* Flash partition information */
typedef enum {
	FLASH_PARTITION_TP  = 0x000000,
	FLASH_PARTITION_DP  = 0x03E000,
	FLASH_PARTITION_OSD = 0x040000,
	FLASH_PARTITION_MAX = 0x200000,
} FLASH_PARTITION_t;

typedef enum {
	BIN_START_ADDR_TP   = FLASH_PARTITION_TP,
	BIN_START_ADDR_DP   = FLASH_PARTITION_DP,
	BIN_START_ADDR_OSD  = FLASH_PARTITION_OSD,
	BIN_START_ADDR_MAX,
} FW_BIN_START_ADDR_t;

typedef enum {
	BIN_SIZE_TP	  	= SIZE_192KB,
	BIN_SIZE_DP   	= SIZE_4KB, // Do not overwrite 0x3F000~0x3FFFF to protect gama, vcom, etc (original: SIZE_8KB)
	BIN_SIZE_TP_DP	= SIZE_256KB,
	BIN_SIZE_OSD  	= SIZE_1792KB,
	BIN_SIZE_MAX,
} FW_BIN_SIZE_t;

typedef struct flash_info {
	FLASH_MFR_t mid;					/* manufacturer identification */
	uint16_t did;						/* 2 bytes device identification read by 9Fh cmd*/
	FLASH_QEB_INFO_t qeb_info;
	FLASH_READ_METHOD_t rd_method;		/* flash read method */
	FLASH_PROG_METHOD_t prog_method;	/* flash program method */
	FLASH_WRSR_METHOD_t wrsr_method;	/* write status register method */
	uint8_t rdsr1_cmd;					/* cmd for read status register-1 (S15-S8) */
	/* Flash FW update */
	uint32_t partition_tp;
	uint32_t bin_start_addr_tp;
	uint32_t bin_size_tp;
	uint32_t partition_dp;
	uint32_t bin_start_addr_dp;
	uint32_t bin_size_dp;
	uint32_t partition_osd;
	uint32_t bin_start_addr_osd;
	uint32_t bin_size_osd;
} flash_info_t;

static const flash_info_t flash_info_table[] = {
	/* Please put flash info items which will use quad mode and is verified before those with "did = FLASH_DID_ALL"! */

	/* Please note that flash info item which have the same info for all flash did of the same mid should be place after this line!
	 * Please make sure that the info item have the same info for all flash did of the same mid then you can add it in the section!
	 */
	{.mid = FLASH_MFR_GIGADEVICE, .did = FLASH_DID_ALL, .qeb_info = {.qeb_pos = QEB_POS_OTHER, .qeb_order = 0xFF},
		.rd_method = SISO_0x03, .prog_method = SPP_0x02, .wrsr_method = WRSR_01H2BYTE, .rdsr1_cmd = 0x35},
	{.mid = FLASH_MFR_MACRONIX,   .did = FLASH_DID_ALL, .qeb_info = {.qeb_pos = QEB_POS_SR_1B, .qeb_order = 6},
		.rd_method = SISO_0x03, .prog_method = SPP_0x02, .wrsr_method = WRSR_01H1BYTE, .rdsr1_cmd = 0xFF},
	{.mid = FLASH_MFR_WINBOND,    .did = FLASH_DID_ALL, .qeb_info = {.qeb_pos = QEB_POS_OTHER, .qeb_order = 0xFF},
		.rd_method = SISO_0x03, .prog_method = SPP_0x02, .wrsr_method = WRSR_01H1BYTE, .rdsr1_cmd = 0x35},
	/* Please note that the following flash info item should be keep at the last one! Do not move it! */
	{.mid = FLASH_MFR_UNKNOWN,    .did = FLASH_DID_ALL, .qeb_info = {.qeb_pos = QEB_POS_UNKNOWN, .qeb_order = 0xFF},
		.rd_method = SISO_0x03, .prog_method = SPP_0x02, .wrsr_method = FLASH_WRSR_METHOD_UNKNOWN, .rdsr1_cmd = 0xFF}
};
