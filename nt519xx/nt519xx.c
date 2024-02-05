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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#if defined(CONFIG_FB)
#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#endif
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt519xx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#if NVT_TOUCH_FRAME_COUNTER
#include <linux/jiffies.h>
#endif

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer;
uint8_t esd_check;
uint8_t esd_retry;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
static struct delayed_work nvt_frame_counter_work;
static struct workqueue_struct *nvt_frame_counter_wq;
static unsigned long irq_timer;
bool frame_counter_check;
uint8_t frame_counter_retry;
uint16_t frame_counter;
#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
extern void nvt_extra_proc_deinit(void);
#endif /* #if NVT_TOUCH_EXT_PROC */

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
extern void nvt_mp_proc_deinit(void);
#endif /* #if NVT_TOUCH_MP */

#if NVT_OSD_TRIGGER
extern int32_t nvt_dp_proc_init(void);
extern void nvt_dp_proc_deinit(void);
#endif /* NVT_OSD_TRIGGER */

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif /* #if BOOT_UPDATE_FIRMWARE */

#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#else
static int nvt_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif /* #if TOUCH_KEY_NUM > 0 */

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif /* #if WAKEUP_GESTURE */

static uint8_t bTouchIsAwake;

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	struct irq_desc *desc;

	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}

	desc = irq_to_desc(ts->client->irq);
	NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	memcpy(buf + 1, ts->xbuf, len - 1);

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	memcpy(ts->xbuf, buf, len);
	msg.buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	return CTP_I2C_WRITE(ts->client, i2c_addr, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[2] = {0};

	/* Set xdata index */
	ret = nvt_set_page(I2C_FW_Address, addr);
	if (ret < 0) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	/* Write data to index */
	buf[0] = addr & 0xFF;
	buf[1] = data;
	ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}
	ret = 0;

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen read value to specific register.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_read_reg(nvt_ts_reg_t reg, uint8_t *val)
{
	int32_t ret = 0;
	uint32_t addr = 0;
	uint8_t bitmask = 0;
	uint8_t shift = 0;
	uint8_t buf[8] = {0};
	uint8_t temp = 0;

	addr = reg.addr;
	bitmask = reg.bitmask;
	/* Get shift */
	temp = reg.bitmask;
	shift = 0;
	while (1) {
		if ((temp >> shift) & 0x01)
			break;
		if (shift == 8) {
			NVT_ERR("bitmask all bits zero!\n");
			ret = -1;
			break;
		}
		shift++;
	}
	/* Read the byte of the register is in */
	nvt_set_page(I2C_FW_Address, addr);
	buf[0] = addr & 0xFF;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed!(%d)\n", ret);
		goto nvt_read_register_exit;
	}
	/* Get register's value in its field of the byte */
	*val = (buf[1] & bitmask) >> shift;

nvt_read_register_exit:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen write value to specific register.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_reg(nvt_ts_reg_t reg, uint8_t val)
{
	int32_t ret = 0;
	uint32_t addr = 0;
	uint8_t bitmask = 0;
	uint8_t shift = 0;
	uint8_t buf[8] = {0};
	uint8_t temp = 0;

	addr = reg.addr;
	bitmask = reg.bitmask;
	/* Get shift */
	temp = reg.bitmask;
	shift = 0;
	while (1) {
		if ((temp >> shift) & 0x01)
			break;
		if (shift == 8) {
			NVT_ERR("bitmask all bits zero!\n");
			break;
		}
		shift++;
	}
	/* Read the byte including this register */
	nvt_set_page(I2C_FW_Address, addr);
	buf[0] = addr & 0xFF;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed!(%d)\n", ret);
		goto nvt_write_register_exit;
	}
	/* Set register's value in its field of the byte */
	temp = buf[1] & (~bitmask);
	temp |= ((val << shift) & bitmask);
	/* Write back the whole byte including this register */
	buf[0] = addr & 0xFF;
	buf[1] = temp;
	ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_WRITE failed!(%d)\n", ret);
		goto nvt_write_register_exit;
	}

nvt_write_register_exit:
	return ret;
}

#if CHANGE_I2C_DEVICE_ADDR
/*******************************************************
Description:
	Novatek touchscreen initiate flash block function
return:
	n.a.
*******************************************************/
void nvt_init_flash_block(uint8_t address)
{
	uint8_t buf[3] = {0};

	NVT_LOG("start\n");

	/* Initiate Flash Block */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = address;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 3);

	msleep(5);

	NVT_LOG("end\n");
}
#endif /* #if CHANGE_I2C_DEVICE_ADDR */

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
	function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	/* MCU idle cmds to SWRST_SIF_ADDR */
	nvt_write_addr(SWRST_SIF_ADDR, 0xAA);

	msleep(15);

#if CHANGE_I2C_DEVICE_ADDR
	nvt_init_flash_block(I2C_FW_Address);
#endif /* #if CHANGE_I2C_DEVICE_ADDR */

	nvt_clear_crc_err_flag();
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	NVT_LOG("start\n");
	
	/* Reset cmds to SWRST_SIF_ADDR */
	nvt_write_addr(SWRST_SIF_ADDR, 0x69);

	/* Need 35ms delay after bootloader reset */
	msleep(35);

#if CHANGE_I2C_DEVICE_ADDR
	nvt_init_flash_block(I2C_FW_Address);
#endif /* #if CHANGE_I2C_DEVICE_ADDR */

	NVT_LOG("end\n");
}

/*******************************************************
Description:
  Novatek touchscreen wait bootloader done function.

return:
    n.a.
*******************************************************/
int32_t nvt_wait_bootloader_done(void)
{
	uint8_t crc_done = 0;
	int32_t ret = 0;
	int32_t retry = 0;

	NVT_LOG("start\n");
	
	/* Wait until ILM & DLM crc done */
	while (1) {
		nvt_set_page(I2C_FW_Address, ts->mmap->CRC_DONE_REG.addr);
		nvt_read_reg(ts->mmap->CRC_DONE_REG, &crc_done);

		if (crc_done)
			break;
		else
			retry++;

		if (retry > 150) {
			NVT_ERR("%s failed! crc_done = 0x%02X\n", __func__, crc_done);
			return -1;
		}
		msleep(1);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		/* Set xdata index to EVENT BUF ADDR */
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Clear fw status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		/* Read fw status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		/* Set xdata index to EVENT BUF ADDR */
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Read fw status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);

	while (1) {
		usleep_range(10000, 10000);

		/* Read reset state */
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if (unlikely (retry > 100)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check cascade numbers.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
int32_t nvt_check_cascade_chip_numbers(void)
{
	uint8_t enb_casc = 0;
	uint8_t sdloc = 0;
	uint8_t ic_num = 0;
	int32_t ret = 0;

	if (ts->cascade_type) {
		if (ts->cascade_type == CASCADE_ENB_CASC) {

			if (!ts->mmap->ENB_CASC_REG.addr) {
				NVT_ERR("ENB_CASC_REG.addr is missing\n");
				return (-EIO);
			}

			nvt_set_page(I2C_FW_Address, ts->mmap->ENB_CASC_REG.addr);
			nvt_read_reg(ts->mmap->ENB_CASC_REG, &enb_casc);

			if (enb_casc == ENB_CASC_1CHIP)
				ts->cascade_num = CASCADE_1CHIP;
			else
				ts->cascade_num = CASCADE_2CHIP;

		} else if (ts->cascade_type == CASCADE_SDLOC) {

			if (!ts->mmap->SDLOC_REG.addr) {
				NVT_ERR("SDLOC_REG.addr is missing\n");
				return (-EIO);
			}

			nvt_set_page(I2C_FW_Address, ts->mmap->SDLOC_REG.addr);
			nvt_read_reg(ts->mmap->SDLOC_REG, &sdloc);

			switch (sdloc) {
			case SDLOC_1CHIP:
				ts->cascade_num = CASCADE_1CHIP;
				break;
			case SDLOC_2CHIP:
				ts->cascade_num = CASCADE_2CHIP;
				break;
			case SDLOC_3CHIP_AND_ABOVE:
				ts->cascade_num = CASCADE_3CHIP;
				break;
			default:
				ts->cascade_num = CASCADE_CHECK_ERR;
				NVT_ERR("Undefined CASCADE_TYPE_SDLOC: 0x%X\n", sdloc);
				ret = (-EIO);
				break;
			}
		} else if (ts->cascade_type == CASCADE_NT51925_SDLOC) {

			if (!ts->mmap->SDLOC_REG.addr) {
				NVT_ERR("SDLOC_REG.addr is missing\n");
				return (-EIO);
			}

			nvt_set_page(I2C_FW_Address, ts->mmap->SDLOC_REG.addr);
			nvt_read_reg(ts->mmap->SDLOC_REG, &sdloc);

			switch (sdloc) {
			case NT51925_SDLOC_1CHIP:
				ts->cascade_num = CASCADE_1CHIP;
				break;
			case NT51925_SDLOC_2CHIP:
				ts->cascade_num = CASCADE_2CHIP;
				break;
			case NT51925_SDLOC_3CHIP_AND_ABOVE:
				ts->cascade_num = CASCADE_3CHIP;
				break;
			default:
				ts->cascade_num = CASCADE_CHECK_ERR;
				NVT_ERR("Undefined CASCADE_TYPE_NT51925_SDLOC: 0x%X\n", sdloc);
				ret = (-EIO);
				break;
			}
		} else if (ts->cascade_type == CASCADE_IC_NUM) {

			if (!ts->mmap->IC_NUM_REG.addr) {
				NVT_ERR("IC_NUM_REG.addr is missing\n");
				return (-EIO);
			}

			nvt_set_page(I2C_FW_Address, ts->mmap->IC_NUM_REG.addr);
			nvt_read_reg(ts->mmap->IC_NUM_REG, &ic_num);

			switch (ic_num) {
			case IC_NUM_1CHIP:
				ts->cascade_num = CASCADE_1CHIP;
				break;
			case IC_NUM_2CHIP:
				ts->cascade_num = CASCADE_2CHIP;
				break;
			case IC_NUM_3CHIP:
			case IC_NUM_3CHIP_AND_ABOVE:
				ts->cascade_num = CASCADE_3CHIP;
				break;
			default:
				ts->cascade_num = CASCADE_CHECK_ERR;
				NVT_ERR("Undefined CASCADE_TYPE_IC_NUM: 0x%X\n", ic_num);
				ret = (-EIO);
				break;
			}
		}
	} else {
		ts->cascade_num = NONE_CASCADE_CASE;
		NVT_LOG("CASCADE_NONE cascade_type: %d, cascade_num: %d\n",
			ts->cascade_type, ts->cascade_num);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[40] = {0};
	uint8_t len = sizeof(buf)/sizeof(uint8_t);
	uint8_t retry_count = 2;
	int32_t ret = 0;

info_retry:
	/* Set xdata index to EVENT BUF ADDR */
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	/* Read fw info */
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, len);
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			ts->fw_ver = 0;
			ts->max_button_num = TOUCH_KEY_NUM;
			NVT_ERR("Set default fw_ver=0x%02X, max_button_num=%d\n", 
					ts->fw_ver, ts->max_button_num);
			ret = -1;
			goto out;
		}
	}
	ts->fw_ver         = buf[1];
	ts->x_num          = buf[3];
	ts->y_num          = buf[4];
	ts->max_button_num = buf[11];
	ts->evtbuf_fmt     = buf[13];
	ts->cust_func      = buf[14]; /* Customized Function */
	ts->combuf_report  = buf[23]; /* 0x01: FW Distributed System */
	ts->nvt_pid = (uint16_t)((buf[36] << 8) | buf[35]);

	NVT_LOG("fw_ver=0x%02X, x_num=%0d, y_num=%0d, evtbuf_fmt=0x%02X, "
		"cust_func=0x%02X, combuf_report=0x%02X, PID=0x%04X\n", ts->fw_ver,
			ts->x_num, ts->y_num, ts->evtbuf_fmt, ts->cust_func, ts->combuf_report,
				ts->nvt_pid);

	ret = 0;
out:
	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str)) {
		NVT_ERR("error count=%zu\n", count);
		return -EFAULT;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
	NVT_LOG("cancel frame-counter delayed work sync\n");
	cancel_delayed_work_sync(&nvt_frame_counter_work);
	nvt_frame_counter_enable(false);
#endif

	i2c_wr = str[0] >> 7;

	if (i2c_wr == NVTWRITE) { //I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == NVTREAD) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		/* Copy buff to user if i2c transfer */
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct proc_ops nvt_flash_fops = {
	.proc_open = nvt_flash_open,
	.proc_release = nvt_flash_close,
	.proc_read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL, &nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/%s\n", DEVICE_NAME);
	NVT_LOG("============================================================\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash deinitial function.

return:
	n.a.
*******************************************************/
static void nvt_flash_proc_deinit(void)
{
	if (NVT_proc_entry != NULL) {
		remove_proc_entry(DEVICE_NAME, NULL);
		NVT_proc_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", DEVICE_NAME);
	}
}
#endif /* #if NVT_TOUCH_PROC */

#if WAKEUP_GESTURE
#define GESTURE_WORD_C          12 // 0x0C
#define GESTURE_WORD_W          13 // 0x0D
#define GESTURE_WORD_V          14 // 0x0E
#define GESTURE_DOUBLE_CLICK    15 // 0x0F
#define GESTURE_WORD_Z          16 // 0x10
#define GESTURE_WORD_M          17 // 0x11
#define GESTURE_WORD_O          18 // 0x12
#define GESTURE_WORD_e          19 // 0x13
#define GESTURE_WORD_S          20 // 0x14
#define GESTURE_SLIDE_UP        21 // 0x15
#define GESTURE_SLIDE_DOWN      22 // 0x16
#define GESTURE_SLIDE_LEFT      23 // 0x17
#define GESTURE_SLIDE_RIGHT     24 // 0x18
/* Customized gesture id */
#define DATA_PROTOCOL           15 // ID[7:4] = 0x0F means special command

/* Function page definition */
#define FUNCPAGE_GESTURE         1 // LPWKG

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[3];
	uint8_t func_id = data[4];

	/* Support FW special data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
	case GESTURE_WORD_C:
		NVT_LOG("Gesture : Word-C.\n");
		keycode = gesture_key_array[0];
		break;
	case GESTURE_WORD_W:
		NVT_LOG("Gesture : Word-W.\n");
		keycode = gesture_key_array[1];
		break;
	case GESTURE_WORD_V:
		NVT_LOG("Gesture : Word-V.\n");
		keycode = gesture_key_array[2];
		break;
	case GESTURE_DOUBLE_CLICK:
		NVT_LOG("Gesture : Double Click.\n");
		keycode = gesture_key_array[3];
		break;
	case GESTURE_WORD_Z:
		NVT_LOG("Gesture : Word-Z.\n");
		keycode = gesture_key_array[4];
		break;
	case GESTURE_WORD_M:
		NVT_LOG("Gesture : Word-M.\n");
		keycode = gesture_key_array[5];
		break;
	case GESTURE_WORD_O:
		NVT_LOG("Gesture : Word-O.\n");
		keycode = gesture_key_array[6];
		break;
	case GESTURE_WORD_e:
		NVT_LOG("Gesture : Word-e.\n");
		keycode = gesture_key_array[7];
		break;
	case GESTURE_WORD_S:
		NVT_LOG("Gesture : Word-S.\n");
		keycode = gesture_key_array[8];
		break;
	case GESTURE_SLIDE_UP:
		NVT_LOG("Gesture : Slide UP.\n");
		keycode = gesture_key_array[9];
		break;
	case GESTURE_SLIDE_DOWN:
		NVT_LOG("Gesture : Slide DOWN.\n");
		keycode = gesture_key_array[10];
		break;
	case GESTURE_SLIDE_LEFT:
		NVT_LOG("Gesture : Slide LEFT.\n");
		keycode = gesture_key_array[11];
		break;
	case GESTURE_SLIDE_RIGHT:
		NVT_LOG("Gesture : Slide RIGHT.\n");
		keycode = gesture_key_array[12];
		break;
	default:
		NVT_ERR("Unkown Gesture ID: %d.\n", gesture_id);
		break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif /* #if WAKEUP_GESTURE */

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static void nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);
#if NVT_TOUCH_ASIL
	ts->abd_gpio = of_get_named_gpio_flags(np, "novatek,abd-gpio", 0, &ts->abd_flags);
	NVT_LOG("novatek,abd-gpio=%d\n", ts->abd_gpio);
#endif
}
#else
static void nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* Request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* Request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

#if NVT_TOUCH_ASIL
	/* request ABD-pin (Input) */
	if (gpio_is_valid(ts->abd_gpio)) {
		ret = gpio_request_one(ts->abd_gpio, GPIOF_IN, "NVT-abd");
		if (ret) {
			NVT_ERR("Failed to request NVT-abd GPIO\n");
			goto err_request_abd_gpio;
		}
	}
#endif

	return ret;

#if NVT_TOUCH_ASIL
err_request_abd_gpio:
	gpio_free(ts->abd_gpio);
#endif
err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_ASIL
	if (gpio_is_valid(ts->abd_gpio))
		gpio_free(ts->abd_gpio);
#endif
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

#if NVT_TOUCH_ESD_PROTECT
static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* Check heartbeat pattern */
	for (i = 2; i < 7; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

void nvt_esd_check_enable(uint8_t enable)
{
	/* Update interrupt timer */
	irq_timer = jiffies;
	/* Clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* Enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_ERR("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* Do esd recovery, bootloader reset */
		nvt_bootloader_reset();
		mutex_unlock(&ts->lock);
		/* Update interrupt timer */
		irq_timer = jiffies;
		/* Update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
void nvt_frame_counter_enable(uint8_t enable)
{
	irq_timer = jiffies;
	frame_counter_retry = enable ? 0 : frame_counter_retry;
	frame_counter_check = enable;
}

static void nvt_frame_counter_func(struct work_struct *work)
{
	unsigned long timer = jiffies_to_msecs(jiffies - irq_timer);
	uint8_t buf[3] = {0x00, 0x00, 0x00};
	uint16_t tmp = 0;
	int ret = 0;

	//NVT_LOG("%s\n", (frame_counter_check)? "start":"skip Frame Counter check");

	mutex_lock(&ts->lock);

	if (timer > (NVT_TOUCH_ESD_CHECK_PERIOD * 3)) {
		/*
		 * If the IRQ handler has stopped for a period of time,
		 * the frame-counter checking process should be waked up.
		 */
		nvt_frame_counter_enable(true);
	}

	if (frame_counter_check) {
		/* Check the value of Frame-Counter */
		buf[0] = EVENT_MAP_FRAME_COUNTER & 0xFF;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);
		if (ret < 0) {
			NVT_ERR("CTP_I2C_READ 0x%02X failed.(%d)\n", buf[0], ret);

		} else {
			tmp = (uint16_t)(buf[2] << 8) + (uint16_t)(buf[1]);

			if (frame_counter == tmp) {
				frame_counter_retry++;
				NVT_ERR("Frame Counter: %u (frame_counter_retry = %d)\n",
					frame_counter, frame_counter_retry);
			} else {
				frame_counter = tmp;
				if (frame_counter_retry)
					frame_counter_retry--;
			}
			//NVT_LOG("FW frame-counter: %u\n", frame_counter);

			if (frame_counter_retry > 2) {
				/* Do reboot-reset if the value of Frame-Counter doen't changed */
				NVT_ERR("do ESD recovery\n");
				nvt_bootloader_reset();
				nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);
				nvt_frame_counter_enable(true);
			}
		}
	}

	mutex_unlock(&ts->lock);

	queue_delayed_work(nvt_frame_counter_wq, &nvt_frame_counter_work,
		msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif

void nvt_read_fw_history(uint32_t fw_history_addr)
{
	uint8_t i = 0;
	uint8_t buf[65];
	char str[128];

	if (fw_history_addr == 0)
		return;

	nvt_set_page(I2C_FW_Address, fw_history_addr);

	buf[0] = (uint8_t) (fw_history_addr & 0xFF);
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 64+1);	//read 64bytes history

	//print all data
	NVT_LOG("fw history 0x%X: \n", fw_history_addr);
	for (i = 0; i < 4; i++) {
		snprintf(str, sizeof(str),
				"%02X %02X %02X %02X %02X %02X %02X %02X  "
				"%02X %02X %02X %02X %02X %02X %02X %02X\n",
				buf[1+i*16], buf[2+i*16], buf[3+i*16], buf[4+i*16],
				buf[5+i*16], buf[6+i*16], buf[7+i*16], buf[8+i*16],
				buf[9+i*16], buf[10+i*16], buf[11+i*16], buf[12+i*16],
				buf[13+i*16], buf[14+i*16], buf[15+i*16], buf[16+i*16]);
		NVT_LOG("%s", str);
	}

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);
}

#if NVT_TOUCH_EVENT_CRC8
static uint8_t nvt_calculate_crc8(uint8_t *buf, uint8_t length)
{
	uint8_t i = 0, j = 0;
	uint8_t CRC8 = 0;
	uint8_t poly = 0x1D; // x8 + x4 + x3 + x2 + 1

	for (i = 0; i < length; i++) {
		CRC8 ^=  *(buf + i);

		for (j = 0; j < 8; j++) {
			if (CRC8 & 0x80) {
				CRC8 = (CRC8 << 1) ^ poly;
			} else {
				CRC8 = (CRC8 << 1);
			}
 		}
	}

	CRC8 ^= (uint8_t)0x00;
	return CRC8;
}

static int32_t nvt_ts_event_crc8(uint8_t *buf, uint8_t length)
{
	uint8_t CRC8 = nvt_calculate_crc8(buf, length - 1);
	//uint32_t i = 0;

	/* The lastest item is CRC8 which is reported by FW */
	if (CRC8 != buf[length - 1]) {
		NVT_ERR("i2c/spi packet CRC8 not match, (length = %d): "
			"FW reported: 0x%02X, Calulated CRC8: 0x%02X\n",
				length, buf[length - 1], CRC8);
#if 0
		NVT_ERR("length = %d\n", length);
		for (i = 0; i < length; i++) {
			printk("%02X ", buf[i]);
			if (i % POINT_DATA_LEN == 0)
				printk("\n");
		}
		printk("\n");
#endif
		return -1;
	}
	return CRC8;
}
#endif /* NVT_TOUCH_EVENT_CRC8 */

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
#if (!NVT_TOUCH_EVENT_CRC8)
	uint8_t touch_event[DUMMY_BYTES + TOUCH_EVENT_LEN] = {0};
#else
	uint8_t touch_event[DUMMY_BYTES + TOUCH_EVENT_CRC_LEN] = {0};
#endif /* (!NVT_TOUCH_EVENT_CRC8) */
	uint32_t event_size = sizeof(touch_event)/sizeof(uint8_t);
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, touch_event, event_size);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
#if CHANGE_I2C_DEVICE_ADDR
	nvt_init_flash_block(I2C_FW_Address);
#endif /* #if CHANGE_I2C_DEVICE_ADDR */
		goto XFER_ERROR;
	}

#if 0
	/* Dump event buf */
	printk("touch event buffer length: %d\n", event_size);
	for (i = DUMMY_BYTES; i < event_size; i++) {
		printk("%02X ", touch_event[i]);
		if (i % POINT_DATA_LEN == 0)
			printk("\n");
	}
	printk("\n");
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_fw_recovery(touch_event)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_EVENT_CRC8
	/* Calculate CRC8 without dummy bytes */
	ret = nvt_ts_event_crc8(touch_event + DUMMY_BYTES, event_size - DUMMY_BYTES);
	if (ret < 0) {
		goto XFER_ERROR;
	}
#endif /* NVT_TOUCH_EVENT_CRC8 */

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		input_id = (uint8_t)(touch_event[2] >> 4);
		nvt_ts_wakeup_gesture_report(input_id, touch_event);
		mutex_unlock(&ts->lock);
		return IRQ_HANDLED;
	}
#endif

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = DUMMY_BYTES + ASIL_FLAG_LEN + POINT_DATA_LEN * i;
		input_id = (uint8_t)(touch_event[position] >> 4);

		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		/* finger down (enter or moving) */
		if (((touch_event[position] & 0x07) == 0x01) ||
			((touch_event[position] & 0x07) == 0x02)) {
#if NVT_TOUCH_ESD_PROTECT
			/* Update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#if NVT_TOUCH_FRAME_COUNTER
			nvt_frame_counter_enable(false);
			irq_timer = jiffies;
#endif
			input_x = (uint32_t)(touch_event[position + 1] << 8) + (uint32_t)(touch_event[position + 2]);
			input_y = (uint32_t)(touch_event[position + 3] << 8) + (uint32_t)(touch_event[position + 4]);

			if ((input_x > TOUCH_MAX_WIDTH) || (input_y > TOUCH_MAX_HEIGHT))
				continue;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (touch_event[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#if NVT_TOUCH_FRAME_COUNTER
		nvt_frame_counter_enable(false);
		irq_timer = jiffies;
#endif
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((touch_event[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

XFER_ERROR:
	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

#if NVT_TOUCH_ASIL
#define ASIL_OPEN_SHORT_DATA_LEN 20
/*******************************************************
Description:
	Novatek touchscreen ASIL notifier function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_asil_func(int irq, void *data)
{
	uint8_t dp_err_flag = 0, tp_err_flag = 0;
	uint8_t open_err_flag = 0, short_err_flag = 0;
	uint8_t buf[DUMMY_BYTES + ASIL_OPEN_SHORT_DATA_LEN] = {0x00};
	int32_t i = 0;
	int32_t ret = 0;

	mutex_lock(&ts->lock);

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);

	/* ASIL error detect flags */
	buf[0] = 0x00;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ (0x2F000) failed.(%d)\n", ret);
		goto err_nvt_ts_asil_func;
	}

	NVT_ERR("ASIL error flag: 0x%02X\n", buf[1]);
	dp_err_flag = ((buf[1] >> 5) & 0x01);
	tp_err_flag = ((buf[1] >> 4) & 0x01);

	/* open/short error flags */
	buf[0] = 0x49;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ (0x2F049) failed.(%d)\n", ret);
		goto err_nvt_ts_asil_func;
	}

	NVT_ERR("ASIL open/short error flag: 0x%02X\n", buf[1]);
	open_err_flag = ((buf[1] >> 3) & 0x01);
	short_err_flag = ((buf[1] >> 4) & 0x01);

	if (tp_err_flag && (open_err_flag || short_err_flag)) {

		memset(buf, 0, sizeof(buf));

		/* open/short fail coordinates */
		buf[0] = 0xA0;
		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, sizeof(buf)/sizeof(uint8_t));
		if (ret < 0) {
			NVT_ERR("CTP_I2C_READ (0x2F0A0) failed.(%d)\n", ret);
			goto err_nvt_ts_asil_func;
		}

		for (i = DUMMY_BYTES; i < ASIL_OPEN_SHORT_DATA_LEN; i += 2) {
			if ((buf[i] != 0xFF) && (buf[i + 1] != 0xFF)) {
				NVT_ERR("%s fail pad %d (%d, %d), Point: %d\n",
					(i < (ASIL_OPEN_SHORT_DATA_LEN / 2)) ? "Open":"Short", ((i % 10) / 2) + 1,
						buf[i], buf[i + 1], (buf[i + 1] * ts->x_num) + buf[i]);
			}
		}
	}

err_nvt_ts_asil_func:
	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen clear crc err flag.

return:
	n.a.
*******************************************************/
int32_t nvt_clear_crc_err_flag(void)
{
	uint8_t buf[2] = {0x00};
	int ret = 0;

	nvt_set_page(I2C_FW_Address, (uint32_t)BLD_SPE_PUPS_ADDR);

	buf[0] = (uint32_t)BLD_SPE_PUPS_ADDR & 0xFF;
	buf[1] = 0xA5;
	ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_LOG("write 0x%02X to BLD_SPE_PUPS (0x%X) failed (%d)\n",
			(uint32_t)BLD_SPE_PUPS_ADDR, buf[1], ret);
		return ret;
	}

	buf[0] = (uint32_t)BLD_SPE_PUPS_ADDR & 0xFF;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	if (ret < 0) {
		NVT_LOG("read BLD_SPE_PUPS (0x%X) failed (%d)\n",
			(uint32_t)BLD_SPE_PUPS_ADDR, ret);
		return ret;
	}

	NVT_LOG("BLD_SPE_PUPS (0x%X) = 0x%02X\n",
		(uint32_t)BLD_SPE_PUPS_ADDR, buf[1]);

	if (buf[1] != 0xA5)
		return (-EIO);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check and stop crc reboot loop.

return:
	n.a.
*******************************************************/
void nvt_stop_crc_reboot(void)
{
	uint8_t buf[4] = {0};
	int32_t retry = 0;

	/* Read dummy buffer to check CRC fail reboot is happening or not */

	/* Change I2C index to prevent geting 0xFF, but not 0xFC */
	nvt_set_page(I2C_FW_Address, (uint32_t)CHIP_VER_TRIM_ADDR);

	/* Read to check if buf is 0xFC which means IC is in CRC reboot */
	buf[0] = (uint32_t)CHIP_VER_TRIM_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 4);

	if (((buf[1] == 0xFB) && (buf[2] == 0xFB) && (buf[3] == 0xFB)) ||
		((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
		((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

		/* IC is in CRC fail reboot loop, needs to be stopped! */
		for (retry = 5; retry > 0; retry--) {

			/* Reset idle : 1st */
			nvt_sw_reset_idle();

			/* Reset idle : 2rd */
			nvt_sw_reset_idle();
			msleep(1);

			if (nvt_clear_crc_err_flag() >= 0)
				break;
		}
		if (retry == 0)
			NVT_ERR("CRC auto reboot is not able to be stopped! buf[1] = 0x%02X\n", buf[1]);
	}

	return;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nvt_bootloader_reset(); // NOT in retry loop

	/* Check for 5 times */
	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		nvt_set_page(I2C_FW_Address, (uint32_t)CHIP_VER_TRIM_ADDR);

		buf[0] = ((uint32_t)CHIP_VER_TRIM_ADDR & 0xFF);
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 7);
		NVT_LOG("EXT_CHIP_ID_RD (0x%X) = 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
			(uint32_t)CHIP_VER_TRIM_ADDR, buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		/* Stop CRC check to prevent IC auto reboot */
		if (((buf[1] == 0xFB) && (buf[2] == 0xFB) && (buf[3] == 0xFB)) ||
			((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
			continue;
		}

		/* Compare read chip id on supported list */
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			/* Compare each byte */
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				ts->mmap = trim_id_table[list].mmap;
				ts->cascade_type = trim_id_table[list].hwinfo->cascade_type;

				ret = nvt_check_cascade_chip_numbers();
				if (ret < 0)
					NVT_ERR("Get cascade chip numbers failed (%d)\n", ret);

				/* Find out the memory map of target chip numbers */
				if (ts->cascade_num == trim_id_table[list].hwinfo->cascade_num) {
					if (ts->cascade_type == CASCADE_ENB_CASC) {
						NVT_LOG("ENB_CASC_REG.addr (0x%X), cascade_type = %d, cascade_num = %d\n",
							ts->mmap->ENB_CASC_REG.addr, ts->cascade_type, ts->cascade_num);
					} else if ((ts->cascade_type == CASCADE_SDLOC) || (ts->cascade_type == CASCADE_NT51925_SDLOC)) {
						NVT_LOG("SDLOC_REG.addr (0x%X), cascade_type = %d, cascade_num = %d\n",
							ts->mmap->SDLOC_REG.addr, ts->cascade_type, ts->cascade_num);
					} else if (ts->cascade_type == CASCADE_IC_NUM) {
						NVT_LOG("IC_NUM_REG.addr (0x%X), cascade_type = %d, cascade_num = %d\n",
							ts->mmap->IC_NUM_REG.addr, ts->cascade_type, ts->cascade_num);
					} else {
						NVT_LOG("Non cascade case, cascade_type = %d, cascade_num = %d\n",
							ts->cascade_type, ts->cascade_num);
					}
					NVT_LOG("trim_id_table[%d] is matched!\n", list);
					NVT_LOG("This is NVT touch IC\n");
					goto out;
				}
			}
		}
		msleep(10);
	}

	ts->mmap = NULL;
	ret = -1;

out:
#if DRIVER_FORCE_PROBE
	/* Set a deafult memory map for driver froce probe */
	if (ret < 0) {
		ts->mmap = &(DEFAULT_MMAP);
		ts->cascade_type = (DEFAULT_HW_INFO).cascade_type;
		ts->cascade_num = (DEFAULT_HW_INFO).cascade_num;
		ts->force_probe = true;
	} else {
		ts->force_probe = false;
	}
#endif /* DRIVER_FORCE_PROBE */
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif

	ts = (struct nvt_ts_data *)kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc(NVT_XBUF_LEN, GFP_KERNEL);
	if (ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* Parse dts */
	nvt_parse_dt(&client->dev);

	/* Request and config GPIOs */
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	/* Need 10ms delay after POR(power on reset) */
	msleep(10);

#if DRIVER_FORCE_PROBE
    ts->force_probe = false;
#endif

	/* Check chip version trim */
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("This chip is not identified in mapping table\n");
		ret = -EINVAL;
#if (!DRIVER_FORCE_PROBE)
		goto err_chipvertrim_failed;
#endif /* (!DRIVER_FORCE_PROBE) */
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();

	/* Allocate input device */
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
#if (!DRIVER_FORCE_PROBE)
		goto err_input_dev_alloc_failed;
#endif /* (!DRIVER_FORCE_PROBE) */
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;
#if NVT_TOUCH_ASIL
	ts->abd_trigger_type = ABD_TRIGGER_TYPE;
#endif
	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_HEIGHT, 0, 0);
#if MT_PROTOCOL_B
	/* No need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it */
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	/* Register input device */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	/* Set int-pin & request irq */
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_I2C_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			nvt_irq_enable(false);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

#if NVT_TOUCH_ASIL
	/* Set abd-pin & request irq */
	ts->abd_irq = gpio_to_irq(ts->abd_gpio);
	if (ts->abd_irq) {
		NVT_LOG("abd_trigger_type=%d\n", ts->abd_trigger_type);

		ret = request_threaded_irq(ts->abd_irq, NULL, nvt_ts_asil_func,
				ts->abd_trigger_type | IRQF_ONESHOT, NVT_ASIL_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request abd-irq failed. ret=%d\n", ret);
			goto err_abd_request_failed;
		} else {
			disable_irq(ts->abd_irq);
			NVT_LOG("request abd-irq %d succeed\n", ts->abd_irq);
		}
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}

	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);

	/* Please make sure boot update start after display reset(RESX) sequence */
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);

	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}

	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
	if (ts->cust_func & BIT0) {
		/* BIT0: 1: Frame-Counter enable, 0: Frame-Counter disalbe */
		NVT_LOG("NVT_TOUCH_FRAME_COUNTER is enabled\n");

		nvt_frame_counter_wq = alloc_workqueue("nvt_frame_counter_wq", WQ_MEM_RECLAIM, 1);
		if (!nvt_frame_counter_wq) {
			NVT_ERR("nvt_frame_counter_wq create workqueue failed\n");
			ret = -ENOMEM;
			goto err_create_nvt_frame_counter_wq_failed;
		}

		INIT_DELAYED_WORK(&nvt_frame_counter_work, nvt_frame_counter_func);

		queue_delayed_work(nvt_frame_counter_wq, &nvt_frame_counter_work,
				msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
	}
#endif

	/* Set device node */
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_flash_proc_init_failed;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_extra_proc_init_failed;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_mp_proc_init_failed;
	}
#endif

#if NVT_OSD_TRIGGER
	ret = nvt_dp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt dp proc init failed. ret=%d\n", ret);
		goto err_dp_proc_init_failed;
	}
#endif /* NVT_OSD_TRIGGER */
#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	ts->drm_notif.notifier_call = nvt_drm_notifier_callback;
	ret = msm_drm_register_client(&ts->drm_notif);
	if (ret) {
		NVT_ERR("register drm_notifier failed. ret=%d\n", ret);
		goto err_register_drm_notif_failed;
	}
#else
	ts->fb_notif.notifier_call = nvt_fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts->early_suspend);
	if (ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

	bTouchIsAwake = 1;
	NVT_LOG("end\n");

#if DRIVER_FORCE_PROBE
	NVT_LOG("force_probe: %s\n", ts->force_probe ? "true" : "false");
#endif /* DRIVER_FORCE_PROBE */

	nvt_irq_enable(true);

#if NVT_TOUCH_ASIL
	enable_irq(ts->abd_irq);
#endif
	return 0;

#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
err_register_drm_notif_failed:
#else
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
err_register_fb_notif_failed:
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
err_register_early_suspend_failed:
#endif
#if NVT_OSD_TRIGGER
	nvt_dp_proc_deinit();
err_dp_proc_init_failed:
#endif /* NVT_OSD_TRIGGER */
#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
err_mp_proc_init_failed:
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
err_extra_proc_init_failed:
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
err_flash_proc_init_failed:
#endif
#if NVT_TOUCH_FRAME_COUNTER
	if (nvt_frame_counter_wq) {
		cancel_delayed_work_sync(&nvt_frame_counter_work);
		destroy_workqueue(nvt_frame_counter_wq);
		nvt_frame_counter_wq = NULL;
	}
err_create_nvt_frame_counter_wq_failed:
#endif
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
err_create_nvt_esd_check_wq_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
#if NVT_TOUCH_ASIL
	free_irq(ts->abd_irq, ts);
err_abd_request_failed:
#endif
	free_irq(client->irq, ts);
err_int_request_failed:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
#if (!DRIVER_FORCE_PROBE)
err_input_dev_alloc_failed:
err_chipvertrim_failed:
#endif /* (!DRIVER_FORCE_PROBE) */
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
err_check_functionality_failed:
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_remove(struct i2c_client *client)
{
	NVT_LOG("Removing driver...\n");

#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#else
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#if NVT_OSD_TRIGGER
	nvt_dp_proc_deinit();
#endif /* NVT_OSD_TRIGGER */
#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif

#if NVT_TOUCH_FRAME_COUNTER
	if (nvt_frame_counter_wq) {
		cancel_delayed_work_sync(&nvt_frame_counter_work);
		nvt_frame_counter_enable(false);
		destroy_workqueue(nvt_frame_counter_wq);
		nvt_frame_counter_wq = NULL;
	}
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);
#if NVT_TOUCH_ASIL
	free_irq(ts->abd_irq, ts);
#endif

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	i2c_set_clientdata(client, NULL);

	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}

	if (ts) {
		kfree(ts);
		ts = NULL;
	}
}

static void nvt_ts_shutdown(struct i2c_client *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#else
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#if NVT_OSD_TRIGGER
	nvt_dp_proc_deinit();
#endif /* NVT_OSD_TRIGGER */
#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
	if (nvt_frame_counter_wq) {
		cancel_delayed_work_sync(&nvt_frame_counter_work);
		nvt_frame_counter_enable(false);
		destroy_workqueue(nvt_frame_counter_wq);
		nvt_frame_counter_wq = NULL;
	}
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

#if (defined(CONFIG_FB) && defined(_MSM_DRM_NOTIFY_H_)) || \
	defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(false);
#endif

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("cancel delayed work sync\n");
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_FRAME_COUNTER
	NVT_LOG("cancel frame-counter delayed work sync\n");
	cancel_delayed_work_sync(&nvt_frame_counter_work);
	nvt_frame_counter_enable(false);
#endif

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

#if WAKEUP_GESTURE
	/* Write command to enter "wakeup gesture mode" */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	enable_irq_wake(ts->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	/* Write command to enter "deep sleep mode" */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif // WAKEUP_GESTURE

	mutex_unlock(&ts->lock);

	/* Release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	NVT_LOG("end\n");

	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	/* Please make sure display reset(RESX) sequence and mipi dsi cmds sent before this */
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	if (nvt_check_fw_reset_state(RESET_STATE_REK)) {
		NVT_ERR("FW is not ready! Try to bootloader reset...\n");
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_REK);
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(true);
#endif

#if NVT_TOUCH_FRAME_COUNTER
	nvt_frame_counter_enable(true);
	queue_delayed_work(nvt_frame_counter_wq, &nvt_frame_counter_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return 0;
}


#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, drm_notif);

	if (!evdata || (evdata->id != 0))
		return 0;

	if (evdata->data && ts) {
		blank = evdata->data;
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_POWERDOWN) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				nvt_ts_suspend(&ts->client->dev);
			}
		} else if (event == MSM_DRM_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_UNBLANK) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				nvt_ts_resume(&ts->client->dev);
			}
		}
	}

	return 0;
}
#else
static int nvt_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
			nvt_ts_resume(&ts->client->dev);
		}
	}

	return 0;
}
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts",},
	{ },
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

#if NVT_TOUCH_ESD_PROTECT
	irq_timer = 0;
	esd_check = false;
	esd_retry = 0;
#endif

#if NVT_TOUCH_FRAME_COUNTER
	irq_timer = 0;
	frame_counter_check = false;
	frame_counter_retry = 0;
#endif

	bTouchIsAwake = 0;

	/* Add I2C driver */
	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		NVT_ERR("failed to add i2c driver");
		goto err_driver;
	}

	NVT_LOG("finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);
}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
 