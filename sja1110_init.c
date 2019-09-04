/*
* Switch driver module for SJA1110
* Copyright (C) 2019 NXP Semiconductors
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

/**
*
* \file  sja1110_init.c
*
* \author Marco Hartmann
*
* \date 2019-07-09
*
* \brief SJA1110 SPI driver
*
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "sja1110_init.h"


/*******************************************************************************
 * Global variables
 ******************************************************************************/
static const u32 device_id_list[] = {SJA1110_VAL_DEVICEID};
#define NUM_DEVICE_IDS ARRAY_SIZE(device_id_list)

struct sja1110_state g_state;


/*******************************************************************************
 * Module Parameter
 ******************************************************************************/
static char *firmware_name = DEF_FNAME_UC;
module_param(firmware_name, charp, S_IRUGO);
MODULE_PARM_DESC(firmware_name,
	"Name of the SJA1110 µC firmware to be loaded");

static char *config_name =  DEF_FNAME_SWITCH;
module_param(config_name, charp, S_IRUGO);
MODULE_PARM_DESC(config_name,
	"Name of the SJA1110 switch configuration to be loaded");

static int max_spi_speed = 10000000;
module_param(max_spi_speed, int, S_IRUGO);
MODULE_PARM_DESC(max_spi_speed, "Max SPI frequency in Hz");

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/
static void dump_msg(const char *data, int len)
{
	int i;
	for (i=0; i<len; i++)
		pr_alert("0x%x\n", data[i]);
}

/**
 * Read a given register of the switch
 * sja1110->devtype needs to be SJA1110_SWITCH
 */
static u32 sja1110_read_reg(struct sja1110_priv *sja1110, u32 reg_addr)
{
	int ret;
	u32 cmd[2], resp[2];

	BUG_ON(sja1110->devtype != SJA1110_SWITCH);

	cmd[0] = cpu_to_le32(CMD_ENCODE_RWOP(CMD_RD_OP) |
			     CMD_ENCODE_ADDR(reg_addr) |
			     CMD_ENCODE_WRD_CNT(0));
	cmd[1] = 0;

	ret = sja1110_spi_transfer(sja1110, (u8*)cmd, (u8*)resp, 8, 8);
	if (ret)
		dev_err(&sja1110->spi->dev,
			"Reading register failed with %d\n", ret);

	return resp[1];
}

/**
 * Write a given value to a given register of the switch
 * sja1110->devtype needs to be SJA1110_SWITCH
 */
static u32 sja1110_write_reg(struct sja1110_priv *sja1110, u32 reg_addr, u32 val)
{
	int ret;
	u32 cmd[2];

	BUG_ON(sja1110->devtype != SJA1110_SWITCH);

	cmd[0] = cpu_to_le32(CMD_ENCODE_RWOP(CMD_WR_OP) |
			     CMD_ENCODE_ADDR(reg_addr));
	cmd[1] = val;

	ret = sja1110_spi_transfer(sja1110, (u8*)cmd, NULL, 8, 8);
	if (ret)
		dev_err(&sja1110->spi->dev,
			"Writing register failed with %d\n", ret);

	return ret;
}

/* Retrieve GPIO number from the device tree and request it */
static int get_and_request_gpio(struct sja1110_priv *sja1110)
{
	struct device_node *node;
	int rst_gpio = -1, ret = -ENODEV;

	node = sja1110->spi->dev.of_node;
	if (!node)
		goto out;

	of_property_read_u32(node, "reset-gpio", &rst_gpio);
	if (!gpio_is_valid(rst_gpio))
		goto out;

	if (gpio_request(rst_gpio, "SJA1110 soft reset") != 0) {
		dev_err(&sja1110->spi->dev,
			"GPIO request failed (already requested?)\n");
		goto out;
	}

	if (gpio_direction_output(rst_gpio, 1) != 0) {
		dev_err(&sja1110->spi->dev,
			"GPIO direction could not be set\n");
		gpio_free(rst_gpio);
		goto out;
	}

	ret = rst_gpio;

out:
	return ret;
}

/**
 * Check if a given value is a valid device id.
 *
 * Returns index into device_id_list[] array if valid,
 * returns -1 if value is not a valid device id
 */
static int sja1110_check_device_id(u32 reg_val)
{
	int dev_id_idx;

	for (dev_id_idx = 0; dev_id_idx < NUM_DEVICE_IDS; dev_id_idx++)
		if (reg_val == device_id_list[dev_id_idx])
			break;

	if (dev_id_idx >= NUM_DEVICE_IDS)
		return -1;

	return dev_id_idx;
}


/*******************************************************************************
 * Switch specific handlers
 ******************************************************************************/
static int sja1110_pre_switch_upload(struct sja1110_priv *sja1110,
				     const u8 *cfg_data, int cfg_size)
{
	int dev_id_idx, ret = -EINVAL;
	u32 registerValue, cfg_device_id;

	/* read the device id */
	registerValue = sja1110_read_reg(sja1110, D_ID_ADDR);
	if (sja1110_check_device_id(registerValue) < 0) {
		dev_err(&sja1110->spi->dev,
			"Device id 0x%x is not supported!\n", registerValue);
		goto out;
	}

	if(!cfg_data || cfg_size == 0) {
		dev_err(&sja1110->spi->dev, "No switch config loaded\n");
		goto out;
	}

	if (cfg_size % 4) {
		dev_err(&sja1110->spi->dev, "Invalid config file size\n");
		goto out;
	}
	
	/* check if dev id matches the one in the cfg file */
	cfg_device_id = *((u32*)cfg_data);
	if (cfg_device_id != device_id_list[dev_id_idx]) {
		dev_err(&sja1110->spi->dev,
			"Device id (0x%x) does not match that of the static config (0x%x)\n",
			device_id_list[dev_id_idx], cfg_device_id);
		goto out;
	}

	/* firmware ok */
	ret = 0;
	dev_info(&sja1110->spi->dev,
		"[%s] Found switch config of size %d\n", __func__, cfg_size);

out:
	return ret;
}

/**
 * Do a soft reset of the switch using a reset pin
 * sja1110->devtype needs to be SJA1110_SWITCH
 */
static int sja1110_reset_gpio(struct sja1110_priv *sja1110)
{
	int us = RESET_DELAY_US;
	int rst_gpio;

	BUG_ON(sja1110->devtype != SJA1110_SWITCH);
	rst_gpio = sja1110->switch_priv->rst_gpio;
	if (rst_gpio < 0)
		return rst_gpio;

	gpio_set_value_cansleep(rst_gpio, 0);
	usleep_range(us, us + DIV_ROUND_UP(us, 10));
	gpio_set_value_cansleep(rst_gpio, 1);

	return 0;
}

/**
 * Do a cold reset of the switch using SPI
 * sja1110->devtype needs to be SJA1110_SWITCH
 */
static int sja1110_reset_spi(struct sja1110_priv *sja1110)
{
	BUG_ON(sja1110->devtype != SJA1110_SWITCH);

	return sja1110_write_reg(sja1110, R_CTRL_ADDR, RESET_CTRL_COLDRESET);
}

/**
 * Do a reset of the switch, either via a reset pin or via SPI
 * sja1110->devtype needs to be SJA1110_SWITCH
 */
static int sja1110_reset(struct sja1110_priv *sja1110)
{
	int ret, count = 0;

	BUG_ON(sja1110->devtype != SJA1110_SWITCH);
	if (sja1110->switch_priv->rst_gpio > 0)
		ret = sja1110_reset_gpio(sja1110);
	else
		ret = sja1110_reset_spi(sja1110);

	if (ret)
		goto out;

	/* wait until device is up again */
	ret = -ETIMEDOUT;
	while (count++ < 30) {
		u32 registerValue = sja1110_read_reg(sja1110, D_ID_ADDR);
		if (sja1110_check_device_id(registerValue) >= 0) {
			ret = 0;
			break;
		}
	}

out:
	return ret;
}

static int sja1110_switch_upload(struct sja1110_priv *sja1110,
				 const u8 *cfg_data, int cfg_size)
{
	int i, ret = 0, page = 0;
	const int block_len = 4;
	const int tx_len = block_len + 4;


	ret = sja1110_reset(sja1110);
	if (ret) {
		dev_err(&sja1110->spi->dev, "reset failed\n");
		goto out;
	}

	/* power down watchdog clock */
	sja1110_write_reg(sja1110, CGU_OUTCLK_C_WATCHDOG, 0x5000001);

	dev_info(&sja1110->spi->dev, "Uploading config...\n");
	for (i = cfg_size; i > 0; i -= block_len) {
		u32 cmd[2];
		const u8 *buf_ptr = &cfg_data[page * block_len];

		cmd[0] = CMD_ENCODE_RWOP(CMD_WR_OP) |
			 CMD_ENCODE_ADDR(CONFIG_START_ADDRESS + page);
		cmd[1] = *((u32*)buf_ptr);

		ret = sja1110_simple_upload(sja1110, (u8*)cmd, tx_len, tx_len);
		if (ret) {
			dev_err(&sja1110->spi->dev, "Could not send config!\n");
			goto out;
		}
		page++;
	}

out:
	return ret;
}

static int sja1110_post_switch_upload(struct sja1110_priv *sja1110,
				      const u8 *unused, int unused2)
{
	int ret = 0;
	u32 registerValue, configs, crcchkl, ids, crcchkg;

	/* read and parse the configuration_flags register */
	registerValue = sja1110_read_reg(sja1110, CONFIGURATIONFLAG_ADDR);
	configs = !!(registerValue & CF_CONFIGS_MASK);
	crcchkl = !!(registerValue & CF_CRCCHKL_MASK);
	ids     = !!(registerValue & CF_IDS_MASK);
	crcchkg = !!(registerValue & CF_CRCCHKG_MASK);

	if (configs) {
		dev_info(&sja1110->spi->dev, "Successfully configured!\n");
	} else {
		dev_err(&sja1110->spi->dev,
			"Configuration failed: LocalCRCfail=%d,DevIDunmatched=%d,GlobalCRCfail=%d\n",
			crcchkl, ids, crcchkg);
		ret = -EINVAL;
	}

	return ret;
}


/*******************************************************************************
 * µC specific handlers
 ******************************************************************************/
/**
 * Parse a 4-byte-long boot status message
 * sja1110->devtype needs to be SJA1110_UC
 */
static int sja1110_uc_parse_status(struct sja1110_priv *sja1110,
				   struct sja1110_uc_status_pkt *uc_status,
				   const u8 *status_resp)
{
	int ret = 0;

	BUG_ON(sja1110->devtype != SJA1110_UC);

	/* parse the response */
	if (status_resp[0] != STATUS_PKT_HEADER) {
		dev_err(&sja1110->spi->dev,
			"[%s] Invalid status header (0x%x)\n",
			__func__,
			status_resp[0]);
		ret = -EINVAL;
		goto out;
	}

	uc_status->status   = status_resp[1];
	uc_status->err_code = status_resp[2];
	uc_status->attempt  = status_resp[3];

	// dev_info(&sja1110->spi->dev,
	// 	"[%s] Status=0x%x, Error=0x%x, Attempt=0x%x\n",
	// 	__func__,
	// 	uc_status->status,
	// 	uc_status->err_code,
	// 	uc_status->attempt);

out:
	return ret;
}

/**
 * Parse an SPI RX buffer of length len,
 * containing n = floor(len / 4) boot status messages.
 * sja1110->devtype needs to be SJA1110_UC.
 * 
 * Returns 0 if no boot messages indicates an error
 * or boot state other than DOWNLOADING,
 * returns the respective error code otherwise.
 */
static int sja1110_uc_parse_status_many(struct sja1110_priv *sja1110,
					const u8 *buf, size_t len)
{
	int pos, ret = 0;
	struct sja1110_uc_status_pkt uc_status;

	BUG_ON(sja1110->devtype != SJA1110_UC);

	/**
	 * 2 bytes of the first boot msg were already transmitted when sending 
	 * the 2-byte fw header. Start parsing at position 2 in order to skip
	 * the first boot message (of length 4).
	 * Iterate until len-3 in order to skip a potential incomplete last
	 * boot message (len does not need to be a multiple of 4)
	 */
	for (pos = 2; pos < len - 3; pos += 4) {
		ret = sja1110_uc_parse_status(sja1110, &uc_status, buf + pos);
		if (ret) {
			dev_err(&sja1110->spi->dev,
				"[%s] Parsing uC status message %d failed with %d\n",
				__func__, pos / 4, ret);
			goto out;
		}

		if (uc_status.status != DOWNLOADING ||
		    uc_status.err_code != NO_ERROR) {
			dev_err(&sja1110->spi->dev,
				"[%s] Upload error detected in status message %d (status=0x%x,err=0x%x)\n",
				__func__, pos / 4,
				uc_status.status,uc_status.err_code);
			ret = uc_status.err_code;
			goto out;
		}
	}

out:
	return ret;
}

/**
 * Transfer 4 dummy bytes and parse the response of the uC.
 * Response contains status information and error codes.
 * sja1110->devtype needs to be SJA1110_UC
 */
static int sja1110_uc_read_status(struct sja1110_priv *sja1110,
				  struct sja1110_uc_status_pkt *uc_status)
{
	int ret;
	u8 dummy_bytes[4] = {0};
	u8 status_resp[4];

	BUG_ON(sja1110->devtype != SJA1110_UC);

	ret = sja1110_spi_transfer(sja1110, dummy_bytes, status_resp, 4, 4);
	if (ret) {
		dev_err(&sja1110->spi->dev,
			"[%s] Reading uC status failed with %d\n",
			__func__, ret);
		goto out;
	}

	ret = sja1110_uc_parse_status(sja1110, uc_status, status_resp);
	if (ret) {
		dev_err(&sja1110->spi->dev,
			"[%s] Parsing uC status failed with %d\n",
			__func__, ret);
		goto out;
	}

out:
	return ret;
}

static int sja1110_pre_uc_upload(struct sja1110_priv *sja1110,
				 const u8 *fw_data, int fw_size)
{
	int i, retry;
	int ret = 0;
	const u8 fw_header[] = IMAGE_VALID_MARKER;
	struct sja1110_uc_status_pkt uc_status = {};

	/* TODO: sanity check the loaded binary */
	dev_info(&sja1110->spi->dev,
		"[%s] Found firmware of size %d\n", __func__, fw_size);

	if(!fw_data || fw_size == 0) {
		dev_err(&sja1110->spi->dev, "No firmware loaded\n");
		ret = -ENOENT;
		goto out;
	}
	
	/* check header of the binary */
	i = 0;
	while (i < ARRAY_SIZE(fw_header) && fw_data[i] == fw_header[i])
		i++;

	if (i < ARRAY_SIZE(fw_header)) {
		dev_err(&sja1110->spi->dev, "Invalid firmware header\n");
		ret = -EINVAL;
		goto out;
	}

	/* enable watchdog clock using the switch spi device */
	if (g_state.sja1110[SJA1110_SWITCH])
		sja1110_write_reg(g_state.sja1110[SJA1110_SWITCH],
				  CGU_OUTCLK_C_WATCHDOG, 0x5000000);

	/* firmware ok */
	dev_info(&sja1110->spi->dev,
		"[%s] firmware appears to be valid\n", __func__);

	/* Verify that µC is ready to receive data */
	retry = 10;
	while (retry > 0) {
		ret = sja1110_uc_read_status(sja1110, &uc_status);
		if (ret) {
			dev_err(&sja1110->spi->dev, 
				"[%s] Could not read µC status\n", __func__);
		} else if (uc_status.status == WAITING) {
			break;
		} else {
			dev_err(&sja1110->spi->dev,
				"[%s] µC not ready for download (Status=0x%x)\n",
				__func__, uc_status.status);
			ret = -EBUSY;
		}
		retry--;
	}

out:
	return ret;
}

static int sja1110_uc_upload(struct sja1110_priv *sja1110,
			     const u8 *fw_data, int fw_size)
{
	int ret = 0;
	u8 *rx_buf;
	u8 header_buf[] = HEADER_EXEC;

	rx_buf = kzalloc(fw_size, GFP_KERNEL);
	if (!rx_buf) {
		dev_err(&sja1110->spi->dev, "Could not allocate SPI RX buf\n");
		ret = -ENOMEM;
		goto out;
	}

	/* send the header */
	ret = sja1110_simple_upload(sja1110, header_buf,
				    ARRAY_SIZE(header_buf),
				    ARRAY_SIZE(header_buf));
	if (ret) {
		dev_err(&sja1110->spi->dev, "Could not send fw header!\n");
		goto free_out;
	}

	/* send the firmware */
	dev_info(&sja1110->spi->dev, "Uploading firmware...\n");
	ret = sja1110_spi_transfer(sja1110, fw_data, rx_buf, fw_size, fw_size);
	if (ret) {
		dev_err(&sja1110->spi->dev, "Could not send fw!\n");
		goto free_out;
	}

	/* parse the RX buffer to detect any upload errors */
	ret = sja1110_uc_parse_status_many(sja1110, rx_buf, fw_size);
	if (ret) {
		dev_err(&sja1110->spi->dev, "Upload error detected!\n");
		goto free_out;
	}
	dev_info(&sja1110->spi->dev, "Upload successfully verified!\n");

free_out:
	kfree(rx_buf);
out:
	return ret;
}

static int sja1110_post_uc_upload(struct sja1110_priv *sja1110,
				  const u8 *fw_data, int fw_size)
{
	return 0;
}


/*******************************************************************************
 * Generic hw init
 ******************************************************************************/

/**
 * This function uploads a given binary to an SPI device.
 * spi_pagesize indicates the number of bytes per SPI transfer
 */
static int sja1110_simple_upload(struct sja1110_priv *sja1110,
				 const u8 *bin_data, int bin_size,
				 int spi_pagesize)
{
	return sja1110_spi_transfer(sja1110, bin_data, NULL,
				    bin_size, spi_pagesize);
}


static int sja1110_spi_transfer(struct sja1110_priv *sja1110, const u8 *tx_data,
				u8 *rx_data, int size, int spi_pagesize)
{
	int ret = 0, page = 0, i;
	struct spi_message msg;
	struct spi_transfer xfer_buf = {0};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_buf, &msg);

	/* send binary */
	for (i = size; i > 0; i -= spi_pagesize) {
		xfer_buf.len = (i >= spi_pagesize) ? spi_pagesize : i;
		xfer_buf.tx_buf = &tx_data[page * spi_pagesize];
		if (rx_data)
			xfer_buf.rx_buf = &rx_data[page * spi_pagesize];

		// dev_info(&sja1110->spi->dev,
		// 	"[%s] writing %d bytes with bits_per_word=%d (max hz: %d, mode: %d)\n",
		// 	__func__, xfer_buf.len, sja1110->spi->bits_per_word,
		// 	sja1110->spi->max_speed_hz, sja1110->spi->mode);
		// dump_msg(xfer_buf.tx_buf, xfer_buf.len);

		ret = spi_sync(sja1110->spi, &msg);
		page++;

		if (ret) {
			dev_err(&sja1110->spi->dev, "SPI sync error (%d)\n", ret);
			break;
		}
	}

	return ret;
}

/**
 * This function loads a given binary, checks and uploads it via SPI,
 * and verifies that the upload was successful.
 * It initializes both switch and µC of the SJA1110 by using device specific
 * callbacks from the sja1110_priv structure.
 */
static int sja1110_init_hw(struct sja1110_priv *sja1110)
{
	int ret;
	const struct firmware *fw;

	mutex_lock(&sja1110->lock);

	ret = request_firmware(&fw, sja1110->bin_name, &sja1110->spi->dev);
	if (ret) {
		dev_err(&sja1110->spi->dev, 
			"request_firmware failed with %d\n", ret);
		goto out_return;
	}

	dev_info(&sja1110->spi->dev, "[%s] loaded fw '%s'\n", __func__,
		sja1110->bin_name);

	ret = sja1110->pre_upload(sja1110, (u8 *)fw->data, fw->size);
	if (ret) {
		dev_err(&sja1110->spi->dev, 
			"checking firmware failed with %d\n", ret);
		goto out_release;
	}

	ret = sja1110->upload(sja1110, (u8 *)fw->data, fw->size);
	if (ret) {
		dev_err(&sja1110->spi->dev, 
			"uploading firmware failed with %d\n", ret);
		goto out_release;
	}

	ret = sja1110->post_upload(sja1110, (u8 *)fw->data, fw->size);
	if (ret) {
		dev_err(&sja1110->spi->dev,
			"verify firmware failed with %d\n", ret);
		goto out_release;
	}


out_release:
	release_firmware(fw);
out_return:
	mutex_unlock(&sja1110->lock);
	return ret;
}

static void sja1110_init_hw_worker(struct work_struct *work)
{
	struct sja1110_priv *sja1110;

	sja1110 = container_of(work, struct sja1110_priv, work);
	if (sja1110_init_hw(sja1110))
		dev_err(&sja1110->spi->dev,
			"Could not initialize hw (type=%d)\n",
			sja1110->devtype);
}

static void sja1110_post_probe_upload_worker(struct work_struct *work)
{
	struct sja1110_priv *sja1110 = g_state.sja1110[SJA1110_SWITCH];
	if (!sja1110)
		return;

	dev_info(&sja1110->spi->dev,
		 "[%s] Trying to initialize switch\n", __func__);
	sja1110_init_hw(sja1110);

	sja1110 = g_state.sja1110[SJA1110_UC];
	if (!sja1110)
		return;

	if (wait_for_completion_killable(&g_state.uc_initialized) != 0)
		return;

	dev_info(&sja1110->spi->dev,
		 "[%s] Trying to initialize uC\n", __func__);
	sja1110_init_hw(sja1110);
}


/*******************************************************************************
 * sysfs userspace interface
 ******************************************************************************/
static ssize_t sysfs_get_cfg_upload(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE,
		"[%s] Write a filename to start the upload of that file (\"%s\" for default firmware name)\n",
		__func__, DEF_INDICATOR);
}

static ssize_t sysfs_set_cfg_upload(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct sja1110_priv *sja1110 = spi_get_drvdata(spi);

	mutex_lock(&sja1110->lock);

	/**
	 * Update char array 'bin_name' to contain the requested binary name.
	 * String in 'buf' is newline terminated (buf[count - 1] == '\n')
	 */
	if (strncmp(buf, DEF_INDICATOR, sizeof(DEF_INDICATOR) - 1) == 0 &&
	    buf[sizeof(DEF_INDICATOR) - 1] == '\n') {
		strcpy(sja1110->bin_name, sja1110->def_bin_name);
	} else {
		strncpy(sja1110->bin_name, buf, PATH_LEN);
		if (count < PATH_LEN)
			sja1110->bin_name[count - 1] = 0;
		else
			sja1110->bin_name[PATH_LEN - 1] = 0;
	}

	mutex_unlock(&sja1110->lock);

	dev_info(&spi->dev,
		 "[%s] received command to upload binary '%s' to %s\n",
		 __func__, sja1110->bin_name,
		 sja1110->devtype == SJA1110_SWITCH ? "switch" : "uc");

	/* schedule firmware upload in separate kernel thread */
	schedule_work(&sja1110->work);

	return count;
}

static DEVICE_ATTR(switch_cfg_upload, S_IWUSR | S_IRUSR,
		   sysfs_get_cfg_upload, sysfs_set_cfg_upload);
static DEVICE_ATTR(uc_fw_upload, S_IWUSR | S_IRUSR,
		   sysfs_get_cfg_upload, sysfs_set_cfg_upload);

static ssize_t sysfs_get_reset(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE,
		"[%s] Write to this file to reset the SJA1110\n", __func__);
}

static ssize_t sysfs_set_reset(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct sja1110_priv *sja1110 = spi_get_drvdata(spi);

	sja1110_reset(sja1110);

	return count;
}

static DEVICE_ATTR(switch_reset, S_IWUSR | S_IRUSR,
		   sysfs_get_reset, sysfs_set_reset);

/* uc attribute group */
static struct attribute *uc_sysfs_attrs[] = {
	&dev_attr_uc_fw_upload.attr,
	NULL
};

static struct attribute_group uc_attribute_group = {
	.name = "uc-configuration",
	.attrs = uc_sysfs_attrs,
};

/* switch attribute group */
static struct attribute *switch_sysfs_attrs[] = {
	&dev_attr_switch_cfg_upload.attr,
	&dev_attr_switch_reset.attr,
	NULL
};

static struct attribute_group switch_attribute_group = {
	.name = "switch-configuration",
	.attrs = switch_sysfs_attrs,
};


/*******************************************************************************
 * SPI Driver
 ******************************************************************************/
static int sja1110_probe(struct spi_device *spi)
{
	int ret;
	struct sja1110_priv *sja1110;
	const struct of_device_id *match;

	sja1110 = devm_kzalloc(&spi->dev, sizeof(*sja1110), GFP_KERNEL);
	if (!sja1110) {
		dev_err(&spi->dev,
			"Memory allocation for sja1110_priv failed\n");
		ret = -ENOMEM;
		goto out;
	}
	spi_set_drvdata(spi, sja1110);

	/* determine if SPI device is µC or switch */
	match = of_match_device(sja1110_dt_ids, &spi->dev);
	if (!match) {
		dev_err(&spi->dev, "No matching DTS node was found\n");
		ret = -ENODEV;
		goto out;
	}

	/* init common private data structure */
	spi->max_speed_hz = max_spi_speed;
	sja1110->spi      = spi;
	sja1110->devtype  = (long)match->data;
	mutex_init(&sja1110->lock);
	INIT_WORK(&sja1110->work, sja1110_init_hw_worker);

	/* device specific probe */
	if (sja1110->devtype == SJA1110_UC) {
		/* µC firmware */
		dev_info(&spi->dev, "probing uc\n");

		strncpy(sja1110->bin_name, firmware_name, PATH_LEN);
		sja1110->bin_name[PATH_LEN - 1] = 0;

		spi->bits_per_word    = 8;
		sja1110->def_bin_name = firmware_name;
		sja1110->pre_upload   = sja1110_pre_uc_upload;
		sja1110->upload       = sja1110_uc_upload;
		sja1110->post_upload  = sja1110_post_uc_upload;
		sja1110->switch_priv  = NULL;

		/* indicate that µC data structures are initialized */
		g_state.sja1110[sja1110->devtype] = sja1110;
		complete(&g_state.uc_initialized);

		/* create sysfs file and attach attributes */
		ret = sysfs_create_group(&spi->dev.kobj,
					 &uc_attribute_group);
		if (ret)
			goto out;
	} else {
		/* switch static config */
		dev_info(&spi->dev, "probing switch\n");

		strncpy(sja1110->bin_name, config_name, PATH_LEN);
		sja1110->bin_name[PATH_LEN - 1] = 0;

		spi->bits_per_word    = 64;
		sja1110->def_bin_name = config_name;
		sja1110->pre_upload   = sja1110_pre_switch_upload;
		sja1110->upload       = sja1110_switch_upload;
		sja1110->post_upload  = sja1110_post_switch_upload;

		sja1110->switch_priv = devm_kzalloc(&spi->dev,
			sizeof(struct sja1110_switch_priv), GFP_KERNEL);
		if (!sja1110->switch_priv) {
			dev_err(&spi->dev,
				"Memory allocation for sja1110_switch failed\n");
			ret = -ENOMEM;
			goto out;
		}

		/* try to get a GPIO pin from the device tree */
		sja1110->switch_priv->rst_gpio = get_and_request_gpio(sja1110);

		/**
		 * Try to upload binaries with default names after probing.
		 * Upload switch config FIRST, µC firmware SECOND.
		 * Wait until µC is probed BEFORE starting the fw upload
		 * (switch is already probed when we reach this section).
		 * Synchronization via 'uc_initialized' completion.
		 */
		g_state.sja1110[sja1110->devtype] = sja1110;
		schedule_work(&g_state.post_probe_work);

		/* create sysfs file and attach attributes */
		ret = sysfs_create_group(&spi->dev.kobj,
					 &switch_attribute_group);
		if (ret)
			goto out;
	}


out:
	return ret;
}

static int sja1110_remove(struct spi_device *spi)
{
	struct sja1110_priv *sja1110;

	dev_info(&spi->dev, "[%s]\n", __func__);

	sja1110 = spi_get_drvdata(spi);
	if (sja1110->devtype == SJA1110_UC) {
		sysfs_remove_group(&spi->dev.kobj,
				   &uc_attribute_group);
	} else {
		sysfs_remove_group(&spi->dev.kobj,
				   &switch_attribute_group);

		if (sja1110->switch_priv->rst_gpio > 0)
			gpio_free(sja1110->switch_priv->rst_gpio);
	}

	return 0;
}

static const struct of_device_id sja1110_dt_ids[] = {
	{ .compatible = "nxp,sja1110-switch", .data = (const void*)SJA1110_SWITCH },
	{ .compatible = "nxp,sja1110-uc", .data = (const void*)SJA1110_UC },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sja1110_dt_ids);

static struct spi_driver sja1110_driver = {
	.probe  = sja1110_probe,
	.remove = sja1110_remove,
	.driver = {
		.name = "sja1110",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sja1110_dt_ids),
	},
};


/*******************************************************************************
 * Module
 ******************************************************************************/
static int __init sja1110_driver_init(void)
{
	init_completion(&g_state.uc_initialized);
	INIT_WORK(&g_state.post_probe_work, sja1110_post_probe_upload_worker);

	return spi_register_driver(&sja1110_driver);
}
module_init(sja1110_driver_init);

static void __exit sja1110_driver_exit(void)
{
	spi_unregister_driver(&sja1110_driver);
}
module_exit(sja1110_driver_exit);

MODULE_VERSION("0.4.0");
MODULE_AUTHOR("Marco Hartmann <marco.hartmann@nxp.com>");
MODULE_DESCRIPTION("SJA1110 SPI driver");
MODULE_LICENSE("GPL");
