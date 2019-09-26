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
* \file  sja1110_init.h
*
* \author Marco Hartmann
*
* \date 2019-07-09
*
* \brief Functions definitions for SJA1110 SPI driver
*
*******************************************************************************/
#ifndef _SJA1110_SPI_H__
#define _SJA1110_SPI_H__

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define DEF_FNAME_UC     "sja1110_uc.bin"
#define DEF_FNAME_SWITCH "sja1110_switch.bin"
#define DEF_INDICATOR    "def"
#define PATH_LEN             128

#define HEADER_EXEC          {0xDD, 0x11}
#define HEADER_STATUS_PACKET {0xCC}
#define IMAGE_VALID_MARKER   {0x6A,0xA6,0x6A,0xA6,0x6A,0xA6,0x6A,0xA6}
#define STATUS_PKT_HEADER    0xCC
enum uc_status {
	INITIALIZING = 0x31,
	BOOTING      = 0x32,
	WAITING      = 0x33,
	DOWNLOADING  = 0x34,
	VERIFYING    = 0x35,
	COMPLETED    = 0x36
};
enum uc_err_code {
	SFDP_FAILURE                = 0x51,
	INVALID_CLOCK_SETTING_VALUE = 0x52,
	INVALID_MARKER              = 0x53,
	INVALID_PUBLIC_ROOT_KEY     = 0x54,
	INVALID_PUBLIC_KEY_VERSION  = 0x55,
	INVALID_COT_SIGNATURE       = 0x56,
	INVALID_FW_LENGTH           = 0x57,
	INVALID_FW_VERSION          = 0x58,
	INVALID_ENTRY_PTR           = 0x59,
	INVALID_FW_SIGNATURE        = 0x5A,
	NO_ERROR                    = 0x5B
};

#define CMD_RWOP_SHIFT       31
#define CMD_RD_OP             0
#define CMD_WR_OP             1
#define CMD_REG_ADDR_SHIFT    4
#define CMD_REG_ADDR_WIDTH   21
#define CMD_REG_RD_CNT_SHIFT 25
#define CMD_REG_RD_CNT_WIDTH  6
#define CMD_ENCODE_RWOP(_write_)  ((_write_) << CMD_RWOP_SHIFT)
#define CMD_ENCODE_ADDR(_addr_)   (((_addr_) << CMD_REG_ADDR_SHIFT)   & GENMASK(CMD_REG_ADDR_SHIFT + CMD_REG_ADDR_WIDTH, CMD_REG_ADDR_SHIFT))
#define CMD_ENCODE_WRD_CNT(_cnt_) (((_cnt_)  << CMD_REG_RD_CNT_SHIFT) & GENMASK(CMD_REG_RD_CNT_SHIFT + CMD_REG_RD_CNT_WIDTH, CMD_REG_RD_CNT_SHIFT))

#define CF_CONFIGS_MASK  BIT(31) /**< Bit mask for the CONFIGS field */
#define CF_CRCCHKL_MASK  BIT(30) /**< Bit mask for the CRCCHKL field */
#define CF_IDS_MASK      BIT(29) /**< Bit mask for the IDS field */
#define CF_CRCCHKG_MASK  BIT(28) /**< Bit mask for the CRCCHKG field */

#define CONFIG_START_ADDRESS   (0x20000UL)  /**< Start Address of the configuration */
#define D_ID_ADDR              (0x0UL)      /**< Address of the deviceId register */
#define CONFIGURATIONFLAG_ADDR (0x1UL)      /**< Address of the configurationFlags register */
#define R_CTRL_ADDR            (0x1C6000UL) /**< Address of the resetCtrl register */
#define RESET_CTRL_COLDRESET   BIT(5)
#define RESET_DELAY_US         50
#define CGU_SPI_BASE_ADDR      (0x1C6400UL)
#define CGU_OUTCLK_C_WATCHDOG  (CGU_SPI_BASE_ADDR + 0x1DUL)

#define SJA1110_VAL_DEVICEID (0xb700030eUL)

#define SJA1110_NUM_GPIOS  16

#define GPIO_SPI_BASE_ADDR (0x1C4800UL)
#define GPIO_PDO_ADDR      (GPIO_SPI_BASE_ADDR + 0x00UL)
#define GPIO_PDOSET_ADDR   (GPIO_SPI_BASE_ADDR + 0x01UL)
#define GPIO_PDOCLR_ADDR   (GPIO_SPI_BASE_ADDR + 0x02UL)
#define GPIO_PDI_ADDR      (GPIO_SPI_BASE_ADDR + 0x40UL)
#define GPIO_PCOE_ADDR     (GPIO_SPI_BASE_ADDR + 0x80UL)
#define GPIO_PCOM_ADDR     (GPIO_SPI_BASE_ADDR + 0x81UL)
#define GPIO_PCIE_ADDR     (GPIO_SPI_BASE_ADDR + 0xC0UL)


/*******************************************************************************
 * Data Types
 ******************************************************************************/
enum spi_devtype {SJA1110_SWITCH, SJA1110_UC};

struct sja1110_switch_priv {
	struct gpio_desc *rst_gpio;  /**< descriptor of GPIO used to reset the device */
	struct gpio_chip gpio_chip;  /**< controller for SJA110's own GPIOs */
};

struct sja1110_priv {
	struct spi_device *spi;      /**< Passsed at SPI probing */
	char bin_name[PATH_LEN];     /**< Name of the binary (fw or config) */
	char *def_bin_name;          /**< Default name of the binary */
	struct mutex lock;           /**< Protect private data structure */
	enum spi_devtype devtype;    /**< Type of the SPI device */
	int (*pre_upload) (struct sja1110_priv*, const u8*, int);
	int (*upload)     (struct sja1110_priv*, const u8*, int);
	int (*post_upload)(struct sja1110_priv*, const u8*, int);
	struct work_struct work;
	struct sja1110_switch_priv *switch_priv; /**< Additional data only required for the switch SPI endpoint */ 
};

struct sja1110_uc_status_pkt {
	enum uc_status status;
	enum uc_err_code err_code;
	u8 attempt;
};

struct sja1110_state {
	struct completion uc_initialized;
	struct sja1110_priv *sja1110[2];
	struct work_struct post_probe_work;
};


/*******************************************************************************
 * Fwd declarations
 ******************************************************************************/
static const struct of_device_id sja1110_dt_ids[];
static int sja1110_simple_upload(struct sja1110_priv*, const u8*, int, int);
static int sja1110_spi_transfer(struct sja1110_priv *sja1110,
				const u8 *tx_data, u8 *rx_data,
				int size, int spi_pagesize);

#endif /* _SJA1110_SPI_H__ */
