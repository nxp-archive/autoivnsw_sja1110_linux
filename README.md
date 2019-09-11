# SJA1110 Linux Driver Release v0.4.0 - User Notes

## Changelog
v0.4.0: (Aug 2019)
- Initial release

---

## Table of Contents
1. [Overview](#Overview)
2. [SJA1110 Loading](#SJA1110-loading)
3. [Userspace Interface](#Userspace-Interface)
4. [DTS Information](#DTS-Information)
5. [Supported Linux versions](#Supported-Linux-versions)

---

## Overview
This Document describes a driver for the NXP **SJA1110** Ethernet Switch. Logically, the **SJA1110** appears as two distinct SPI devices:
- **SPI_HOST**: Microcontroller Subsystem (MCSS), in the following just *uC*
- **SPI_AP**: Memory mapped access to the switch device, in the following just *switch*

## SJA1110 Loading
- The kernel module can for example be loaded by executing `insmod sja1110.ko`
- There are multiple module parameters that can be set during module loading:
	- `firmware_name`: Default filename of the firmware binary for the uC (default value: `sja1110_uc.bin`)
	- `config_name`: Default filename of the static configuration binary for the switch (default value: `sja1110_switch.bin`)
	- `max_spi_speed`: Clock Frequency that is used for SPI transfers (default value: `10 MHz`)

### Auto Upload
The driver will try to upload both the firmware and the static configuration with the above default names during device probing.
The switch configuration will be uploaded first, the uC firmware second.
Prior to the switch configuration upload, the **SJA1110** will be reset. Refer to [SJA1110 Reset](#SJA1110-Reset) for reset details.

In case the files do not exist, the upload of configuration and firmware files can be initiated from userspace at a later point in time via the [sysfs userspace interface](#Userspace-Interface)

## Userspace Interface
The **SJA1110** driver exposes multiple files via the *sysfs* filesystem to userspace, that allow interfacing with the driver by writing to them.
The files are usually located in `/sys/bus/spi/devices/spiX.X/uc-configuration/` and `/sys/bus/spi/devices/spiY.Y/switch-configuration/`, where `spiX.X` is the SPI endpoint of the uC and `spiY.Y` is the SPI endpoint of the switch.
- Directory `uc-configuration`
	- `uc_fw_upload`
		- Writing a filename initiates a firmware upload of the given file to the uC
		- Writing `def` initiates an upload using the default filename given by the `firmware_name` module parameter
- Directory `switch-configuration`
	- `switch_cfg_upload`
		- Writing a filename initiates a static configuration upload of the given file to the switch
		- Writing `def` initiates an upload using the default filename given by the `config_name` module parameter
	- `switch_reset`
		- Writing anything to this file resets the **SJA1110** according to [SJA1110 Reset](#SJA1110-Reset).

**Note:** The firmware binaries need to be located in the default firmware directory (usually `/lib/firmware/`).
Alternatively, the provided path needs to be a relative path starting from this default firmware directory.

## SJA1110 Reset
The **SJA1110** can be reset by either pulling the reset pin low, or by writing to the *ResetCtrl* register via SPI. In case a GPIO pin is configured in the [device tree](#DTS-Information), the former method will be preferred. Reset is done automatically during [auto upload](#Auto-Upload) but can also be manually triggered via the [sysfs userspace interface](#Userspace-Interface).

## DTS Information
Since the **SJA1110** appears as two logical SPI devices, two distinct device tree entries are required. They are described in the sections below. For a full example of the device tree node refer to `doc/example_device_tree.dtsi`.

### switch node
The switch node **must** have the follong properties:
- Compatible string (is matched against `sja1110_dt_ids[]` list)
	> `compatible = "nxp,sja1110-switch";`
- Address of the Chipselect (CS) line
	> `reg = <0x0>;`(example)
- Maximum SPI Frequency
	> `spi-max-frequency = <12000000>;` (example)

The uC node **may** have the following properties:
- Reset GPIO Number
	> `reset-gpio = <195>;` (example)

### uC node
The uC node **must** have the following properties:
- Compatible string (is matched against `sja1110_dt_ids[]` list)
	> `compatible = "nxp,sja1110-uc";`
- Address of the Chipselect (cs) line
	> `reg = <0x1>;`(example)
- Maximum SPI Frequency
	> `spi-max-frequency = <12000000>;` (example)

## Supported Linux versions
The driver was developed for Linux v4.1.15 but should be compatible with all versions from v4.0 up to v5.3