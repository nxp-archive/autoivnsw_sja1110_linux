# SJA1110 Linux Driver Release v0.0.1 - User Notes

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
- There are two available module parameters that can be set during module loading:
	- `firmware_name`: Default filename of the firmware binary for the uC (default value: `sja1110_uc.bin`)
	- `config_name`: Default filename of the static configuration binary for the switch (default value: `sja1110_switch.bin`)

**Note:** The driver will try to upload both the firmware and the static configuration with the above default names during device probing. In case the files do not exist, the upload can be initiated from userspace at a later point in time via the [sysfs userspace interface](#Userspace-Interface)

## Userspace Interface
The **SJA1110** driver exposes two files via the *sysfs* filesystem to userspace, that allow interfacing with the driver by writing to them. The files are usually located in `/sys/bus/spi/devices/spiX.X/`.
- `uc_fw_upload`
	- Writing a filename initiates a firmware upload of the given file to the uC
	- Writing `def` initiates an upload using the default filename given by the `firmware_name` module parameter
- `switch_cfg_upload`
	- Writing a filename initiates a static configuration upload of the given file to the switch
	- Writing `def` initiates an upload using the default filename given by the `config_name` module parameter

**Note:** The firmware binaries need to be located in the default firmware directory (usually `/lib/firmware/`).
Alternatively, the provided path needs to be a relative path starting from this default firmware directory.

## DTS Information
Since the **SJA1110** appears as two logical SPI devices, two distinct device tree entries are required. They are described in the sections below. For a full example of the device tree node refer to `doc/example_device_tree.dtsi`.

### uC node
The uC node needs to have the following properties:
- Compatible string (is matched against `sja1110_dt_ids[]` list)
	> `compatible = "nxp,sja1110-uc";`
- Address of the Chipselect (cs) line
	> `reg = <0x0>;`(example)
- Maximum SPI Frequency
	> `spi-max-frequency = <12000000>;` (example)

### switch node
The switch node needs to have the follong properties:
- Compatible string (is matched against `sja1110_dt_ids[]` list)
	> `compatible = "nxp,sja1110-switch";`
- Address of the Chipselect (CS) line
	> `reg = <0x1>;`(example)
- Maximum SPI Frequency
	> `spi-max-frequency = <12000000>;` (example)


## Supported Linux versions
The driver was developed for Linux v4.1.15 but should be compatible with all versions from v4.0 up to v5.3