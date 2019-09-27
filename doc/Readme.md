# Device Tree Overlay for SJA1110-EVM

## Structure
The Overlay file `SJA1110-EVM-Overlay.dts` contains a description of some of the hardware components on the SJA1110-EVM Board.
It is designed to be platform independent, such that the overlay can be applied to a variety of base device trees that describe a target platform.

Some glue-component is required, which connects the highly platform specific base device tree with the platform independent overlay:
This is implemented with help of the header file `platform_integration.h`. It contains a set of C-macros, that translate between the generic placeholder values of the overlay and the specific values of the base device tree.

Included in this directory is a platform integration header file for the i.MX 8QXPlus MEK CPU Board (device tree name: `fsl-imx8qxp-mek-rpmsg.dtb`).
**It serves as an example and needs to be adjusted to the target hardware.**

### Platform integration points
- `ETH_CON_SPI_CONTROLLER`
	- Label of the SPI controller that drives the SPI bus of SJA1110
- `ETH_CONTROLLER`
	- Label of the ethernet controller that drives the MDIO bus to which the TJA1101 PHY is connected
- `ETH_CON_RST_CONTROLLER`
	- Label of the GPIO controller that controls the reset pin of SJA1110
- `GPIO_SW0_RST`
	- Number of the GPIO pin connected to the reset pin of SJA1110
- `CS0_ADDR`
	- Address of the Chip Select (CS) pin attached to the `switch` SPI endpoint of SJA1110
- `CS1_ADDR`
	- Address of the Chip Select (CS) pin attached to the `uC` SPI endpoint of SJA1110

## Macro Fix
The symbol `linux` is a built in macro of the GCC preprocessor. To avoid substitution during preprocessing, the macro needs to be undefined before use.
This is the reason for the line
```
#undef linux
```
in the overlay file `SJA1110-EVM-Overlay.dts`.
