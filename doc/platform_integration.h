/**
 * Macros that allow interfacing between generic SJA1110 EVM Board overlay
 * and the device tree of the target platform.
 * In this example, the target platform is 'i.MX 8QXPlus MEK'
 */
#define GPIO_SW0_RST 3
#define CS0_ADDR 0x0
#define CS1_ADDR 0x1

#define ETH_CON_SPI_CONTROLLER lpspi3
#define ETH_CON_RST_CONTROLLER pca9557_a
#define ETH_CONTROLLER fec1
