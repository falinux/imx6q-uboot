/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014, HKC Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <splash.h>

#include "lcd_logo.c"

#define DPRINT printf ("_____file [ %s ] / FUNC [ %s ]  / LINE [ %d ] ______\n", __FILE__, __func__, __LINE__)

#define CS0_ADDRESS     0x08000000 
#define CS1_ADDRESS     0x0A000000 

#define	PHY_CS0_ADDR	*(volatile unsigned char  *)(CS0_ADDRESS)
#define	PHY_CS1_ADDR	*(volatile unsigned short *)(CS1_ADDRESS)
                     
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL   (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
#define USDHC_PAD_CTRL  (PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
#define ENET_PAD_CTRL   (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define SPI_PAD_CTRL    (PAD_CTL_HYS         | PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |	PAD_CTL_DSE_40ohm  | PAD_CTL_SRE_FAST)
#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_SRE_FAST)
#define WEIM_PAD_CTRL   (PAD_CTL_SRE_FAST    | PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)	
#define LCD_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define OUTPUT_40OHM    (PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm)
#define GPMI_PAD_CTRL0  (PAD_CTL_PKE         | PAD_CTL_PUE       | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1  (PAD_CTL_DSE_40ohm   | PAD_CTL_SPEED_MED | PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2  (GPMI_PAD_CTRL0      | GPMI_PAD_CTRL1)
#define GPIO_PAD_CTRL   (PAD_CTL_DSE_34ohm)


#define T_ANAL_PWON         IMX_GPIO_NR(2,8)    // Output Default : High
#define T_PWR_SERIAL_EN     IMX_GPIO_NR(2,11)   // Output Default : High
#define T_PWR_UART_EN1      IMX_GPIO_NR(2,12)   // Output Default : High
#define T_PWR_UART_EN2      IMX_GPIO_NR(2,15)   // Output Default : High
#define T_ALARM_OUT         IMX_GPIO_NR(3,23)   // Output Default : Low
#define T_RED_LED_CTL       IMX_GPIO_NR(7,2)    // Output Default : High
#define T_YELLOW_LED_CTL    IMX_GPIO_NR(7,3)    // Output Default : Low
#define T_WD_EN             IMX_GPIO_NR(7,4)    // Output Default : High   
#define T_WD_WDI            IMX_GPIO_NR(7,5)    // Output Default : High   
#define T_VALVE_N_CTL       IMX_GPIO_NR(5,22)   // Output Default : Low
#define T_VALVE_P_CTL       IMX_GPIO_NR(5,23)   // Output Default : Low
#define T_SOUND_ONOFF       IMX_GPIO_NR(5,24)   // Output Default : Low
#define T_PWR_ALARM_EN      IMX_GPIO_NR(1,5)    // Output Default : Low
#define T_PWR_EON           IMX_GPIO_NR(5,28)   // Output Default : High

#define T_RF_PEAK           IMX_GPIO_NR(3,26)   // Input
#define T_CRADLE_DET        IMX_GPIO_NR(3,27)   // Input
#define T_EX12V_DET         IMX_GPIO_NR(1,6)    // Input
#define T_PWR_ALIVE         IMX_GPIO_NR(5,29)   // Input

#define LCD_RESET			IMX_GPIO_NR(3, 29)  // Ouptu Default : High   
#define LCD_CS 				IMX_GPIO_NR(5, 20)  // Ouptu Default : High   
#define LCD_RS 				IMX_GPIO_NR(5, 21)  // Ouptu Default : Low   
#define LCD_MUX_SEL 		IMX_GPIO_NR(3, 30)  // Ouptu Default : Low
#define LCD_MUX_EN 			IMX_GPIO_NR(3, 31)  // Ouptu Default : Low   
#define LCD_SYNC_IN         IMX_GPIO_NR(7,  7)  // Input
#define LCD_BACKLIGHT    	IMX_GPIO_NR(4,  5)  // Ouptu Default : High   

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_pwr_pads[] = {
	MX6_PAD_SD4_DAT3__GPIO2_IO11    | MUX_PAD_CTRL(NO_PAD_CTRL), // serial power enable
	MX6_PAD_SD4_DAT4__GPIO2_IO12    | MUX_PAD_CTRL(NO_PAD_CTRL), // console power enable
	MX6_PAD_SD4_DAT7__GPIO2_IO15    | MUX_PAD_CTRL(NO_PAD_CTRL), // user serial power enable
};

static iomux_v3_cfg_t const weim_pads[] = {
	MX6_PAD_EIM_DA0__EIM_AD00    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA1__EIM_AD01    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA2__EIM_AD02    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA3__EIM_AD03    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA4__EIM_AD04    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA5__EIM_AD05    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA6__EIM_AD06    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA7__EIM_AD07    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA8__EIM_AD08    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA9__EIM_AD09    | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA10__EIM_AD10   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA11__EIM_AD11   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA12__EIM_AD12   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA13__EIM_AD13   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA14__EIM_AD14   | MUX_PAD_CTRL(WEIM_PAD_CTRL),     
	MX6_PAD_EIM_DA15__EIM_AD15   | MUX_PAD_CTRL(WEIM_PAD_CTRL),                                                                 
	                                                         
	MX6_PAD_EIM_OE__EIM_OE_B     | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_RW__EIM_RW       | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_WAIT__EIM_WAIT_B | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_LBA__EIM_LBA_B   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_BCLK__EIM_BCLK   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_CS0__EIM_CS0_B   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_CS1__EIM_CS1_B   | MUX_PAD_CTRL(WEIM_PAD_CTRL),
};

static iomux_v3_cfg_t const gpio_ctrl_pads[] = {
	MX6_PAD_SD4_DAT0__GPIO2_IO08	 | MUX_PAD_CTRL(GPIO_PAD_CTRL),  //GPIO2_IO08 -O T_ANAL_PWON
	MX6_PAD_SD4_DAT3__GPIO2_IO11     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO2_IO11 -O T_PWR_SERIAL_EN
	MX6_PAD_SD4_DAT4__GPIO2_IO12     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO2_IO12 -O T_PWR_UART_EN1
	MX6_PAD_SD4_DAT7__GPIO2_IO15     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO2_IO15 -O T_PWR_UART_EN2
	MX6_PAD_EIM_D23__GPIO3_IO23      | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO3_IO23 -O T_ALARM_OUT
	MX6_PAD_SD3_CMD__GPIO7_IO02      | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO7_IO02 -O T_RED_LED_CTL
	MX6_PAD_SD3_CLK__GPIO7_IO03      | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO7_IO03 -O T_YELLOW_LED_CTL
	MX6_PAD_SD3_DAT0__GPIO7_IO04     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO7_IO04 -O T_WD_EN#
	MX6_PAD_SD3_DAT1__GPIO7_IO05     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO7_IO05 -O T_WD_WDI
	MX6_PAD_CSI0_DAT4__GPIO5_IO22    | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO22 -O T_VALVE_N_CTL
	MX6_PAD_CSI0_DAT5__GPIO5_IO23    | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO23 -O T_VALVE_P_CTL
	MX6_PAD_CSI0_DAT6__GPIO5_IO24    | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO24 -O T_SOUND_ONOFF
	MX6_PAD_GPIO_5__GPIO1_IO05	     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO26 -O T_PWR_ALARM_EN
	MX6_PAD_CSI0_DAT10__GPIO5_IO28   | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO28 -O T_PWR_EON
	                                 
	MX6_PAD_EIM_D26__GPIO3_IO26      | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO3_IO26 -I T_RF_PEAK
	MX6_PAD_EIM_D27__GPIO3_IO27      | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO3_IO27 -I T_CRADLE_DET#
	MX6_PAD_GPIO_6__GPIO1_IO06	     | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO27 -I T_EX12V_DET
	MX6_PAD_CSI0_DAT11__GPIO5_IO29   | MUX_PAD_CTRL(NO_PAD_CTRL),  //GPIO5_IO29 -I T_PWR_ALIVE
};

static iomux_v3_cfg_t const lcd_ctrl_pads[] = {
	MX6_PAD_EIM_D29__GPIO3_IO29      | MUX_PAD_CTRL(NO_PAD_CTRL),	// LCD_RESET
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),	// LCD_CS
	MX6_PAD_CSI0_VSYNC__GPIO5_IO21   | MUX_PAD_CTRL(NO_PAD_CTRL),	// LCD_RS ( COMMAND / DATA SEL )
	MX6_PAD_EIM_D30__GPIO3_IO30      | MUX_PAD_CTRL(NO_PAD_CTRL),	// LCD_MUX_SEL
	MX6_PAD_EIM_D31__GPIO3_IO31      | MUX_PAD_CTRL(NO_PAD_CTRL),	// LCD_MUX_EN#
	MX6_PAD_SD3_DAT3__GPIO7_IO07     | MUX_PAD_CTRL(NO_PAD_CTRL),   // LCD_SYNC_IN
};

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on RGB connector: J15 */
	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),        // LCD_BL
};

static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15	    | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02	    | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03	    | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DI0_PIN4__GPIO4_IO20	        | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1 SGTL5000, */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL    | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | PC,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA    | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | PC,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

/* I2C2 PMIC, RTC, MONITOR */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL    | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA    | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};


#ifdef CONFIG_PHY_ATHEROS_AR8031
static iomux_v3_cfg_t const enet_pads[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC          | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__RGMII_TXC        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__RGMII_TD0        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__RGMII_TD1        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__RGMII_TD2        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__RGMII_TD3        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RXC__RGMII_RXC        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__RGMII_RD0        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__RGMII_RD1        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__RGMII_RD2        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__RGMII_RD3        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
    /* AR8031 PHY Reset */
    MX6_PAD_ENET_CRS_DV__GPIO1_IO25     | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t const ecspi4_pads[] = {
	/* SS1 */
	MX6_PAD_EIM_A25__GPIO5_IO02  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi4_pads, ARRAY_SIZE(ecspi4_pads));
}
#endif

#ifdef CONFIG_SYS_USE_NAND
static iomux_v3_cfg_t gpmi_pads[] = {
    MX6_PAD_NANDF_CS0__NAND_CE0_B    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_CLE__NAND_CLE      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_ALE__NAND_ALE      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_CMD__NAND_RE_B 		 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_SD4_CLK__NAND_WE_B  	 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_WP_B__NAND_WP_B 	 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_RB0__NAND_READY_B  | MUX_PAD_CTRL(GPMI_PAD_CTRL0),
    MX6_PAD_NANDF_D0__NAND_DATA00    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D1__NAND_DATA01    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D2__NAND_DATA02    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D3__NAND_DATA03    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D4__NAND_DATA04    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D5__NAND_DATA05    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D6__NAND_DATA06    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NANDF_D7__NAND_DATA07    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
};

static void setup_gpmi_nand(void)
{
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

    /* config gpmi nand iomux */
    imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

    /* gate ENFC_CLK_ROOT clock first,before clk source switch */
    clrbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

    /* config gpmi and bch clock to 100 MHz */
    clrsetbits_le32(&mxc_ccm->cs2cdr,
            MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
            MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
            MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
            MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
            MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
            MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

    /* enable ENFC_CLK_ROOT clock */
    setbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

    /* enable gpmi and bch clock gating */
    setbits_le32(&mxc_ccm->CCGR4,
             MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
             MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
             MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
             MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
             MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

    /* enable apbh clock gating */
    setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif



static void setup_iomux_enet(void)
{
#ifdef CONFIG_PHY_ATHEROS_AR8031
	
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);

#endif
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_pwr_pads, ARRAY_SIZE(uart_pwr_pads));
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_weim(void)
{
	imx_iomux_v3_setup_multiple_pads(weim_pads, ARRAY_SIZE(weim_pads));
}

static void setup_iomux_lcd_ctrl(void)
{
	imx_iomux_v3_setup_multiple_pads(lcd_ctrl_pads, ARRAY_SIZE(lcd_ctrl_pads));
}

static void setup_iomux_gpio_ctrl(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_ctrl_pads, ARRAY_SIZE(gpio_ctrl_pads));	
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	return 0;
}
#endif


int mx6_rgmill_rework(struct phy_device *phydev)
{
	unsigned short val;
	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */ 
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
    // AR8031
	mx6_rgmill_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}
#endif

#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	usb_eth_initialize(bis);
#endif
	return 0;
}


void lcd_write_Bus(unsigned short data)
{
	//CS L
	gpio_set_value(LCD_CS,0);
	asm ("nop");
	asm ("nop");	
	//DATA OUT
	PHY_CS1_ADDR = data;
	//CS H
	asm ("nop");
	asm ("nop");	
	gpio_set_value(LCD_CS,1);
}
 
void lcd_write_command(unsigned short lcd_command)  
{   
  gpio_set_value(LCD_RS,0);
  udelay(1);
  lcd_write_Bus(lcd_command);
}
 
void lcd_write_data(unsigned short lcd_data)    
{
  gpio_set_value(LCD_RS,1);
  udelay(1);
  lcd_write_Bus(lcd_data);
}

void lcd_address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
	lcd_write_command(0x2a);
	lcd_write_data(x1>>8);
	lcd_write_data(x1);
	lcd_write_data(x2>>8);
	lcd_write_data(x2);
	
	lcd_write_command(0x2b);
	lcd_write_data(y1>>8);
	lcd_write_data(y1);
	lcd_write_data(y2>>8);
	lcd_write_data(y2);
	
	lcd_write_command(0x2C);  
}

void lcd_mdelay(int delay)
{
	int i;
	for(i=0; i<delay; i++) {
		udelay(1000);
	}
} 

void lcd_init(void)
{
	//bus 채널 선택 
	gpio_set_value(LCD_MUX_SEL,0);  // EIM Interface Selection    
	gpio_set_value(LCD_MUX_EN,0);   // ENABLE
	
	//RESET
	gpio_set_value(LCD_RESET,1);
	lcd_mdelay(1);
	gpio_set_value(LCD_RESET,0);
	lcd_mdelay(10);
	gpio_set_value(LCD_RESET,1);
	lcd_mdelay(120);
		
	//************* Start Initial Sequence **********// 
	lcd_write_command(0xCF);  
	lcd_write_data(0x00); 
	lcd_write_data(0xC1); 
	lcd_write_data(0X30); 
	
	lcd_write_command(0xED);  
	lcd_write_data(0x64); 
	lcd_write_data(0x03); 
	lcd_write_data(0X12); 
	lcd_write_data(0X81); 
	
	lcd_write_command(0xE8);  
	lcd_write_data(0x85); 
	lcd_write_data(0x10); 
	lcd_write_data(0x78); 
	
	lcd_write_command(0xCB);  
	lcd_write_data(0x39); 
	lcd_write_data(0x2C); 
	lcd_write_data(0x00); 
	lcd_write_data(0x34); 
	lcd_write_data(0x02); 
	
	lcd_write_command(0xF7);  
	lcd_write_data(0x20); 
	
	lcd_write_command(0xEA);  
	lcd_write_data(0x00); 
	lcd_write_data(0x00); 
	
	lcd_write_command(0xC0);    // Power control 
	lcd_write_data(0x21);       // VRH[5:0] 
	
	lcd_write_command(0xC1);    // Power control 
	lcd_write_data(0x12);       // SAP[2:0];BT[3:0] 
	
	lcd_write_command(0xC5);    // VCM control 
	lcd_write_data(0x32); 
	lcd_write_data(0x3C); 
	
	lcd_write_command(0xC7);    // VCM control2 
	lcd_write_data(0XB2); 
	
	lcd_write_command(0x36);    // Memory Access Control 
	lcd_write_data(0x08); 
	
	lcd_write_command(0x3A);   
	lcd_write_data(0x55); 
	
	lcd_write_command(0xB1);   
	lcd_write_data(0x00);   
	lcd_write_data(0x17); 
	
	lcd_write_command(0xB6);    // Display Function Control 
	lcd_write_data(0x0A); 
	lcd_write_data(0xA2); 
	
	lcd_write_command(0xF6);    
	lcd_write_data(0x01); 
	lcd_write_data(0x30); 
	
	lcd_write_command(0xF2);    // 3Gamma Function Disable 
	lcd_write_data(0x00); 
	
	lcd_write_command(0x26);    // Gamma curve selected 
	lcd_write_data(0x01); 
	
	lcd_write_command(0xE0);    // Set Gamma 
	lcd_write_data(0x0F); 
	lcd_write_data(0x20); 
	lcd_write_data(0x1E); 
	lcd_write_data(0x07); 
	lcd_write_data(0x0A); 
	
	lcd_write_data(0x03); 
	lcd_write_data(0x52); 
	lcd_write_data(0X63); 
	lcd_write_data(0x44); 
	lcd_write_data(0x08);
	
	lcd_write_data(0x17); 
	lcd_write_data(0x09); 
	lcd_write_data(0x19); 
	lcd_write_data(0x13); 
	lcd_write_data(0x00); 
	
	lcd_write_command(0XE1);    // Set Gamma 
	lcd_write_data(0x00); 
	lcd_write_data(0x16); 
	lcd_write_data(0x19); 
	lcd_write_data(0x02); 
	lcd_write_data(0x0F); 
	
	lcd_write_data(0x03); 
	lcd_write_data(0x2F); 
	lcd_write_data(0x13); 
	lcd_write_data(0x40); 
	lcd_write_data(0x01);
	
	lcd_write_data(0x08); 
	lcd_write_data(0x07); 
	lcd_write_data(0x2E); 
	lcd_write_data(0x3C); 
	lcd_write_data(0x0F); 
	
	lcd_write_command(0x11);    // Exit Sleep 
	lcd_mdelay(120); 
	lcd_write_command(0x29);    // Display on 
	
	lcd_write_command(0x22);    // Memory Write
}
	
void imx6_sel_fb_out(void)
{
	lcd_address_set(0,0,240,320); 

    // BUS 채널 선택 ( LCD INTERFACE )
	gpio_set_value(LCD_RS,1);               
	gpio_set_value(LCD_MUX_SEL,0);  // EIM Interface Selection    
	gpio_set_value(LCD_MUX_EN,0); 	// ENABLE
	while(1) {
		if( !gpio_get_value(LCD_SYNC_IN) ) {	
			gpio_set_value(LCD_MUX_SEL,1);  // RGB Interface
	    	break;
		}
	} 		
}

U_BOOT_CMD(
	flcd, 1, 0, imx6_sel_fb_out,
	"Tests for LCD RGB Interface.",
	"Returns 0 (true) to shell if key is pressed."
);

void imx6_sel_cpu_out(void)
{  
  	int i,j;
  	int	k = 0;

	lcd_address_set(0,0,240,320); 
	                       
    // BUS 채널 선택 ( LCD INTERFACE )
	gpio_set_value(LCD_RS,1);               
	gpio_set_value(LCD_MUX_EN,1);   // DISABLE
	gpio_set_value(LCD_MUX_SEL,0);  // EIM Interface Selection
	gpio_set_value(LCD_MUX_EN,0);   // ENABLE
	gpio_set_value(LCD_CS,0);       // 

  	for(i=0;i<240;i++) {
    	for (j=0;j<320;j++) {
			PHY_CS1_ADDR = lcd_logo_image[k];
			k++; 
    	}
  	}

	gpio_set_value(LCD_CS,1);
	udelay(10);
}

U_BOOT_CMD(
	clcd, 1, 0, imx6_sel_cpu_out,
	"Tests for LCD CPU Interface.",
	"Returns 0 (true) to shell if key is pressed."
);

void imx6_power_on(void)
{
	gpio_set_value(T_ANAL_PWON,1);
	mdelay(5000);
	gpio_set_value(T_ANAL_PWON,0);
	mdelay(5000);
	gpio_set_value(T_ANAL_PWON,1);
	mdelay(5000);
	gpio_set_value(T_ANAL_PWON,0);
	mdelay(5000);
}	

U_BOOT_CMD(
	pwr, 1, 0, imx6_power_on,
	"Tests for LCD CPU Interface.",
	"Returns 0 (true) to shell if key is pressed."
);

int splash_screen_prepare(void)
{
	char *env_loadsplash;

	if (!getenv("splashimage") || !getenv("splashsize")) {
		return -1;
	}

	env_loadsplash = getenv("loadsplash");
	if (env_loadsplash == NULL) {
		printf("Environment variable loadsplash not found!\n");
		return -1;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		printf("failed to run loadsplash %s\n\n", env_loadsplash);
		return -1;
	}

	return 0;
}

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};


static void enable_rgb(struct display_info_t const *dev)
{
	gpio_direction_output(LCD_BACKLIGHT, 1);
}

static struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	=  0,
	.pixfmt	= IPU_PIX_FMT_RGB565,		// IPU_PIX_FMT_BGR24, IPU_PIX_FMT_RGB565
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "ITST240QV-035S",
		.refresh        = 60,
		.xres           = 240,
		.yres           = 320,
		.pixclock       = 8350,  // 6.35MHz
		.left_margin    = 20,   // HBP 20
		.right_margin   = 10,   // HFP 10
		.upper_margin   = 2,    // VBP 2
		.lower_margin   = 4,    // VFP 4
		.hsync_len      = 4,    // HSYNC 4
		.vsync_len      = 2,    // VSYNC 2
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_cfb_skip(void)
{
	return 0 != getenv("novideo");
}

int board_video_skip(void)
{
	int i=0;
	int ret;
	char const *panel;
	
	imx_iomux_v3_setup_multiple_pads( rgb_pads, ARRAY_SIZE(rgb_pads));

	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else {
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
		}
	} else {
		printf("unsupported panel %s\n", panel);
		ret = -EINVAL;
	}

	if (!ret) 
		splash_screen_prepare();

	return (0 != ret);
}


static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
         |(IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	imx_iomux_v3_setup_multiple_pads(backlight_pads, ARRAY_SIZE(backlight_pads));	
	gpio_direction_output(LCD_BACKLIGHT,1);		
}

static void mx6_setup_weimcs(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;	
	unsigned int reg;

	/* CLKCTL_CCGR6: Set emi_slow_clock to be on in all modes */
	reg = readl(&mxc_ccm->CCGR6);
	reg |= 0x00000C00;
	writel(reg, &mxc_ccm->CCGR6);

    /*****************************************************************************/
    /*
     * IOMUXC_GPR1
     *
     *      CS0(128M), CS1 (0M), CS2 (0M), CS3(0M) [default configuration]
     *      CS0(64M), CS1(64M), CS2(0M), CS3(0M)
     *      CS0(64M), CS1(32M), CS2(32M), CS3(0M)
     *      CS0(32M), CS1(32M), CS2(32M), CS3(32M)
     */
	reg = readl(&iomux->gpr[3]);

	reg  = readl(&iomux->gpr[1]);
	reg &= 0xfffff000;
	reg |= 0x249;
	writel(reg, &iomux->gpr[1]);

    /*****************************************************************************/    
	/*
	 * For EIM General Configuration registers. ( EIM_CS0 : 0x08000000 )
	 *
	 * CS0GCR1:
	 *	GBC = 0; CSREC = 6; 
	 *  DSZ = 1;
            001 : 16 bit port resides on DATA[15:0]
            010 : 16 bit port resides on DATA[31:16]
            011 : 32 bit port resides on DATA[31:0]
            100 :  8 bit port resides on DATA[7:0]
            101 :  8 bit port resides on DATA[15:8]
            110 :  8 bit port resides on DATA[23:16]
            111 :  8 bit port resides on DATA[31:24]
	 *  BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *
	 *	EIM Operation Mode: 
	 *      MUM = 1 : multiplexed       0 : none multiplexed 
	 *      SRD = 0 : Async read        1 : Sync read
	 *      SWR = 0 : Async write       1 : Sync write
	 *
	 * CS0GCR2:
	 *	ADH = 1
	 */
	writel(0x00610089, WEIM_BASE_ADDR);                // EIM_CS0GCR1 : 0x00620081
	//writel(0x01210081, WEIM_BASE_ADDR);                // EIM_CS0GCR1 : 0x01210081
	writel(0x00000000, WEIM_BASE_ADDR + 0x00000004);   // EIM_CS0GCR2 : 0x00000001

	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS0RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS0RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	writel(0x1C022000, WEIM_BASE_ADDR + 0x00000008);   // EIM_CS0RCR1 : 0x1C022000
	writel(0x0000C000, WEIM_BASE_ADDR + 0x0000000C);   // EIM_CS0RCR2 : 0x0000C000

	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS0WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS0WCR2:
	 *	WBCDD = 0
	 */
	writel(0x1404a38e, WEIM_BASE_ADDR + 0x00000010);   // EIM_CS0WCR1 : 0x1404a38e
	writel(0x00000000, WEIM_BASE_ADDR + 0x00000014);   // EIM_CS0WCR2 : 0x00000000


    /*****************************************************************************/
	/*
	 * For EIM General Configuration registers. ( EIM_CS1 : 0x0A000000 )
	 *
	 * CS1GCR1:
	 *	GBC = 0; CSREC = 6; 
	 *  DSZ = 1;
            001 : 16 bit port resides on DATA[15:0]
            010 : 16 bit port resides on DATA[31:16]
            011 : 32 bit port resides on DATA[31:0]
            100 :  8 bit port resides on DATA[7:0]
            101 :  8 bit port resides on DATA[15:8]
            110 :  8 bit port resides on DATA[23:16]
            111 :  8 bit port resides on DATA[31:24]
	 *  BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *
	 *	EIM Operation Mode: 
	 *      MUM = 1 : multiplexed       0 : none multiplexed 
	 *      SRD = 0 : Async read        1 : Sync read
	 *      SWR = 0 : Async write       1 : Sync write
	 *
	 * CS1GCR2:
	 *	ADH = 1
	 */
	writel(0x00610089, WEIM_BASE_ADDR + 0x00000018);   // EIM_CS1GCR1 : 0x00620081
//	writel(0x01210081, WEIM_BASE_ADDR + 0x00000018);   // EIM_CS1GCR1 : 0x01210081

	writel(0x00000000, WEIM_BASE_ADDR + 0x0000001C);   // EIM_CS1GCR2 : 0x00000001

	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS1RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS1RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	writel(0x1C022000, WEIM_BASE_ADDR + 0x00000020);   // EIM_CS1RCR1 : 0x1C022000
	writel(0x0000C000, WEIM_BASE_ADDR + 0x00000024);   // EIM_CS1RCR2 : 0x0000C000

	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS1WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS1WCR2:
	 *	WBCDD = 0
	 */
	writel(0x1404a38e, WEIM_BASE_ADDR + 0x00000028);   // EIM_CS1WCR1 : 0x1404a38e
	writel(0x00000000, WEIM_BASE_ADDR + 0x0000002C);   // EIM_CS1WCR2 : 0x00000000
}

static unsigned gpios_out_low[] = {
	// Output
    T_ANAL_PWON,        // IMX_GPIO_NR(2, 8)

    T_VALVE_N_CTL,      // IMX_GPIO_NR(5,22)
	T_PWR_ALARM_EN,     // IMX_GPIO_NR(1, 5)
    T_SOUND_ONOFF,      // IMX_GPIO_NR(5,24)
    T_RED_LED_CTL,      // IMX_GPIO_NR(7, 2)

	// LCD Control
    LCD_RS, 			// IMX_GPIO_NR(5, 21)
    LCD_MUX_SEL, 		// IMX_GPIO_NR(3, 30)
    LCD_MUX_EN, 		// IMX_GPIO_NR(3, 31)
};

static unsigned gpios_out_high[] = {
	// Output
    T_VALVE_P_CTL,      // IMX_GPIO_NR(5,23)
    T_ALARM_OUT,        // IMX_GPIO_NR(3,23)
    T_PWR_SERIAL_EN,    // IMX_GPIO_NR(2,11) : console power enable
    T_PWR_UART_EN1,     // IMX_GPIO_NR(2,12)
    T_PWR_UART_EN2,     // IMX_GPIO_NR(2,15)
    T_YELLOW_LED_CTL,   // IMX_GPIO_NR(7, 3)
    T_WD_EN,            // IMX_GPIO_NR(7, 4)
    T_WD_WDI,           // IMX_GPIO_NR(7, 5)
	T_PWR_EON,          // IMX_GPIO_NR(5,28)

	// LCD Control
    LCD_RESET,			// IMX_GPIO_NR(3, 29)
    LCD_CS, 			// IMX_GPIO_NR(5, 20)
};

static unsigned gpios_in[] = {
    // Input
    T_RF_PEAK,          // IMX_GPIO_NR(3,26)
    T_CRADLE_DET,       // IMX_GPIO_NR(3,27)
    T_EX12V_DET,        // IMX_GPIO_NR(1, 6)
    T_PWR_ALIVE,        // IMX_GPIO_NR(5,29)
    LCD_SYNC_IN,        // IMX_GPIO_NR(7, 7)
};

static void set_gpios(unsigned *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

static void get_gpios(unsigned *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_iomux_weim();
	setup_iomux_gpio_ctrl();
	setup_iomux_lcd_ctrl();
	
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low,  ARRAY_SIZE(gpios_out_low),  0);
	get_gpios(gpios_in,       ARRAY_SIZE(gpios_in)         );
	
#ifdef CONFIG_SYS_USE_NAND
    setup_gpmi_nand();
#endif

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

//	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
//	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	
	mx6_setup_weimcs();
	
	lcd_init();
	lcd_address_set(0,0,240,320);	
	
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

static char const *board_type = "uninitialized";

int checkboard(void)
{
	puts("Board: HKC [K-ICAD]\n");
	board_type = "kicad";
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	if (0 == getenv("board"))
		setenv("board",board_type);

    // [FALINUX] LCD의 인터페이스 Sync를 위해 아래와 같이 사용해야 함
    imx6_sel_cpu_out();
//	imx6_sel_fb_out();  // 부트로더에서는 프레임버퍼를 위한 설정을 하지 않는다.

	return 0;
}
