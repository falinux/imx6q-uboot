/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL   (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
#define USDHC_PAD_CTRL  (PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
#define ENET_PAD_CTRL   (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define SPI_PAD_CTRL    (PAD_CTL_HYS         | PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |  PAD_CTL_DSE_40ohm  | PAD_CTL_SRE_FAST)
#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_SRE_FAST)
#define WEIM_PAD_CTRL   (PAD_CTL_SRE_FAST    | PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define LCD_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define OUTPUT_40OHM    (PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm)
#define GPMI_PAD_CTRL0  (PAD_CTL_PKE         | PAD_CTL_PUE       | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1  (PAD_CTL_DSE_40ohm   | PAD_CTL_SPEED_MED | PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2  (GPMI_PAD_CTRL0      | GPMI_PAD_CTRL1)
#define GPIO_PAD_CTRL   (PAD_CTL_DSE_34ohm)


int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C MAC chip*/
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO1_IO06 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};

iomux_v3_cfg_t const usdhc3_pads[] = {
    MX6_PAD_SD3_CLK__SD3_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_CMD__SD3_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT0__SD3_DATA0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT1__SD3_DATA1  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT2__SD3_DATA2  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT3__SD3_DATA3  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* CD */
};

iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	MX6_PAD_KEY_COL2__GPIO4_IO10   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi3_pads[] = {
	/* SS2 */
	MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__GPIO4_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi5_pads[] = {
	/* SS5 */
	MX6_PAD_SD1_CLK__ECSPI5_SCLK	| MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_SD1_CMD__ECSPI5_MOSI 	| MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_SD1_DAT0__ECSPI5_MISO	| MUX_PAD_CTRL(SPI_PAD_CTRL), 
	MX6_PAD_SD1_DAT1__GPIO1_IO17 	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_DAT2__GPIO1_IO19	| MUX_PAD_CTRL(NO_PAD_CTRL), 
	MX6_PAD_SD1_DAT3__GPIO1_IO21	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_spi(void)
{
//	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
}

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

iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		    /* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		    /* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC3_BASE_ADDR},
};

#define USDHC3_CD_GPIO	IMX_GPIO_NR(7, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif

#ifdef CONFIG_SYS_USE_NAND
static iomux_v3_cfg_t gpmi_pads[] = {
	MX6_PAD_NANDF_CS0__NAND_CE0_B    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_CLE__NAND_CLE      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_ALE__NAND_ALE      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_CMD__NAND_RE_B       | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_SD4_CLK__NAND_WE_B       | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NANDF_WP_B__NAND_WP_B    | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
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

int mx6_rgmii_rework(struct phy_device *phydev)
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
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
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
//	imx_iomux_v3_setup_multiple_pads( rgb_pads, ARRAY_SIZE(rgb_pads));
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

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");

	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#ifdef CONFIG_SYS_USE_NAND
    setup_gpmi_nand();
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{NULL,	 0},
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
	return 0;
}

int checkboard(void)
{
	puts("Board: MX6-BTDS\n");
	return 0;
}
