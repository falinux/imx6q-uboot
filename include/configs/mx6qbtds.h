/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#define CONFIG_MX6
#define CONFIG_MX6Q

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO


/*
Kernel : arch/arm/tools/mach-types
mx6q_sabreauto          MACH_MX6Q_SABREAUTO     MX6Q_SABREAUTO          3529
mx6q_sabrelite          MACH_MX6Q_SABRELITE     MX6Q_SABRELITE          3769
mx6q_sabresd            MACH_MX6Q_SABRESD       MX6Q_SABRESD            3980
mx6q_arm2               MACH_MX6Q_ARM2          MX6Q_ARM2               3837
mx6sl_arm2              MACH_MX6SL_ARM2         MX6SL_ARM2              4091
mx6q_hdmidongle         MACH_MX6Q_HDMIDONGLE    MX6Q_HDMIDONGLE         4284
mx6sl_evk               MACH_MX6SL_EVK          MX6SL_EVK               4307
hb                      MACH_HB                 HB                      4773
cuboxi                  MACH_CUBOXI             CUBOXI                  4821
mx6dbtds                MACH_MX6D_BTDS          MX6D_BTDS               4830
*/

#define CONFIG_MACH_TYPE	4830

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(12 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	        UART1_BASE

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS           2
#define CONFIG_SF_DEFAULT_CS            (0|(IMX_GPIO_NR(4, 24)<<8))
#define CONFIG_SF_DEFAULT_SPEED         1000000         // 1MHz
#define CONFIG_SF_DEFAULT_MODE          (SPI_MODE_0)
#endif

#define CONFIG_SYS_USE_NAND
#ifdef  CONFIG_SYS_USE_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE      1
#define CONFIG_SYS_NAND_BASE            0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       1

#define CONFIG_MMC
#define CONFIG_CMD_MMC

#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION


/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MXC              1
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_BASE         I2C2_BASE_ADDR
#define CONFIG_SYS_I2C_SPEED	    100000
#define CONFIG_SYS_I2C_SLAVE        0x8
#define CONFIG_I2C_EDID

#define CONFIG_LDO_BYPASS_CHECK

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			    ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		    RGMII
#define CONFIG_ETHPRIME			    "FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_ATHEROS_AR8031

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_MAX_CONTROLLER_COUNT     2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	            (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	            0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

/* Framebuffer and LCD */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN

#define CONFIG_VIDEO_BMP_GZIP
#ifdef CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE  (6 * 1024 * 1024)
#endif

#define CONFIG_BMP_16BPP
#define CONFIG_IPUV3_CLK                260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_CONSOLE_MUX
//#define CONFIG_IMX_HDMI

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	            1
#define CONFIG_BAUDRATE			        115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	            3

#define CONFIG_LOADADDR			        0x12000000
#define CONFIG_SYS_TEXT_BASE	        0x17800000

// MMC
#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
    "fdt_addr=0x18000000\0" \
	"loadaddr=0x12000000\0" \
	"boot_fdt=try\0" \
	"bootdelay=2\0" \
	"bootargs=console=ttymxc0,115200 root=/dev/ram0 rw --no-log initrd=0x1a000000,16M ramdisk=32768" \
	        " video=mxcfb0:dev=lcd,ITST240QV-035S,if=RGB565 fbmem=10M\0 " \
    "bootargs_sata=setenv bootargs console=ttymxc0,115200 root=/dev/sda1 rw --no-log rootfstype=ext4 rootwait" \
            " video=mxcfb0:dev=lcd,ITST240QV-035S,if=RGB565 fbmem=10M\0 " \
    "bootargs_ram=setenv bootargs console=ttymxc0,115200 root=/dev/ram0 rw --no-log initrd=0x1a000000,16M ramdisk=32768" \
            " video=mxcfb0:dev=lcd,ITST240QV-035S,if=RGB565 fbmem=10M\0 " \
    "bootargs_mmc=setenv bootargs console=ttymxc0,115200 root=/dev/mmcblk0p1 rw --no-log rootfstype=ext4 rootdelay=5" \
            " video=mxcfb0:dev=lcd,ITST240QV-035S,if=RGB565 fbmem=10M\0 " \
	"ethaddr=00:FA:14:06:03:20\0" \
	"serverip=199.189.179.120\0" \
	"ipaddr=199.189.179.60\0" \
	"netmask=255.255.255.0\0" \
	"gatewayip=199.189.179.120\0" \
	"ip_dyn=yes\0" \
    "uboot=tftpboot 0x12000000 u-boot.imx; mmc write 0x12000000 2 800\0" \
    "suboot=loady 0x12000000 115200; mmc write 0x12000000 2 800\0" \
    "sfwrite=sf probe; sf erase 0 0x80000; tftpboot 0x12000000 u-boot.imx; sf write 0x12000000 0x400 ${filesize}\0" \
    "kernel=tftpboot 0x10000000 uImage.imx6; nand erase 0x0 0x800000; nand write 0x10000000 0x0 0x800000\0" \
    "dtb=tftpboot 0x12000000 imx6q.dtb; mmc write 0x12000000 3A00 400\0" \
    "ramdisk=tftpboot 0x1a000000 ramdisk.imx6.gz; nand erase 0x800000 0x1000000; nand write 0x1a000000 0x800000 0x1000000\0" \
    "bootdtbram=mmc dev 0; mmcinfo; mmc read 0x12000000 1000 2800; mmc read 0x18000000 3A00 400; mmc read 0x1a000000 4000 8000; bootm 0x12000000 - 0x18000000\0" \
    "bootram=mmc dev 0; mmcinfo; mmc read 0x12000000 1000 2800; mmc read 0x1a000000 4000 8000; bootm 0x12000000\0" \
    "bootnand=nand read 0x10000000 0x0 0x00800000; nand read 0x1a000000 0x00800000 0x01000000; bootm 0x10000000\0 " \
    "bootmmc=mmc dev 0; mmcinfo; mmc read 0x12000000 1000 2800; bootm 0x12000000\0" \
    "bootcmd=run bootargs_ram bootnand\0" \

//// SPI
/*
#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc0\0" \
	"fdt_addr=0x11000000\0" \
	"fdt_high=0xffffffff\0" \
	"ram_addr=0x1a000000\0" \
	"ipaddr=192.168.10.247\0" \
	"netmask=255.255.0.0\0" \
	"serverip=192.168.10.132\0 " \
	"ethaddr=00:fa:12:34:56:89\0 " \
	"bootargs=console=ttymxc0,115200 console=ttymxc0,115200 root=/dev/ram0 rw --no-log initrd=0x1a000000,16M ramdisk=32768\0" \
    "bootargs_ram=setenv bootargs console=ttymxc0,115200 root=/dev/ram0 rw --no-log initrd=0x1a000000,16M ramdisk=32768\0" \
	"upkernel=tftpboot 0x10000000 uImage.imx6; tftpboot 0x1a000000 ramdisk.imx6-1.0-32M.gz; bootm 0x10000000\0 " \
	"nandboot=nand read 0x10000000 0x0 0x00800000; nand read 0x1a000000 0x00800000 0x00800000; bootm 0x10000000\0 " \
    "uboot=tftpboot 0x10000000 u-boot.imx; sf probe; sf erase 0x0 0xC0000; sf write 0x10000000 0x400 0xC0000\0" \
    "kernel=tftpboot 0x10000000 uImage.imx6; nand erase 0x0 0x800000; nand write 0x10000000 0x0 0x800000\0" \
    "ramdisk=tftpboot 0x1a000000 ramdisk.imx6-1.0-32M.gz; nand erase 0x800000 0x1000000; nand write 0x1a000000 0x800000 0x1000000\0" \
    "bootnand=nand read 0x10000000 0x0 0x00800000; nand read 0x1a000000 0x00800000 0x01000000; bootm 0x10000000\0 " \
	"bootcmd=run bootargs_ram bootnand\0" \
	"loadsplash=if sf probe ; then sf read ${splashimage} c2000 ${splashsize} ; fi\0" \
*/

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	            "BTDS > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE	            1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	            48
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START        0x10000000
#define CONFIG_SYS_MEMTEST_END	        0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH      0x10800000

#define CONFIG_SYS_LOAD_ADDR	        CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	        1
#define PHYS_SDRAM                      MMDC0_ARB_BASE_ADDR
//#define PHYS_SDRAM_SIZE                 (2u * 1024 * 1024 * 1024)
#define PHYS_SDRAM_SIZE                 (512u * 1024 * 1024)

#define CONFIG_RESET_CAUSE_ADDR	       (PHYS_SDRAM + 0x80)

#define CONFIG_SYS_SDRAM_BASE	        PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR        IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE        IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			       (8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
/* #define CONFIG_ENV_IS_IN_SPI_FLASH */

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET               (512 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV           0
#define	CONFIG_SYS_MMC_ENV_PART          0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		       (768 * 1024)
#define CONFIG_ENV_SECT_SIZE		   (64 * 1024)
#define CONFIG_ENV_SPI_BUS		        CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		        CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		        CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		    CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET 			    (8 << 20)
#define CONFIG_ENV_SECT_SIZE        	(128 << 10)
#define CONFIG_ENV_SIZE        			CONFIG_ENV_SECT_SIZE
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#define CONFIG_CMD_BMP

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_BOARD_LATE_INIT


#define CONFIG_CMD_ELF

#define CONFIG_USB_GADGET
//#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	    2

/* Netchip IDs */
#define CONFIG_G_DNL_VENDOR_NUM         0x0525
#define CONFIG_G_DNL_PRODUCT_NUM        0xa4a5
#define CONFIG_G_DNL_MANUFACTURER       "BTDS"

#endif	       /* __CONFIG_H */

