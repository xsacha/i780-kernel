/*
 * linux/arch/arm/mach-pxa/i780.c
 *
 * Support for the Samsung i780 Mobile Phone
 *
 * Copyright (C) 2006 Marvell International Ltd.
 *
 * 2007-09-04: eric miao <eric.miao@marvell.com>
 *             rewrite to align with latest kernel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/gpio.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <mach/pxafb.h>
#include <mach/mmc.h>
#include <mach/ohci.h>
#include <mach/i2c.h>
#include <mach/mfp-pxa300.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa3xx_nand.h>

#include "devices.h"
#include "generic.h"

int wm9713_irq;

int lcd_id;
int lcd_orientation;

void i780_pxa300_init(void);

#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULE)
static struct platform_pwm_backlight_data i780_backlight_data = {
	.pwm_id		= 3,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 10000,
};

static struct platform_device i780_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent = &pxa27x_device_pwm1.dev,
		.platform_data	= &i780_backlight_data,
	},
};

static struct pxafb_mode_info i780_mode = {
		.pixclock	= 110000,
		.xres		= 320,
		.yres		= 320,
		.bpp		= 16,
		.hsync_len	= 16,
		.left_margin	= 24,
		.right_margin	= 24,
		.vsync_len	= 2,
		.upper_margin	= 3,
		.lower_margin	= 1,
		.sync		= 0,
};

static struct pxafb_mach_info i780_lcd_info = {
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static void __init i780_init_lcd(void)
{
	platform_device_register(&i780_backlight_device);
	i780_lcd_info.modes = &i780_mode;

	set_pxa_fb_info(&i780_lcd_info);
}
#else
static inline void i780_init_lcd(void) {}
#endif

#if defined(CONFIG_MMC)

static int i780_mci_init(struct device *dev,
			     irq_handler_t i780_detect_int,
			     void *data)
{
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(105);
	gpio_cd = 105;

	/*
	 * setup GPIO for I780 MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, i780_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static void i780_mci_exit(struct device *dev, void *data)
{
	int cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(105);
	gpio_cd = 105;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data i780_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= i780_mci_init,
	.exit		= i780_mci_exit,
};

static void __init i780_init_mmc(void)
{
	 pxa_set_mci_info(&i780_mci_platform_data);
}
#else
static inline void i780_init_mmc(void) {}
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int i780_matrix_key_map[] = {
	/* KEY(row, col, key_code) */

/* Samsung I780 QWERTY Keyboard */
/* 1st row */
KEY(0, 0, KEY_Q), KEY(7, 1, KEY_W), KEY(2, 0, KEY_E), KEY(3, 0, KEY_R), KEY(4, 0, KEY_T),
KEY(0, 4, KEY_Y), KEY(1, 4, KEY_U), KEY(2, 4, KEY_I), KEY(3, 4, KEY_O), KEY(4, 4, KEY_P),
/* 2nd row */
KEY(0, 1, KEY_A), KEY(7, 2, KEY_S), KEY(2, 1, KEY_D), KEY(3, 1, KEY_F), KEY(4, 1, KEY_G),
KEY(0, 5, KEY_H), KEY(1, 5, KEY_J), KEY(2, 5, KEY_K), KEY(3, 5, KEY_L), KEY(4, 5, KEY_BACKSPACE),
/* 3rd row */
KEY(0, 2, KEY_LEFTCTRL), KEY(1, 2, KEY_Z), KEY(2, 2, KEY_X), KEY(3, 2, KEY_C), KEY(4, 2, KEY_V),
KEY(0, 6, KEY_B), KEY(1, 6, KEY_N), KEY(2, 6, KEY_M), KEY(3, 6, KEY_DOT), KEY(4, 6, KEY_ENTER),
/* 4th row */
KEY(0, 3, KEY_LEFTSHIFT), KEY(1, 3, KEY_LEFTALT), KEY(2, 3, KEY_0), KEY(3, 3, KEY_SPACE),
KEY(4, 3, KEY_COMMA), KEY(7, 6, KEY_SLASH), /* Message */ KEY(5, 1, KEY_TAB), /* GPS */

/* Volume Keys */
KEY(1, 0, KEY_UP),
KEY(1, 1, KEY_DOWN),

/* Left Softkey */	/* Windows Key */	/* OK */	/* Right Softkey */
KEY(5, 4, KEY_MINUS), KEY(5, 2, KEY_HOME), KEY(5, 3, KEY_END),  KEY(5, 6, KEY_EQUAL),
KEY(5, 5, KEY_PAGEUP), 		KEY(6, 4, KEY_ENTER),		KEY(7, 0, KEY_PAGEDOWN),	
/* Green Key */			 /* Center */			/* Red Key */

/* Camera */
KEY(7, 3, KEY_1),

/* Not used (Numpad) */
KEY(5, 0, KEY_1),
KEY(6, 0, KEY_2),
KEY(6, 1, KEY_3),
KEY(6, 2, KEY_4),
KEY(6, 5, KEY_5),
KEY(6, 6, KEY_6),
KEY(7, 4, KEY_7),
KEY(7, 5, KEY_8),

};

static struct pxa27x_keypad_platform_data i780_keypad_info = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 7,
	.matrix_key_map		= i780_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(i780_matrix_key_map),

	.enable_rotary0		= 1,
	.rotary0_up_key		= KEY_UP,
	.rotary0_down_key	= KEY_DOWN,

	.debounce_interval	= 30,
};

static void __init i780_init_keypad(void)
{
	pxa_set_keypad_info(&i780_keypad_info);
}
#else
static inline void i780_init_keypad(void) {}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct mtd_partition i780_nand_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x060000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x060000,
		.size        = 0x200000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x0260000,
		.size        = 0x3000000,     /* 48M - rootfs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x3260000,
		.size        = 0x3d40000,
	},
	[4] = {
		.name        = "BBT",
		.offset      = 0x6FA0000,
		.size        = 0x80000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTE: we reserve some blocks at the end of the NAND flash for
	 * bad block management, and the max number of relocation blocks
	 * differs on different platforms. Please take care with it when
	 * defining the partition table.
	 */
};

static struct pxa3xx_nand_platform_data i780_nand_info = {
	.enable_arbiter	= 1,
	.parts		= i780_nand_partitions,
	.nr_parts	= ARRAY_SIZE(i780_nand_partitions),
};

static void __init i780_init_nand(void)
{
	pxa3xx_set_nand_info(&i780_nand_info);
}
#else
static inline void i780_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data i780_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT1 | ENABLE_PORT2 |
			  POWER_CONTROL_LOW | POWER_SENSE_LOW,
};

static void __init i780_init_ohci(void)
{
	pxa_set_ohci_info(&i780_ohci_info);
}
#else
static inline void i780_init_ohci(void) {}
#endif /* CONFIG_USB_OHCI_HCD || CONFIG_USB_OHCI_HCD_MODULE */

static void __init i780_init(void)
{
	i780_pxa300_init();
	pxa_set_ac97_info(NULL);
	i780_init_lcd();
	i780_init_mmc();
	i780_init_keypad();
	i780_init_nand();
	i780_init_ohci();
}

MACHINE_START(I780, "Samsung i780 Mobile Phone")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= i780_init,
MACHINE_END

/* PXA300/PXA310 common configurations */
static mfp_cfg_t common_mfp_cfg[] __initdata = {
	/* LCD */
	GPIO54_LCD_LDD_0,
	GPIO55_LCD_LDD_1,
	GPIO56_LCD_LDD_2,
	GPIO57_LCD_LDD_3,
	GPIO58_LCD_LDD_4,
	GPIO59_LCD_LDD_5,
	GPIO60_LCD_LDD_6,
	GPIO61_LCD_LDD_7,
	GPIO62_LCD_LDD_8,
	GPIO63_LCD_LDD_9,
	GPIO64_LCD_LDD_10,
	GPIO65_LCD_LDD_11,
	GPIO66_LCD_LDD_12,
	GPIO67_LCD_LDD_13,
	GPIO68_LCD_LDD_14,
	GPIO69_LCD_LDD_15,
	GPIO70_LCD_LDD_16,
	GPIO71_LCD_LDD_17,
	GPIO72_LCD_FCLK,
	GPIO73_LCD_LCLK,
	GPIO74_LCD_PCLK,
	GPIO75_LCD_BIAS,
	GPIO76_LCD_VSYNC,
	GPIO127_LCD_CS_N,
	GPIO20_PWM3_OUT,	/* backlight */

	/* BTUART */
	GPIO111_UART2_RTS,
	GPIO112_UART2_RXD | MFP_LPM_EDGE_FALL,
	GPIO113_UART2_TXD,
	GPIO114_UART2_CTS | MFP_LPM_EDGE_BOTH,

	/* STUART */
	GPIO109_UART3_TXD,
	GPIO110_UART3_RXD | MFP_LPM_EDGE_FALL,

	/* AC97 */
	GPIO23_AC97_nACRESET,
	GPIO24_AC97_SYSCLK,
	GPIO29_AC97_BITCLK,
	GPIO25_AC97_SDATA_IN_0,
	GPIO27_AC97_SDATA_OUT,
	GPIO28_AC97_SYNC,

	/* SSP3 */
	GPIO91_SSP3_SCLK,
	GPIO92_SSP3_FRM,
	GPIO93_SSP3_TXD,
	GPIO94_SSP3_RXD,

	/* WM9713 IRQ */
	GPIO26_GPIO,

	/* Keypad */
	GPIO107_KP_DKIN_0 | MFP_LPM_EDGE_BOTH,
	GPIO108_KP_DKIN_1 | MFP_LPM_EDGE_BOTH,
	GPIO115_KP_MKIN_0 | MFP_LPM_EDGE_BOTH,
	GPIO116_KP_MKIN_1 | MFP_LPM_EDGE_BOTH,
	GPIO117_KP_MKIN_2 | MFP_LPM_EDGE_BOTH,
	GPIO118_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,
	GPIO119_KP_MKIN_4 | MFP_LPM_EDGE_BOTH,
	GPIO120_KP_MKIN_5 | MFP_LPM_EDGE_BOTH,
	GPIO2_2_KP_MKIN_6 | MFP_LPM_EDGE_BOTH,
	GPIO3_2_KP_MKIN_7 | MFP_LPM_EDGE_BOTH,
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
	GPIO124_KP_MKOUT_3,
	GPIO125_KP_MKOUT_4,
	GPIO4_2_KP_MKOUT_5,
	GPIO5_2_KP_MKOUT_6,
	GPIO6_2_KP_MKOUT_7,

	/* MMC1 */
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1 | MFP_LPM_EDGE_BOTH,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	GPIO8_MMC1_CMD,	/* CMD0 for slot 0 */
	GPIO15_GPIO,	/* CMD1 default as GPIO for slot 0 */

	/* MMC2 */
	GPIO9_MMC2_DAT0,
	GPIO10_MMC2_DAT1 | MFP_LPM_EDGE_BOTH,
	GPIO11_MMC2_DAT2,
	GPIO12_MMC2_DAT3,
	GPIO13_MMC2_CLK,
	GPIO14_MMC2_CMD,

	/* USB Host */
	GPIO0_2_USBH_PEN,
	GPIO1_2_USBH_PWR,

	/* Standard I2C */
	GPIO21_I2C_SCL,
	GPIO22_I2C_SDA,
};

static mfp_cfg_t pxa310_mfp_cfg[] __initdata = {
	/* FFUART */
	GPIO99_UART1_RXD | MFP_LPM_EDGE_FALL,
	GPIO100_UART1_TXD,
	GPIO101_UART1_CTS,
	GPIO106_UART1_RTS,

	/* MMC3 */
	GPIO7_2_MMC3_DAT0,
	GPIO8_2_MMC3_DAT1 | MFP_LPM_EDGE_BOTH,
	GPIO9_2_MMC3_DAT2,
	GPIO10_2_MMC3_DAT3,
	GPIO103_MMC3_CLK,
	GPIO105_MMC3_CMD,
};

#define NUM_LCD_DETECT_PINS	7

static int lcd_detect_pins[] __initdata = {
	MFP_PIN_GPIO71,	/* LCD_LDD_17 - ORIENT */
	MFP_PIN_GPIO70, /* LCD_LDD_16 - LCDID[5] */
	MFP_PIN_GPIO75, /* LCD_BIAS   - LCDID[4] */
	MFP_PIN_GPIO73, /* LCD_LCLK   - LCDID[3] */
	MFP_PIN_GPIO72, /* LCD_FCLK   - LCDID[2] */
	MFP_PIN_GPIO127,/* LCD_CS_N   - LCDID[1] */
	MFP_PIN_GPIO76, /* LCD_VSYNC  - LCDID[0] */
};

static void __init i780_detect_lcd_panel(void)
{
	unsigned long mfpr_save[NUM_LCD_DETECT_PINS];
	int i, gpio, id = 0;

	/* save the original MFP settings of these pins and configure
	 * them as GPIO Input, DS01X, Pull Neither, Edge Clear
	 */
	for (i = 0; i < NUM_LCD_DETECT_PINS; i++) {
		mfpr_save[i] = pxa3xx_mfp_read(lcd_detect_pins[i]);
		pxa3xx_mfp_write(lcd_detect_pins[i], 0x8440);
	}

	for (i = 0; i < NUM_LCD_DETECT_PINS; i++) {
		id = id << 1;
		gpio = mfp_to_gpio(lcd_detect_pins[i]);
		gpio_direction_input(gpio);

		if (gpio_get_value(gpio))
			id = id | 0x1;
	}

	/* lcd id, flush out bit 1 */
	lcd_id = id & 0x3d;

	/* lcd orientation, portrait or landscape */
	lcd_orientation = (id >> 6) & 0x1;

	/* restore the original MFP settings */
	for (i = 0; i < NUM_LCD_DETECT_PINS; i++)
		pxa3xx_mfp_write(lcd_detect_pins[i], mfpr_save[i]);
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static struct pca953x_platform_data gpio_exp[] = {
	[0] = {
		.gpio_base	= 128,
	},
	[1] = {
		.gpio_base	= 144,
	},
};

static struct i2c_board_info i780_i2c_board_info[] = {
	{
		.type		= "pca9539",
		.addr		= 0x74,
		.platform_data	= &gpio_exp[0],
		.irq		= IRQ_GPIO(18),
	}, {
		.type		= "pca9539",
		.addr		= 0x75,
		.platform_data	= &gpio_exp[1],
		.irq		= IRQ_GPIO(19),
	},
};

static void __init i780_init_i2c(void)
{
	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(i780_i2c_board_info));
}
#else
static inline void i780_init_i2c(void) {}
#endif

void __init i780_pxa300_init(void)
{
	/* initialize MFP */
	// pxa3xx_mfp_config(ARRAY_AND_SIZE(common_mfp_cfg));

	/* detect LCD panel */
	i780_detect_lcd_panel();

	/* WM9713 IRQ */
	wm9713_irq = mfp_to_gpio(MFP_PIN_GPIO23);

	i780_init_i2c();

	// pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_mfp_cfg));

}
