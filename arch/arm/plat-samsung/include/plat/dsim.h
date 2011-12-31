/* linux/arm/arch/plat-s5p/include/plat/dsim.h
 *
 * Platform data header for Samsung SoC MIPI-DSIM.
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * InKi Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _DSIM_H
#define _DSIM_H

#include <linux/device.h>
#include <linux/fb.h>

#include <linux/regulator/consumer.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define PANEL_NAME_SIZE		(32)

#ifdef CONFIG_CPU_S5PV310
#define MIPI_DSIM_NUM		2
#else
#define MIPI_DSIM_NUM		1
#endif

#define to_dsim_plat(d)		(to_platform_device(d)->dev.platform_data)

enum mipi_dsim_interface_type {
	DSIM_COMMAND,
	DSIM_VIDEO
};

enum mipi_dsim_virtual_ch_no {
	DSIM_VIRTUAL_CH_0,
	DSIM_VIRTUAL_CH_1,
	DSIM_VIRTUAL_CH_2,
	DSIM_VIRTUAL_CH_3
};

enum mipi_dsim_burst_mode_type {
	DSIM_NON_BURST_SYNC_EVENT,
	DSIM_NON_BURST_SYNC_PULSE = 2,
	DSIM_BURST,
	DSIM_NON_VIDEO_MODE
};

enum mipi_dsim_no_of_data_lane {
	DSIM_DATA_LANE_1,
	DSIM_DATA_LANE_2,
	DSIM_DATA_LANE_3,
	DSIM_DATA_LANE_4
};

enum mipi_dsim_byte_clk_src {
	DSIM_PLL_OUT_DIV8,
	DSIM_EXT_CLK_DIV8,
	DSIM_EXT_CLK_BYPASS
};

enum mipi_dsim_pixel_format {
	DSIM_CMD_3BPP,
	DSIM_CMD_8BPP,
	DSIM_CMD_12BPP,
	DSIM_CMD_16BPP,
	DSIM_VID_16BPP_565,
	DSIM_VID_18BPP_666PACKED,
	DSIM_18BPP_666LOOSELYPACKED,
	DSIM_24BPP_888
};

/**
 * struct mipi_dsim_config - interface for configuring mipi-dsi controller.
 *
 * @auto_flush: enable or disable Auto flush of MD FIFO using VSYNC pulse.
 * @eot_disable: enable or disable EoT packet in HS mode.
 * @auto_vertical_cnt: specifies auto vertical count mode.
 *	in Video mode, the vertical line transition uses line counter
 *	configured by VSA, VBP, and Vertical resolution.
 *	If this bit is set to '1', the line counter does not use VSA and VBP
 *	registers.(in command mode, this variable is ignored)
 * @hse: set horizontal sync event mode.
 *	In VSYNC pulse and Vporch area, MIPI DSI master transfers only HSYNC
 *	start packet to MIPI DSI slave at MIPI DSI spec1.1r02.
 *	this bit transfers HSYNC end packet in VSYNC pulse and Vporch area
 *	(in mommand mode, this variable is ignored)
 * @hfp: specifies HFP disable mode.
 *	if this variable is set, DSI master ignores HFP area in VIDEO mode.
 *	(in command mode, this variable is ignored)
 * @hbp: specifies HBP disable mode.
 *	if this variable is set, DSI master ignores HBP area in VIDEO mode.
 *	(in command mode, this variable is ignored)
 * @hsa: specifies HSA disable mode.
 *	if this variable is set, DSI master ignores HSA area in VIDEO mode.
 *	(in command mode, this variable is ignored)
 * @e_interface: specifies interface to be used.(CPU or RGB interface)
 * @e_virtual_ch: specifies virtual channel number that main or
 *	sub diaplsy uses.
 * @e_pixel_format: specifies pixel stream format for main or sub display.
 * @e_burst_mode: selects Burst mode in Video mode.
 *	in Non-burst mode, RGB data area is filled with RGB data and NULL
 *	packets, according to input bandwidth of RGB interface.
 *	In Burst mode, RGB data area is filled with RGB data only.
 * @e_no_data_lane: specifies data lane count to be used by Master.
 * @e_byte_clk: select byte clock source. (it must be DSIM_PLL_OUT_DIV8)
 *	DSIM_EXT_CLK_DIV8 and DSIM_EXT_CLK_BYPASSS are not supported.
 * @pll_stable_time: specifies the PLL Timer for stability of the ganerated
 *	clock(System clock cycle base)
 *	if the timer value goes to 0x00000000, the clock stable bit of
status
 *	and interrupt register is set.
 * @esc_clk: specifies escape clock frequency for getting the escape clock
 *	prescaler value.
 * @stop_holding_cnt: specifies the interval value between transmitting
 *	read packet(or write "set_tear_on" command) and BTA request.
 *	after transmitting read packet or write "set_tear_on" command,
 *	BTA requests to D-PHY automatically. this counter value specifies
 *	the interval between them.
 * @bta_timeout: specifies the timer for BTA.
 *	this register specifies time out from BTA request to change
 *	the direction with respect to Tx escape clock.
 * @rx_timeout: specifies the timer for LP Rx mode timeout.
 *	this register specifies time out on how long RxValid deasserts,
 *	after RxLpdt asserts with respect to Tx escape clock.
 *	- RxValid specifies Rx data valid indicator.
 *	- RxLpdt specifies an indicator that D-PHY is under RxLpdt mode.
 *	- RxValid and RxLpdt specifies signal from D-PHY.
 * @lcd_panel_info: pointer for lcd panel specific structure.
 *	this structure specifies width, height, timing and polarity and so
on.
 * @mipi_ddi_pd: pointer to lcd panel platform data.
 */
struct mipi_dsim_config {
	unsigned char auto_flush;
	unsigned char eot_disable;

	unsigned char auto_vertical_cnt;
	unsigned char hse;
	unsigned char hfp;
	unsigned char hbp;
	unsigned char hsa;

	enum mipi_dsim_interface_type	e_interface;
	enum mipi_dsim_virtual_ch_no	e_virtual_ch;
	enum mipi_dsim_pixel_format	e_pixel_format;
	enum mipi_dsim_burst_mode_type	e_burst_mode;
	enum mipi_dsim_no_of_data_lane	e_no_data_lane;
	enum mipi_dsim_byte_clk_src	e_byte_clk;

	/*
	 * ===========================================
	 * |    P    |    M    |    S    |    MHz    |
	 * -------------------------------------------
	 * |    3    |   100   |    3    |    100    |
	 * |    3    |   100   |    2    |    200    |
	 * |    3    |    63   |    1    |    252    |
	 * |    4    |   100   |    1    |    300    |
	 * |    4    |   110   |    1    |    330    |
	 * |   12    |   350   |    1    |    350    |
	 * |    3    |   100   |    1    |    400    |
	 * |    4    |   150   |    1    |    450    |
	 * |    3    |   118   |    1    |    472    |
	 * |   12    |   250   |    0    |    500    |
	 * |    4    |   100   |    0    |    600    |
	 * |    3    |    81   |    0    |    648    |
	 * |    3    |    88   |    0    |    704    |
	 * |    3    |    90   |    0    |    720    |
	 * |    3    |   100   |    0    |    800    |
	 * |   12    |   425   |    0    |    850    |
	 * |    4    |   150   |    0    |    900    |
	 * |   12    |   475   |    0    |    950    |
	 * |    6    |   250   |    0    |   1000    |
	 * -------------------------------------------
	 */
	unsigned char p;
	unsigned short m;
	unsigned char s;

	unsigned int pll_stable_time;
	unsigned long esc_clk;

	unsigned short stop_holding_cnt;
	unsigned char bta_timeout;
	unsigned short rx_timeout;

	void *lcd_panel_info;
	void *dsim_ddi_pd;
};

/* for RGB Interface */
struct mipi_dsi_lcd_timing {
	int	left_margin;
	int	right_margin;
	int	upper_margin;
	int	lower_margin;
	int	hsync_len;
	int	vsync_len;
};

/* for CPU Interface */
struct mipi_dsi_cpu_timing {
	unsigned int	cs_setup;
	unsigned int	wr_setup;
	unsigned int	wr_act;
	unsigned int	wr_hold;
};

struct mipi_dsi_lcd_size {
	unsigned int	width;
	unsigned int	height;
};

struct mipi_dsim_lcd_config {
	enum mipi_dsim_interface_type e_interface;
	unsigned int parameter[3];

	/* lcd panel info */
	struct	mipi_dsi_lcd_timing rgb_timing;
	struct	mipi_dsi_cpu_timing cpu_timing;
	struct	mipi_dsi_lcd_size lcd_size;
	/* platform data for lcd panel based on MIPI-DSI. */
	void *mipi_ddi_pd;
};

/**
 * struct mipi_dsim_device - global interface for mipi-dsi driver.
 *
 * @dev: driver model representation of the device.
 * @clock: pointer to MIPI-DSI clock of clock framework.
 * @irq: interrupt number to MIPI-DSI controller.
 * @reg_base: base address to memory mapped SRF of MIPI-DSI controller.
 *	(virtual address)
 * @pd: pointer to MIPI-DSI driver platform data.
 * @dsim_info: infomation for configuring mipi-dsi controller.
 * @master_ops: callbacks to mipi-dsi operations.
 * @lcd_info: pointer to mipi_lcd_info structure.
 * @state: specifies status of MIPI-DSI controller.
 *	the status could be RESET, INIT, STOP, HSCLKEN and ULPS.
 * @resume_complete: indicates whether resume operation is completed or
not.
 * @data_lane: specifiec enabled data lane number.
 *	this variable would be set by driver according to e_no_data_lane
 *	automatically.
 * @e_clk_src: select byte clock source.
 *	this variable would be set by driver according to e_byte_clock
 *	automatically.
 * @hs_clk: HS clock rate.
 *	this variable would be set by driver automatically.
 * @byte_clk: Byte clock rate.
 *	this variable would be set by driver automatically.
 * @escape_clk: ESCAPE clock rate.
 *	this variable would be set by driver automatically.
 * @freq_band: indicates Bitclk frequency band for D-PHY global timing.
 *	Serial Clock(=ByteClk X 8)		FreqBand[3:0]
 *		~ 99.99 MHz				0000
 *		100 ~ 119.99 MHz			0001
 *		120 ~ 159.99 MHz			0010
 *		160 ~ 199.99 MHz			0011
 *		200 ~ 239.99 MHz			0100
 *		140 ~ 319.99 MHz			0101
 *		320 ~ 389.99 MHz			0110
 *		390 ~ 449.99 MHz			0111
 *		450 ~ 509.99 MHz			1000
 *		510 ~ 559.99 MHz			1001
 *		560 ~ 639.99 MHz			1010
 *		640 ~ 689.99 MHz			1011
 *		690 ~ 769.99 MHz			1100
 *		770 ~ 869.99 MHz			1101
 *		870 ~ 949.99 MHz			1110
 *		950 ~ 1000 MHz				1111
 *	this variable would be calculated by driver automatically.
 */
struct mipi_dsim_device {
	struct device *dev;
	struct resource *res;
	struct clk *clock;
	unsigned int irq;
	void __iomem *reg_base;

	struct s5p_platform_mipi_dsim *pd;
	struct mipi_dsim_config *dsim_config;
	struct mipi_dsim_ddi *dsim_ddi;

	unsigned int state;
	unsigned int resume_complete;
	unsigned int data_lane;
	enum mipi_dsim_byte_clk_src e_clk_src;
	unsigned long hs_clk;
	unsigned long byte_clk;
	unsigned long escape_clk;
	unsigned char freq_band;
	unsigned char id;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

/**
 * struct s5p_platform_mipi_dsim - interface to platform data
 *	for mipi-dsi driver.
 *
 * @lcd_panel_name: specifies lcd panel name registered to mipi-dsi driver.
 *	lcd panel driver searched would be actived.
 * @dsim_config: pointer of structure for configuring mipi-dsi controller.
 * @dsim_lcd_info: pointer to structure for configuring
 *	mipi-dsi based lcd panel.
 * @mipi_power: callback pointer for enabling or disabling mipi power.
 * @part_reset: callback pointer for reseting mipi phy.
 * @init_d_phy: callback pointer for enabing d_phy of dsi master.
 * @get_fb_frame_done: callback pointer for getting frame done status of
the
 *	display controller(FIMD).
 * @trigger: callback pointer for triggering display controller(FIMD)
 *	in case of CPU mode.
 * @delay_for_stabilization: specifies stable time.
 *	this delay needs when writing data on SFR
 *	after mipi mode became LP mode.
 */
struct s5p_platform_mipi_dsim {
	const char	clk_name[16];
	char	lcd_panel_name[PANEL_NAME_SIZE];

	struct mipi_dsim_config *dsim_config;
	struct mipi_dsim_lcd_config *dsim_lcd_config;

	unsigned int delay_for_stabilization;

	int (*mipi_power) (struct mipi_dsim_device *dsim, unsigned int
		enable);
	int (*part_reset) (struct mipi_dsim_device *dsim);
	int (*init_d_phy) (struct mipi_dsim_device *dsim);
	int (*get_fb_frame_done) (struct fb_info *info);
	void (*trigger) (struct fb_info *info);
};
/**
 * struct mipi_dsim_master_ops - callbacks to mipi-dsi operations.
 *
 * @cmd_write: transfer command to lcd panel at LP mode.
 * @cmd_read: read command from rx register.
 * @get_dsim_frame_done: get the status that all screen data have been
 *	transferred to mipi-dsi.
 * @clear_dsim_frame_done: clear frame done status.
 * @change_dsim_transfer_mode: change transfer mode to LP or HS mode.
 *	- LP mode is used when commands data ard transferred to lcd panel.
 * @get_fb_frame_done: get frame done status of display controller.
 * @trigger: trigger display controller.
 *	- this one would be used only in case of CPU mode.
 */

struct mipi_dsim_master_ops {
	int (*cmd_write) (struct mipi_dsim_device *dsim, unsigned int
		data_id,
		unsigned int data0, unsigned int data1);
	int (*cmd_read) (struct mipi_dsim_device *dsim, unsigned int
		data_id,
		unsigned int data0, unsigned int data1);
	int (*get_dsim_frame_done) (struct mipi_dsim_device *dsim);
	int (*clear_dsim_frame_done) (struct mipi_dsim_device *dsim);

	int (*change_dsim_transfer_mode) (struct mipi_dsim_device *dsim,
						unsigned int mode);

	int (*get_fb_frame_done) (struct fb_info *info);
	void (*trigger) (struct fb_info *info);
};

/**
 * device structure for mipi-dsi based lcd panel.
 *
 * @dev: driver model representation of the device.
 * @id: id of device registered and when device is registered
 *	id would be counted.
 * @modalias: name of the driver to use with this device, or an
 *	alias for that name.
 * @mipi_lcd_drv: pointer of mipi_lcd_driver.
 * @master: pointer to dsim_device.
 */
struct mipi_dsim_lcd_device {
	struct	device	dev;
	int	id;
	char	modalias[64];

	struct mipi_dsim_lcd_driver	*dsim_drv;
	struct mipi_dsim_device		*master;
};
/**
 * driver structure for mipi-dsi based lcd panel.
 *
 * this structure should be registered by lcd panel driver.
 * mipi-dsi driver seeks lcd panel registered through name field
 * and calls these callback functions in appropriate time.
 */

struct mipi_dsim_lcd_driver {
	char		*name;

	int	(*probe)(struct mipi_dsim_lcd_device *dsim_dev);
	int	(*suspend)(struct mipi_dsim_lcd_device *dsim_dev);
	int	(*displayon)(struct mipi_dsim_lcd_device *dsim_dev);
	int	(*resume)(struct mipi_dsim_lcd_device *dsim_dev);
};

/**
 * register mipi_dsim_lcd_driver object defined by lcd panel driver
 * to mipi-dsi driver.
 */
extern int s5p_mipi_dsi_register_lcd_driver(struct mipi_dsim_lcd_driver
			*lcd_drv);

extern int s5p_dsim_part_reset(struct mipi_dsim_device *dsim);
extern int s5p_dsim_init_d_phy(struct mipi_dsim_device *dsim);

#endif /* _DSIM_H */
